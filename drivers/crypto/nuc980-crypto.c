/* Copyright (C) 2022, Nuvoton Technology Corporation
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <linux/scatterlist.h>
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/aead.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/skcipher.h>
#include <crypto/sha.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-crypto.h>
#include <mach/nuc980-crypto.h>


#define DMA_BUFSZ		(4096)

#define AES_FLAGS_BUSY		BIT(1)

#define HMAC_KEY_MAX_LEN	256

/* Static structures */
struct nu_aes_dev;
typedef int (*nu_aes_fn_t)(struct nu_aes_dev *, int);

struct nuc980_base_ctx {
	nu_aes_fn_t  start;
	int     channel;
	u32     mode;
	u32     keysize;
	u32     aes_key[8];
	int     sha_buffer_cnt;     /* byte count of data bufferred */
	int     hmac_key_len;
	u8      hmac_key_buff[HMAC_KEY_MAX_LEN];
};

struct nu_aes_ctx {
	struct nuc980_base_ctx  base;
};

struct nu_aes_dev {
	struct device *dev;
	u32 flags;
	struct crypto_async_request *areq;
	struct nuc980_base_ctx  *ctx;
	nu_aes_fn_t resume;
	spinlock_t lock;
	struct crypto_queue	queue;
	struct tasklet_struct done_task;
	struct tasklet_struct queue_task;

	/* for request handling */
	int req_len;
	int dma_len;
	struct scatterlist *in_sg;
	struct scatterlist *out_sg;
	int in_sg_off;
	int out_sg_off;
};

struct nuc980_crypto_dev nuc980_crdev;
static struct nu_aes_dev aes_dd;
static int nuc980_aes_dma_cascade(struct nu_aes_dev *dd, int err);

extern int nuc980_crypto_raw_probe(struct platform_device *pdev);
extern int nuc980_crypto_raw_remove(struct platform_device *pdev);


static int nuc980_aes_sg_to_buffer(struct nu_aes_dev *dd, u8 *bptr,
				    int max_cnt)
{
	int	in_cnt, copy_len;

	in_cnt = 0;
	while (dd->in_sg && (dd->req_len > 0) && (in_cnt < max_cnt)) {
		copy_len = min((int)dd->in_sg->length - dd->in_sg_off,
				dd->req_len);
		if (copy_len + in_cnt > max_cnt)
			copy_len = max_cnt - in_cnt;

		memcpy(bptr + in_cnt, (u8 *)sg_virt(dd->in_sg) +
			dd->in_sg_off, copy_len);

		in_cnt += copy_len;
		dd->req_len -= copy_len;
		dd->in_sg_off += copy_len;

		if (dd->in_sg_off >= dd->in_sg->length) {
			dd->in_sg = sg_next(dd->in_sg);
			dd->in_sg_off = 0;
		}
	}
	return in_cnt;
}

static int nuc980_aes_buffer_to_sg(struct nu_aes_dev *dd, u8 *bptr,
				    int max_cnt)
{
	int	copy_len, out_cnt = 0;

	while ((max_cnt > 0) && (dd->out_sg != NULL)) {
		copy_len = min((int)dd->out_sg->length - dd->out_sg_off,
			max_cnt);
		memcpy((u8 *)sg_virt(dd->out_sg) + dd->out_sg_off, bptr +
			out_cnt, copy_len);

		max_cnt -= copy_len;
		dd->out_sg_off += copy_len;
		out_cnt += copy_len;

		if (dd->out_sg_off >= dd->out_sg->length) {
			dd->out_sg = sg_next(dd->out_sg);
			dd->out_sg_off = 0;
		}
	}
	return out_cnt;
}

static int nuc980_aes_get_output(struct nu_aes_dev *dd)
{
	int     retval;

	retval = nuc980_aes_buffer_to_sg(dd, (u8 *)nuc980_crdev.aes_outbuf, dd->dma_len);
	dd->dma_len = 0;
	return retval;
}

static int nuc980_aes_complete(struct nu_aes_dev *dd, int err)
{
	struct skcipher_request *req = skcipher_request_cast(dd->areq);
	volatile struct nuc980_crypto_regs *cregs = nuc980_crdev.regs;
	u32	*ivec, reg32;
	int	i;

	nuc980_aes_get_output(dd);

	if ((req->iv) &&
	    ((dd->ctx->mode & AES_OPMODE_MASK) != AES_ECB_MODE)) {
		ivec = (u32 *)req->iv;
		for (i = 0; i < 4; i++) {
			 reg32 = cregs->CRPT_AES_FDBCK[i];
			 ivec[i] = ((reg32 >> 24) & 0xff) | ((reg32 >> 8) & 0xff00) |
				((reg32 << 8) & 0xff0000) | ((reg32 << 24) & 0xff000000);
		}
	}

	dd->flags &= ~AES_FLAGS_BUSY;
	dd->areq->complete(dd->areq, err);
	/* Handle new request */
	tasklet_schedule(&dd->queue_task);
	return err;
}

static int nuc980_aes_dma_run(struct nu_aes_dev *dd, u32 cascade)
{
	volatile struct nuc980_crypto_regs *cregs = nuc980_crdev.regs;
	u32 dma_ctl;

	dd->dma_len += nuc980_aes_sg_to_buffer(dd, (u8 *)nuc980_crdev.aes_inbuf + dd->dma_len,
			DMA_BUFSZ - dd->dma_len);

	if (!dd->in_sg || (dd->req_len == 0)) {
		dd->resume = nuc980_aes_complete;	/* no more data */
		dma_ctl = cascade | AES_DMALAST;
	} else {
		dd->resume = nuc980_aes_dma_cascade;
		dma_ctl = cascade;
	}

	/*
	 *  Execute AES encrypt/decrypt
	 */
	cregs->CRPT_AES0_CNT = dd->dma_len;
	cregs->CRPT_AES0_SADDR = nuc980_crdev.aes_inbuf_dma_addr;
	cregs->CRPT_AES0_DADDR = nuc980_crdev.aes_outbuf_dma_addr;

	// dump_AES_registers(dd);
	// printk("AES start 0x%x dma_len = %d\n", cregs->CRPT_AES_CTL | dma_ctl | AES_START, dd->dma_len);

	/* start AES */
	cregs->CRPT_AES_CTL = cregs->CRPT_AES_CTL | dma_ctl | AES_START;
	return -EINPROGRESS;
}

static int nuc980_aes_dma_cascade(struct nu_aes_dev *dd, int err)
{
	nuc980_aes_get_output(dd);

	/* cascade AES DMA to process remaining data */
	return nuc980_aes_dma_run(dd, AES_DMACSCAD);
}

static int nuc980_aes_dma_start(struct nu_aes_dev *dd, int err)
{
	struct skcipher_request *req = skcipher_request_cast(dd->areq);
	struct nuc980_base_ctx *ctx = dd->ctx;
	volatile struct nuc980_crypto_regs *cregs = nuc980_crdev.regs;
	u8	*iv = (u8 *)req->iv;
	int	i;

	if ((req->cryptlen == 0) || (req->src == NULL) ||	(req->dst == NULL))
		return nuc980_aes_complete(dd, 0);  /* no data */

	dd->req_len = req->cryptlen;
	dd->in_sg = req->src;
	dd->out_sg = req->dst;
	dd->in_sg_off = 0;
	dd->out_sg_off = 0;
	dd->dma_len = 0;

	/* program AES key */
	memcpy((void *)cregs->CRPT_AES0_KEY, (void *)ctx->aes_key, 32);

	/* program AES IV */
	if (iv) {
		for (i = 0; i < 4; i++)
			cregs->CRPT_AES0_IV[i] = (iv[i*4]<<24) | (iv[i*4+1]<<16) | (iv[i*4+2]<<8) | iv[i*4+3];
	} else {
		for (i = 0; i < 4; i++)
			cregs->CRPT_AES0_IV[i] = 0;
	}

	cregs->CRPT_INTEN |= (AESIEN | AESERRIEN);
	cregs->CRPT_AES_CTL = 0;
	cregs->CRPT_INTSTS = (AESIF | AESERRIF);

	cregs->CRPT_AES_CTL = ctx->keysize | ctx->mode | AES_INSWAP | AES_OUTSWAP |
							  AES_DMAEN | (ctx->channel << 24);

	pr_debug("[%s] - mode: 0x%08x, AES_CTL = 0x%x\n", __func__,
			ctx->mode, cregs->CRPT_AES_CTL);

	return nuc980_aes_dma_run(dd, 0);
}

static int nuc980_aes_handle_queue(struct crypto_async_request *new_areq)
{
	struct crypto_async_request *areq, *backlog;
	struct nuc980_base_ctx *ctx;
	struct nu_aes_dev *dd = &aes_dd;
	unsigned long  flags;
	int  ret = 0;

	spin_lock_irqsave(&dd->lock, flags);
	if (new_areq)
		ret = crypto_enqueue_request(&dd->queue, new_areq);
	if (dd->flags & AES_FLAGS_BUSY) {
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&dd->queue);
	areq = crypto_dequeue_request(&dd->queue);
	if (areq)
		dd->flags |= AES_FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!areq)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	ctx = crypto_tfm_ctx(areq->tfm);

	dd->areq = areq;
	dd->ctx = ctx;
	return ctx->start(dd, 0);
}

static int nuc980_aes_decrypt(struct skcipher_request *req, u32 mode)
{
	struct crypto_skcipher *skcipher = crypto_skcipher_reqtfm(req);
	struct nuc980_base_ctx *ctx = crypto_skcipher_ctx(skcipher);

	ctx->mode &= ~AES_ENCRYPT;
	ctx->mode = (ctx->mode & ~AES_OPMODE_MASK) | mode;
	return nuc980_aes_handle_queue(&req->base);
}

static int nuc980_aes_encrypt(struct skcipher_request *req, u32 mode)
{
	struct crypto_skcipher *skcipher = crypto_skcipher_reqtfm(req);
	struct nuc980_base_ctx *ctx = crypto_skcipher_ctx(skcipher);

	ctx->mode |= AES_ENCRYPT;
	ctx->mode = (ctx->mode & ~AES_OPMODE_MASK) | mode;
	return nuc980_aes_handle_queue(&req->base);
}

static int nuc980_aes_ecb_decrypt(struct skcipher_request *req)
{
	return nuc980_aes_decrypt(req, AES_ECB_MODE);
}

static int nuc980_aes_ecb_encrypt(struct skcipher_request *req)
{
	return nuc980_aes_encrypt(req, AES_ECB_MODE);
}

static int nuc980_aes_cbc_decrypt(struct skcipher_request *req)
{
	return nuc980_aes_decrypt(req, AES_CBC_MODE);
}

static int nuc980_aes_cbc_encrypt(struct skcipher_request *req)
{
	return nuc980_aes_encrypt(req, AES_CBC_MODE);
}

static int nuc980_aes_cfb_decrypt(struct skcipher_request *req)
{
	return nuc980_aes_decrypt(req, AES_CFB_MODE);
}

static int nuc980_aes_cfb_encrypt(struct skcipher_request *req)
{
	return nuc980_aes_encrypt(req, AES_CFB_MODE);
}

static int nuc980_aes_ofb_decrypt(struct skcipher_request *req)
{
	return nuc980_aes_decrypt(req, AES_OFB_MODE);
}

static int nuc980_aes_ofb_encrypt(struct skcipher_request *req)
{
	return nuc980_aes_encrypt(req, AES_OFB_MODE);
}

static int nuc980_aes_ctr_decrypt(struct skcipher_request *req)
{
	return nuc980_aes_decrypt(req, AES_CTR_MODE);
}

static int nuc980_aes_ctr_encrypt(struct skcipher_request *req)
{
	return nuc980_aes_encrypt(req, AES_CTR_MODE);
}

static int nuc980_aes_setkey(struct crypto_skcipher *tfm,
			     const u8 *key, unsigned int keylen)
{
	struct nuc980_base_ctx  *ctx = crypto_skcipher_ctx(tfm);
	int  i;

	switch (keylen) {
	case AES_KEYSIZE_128:
		ctx->keysize = AES_KEYSZ_128;
		break;

	case AES_KEYSIZE_192:
		ctx->keysize = AES_KEYSZ_192;
		break;

	case AES_KEYSIZE_256:
		ctx->keysize = AES_KEYSZ_256;
		break;

	default:
		pr_err("[%s]: Unsupported keylen %d!\n", __func__, keylen);
		return -EINVAL;
	}

	for (i = 0; i < keylen/4; i++)
		ctx->aes_key[i] = (key[i*4]<<24) | (key[i*4+1]<<16) | (key[i*4+2]<<8) | key[i*4+3];
	return 0;
}

static int nuc980_aes_init(struct crypto_skcipher *tfm)
{
	struct nu_aes_ctx  *ctx = crypto_skcipher_ctx(tfm);

	ctx->base.start = nuc980_aes_dma_start;
	ctx->base.channel = 0;    /*  NUC980 has only one channel, the channel 0.  */
	// printk("[%s], %s\n", __func__,  tfm->base.__crt_alg->cra_driver_name);
	return 0;
}

static void nuc980_aes_queue_task(unsigned long data)
{
	nuc980_aes_handle_queue(NULL);
}

static void nuc980_aes_done_task(unsigned long data)
{
	struct nu_aes_dev *dd = (struct nu_aes_dev *)data;
	//dma_sync_single_for_cpu(nuc980_crdev.dev, (dma_addr_t)nuc980_crdev.aes_inbuf_dma_addr, DMA_BUFSZ, DMA_TO_DEVICE);
	//dma_sync_single_for_cpu(nuc980_crdev.dev, (dma_addr_t)nuc980_crdev.aes_outbuf_dma_addr, DMA_BUFSZ, DMA_FROM_DEVICE);
	(void)dd->resume(dd, 0);
}

static struct skcipher_alg nuc980_aes_algs[] = {
{
	.base.cra_name		= "ecb(aes)",
	.base.cra_driver_name	= "ecb-aes-nuc980",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= AES_BLOCK_SIZE,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_module	= THIS_MODULE,
	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.setkey			= nuc980_aes_setkey,
	.encrypt		= nuc980_aes_ecb_encrypt,
	.decrypt		= nuc980_aes_ecb_decrypt,
	.ivsize			= AES_BLOCK_SIZE,
	.init			= nuc980_aes_init,
},
{
	.base.cra_name		= "cbc(aes)",
	.base.cra_driver_name	= "cbc-aes-nuc980",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= AES_BLOCK_SIZE,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_module	= THIS_MODULE,
	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.setkey			= nuc980_aes_setkey,
	.encrypt		= nuc980_aes_cbc_encrypt,
	.decrypt		= nuc980_aes_cbc_decrypt,
	.ivsize			= AES_BLOCK_SIZE,
	.init			= nuc980_aes_init,
},
{
	.base.cra_name		= "cfb(aes)",
	.base.cra_driver_name	= "cfb-aes-nuc980",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= AES_BLOCK_SIZE,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_module	= THIS_MODULE,
	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.setkey			= nuc980_aes_setkey,
	.encrypt		= nuc980_aes_cfb_encrypt,
	.decrypt		= nuc980_aes_cfb_decrypt,
	.ivsize			= AES_BLOCK_SIZE,
	.init			= nuc980_aes_init,
},
{
	.base.cra_name		= "ofb(aes)",
	.base.cra_driver_name	= "ofb-aes-nuc980",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= AES_BLOCK_SIZE,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_module	= THIS_MODULE,
	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.setkey			= nuc980_aes_setkey,
	.encrypt		= nuc980_aes_ofb_encrypt,
	.decrypt		= nuc980_aes_ofb_decrypt,
	.ivsize			= AES_BLOCK_SIZE,
	.init			= nuc980_aes_init,
},
{
	.base.cra_name		= "ctr(aes)",
	.base.cra_driver_name	= "ctr-aes-nuc980",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= AES_BLOCK_SIZE,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_module	= THIS_MODULE,
	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.setkey			= nuc980_aes_setkey,
	.encrypt		= nuc980_aes_ctr_encrypt,
	.decrypt		= nuc980_aes_ctr_decrypt,
	.ivsize			= AES_BLOCK_SIZE,
	.init			= nuc980_aes_init,
},
};

/*---------------------------------------------------------------------*/
/*                                                                     */
/*        NUC980 SHA/HMAC driver                                       */
/*                                                                     */
/*---------------------------------------------------------------------*/
extern void hexdump(unsigned char *buf, unsigned int len);

static int do_sha(struct ahash_request *req, int is_last)
{
	struct nuc980_base_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct scatterlist   *in_sg;
	volatile struct nuc980_crypto_regs  *cregs = nuc980_crdev.regs;
	int  req_len, sg_len;
	unsigned long   timeout;

	mutex_lock(&nuc980_crdev.sha_lock);
	in_sg = req->src;
	req_len = req->nbytes;

	// printk("do_sha - keylen = %d, req_len = %d, sha_buffer_cnt=%d\n", ctx->hmac_key_len, req_len, ctx->sha_buffer_cnt);

	if (is_last) {
		if (ctx->sha_buffer_cnt <= 0) {
			pr_err("Warning NUC980 SHA engine does not support zero length data!\n");
			mutex_unlock(&nuc980_crdev.sha_lock);
			return -EINVAL;
		}

		cregs->CRPT_HMAC_DMACNT = ctx->sha_buffer_cnt;
		cregs->CRPT_HMAC_SADDR = nuc980_crdev.hmac_inbuf_dma_addr;
		cregs->CRPT_HMAC_CTL |= HMAC_START | HMAC_DMAEN | HMAC_DMALAST;

		timeout = jiffies+200;
		while ((cregs->CRPT_HMAC_STS & HMAC_BUSY) && time_before(jiffies, timeout))
			;
		ctx->sha_buffer_cnt = 0;

		if (!time_before(jiffies, timeout)) {
			pr_err("keylen=%d, dma_len = %d, req_len = %d\n", ctx->hmac_key_len, ctx->sha_buffer_cnt, req_len);
			pr_err("Crypto SHA/HMAC engine failed!\n");
			mutex_unlock(&nuc980_crdev.sha_lock);
			return -EIO;
		}
		//printk("H/W output: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", cregs->CRPT_HMAC_DGST[0], cregs->CRPT_HMAC_DGST[1],
		//	cregs->CRPT_HMAC_DGST[2], cregs->CRPT_HMAC_DGST[3], cregs->CRPT_HMAC_DGST[4], cregs->CRPT_HMAC_DGST[5]);
		mutex_unlock(&nuc980_crdev.sha_lock);
		return 0;
	}

	while ((req_len > 0) && (in_sg != NULL)) {
		sg_len = in_sg->length;

		if (sg_len > 0) {
			if (sg_len > DMA_BUFSZ - ctx->sha_buffer_cnt) {
				pr_err("NUC980 SHA data overrun!\n");
				mutex_unlock(&nuc980_crdev.sha_lock);
				return -EINVAL;
			}
			memcpy(&nuc980_crdev.hmac_inbuf[ctx->sha_buffer_cnt], (u8 *)sg_virt(in_sg), sg_len);
			ctx->sha_buffer_cnt += sg_len;
			req_len -= sg_len;
		}
		in_sg = sg_next(in_sg);
	}
	mutex_unlock(&nuc980_crdev.sha_lock);
	return 0;
}


static int nuc980_sha_update(struct ahash_request *req)
{
	return do_sha(req, 0);
}

static int nuc980_sha_final(struct ahash_request *req)
{
	volatile struct nuc980_crypto_regs  *cregs = nuc980_crdev.regs;

	do_sha(req, 1);

	if ((cregs->CRPT_HMAC_CTL & HMAC_OPMODE_MASK) == HMAC_SHA1)
		memcpy(req->result, (u8 *)&(cregs->CRPT_HMAC_DGST[0]), SHA1_DIGEST_SIZE);
	else if ((cregs->CRPT_HMAC_CTL & HMAC_OPMODE_MASK) == HMAC_SHA224)
		memcpy(req->result, (u8 *)&(cregs->CRPT_HMAC_DGST[0]), SHA224_DIGEST_SIZE);
	else if ((cregs->CRPT_HMAC_CTL & HMAC_OPMODE_MASK) == HMAC_SHA256)
		memcpy(req->result, (u8 *)&(cregs->CRPT_HMAC_DGST[0]), SHA256_DIGEST_SIZE);
	else if ((cregs->CRPT_HMAC_CTL & HMAC_OPMODE_MASK) == HMAC_SHA384)
		memcpy(req->result, (u8 *)&(cregs->CRPT_HMAC_DGST[0]), SHA384_DIGEST_SIZE);
	else
		memcpy(req->result, (u8 *)&(cregs->CRPT_HMAC_DGST[0]), SHA512_DIGEST_SIZE);

	return 0;
}

static int nuc980_hmac_sha_init(struct ahash_request *req, int is_hmac)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	volatile struct nuc980_crypto_regs  *cregs = nuc980_crdev.regs;

	mutex_lock(&nuc980_crdev.sha_lock);
	cregs->CRPT_HMAC_CTL = HMAC_STOP;
	// printk("nuc980_sha_init: digest size: %d %s\n", crypto_ahash_digestsize(tfm), is_hmac ? "(HMAC)" : "");
	cregs->CRPT_HMAC_CTL = HMAC_INSWAP | HMAC_OUTSWAP;

	if (is_hmac)
		cregs->CRPT_HMAC_CTL |= HMAC_EN;
	else
		cregs->CRPT_HMAC_KEYCNT = 0;

	switch (crypto_ahash_digestsize(tfm)) {
	case SHA1_DIGEST_SIZE:
		cregs->CRPT_HMAC_CTL |= HMAC_SHA1;
		break;

	case SHA224_DIGEST_SIZE:
		cregs->CRPT_HMAC_CTL |= HMAC_SHA224;
		break;

	case SHA256_DIGEST_SIZE:
		cregs->CRPT_HMAC_CTL |= HMAC_SHA256;
		break;

	case SHA384_DIGEST_SIZE:
		cregs->CRPT_HMAC_CTL |= HMAC_SHA384;
		break;

	case SHA512_DIGEST_SIZE:
		cregs->CRPT_HMAC_CTL |= HMAC_SHA512;
		break;

	default:
		return -EINVAL;
	}
	mutex_unlock(&nuc980_crdev.sha_lock);
	return 0;
}


static int nuc980_sha_init(struct ahash_request *req)
{
	return nuc980_hmac_sha_init(req, 0);
}

static int nuc980_hmac_init(struct ahash_request *req)
{
	struct nuc980_base_ctx *ctx = crypto_tfm_ctx(req->base.tfm);

	memcpy((u8 *)nuc980_crdev.hmac_inbuf, ctx->hmac_key_buff, ctx->hmac_key_len);
	ctx->sha_buffer_cnt = ctx->hmac_key_len;
	return nuc980_hmac_sha_init(req, 1);
}

static int nuc980_hmac_setkey(struct crypto_ahash *tfm, const u8 *key, unsigned int keylen)
{
	struct nuc980_base_ctx *ctx = crypto_ahash_ctx(tfm);
	volatile struct nuc980_crypto_regs  *cregs = nuc980_crdev.regs;

	if (keylen > HMAC_KEY_MAX_LEN)
		return -EINVAL;

	ctx->hmac_key_len = keylen;
	memcpy(ctx->hmac_key_buff, key, keylen);
	cregs->CRPT_HMAC_KEYCNT = keylen;
	return 0;
}

static int nuc980_sha_finup(struct ahash_request *req)
{
	int err1, err2;

	err1 = nuc980_sha_update(req);
	if (err1 == -EINPROGRESS || err1 == -EBUSY)
		return err1;

	/*
	 * final() has to be always called to cleanup resources
	 * even if udpate() failed, except EINPROGRESS
	 */
	err2 = nuc980_sha_final(req);

	return err1 ?: err2;
}

static int nuc980_sha_export(struct ahash_request *req, void *out)
{
	struct nuc980_base_ctx *ctx = crypto_tfm_ctx(req->base.tfm);

	memcpy(out, ctx, sizeof(*ctx));
	return 0;
}

static int nuc980_sha_import(struct ahash_request *req, const void *in)
{
	struct nuc980_base_ctx *ctx = crypto_tfm_ctx(req->base.tfm);

	memcpy(ctx, in, sizeof(*ctx));
	return 0;
}

static int nuc980_sha_digest(struct ahash_request *req)
{
	return nuc980_hmac_sha_init(req, 0) ?: nuc980_sha_finup(req);
}

static int nuc980_hmac_digest(struct ahash_request *req)
{
	return nuc980_hmac_sha_init(req, 1) ?: nuc980_sha_finup(req);
}


static int nuc980_sha_cra_init(struct crypto_tfm *tfm)
{
	struct nuc980_base_ctx  *ctx = crypto_tfm_ctx(tfm);

	ctx->sha_buffer_cnt = 0;
	return 0;
}

static void nuc980_sha_cra_exit(struct crypto_tfm *tfm)
{
}

static struct ahash_alg nuc980_hash_algs[] = {
{
	.init       = nuc980_sha_init,
	.update     = nuc980_sha_update,
	.final      = nuc980_sha_final,
	.finup      = nuc980_sha_finup,
	.digest     = nuc980_sha_digest,
	.export     = nuc980_sha_export,
	.import     = nuc980_sha_import,
	.halg = {
		.digestsize = SHA1_DIGEST_SIZE,
		.statesize  = sizeof(struct nuc980_base_ctx),
		.base   = {
			.cra_name       = "sha1",
			.cra_driver_name    = "nuc980-sha1",
			.cra_priority   = 100,
			.cra_flags      = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize  = SHA1_BLOCK_SIZE,
			.cra_ctxsize    = sizeof(struct nuc980_base_ctx),
			.cra_alignmask  = 0,
			.cra_module     = THIS_MODULE,
			.cra_init       = nuc980_sha_cra_init,
			.cra_exit       = nuc980_sha_cra_exit,
		}
	}
},
{
	.init       = nuc980_sha_init,
	.update     = nuc980_sha_update,
	.final      = nuc980_sha_final,
	.finup      = nuc980_sha_finup,
	.digest     = nuc980_sha_digest,
	.export     = nuc980_sha_export,
	.import     = nuc980_sha_import,
	.halg = {
		.digestsize = SHA224_DIGEST_SIZE,
		.statesize  = sizeof(struct nuc980_base_ctx),
		.base   = {
			.cra_name       = "sha224",
			.cra_driver_name    = "nuc980-sha224",
			.cra_priority   = 100,
			.cra_flags      = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize  = SHA224_BLOCK_SIZE,
			.cra_ctxsize    = sizeof(struct nuc980_base_ctx),
			.cra_alignmask  = 0,
			.cra_module     = THIS_MODULE,
			.cra_init       = nuc980_sha_cra_init,
			.cra_exit       = nuc980_sha_cra_exit,
		}
	}
},
{
	.init       = nuc980_sha_init,
	.update     = nuc980_sha_update,
	.final      = nuc980_sha_final,
	.finup      = nuc980_sha_finup,
	.digest     = nuc980_sha_digest,
	.export     = nuc980_sha_export,
	.import     = nuc980_sha_import,
	.halg = {
		.digestsize = SHA256_DIGEST_SIZE,
		.statesize  = sizeof(struct nuc980_base_ctx),
		.base   = {
			.cra_name       = "sha256",
			.cra_driver_name    = "nuc980-sha256",
			.cra_priority   = 100,
			.cra_flags      = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize  = SHA256_BLOCK_SIZE,
			.cra_ctxsize    = sizeof(struct nuc980_base_ctx),
			.cra_alignmask  = 0,
			.cra_module     = THIS_MODULE,
			.cra_init       = nuc980_sha_cra_init,
			.cra_exit       = nuc980_sha_cra_exit,
		}
	}
},
{
	.init       = nuc980_sha_init,
	.update     = nuc980_sha_update,
	.final      = nuc980_sha_final,
	.finup      = nuc980_sha_finup,
	.digest     = nuc980_sha_digest,
	.export     = nuc980_sha_export,
	.import     = nuc980_sha_import,
	.halg = {
		.digestsize = SHA384_DIGEST_SIZE,
		.statesize  = sizeof(struct nuc980_base_ctx),
		.base   = {
			.cra_name       = "sha384",
			.cra_driver_name    = "nuc980-sha384",
			.cra_priority   = 100,
			.cra_flags      = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize  = SHA384_BLOCK_SIZE,
			.cra_ctxsize    = sizeof(struct nuc980_base_ctx),
			.cra_alignmask  = 0,
			.cra_module     = THIS_MODULE,
			.cra_init       = nuc980_sha_cra_init,
			.cra_exit       = nuc980_sha_cra_exit,
		}
	}
},
{
	.init       = nuc980_sha_init,
	.update     = nuc980_sha_update,
	.final      = nuc980_sha_final,
	.finup      = nuc980_sha_finup,
	.digest     = nuc980_sha_digest,
	.export     = nuc980_sha_export,
	.import     = nuc980_sha_import,
	.halg = {
		.digestsize = SHA512_DIGEST_SIZE,
		.statesize  = sizeof(struct nuc980_base_ctx),
		.base   = {
			.cra_name       = "sha512",
			.cra_driver_name    = "nuc980-sha512",
			.cra_priority   = 100,
			.cra_flags      = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize  = SHA512_BLOCK_SIZE,
			.cra_ctxsize    = sizeof(struct nuc980_base_ctx),
			.cra_alignmask  = 0,
			.cra_module     = THIS_MODULE,
			.cra_init       = nuc980_sha_cra_init,
			.cra_exit       = nuc980_sha_cra_exit,
		}
	}
},
{
	.init       = nuc980_hmac_init,
	.setkey     = nuc980_hmac_setkey,
	.update     = nuc980_sha_update,
	.final      = nuc980_sha_final,
	.finup      = nuc980_sha_finup,
	.digest     = nuc980_hmac_digest,
	.export     = nuc980_sha_export,
	.import     = nuc980_sha_import,
	.halg = {
		.digestsize = SHA1_DIGEST_SIZE,
		.statesize  = sizeof(struct nuc980_base_ctx),
		.base   = {
			.cra_name       = "hmac-sha1",
			.cra_driver_name    = "nuc980-hmac-sha1",
			.cra_priority   = 100,
			.cra_flags      = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize  = SHA1_BLOCK_SIZE,
			.cra_ctxsize    = sizeof(struct nuc980_base_ctx),
			.cra_alignmask  = 0,
			.cra_module     = THIS_MODULE,
			.cra_init       = nuc980_sha_cra_init,
			.cra_exit       = nuc980_sha_cra_exit,
		}
	}
},
{
	.init       = nuc980_hmac_init,
	.setkey     = nuc980_hmac_setkey,
	.update     = nuc980_sha_update,
	.final      = nuc980_sha_final,
	.finup      = nuc980_sha_finup,
	.digest     = nuc980_hmac_digest,
	.export     = nuc980_sha_export,
	.import     = nuc980_sha_import,
	.halg = {
		.digestsize = SHA224_DIGEST_SIZE,
		.statesize  = sizeof(struct nuc980_base_ctx),
		.base   = {
			.cra_name       = "hmac-sha224",
			.cra_driver_name    = "nuc980-hmac-sha224",
			.cra_priority   = 100,
			.cra_flags      = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize  = SHA224_BLOCK_SIZE,
			.cra_ctxsize    = sizeof(struct nuc980_base_ctx),
			.cra_alignmask  = 0,
			.cra_module     = THIS_MODULE,
			.cra_init       = nuc980_sha_cra_init,
			.cra_exit       = nuc980_sha_cra_exit,
		}
	}
},
{
	.init       = nuc980_hmac_init,
	.setkey     = nuc980_hmac_setkey,
	.update     = nuc980_sha_update,
	.final      = nuc980_sha_final,
	.finup      = nuc980_sha_finup,
	.digest     = nuc980_hmac_digest,
	.export     = nuc980_sha_export,
	.import     = nuc980_sha_import,
	.halg = {
		.digestsize = SHA256_DIGEST_SIZE,
		.statesize  = sizeof(struct nuc980_base_ctx),
		.base   = {
			.cra_name       = "hmac-sha256",
			.cra_driver_name    = "nuc980-hmac-sha256",
			.cra_priority   = 100,
			.cra_flags      = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize  = SHA256_BLOCK_SIZE,
			.cra_ctxsize    = sizeof(struct nuc980_base_ctx),
			.cra_alignmask  = 0,
			.cra_module     = THIS_MODULE,
			.cra_init       = nuc980_sha_cra_init,
			.cra_exit       = nuc980_sha_cra_exit,
		}
	}
},
{
	.init       = nuc980_hmac_init,
	.setkey     = nuc980_hmac_setkey,
	.update     = nuc980_sha_update,
	.final      = nuc980_sha_final,
	.finup      = nuc980_sha_finup,
	.digest     = nuc980_hmac_digest,
	.export     = nuc980_sha_export,
	.import     = nuc980_sha_import,
	.halg = {
		.digestsize = SHA384_DIGEST_SIZE,
		.statesize  = sizeof(struct nuc980_base_ctx),
		.base   = {
			.cra_name       = "hmac-sha384",
			.cra_driver_name    = "nuc980-hmac-sha384",
			.cra_priority   = 100,
			.cra_flags      = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize  = SHA384_BLOCK_SIZE,
			.cra_ctxsize    = sizeof(struct nuc980_base_ctx),
			.cra_alignmask  = 0,
			.cra_module     = THIS_MODULE,
			.cra_init       = nuc980_sha_cra_init,
			.cra_exit       = nuc980_sha_cra_exit,
		}
	}
},
{
	.init       = nuc980_hmac_init,
	.setkey     = nuc980_hmac_setkey,
	.update     = nuc980_sha_update,
	.final      = nuc980_sha_final,
	.finup      = nuc980_sha_finup,
	.digest     = nuc980_hmac_digest,
	.export     = nuc980_sha_export,
	.import     = nuc980_sha_import,
	.halg = {
		.digestsize = SHA512_DIGEST_SIZE,
		.statesize  = sizeof(struct nuc980_base_ctx),
		.base   = {
			.cra_name       = "hmac-sha512",
			.cra_driver_name    = "nuc980-hmac-sha512",
			.cra_priority   = 100,
			.cra_flags      = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize  = SHA512_BLOCK_SIZE,
			.cra_ctxsize    = sizeof(struct nuc980_base_ctx),
			.cra_alignmask  = 0,
			.cra_module     = THIS_MODULE,
			.cra_init       = nuc980_sha_cra_init,
			.cra_exit       = nuc980_sha_cra_exit,
		}
	}
},
};

static irqreturn_t nuc980_crypto_irq(int irq, void *data)
{
	volatile struct nuc980_crypto_regs  *cregs = nuc980_crdev.regs;
	u32  status, ret = IRQ_NONE;

	status = cregs->CRPT_INTSTS;
	if (status & (AESIF | AESERRIF)) {
		if (cregs->CRPT_AES_STS & AESERRIF)
			pr_err("AESERRIF set!\n");
		cregs->CRPT_INTSTS = AESIF | AESERRIF;
		tasklet_schedule(&aes_dd.done_task);
		ret = IRQ_HANDLED;
	}
	return ret;
}

static int nuc980_crypto_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	int irq;
	int i, err, ret;

	if (IS_ERR(clk_get(NULL, "crypto_hclk"))) {
		pr_err("%s clk_get error!!\n", __func__);
		return -ENODEV;
	}

	/* Enable Cryptographic Accerlator clock */
	clk_prepare(clk_get(NULL, "crypto_hclk"));
	clk_enable(clk_get(NULL, "crypto_hclk"));

	memset((u8 *)&nuc980_crdev, 0, sizeof(nuc980_crdev));

	nuc980_crdev.dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(dev, "Failed to get Crypto irq!\n");
		return -ENODEV;
	}

	err = devm_request_irq(dev, irq, nuc980_crypto_irq,
			       IRQF_SHARED, "nuc980-crypto", &nuc980_crdev);
	if (err) {
		dev_err(dev, "Failed to request IRQ%d: err: %d.\n", irq, err);
		return err;
	}

	if (!request_mem_region(res->start, resource_size(res), pdev->name))
		return -EBUSY;

	nuc980_crdev.regs = ioremap(res->start, resource_size(res));
	if (!nuc980_crdev.regs)
		return -ENOMEM;

	nuc980_crdev.aes_inbuf = dma_alloc_coherent(dev, DMA_BUFSZ, &nuc980_crdev.aes_inbuf_dma_addr, GFP_KERNEL);
	nuc980_crdev.aes_outbuf = dma_alloc_coherent(dev, DMA_BUFSZ, &nuc980_crdev.aes_outbuf_dma_addr, GFP_KERNEL);
	nuc980_crdev.hmac_inbuf = dma_alloc_coherent(dev, DMA_BUFSZ, &nuc980_crdev.hmac_inbuf_dma_addr, GFP_KERNEL);

	if (!nuc980_crdev.aes_inbuf || !nuc980_crdev.aes_outbuf || !nuc980_crdev.hmac_inbuf) {
		ret = -ENOMEM;
		goto failed_dmabuff;
	}

	nuc980_crdev.aes_inbuf_size  = DMA_BUFSZ;
	nuc980_crdev.aes_outbuf_size = DMA_BUFSZ;
	nuc980_crdev.hmac_inbuf_size = DMA_BUFSZ;

	memset(&aes_dd, 0, sizeof(aes_dd));
	aes_dd.dev = dev;

	spin_lock_init(&aes_dd.lock);

	tasklet_init(&aes_dd.done_task, nuc980_aes_done_task,
					(unsigned long)&aes_dd);
	tasklet_init(&aes_dd.queue_task, nuc980_aes_queue_task,
					(unsigned long)&aes_dd);
	crypto_init_queue(&aes_dd.queue, 32);

	for (i = 0; i < ARRAY_SIZE(nuc980_aes_algs); i++) {
		err = crypto_register_skcipher(&nuc980_aes_algs[i]);
		if (err)
			goto failed;
	}

	for (i = 0; i < ARRAY_SIZE(nuc980_hash_algs); i++) {
		err = crypto_register_ahash(&nuc980_hash_algs[i]);
		if (err)
			goto failed;
	}

	mutex_init(&(nuc980_crdev.aes_lock));
	mutex_init(&(nuc980_crdev.sha_lock));
	nuc980_crypto_raw_probe(pdev);

	printk(KERN_NOTICE "NUC980 Crypto engine enabled.\n");
	return 0;

failed:
	spin_unlock(&aes_dd.lock);
	tasklet_kill(&aes_dd.done_task);
	tasklet_kill(&aes_dd.queue_task);

failed_dmabuff:
	if (nuc980_crdev.aes_inbuf)
		dma_free_coherent(dev, DMA_BUFSZ, nuc980_crdev.aes_inbuf, nuc980_crdev.aes_inbuf_dma_addr);
	if (nuc980_crdev.aes_outbuf)
		dma_free_coherent(dev, DMA_BUFSZ, nuc980_crdev.aes_outbuf, nuc980_crdev.aes_outbuf_dma_addr);
	if (nuc980_crdev.hmac_inbuf)
		dma_free_coherent(dev, DMA_BUFSZ, nuc980_crdev.hmac_inbuf, nuc980_crdev.hmac_inbuf_dma_addr);

	iounmap(nuc980_crdev.regs);
	release_mem_region(res->start, resource_size(res));
	pr_err("NUC980 Crypto initialization failed.\n");
	return ret;
}

static int nuc980_crypto_remove(struct platform_device *pdev)
{
	int  i;
	struct device *dev = &pdev->dev;
	struct resource *res;

	nuc980_crypto_raw_remove(pdev);

	spin_unlock(&aes_dd.lock);
	tasklet_kill(&aes_dd.done_task);
	tasklet_kill(&aes_dd.queue_task);

	for (i = 0; i < ARRAY_SIZE(nuc980_aes_algs); i++)
		crypto_unregister_skcipher(&nuc980_aes_algs[i]);

	for (i = 0; i < ARRAY_SIZE(nuc980_hash_algs); i++)
		crypto_unregister_ahash(&nuc980_hash_algs[i]);

	dma_free_coherent(dev, DMA_BUFSZ, nuc980_crdev.aes_inbuf, nuc980_crdev.aes_inbuf_dma_addr);
	dma_free_coherent(dev, DMA_BUFSZ, nuc980_crdev.aes_outbuf, nuc980_crdev.aes_outbuf_dma_addr);
	dma_free_coherent(dev, DMA_BUFSZ, nuc980_crdev.hmac_inbuf, nuc980_crdev.hmac_inbuf_dma_addr);

	iounmap(nuc980_crdev.regs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	clk_disable(clk_get(NULL, "crypto_hclk"));

	return 0;
}

static int nuc980_crypto_suspend(struct platform_device *pdev, pm_message_t state)
{
	volatile struct nuc980_crypto_regs  *cregs = nuc980_crdev.regs;
	unsigned long  timeout;

	timeout = jiffies+200;   // 2 seconds time out

	while (time_before(jiffies, timeout)) {
		if (cregs->CRPT_AES_STS & AES_BUSY)
			continue;

		if (cregs->CRPT_HMAC_STS & HMAC_BUSY)
			continue;

		break;
	}

	clk_disable(clk_get(NULL, "crypto_hclk"));

	return 0;
}

static int nuc980_crypto_resume(struct platform_device *pdev)
{
	clk_enable(clk_get(NULL, "crypto_hclk"));
	return 0;
}


static const struct of_device_id nuc980_crypto_of_match[] = {
	{ .compatible = "nuvoton,nuc980-crypto" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_crypto_of_match);


static struct platform_driver nuc980_crypto_driver = {
	.probe      = nuc980_crypto_probe,
	.remove     = nuc980_crypto_remove,
	.resume     = nuc980_crypto_resume,
	.suspend    = nuc980_crypto_suspend,
	.driver     = {
		.name   = "nuc980-crypto",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuc980_crypto_of_match),
	},
};

module_platform_driver(nuc980_crypto_driver);

MODULE_AUTHOR("Nuvoton Technology Corporation");
MODULE_DESCRIPTION("NUC980 Cryptographic Accerlerator");
MODULE_LICENSE("GPL");
