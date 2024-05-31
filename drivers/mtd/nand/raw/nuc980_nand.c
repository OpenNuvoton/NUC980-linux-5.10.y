/*
 * Copyright © 2018 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>

#include <linux/pinctrl/consumer.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/rawnand.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>

#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gcr.h>
#include <mach/regs-timer.h>
#include <mach/regs-fmi.h>


#define RESET_FMI   0x01
#define NAND_EN     0x08
#define READYBUSY   0x400

#define SWRST       0x01
#define PSIZE       (0x01 << 3)
#define DMARWEN     (0x03 << 1)
#define BUSWID      (0x01 << 4)
#define ECC4EN      (0x01 << 5)
#define WP          (0x01 << 24)
#define NANDCS      (0x01 << 25)
#define ENDADDR     (0x01 << 31)

#define BCH_T15     0x00400000
#define BCH_T12     0x00200000
#define BCH_T8      0x00100000
#define BCH_T4      0x00080000
#define BCH_T24     0x00040000

#define NUC980_DRV_VERSION "20220317"
#define DEF_RESERVER_OOB_SIZE_FOR_MARKER 4

//#define NUC980_NAND_DEBUG
#ifndef NUC980_NAND_DEBUG
#define DBG(fmt, arg...)
#define ENTER()
#define LEAVE()
#else
#define DBG(fmt, arg...)    printk(fmt, ##arg)
#define ENTER()
#define LEAVE()
#endif


struct nuc980_nand_info {
	struct nand_controller	controller;
	struct device		*dev;
	struct mtd_info	mtd;
	struct nand_chip	chip;
	struct mtd_partition	*parts;     // mtd partition
	int			nr_parts;   // mtd partition number
	struct platform_device	*pdev;
	struct clk		*clk;
	struct clk		*fmi_clk;

//	void __iomem            *reg;
	int			eBCHAlgo;
	int			m_i32SMRASize;

	int			irq;
	struct completion	complete;
	unsigned char		*dma_buf;
};

static int nuc980_ooblayout_ecc(struct mtd_info *mtd, int section,
				 struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nand_ecc_ctrl *ecc = &chip->ecc;

	if (section || !ecc->total)
		return -ERANGE;

	oobregion->length = ecc->total;
	oobregion->offset = mtd->oobsize - oobregion->length;

	return 0;
}

static int nuc980_ooblayout_free(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nand_ecc_ctrl *ecc = &chip->ecc;

	if (section)
		return -ERANGE;

	oobregion->length = mtd->oobsize - ecc->total - 2;
	oobregion->offset = 2;

	return 0;
}

static const struct mtd_ooblayout_ops nuc980_ooblayout_ops = {
	.ecc = nuc980_ooblayout_ecc,
	.free = nuc980_ooblayout_free,
};

typedef enum  {
	eBCH_NONE=0,
	eBCH_T8,
	eBCH_T12,
	eBCH_T24,
	eBCH_CNT
} E_BCHALGORITHM;


static const int g_i32BCHAlgoIdx[eBCH_CNT] = { BCH_T8, BCH_T8, BCH_T12, BCH_T24 };
static struct nand_ecclayout_user nuc980_nand_oob;
static const int g_i32ParityNum[3][eBCH_CNT] = {
	{ 0,  60,  92,  90 },  // for 2K
	{ 0, 120, 184, 180 },  // for 4K
	{ 0, 240, 368, 360 },  // for 8K
};

#if 0
#ifndef CONFIG_MTD_CMDLINE_PARTS
#ifndef CONFIG_OF
static struct mtd_partition partitions[] = {
	{
		.name = "u-boot",
		.offset = 0,
		.size = 2 * 1024 * 1024,
		.ecclayout = (struct nand_ecclayout*)&nuc980_nand_oob
	},
	{
		.name = "Kernel",
		.size = 20 * 1024 * 1024,
		.offset = MTDPART_OFS_APPEND,
		.ecclayout = (struct nand_ecclayout*)&nuc980_nand_oob
	},
	{
		.name = "user",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL
	}
};
#endif
#endif
#endif	// if 0

/*
 * nuc980_nand_hwecc_init - Initialize hardware ECC IP
 */
static void nuc980_nand_hwecc_init (struct nuc980_nand_info *nand)
{
	writel ( readl(REG_SMCSR)|0x1, REG_SMCSR);    // reset SM controller

	// Redundant area size
	writel( nand->m_i32SMRASize , REG_SMREACTL );

	// Protect redundant 3 bytes
	// because we need to implement write_oob function to partial data to oob available area.
	// Please note we skip 4 bytes
	writel( readl(REG_SMCSR) | 0x100, REG_SMCSR);

	// To read/write the ECC parity codes automatically from/to NAND Flash after data area field written.
	writel( readl(REG_SMCSR) | 0x10, REG_SMCSR);

	if ( nand->eBCHAlgo == eBCH_NONE ) {
		// Disable H/W ECC / ECC parity check enable bit during read page
		writel( readl(REG_SMCSR) & (~0x00800000), REG_SMCSR);
	} else  {
		// Set BCH algorithm
		writel( (readl(REG_SMCSR) & (~0x007C0000)) | g_i32BCHAlgoIdx[nand->eBCHAlgo], REG_SMCSR);

		// Enable H/W ECC, ECC parity check enable bit during read page
		writel( readl(REG_SMCSR) | 0x00800000, REG_SMCSR);
	}
}


static void nuc980_nand_initialize (void)
{
	// Enable SM_EN
	writel( NAND_EN, REG_NAND_FMICSR );

	// Enable SM_CS0
	writel((readl(REG_SMCSR)&(~0x06000000))|0x04000000, REG_SMCSR);
	writel(0x1, REG_NFECR); /* un-lock write protect */

	// NAND Reset
	writel(readl(REG_SMCSR) | 0x1, REG_SMCSR);    // software reset
}

/*-----------------------------------------------------------------------------
 * Define some constants for BCH
 *---------------------------------------------------------------------------*/
// define the total padding bytes for 512/1024 data segment
#define BCH_PADDING_LEN_512     32
#define BCH_PADDING_LEN_1024    64
// define the BCH parity code lenght for 512 bytes data pattern
#define BCH_PARITY_LEN_T8  15
#define BCH_PARITY_LEN_T12 23
// define the BCH parity code lenght for 1024 bytes data pattern
#define BCH_PARITY_LEN_T24 45


/*-----------------------------------------------------------------------------
 * Correct data by BCH alrogithm.
 *      Support 8K page size NAND and BCH T4/8/12/15/24.
 *---------------------------------------------------------------------------*/
void fmiSM_CorrectData_BCH(u8 ucFieidIndex, u8 ucErrorCnt, u8* pDAddr)
{
	u32 uaData[24], uaAddr[24];
	u32 uaErrorData[6];
	u8  ii, jj;
	u32 uPageSize;
	u32 field_len, padding_len, parity_len;
	u32 total_field_num;
	u8  *smra_index;

	ENTER();

	//--- assign some parameters for different BCH and page size
	switch (readl(REG_SMCSR) & 0x007C0000)
	{
		case BCH_T24:
			field_len   = 1024;
			padding_len = BCH_PADDING_LEN_1024;
			parity_len  = BCH_PARITY_LEN_T24;
			break;

		case BCH_T12:
			field_len   = 512;
			padding_len = BCH_PADDING_LEN_512;
			parity_len  = BCH_PARITY_LEN_T12;
			break;

		case BCH_T8:
			field_len   = 512;
			padding_len = BCH_PADDING_LEN_512;
			parity_len  = BCH_PARITY_LEN_T8;
			break;

		default:
			printk("NAND ERROR: %s(): invalid SMCR_BCH_TSEL = 0x%08X\n", __FUNCTION__, (u32)(readl(REG_SMCSR) & 0x7C0000));
			LEAVE();
			return;
	}

	uPageSize = readl(REG_SMCSR) & 0x00030000;
	switch (uPageSize)
	{
		case 0x30000:  total_field_num = 8192 / field_len; break;
		case 0x20000:  total_field_num = 4096 / field_len; break;
		case 0x10000:  total_field_num = 2048 / field_len; break;
		default:
			printk("NAND ERROR: %s(): invalid SMCR_PSIZE = 0x%08X\n", __FUNCTION__, uPageSize);
			LEAVE();
			return;
	}

	//--- got valid BCH_ECC_DATAx and parse them to uaData[]
	// got the valid register number of BCH_ECC_DATAx since one register include 4 error bytes
	jj = ucErrorCnt/4;
	jj ++;
	if (jj > 6)
		jj = 6;     // there are 6 BCH_ECC_DATAx registers to support BCH T24

	for(ii=0; ii<jj; ii++)
	{
		uaErrorData[ii] = readl(REG_BCH_ECC_DATA0 + ii*4);
	}

	for(ii=0; ii<jj; ii++)
	{
		uaData[ii*4+0] = uaErrorData[ii] & 0xff;
		uaData[ii*4+1] = (uaErrorData[ii]>>8) & 0xff;
		uaData[ii*4+2] = (uaErrorData[ii]>>16) & 0xff;
		uaData[ii*4+3] = (uaErrorData[ii]>>24) & 0xff;
	}

	//--- got valid REG_BCH_ECC_ADDRx and parse them to uaAddr[]
	// got the valid register number of REG_BCH_ECC_ADDRx since one register include 2 error addresses
	jj = ucErrorCnt/2;
	jj ++;
	if (jj > 12)
		jj = 12;    // there are 12 REG_BCH_ECC_ADDRx registers to support BCH T24

	for(ii=0; ii<jj; ii++)
	{
		uaAddr[ii*2+0] = readl(REG_BCH_ECC_ADDR0 + ii*4) & 0x07ff;   // 11 bits for error address
		uaAddr[ii*2+1] = (readl(REG_BCH_ECC_ADDR0 + ii*4)>>16) & 0x07ff;
	}

	//--- pointer to begin address of field that with data error
	pDAddr += (ucFieidIndex-1) * field_len;

	//--- correct each error bytes
	for(ii=0; ii<ucErrorCnt; ii++)
	{
		// for wrong data in field
		if (uaAddr[ii] < field_len)
		{
#ifdef NUC980_NAND_DEBUG
			printk("BCH error corrected for data: address 0x%08X, data [0x%02X] --> ",
				(unsigned int)(pDAddr+uaAddr[ii]), (unsigned int)(*(pDAddr+uaAddr[ii])));
#endif
			*(pDAddr+uaAddr[ii]) ^= uaData[ii];

#ifdef NUC980_NAND_DEBUG
			printk("[0x%02X]\n", *(pDAddr+uaAddr[ii]));
#endif
		}
		// for wrong first-3-bytes in redundancy area
		else if (uaAddr[ii] < (field_len+3))
		{
			uaAddr[ii] -= field_len;
			uaAddr[ii] += (parity_len*(ucFieidIndex-1));    // field offset

#ifdef NUC980_NAND_DEBUG
			printk("BCH error corrected for 3 bytes: address 0x%08X, data [0x%02X] --> ",
				(unsigned int)((u8 *)REG_SMRA0 + uaAddr[ii]), (unsigned int)(*((u8 *)REG_SMRA0 + uaAddr[ii])));
#endif
			*((u8 *)REG_SMRA0 + uaAddr[ii]) ^= uaData[ii];

#ifdef NUC980_NAND_DEBUG
			printk("[0x%02X]\n", *((u8 *)REG_SMRA0+uaAddr[ii]));
#endif
		}
		// for wrong parity code in redundancy area
		else
		{
			// BCH_ERR_ADDRx = [data in field] + [3 bytes] + [xx] + [parity code]
			//                                   |<--     padding bytes      -->|
			// The BCH_ERR_ADDRx for last parity code always = field size + padding size.
			// So, the first parity code = field size + padding size - parity code length.
			// For example, for BCH T12, the first parity code = 512 + 32 - 23 = 521.
			// That is, error byte address offset within field is
			uaAddr[ii] = uaAddr[ii] - (field_len + padding_len - parity_len);

			// smra_index point to the first parity code of first field in register SMRA0~n
			smra_index = (u8 *)
						 (REG_SMRA0 + (readl(REG_SMREACTL) & 0x1ff) - // bottom of all parity code -
						  (parity_len * total_field_num)                             // byte count of all parity code
						 );

			// final address = first parity code of first field +
			//                 offset of fields +
			//                 offset within field

#ifdef NUC980_NAND_DEBUG
			printk("BCH error corrected for parity: address 0x%08X, data [0x%02X] --> ",
				(unsigned int)(smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]),
				(unsigned int)(*(smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii])));
#endif
			*((u8 *)smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]) ^= uaData[ii];

#ifdef NUC980_NAND_DEBUG
			printk("[0x%02X]\n",
				*((u8 *)smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]));
#endif
		}
	}   // end of for (ii<ucErrorCnt)
	LEAVE();
}

int fmiSMCorrectData (struct nand_chip *chip, unsigned long uDAddr )
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	int uStatus, ii, jj, i32FieldNum=0;
	volatile int uErrorCnt = 0;
	volatile int uReportErrCnt = 0;

	if ( readl ( REG_SMISR ) & 0x4 )
	{
		if ( ( readl(REG_SMCSR) & 0x7C0000) == BCH_T24 )
			i32FieldNum = mtd->writesize / 1024;    // Block=1024 for BCH
		else
			i32FieldNum = mtd->writesize / 512;

		if ( i32FieldNum < 4 )
			i32FieldNum  = 1;
		else
			i32FieldNum /= 4;

		for ( jj=0; jj<i32FieldNum; jj++ )
		{
			uStatus = readl ( REG_SMECC_ST0+jj*4 );
			if ( !uStatus )
				continue;

			for ( ii=1; ii<5; ii++ )
			{
				if ( !(uStatus & 0x03) ) { // No error
					uStatus >>= 8;
					continue;
				} else if ( (uStatus & 0x03)==0x01 ) { // Correctable error
					uErrorCnt = (uStatus >> 2) & 0x1F;
#ifdef NUC980_NAND_DEBUG
					printk("Field (%d, %d) have %d error!!\n", jj, ii, uErrorCnt);
#endif
					fmiSM_CorrectData_BCH(jj*4+ii, uErrorCnt, (char*)uDAddr);
					uReportErrCnt += uErrorCnt;
					uStatus >>= 8;
					continue;
				} else // uncorrectable error or ECC error
				{
#ifdef NUC980_NAND_DEBUG
					printk("SM uncorrectable error is encountered, 0x%4x !!\n", uStatus);
#endif
					LEAVE();
					return -1;
				}
				uStatus >>= 8;
			}
		} //jj
	}
	return uReportErrCnt;
}

/*
 * HW ECC Correction
 * function called after a read
 */
static int nuc980_nand_correct_data(struct nand_chip *chip, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	return 0;
}


/*
 * Enable HW ECC : unused on most chips
 */
void nuc980_nand_enable_hwecc(struct nand_chip *chip, int mode)
{

}

/*
 * nuc980_nand_dmac_init - Initialize dma controller
 */
static void nuc980_nand_dmac_init( struct nuc980_nand_info *nand )
{
	// DMAC enable
	writel( readl(REG_NAND_DMACCSR) | 0x3, REG_NAND_DMACCSR);
	writel( readl(REG_NAND_DMACCSR) & (~0x2), REG_NAND_DMACCSR);

	// Clear DMA finished flag
	writel( readl(REG_SMISR) | 0x5, REG_SMISR);

	init_completion(&nand->complete);
	/* enable ecc and dma */
	writel(0x5, REG_SMIER);
}


/*
 * nuc980_nand_read_byte - read a byte from NAND controller into buffer
 */
static unsigned char nuc980_nand_read_byte(struct nand_chip *chip)
{
	unsigned char ret;

	writel(readl(REG_SMCSR) & (~0x02000000), REG_SMCSR);
	ret = (unsigned char)readl(REG_SMDATA);
	writel(readl(REG_SMCSR)|0x02000000, REG_SMCSR);

	return ret;
}


static void nuc980_nand_read_buf(struct nand_chip *chip, unsigned char *buf, int len)
{
	int i;

	writel(readl(REG_SMCSR) & (~0x02000000), REG_SMCSR);
	for (i = 0; i < len; i++)
		buf[i] = (unsigned char)readl(REG_SMDATA);
	writel(readl(REG_SMCSR)|0x02000000, REG_SMCSR);
}

static void nuc980_nand_write_buf(struct nand_chip *chip, const unsigned char *buf, int len)
{
	int i;

	writel(readl(REG_SMCSR) & (~0x02000000), REG_SMCSR);
	for (i = 0; i < len; i++)
		writel(buf[i], REG_SMDATA);
	writel(readl(REG_SMCSR)|0x02000000, REG_SMCSR);
}

static inline int nuc980_nand_dma_transfer(struct nand_chip *chip, const u_char *addr, unsigned int len, int is_write)
{
	struct nuc980_nand_info *nand = nand_get_controller_data(chip);
	dma_addr_t dma_addr;
	int ret;
	unsigned long timeo = jiffies + HZ/2;

	writel(readl(REG_SMCSR) & (~0x02000000), REG_SMCSR);
	// For save, wait DMAC to ready
	while (1) {
		if ((readl(REG_NAND_DMACCSR) & 0x200) == 0)
			break;
		if (timeo < jiffies)
			return -ETIMEDOUT;
	}

	// Reinitial dmac
	nuc980_nand_dmac_init(nand);

	writel( nand->m_i32SMRASize , REG_SMREACTL );

	/* setup and start DMA using dma_addr */
	if ( is_write ) {
		register char * ptr=REG_SMRA0;
		// To mark this page as dirty.
		if ( ptr[3] == 0xFF )
			ptr[3] = 0;
		if ( ptr[2] == 0xFF )
			ptr[2] = 0;

		// Fill dma_addr
		dma_addr = dma_map_single(nand->dev, (void *)addr, len, DMA_TO_DEVICE);
		ret = dma_mapping_error(nand->dev, dma_addr);
		if (ret) {
			dev_err(nand->dev, "dma mapping error\n");
			return -EINVAL;
		}

		writel((unsigned long)dma_addr, REG_NAND_DMACSAR);
		writel ( readl(REG_SMCSR) | 0x4, REG_SMCSR );
		wait_for_completion_timeout(&nand->complete, msecs_to_jiffies(1000));

		dma_unmap_single(nand->dev, dma_addr, len, DMA_TO_DEVICE);
	} else {
		// Fill dma_addr
		dma_addr = dma_map_single(nand->dev, (void *)addr, len, DMA_FROM_DEVICE);
		ret = dma_mapping_error(nand->dev, dma_addr);
		if (ret) {
			dev_err(nand->dev, "dma mapping error\n");
			return -EINVAL;
		}
		nand->dma_buf = (unsigned char *) addr;

		writel((unsigned long)dma_addr, REG_NAND_DMACSAR);
		writel ( readl(REG_SMCSR) | 0x2, REG_SMCSR);
		wait_for_completion_timeout(&nand->complete, msecs_to_jiffies(1000));

		dma_unmap_single(nand->dev, dma_addr, len, DMA_FROM_DEVICE);
	}

	writel(readl(REG_SMCSR)|0x02000000, REG_SMCSR);

	return 0;
}

static void nuc980_read_buf_dma(struct nand_chip *chip, u_char *buf, int len)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	if ( len == mtd->writesize ) /* start transfer in DMA mode */
		nuc980_nand_dma_transfer (chip, buf, len, 0x0);
	else {
		nuc980_nand_read_buf(chip, buf, len);

#ifdef NUC980_NAND_DEBUG
		{
		int i;
		printk("R OOB %d\n", len );
		for ( i=0; i<len; i++ )
		{
			printk("%02X ", buf[i] );
			if ( i%32 == 31 )   printk("\n");
		}
		printk("\n");
		}
#endif
	}
}

static void nuc980_write_buf_dma(struct nand_chip *chip, const u_char *buf, int len)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	if ( len == mtd->writesize ) /* start transfer in DMA mode */
		nuc980_nand_dma_transfer(chip, (u_char *)buf, len, 0x1);
	else
	{
#ifdef NUC980_NAND_DEBUG
		int i;
		printk("W OOB %d\n", len);
		for ( i=0; i<len; i++ )
		{
			printk("%02X ", buf[i] );
			if ( i%32 == 31 )   printk("\n");
		}
#endif
		nuc980_nand_write_buf(chip, buf, len);
	}
}


static int nuc980_nand_devready(struct nand_chip *chip)
{
	unsigned int volatile val;

	writel(readl(REG_SMCSR) & (~0x02000000), REG_SMCSR);
	val = (readl(REG_SMISR) & 0x40000) ? 1 : 0;
	writel(readl(REG_SMCSR)|0x02000000, REG_SMCSR);

	return val;
}


static int nuc980_waitfunc(struct nand_chip *chip)
{
	unsigned long timeo = jiffies;
	int volatile status = -1;

	timeo += msecs_to_jiffies(400);

	writel(readl(REG_SMCSR) & (~0x02000000), REG_SMCSR);
	while (time_before(jiffies, timeo)) {
		status = readl(REG_SMISR);
		if (status & 0x400)	/* check r/b# flag */
		{
			writel(0x400, REG_SMISR);
			status = 0;
			break;
		}
		cond_resched();
	}
	writel(readl(REG_SMCSR)|0x02000000, REG_SMCSR);
	return status;
}

static void nuc980_nand_command(struct nand_chip *chip, unsigned int command, int column, int page_addr)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	writel(readl(REG_SMCSR) & (~0x02000000), REG_SMCSR);
	writel(0x400, REG_SMISR);

	if (command == NAND_CMD_READOOB) {
		command = NAND_CMD_READ0;
		column += mtd->writesize;
	}

	switch (command) {

	case NAND_CMD_RESET:
		writel(command, REG_SMCMD);
		break;

	case NAND_CMD_READID:
		writel(command, REG_SMCMD);
		writel(ENDADDR|column, REG_SMADDR);
		break;

	case NAND_CMD_PARAM:
		writel(command, REG_SMCMD);
		writel(ENDADDR|column, REG_SMADDR);
		nuc980_waitfunc(chip);
		break;

	case NAND_CMD_READ0:
		writel(0x0, REG_NFECR); /* lock write protect */
		writel(command, REG_SMCMD);
		if (column != -1) {
			writel(column & 0xff, REG_SMADDR);
			writel((column >> 8) & 0xff, REG_SMADDR);
		}
		if (page_addr != -1) {
			writel(page_addr & 0xff, REG_SMADDR);
			if ( chip->options & NAND_ROW_ADDR_3) {
				writel((page_addr >> 8) & 0xff, REG_SMADDR);
				writel(((page_addr >> 16) & 0xff)|ENDADDR, REG_SMADDR);
			} else {
				writel(((page_addr >> 8) & 0xff)|ENDADDR, REG_SMADDR);
			}
		}
		writel(NAND_CMD_READSTART, REG_SMCMD);
		nuc980_waitfunc(chip);
		break;


	case NAND_CMD_ERASE1:
		writel(0x1, REG_NFECR); /* un-lock write protect */
		writel(command, REG_SMCMD);
		writel(page_addr & 0xff, REG_SMADDR);
		if ( chip->options & NAND_ROW_ADDR_3) {
			writel((page_addr >> 8) & 0xff, REG_SMADDR);
			writel(((page_addr >> 16) & 0xff)|ENDADDR, REG_SMADDR);
		} else {
			writel(((page_addr >> 8) & 0xff)|ENDADDR, REG_SMADDR);
		}
		break;


	case NAND_CMD_SEQIN:
		writel(0x1, REG_NFECR); /* un-lock write protect */
		writel(command, REG_SMCMD);
		writel(column & 0xff, REG_SMADDR);
		writel(column >> 8, REG_SMADDR);
		writel(page_addr & 0xff, REG_SMADDR);
		if ( chip->options & NAND_ROW_ADDR_3) {
			writel((page_addr >> 8) & 0xff, REG_SMADDR);
			writel(((page_addr >> 16) & 0xff)|ENDADDR, REG_SMADDR);
		} else {
			writel(((page_addr >> 8) & 0xff)|ENDADDR, REG_SMADDR);
		}
		break;

	case NAND_CMD_STATUS:
		writel(0x1, REG_NFECR); /* un-lock write protect */
		writel(command, REG_SMCMD);
		break;

	default:
		writel(command, REG_SMCMD);
	}
	writel(readl(REG_SMCSR)|0x02000000, REG_SMCSR);
}

/* select chip */
static void nuc980_nand_select_chip(struct nand_chip *chip, int cs)
{

	if (cs == 0)
		writel(readl(REG_SMCSR) & (~0x02000000), REG_SMCSR);
	else
		writel(readl(REG_SMCSR) | 0x02000000, REG_SMCSR);
}

static int nuc980_nand_calculate_ecc(struct nand_chip *chip, const u_char *dat, u_char *ecc_code)
{
	return 0;
}

static int nuc980_nand_write_page_hwecc(struct nand_chip *chip, const uint8_t *buf, int oob_required, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	uint8_t *ecc_calc = chip->ecc.calc_buf;
	register char * ptr=REG_SMRA0;

	memset ( (void*)ptr, 0xFF, mtd->oobsize );
	memcpy ( (void*)ptr, (void*)chip->oob_poi,  mtd->oobsize - chip->ecc.total );

	nuc980_nand_command(chip, NAND_CMD_SEQIN, 0, page);
	nuc980_nand_dma_transfer(chip, buf, mtd->writesize , 0x1);
	nuc980_nand_command(chip, NAND_CMD_PAGEPROG, -1, -1);
	nuc980_waitfunc(chip);

	// Copy parity code in SMRA to calc
	memcpy ( (void*)ecc_calc,  (void*)( REG_SMRA0 + ( mtd->oobsize - chip->ecc.total ) ), chip->ecc.total );

	// Copy parity code in calc to oob_poi
	memcpy ( (void*)(chip->oob_poi+(mtd->oobsize-chip->ecc.total)), (void*)ecc_calc, chip->ecc.total);

	return 0;
}

static int nuc980_nand_read_page_hwecc_oob_first(struct nand_chip *chip, uint8_t *buf, int oob_required, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	uint8_t *p = buf;
	char * ptr=REG_SMRA0;

	/* At first, read the OOB area  */
	nuc980_nand_command(chip, NAND_CMD_READOOB, 0, page);
	nuc980_nand_read_buf(chip, chip->oob_poi, mtd->oobsize);

	// Second, copy OOB data to SMRA for page read
	memcpy ( (void*)ptr, (void*)chip->oob_poi, mtd->oobsize );

	if ((*(ptr+2) != 0) && (*(ptr+3) != 0))
	{
		memset((void*)p, 0xff, mtd->writesize);
	}
	else
	{
		// Third, read data from nand
		nuc980_nand_command(chip, NAND_CMD_READ0, 0, page);
		nuc980_nand_dma_transfer(chip, p, mtd->writesize, 0x0);

		// Fouth, restore OOB data from SMRA
		memcpy ( (void*)chip->oob_poi, (void*)ptr, mtd->oobsize );
	}

	return 0;
}

static void nuc980_layout_oob_table ( struct nand_ecclayout_user* pNandOOBTbl, int oobsize , int eccbytes )
{
	pNandOOBTbl->eccbytes = eccbytes;
	pNandOOBTbl->oobavail = oobsize - DEF_RESERVER_OOB_SIZE_FOR_MARKER - eccbytes ;
	pNandOOBTbl->oobfree[0].offset = DEF_RESERVER_OOB_SIZE_FOR_MARKER;  // Bad block marker size
	pNandOOBTbl->oobfree[0].length = oobsize - eccbytes - pNandOOBTbl->oobfree[0].offset ;

	pNandOOBTbl->oobfree[1].offset = 0;
	pNandOOBTbl->oobfree[1].length = 0;
}

static int nuc980_nand_read_oob_hwecc(struct nand_chip *chip, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	char * ptr=REG_SMRA0;

	nuc980_nand_command(chip, NAND_CMD_READOOB, 0, page);

	nuc980_nand_read_buf(chip, chip->oob_poi, mtd->oobsize);

	// Second, copy OOB data to SMRA for page read
	memcpy ( (void*)ptr, (void*)chip->oob_poi, mtd->oobsize );

	if ((*(ptr+2) != 0) && (*(ptr+3) != 0))
	{
		memset((void*)chip->oob_poi, 0xff, mtd->oobsize);
	}

	return 0;
}

static irqreturn_t nuc980_nand_irq(int irq, struct nuc980_nand_info *nand)
{
	struct mtd_info *mtd = nand_to_mtd(&nand->chip);
	unsigned int isr;

	/* Clear interrupt flag */
	// SM interrupt status
	isr = readl(REG_SMISR);
	if (isr & 0x01)
	{
		writel(0x1, REG_SMISR);
		complete(&nand->complete);
	}
	if (isr & 0x04)
	{
		int stat=0;
		if ( (stat=fmiSMCorrectData(&nand->chip,  (unsigned long)nand->dma_buf)) < 0 )
		{
			mtd->ecc_stats.failed++;
			writel ( 0x3, REG_NAND_DMACCSR);          // reset DMAC
			writel ( readl(REG_SMCSR)|0x1, REG_SMCSR);    // reset SM controller
		}
		else if ( stat > 0 ) {
			mtd->ecc_stats.corrected += stat;   // Add corrected bit count
		}
		writel(0x4, REG_SMISR);
	}

	return IRQ_HANDLED;
}


static int nuc980_nand_attach_chip(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct nuc980_nand_info *host = nand_get_controller_data(chip);
	unsigned int reg;

	//Set PSize bits of SMCSR register to select NAND card page size
	reg = readl(REG_SMCSR) & (~0x30000);
	if (mtd->writesize == 2048)
		writel(reg | 0x10000, REG_SMCSR);
	else if (mtd->writesize == 4096)
		writel(reg | 0x20000, REG_SMCSR);
	else if (mtd->writesize == 8192)
		writel(reg | 0x30000, REG_SMCSR);

	if (chip->ecc.strength == 0) {
		host->eBCHAlgo = eBCH_NONE; /* No ECC */
		nuc980_layout_oob_table(&nuc980_nand_oob, mtd->oobsize, g_i32ParityNum[mtd->writesize>>12][host->eBCHAlgo]);

	} else if (chip->ecc.strength <= 8) {
		host->eBCHAlgo = eBCH_T8; /* T8 */
		nuc980_layout_oob_table(&nuc980_nand_oob, mtd->oobsize, g_i32ParityNum[mtd->writesize>>12][host->eBCHAlgo]);

	} else if (chip->ecc.strength <= 12) {
		host->eBCHAlgo = eBCH_T12; /* T12 */
		nuc980_layout_oob_table(&nuc980_nand_oob, mtd->oobsize, g_i32ParityNum[mtd->writesize>>12][host->eBCHAlgo]);

	} else if (chip->ecc.strength <= 24) {
		host->eBCHAlgo = eBCH_T24; /* T24 */
		nuc980_layout_oob_table(&nuc980_nand_oob, mtd->oobsize, g_i32ParityNum[mtd->writesize>>12][host->eBCHAlgo]);

	} else {
		pr_warn("NAND Controller is not support this flash. (%d, %d)\n", mtd->writesize, mtd->oobsize);
	}

	host->m_i32SMRASize = mtd->oobsize;
	chip->ecc.steps = mtd->writesize / chip->ecc.size;
	chip->ecc.bytes = nuc980_nand_oob.eccbytes / chip->ecc.steps;
	chip->ecc.total = nuc980_nand_oob.eccbytes;
	mtd_set_ooblayout(mtd, &nuc980_ooblayout_ops);

	pr_info("attach: page %d, SMRA size %d, %d\n", mtd->writesize, mtd->oobsize, nuc980_nand_oob.eccbytes);

	/* add mtd-id. The string should same as uboot definition */
	mtd->name = "nand0";

	nuc980_nand_hwecc_init(host);

	writel(0x0, REG_NFECR); /* lock write protect */

	return 0;
}


static const struct nand_controller_ops nuc980_nand_controller_ops = {
	.attach_chip = nuc980_nand_attach_chip,
};

static int nuc980_nand_probe(struct platform_device *pdev)
{
	struct nand_chip *chip;
	struct nuc980_nand_info *nuc980_nand;
	struct mtd_info *mtd;
	struct pinctrl *p;

	int retval=0;

	ENTER() ;

	nuc980_nand = devm_kzalloc(&pdev->dev, sizeof(struct nuc980_nand_info), GFP_KERNEL);
	if (!nuc980_nand)
		return -ENOMEM;

	nand_controller_init(&nuc980_nand->controller);

	nuc980_nand->pdev = pdev;
	nuc980_nand->dev = &pdev->dev;
	chip = &nuc980_nand->chip;
	mtd = nand_to_mtd(chip);
	nand_set_controller_data(chip, nuc980_nand);
	nand_set_flash_node(chip, pdev->dev.of_node);

	mtd->priv   = chip;
	mtd->owner  = THIS_MODULE;
	mtd->dev.parent = &pdev->dev;

	/*
	 * Get Clock
	 */
	nuc980_nand->fmi_clk = clk_get(NULL, "fmi_hclk");
	if (IS_ERR(nuc980_nand->fmi_clk)) {
		printk("no fmi_clk?\n");
		retval = -ENXIO;
		goto fail;
	}

	nuc980_nand->clk = clk_get(NULL, "nand_hclk");
	if (IS_ERR(nuc980_nand->clk)) {
		printk("no nand_clk?\n");
		goto fail;
	}

	clk_prepare(nuc980_nand->fmi_clk);
	clk_enable(nuc980_nand->fmi_clk);
	clk_prepare(nuc980_nand->clk);
	clk_enable(nuc980_nand->clk);

	nuc980_nand->chip.controller = &nuc980_nand->controller;

	chip->legacy.cmdfunc     = nuc980_nand_command;
	chip->legacy.waitfunc    = nuc980_waitfunc;
	chip->legacy.read_byte   = nuc980_nand_read_byte;
	chip->legacy.select_chip = nuc980_nand_select_chip;
	chip->legacy.read_buf    = nuc980_read_buf_dma;
	chip->legacy.write_buf   = nuc980_write_buf_dma;
	chip->legacy.chip_delay  = 25; /* us */

	// Check NAND device NBUSY0 pin
	chip->legacy.dev_ready   = nuc980_nand_devready;
	/* set up nand options */
	chip->bbt_options = NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB;

	// Read OOB data first, then HW read page
	chip->ecc.hwctl      = nuc980_nand_enable_hwecc;
	chip->ecc.calculate  = nuc980_nand_calculate_ecc;
	chip->ecc.correct    = nuc980_nand_correct_data;
	chip->ecc.write_page = nuc980_nand_write_page_hwecc;
	chip->ecc.read_page  = nuc980_nand_read_page_hwecc_oob_first;
	chip->ecc.read_oob   = nuc980_nand_read_oob_hwecc;
	chip->options |= (NAND_NO_SUBPAGE_WRITE | NAND_USES_DMA);

#ifdef CONFIG_OF

	p = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(p)) {
		return PTR_ERR(p);
	}

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

#else

	p = devm_pinctrl_get_select(&pdev->dev, "nand");
	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
#endif

	nuc980_nand_initialize();
	platform_set_drvdata(pdev, nuc980_nand);

	nuc980_nand->controller.ops = &nuc980_nand_controller_ops;

	nuc980_nand->irq = platform_get_irq(pdev, 0);
	if (nuc980_nand->irq < 0) {
		dev_err(&pdev->dev, "failed to get platform irq\n");
		retval = -EINVAL;
		goto fail;
	}

	if (request_irq(nuc980_nand->irq, (irq_handler_t)&nuc980_nand_irq,
			IRQF_TRIGGER_HIGH, "nuc980-nand", nuc980_nand)) {
		dev_err(&pdev->dev, "Error requesting NAND IRQ\n");
		retval = -ENXIO;
		goto fail;
	}

	if (nand_scan(chip, 1))
		goto fail;

	if (mtd_device_register(mtd, nuc980_nand->parts, nuc980_nand->nr_parts))
		goto fail;

	pr_info("fmi-sm: registered successfully! mtdid=%s\n", mtd->name);
	return retval;

fail:
	mtd_device_unregister(mtd);
	nand_cleanup(chip);
	devm_kfree(&pdev->dev, nuc980_nand);
	return retval;
}

static int nuc980_nand_remove(struct platform_device *pdev)
{
	struct nuc980_nand_info *nuc980_nand = platform_get_drvdata(pdev);

	clk_disable(nuc980_nand->clk);
	clk_put(nuc980_nand->clk);

	kfree(nuc980_nand);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

/* PM Support */
#ifdef CONFIG_PM
static int nuc980_nand_suspend(struct platform_device *pdev, pm_message_t pm)
{
	struct nuc980_nand_info *nuc980_nand = platform_get_drvdata(pdev);
	unsigned long timeo = jiffies + HZ/2;

	// For save, wait DMAC to ready
	while (1) {
		if ((readl(REG_NAND_DMACCSR) & 0x200) == 0)
			break;
		if (timeo < jiffies)
			return -ETIMEDOUT;
	}
	writel(0x0, REG_NFECR); /* write protect */
	clk_disable(nuc980_nand->clk);

	return 0;
}

static int nuc980_nand_resume(struct platform_device *pdev)
{
	struct nuc980_nand_info *nuc980_nand = platform_get_drvdata(pdev);

	clk_enable(nuc980_nand->clk);
	nuc980_nand_hwecc_init(nuc980_nand);
	writel(0x1, REG_NFECR); /* un-lock write protect */
	nuc980_nand_dmac_init(nuc980_nand);


	return 0;
}

#else
#define nuc980_nand_suspend NULL
#define nuc980_nand_resume NULL
#endif

static const struct of_device_id nuc980_fmi_of_match[] = {
	{ .compatible = "nuvoton,nuc980-fmi" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_fmi_of_match);

static struct platform_driver nuc980_nand_driver = {
		.driver = {
		.name   = "nuc980-fmi",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuc980_fmi_of_match),
		},
		.probe      = nuc980_nand_probe,
		.remove     = nuc980_nand_remove,
		.suspend    = nuc980_nand_suspend,
		.resume     = nuc980_nand_resume,
};

static int __init nuc980_nand_init(void)
{
	int ret;
	printk("nuc980 mtd nand driver version: %s\n", NUC980_DRV_VERSION );

	ret = platform_driver_register(&nuc980_nand_driver);
	if (ret) {
		printk("nand: failed to add device driver %s \n", nuc980_nand_driver.driver.name);
		return ret;
	}

	return ret;
}

static void __exit nuc980_nand_exit(void)
{
	platform_driver_unregister(&nuc980_nand_driver);
	printk("nand: unregistered successfully! \n");
}

module_init(nuc980_nand_init);
module_exit(nuc980_nand_exit);

MODULE_AUTHOR("nuvoton");
MODULE_DESCRIPTION("nuc980 nand driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc980-fmi");
