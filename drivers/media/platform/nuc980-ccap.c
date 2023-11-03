// SPDX-License-Identifier: GPL-2.0
/*
 * MA35D1 Camera Capture Interface Controller (CCAP) driver
 *
 * Copyright (C) 2020 Nuvoton Technology Corp.
 */

#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_graph.h>

#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mediabus.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-image-sizes.h>

#include <media/v4l2-async.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-rect.h>
#include <mach/regs-clock.h>

#include <mach/map.h>

#define DRV_NAME "nvt-ccap"

//#define CCAP_DEBUG_ENABLE_ENTER_LEAVE
#ifdef CCAP_DEBUG_ENABLE_ENTER_LEAVE
#define VDEBUG(fmt, arg...)	pr_info(fmt, ##arg)
#define ENTRY()	pr_info("Enter...%s()\n", __func__)
#define LEAVE()	pr_info("Leave...%s()\n", __func__)
#else
#define VDEBUG(fmt, arg...)
#define ENTRY()
#define LEAVE()
#endif

#define VSP_LO      0x000	/* 0 : VS pin output polarity is active low */
#define VSP_HI      0x400	/* 1 : VS pin output polarity is active high. */
#define HSP_LO      0x000	/* 0 : HS pin output polarity is active low */
#define HSP_HI      0x200	/* 1 : HS pin output polarity is active high. */
/*
 * 0 : Input video data and signals are latched
 *	by falling edge of Pixel Clock.
 */
#define PCLKP_LO    0x000
/*
 * 1 : Input video data and signals are latched
 *	by rising edge of Pixel Clock.
 */
#define PCLKP_HI    0x100

#define INFMT_YCbCr  0x0	/*  Sensor Input Data Format YCbCr422 */
#define INFMT_RGB565 0x1	/*  Sensor Input Data Format RGB565 */
#define INTYPE_CCIR601 (0x0<<1)	/*  Sensor Input Type CCIR601 */
#define INTYPE_CCIR656 (0x1<<1)	/*  Sensor Input Type CCIR656 */
#define INORD_YUYV  (0x0<<2)	/*  Sensor Input Data Order YUYV */
#define INORD_YVYU  (0x1<<2)	/*  Sensor Input Data Order YVYU */
#define INORD_UYVY  (0x2<<2)	/*  Sensor Input Data Order UYVY */
#define INORD_VYUY  (0x3<<2)	/*  Sensor Input Data Order VYUY */
#define INMASK 0xF		/*  Sensor Input Mask */

/* Mirror addresses are not available for all registers */
#define CCAP_CTL 0x00
#define CTL_UPDATE (1<<20)
#define CTL_BAYER_10 (1<<21)
#define CCAP_PAR 0x04
#define PAR_INFMT (1<<0)
#define PAR_SENTYPE (1<<1)
#define CCAP_INT 0x08
#define VIEN (1<<16)
#define VINTF (1<<0)
#define CCAP_POSTERIZE 0x0c
#define CCAP_CWSP 0x20
#define CWSP_CWSADDRV (0x3F<<16)
#define CWSP_CWSADDRH (0x3F<<0)
#define CCAP_CWS 0x24
#define CCAP_PKTSL 0x28
#define PKTSL_PKTSVNL (0xFF << 24)
#define PKTSL_PKTSVML (0xFF << 16)
#define PKTSL_PKTSHNL (0xFF << 8)
#define PKTSL_PKTSHML (0xFF << 0)
#define CCAP_PLNSL 0x2C
#define PLNSL_PLNSVNL (0xFF << 24)
#define PLNSL_PLNSVML (0xFF << 16)
#define PLNSL_PLNSHNL (0xFF << 8)
#define PLNSL_PLNSHML (0xFF << 0)
#define CCAP_STRIDE 0x34
#define STRIDE_PLNSTRIDE (0x3FFF << 16)
#define STRIDE_PKTSTRIDE (0x3FFF << 0)
#define CCAP_PKTSM 0x48
#define PKTSM_PKTSVNH (0xFF << 24)
#define PKTSM_PKTSVMH (0xFF << 16)
#define PKTSM_PKTSHNH (0xFF << 8)
#define PKTSM_PKTSHMH (0xFF << 0)
#define CCAP_PLNSM 0x4C
#define PLNSM_PLNSVNH (0xFF << 24)
#define PLNSM_PLNSVMH (0xFF << 16)
#define PLNSM_PLNSHNH (0xFF << 8)
#define PLNSM_PLNSHMH (0xFF << 0)
#define CCAP_PKTBA0 0x60
#define CCAP_YBA 0x80
#define CCAP_UBA 0x84
#define CCAP_VBA 0x88

//u32 video_freq = 24000000;

enum ccap_status {
	CCAP_IDLE,
	CCAP_INITIALISING,
	CCAP_RUNNING,
	CCAP_OVERLAY,
};

#define CCAP_MIN_IMAGE_WIDTH	16U
#define CCAP_MAX_IMAGE_WIDTH	2048U
#define CCAP_MIN_IMAGE_HEIGHT	16U
#define CCAP_MAX_IMAGE_HEIGHT	4096U

#define sensor_call(cam, o, f, args...) \
	v4l2_subdev_call(cam->sensor, o, f, ##args)

struct ccap_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

static inline struct
ccap_buffer *to_ccap_buffer(struct vb2_v4l2_buffer
			    *vb2)
{
	return container_of(vb2, struct ccap_buffer, vb);
}

struct ccap_data {
	u32 polarity;
	u32 infmtord;
	u32 cropstart;
	/* bothenable(packet and planar are enabled ) is enabled,
	 * planarfmt is effective
	 */
	u32 bothenable;
	/* Planar YUV422: V4L2_PIX_FMT_YUV422P or Planar
	 *        YUV420: V4L2_PIX_FMT_YUV411P
	 */
	u32 planarfmt;

	u32 packet;
	u32 planar;
};

static struct ccap_data ccap_data = {
	.infmtord = (INORD_YUYV | INFMT_YCbCr | INTYPE_CCIR601),
	.polarity = (VSP_LO | HSP_LO | PCLKP_HI),
	.cropstart = (0 | 0 << 16),	/*( Vertical | Horizontal<<16 ) */
	.bothenable = 0,
	.packet = 0,
	.planar = 0,
};

struct ccap_format {
	u32 fourcc;
	u32 mbus_code;
	u8 bpp;
};

struct ccap_framesize {
	u32 width;
	u32 height;
};

struct ccap_fmt {
	u32 pfmt;
	unsigned char bpp;
	unsigned char bpl;
};

/* Further pixel formats can be added */
static struct ccap_fmt ccap_fmt[] = {
	{
	 .pfmt = V4L2_PIX_FMT_RGB565,
	 .bpp = 16,
	 .bpl = 2,
	  },
	{
	 .pfmt = V4L2_PIX_FMT_YUYV,
	 .bpp = 16,
	 .bpl = 2,
	  },
	{
	 .pfmt = V4L2_PIX_FMT_RGB555,
	 .bpp = 16,
	 .bpl = 2,
	  },
	{
	 .pfmt = V4L2_PIX_FMT_GREY,
	 .bpp = 8,
	 .bpl = 1,
	  },
};

struct ccap_graph_entity {
	struct v4l2_async_subdev asd;
	struct device_node *remote_node;
	struct v4l2_subdev *source;
};

struct ccap_device {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct video_device *vdev;
	spinlock_t lock;
	void __iomem *base;
	/* State information */
	struct v4l2_pix_format pix;

	struct list_head buf_list;
	int pix_idx;
	struct vb2_queue queue;
	struct ccap_buffer *active;
	enum ccap_status status;
	volatile unsigned int sequence;
	struct mutex fop_lock;
	struct ccap_data *pdata;

	struct v4l2_format fmt;
	struct v4l2_subdev *sensor;
	struct v4l2_async_notifier notifier;
	struct v4l2_async_subdev asd;
	struct ccap_graph_entity entity;
	struct media_pad vid_cap_pad;
	struct media_device mdev;
	const struct ccap_format **sd_formats;
	unsigned int num_of_sd_formats;
	const struct ccap_format *sd_format;
	struct ccap_framesize *sd_framesizes;
	unsigned int num_of_sd_framesizes;
	struct ccap_framesize sd_framesize;
	bool do_crop;
	struct v4l2_rect crop;
	struct v4l2_rect sd_bounds;

#ifdef EN_OVERLAY
	void __iomem *display_base;
#endif

	unsigned int dcultra_framebase;
	unsigned int dcultra_framesize;

};

static void ccap_reg_write(struct ccap_device
			   *ccap_dev, unsigned int reg, u32 value)
{
	__raw_writel(value, ccap_dev->base + reg);
}

static u32 ccap_reg_read(struct ccap_device *ccap_dev, unsigned int reg)
{
	return __raw_readl(ccap_dev->base + reg);
}

static const struct ccap_format *find_format_by_fourcc(struct ccap_device *ccap,
						       unsigned int fourcc)
{
	unsigned int num_formats = ccap->num_of_sd_formats;
	const struct ccap_format *fmt;
	unsigned int i;

	for (i = 0; i < num_formats; i++) {
		fmt = ccap->sd_formats[i];
		if (fmt->fourcc == fourcc)
			return fmt;
	}

	return NULL;
}

static void __find_outer_frame_size(struct ccap_device *ccap,
				    struct v4l2_pix_format *pix,
				    struct ccap_framesize *framesize)
{
	struct ccap_framesize *match = NULL;
	unsigned int i;
	unsigned int min_err = UINT_MAX;

	for (i = 0; i < ccap->num_of_sd_framesizes; i++) {
		struct ccap_framesize *fsize = &ccap->sd_framesizes[i];
		int w_err = (fsize->width - pix->width);
		int h_err = (fsize->height - pix->height);
		int err = w_err + h_err;

		if (w_err >= 0 && h_err >= 0 && err < min_err) {
			min_err = err;
			match = fsize;
		}
	}
	if (!match)
		match = &ccap->sd_framesizes[0];

	*framesize = *match;
}

static int ccap_try_fmt(struct ccap_device *ccap, struct v4l2_format *f,
			const struct ccap_format **sd_format,
			struct ccap_framesize *sd_framesize)
{
	const struct ccap_format *sd_fmt;
	struct ccap_framesize sd_fsize;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_pad_config pad_cfg;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};
	bool do_crop;
	int ret;

	ENTRY();
	sd_fmt = find_format_by_fourcc(ccap, pix->pixelformat);
	if (!sd_fmt) {
		if (!ccap->num_of_sd_formats)
			return -ENODATA;

		sd_fmt = ccap->sd_formats[ccap->num_of_sd_formats - 1];
		pix->pixelformat = sd_fmt->fourcc;
	}

	/* Limit to hardware capabilities */
	pix->width =
	    clamp(pix->width, CCAP_MIN_IMAGE_WIDTH, CCAP_MAX_IMAGE_WIDTH);
	pix->height =
	    clamp(pix->height, CCAP_MIN_IMAGE_HEIGHT, CCAP_MAX_IMAGE_HEIGHT);

	dev_dbg(ccap->dev, "pix->width %d, pix->height %d\n",
		pix->width, pix->height);
	/* No crop if JPEG is requested */
	do_crop = ccap->do_crop && (pix->pixelformat != V4L2_PIX_FMT_JPEG);

	if (do_crop && ccap->num_of_sd_framesizes) {
		struct ccap_framesize outer_sd_fsize;
		/*
		 * If crop is requested and sensor have discrete frame sizes,
		 * select the frame size that is just larger than request
		 */
		__find_outer_frame_size(ccap, pix, &outer_sd_fsize);
		pix->width = outer_sd_fsize.width;
		pix->height = outer_sd_fsize.height;
	}

	v4l2_fill_mbus_format(&format.format, pix, sd_fmt->mbus_code);
	ret = v4l2_subdev_call(ccap->entity.source, pad, set_fmt,
			       &pad_cfg, &format);
	if (ret < 0)
		return ret;

	/* Update pix regarding to what sensor can do */
	v4l2_fill_pix_format(pix, &format.format);

	/* Save resolution that sensor can actually do */
	sd_fsize.width = pix->width;
	sd_fsize.height = pix->height;

	if (do_crop) {
		struct v4l2_rect c = ccap->crop;
		struct v4l2_rect max_rect;

		/*
		 * Adjust crop by making the intersection between
		 * format resolution request and crop request
		 */
		max_rect.top = 0;
		max_rect.left = 0;
		max_rect.width = pix->width;
		max_rect.height = pix->height;
		v4l2_rect_map_inside(&c, &max_rect);
		c.top = clamp_t(s32, c.top, 0, pix->height - c.height);
		c.left = clamp_t(s32, c.left, 0, pix->width - c.width);
		ccap->crop = c;

		/* Adjust format resolution request to crop */
		pix->width = ccap->crop.width;
		pix->height = ccap->crop.height;
	}

	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * sd_fmt->bpp;
	pix->sizeimage = pix->bytesperline * pix->height;

	if (sd_format)
		*sd_format = sd_fmt;
	if (sd_framesize)
		*sd_framesize = sd_fsize;

	LEAVE();
	return 0;
}

static int ccap_set_default_fmt(struct ccap_device *ccap)
{
	struct v4l2_format f = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.fmt.pix = {
			    .width = VGA_WIDTH,
			    .height = VGA_HEIGHT,
			    .field = V4L2_FIELD_NONE,
			    .pixelformat = ccap->sd_formats[0]->fourcc,
			     },
	};
	int ret;

	ENTRY();
	ret = ccap_try_fmt(ccap, &f, NULL, NULL);
	if (ret)
		return ret;
	ccap->sd_format = ccap->sd_formats[0];
	ccap->fmt = f;

	LEAVE();
	return 0;
}

/*
 * FIXME: For the time being we only support subdevices
 * which expose RGB & YUV "parallel form" mbus code (_2X8).
 * Nevertheless, this allows to support serial source subdevices
 * and serial to parallel bridges which conform to this.
 */
static const struct ccap_format ccap_formats[] = {
	{
	 .fourcc = V4L2_PIX_FMT_YUYV,
	 .mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
	 .bpp = 2,
	  },
	{
	 .fourcc = V4L2_PIX_FMT_RGB565,
	 .mbus_code = MEDIA_BUS_FMT_RGB565_2X8_LE,
	 .bpp = 2,
	  },
};

static void ccap_schedule_next(struct ccap_device
			       *ccap_dev, struct vb2_v4l2_buffer *vbuf)
{
	dma_addr_t addr1;

	ENTRY();
	addr1 = vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 0);

	VDEBUG("packet=%d,planar=%d\n", ccap_dev->pdata->packet,
	       ccap_dev->pdata->planar);

	if (ccap_dev->pdata->packet)
		ccap_reg_write(ccap_dev, CCAP_PKTBA0, addr1);

	if (ccap_dev->pdata->planar) {
		u32 tmp;

		/* Setting planar buffer Y address */
		ccap_reg_write(ccap_dev, CCAP_YBA, addr1);

		/* Setting planar buffer U address */
		tmp = addr1 + ccap_dev->pix.width * ccap_dev->pix.height;
		ccap_reg_write(ccap_dev, CCAP_UBA, tmp);

		/* Setting planar buffer V address */
		tmp = tmp + (ccap_dev->pix.width * ccap_dev->pix.height) / 2;
		ccap_reg_write(ccap_dev, CCAP_VBA, tmp);
	}

	/* Update New frame */
	ccap_reg_write(ccap_dev, CCAP_CTL,
		       ccap_reg_read(ccap_dev, CCAP_CTL) | CTL_UPDATE);
	LEAVE();
}

/* Locking: caller holds fop_lock mutex */
static int ccap_queue_setup(struct vb2_queue *vq,
			    unsigned int *nbuffers,
			    unsigned int *nplanes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct ccap_device *ccap_dev = vb2_get_drv_priv(vq);
	struct v4l2_pix_format *pix = &ccap_dev->pix;
	int bytes_per_line = ccap_fmt[ccap_dev->pix_idx].bpp * pix->width / 8;

	ENTRY();
	if (*nplanes)
		return sizes[0] < pix->height * bytes_per_line ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = pix->height * bytes_per_line;
	LEAVE();
	return 0;
}

static int ccap_buf_prepare(struct vb2_buffer *vb)
{
	struct ccap_device *ccap_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format *pix = &ccap_dev->pix;
	unsigned int bytes_per_line = (ccap_fmt[ccap_dev->pix_idx].bpp *
				       pix->width) / 8;
	unsigned int size = pix->height * bytes_per_line;

	ENTRY();
	if (vb2_plane_size(vb, 0) < size) {
		/* User buffer too small */
		dev_warn(ccap_dev->v4l2_dev.dev,
			 "buffer too small (%lu < %u)\n", vb2_plane_size(vb, 0),
			 size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);
	LEAVE();
	return 0;
}

/* Locking: caller holds fop_lock mutex and vq->irqlock spinlock */
static void ccap_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct ccap_device *ccap_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct ccap_buffer *cbuf = to_ccap_buffer(vbuf);
	unsigned long flags;

	ENTRY();
	spin_lock_irqsave(&ccap_dev->lock, flags);
	list_add_tail(&cbuf->list, &ccap_dev->buf_list);
	spin_unlock_irqrestore(&ccap_dev->lock, flags);
	LEAVE();
}

static int ccap_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct ccap_device *ccap_dev = vb2_get_drv_priv(vq);
	struct ccap_buffer *buf, *node;
	int engine;
	int ret;

	ENTRY();
	ccap_dev->sequence = 0;
	ret = v4l2_device_call_until_err(&ccap_dev->v4l2_dev, 0,
					 video, s_stream, 1);

	if (ret < 0 && ret != -ENOIOCTLCMD) {
		list_for_each_entry_safe(buf, node, &ccap_dev->buf_list, list) {
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
			list_del(&buf->list);
		}
		ccap_dev->active = NULL;
		VDEBUG("ccap_dev->active = NULL\n");
		return ret;
	}

	buf = list_entry(ccap_dev->buf_list.next, struct ccap_buffer, list);
	ccap_dev->active = buf;

	engine = (ccap_dev->pdata->packet | ccap_dev->pdata->planar);
	ccap_schedule_next(ccap_dev, &buf->vb);
	/* Enable CAP Interrupt */
	ccap_reg_write(ccap_dev, CCAP_INT,
		       ccap_reg_read(ccap_dev, CCAP_INT) | 0x10000);
	/* Enable CAP engine */
	ccap_reg_write(ccap_dev, CCAP_CTL,
		       ccap_reg_read(ccap_dev,
				     CCAP_CTL) |
		       ((engine << 0) | (ccap_dev->pdata->packet << 6)
			| (ccap_dev->pdata->planar << 5)));

	ccap_dev->status = CCAP_RUNNING;

	LEAVE();
	return 0;
}

static void ccap_stop_streaming(struct vb2_queue *vq)
{
	struct ccap_device *ccap_dev = vb2_get_drv_priv(vq);
	struct ccap_buffer *buf, *node;
	unsigned long flags;
	unsigned int value;

	ENTRY();
	/* Disable output */
	if ((ccap_reg_read(ccap_dev, CCAP_CTL) & ((1 << 6) | (1 << 5)))
	    != 0) {
		/* Clean interrupt flags */
                ccap_reg_write(ccap_dev, CCAP_INT,
                                ccap_reg_read(ccap_dev, CCAP_INT));
		/* Disable interrupt */
		ccap_reg_write(ccap_dev, CCAP_INT, 0);

		value = ccap_reg_read(ccap_dev, CCAP_CTL) | (1 << 16);
		ccap_reg_write(ccap_dev, CCAP_CTL, value);

		/* waiting for the current frame complete */
		while (ccap_reg_read(ccap_dev, CCAP_CTL) & 0x1)
			udelay(10);

		value =
		    ccap_reg_read(ccap_dev, CCAP_CTL) | ~((1 << 6) | (1 << 5));
		ccap_reg_write(ccap_dev, CCAP_CTL, value);
	}
	v4l2_device_call_until_err(&ccap_dev->v4l2_dev, 0, video, s_stream, 0);

	spin_lock_irqsave(&ccap_dev->lock, flags);
	list_for_each_entry_safe(buf, node, &ccap_dev->buf_list, list) {
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		list_del(&buf->list);
	}
	ccap_dev->active = NULL;
	spin_unlock_irqrestore(&ccap_dev->lock, flags);
	LEAVE();
}

static const struct vb2_ops ccap_qops = {
	.queue_setup = ccap_queue_setup,
	.buf_prepare = ccap_buf_prepare,
	.buf_queue = ccap_buf_queue,
	.start_streaming = ccap_start_streaming,
	.stop_streaming = ccap_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int ccap_formats_init(struct ccap_device *ccap)
{
	const struct ccap_format *sd_fmts[ARRAY_SIZE(ccap_formats)];
	unsigned int num_fmts = 0, i, j;
	struct v4l2_subdev *subdev = ccap->entity.source;
	struct v4l2_subdev_mbus_code_enum mbus_code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	while (!v4l2_subdev_call(subdev, pad, enum_mbus_code,
		NULL, &mbus_code)) {
		for (i = 0; i < ARRAY_SIZE(ccap_formats); i++) {
			if (ccap_formats[i].mbus_code != mbus_code.code)
				continue;

			/* Code supported, have we got this fourcc yet? */
			for (j = 0; j < num_fmts; j++)
				if (sd_fmts[j]->fourcc ==
				    ccap_formats[i].fourcc) {
					/* Already available */
					dev_dbg(ccap->dev,
						"Skipping fourcc/code: %4.4s/0x%x\n",
						(char *)&sd_fmts[j]->fourcc,
						mbus_code.code);
					break;
				}
			if (j == num_fmts) {
				/* New */
				sd_fmts[num_fmts++] = ccap_formats + i;
				dev_dbg(ccap->dev,
					"Supported fourcc/code: %4.4s/0x%x\n",
					(char *)&sd_fmts[num_fmts - 1]->fourcc,
					sd_fmts[num_fmts - 1]->mbus_code);
			}
		}
		mbus_code.index++;
	}
	if (!num_fmts)
		return -ENXIO;

	ccap->num_of_sd_formats = num_fmts;
	ccap->sd_formats = devm_kcalloc(ccap->dev,
					num_fmts, sizeof(struct ccap_format *),
					GFP_KERNEL);
	if (!ccap->sd_formats) {
		dev_err(ccap->dev, "Could not allocate memory\n");
		return -ENOMEM;
	}

	memcpy(ccap->sd_formats, sd_fmts,
	       num_fmts * sizeof(struct ccap_format *));
	ccap->sd_format = ccap->sd_formats[0];

	return 0;
}

/* Video IOCTLs */
static int ccap_querycap(struct file *file, void *priv,
			 struct v4l2_capability *cap)
{
	ENTRY();

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE |
	    V4L2_CAP_VIDEO_OVERLAY | V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS;

	strscpy(cap->driver, DRV_NAME, sizeof(cap->driver));
	strscpy(cap->card, "MA35D1 CCAP Interface", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:ccap", sizeof(cap->bus_info));

	LEAVE();
	return 0;
}

/* Enumerate formats, that the device can accept from the user */
static int ccap_enum_fmt_vid_cap(struct file *file,
				 void *priv, struct v4l2_fmtdesc *fmt)
{
	ENTRY();
	if (fmt->index >= ARRAY_SIZE(ccap_fmt))
		return -EINVAL;

	fmt->pixelformat = ccap_fmt[fmt->index].pfmt;
	return 0;
}

static int ccap_try_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct ccap_device *ccap_dev = video_drvdata(file);
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	int pix_idx;
	u32 outfmt;
	u32 heightM, heightN, widthM, widthN;
	u32 u32GCD0;
	u32 value;

	ENTRY();
	dev_dbg(ccap_dev->dev, "pix->width %d, pix->height %d\n",
			pix->width, pix->height);

	if (vb2_is_streaming(&ccap_dev->queue))
		return -EBUSY;

	if (!(fmt->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ||
			fmt->type == V4L2_BUF_TYPE_VIDEO_OVERLAY))
		return -EINVAL;

	for (pix_idx = 0; pix_idx < ARRAY_SIZE(ccap_fmt); pix_idx++)
		if (ccap_fmt[pix_idx].pfmt == pix->pixelformat)
			break;

	ccap_dev->pix_idx = pix_idx;
	if (pix_idx == ARRAY_SIZE(ccap_fmt))
		return -EINVAL;

	v4l_bound_align_image(&pix->width,
			      CCAP_MIN_IMAGE_WIDTH, ccap_dev->fmt.fmt.pix.width,
			      2, &pix->height, CCAP_MIN_IMAGE_HEIGHT,
			      ccap_dev->fmt.fmt.pix.height, 1, 0);

	VDEBUG("pix->width=%d,pix->height=%d\n ", pix->width, pix->height);
	if(fmt->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		pix->bytesperline = pix->width * ccap_fmt[pix_idx].bpl;
	pix->sizeimage = pix->height * ((pix->width * ccap_fmt[pix_idx].bpp)
					>> 3);
	VDEBUG("bytesperline=%d,sizeimage=%d\n ", pix->bytesperline,
	       pix->sizeimage);
	VDEBUG("pix->width=%d,pix->height=%d\n ", pix->width, pix->height);
	/*Set capture format for nuvoton sensor interface */
	ccap_reg_write(ccap_dev, CCAP_CWS,
		(ccap_reg_read(ccap_dev, CCAP_CWS) & ~(0x0fff0fff))
		| (ccap_dev->fmt.fmt.pix.width)
		| (ccap_dev->fmt.fmt.pix.height << 16));

	switch (pix->pixelformat) {
		/* Packet YUV422 */
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_GREY:
		VDEBUG("Packet\n");
		if (pix->pixelformat == V4L2_PIX_FMT_YUYV)
			outfmt = 0 << 4;

		if (pix->pixelformat == V4L2_PIX_FMT_GREY) {
			pix->priv = 8;
			outfmt = 1 << 4;
		}

		if (pix->pixelformat == V4L2_PIX_FMT_RGB555)
			outfmt = 2 << 4;

		if (pix->pixelformat == V4L2_PIX_FMT_RGB565)
			outfmt = 3 << 4;

		value =
		    (ccap_reg_read(ccap_dev, CCAP_PAR) & ~(3 << 4)) | outfmt;
		ccap_reg_write(ccap_dev, CCAP_PAR, value);
		value = (ccap_reg_read(ccap_dev, CCAP_PAR) & ~INMASK);
		if (ccap_dev->sd_format->mbus_code ==
		    MEDIA_BUS_FMT_RGB565_2X8_LE)
			value |= INFMT_RGB565 | INTYPE_CCIR601;
		else if (ccap_dev->sd_format->mbus_code ==
			 MEDIA_BUS_FMT_YUYV8_2X8)
			value |= INFMT_YCbCr | INTYPE_CCIR601;
		ccap_reg_write(ccap_dev, CCAP_PAR, value);

		/* Set_Cropping start position for sensor */
		value =
		    (ccap_reg_read(ccap_dev, CCAP_CWSP) &
		     ~(CWSP_CWSADDRV | CWSP_CWSADDRH)) |
		    ccap_dev->pdata->cropstart;
		ccap_reg_write(ccap_dev, CCAP_CWSP, value);

		/* Packet Scaling Vertical Factor Register (LSB) */
		VDEBUG("pix height=%d, fmt height = %d\n",
		       pix->height, ccap_dev->fmt.fmt.pix.height);

		if (pix->height > ccap_dev->fmt.fmt.pix.height) {
			heightN = 1;
			heightM = 1;

		} else {
			heightM = ccap_dev->fmt.fmt.pix.height;
			heightN = pix->height;
		}

		value =
		    (ccap_reg_read(ccap_dev, CCAP_PKTSL) &
		     ~(PKTSL_PKTSVNL | PKTSL_PKTSVML)) | ((heightN & 0xff) << 24
							  | (heightM & 0xff) <<
							  16);
		ccap_reg_write(ccap_dev, CCAP_PKTSL, value);

		/* Packet Scaling Vertical Factor Register (MSB) */
		value =
		    (ccap_reg_read(ccap_dev, CCAP_PKTSM) &
		     ~(PKTSM_PKTSVNH | PKTSM_PKTSVMH)) | ((heightN >> 8) << 24 |
							  (heightM >> 8) << 16);
		ccap_reg_write(ccap_dev, CCAP_PKTSM, value);

		/* Packet Scaling Horizontal Factor Register (LSB) */
		VDEBUG("pix width=%d, fmt width = %d\n", pix->width,
		       ccap_dev->fmt.fmt.pix.width);

		if (pix->width > ccap_dev->fmt.fmt.pix.width) {
			widthN = 1;
			widthM = 1;
		} else {
			widthM = ccap_dev->fmt.fmt.pix.width;
			widthN = pix->width;
		}

		value =
		    (ccap_reg_read(ccap_dev, CCAP_PKTSL) &
		     ~(PKTSL_PKTSHNL | PKTSL_PKTSHML)) | ((widthN & 0xff) << 8 |
							  (widthM & 0xff) << 0);
		ccap_reg_write(ccap_dev, CCAP_PKTSL, value);

		/* Packet Scaling Horizontal Factor Register (MSB) */
		value =
		    (ccap_reg_read(ccap_dev, CCAP_PKTSM) &
		     ~(PKTSM_PKTSHNH | PKTSM_PKTSHMH)) | ((widthN >> 8) << 8 |
							  (widthM >> 8) << 0);
		ccap_reg_write(ccap_dev, CCAP_PKTSM, value);

		/* Frame Output Pixel Stride Width Register(Packet/Planar) */
		if (pix->pixelformat != V4L2_PIX_FMT_GREY) {
			value =
				(ccap_reg_read(ccap_dev, CCAP_STRIDE) &
				~STRIDE_PKTSTRIDE) | ((pix->bytesperline /
					2) << 0);
			ccap_reg_write(ccap_dev, CCAP_STRIDE, value);

		} else {
			value =
				(ccap_reg_read(ccap_dev, CCAP_STRIDE) &
				~STRIDE_PKTSTRIDE) | ((pix->bytesperline /
				1) << 0);
			ccap_reg_write(ccap_dev, CCAP_STRIDE, value);
		}
		ccap_dev->pdata->packet = 1;

		break;

		/* Planar YUV422 */
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUV411P:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		if (pix->pixelformat == V4L2_PIX_FMT_YUV422P)
			outfmt = 0 << 7;
		if (pix->pixelformat == V4L2_PIX_FMT_NV16)
			outfmt = 0 << 7;
		if (pix->pixelformat == V4L2_PIX_FMT_NV61)
			outfmt = 0 << 7;
		if (pix->pixelformat == V4L2_PIX_FMT_YUV411P)
			outfmt = 1 << 7;

		VDEBUG("Planar, pix  height=%d, width=%d\n",
		       pix->height, pix->width);
		VDEBUG("Planar, fmt height=%d, width=%d\n",
		       ccap_dev->fmt.fmt.pix.height,
		       ccap_dev->fmt.fmt.pix.width);

		value =
		    (ccap_reg_read(ccap_dev, CCAP_PAR) & ~(1 << 7)) | outfmt;
		ccap_reg_write(ccap_dev, CCAP_PAR, value);
		/* Set_Cropping start position for sensor */
		value =
		    (ccap_reg_read(ccap_dev, CCAP_CWSP) &
		     ~(CWSP_CWSADDRV | CWSP_CWSADDRH)) | (0 | 2 << 16);
		ccap_reg_write(ccap_dev, CCAP_CWSP, value);

		/* Planar Scaling Vertical Factor Register (LSB) */
		u32GCD0 = pix->height / ccap_dev->fmt.fmt.pix.height;
		if (u32GCD0 <= 0)
			u32GCD0 = 1;

		value =
		    (ccap_reg_read(ccap_dev, CCAP_PLNSL) &
		     ~(PLNSL_PLNSVNL | PLNSL_PLNSVML)) |
		    (((pix->height /
		       u32GCD0) & 0xff) << 24 | ((ccap_dev->fmt.fmt.pix.height /
						  u32GCD0) & 0xff) << 16);
		ccap_reg_write(ccap_dev, CCAP_PLNSL, value);

		/* Planar Scaling Vertical Factor Register (MSB) */
		u32GCD0 = pix->height / ccap_dev->fmt.fmt.pix.height;

		if (u32GCD0 <= 0)
			u32GCD0 = 1;

		value = (ccap_reg_read(ccap_dev,
				       CCAP_PLNSM) &
			 ~(PLNSM_PLNSVNH | PLNSM_PLNSVMH)) |
		    (((pix->height / u32GCD0) >> 8)
		     << 24 | ((ccap_dev->fmt.fmt.pix.height) >> 8) /
		     u32GCD0 << 16);
		ccap_reg_write(ccap_dev, CCAP_PLNSM, value);
		/* Planar Scaling Horizontal Factor Register (LSB) */
		u32GCD0 = pix->width / ccap_dev->fmt.fmt.pix.width;

		if (u32GCD0 <= 0)
			u32GCD0 = 1;

		value = (ccap_reg_read(ccap_dev,
				       CCAP_PLNSL) &
			 ~(PLNSL_PLNSHNL | PLNSL_PLNSHML)) |
		    (((pix->width /
		       u32GCD0) & 0xff) << 8 |
		     ((ccap_dev->fmt.fmt.pix.width / u32GCD0) & 0xff) << 0);
		ccap_reg_write(ccap_dev, CCAP_PLNSL, value);
		/* Planar Scaling Horizontal Factor Register (MSB) */
		u32GCD0 = pix->width / ccap_dev->fmt.fmt.pix.width;

		if (u32GCD0 <= 0)
			u32GCD0 = 1;

		value = (ccap_reg_read(ccap_dev,
				       CCAP_PLNSM) &
			 ~(PLNSM_PLNSHNH | PLNSM_PLNSHMH)) |
		    (((pix->width / u32GCD0) >> 8)
		     << 8 | ((ccap_dev->fmt.fmt.pix.width / u32GCD0)
			     >> 8) << 0);
		ccap_reg_write(ccap_dev, CCAP_PLNSM, value);

		/* Frame Output Pixel Stride Width Register(Planar) */
		value = (ccap_reg_read(ccap_dev,
				       CCAP_STRIDE) &
			 ~STRIDE_PLNSTRIDE) | ((pix->bytesperline / 2)
					       << 16);
		ccap_reg_write(ccap_dev, CCAP_STRIDE, value);

		ccap_dev->pdata->planar = 1;
		VDEBUG("V4L2_PIX_FMT_YUV422P END\n");
		break;

	default:
		return -EINVAL;
	}

	ccap_reg_write(ccap_dev, 0x30, 0x12);
	/* Set CCAP Polarity */
	value = (ccap_reg_read(ccap_dev,
			       CCAP_PAR) & ~(VSP_HI |
					     HSP_HI |
					     PCLKP_HI)) |
	    ccap_dev->pdata->polarity;
	ccap_reg_write(ccap_dev, CCAP_PAR, value);
	/* Enable CCAP Interrupt */
	value = ccap_reg_read(ccap_dev, CCAP_INT) | 0x10000;
	ccap_reg_write(ccap_dev, CCAP_INT, value);

	LEAVE();

	return 0;
}

static int ccap_g_fmt_vid_cap(struct file *file, void *priv,
			      struct v4l2_format *fmt)
{
	struct ccap_device *ccap_dev = video_drvdata(file);

	ENTRY();
	fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt->fmt.pix = ccap_dev->pix;
	LEAVE();
	return 0;
}

static int ccap_log_status(struct file *file, void *priv)
{
	struct ccap_device *ccap_dev = video_drvdata(file);

	ENTRY();
	pr_info("CCAP_CTL:        0x%08x\n", ccap_reg_read(ccap_dev, CCAP_CTL));
	pr_info("CCAP_PAR:        0x%08x\n", ccap_reg_read(ccap_dev, CCAP_PAR));
	pr_info("CCAP_INT:        0x%08x\n", ccap_reg_read(ccap_dev, CCAP_INT));
	pr_info("CCAP_POSTERIZE:  0x%08x\n",
		ccap_reg_read(ccap_dev, CCAP_POSTERIZE));
	pr_info("CCAP_CWSP:       0x%08x\n",
		ccap_reg_read(ccap_dev, CCAP_CWSP));
	pr_info("CCAP_CWS:        0x%08x\n", ccap_reg_read(ccap_dev, CCAP_CWS));
	pr_info("CCAP_PKTSL:      0x%08x\n",
		ccap_reg_read(ccap_dev, CCAP_PKTSL));
	pr_info("CCAP_PLNSL:      0x%08x\n",
		ccap_reg_read(ccap_dev, CCAP_PLNSL));
	pr_info("CCAP_STRIDE:     0x%08x\n",
		ccap_reg_read(ccap_dev, CCAP_STRIDE));
	pr_info("CCAP_PKTSM:      0x%08x\n",
		ccap_reg_read(ccap_dev, CCAP_PKTSM));
	pr_info("CCAP_PLNSM:      0x%08x\n",
		ccap_reg_read(ccap_dev, CCAP_PLNSM));
	pr_info("CCAP_PKTBA0:     0x%08x\n",
		ccap_reg_read(ccap_dev, CCAP_PKTBA0));
	pr_info("CCAP_YBA:        0x%08x\n", ccap_reg_read(ccap_dev, CCAP_YBA));
	pr_info("CCAP_UBA:        0x%08x\n", ccap_reg_read(ccap_dev, CCAP_UBA));
	pr_info("CCAP_VBA:        0x%08x\n", ccap_reg_read(ccap_dev, CCAP_VBA));
	LEAVE();
	return 0;
}

static int ccap_enum_input(struct file *file, void *priv, struct v4l2_input *i)
{
	if (i->index > 0)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(i->name, "Camera", sizeof(i->name));

	return 0;
}

static int ccap_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;

	return 0;
}

static int ccap_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i > 0)
		return -EINVAL;

	return 0;
}

static irqreturn_t ccap_isr(int irq, void *dev_id)
{
	static unsigned long j;
	struct ccap_device *ccap_dev = dev_id;
	struct ccap_buffer *vb;
	u32 irq_status = ccap_reg_read(ccap_dev, CCAP_INT);

	ENTRY();
	VDEBUG("irq_status=0x%08x\n", irq_status);
#ifdef EN_OVERLAY
	if (ccap_dev->status == CCAP_OVERLAY) {
		ccap_reg_write(ccap_dev, CCAP_INT, irq_status);
		if (ccap_dev->dcultra_framebase ==
			ccap_reg_read(ccap_dev, CCAP_PKTBA0)) {
			__raw_writel(ccap_dev->dcultra_framebase,
					ccap_dev->display_base + 0x1400);
			ccap_reg_write(ccap_dev, CCAP_PKTBA0,
					ccap_dev->dcultra_framebase +
					ccap_dev->dcultra_framesize);
		} else {
                      __raw_writel(ccap_dev->dcultra_framebase +
				      ccap_dev->dcultra_framesize,
				      ccap_dev->display_base + 0x1400);
			ccap_reg_write(ccap_dev, CCAP_PKTBA0,
					ccap_dev->dcultra_framebase);
		}
		ccap_reg_write(ccap_dev, CCAP_CTL,
				ccap_reg_read(ccap_dev, CCAP_CTL) | CTL_UPDATE);
		return IRQ_HANDLED;
	}
#endif
	/* Clear only set interrupts */
	ccap_reg_write(ccap_dev, CCAP_INT, irq_status);

	spin_lock(&ccap_dev->lock);
	if (!ccap_dev->active || list_empty(&ccap_dev->buf_list)) {
		if (printk_timed_ratelimit(&j, 500))
			dev_warn(ccap_dev->v4l2_dev.dev,
				 "IRQ without active buffer: %x!\n",
				 irq_status);

		/* Just ack: buf_release will disable further interrupts */
		ccap_reg_write(ccap_dev, CCAP_INT, 0);
		spin_unlock(&ccap_dev->lock);
		return IRQ_HANDLED;
	}

	vb = ccap_dev->active;
	if (list_is_singular(&vb->list)) {
		/* Keep cycling while no next buffer is available */
		spin_unlock(&ccap_dev->lock);
		LEAVE();
		return IRQ_HANDLED;
	}

	list_del(&vb->list);
	vb->vb.vb2_buf.timestamp = ktime_get_ns();
	vb->vb.sequence = ccap_dev->sequence++;
	vb->vb.field = V4L2_FIELD_NONE;
	vb2_buffer_done(&vb->vb.vb2_buf, VB2_BUF_STATE_DONE);
	ccap_dev->active = list_entry(ccap_dev->buf_list.next,
				      struct ccap_buffer, list);
	ccap_schedule_next(ccap_dev, &ccap_dev->active->vb);

	spin_unlock(&ccap_dev->lock);
	LEAVE();
	return IRQ_HANDLED;
}

static int ccap_pipeline_s_fmt(struct ccap_device *ccap,
			       struct v4l2_subdev_pad_config *pad_cfg,
			       struct v4l2_subdev_format *format)
{
	struct media_entity *entity = &ccap->entity.source->entity;
	struct v4l2_subdev *subdev;
	struct media_pad *sink_pad = NULL;
	struct media_pad *src_pad = NULL;
	struct media_pad *pad = NULL;
	struct v4l2_subdev_format fmt = *format;
	bool found = false;
	int ret;

	ENTRY();
	/*
	 * Starting from sensor subdevice, walk within
	 * pipeline and set format on each subdevice
	 */
	while (1) {
		unsigned int i;

		/* Search if current entity has a source pad */
		for (i = 0; i < entity->num_pads; i++) {
			pad = &entity->pads[i];
			if (pad->flags & MEDIA_PAD_FL_SOURCE) {
				src_pad = pad;
				found = true;
				break;
			}
		}
		if (!found)
			break;

		subdev = media_entity_to_v4l2_subdev(entity);

		/* Propagate format on sink pad if any, otherwise source pad */
		if (sink_pad)
			pad = sink_pad;

		dev_dbg(ccap->dev, "\"%s\":%d pad format set to 0x%x %ux%u\n",
			subdev->name, pad->index, format->format.code,
			format->format.width, format->format.height);

		fmt.pad = pad->index;
		ret = v4l2_subdev_call(subdev, pad, set_fmt, pad_cfg, &fmt);
		if (ret < 0) {
			dev_err(ccap->dev,
				"%s: Failed to set format 0x%x %ux%u on \"%s\":%d pad (%d)\n",
				__func__, format->format.code,
				format->format.width, format->format.height,
				subdev->name, pad->index, ret);
			return ret;
		}

		if (fmt.format.code != format->format.code ||
		    fmt.format.width != format->format.width ||
		    fmt.format.height != format->format.height) {
			dev_dbg(ccap->dev,
				"\"%s\":%d pad format has been changed to 0x%x %ux%u\n",
				subdev->name, pad->index, fmt.format.code,
				fmt.format.width, fmt.format.height);
		}

		/* Walk to next entity */
		sink_pad = media_entity_remote_pad(src_pad);
		if (!sink_pad || !is_media_entity_v4l2_subdev(sink_pad->entity))
			break;

		entity = sink_pad->entity;
	}
	*format = fmt;
	LEAVE();
	return 0;
}

static int ccap_set_fmt(struct ccap_device *ccap, struct v4l2_format *f)
{
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	const struct ccap_format *sd_format;
	struct ccap_framesize sd_framesize;
	struct v4l2_mbus_framefmt *mf = &format.format;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int ret;

	ENTRY();
	/*
	 * Try format, fmt.width/height could have been changed
	 * to match sensor capability or crop request
	 * sd_format & sd_framesize will contain what subdev
	 * can do for this request.
	 */
	dev_dbg(ccap->dev, "pix->width %d, pix->height %d\n",
			pix->width, pix->height);
	ret = ccap_try_fmt(ccap, f, &sd_format, &sd_framesize);
	if (ret)
		return ret;

	/* pix to mbus format */
	v4l2_fill_mbus_format(mf, pix, sd_format->mbus_code);
	mf->width = sd_framesize.width;
	mf->height = sd_framesize.height;

	ret = ccap_pipeline_s_fmt(ccap, NULL, &format);
	if (ret < 0)
		return ret;

	dev_dbg(ccap->dev, "Sensor format set to 0x%x %ux%u\n",
		mf->code, mf->width, mf->height);
	dev_dbg(ccap->dev, "Buffer format set to %4.4s %ux%u\n",
		(char *)&pix->pixelformat, pix->width, pix->height);

	ccap->fmt = *f;
	ccap->sd_format = sd_format;
	ccap->sd_framesize = sd_framesize;

	LEAVE();
	return 0;
}

static int ccap_s_fmt_vid_cap(struct file *file, void *priv,
			      struct v4l2_format *fmt)
{
	struct ccap_device *ccap_dev = video_drvdata(file);
	int ret;

	ENTRY();
	dev_dbg(ccap_dev->dev, "width %d, heigth %d\n",
			fmt->fmt.pix.width, fmt->fmt.pix.height);

	memcpy(&ccap_dev->fmt, fmt, sizeof(struct v4l2_format));
	ccap_set_fmt(ccap_dev, &ccap_dev->fmt);
	ret = ccap_try_fmt_vid_cap(file, priv, fmt);
	if (ret < 0)
		return ret;
	memcpy(&ccap_dev->pix, &fmt->fmt.pix, sizeof(struct v4l2_pix_format));
	LEAVE();
	return 0;
}

static int ccap_enum_framesizes(struct file *file, void *priv,
                                     struct v4l2_frmsizeenum *fsize)
{

	struct ccap_device *ccap_dev = video_drvdata(file);
        const struct ccap_format *sd_fmt;
	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	sd_fmt = find_format_by_fourcc(ccap_dev, fsize->pixel_format);
	if (!sd_fmt)
		return -EINVAL;

	fse.code = sd_fmt->mbus_code;

	ret = v4l2_subdev_call(ccap_dev->entity.source, pad, enum_frame_size,
			NULL, &fse);
	if (ret)
		return ret;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = fse.max_width;
	fsize->discrete.height = fse.max_height;

	return 0;
}

#ifdef EN_OVERLAY
static int ccap_overlay(struct file *file, void *priv, unsigned int on)
{
	struct ccap_device *ccap_dev = video_drvdata(file);

	dev_dbg(ccap_dev->dev, "VIDIOC_OVERLAY on:%d\n", on);
	if (on) {
		int engine, framesize;

		engine = (ccap_dev->pdata->packet | ccap_dev->pdata->planar);

		/* Set dcultra base to ccap base */
		ccap_dev->dcultra_framebase =
			__raw_readl(ccap_dev->display_base + 0x1400);
		ccap_reg_write(ccap_dev, CCAP_PKTBA0,
				ccap_dev->dcultra_framebase);

		/* Get dcultra frame size */
		framesize = __raw_readl(ccap_dev->display_base + 0x1810);
		ccap_dev->dcultra_framesize = ((framesize>>15)&0xffff) *
				(framesize&0xffff) *
				ccap_fmt[ccap_dev->pix_idx].bpl;
		/* Enable CAP engine */
		ccap_reg_write(ccap_dev, CCAP_CTL,
			ccap_reg_read(ccap_dev,
			CCAP_CTL) |
			((engine << 0) | (ccap_dev->pdata->packet << 6)
			| (ccap_dev->pdata->planar << 5)));
		ccap_dev->status = CCAP_OVERLAY;
	} else {
		unsigned int value;

		/* Disable output */
		if ((ccap_reg_read(ccap_dev, CCAP_CTL) & ((1 << 6) | (1 << 5)))
		!= 0) {
			value = ccap_reg_read(ccap_dev, CCAP_CTL) | (1 << 16);
			ccap_reg_write(ccap_dev, CCAP_CTL, value);

			/* waiting for the current frame complete */
			while (ccap_reg_read(ccap_dev, CCAP_CTL) & 0x1)
				udelay(10);

			value = ccap_reg_read(ccap_dev, CCAP_CTL) |
				~((1 << 6) | (1 << 5));
			ccap_reg_write(ccap_dev, CCAP_CTL, value);
		}
		ccap_dev->status = CCAP_IDLE;
	}

	return 0;
}
#endif
static inline struct ccap_device *notifier_to_ccap(struct v4l2_async_notifier
						   *n)
{
	return container_of(n, struct ccap_device, notifier);
}

static int ccap_sensor_bound(struct v4l2_async_notifier *notifier,
			     struct v4l2_subdev *subdev,
			     struct v4l2_async_subdev *asd)
{
	struct ccap_device *ccap = notifier_to_ccap(notifier);
	unsigned int ret;
	int src_pad;

	ENTRY();
	dev_dbg(ccap->dev, "Subdev \"%s\" bound\n", subdev->name);

	/*
	 * Link this sub-device to CCAP, it could be
	 * a parallel camera sensor or a bridge
	 */
	src_pad = media_entity_get_fwnode_pad(&subdev->entity,
					      subdev->fwnode,
					      MEDIA_PAD_FL_SOURCE);
	ret = media_create_pad_link(&subdev->entity, src_pad,
				    &ccap->vdev->entity, 0,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
	if (ret)
		dev_err(ccap->dev,
			"Failed to create media pad link with subdev \"%s\"\n",
			subdev->name);
	else
		dev_dbg(ccap->dev, "CCAP is now linked to \"%s\"\n",
			subdev->name);

	LEAVE();
	return ret;
}

static void ccap_sensor_unbind(struct v4l2_async_notifier *notifier,
			       struct v4l2_subdev *subdev,
			       struct v4l2_async_subdev *asd)
{

	struct ccap_device *ccap = notifier_to_ccap(notifier);

	dev_dbg(ccap->dev, "Removing %s\n", video_device_node_name(ccap->vdev));

	/* Checks internally if vdev has been init or not */
	video_unregister_device(ccap->vdev);
}

static struct media_entity *ccap_find_source(struct ccap_device *ccap)
{
	struct media_entity *entity = &ccap->vdev->entity;
	struct media_pad *pad;

	/* Walk searching for entity having no sink */
	while (1) {
		pad = &entity->pads[0];
		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
			break;

		entity = pad->entity;
	}

	return entity;
}

static int ccap_get_sensor_format(struct ccap_device *ccap,
				  struct v4l2_pix_format *pix)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	ret = v4l2_subdev_call(ccap->entity.source, pad, get_fmt, NULL, &fmt);
	if (ret)
		return ret;
	v4l2_fill_pix_format(pix, &fmt.format);
	LEAVE();
	return 0;
}

static int ccap_get_sensor_bounds(struct ccap_device *ccap, struct v4l2_rect *r)
{
	struct v4l2_subdev_selection bounds = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.target = V4L2_SEL_TGT_CROP_BOUNDS,
	};
	unsigned int max_width, max_height, max_pixsize;
	struct v4l2_pix_format pix;
	unsigned int i;
	int ret;

	ENTRY();
	/*
	 * Get sensor bounds first
	 */
	ret = v4l2_subdev_call(ccap->entity.source, pad, get_selection,
			       NULL, &bounds);
	if (!ret)
		*r = bounds.r;
	if (ret != -ENOIOCTLCMD)
		return ret;

	/*
	 * If selection is not implemented,
	 * fallback by enumerating sensor frame sizes
	 * and take the largest one
	 */
	max_width = 0;
	max_height = 0;
	max_pixsize = 0;
	for (i = 0; i < ccap->num_of_sd_framesizes; i++) {
		struct ccap_framesize *fsize = &ccap->sd_framesizes[i];
		unsigned int pixsize = fsize->width * fsize->height;

		if (pixsize > max_pixsize) {
			max_pixsize = pixsize;
			max_width = fsize->width;
			max_height = fsize->height;
		}
	}
	if (max_pixsize > 0) {
		r->top = 0;
		r->left = 0;
		r->width = max_width;
		r->height = max_height;
		LEAVE();
		return 0;
	}

	/*
	 * If frame sizes enumeration is not implemented,
	 * fallback by getting current sensor frame size
	 */
	ret = ccap_get_sensor_format(ccap, &pix);
	if (ret)
		return ret;

	r->top = 0;
	r->left = 0;
	r->width = pix.width;
	r->height = pix.height;

	LEAVE();
	return 0;
}

static int ccap_framesizes_init(struct ccap_device *ccap)
{
	unsigned int num_fsize = 0;
	struct v4l2_subdev *subdev = ccap->entity.source;
	struct v4l2_subdev_frame_size_enum fse = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.code = ccap->sd_format->mbus_code,
	};
	unsigned int ret;
	unsigned int i;

	ENTRY();
	/* Allocate discrete framesizes array */
	while (!v4l2_subdev_call(subdev, pad, enum_frame_size, NULL, &fse))
		fse.index++;

	num_fsize = fse.index;
	if (!num_fsize)
		return 0;

	ccap->num_of_sd_framesizes = num_fsize;
	ccap->sd_framesizes = devm_kcalloc(ccap->dev, num_fsize,
					   sizeof(struct ccap_framesize),
					   GFP_KERNEL);
	if (!ccap->sd_framesizes) {
		dev_err(ccap->dev, "Could not allocate memory\n");
		return -ENOMEM;
	}

	/* Fill array with sensor supported framesizes */
	dev_dbg(ccap->dev, "Sensor supports %u frame sizes:\n", num_fsize);
	for (i = 0; i < ccap->num_of_sd_framesizes; i++) {
		fse.index = i;
		ret = v4l2_subdev_call(subdev, pad, enum_frame_size,
				       NULL, &fse);
		if (ret)
			return ret;
		ccap->sd_framesizes[fse.index].width = fse.max_width;
		ccap->sd_framesizes[fse.index].height = fse.max_height;
		dev_dbg(ccap->dev, "%ux%u\n", fse.max_width, fse.max_height);
	}
	LEAVE();
	return 0;
}

static int ccap_sensor_complete(struct v4l2_async_notifier *notifier)
{

	struct ccap_device *ccap = notifier_to_ccap(notifier);
	int ret;

	ENTRY();
	/*
	 * Now that the graph is complete,
	 * we search for the source subdevice
	 * in order to expose it through V4L2 interface
	 */
	ccap->entity.source =
	    media_entity_to_v4l2_subdev(ccap_find_source(ccap));
	if (!ccap->entity.source) {
		dev_err(ccap->dev, "Source subdevice not found\n");
		return -ENODEV;
	}

	ccap->vdev->ctrl_handler = ccap->entity.source->ctrl_handler;

	ret = ccap_formats_init(ccap);
	if (ret) {
		dev_err(ccap->dev, "No supported mediabus format found\n");
		return ret;
	}

	ret = ccap_framesizes_init(ccap);
	if (ret) {
		dev_err(ccap->dev, "Could not initialize framesizes\n");
		return ret;
	}

	ret = ccap_get_sensor_bounds(ccap, &ccap->sd_bounds);
	if (ret) {
		dev_err(ccap->dev, "Could not get sensor bounds\n");
		return ret;
	}

	ret = ccap_set_default_fmt(ccap);
	if (ret) {
		dev_err(ccap->dev, "Could not set default format\n");
		return ret;
	}

	LEAVE();
	return 0;

}

/* File operations */
static int ccap_open(struct file *file)
{
	struct ccap_device *ccap_dev = video_drvdata(file);
	struct v4l2_subdev *sd = ccap_dev->entity.source;
	int err;

	ENTRY();
	err = v4l2_fh_open(file);
	if (err)
		goto done_open;

	if (!v4l2_fh_is_singular_file(file))
		goto done_open;

	err = v4l2_subdev_call(sd, core, s_power, 1);
	if (err < 0 && err != -ENOIOCTLCMD)
		goto fh_rel;

	err = ccap_set_fmt(ccap_dev, &ccap_dev->fmt);
	if (err)
		v4l2_subdev_call(sd, core, s_power, 0);

fh_rel:
	if (err)
		v4l2_fh_release(file);
done_open:
	LEAVE();
	return err;
}

static int ccap_release(struct file *file)
{
	struct ccap_device *ccap_dev = video_drvdata(file);
	bool is_last;

	ENTRY();
	is_last = v4l2_fh_is_singular_file(file);
	_vb2_fop_release(file, NULL);

	if (is_last) {
		/* Last close */
		ccap_dev->status = CCAP_INITIALISING;
		ccap_reg_write(ccap_dev, CCAP_CTL, 0);
		pm_runtime_put(ccap_dev->v4l2_dev.dev);
	}
	LEAVE();
	return 0;
}

static const struct v4l2_async_notifier_operations ccap_sensor_ops = {
	.bound = ccap_sensor_bound,
	.unbind = ccap_sensor_unbind,
	.complete = ccap_sensor_complete,
};

/* nuc980_ccap display ioctl operations */
static const struct v4l2_ioctl_ops ccap_ioctl_ops = {
	.vidioc_querycap = ccap_querycap,
	.vidioc_log_status = ccap_log_status,
	.vidioc_enum_fmt_vid_cap = ccap_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = ccap_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = ccap_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = ccap_s_fmt_vid_cap,
	.vidioc_enum_input = ccap_enum_input,
	.vidioc_g_input = ccap_g_input,
	.vidioc_s_input = ccap_s_input,
	.vidioc_enum_framesizes = ccap_enum_framesizes,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_expbuf = vb2_ioctl_expbuf,

#ifdef EN_OVERLAY
	.vidioc_overlay = ccap_overlay,
	.vidioc_g_fmt_vid_overlay    = ccap_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_overlay  = ccap_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_overlay    = ccap_s_fmt_vid_cap,
#endif

	.vidioc_subscribe_event         = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event       = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations ccap_fops = {
	.owner = THIS_MODULE,
	.open = ccap_open,
	.release = ccap_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
	.write = vb2_fop_write,
};

static const struct video_device ccap_video_template = {
	.name = "nuc980_ccap",
	.fops = &ccap_fops,
	.ioctl_ops = &ccap_ioctl_ops,
	/* PAL only supported in 8-bit non-bt656 mode */
	.tvnorms = V4L2_STD_525_60,
	.vfl_dir = VFL_DIR_RX,
	.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE |
	    V4L2_CAP_STREAMING | V4L2_FBUF_CAP_EXTERNOVERLAY,
};

static int ccap_data_from_dt(struct platform_device *pdev,
			     struct ccap_device *ccap_dev)
{
	int polarity = 0;
	int err;
	//u32 mclk_rate;
	struct device_node *np = pdev->dev.of_node;
	struct v4l2_fwnode_endpoint ep = {.bus_type = 0 };
	struct v4l2_fwnode_bus_parallel *bus = &ep.bus.parallel;

	ccap_dev->pdata = &ccap_data;

	np = of_graph_get_next_endpoint(np, NULL);
	if (!np) {
		pr_info("could not find endpoint\n");
		return -EINVAL;
	}
	err = v4l2_fwnode_endpoint_parse(of_fwnode_handle(np), &ep);
	if (err) {
		pr_info("could not parse endpoint\n");
		return -EINVAL;
	}

	if (ep.bus_type & V4L2_MBUS_BT656)
		ccap_dev->pdata->infmtord =
		    (INORD_YUYV | INFMT_YCbCr | INTYPE_CCIR656);

	if (bus->flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH)
		polarity = HSP_HI;
	else
		polarity = HSP_LO;

	if (bus->flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH)
		polarity |= VSP_HI;
	else
		polarity |= VSP_LO;

	if (bus->flags & V4L2_MBUS_PCLK_SAMPLE_RISING)
		polarity |= PCLKP_HI;
	else
		polarity |= PCLKP_LO;
	ccap_dev->pdata->polarity = polarity;

	if (ep.bus_type & V4L2_MBUS_PARALLEL)
		VDEBUG("parallel mode\n");
	else if (ep.bus_type & V4L2_MBUS_BT656)
		VDEBUG("bt656 mode\n");
	else
		VDEBUG("unknown mode\n");
	VDEBUG("hsync-active=%d\n",
	       (bus->flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH) ? 1 : 0);
	VDEBUG("vsync-active=%d\n",
	       (bus->flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH) ? 1 : 0);
	VDEBUG("pclk-sample=%d\n",
	       (bus->flags & V4L2_MBUS_PCLK_SAMPLE_RISING) ? 1 : 0);
	VDEBUG("bus-width=%d\n", bus->bus_width);
	return 0;
}

static int ccap_graph_parse(struct ccap_device *ccap, struct device_node *node)
{
	struct device_node *ep = NULL;
	struct device_node *remote;

	ep = of_graph_get_next_endpoint(node, ep);
	if (!ep)
		return -EINVAL;

	remote = of_graph_get_remote_port_parent(ep);
	of_node_put(ep);
	if (!remote)
		return -EINVAL;

	/* Remote node to connect */
	ccap->entity.remote_node = remote;
	ccap->entity.asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
	ccap->entity.asd.match.fwnode = of_fwnode_handle(remote);
	return 0;
}

static int ccap_graph_init(struct ccap_device *ccap)
{
	int ret;

	/* Parse the graph to extract a list of subdevice DT nodes. */
	ret = ccap_graph_parse(ccap, ccap->dev->of_node);
	if (ret < 0)
		return ret;

	v4l2_async_notifier_init(&ccap->notifier);

	ret = v4l2_async_notifier_add_subdev(&ccap->notifier,
					     &ccap->entity.asd);
	if (ret) {
		of_node_put(ccap->entity.remote_node);
		return ret;
	}

	ccap->notifier.ops = &ccap_sensor_ops;

	ret = v4l2_async_notifier_register(&ccap->v4l2_dev, &ccap->notifier);
	if (ret < 0) {
		v4l2_async_notifier_cleanup(&ccap->notifier);
		return ret;
	}

	return 0;
}

static int ccap_probe(struct platform_device *pdev)
{
	struct v4l2_pix_format *pix;
	struct video_device *vdev;
	struct ccap_device *ccap_dev;
	struct resource *reg_res;
	struct vb2_queue *q;
	int irq, ret;

	ENTRY();
	pr_info("%s - pdev = %s\n", __func__, pdev->name);
	reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	irq = platform_get_irq(pdev, 0);

	if (!reg_res || irq <= 0) {
		dev_err(&pdev->dev,
			"Insufficient CCAP platform information.\n");
		return -ENODEV;
	}

	ccap_dev = devm_kzalloc(&pdev->dev, sizeof(*ccap_dev), GFP_KERNEL);

	if (!ccap_dev)
		return -ENOMEM;
	INIT_LIST_HEAD(&ccap_dev->buf_list);
	spin_lock_init(&ccap_dev->lock);
	mutex_init(&ccap_dev->fop_lock);

	ccap_dev->v4l2_dev.mdev = &ccap_dev->mdev;

	/* Initialize media device */
	strscpy(ccap_dev->mdev.model, DRV_NAME, sizeof(ccap_dev->mdev.model));
	snprintf(ccap_dev->mdev.bus_info, sizeof(ccap_dev->mdev.bus_info),
		 "platform:%s", DRV_NAME);
	ccap_dev->mdev.dev = &pdev->dev;
	media_device_init(&ccap_dev->mdev);

	ccap_dev->dev = &pdev->dev;
	ccap_dev->status = CCAP_INITIALISING;
	ccap_dev->pix_idx = 1;

#ifdef EN_OVERLAY
	ccap_dev->display_base = ioremap(0x40260000, 0x2000);
#endif
	pix = &ccap_dev->pix;
	/* Fill in defaults */
	pix->width = CCAP_MAX_IMAGE_WIDTH;
	pix->height = CCAP_MAX_IMAGE_HEIGHT;
	pix->pixelformat = V4L2_PIX_FMT_RGB565;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = CCAP_MAX_IMAGE_WIDTH * 2;
	pix->sizeimage = CCAP_MAX_IMAGE_WIDTH * 2 * CCAP_MAX_IMAGE_HEIGHT;
	pix->colorspace = V4L2_COLORSPACE_DEFAULT;
	ccap_dev->base = devm_ioremap_resource(&pdev->dev, reg_res);

	if (IS_ERR(ccap_dev->base))
		return PTR_ERR(ccap_dev->base);

	ret = devm_request_irq(&pdev->dev, irq, ccap_isr, 0,
			       pdev->name, ccap_dev);

	if (ret < 0)
		return ret;

	ret = v4l2_device_register(&pdev->dev, &ccap_dev->v4l2_dev);

	if (ret < 0) {
		dev_err(&pdev->dev, "Error registering v4l2 device\n");
		return ret;
	}

	ccap_dev->vdev = video_device_alloc();
	if (!ccap_dev->vdev) {
		ret = -ENOMEM;
		goto evregdev;
	}

	vdev = ccap_dev->vdev;
	ccap_data_from_dt(pdev, ccap_dev);

	vdev->v4l2_dev = &ccap_dev->v4l2_dev;
	vdev->release = video_device_release_empty;
	vdev->lock = &ccap_dev->fop_lock;

	/* Video node */
	vdev->fops = &ccap_fops;
	vdev->v4l2_dev = &ccap_dev->v4l2_dev;
	vdev->queue = &ccap_dev->queue;
	strscpy(vdev->name, KBUILD_MODNAME, sizeof(ccap_dev->vdev->name));
	vdev->release = video_device_release;
	vdev->ioctl_ops = &ccap_ioctl_ops;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
	    V4L2_CAP_READWRITE;

	video_set_drvdata(vdev, ccap_dev);

	/* Media entity pads */
	ccap_dev->vid_cap_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&ccap_dev->vdev->entity,
				     1, &ccap_dev->vid_cap_pad);
	if (ret) {
		dev_err(ccap_dev->dev, "Failed to init media entity pad\n");
		return ret;
	}
	ccap_dev->vdev->entity.flags |= MEDIA_ENT_FL_DEFAULT;

	/* Initialize the vb2 queue */
	q = &ccap_dev->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->drv_priv = ccap_dev;
	q->buf_struct_size = sizeof(struct ccap_buffer);
	q->ops = &ccap_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 3;
	q->lock = &ccap_dev->fop_lock;
	q->dev = &pdev->dev;
	ret = vb2_queue_init(q);
	if (ret)
		goto evregdev;

	vdev->queue = q;
	INIT_LIST_HEAD(&ccap_dev->buf_list);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_resume(&pdev->dev);
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret < 0)
		goto evregdev;

	ret = ccap_graph_init(ccap_dev);
	if (ret < 0)
		goto evregdev;

	LEAVE();
	return 0;

evregdev:
	pm_runtime_disable(&pdev->dev);
	v4l2_device_unregister(&ccap_dev->v4l2_dev);
	return ret;
}

static int ccap_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct ccap_device *ccap_dev = container_of(v4l2_dev,
						    struct
						    ccap_device,
						    v4l2_dev);

	ENTRY();
	pm_runtime_disable(&pdev->dev);
	video_unregister_device(ccap_dev->vdev);
	v4l2_device_unregister(&ccap_dev->v4l2_dev);
	return 0;
}

static const struct of_device_id ccap_of_match[] = {
	{.compatible = "nuvoton,nuc980-ccap" },
	{ },
};

static struct platform_driver __refdata nuc980_ccap = {
	.probe = ccap_probe,
	.remove = ccap_remove,
	.driver = {
		   .name = "nuc980-ccap",
		   .of_match_table = of_match_ptr(ccap_of_match),
		},
};

module_platform_driver_probe(nuc980_ccap, ccap_probe);

MODULE_DESCRIPTION("NUC980 CCAP driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0");
MODULE_ALIAS("platform:nuc980-ccap");
