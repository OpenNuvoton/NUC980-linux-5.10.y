// SPDX-License-Identifier: GPL-2.0+
//
// drivers/media/i2c/nt99141.c
//
// This file contains a driver for the NT99141 sensor
//
// Copyright (C) 2020 Nuvoton Technology Corp.

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

MODULE_DESCRIPTION("NT99141 I2C subdev driver");
MODULE_LICENSE("GPL v2");

/* min/typical/max system clock (xclk) frequencies */
#define NT99141_XCLK_MIN  6000000
#define NT99141_XCLK_MAX 48000000

#define NT99141_DEFAULT_SLAVE_ID 0x2A

enum nt99141_mode_id {
	NT99141_MODE_VGA_640_480 = 0,
	NT99141_NUM_MODES,
};


enum nt99141_format_mux {
	NT99141_FMT_MUX_YUV422 = 0,
	NT99141_FMT_MUX_RGB,
};

struct nt99141_pixfmt {
	u32 code;
	u32 colorspace;
};

static const struct nt99141_pixfmt nt99141_formats[] = {
//	{ MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_SRGB, },
//	{ MEDIA_BUS_FMT_RGB565_2X8_LE, V4L2_COLORSPACE_SRGB, },
//	{ MEDIA_BUS_FMT_RGB565_2X8_BE, V4L2_COLORSPACE_SRGB, },
};

/*
 * Image size under 1280 * 960 are SUBSAMPLING
 * Image size upper 1280 * 960 are SCALING
 */
enum nt99141_downsize_mode {
	SUBSAMPLING,
	SCALING,
};

struct reg_value {
	u16 reg_addr;
	u8 val;
};

struct nt99141_mode_info {
	enum nt99141_mode_id id;
	enum nt99141_downsize_mode dn_mode;
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
	const struct reg_value *reg_data;
	u32 reg_data_size;
};

struct nt99141_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
	};
	struct {
		struct v4l2_ctrl *auto_wb;
		struct v4l2_ctrl *blue_balance;
		struct v4l2_ctrl *red_balance;
	};
	struct {
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *light_freq;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

struct nt99141_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to NT99141 */
	u32 xclk_freq;

	//struct regulator_bulk_data supplies[NT99141_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	bool   upside_down;

	/* lock to protect all members below */
	struct mutex lock;

	int power_count;

	struct v4l2_mbus_framefmt fmt;
	bool pending_fmt_change;

	const struct nt99141_mode_info *current_mode;
	const struct nt99141_mode_info *last_mode;
	//enum nt99141_frame_rate current_fr;
	struct v4l2_fract frame_interval;

	struct nt99141_ctrls ctrls;

	u32 prev_sysclk, prev_hts;
	u32 ae_low, ae_high, ae_target;

	bool pending_mode_change;
	bool streaming;
};

static inline struct nt99141_dev *to_nt99141_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct nt99141_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct nt99141_dev,
			     ctrls.handler)->sd;
}

static struct reg_value nt99141_setting_VGA_640_480[] = {
	/* [Inti] */
	{0x3109, 0x04}, {0x3040, 0x04}, {0x3041, 0x02}, {0x3042, 0xFF},
	{0x3043, 0x08}, {0x3052, 0xE0}, {0x305F, 0x33}, {0x3100, 0x07},
	{0x3106, 0x03}, {0x3108, 0x00}, {0x3110, 0x22}, {0x3111, 0x57},
	{0x3112, 0x22}, {0x3113, 0x55}, {0x3114, 0x05}, {0x3135, 0x00},
	{0x32F0, 0x01}, {0x306a, 0x01},
	/* Initial AWB Gain */
	{0x3290, 0x01}, {0x3291, 0x80}, {0x3296, 0x01}, {0x3297, 0x73},
	/* CA Ratio */
	{0x3250, 0x80}, {0x3251, 0x03}, {0x3252, 0xFF}, {0x3253, 0x00},
	{0x3254, 0x03}, {0x3255, 0xFF}, {0x3256, 0x00}, {0x3257, 0x50},
	/* Gamma */
	{0x3270, 0x00}, {0x3271, 0x0C}, {0x3272, 0x18}, {0x3273, 0x32},
	{0x3274, 0x44}, {0x3275, 0x54}, {0x3276, 0x70}, {0x3277, 0x88},
	{0x3278, 0x9D}, {0x3279, 0xB0}, {0x327A, 0xCF}, {0x327B, 0xE2},
	{0x327C, 0xEF}, {0x327D, 0xF7}, {0x327E, 0xFF},
	/* Color Correction */
	{0x3302, 0x00}, {0x3303, 0x40}, {0x3304, 0x00}, {0x3305, 0x96},
	{0x3306, 0x00}, {0x3307, 0x29}, {0x3308, 0x07}, {0x3309, 0xBA},
	{0x330A, 0x06}, {0x330B, 0xF5}, {0x330C, 0x01}, {0x330D, 0x51},
	{0x330E, 0x01}, {0x330F, 0x30}, {0x3310, 0x07}, {0x3311, 0x16},
	{0x3312, 0x07}, {0x3313, 0xBA},
	/* EExt */
	{0x3326, 0x02}, {0x32F6, 0x0F}, {0x32F9, 0x42}, {0x32FA, 0x24},
	{0x3325, 0x4A}, {0x3330, 0x00}, {0x3331, 0x0A}, {0x3332, 0xFF},
	{0x3338, 0x30}, {0x3339, 0x84}, {0x333A, 0x48}, {0x333F, 0x07},
	/* Auto Function */
	{0x3360, 0x10}, {0x3361, 0x18}, {0x3362, 0x1f}, {0x3363, 0x37},
	{0x3364, 0x80}, {0x3365, 0x80}, {0x3366, 0x68}, {0x3367, 0x60},
	{0x3368, 0x30}, {0x3369, 0x28}, {0x336A, 0x20}, {0x336B, 0x10},
	{0x336C, 0x00}, {0x336D, 0x20}, {0x336E, 0x1C}, {0x336F, 0x18},
	{0x3370, 0x10}, {0x3371, 0x38}, {0x3372, 0x3C}, {0x3373, 0x3F},
	{0x3374, 0x3F}, {0x338A, 0x34}, {0x338B, 0x7F}, {0x338C, 0x10},
	{0x338D, 0x23}, {0x338E, 0x7F}, {0x338F, 0x14}, {0x3375, 0x0A},
	{0x3376, 0x0C}, {0x3377, 0x10}, {0x3378, 0x14}, {0x3012, 0x02},
	{0x3013, 0xD0}, {0x3060, 0x01},

	/* [YUYV_640x480_25_Fps]---MCLK:12M hz PCLK:24M hz */
	{0x32BF, 0x60}, {0x32C0, 0x60}, {0x32C1, 0x60}, {0x32C2, 0x60},
	{0x32C3, 0x00}, {0x32C4, 0x20}, {0x32C5, 0x20}, {0x32C6, 0x20},
	{0x32C7, 0x00}, {0x32C8, 0xB3}, {0x32C9, 0x60}, {0x32CA, 0x80},
	{0x32CB, 0x80}, {0x32CC, 0x80}, {0x32CD, 0x80}, {0x32DB, 0x76},
	{0x32E0, 0x02}, {0x32E1, 0x80}, {0x32E2, 0x01}, {0x32E3, 0xE0},
	{0x32E4, 0x00}, {0x32E5, 0x80}, {0x32E6, 0x00}, {0x32E7, 0x80},
	{0x3200, 0x3E}, {0x3201, 0x0F}, {0x3028, 0x07}, {0x3029, 0x00},
	{0x302A, 0x14}, {0x3022, 0x25}, {0x3023, 0x24}, {0x3002, 0x00},
	{0x3003, 0xA4}, {0x3004, 0x00}, {0x3005, 0x04}, {0x3006, 0x04},
	{0x3007, 0x63}, {0x3008, 0x02}, {0x3009, 0xD3}, {0x300A, 0x05},
	{0x300B, 0x3C}, {0x300C, 0x02}, {0x300D, 0xE1}, {0x300E, 0x03},
	{0x300F, 0xC0}, {0x3010, 0x02}, {0x3011, 0xD0}, {0x32B8, 0x3F},
	{0x32B9, 0x31}, {0x32BB, 0x87}, {0x32BC, 0x38}, {0x32BD, 0x3C},
	{0x32BE, 0x34}, {0x3201, 0x7F}, {0x3021, 0x06}, {0x3060, 0x01}
};

static const struct nt99141_mode_info
nt99141_mode_data[NT99141_NUM_MODES] = {
	{NT99141_MODE_VGA_640_480, SUBSAMPLING,
	 640, 640, 480, 480,
	 nt99141_setting_VGA_640_480,
	 ARRAY_SIZE(nt99141_setting_VGA_640_480)},
};

static int nt99141_write_reg(struct nt99141_dev *sensor, u16 reg, u8 val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x, val=%x\n",
			__func__, reg, val);
		return ret;
	}

	return 0;
}

static int nt99141_read_reg(struct nt99141_dev *sensor, u16 reg, u8 *val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x\n",
			__func__, reg);
		return ret;
	}

	*val = buf[0];
	return 0;
}

static int nt99141_load_regs(struct nt99141_dev *sensor,
			    const struct nt99141_mode_info *mode)
{
	int i;
	int ret = 0;
	const struct reg_value *regs = mode->reg_data;
	u16 reg_addr;
	u8 val;

	for (i = 0; i < mode->reg_data_size; ++i, ++regs) {
		reg_addr = regs->reg_addr;
		val = regs->val;
		ret = nt99141_write_reg(sensor, reg_addr, val);
		if (ret)
			break;
	}
	return 0;
}

static void nt99141_power(struct nt99141_dev *sensor, bool enable)
{
	if (enable) {
		gpiod_set_value_cansleep(sensor->pwdn_gpio, 1);
		udelay(100);
		gpiod_set_value_cansleep(sensor->pwdn_gpio, 0);
		udelay(100);
	}
}

static void nt99141_reset(struct nt99141_dev *sensor)
{
	if (!sensor->reset_gpio)
		return;

	/* camera power cycle */
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	udelay(100);
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	udelay(100);
}

static int nt99141_set_power_on(struct nt99141_dev *sensor)
{
	nt99141_power(sensor, true);
	nt99141_reset(sensor);
	return 0;
}

static void nt99141_set_power_off(struct nt99141_dev *sensor)
{
	//nt99141_power(sensor, false);
	//clk_disable_unprepare(sensor->xclk);
}

static int nt99141_s_std(struct v4l2_subdev *sd, v4l2_std_id norm)
{
	return 0;
}

static int nt99141_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;
	if (code->index >= ARRAY_SIZE(nt99141_formats))
		return -EINVAL;

	code->code = nt99141_formats[code->index].code;
	return 0;
}

static int nt99141_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad != 0)
		return -EINVAL;
	if (fse->index >= NT99141_NUM_MODES)
		return -EINVAL;

	fse->min_width =
		nt99141_mode_data[fse->index].hact;
	fse->max_width = fse->min_width;
	fse->min_height =
		nt99141_mode_data[fse->index].vact;
	fse->max_height = fse->min_height;

	return 0;
}

static int nt99141_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct nt99141_dev *sensor = to_nt99141_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						 format->pad);
	else
		fmt = &sensor->fmt;

	format->format = *fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static int nt99141_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	return 0;
}

static int nt99141_s_power(struct v4l2_subdev *sd, int on)
{
	struct nt99141_dev *sensor = to_nt99141_dev(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);
	if (sensor->power_count == !on) {
		ret = nt99141_set_power_on(sensor);
		if (ret)
			goto out;
		nt99141_load_regs(sensor, sensor->current_mode);
	}

	/* Update the power count. */
	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int nt99141_log_status(struct v4l2_subdev *sd)
{
	return 0;
}


static const struct v4l2_subdev_core_ops nt99141_core_ops = {
	.s_power = nt99141_s_power,
	.log_status = nt99141_log_status,
};

static const struct v4l2_subdev_video_ops nt99141_video_ops = {
	.s_std = nt99141_s_std,
};

static const struct v4l2_subdev_pad_ops nt99141_pad_ops = {
	.enum_mbus_code = nt99141_enum_mbus_code,
	.enum_frame_size = nt99141_enum_frame_size,
	.get_fmt = nt99141_get_fmt,
	.set_fmt = nt99141_set_fmt,
};

static const struct v4l2_subdev_ops nt99141_subdev_ops = {
	.core = &nt99141_core_ops,
	.video = &nt99141_video_ops,
	.pad = &nt99141_pad_ops,
};

static int nt99141_check_chip_id(struct nt99141_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;
	u8 chip_id[2];

	ret = nt99141_set_power_on(sensor);
	if (ret)
		return ret;

	nt99141_read_reg(sensor, 0x3000, &chip_id[0]);
	nt99141_read_reg(sensor, 0x3001, &chip_id[1]);
	dev_info(&client->dev, "chip id %x%x\n", chip_id[0], chip_id[1]);
	if (chip_id[0] != 0x14 && chip_id[1] != 0x10) {
		dev_err(&client->dev, "%s: wrong chip identifier, expected 0x1410, got 0x%x%x\n",
			__func__, chip_id[0], chip_id[1]);
		ret = -ENXIO;
		nt99141_set_power_off(sensor);
		return ret;
	}
	return 0;
}

static int  ccap_get_port_number(struct device *dev)
{
        u32   val32[2];

        if (of_property_read_u32_array(dev->of_node, "port-number", val32, 1) != 0) {
                printk("%s - can not get port-number!\n", __func__);
                return -EINVAL;
        }

        return val32[0];
}

int ccap_clk_init(struct device *dev)
{
	int ret;
	int idx;
	struct clk *clkcap,*clkaplldiv,*clkmux;
	struct clk *clk;
	int i32Div;
	u32 video_freq = 24000000;
	//ENTRY();
	
	idx = ccap_get_port_number(dev);
	of_property_read_u32_array(dev->of_node,"frequency", &video_freq,1);
	
	if (idx == 0) {
		clk = clk_get(NULL, "ccap0_eclk");
		if (IS_ERR(clk)) {
			return -ENOENT;
		}
		clk_prepare(clk);
		clk_enable(clk);
		clk_prepare(clk_get(NULL, "ccap0_hclk"));
		clk_enable(clk_get(NULL, "ccap0_hclk"));
		clk_prepare(clk_get(NULL, "sensor_hclk"));
		clk_enable(clk_get(NULL, "sensor_hclk"));
		clkmux = clk_get(NULL, "ccap0_eclk_mux");
		if (IS_ERR(clkmux)) {
			printk(KERN_ERR "nuc980-ccap0:failed to get clock source\n");
			ret = PTR_ERR(clkmux);
			return ret;
		}
		clkcap = clk_get(NULL, "ccap0_eclk");
		if (IS_ERR(clkcap)) {
			printk(KERN_ERR "nuc980-cap0:failed to get clock source\n");
			ret = PTR_ERR(clkcap);
			return ret;
		}
		clkaplldiv = clk_get(NULL, "ccap0_uplldiv");
		//clkaplldiv = clk_get(NULL, "cap0_eclk_div");
		if (IS_ERR(clkaplldiv)) {
			printk(KERN_ERR "nuc980-ccap0:failed to get clock source\n");
			ret = PTR_ERR(clkaplldiv);
			return ret;
		}
		clk_set_parent(clkmux, clkaplldiv);
		clk_set_rate(clkcap, video_freq);

		i32Div=(300000000/video_freq)-1;
		if(i32Div>0xF) i32Div=0xf;
		__raw_writel((__raw_readl(REG_CLK_DIV3) & ~(0xF<<24) ) | (i32Div<<24),REG_CLK_DIV3);
		printk("ccap0 clock setting %dHz OK\n",video_freq);

	} else {
		u32 video_freq = 24000000;

		clk = clk_get(NULL, "ccap1_eclk");
		if (IS_ERR(clk)) {
			return -ENOENT;
		}
		clk_prepare(clk);
		clk_enable(clk);
		clk_prepare(clk_get(NULL, "ccap1_hclk"));
		clk_enable(clk_get(NULL, "ccap1_hclk"));
		clk_prepare(clk_get(NULL, "sensor_hclk"));
		clk_enable(clk_get(NULL, "sensor_hclk"));
		clkmux = clk_get(NULL, "ccap1_eclk_mux");
		if (IS_ERR(clkmux)) {
			printk(KERN_ERR "nuc980-ccap1:failed to get clock source\n");
			ret = PTR_ERR(clkmux);
			return ret;
		}
		clkcap = clk_get(NULL, "ccap1_eclk");
		if (IS_ERR(clkcap)) {
			printk(KERN_ERR "nuc980-ccap1:failed to get clock source\n");
			ret = PTR_ERR(clkcap);
			return ret;
		}
		clkaplldiv = clk_get(NULL, "ccap1_uplldiv");
		//clkaplldiv = clk_get(NULL, "ccap1_eclk_div");
		if (IS_ERR(clkaplldiv)) {
			printk(KERN_ERR "nuc980-ccap1:failed to get clock source\n");
			ret = PTR_ERR(clkaplldiv);
			return ret;
		}
		clk_set_parent(clkmux, clkaplldiv);
		clk_set_rate(clkcap, video_freq);

		i32Div=(300000000/video_freq)-1;
		if(i32Div>0xF) i32Div=0xf;
		__raw_writel((__raw_readl(REG_CLK_DIV2) & ~(0xF<<24) ) | i32Div<<24,REG_CLK_DIV2);
		printk("ccap1 clock setting %dHz OK\n",video_freq);
	}

	return 0;
}

static int nt99141_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct nt99141_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	/*
	 * default init sequence initialize sensor to
	 * YUV422 UYVY VGA@30fps
	 */
	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = 640;
	fmt->height = 480;
	fmt->field = V4L2_FIELD_NONE;
	sensor->current_mode =
		&nt99141_mode_data[NT99141_MODE_VGA_640_480];
	sensor->last_mode = sensor->current_mode;

	sensor->ae_target = 52;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev),
						  NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}

	ccap_clk_init(dev);

	/* request optional power down pin */
	sensor->pwdn_gpio = devm_gpiod_get_optional(dev, "powerdown",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->pwdn_gpio))
		return PTR_ERR(sensor->pwdn_gpio);

	/* request optional reset pin */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return PTR_ERR(sensor->reset_gpio);

	v4l2_i2c_subdev_init(&sensor->sd, client, &nt99141_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	ret = nt99141_check_chip_id(sensor);
	if (ret)
		goto entity_cleanup;

	ret = v4l2_async_register_subdev_sensor_common(&sensor->sd);
	if (ret)
		goto free_ctrls;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static int nt99141_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nt99141_dev *sensor = to_nt99141_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);
	return 0;
}

static const struct i2c_device_id nt99141_id[] = {
	{"nt99141", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, nt99141_id);

static const struct of_device_id nt99141_dt_ids[] = {
	{.compatible = "novatek, nt99141"},
	{ /* sentinel */ }
};

static struct i2c_driver nt99141_i2c_driver = {
	.driver = {
		   .name = "nt99141",
		   .of_match_table = nt99141_dt_ids,
		   },
	.id_table = nt99141_id,
	.probe = nt99141_probe,
	.remove = nt99141_remove,
};

module_i2c_driver(nt99141_i2c_driver);

MODULE_DESCRIPTION("NT99141 Camera Subdev Driver");
MODULE_LICENSE("GPL");
