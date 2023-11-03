// SPDX-License-Identifier: GPL v2
//
// drivers/media/i2c/ov7725.c
//
// This file contains a driver for the ov7725 sensor
//
// Copyright (C) 2021 Nuvoton Technology Corp.

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
#include <mach/regs-gcr.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/map.h>

#include <mach/regs-gpio.h>
#include <mach/gpio.h>

/* min/typical/max system clock (xclk) frequencies */
#define ov7725_XCLK_MIN  6000000
#define ov7725_XCLK_MAX 48000000

#define ov7725_DEFAULT_SLAVE_ID 0x48

#define  CAP0_PD_PIN NUC980_PB1
#define  CAP0_RST_PIN NUC980_PC7
#define CAP1_PD_PIN NUC980_PC0
#define CAP1_RST_PIN NUC980_PE10

enum ov7725_mode_id {
	ov7725_MODE_VGA_640_480 = 0,
	ov7725_NUM_MODES,
};


enum ov7725_format_mux {
	ov7725_FMT_MUX_YUV422 = 0,
	ov7725_FMT_MUX_RGB,
};

struct ov7725_pixfmt {
	u32 code;
	u32 colorspace;
};

static const struct ov7725_pixfmt ov7725_formats[] = {
	{ MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_SRGB, },
};

struct reg_value {
	u8 reg_addr;
	u8 val;
};

struct ov7725_mode_info {
	enum ov7725_mode_id id;
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
	const struct reg_value *reg_data;
	u32 reg_data_size;
};

struct ov7725_ctrls {
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

struct ov7725_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to ov7725 */
	struct clk *cclk;
	u32 xclk_freq;

	//struct regulator_bulk_data supplies[ov7725_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	bool   upside_down;

	/* lock to protect all members below */
	struct mutex lock;

	int power_count;

	struct v4l2_mbus_framefmt fmt;
	bool pending_fmt_change;

	const struct ov7725_mode_info *current_mode;
	const struct ov7725_mode_info *last_mode;
	//enum ov7725_frame_rate current_fr;
	struct v4l2_fract frame_interval;

	struct ov7725_ctrls ctrls;

	u32 prev_sysclk, prev_hts;
	u32 ae_low, ae_high, ae_target;

	bool pending_mode_change;
	bool streaming;
};

static inline struct ov7725_dev *to_ov7725_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov7725_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov7725_dev,
			     ctrls.handler)->sd;
}

static struct reg_value ov7725_setting_YUV_VGA[] = {
	{0x12, 0x80}, {0x12, 0x00}, {0x3D, 0x03}, {0x17, 0x22}, {0x18, 0xA4},
	{0x19, 0x07}, {0x1A, 0xF0}, {0x32, 0x02}, {0x29, 0xA0}, {0x2C, 0xF0},
	{0x2A, 0x02}, {0x65, 0x20}, {0x11, 0x01}, {0x42, 0x7F}, {0x63, 0xE0},
	{0x64, 0xFF}, {0x66, 0x00}, {0x67, 0x48}, {0x0D, 0x41}, {0x0E, 0x01},
	{0x0F, 0xC5}, {0x14, 0x11}, {0x22, 0x7F}, {0x23, 0x03}, {0x24, 0x40},
	{0x25, 0x30}, {0x26, 0xA1}, {0x2B, 0x00}, {0x6B, 0xAA}, {0x13, 0xEF},
	{0x90, 0x05}, {0x91, 0x01}, {0x92, 0x03}, {0x93, 0x00}, {0x94, 0x90},
	{0x95, 0x8A}, {0x96, 0x06}, {0x97, 0x0B}, {0x98, 0x95}, {0x99, 0xA0},
	{0x9A, 0x1E}, {0x9B, 0x08}, {0x9C, 0x20}, {0x9E, 0x81}, {0xA6, 0x04},
	{0x7E, 0x0C}, {0x7F, 0x24}, {0x80, 0x3A}, {0x81, 0x60}, {0x82, 0x70},
	{0x83, 0x7E}, {0x84, 0x8A}, {0x85, 0x94}, {0x86, 0x9E}, {0x87, 0xA8},
	{0x88, 0xB4}, {0x89, 0xBE}, {0x8A, 0xCA}, {0x8B, 0xD8}, {0x8C, 0xE2},
	{0x8D, 0x28}, {0x46, 0x05}, {0x47, 0x00}, {0x48, 0x00}, {0x49, 0x12},
	{0x4A, 0x00}, {0x4B, 0x13}, {0x4C, 0x21}, {0x0C, 0x10}, {0x09, 0x00},
	{0xFF, 0xFF}, {0xFF, 0xFF}
};

static const struct ov7725_mode_info
ov7725_mode_data[ov7725_NUM_MODES] = {
	{ov7725_MODE_VGA_640_480,
	 640, 640, 480, 480,
	 ov7725_setting_YUV_VGA,
	 ARRAY_SIZE(ov7725_setting_YUV_VGA)},
};

static int ov7725_write_reg(struct ov7725_dev *sensor, u8 reg, u8 val)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;
	
	ret = i2c_smbus_write_byte_data(client, reg, val);

	return ret;
}

static int ov7725_read_reg(struct ov7725_dev *sensor, u8 reg, u8 *val)
{
	struct i2c_client *client = sensor->i2c_client;

	*val = i2c_smbus_read_byte_data(client, reg);

	return 0;
}

static int ov7725_load_regs(struct ov7725_dev *sensor,
			    const struct ov7725_mode_info *mode)
{
	int i;
	int ret = 0;
	const struct reg_value *regs = mode->reg_data;
	u16 reg_addr;
	u8 val;

	for (i = 0; i < mode->reg_data_size; ++i, ++regs) {
		reg_addr = regs->reg_addr;
		val = regs->val;
		ret = ov7725_write_reg(sensor, reg_addr, val);
		if (ret)
			break;
	}
	return 0;
}

static void ov7725_power(struct ov7725_dev *sensor, bool enable)
{
	if (enable) {
		gpiod_set_value_cansleep(sensor->pwdn_gpio, 1);
		udelay(100);
		gpiod_set_value_cansleep(sensor->pwdn_gpio, 0);
		udelay(100);
	}
}

static void ov7725_reset(struct ov7725_dev *sensor)
{
	if (!sensor->reset_gpio)
		return;

	/* camera power cycle */
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	udelay(100);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	udelay(100);
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	udelay(1000);

}

static int ov7725_set_power_on(struct ov7725_dev *sensor)
{
	ov7725_power(sensor, true);
	ov7725_reset(sensor);
	return 0;
}

static void ov7725_set_power_off(struct ov7725_dev *sensor)
{
	//ov7725_power(sensor, false);
	//clk_disable_unprepare(sensor->xclk);
}

static int ov7725_s_std(struct v4l2_subdev *sd, v4l2_std_id norm)
{
	return 0;
}

static int ov7725_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;
	if (code->index >= ARRAY_SIZE(ov7725_formats))
		return -EINVAL;

	code->code = ov7725_formats[code->index].code;
	return 0;
}

static int ov7725_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad != 0)
		return -EINVAL;
	if (fse->index >= ov7725_NUM_MODES)
		return -EINVAL;

	fse->min_width =
		ov7725_mode_data[fse->index].hact;
	fse->max_width = fse->min_width;
	fse->min_height =
		ov7725_mode_data[fse->index].vact;
	fse->max_height = fse->min_height;

	return 0;
}

static int ov7725_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct ov7725_dev *sensor = to_ov7725_dev(sd);
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

static const struct ov7725_mode_info *
ov7725_find_mode(struct ov7725_dev *sensor, int width, int height, bool nearest)
{
	const struct ov7725_mode_info *mode;

	mode = v4l2_find_nearest_size(ov7725_mode_data,
				      ARRAY_SIZE(ov7725_mode_data),
				      hact, vact,
				      width, height);

	if (!mode ||
	    (!nearest && (mode->hact != width || mode->vact != height)))
		return NULL;

	return mode;
}

static int ov7725_try_fmt_internal(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   const struct ov7725_mode_info **new_mode)
{
	struct ov7725_dev *sensor = to_ov7725_dev(sd);
	const struct ov7725_mode_info *mode;
	int i;

	mode = ov7725_find_mode(sensor, fmt->width, fmt->height, true);
	if (!mode)
		return -EINVAL;
	fmt->width = mode->hact;
	fmt->height = mode->vact;

	if (new_mode)
		*new_mode = mode;

	for (i = 0; i < ARRAY_SIZE(ov7725_formats); i++)
		if (ov7725_formats[i].code == fmt->code)
			break;
	if (i >= ARRAY_SIZE(ov7725_formats))
		i = 0;

	fmt->code = ov7725_formats[i].code;
	fmt->colorspace = ov7725_formats[i].colorspace;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);

	return 0;
}

static int ov7725_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{

	struct ov7725_dev *sensor = to_ov7725_dev(sd);
	const struct ov7725_mode_info *new_mode;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	ret = ov7725_try_fmt_internal(sd, mbus_fmt, &new_mode);
	if (ret)
		goto out;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(sd, cfg, 0);
	else
		fmt = &sensor->fmt;

	*fmt = *mbus_fmt;

	if (new_mode != sensor->current_mode) {
		sensor->current_mode = new_mode;
		dev_dbg(&sensor->i2c_client->dev,
			"id %d, width %d, height %d\n",
			sensor->current_mode->id,
			fmt->width,
			fmt->height);
		ov7725_load_regs(sensor, sensor->current_mode);
		sensor->pending_mode_change = true;
	}
	if (mbus_fmt->code != sensor->fmt.code)
		sensor->pending_fmt_change = true;

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int ov7725_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov7725_dev *sensor = to_ov7725_dev(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);
	if (sensor->power_count == !on) {
		ret = ov7725_set_power_on(sensor);
		if (ret)
			goto out;
		ov7725_load_regs(sensor, sensor->current_mode);
	}

	/* Update the power count. */
	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int ov7725_log_status(struct v4l2_subdev *sd)
{
	return 0;
}


static const struct v4l2_subdev_core_ops ov7725_core_ops = {
	.s_power = ov7725_s_power,
	.log_status = ov7725_log_status,
};

static const struct v4l2_subdev_video_ops ov7725_video_ops = {
	.s_std = ov7725_s_std,
};

static const struct v4l2_subdev_pad_ops ov7725_pad_ops = {
	.enum_mbus_code = ov7725_enum_mbus_code,
	.enum_frame_size = ov7725_enum_frame_size,
	.get_fmt = ov7725_get_fmt,
	.set_fmt = ov7725_set_fmt,
};

static const struct v4l2_subdev_ops ov7725_subdev_ops = {
	.core = &ov7725_core_ops,
	.video = &ov7725_video_ops,
	.pad = &ov7725_pad_ops,
};

static int ov7725_check_chip_id(struct ov7725_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;
	u8 chip_id[4];

	ret = ov7725_set_power_on(sensor);
	if (ret)
		return ret;

	ov7725_write_reg(sensor, 0xFF, 0x01);
	ov7725_read_reg(sensor, 0x0A, &chip_id[0]);  /* PID 0x77 */
	ov7725_read_reg(sensor, 0x0B, &chip_id[1]);  /* VER 0x21 */
	ov7725_read_reg(sensor, 0x1C, &chip_id[2]);  /* Manufacturer ID Byte - High  0x7F */
	ov7725_read_reg(sensor, 0x1D, &chip_id[3]);  /* Manufacturer ID Byte - Low   0xA2 */
	dev_info(&client->dev, "chip id = 0x%02x(0x77) VER = 0x%02x(0x21) MIDH = 0x%02x(0x7F) MIDL = 0x%02x(0xA2)\n", 	
		chip_id[0],chip_id[1],chip_id[2],chip_id[3]);
		
	if (chip_id[0] != 0x77 && chip_id[1] != 0x21) {
		dev_err(&client->dev, "%s: wrong chip identifier, expected 0x7721, got 0x%x%x\n",
			__func__, chip_id[0], chip_id[1]);
		ret = -ENXIO;
		ov7725_set_power_off(sensor);
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

static int ccap_clk_init(struct device *dev)
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

static int ov7725_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct ov7725_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	/* let's see whether this adapter can support what we need */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&(client->adapter)->dev,
			"OV7725: I2C-Adapter doesn't support SMBUS\n");
		return -EIO;
	}

	v4l_info(client, "chip found @ 0x%x (%s)\n",
                        client->addr << 1, client->adapter->name);

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	client->flags = I2C_CLIENT_SCCB;
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
		&ov7725_mode_data[ov7725_MODE_VGA_640_480];
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
	if (IS_ERR(sensor->pwdn_gpio)){
		printk("%s get pd pins failed ret %d\n",__func__,ret);
		return PTR_ERR(sensor->pwdn_gpio);
	}

	/* request optional reset pin */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio)){
		printk("%s get rest pins failed  ret %d\n",__func__,ret);
		return PTR_ERR(sensor->reset_gpio);
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &ov7725_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret){
		printk("%s ret %d\n",__func__,ret);
		return ret;
	}

	mutex_init(&sensor->lock);

	ret = ov7725_check_chip_id(sensor);
	if (ret)
		//return ERR_PTR(-EPROBE_DEFER);
		goto entity_cleanup;

	ret = v4l2_async_register_subdev_sensor_common(&sensor->sd);
	if (ret){
		printk("%s ret 2%d\n",__func__,ret);
		goto free_ctrls;
	}

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static int ov7725_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov7725_dev *sensor = to_ov7725_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);

	return 0;
}

static const struct i2c_device_id ov7725_id[] = {
	{"ov7725", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ov7725_id);

static const struct of_device_id ov7725_dt_ids[] = {
	{.compatible = "ovti,ov7725"},
	{ /* sentinel */ }
};

static struct i2c_driver ov7725_i2c_driver = {
	.driver = {
		   .name = "ov7725",
		   .of_match_table = ov7725_dt_ids,
		   },
	.id_table = ov7725_id,
	.probe = ov7725_probe,
	.remove = ov7725_remove,
};

module_i2c_driver(ov7725_i2c_driver);

MODULE_DESCRIPTION("ov7725 Camera Subdev Driver");
MODULE_LICENSE("GPL v2");

