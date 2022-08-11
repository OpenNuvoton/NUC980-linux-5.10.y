// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * NUC980 normal ADC driver
 *
 * Copyright (c) 2022 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

/* nuc980 adc registers offset */
#define CTL     0x00
#define CONF    0x04
#define IER     0x08
#define ISR     0x0C
#define DATA    0x28

#define ADC_TIMEOUT			msecs_to_jiffies(1000)
#define ADC_MAX_CHANNELS		8

struct nuc980_adc_data {
	const struct iio_chan_spec	*channels;
	int				num_channels;
	unsigned long			clk_rate;
};

struct nuc980_adc {
	void __iomem		*regs;
	struct clk		*pclk;
	struct clk		*clk;
	struct completion	completion;
	struct regulator	*vref;
	struct reset_control	*reset;
	const struct nuc980_adc_data *data;
	u16			last_val;
	const struct iio_chan_spec *last_chan;
};

static void nuc980_adc_power_down(struct nuc980_adc *info)
{
	/* Clear irq & power down adc */
}

static int nuc980_adc_conversion(struct nuc980_adc *info,
				   struct iio_chan_spec const *chan)
{
	reinit_completion(&info->completion);

	info->last_chan = chan;

	// enable channel
	writel((readl(info->regs + CONF) & ~(0xf << 12)) | (chan->channel << 12), info->regs + CONF);

	// enable MST
	writel(readl(info->regs + CTL) | 0x100, info->regs + CTL);

	if (!wait_for_completion_timeout(&info->completion, ADC_TIMEOUT))
		return -ETIMEDOUT;

	return 0;
}

static int nuc980_adc_read_raw(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    int *val, int *val2, long mask)
{
	struct nuc980_adc *info = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		ret = nuc980_adc_conversion(info, chan);
		if (ret) {
			nuc980_adc_power_down(info);
			mutex_unlock(&indio_dev->mlock);
			return ret;
		}

		*val = info->last_val;
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(info->vref);
		if (ret < 0) {
			dev_err(&indio_dev->dev, "failed to get voltage\n");
			return ret;
		}

		*val = ret / 1000;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static irqreturn_t nuc980_adc_isr(int irq, void *dev_id)
{
	struct nuc980_adc *info = dev_id;

	if (readl(info->regs+ISR) & 1) {  //check M_F bit
		writel(0x401, info->regs + ISR); //clear flag
	}

	/* Read value */
	info->last_val = readl(info->regs + DATA);
	info->last_val &= GENMASK(info->last_chan->scan_type.realbits - 1, 0);

	nuc980_adc_power_down(info);

	complete(&info->completion);

	return IRQ_HANDLED;
}

static const struct iio_info nuc980_adc_iio_info = {
	.read_raw = nuc980_adc_read_raw,
};

#define SARADC_CHANNEL(_index, _id) {   			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = _index,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.datasheet_name = _id,					\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 12, 				\
		.storagebits = 16,				\
		.endianness = IIO_CPU,				\
	},							\
}

static const struct iio_chan_spec nuc980_adc_iio_channels[] = {
	SARADC_CHANNEL(0, "adc0"),
	SARADC_CHANNEL(1, "adc1"),
	SARADC_CHANNEL(2, "adc2"),
	SARADC_CHANNEL(3, "adc3"),
	SARADC_CHANNEL(4, "adc4"),
	SARADC_CHANNEL(5, "adc5"),
	SARADC_CHANNEL(6, "adc6"),
	SARADC_CHANNEL(7, "adc7"),
};

static const struct nuc980_adc_data nuc980_adc_data = {
	.channels = nuc980_adc_iio_channels,
	.num_channels = ARRAY_SIZE(nuc980_adc_iio_channels),
	.clk_rate = 4000000,
};

static const struct of_device_id nuc980_adc_match[] = {
	{
		.compatible = "nuvoton,nuc980-nadc",
		.data = &nuc980_adc_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_adc_match);

/*
 * Reset SARADC Controller.
 */
static void nuc980_adc_reset_controller(struct reset_control *reset)
{
	reset_control_assert(reset);
	usleep_range(10, 20);
	reset_control_deassert(reset);
}

static void nuc980_adc_clk_disable(void *data)
{
	struct nuc980_adc *info = data;

	clk_disable_unprepare(info->clk);
}

static void nuc980_adc_pclk_disable(void *data)
{
	struct nuc980_adc *info = data;

	clk_disable_unprepare(info->pclk);
}

#if 0
static void nuc980_adc_regulator_disable(void *data)
{
	struct nuc980_adc *info = data;

	regulator_disable(info->vref);
}
#endif

static irqreturn_t nuc980_adc_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *i_dev = pf->indio_dev;
	struct nuc980_adc *info = iio_priv(i_dev);
	/*
	 * @values: each channel takes an u16 value
	 * @timestamp: will be 8-byte aligned automatically
	 */
	struct {
		u16 values[ADC_MAX_CHANNELS];
		int64_t timestamp;
	} data;
	int ret;
	int i, j = 0;

	mutex_lock(&i_dev->mlock);

	for_each_set_bit(i, i_dev->active_scan_mask, i_dev->masklength) {
		const struct iio_chan_spec *chan = &i_dev->channels[i];

		ret = nuc980_adc_conversion(info, chan);
		if (ret) {
			nuc980_adc_power_down(info);
			goto out;
		}

		data.values[j] = info->last_val;
		j++;
	}

	iio_push_to_buffers_with_timestamp(i_dev, &data, iio_get_time_ns(i_dev));
out:
	mutex_unlock(&i_dev->mlock);

	iio_trigger_notify_done(i_dev->trig);

	return IRQ_HANDLED;
}

static int nuc980_adc_probe(struct platform_device *pdev)
{
	struct nuc980_adc *info = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct iio_dev *indio_dev = NULL;
	struct resource	*mem;
	const struct of_device_id *match;
	int ret;
	int irq;

	if (!np)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev) {
		dev_err(&pdev->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}
	info = iio_priv(indio_dev);

	match = of_match_device(nuc980_adc_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "failed to match device\n");
		return -ENODEV;
	}

	info->data = match->data;

	/* Sanity check for possible later IP variants with more channels */
	if (info->data->num_channels > ADC_MAX_CHANNELS) {
		dev_err(&pdev->dev, "max channels exceeded");
		return -EINVAL;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	info->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(info->regs))
		return PTR_ERR(info->regs);

#if 0
	/*
	 * The reset should be an optional property, as it should work
	 * with old devicetrees as well
	 */
	info->reset = devm_reset_control_get_exclusive(&pdev->dev,
						       "saradc-apb");
	if (IS_ERR(info->reset)) {
		ret = PTR_ERR(info->reset);
		if (ret != -ENOENT)
			return ret;

		dev_dbg(&pdev->dev, "no reset control found\n");
		info->reset = NULL;
	}
#endif

	init_completion(&info->completion);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(&pdev->dev, irq, nuc980_adc_isr,
			       0, dev_name(&pdev->dev), info);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed requesting irq %d\n", irq);
		return ret;
	}

	info->pclk = devm_clk_get(&pdev->dev, "adc_eclk");
	if (IS_ERR(info->pclk)) {
		dev_err(&pdev->dev, "failed to get pclk\n");
		return PTR_ERR(info->pclk);
	}

	info->clk = devm_clk_get(&pdev->dev, "adc");
	if (IS_ERR(info->clk)) {
		dev_err(&pdev->dev, "failed to get adc clock\n");
		return PTR_ERR(info->clk);
	}

	info->vref = devm_regulator_get(&pdev->dev, "vref");
	if (IS_ERR(info->vref)) {
		dev_err(&pdev->dev, "failed to get regulator, %ld\n",
			PTR_ERR(info->vref));
		return PTR_ERR(info->vref);
	}

	if (info->reset)
		nuc980_adc_reset_controller(info->reset);

	/*
	 * Use a default value for the converter clock.
	 * This may become user-configurable in the future.
	 */
	//ret = clk_set_rate(info->clk, info->data->clk_rate);
	ret = clk_set_rate(info->pclk, info->data->clk_rate);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to set adc clk rate, %d\n", ret);
		return ret;
	}

#if 0
	ret = regulator_enable(info->vref);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable vref regulator\n");
		return ret;
	}
	ret = devm_add_action_or_reset(&pdev->dev,
				       nuc980_adc_regulator_disable, info);
	if (ret) {
		dev_err(&pdev->dev, "failed to register devm action, %d\n",
			ret);
		return ret;
	}
#endif

	ret = clk_prepare_enable(info->pclk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable pclk\n");
		return ret;
	}
	ret = devm_add_action_or_reset(&pdev->dev,
				       nuc980_adc_pclk_disable, info);
	if (ret) {
		dev_err(&pdev->dev, "failed to register devm action, %d\n",
			ret);
		return ret;
	}

	ret = clk_prepare_enable(info->clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable converter clock\n");
		return ret;
	}
	ret = devm_add_action_or_reset(&pdev->dev,
				       nuc980_adc_clk_disable, info);
	if (ret) {
		dev_err(&pdev->dev, "failed to register devm action, %d\n",
			ret);
		return ret;
	}

#ifdef CONFIG_NUC980ADC_VREF
	writel(1, info->regs + CTL); //enable AD_EN
#endif
#ifdef CONFIG_NUC980ADC_I33V
	writel(0x3<<6, info->regs + CONF); //select AGND33 vs AVDD33
	writel(1, info->regs + CTL); //enable AD_EN
#endif
#ifdef CONFIG_NUC980ADC_BANDGAP
	writel(readl(info->regs + CTL) | 2, info->regs + CTL); //enable bandgap
#endif

	writel(1, info->regs + IER); //enable M_IEN

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->info = &nuc980_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->channels = info->data->channels;
	indio_dev->num_channels = info->data->num_channels;
	ret = devm_iio_triggered_buffer_setup(&indio_dev->dev, indio_dev, NULL,
					      nuc980_adc_trigger_handler,
					      NULL);
	if (ret)
		return ret;

	writel((readl(info->regs + CONF) | 1<<2), info->regs + CONF); //enable NACEN

	printk("%s: nuc980 Normal ADC adapter\n",
		indio_dev->name);

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

#ifdef CONFIG_PM_SLEEP
static int nuc980_adc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nuc980_adc *info = iio_priv(indio_dev);

	clk_disable_unprepare(info->clk);
	clk_disable_unprepare(info->pclk);
	regulator_disable(info->vref);

	return 0;
}

static int nuc980_adc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct nuc980_adc *info = iio_priv(indio_dev);
	int ret;

	ret = regulator_enable(info->vref);
	if (ret)
		return ret;

	ret = clk_prepare_enable(info->pclk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(info->clk);
	if (ret)
		clk_disable_unprepare(info->pclk);

	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(nuc980_adc_pm_ops,
			 nuc980_adc_suspend, nuc980_adc_resume);

static struct platform_driver nuc980_adc_driver = {
	.probe		= nuc980_adc_probe,
	.driver		= {
		.name	= "nuc980-nadc",
		.of_match_table = nuc980_adc_match,
		.pm	= &nuc980_adc_pm_ops,
	},
};

module_platform_driver(nuc980_adc_driver);

MODULE_DESCRIPTION("NUC980 ADC driver");
MODULE_LICENSE("GPL v2");
