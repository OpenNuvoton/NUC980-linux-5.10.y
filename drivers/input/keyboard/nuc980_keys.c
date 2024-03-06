/* linux/driver/input/nuc980_keys.c
 *
 * Copyright (c) 2017 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 */

#include <linux/init.h>
#include <linux/slab.h>

#include <linux/input.h>
#include <linux/device.h>

#include <asm/errno.h>
#include <asm/delay.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/completion.h>

#include <linux/platform_device.h>

#include <mach/map.h>
#include <mach/mfp.h>

#include <mach/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/map.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/regs-gcr.h>
#include <mach/regs-aic.h>

#include <mach/irqs.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/err.h>
#include <linux/io.h>

#undef BIT
#include <linux/input.h>
#define BIT(x)  (1UL<<((x)%BITS_PER_LONG))
#define ROW_COL_MAX  8

static struct input_dev *nuc980_keys_input_dev;
static struct timer_list kpd_timer;
static char timer_active = 0;

static u64 old_key, new_key;
static u32 open_cnt = 0, key_cnt;
static u32 row_cnt = 1, col_cnt = 1;
static u32 is_keypad_mode = 0;
// Here saves pin info of matrix. gpio_col is reserved in normal button mode
static u32 gpio_row[ROW_COL_MAX], gpio_col[ROW_COL_MAX], key_code[ROW_COL_MAX*ROW_COL_MAX];
spinlock_t scanlock;

static void scan_key(struct timer_list *t)
{
	u32 i, j, group, num;
	u64 index;
	unsigned long flags;

	spin_lock_irqsave(&scanlock, flags);

	if (!timer_active) {
		// disable irq, use timer to track pin status
		for (i = 0; i < row_cnt; i++) {
			disable_irq_nosync(gpio_to_irq(gpio_row[i]));
		}
    }

	new_key = 0;
	// scan key status column by column
	for (j = 0; j < col_cnt; j++) {
		if(is_keypad_mode) {
			for (i = 0; i < col_cnt; i++) {
				group = gpio_col[i] / GPIO_OFFSET;
				num = gpio_col[i] % GPIO_OFFSET;
				if(i == j) {
					writel(readl(REG_GPIOA_DOUT + group*0x40) & ~( 0x1 << num ),
						 		 REG_GPIOA_DOUT + group*0x40); // write low
				} else {
					writel(readl(REG_GPIOA_DOUT + group*0x40) | ( 0x1 << num ),
								 REG_GPIOA_DOUT + group*0x40); // write high
				}
			}
			udelay(100);
		}
		for (i = 0; i < row_cnt; i++) {
			group = gpio_row[i] / GPIO_OFFSET;
			num = gpio_row[i] % GPIO_OFFSET;
			index = !(readl(REG_GPIOA_PIN + group*0x40) & (0x1 << num));
			new_key |= index << (j + col_cnt * i);
		}
	}

	// check key status pin by pin
	for (i = 0; i < key_cnt; i++) {
		if ((new_key ^ old_key) & (0x1 << i)) {
			input_event(nuc980_keys_input_dev, EV_KEY, key_code[i], (new_key & (0x1 << i)) == (0x1 << i));
			input_sync(nuc980_keys_input_dev);
		} else if (new_key & (0x1 << i)) {
			// pressing
			input_event(nuc980_keys_input_dev, EV_KEY, key_code[i], 2);
			input_sync(nuc980_keys_input_dev);
		}
	}

	if (new_key == 0) {
		// if all keys are released, delete timer and resume irq
		del_timer(&kpd_timer);
		for (i = 0; i < row_cnt; i++) {
			enable_irq(gpio_to_irq(gpio_row[i]));
		}
		old_key = 0;
		timer_active = 0;
	} else {
		old_key = new_key;
		timer_active = 1;
		// start timer
		mod_timer(&kpd_timer, jiffies + msecs_to_jiffies(200));
	}

	spin_unlock_irqrestore(&scanlock, flags);
}

static irqreturn_t nuc980_kpd_irq(int irq, void *dev_id) 
{
	u32 group, num;

	// get #GPIO
	group = (irq - IRQ_GPIO_START) / GPIO_OFFSET;
	num = (irq - IRQ_GPIO_START) % GPIO_OFFSET;

	scan_key(0);

	// clear ISR
	writel(readl(REG_GPIOA_INTSRC + group*0x40) & (0x1 << num), REG_GPIOA_INTSRC + group*0x40);

	return IRQ_HANDLED;
}

static irqreturn_t kpi_interrupt_handle__(int irq, void *dev_id)
{
	u32 group, num;

	// get #GPIO
	group = (irq - IRQ_GPIO_START) / GPIO_OFFSET;
	num = (irq - IRQ_GPIO_START) % GPIO_OFFSET;

	// clear ISR
	writel(readl(REG_GPIOA_INTSRC + group*0x40) & (0x1 << num), REG_GPIOA_INTSRC + group*0x40);
    
	return IRQ_HANDLED;
}


int nuc980_kpd_open(struct input_dev *input)
{
	u32 i;
	int error = 0;

	if (open_cnt > 0) {
		goto exit;
	}

	new_key = old_key = 0;

	// init timer
	timer_setup(&kpd_timer, scan_key, 0);

	spin_lock_init(&scanlock);

	// Resgister irq for each row
	for (i = 0; i < row_cnt; i++) {
		// clear isr status
		writel(0x1 << (gpio_row[i] % GPIO_OFFSET), REG_GPIOA_INTSRC + (gpio_row[i] / GPIO_OFFSET)*0x40);
		//gpio_irq_table[i] = IRQ_GPIO_START + gpio_row[i];
		error =  request_irq(IRQ_GPIO_START + gpio_row[i], nuc980_kpd_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND, "Keys", NULL);
		if (error) {
			dev_err(input->dev.parent, "Failed to register GPIO#%d IRQ.\n", gpio_row[i]);
			return -EAGAIN;
		}
	}

	if(is_keypad_mode) {
		// Register irq for gpio wake
		for (i = 0; i < col_cnt; i++) {
			// clear isr status
			writel(0x1 << (gpio_col[i] % GPIO_OFFSET), REG_GPIOA_INTSRC + (gpio_col[i] / GPIO_OFFSET)*0x40);
			//gpio_irq_table[i] = IRQ_GPIO_START + gpio_col[i];
			error =  request_irq(IRQ_GPIO_START + gpio_col[i], kpi_interrupt_handle__, IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, "Keys",NULL);
			if (error) {
				dev_err(input->dev.parent, "Failed to register GPIO#%d IRQ.\n", gpio_col[i]);
				return -EAGAIN;
			}

			enable_irq_wake(IRQ_GPIO_START + gpio_col[i]);
			disable_irq_nosync(IRQ_GPIO_START + gpio_col[i]);
		}
	}

exit:
	open_cnt++;
	return 0;
}



void nuc980_kpd_close(struct input_dev *dev)
{
	u32 i;

	open_cnt--;
	if (open_cnt == 0) {
		//disable interrupt
		for (i = 0; i < row_cnt; i++) {
			free_irq(gpio_to_irq(gpio_row[i]), NULL);
		}
	}
	return;
}


static int nuc980_keys_probe(struct platform_device *pdev)
{
	int i, err, group, num;
	u32	gpio_matrix[2] = {0};
	struct clk *clk;

	clk = clk_get(NULL, "gpio_hclk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Failed to get gpio clock source.\n");
		err = PTR_ERR(clk);
		return err;
	}
	clk_prepare(clk);
	clk_enable(clk);

	// Get properties from dts
	if (pdev->dev.of_node) {
		of_property_read_u32_array(pdev->dev.of_node, "key-matrix", gpio_matrix, 2);
		if(gpio_matrix[0] > 0 && gpio_matrix[1] > 0) {
			row_cnt = gpio_matrix[0];
			col_cnt = gpio_matrix[1];
			key_cnt = row_cnt * col_cnt;
			if(col_cnt > 1) {
				// keypad mode
				if (of_property_read_u32_array(pdev->dev.of_node, "gpio-keys-cols", gpio_col, col_cnt) != 0) {
					dev_err(&pdev->dev, "Failed to get properties of gpio-keys-cols.\n");
					return -EINVAL;	
				}
				is_keypad_mode = 1;
			}
			if (of_property_read_u32_array(pdev->dev.of_node, "gpio-keys-rows", gpio_row, row_cnt) != 0) {
				dev_err(&pdev->dev, "Failed to get properties of gpio-keys-rows.\n");
				return -EINVAL;
			}
			if (of_property_read_u32_array(pdev->dev.of_node, "key-code", key_code, key_cnt) != 0) {
				dev_err(&pdev->dev, "Failed to get properties of key-code.\n");
				return -EINVAL;
			}

		} else {
			dev_err(&pdev->dev, "Failed to parse key-matrix.\n");
			goto fail;
		}
	}

	if((row_cnt > ROW_COL_MAX) || (col_cnt > ROW_COL_MAX))
		goto fail;

	// Parse & init GPIO
	for (i = 0; i < row_cnt; i++) {
		// row - input
		group = gpio_row[i] / GPIO_OFFSET;
		num = gpio_row[i] % GPIO_OFFSET;
		writel(readl(REG_GPIOA_MODE + group*0x40) & ~( 0x3 << num*2 ),
					 REG_GPIOA_MODE + group*0x40); // input
		writel((readl(REG_GPIOA_PUSEL + group*0x40) & ~( 0x3 << num*2 )) | ( 0x1 << num*2 ),
					 REG_GPIOA_PUSEL + group*0x40); // pull-up
		writel(readl(REG_GPIOA_DBEN + group*0x40) | ( 0x1 << num ),
					 REG_GPIOA_DBEN + group*0x40); // debounce enable
	}

	if (is_keypad_mode) {
		for (i = 0; i < col_cnt; i++) {
			// col - output
			group = gpio_col[i] / GPIO_OFFSET;
			num = gpio_col[i] % GPIO_OFFSET;
			writel((readl(REG_GPIOA_MODE + group*0x40) & ~( 0x3 << num*2 )) | ( 0x1 << num*2 ),
						 REG_GPIOA_MODE + group*0x40); // push-pull
			writel(readl(REG_GPIOA_DOUT + group*0x40) & ~( 0x1 << num ),
						 REG_GPIOA_DOUT + group*0x40); // write low
			writel(readl(REG_GPIOA_DBEN + group*0x40) | ( 0x1 << num ),
					 	 REG_GPIOA_DBEN + group*0x40); // debounce enable
		}
	}

	writel(0x2f, REG_GPIO_DBNCECON); // De-bounce sampling cycle select 32768 clock

	if (!(nuc980_keys_input_dev = input_allocate_device())) {
		dev_err(&pdev->dev, "Failed to allocate memory.\n");
		err = -ENOMEM;
		goto fail;
	}

	nuc980_keys_input_dev->name = "nuc980-keys";
	nuc980_keys_input_dev->phys = "input/event1";
	nuc980_keys_input_dev->id.bustype = BUS_HOST;
	nuc980_keys_input_dev->id.vendor  = 0x0005;
	nuc980_keys_input_dev->id.product = 0x0001;
	nuc980_keys_input_dev->id.version = 0x0100;

	nuc980_keys_input_dev->open    = nuc980_kpd_open;
	nuc980_keys_input_dev->close   = nuc980_kpd_close;

	nuc980_keys_input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_SYN) |  BIT(EV_REP);

	for (i = 0; i < key_cnt; i++)
		set_bit(key_code[i], nuc980_keys_input_dev->keybit);

	err = input_register_device(nuc980_keys_input_dev);
	if (err) {
		input_free_device(nuc980_keys_input_dev);
		return err;
	}

	// must set after input device register!!!
	nuc980_keys_input_dev->rep[REP_DELAY] = 250;
	nuc980_keys_input_dev->rep[REP_PERIOD] = 33;

	return 0;

fail:
	input_free_device(nuc980_keys_input_dev);
	return err;
}

static int nuc980_keys_remove(struct platform_device *pdev)
{
	u32 i;
	int err;
	struct clk *clk;

	for (i = 0; i < row_cnt; i++) {
		disable_irq(gpio_to_irq(gpio_row[i]));
	}

	clk = clk_get(NULL, "gpio");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Failed to get gpio clock source.\n");
		err = PTR_ERR(clk);
		return err;
	}

	clk_disable(clk);

	platform_set_drvdata(pdev, NULL);
	input_free_device(nuc980_keys_input_dev);

	return 0;
}

#define nuc980_keys_suspend   NULL
#define nuc980_keys_resume	  NULL

static const struct of_device_id nuc980_kpi_of_match[] = {
	{ .compatible = "nuvoton,nuc980-keys" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_kpi_of_match);

static struct platform_driver nuc980_keys_driver = {
	.probe		= nuc980_keys_probe,
	.remove		= nuc980_keys_remove,
	.suspend	= nuc980_keys_suspend,
	.resume		= nuc980_keys_resume,
	.driver		= {
		.name	= "nuc980-keys",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(nuc980_kpi_of_match),
	},
};
module_platform_driver(nuc980_keys_driver);

MODULE_AUTHOR("nuvoton");
MODULE_DESCRIPTION("nuc980 keys driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc980-keys");
