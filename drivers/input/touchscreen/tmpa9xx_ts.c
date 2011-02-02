/*
 * TMPA9xx touchscreen driver
 *
 * Copyright (c) 2008 bplan GmbH
 *           (c) 2010 Florian Boor <florian@kernelconconcepts.de>
 *           (c) 2011 Michael Hunold <michael@mihu.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <mach/regs.h>
#include <mach/ts.h>
#include <mach/adc.h>

#define DRIVER_DESC "TMPA9xx touchscreen driver"

static int threshold_x = 25;
module_param(threshold_x, int, S_IRUGO);

static int threshold_y = 25;
module_param(threshold_y, int, S_IRUGO);

struct tmpa9xx_ts_priv
{
	volatile struct tmpa9xx_ts *ts;
	struct input_dev *input_dev;
	int pen_is_down;
	int ts_irq;
	int delay;

	struct workqueue_struct *wq;
	struct work_struct wq_irq;
};

static inline void ts_init(struct tmpa9xx_ts_priv *t)
{
	t->ts->tsicr0 = TMPA9XX_TS_CR0_TSI7 | TMPA9XX_TS_CR0_PYEN;
}

static inline void ts_clear_interrupt(void)
{
	_out32(PORTD_GPIOIC, _in32(PORTD_GPIOMIS));
}

static inline void ts_enable_interrupt(struct tmpa9xx_ts_priv *t)
{
	t->ts->tsicr0 |= TMPA9XX_TS_CR0_TWIEN;
}

static inline void ts_disable_interrupt(struct tmpa9xx_ts_priv *t)
{
	t->ts->tsicr0 &= ~TMPA9XX_TS_CR0_TWIEN;
}

static inline void enable_interrupt(struct tmpa9xx_ts_priv *t)
{
	_out32(PORTD_GPIOIE, 0xc0);
	ts_enable_interrupt(t);
}

static inline void disable_interrupt(struct tmpa9xx_ts_priv *t)
{
	_out32(PORTD_GPIOIE, 0x00);
	ts_disable_interrupt(t);
}

static int ts_update_pendown(struct tmpa9xx_ts_priv *t)
{
	int pen_is_down;
	
	/* ts_init() must have been called before */

	pen_is_down = t->ts->tsicr0 & TMPA9XX_TS_CR0_PTST ? 1 : 0;

	if (t->pen_is_down == pen_is_down)
		return t->pen_is_down;

	t->pen_is_down = pen_is_down;

	input_report_key(t->input_dev, BTN_TOUCH, pen_is_down);
	input_report_abs(t->input_dev, ABS_PRESSURE, pen_is_down);
	input_sync(t->input_dev);

	return t->pen_is_down;
}

static void ts_update_pos(struct tmpa9xx_ts_priv *t)
{
	int x;
	int y;

	t->ts->tsicr0 = 0xc5;
	x = tmpa9xx_adc_read(5, 1);

	t->ts->tsicr0 = 0xca;
	y = tmpa9xx_adc_read(4, 1);

	if (x < threshold_x || y < threshold_y)
		return;

	input_report_abs(t->input_dev, ABS_X, x);
	input_report_abs(t->input_dev, ABS_Y, y);
	input_sync(t->input_dev);
}

static void backend_irq_work(struct work_struct *work)
{
	struct tmpa9xx_ts_priv *t = container_of(work, struct tmpa9xx_ts_priv, wq_irq);
	int pen_is_down;

	while(1) {
		pen_is_down = ts_update_pendown(t);

		if (!pen_is_down) {
			ts_enable_interrupt(t);
			break;
		}

		ts_update_pos(t);

		ts_init(t);

		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(t->delay));
	}
}

static irqreturn_t topas910_ts_interrupt(int irq, void *dev_id)
{
	struct tmpa9xx_ts_priv *t = dev_id;

	ts_clear_interrupt();
	ts_disable_interrupt(t);

	queue_work(t->wq, &t->wq_irq);

	return IRQ_HANDLED;
}

static int __devinit tmpa9xx_ts_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_ts_platforminfo *tmpa9xx_ts_platforminfo;
	struct tmpa9xx_ts_priv *t;
	struct resource *ts_r;
	int rate;
	int fuzz;
	int ret;

	tmpa9xx_ts_platforminfo = (struct tmpa9xx_ts_platforminfo *)dev->platform_data;
	if (tmpa9xx_ts_platforminfo) {
		fuzz       = tmpa9xx_ts_platforminfo->fuzz;
		rate       = tmpa9xx_ts_platforminfo->rate;
	}
	else {
		fuzz       = TMPA9XX_TS_DEFAULT_FUZZ;
		rate       = TMPA9XX_TS_DEFAULT_RATE;
	}

	ts_r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!ts_r) {
		dev_err(dev, "platform_get_resource() failed\n");
		ret = -ENXIO;
		return ret;
	}

	/* Now allocate some memory for our private handle */
	t = kzalloc(sizeof(struct tmpa9xx_ts_priv), GFP_KERNEL);
	if (!t) {
		dev_err(dev, "kzalloc() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	t->ts_irq = platform_get_irq(pdev, 0);
	if (t->ts_irq == NO_IRQ) {
		dev_err(dev, "platform_get_irq() failed\n");
		ret = -ENXIO;
		goto err1;
	}

	if (rate)
		t->delay = 1000 / rate;
	else
		t->delay = 10;

	t->ts = ioremap(ts_r->start, resource_size(ts_r));
	if (!t->ts) {
		dev_err(dev, "ioremap() failed\n");
		ret = -ENOMEM;
		goto err2;
	}

	t->input_dev = input_allocate_device();
	if (!t->input_dev) {
		dev_err(dev, "input_allocate_device() failed\n");
		ret = -ENOMEM;
		goto err3;
	}

	t->input_dev->name       = DRIVER_DESC;
	t->input_dev->phys       = (void *) t->ts;
	t->input_dev->id.bustype = BUS_HOST;
	t->input_dev->id.vendor  = 0;
	t->input_dev->id.product = 0;
	t->input_dev->id.version = 0x0100;
	t->input_dev->evbit[0]   = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	t->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	t->wq = create_workqueue("tmpa9xx-ts-wq");
	if (!t->wq) {
		dev_err(dev, "create_workqueue() failed\n");
		ret = -ENOMEM;
		goto err4;
	}
	INIT_WORK(&t->wq_irq, backend_irq_work);

	ret = request_irq(t->ts_irq, topas910_ts_interrupt, IRQF_SHARED, "tmpa9xx-ts-irq", t);
	if (ret) {
		dev_err(dev, "request_irq() failed\n");
		ret = -ENOMEM;
		goto err5;
	}

	/* TMPA9xx ADC is 10bit */
	input_set_abs_params(t->input_dev, ABS_X, 0, 1024, fuzz, 0);
	input_set_abs_params(t->input_dev, ABS_Y, 0, 1024, fuzz, 0);
	input_set_abs_params(t->input_dev, ABS_PRESSURE, 0, 1, 0, 0);

	ret = input_register_device(t->input_dev);
	if (ret) {
		dev_err(dev, "input_register_device() failed\n");
		ret = -ENOMEM;
		goto err6;
	}

	input_report_abs(t->input_dev, ABS_PRESSURE, 0);
	input_report_key(t->input_dev, BTN_TOUCH, 0);
	input_sync(t->input_dev);

	dev_set_drvdata(dev, t);

	/* setup ts controller */
	t->ts->tsicr1 = 0xc5;
	ts_init(t);

	enable_interrupt(t);

	dev_info(dev, DRIVER_DESC " (fuzz=%d, rate=%d Hz) ready\n", fuzz, rate);

	return 0;

err6:
	free_irq(t->ts_irq, t);
err5:
	destroy_workqueue(t->wq);
err4:
	input_free_device(t->input_dev);
err3:
	iounmap(t->ts);
err2:
err1:
	kfree(t);
err0:

	return ret;
}

static int __devexit tmpa9xx_ts_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_ts_priv *t = dev_get_drvdata(dev);

	disable_interrupt(t);

	t->ts->tsicr0 = 0;
	t->ts->tsicr1 = 0;

	flush_workqueue(t->wq);

	destroy_workqueue(t->wq);

	input_unregister_device(t->input_dev);

	free_irq(t->ts_irq, t);

	input_free_device(t->input_dev);

	iounmap(t->ts);

	kfree(t);

	dev_set_drvdata(dev, NULL);

	return 0;
}

static struct platform_driver tmpa9xx_ts_driver = {
	.probe		= tmpa9xx_ts_probe,
	.remove		= __devexit_p(tmpa9xx_ts_remove),

	.driver		= {
		.name	= "tmpa9xx-ts",
		.owner	= THIS_MODULE,
	},
};

static int __init tmpa9xx_ts_init(void)
{
	return platform_driver_register(&tmpa9xx_ts_driver);
}

static void __exit tmpa9xx_ts_exit(void)
{
	platform_driver_unregister(&tmpa9xx_ts_driver);
}

module_init(tmpa9xx_ts_init);
module_exit(tmpa9xx_ts_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florian Boor <florian@kernelconcepts.de>");
MODULE_DESCRIPTION(DRIVER_DESC);

