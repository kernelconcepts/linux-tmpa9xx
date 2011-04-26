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

#include <mach/platform.h>
#include <mach/regs.h>
#include <mach/adc.h>

#define DRIVER_DESC "TMPA9xx touchscreen driver"

#define TS_CR0_TSI7     (1<<7) /* TSI7  R/W 0y0 pull-down resistor(refer to Explanation) */
#define TS_CR0_INGE     (1<<6) /* INGE  R/W 0y0 Input gate control of Port PD6, PD7 */
#define TS_CR0_PTST     (1<<5) /* PTST  R   0y0 Detection condition */
#define TS_CR0_TWIEN	(1<<4) /* TWIEN R/W 0y0 INTA interrupt control */
#define TS_CR0_PYEN     (1<<3) /* PYEN  R/W 0y0 SPY */
#define TS_CR0_PXEN     (1<<2) /* PXEN  R/W 0y0 SPX */
#define TS_CR0_MYEN     (1<<1) /* MYEN  R/W 0y0 SMY */
#define TS_CR0_MXEN     (1<<0) /* MXEN[0] MXEN  R/W 0y0 SMX */

#define TSI_CR0	(0x1f0) /* control register 0 */
#define TSI_CR1	(0x1f4) /* control register 1 */

#define ts_writel(b, o, v)	writel(v, b->regs + o)
#define ts_readl(b, o)		readl(b->regs + o)

struct tmpa9xx_ts_priv
{
	struct device *dev;
	void __iomem *regs;
	struct input_dev *input_dev;
	int irq;
	struct workqueue_struct *wq;
	struct work_struct wq_irq;

	/* post processing */
	int init;
	int x;
	int y;
	int down;
	int oob;
};

static inline void ts_clear_interrupt(struct tmpa9xx_ts_priv *t)
{
	_out32(PORTD_GPIOIC, _in32(PORTD_GPIOMIS));
}

static inline void ts_enable_interrupt(struct tmpa9xx_ts_priv *t)
{
	_out32(PORTD_GPIOIBE, 0x00);
	_out32(PORTD_GPIOIEV, 0x40);
	_out32(PORTD_GPIOIE, 0x40);
}

static inline void ts_disable_interrupt(struct tmpa9xx_ts_priv *t)
{
	_out32(PORTD_GPIOIE, 0x00);
}

static void measure_init(struct tmpa9xx_ts_priv *t)
{
	t->init = 0;
	t->down = 0;
	t->oob = 0;
}

static void measure_put(struct tmpa9xx_ts_priv *t, int x, int y)
{
	if (t->init == 0) {
		dev_dbg(t->dev, "x %3d, y %3d, drop 1st\n", x, y);
		t->x = x;
		t->y = y;
		t->init++;
		return;
	}

	if (t->init == 1) {
		dev_dbg(t->dev, "x %3d, y %3d, drop 2nd\n", x, y);
		t->x = x;
		t->y = y;
		t->init++;
		return;
	}

	if (t->init == 2) {
		dev_dbg(t->dev, "x %3d, y %3d, store value\n", x, y);
		t->x = x;
		t->y = y;
		t->oob = 0;
		t->init++;
		return;
	}

#define THRESHOLD 50
	if (abs(t->x-x) > THRESHOLD || abs(t->y-y) > THRESHOLD) {
		if (t->oob >= 2) {
			dev_dbg(t->dev, "x %3d, y %3d, too many out of bounds\n", x, y);
			t->init = 2;
			return;
		} else {
			dev_dbg(t->dev, "x %3d, y %3d, out of bounds\n", x, y);
			t->oob++;
			return;
		}
	}
	t->oob = 0;

	if (!t->down) {
		input_report_key(t->input_dev, BTN_TOUCH, 1);
		input_report_abs(t->input_dev, ABS_PRESSURE, 1);
		t->down = 1;
	}
	input_report_abs(t->input_dev, ABS_X, t->x);
	input_report_abs(t->input_dev, ABS_Y, t->y);
	input_sync(t->input_dev);

	dev_dbg(t->dev, "x %3d, y %3d, next\n", x, y);

	t->x = x;
	t->y = y;
}

static void measure_exit(struct tmpa9xx_ts_priv *t)
{
	/* if there were just one or two values measured, report it */
	if (t->init >= 0 && t->init <= 1) {
		dev_dbg(t->dev, "just one/two values, reporting\n");
		input_report_key(t->input_dev, BTN_TOUCH, 1);
		input_report_abs(t->input_dev, ABS_PRESSURE, 1);
		input_report_abs(t->input_dev, ABS_X, t->x);
		input_report_abs(t->input_dev, ABS_Y, t->y);
		input_sync(t->input_dev);
		t->down = 1;
	}

	if (t->down) {
		input_report_key(t->input_dev, BTN_TOUCH, 0);
		input_report_abs(t->input_dev, ABS_PRESSURE, 0);
		input_sync(t->input_dev);
	}
	dev_dbg(t->dev, "finished this turn\n");
}

static void backend_irq_work(struct work_struct *work)
{
	struct tmpa9xx_ts_priv *t = container_of(work, struct tmpa9xx_ts_priv, wq_irq);
	int x;
	int y;
	int ret;

	measure_init(t);

	while(1) {

		ret = ts_readl(t, TSI_CR0) & TS_CR0_PTST;
		if (!ret)
			break;

		ts_writel(t, TSI_CR0, TS_CR0_TSI7 | TS_CR0_INGE | TS_CR0_PXEN | TS_CR0_MXEN);
		udelay(25);
		x = tmpa9xx_adc_read(5, 0);
#if 0
	        GPIOLDATA = 1;
#endif
		ts_writel(t, TSI_CR0, TS_CR0_TSI7 | TS_CR0_INGE | TS_CR0_PYEN | TS_CR0_MYEN);
		udelay(25);
		y = tmpa9xx_adc_read(4, 0);

#if 0
	        GPIOLDATA = 0;
#endif
		measure_put(t, x, y);

		/* make sure to wait approx. 1ms between setting TSI_CR0 
		and checking the touch condition at the beginning of loop */

		ts_writel(t, TSI_CR0, TS_CR0_TSI7 | TS_CR0_PYEN);

		/* this delay does not depend on the panel. it should be small
		enough to give good responsiveness withouth hogging the cpu */

		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(1);
	}

	measure_exit(t);

	ts_writel(t, TSI_CR0, TS_CR0_TSI7 | TS_CR0_PYEN | TS_CR0_TWIEN);
	ts_enable_interrupt(t);

	return;
}

static irqreturn_t topas910_ts_interrupt(int irq, void *dev_id)
{
	struct tmpa9xx_ts_priv *t = dev_id;

	ts_clear_interrupt(t);
  	ts_disable_interrupt(t);

	queue_work(t->wq, &t->wq_irq);

	return IRQ_HANDLED;
}

static int __devinit tmpa9xx_ts_probe(struct platform_device *pdev)
{
	struct tmpa9xx_ts_priv *t;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "platform_get_resource() failed\n");
		ret = -ENXIO;
		return ret;
	}

	/* Now allocate some memory for our private handle */
	t = kzalloc(sizeof(struct tmpa9xx_ts_priv), GFP_KERNEL);
	if (!t) {
		dev_err(&pdev->dev, "kzalloc() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	t->dev = &pdev->dev;

	t->irq = platform_get_irq(pdev, 0);
	if (t->irq == NO_IRQ) {
		dev_err(t->dev, "platform_get_irq() failed\n");
		ret = -ENXIO;
		goto err1;
	}

	t->regs = ioremap(res->start, resource_size(res));
	if (!t->regs) {
		dev_err(t->dev, "ioremap() failed\n");
		ret = -ENOMEM;
		goto err2;
	}

	t->input_dev = input_allocate_device();
	if (!t->input_dev) {
		dev_err(t->dev, "input_allocate_device() failed\n");
		ret = -ENOMEM;
		goto err3;
	}

	t->input_dev->name       = DRIVER_DESC;
	t->input_dev->phys       = (void *)t->regs;
	t->input_dev->id.bustype = BUS_HOST;
	t->input_dev->id.vendor  = 0;
	t->input_dev->id.product = 0;
	t->input_dev->id.version = 0x0100;
	t->input_dev->evbit[0]   = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	t->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	t->wq = create_workqueue("tmpa9xx-ts-wq");
	if (!t->wq) {
		dev_err(t->dev, "create_workqueue() failed\n");
		ret = -ENOMEM;
		goto err4;
	}
	INIT_WORK(&t->wq_irq, backend_irq_work);

	ret = request_irq(t->irq, topas910_ts_interrupt, IRQF_SHARED, "tmpa9xx-ts-irq", t);
	if (ret) {
		dev_err(t->dev, "request_irq() failed\n");
		ret = -ENOMEM;
		goto err5;
	}

	/* TMPA9xx ADC is 10bit */
	input_set_abs_params(t->input_dev, ABS_X, 0, 1024, tmpa9xx_ts_fuzz, 0);
	input_set_abs_params(t->input_dev, ABS_Y, 0, 1024, tmpa9xx_ts_fuzz, 0);
	input_set_abs_params(t->input_dev, ABS_PRESSURE, 0, 1, 0, 0);

	ret = input_register_device(t->input_dev);
	if (ret) {
		dev_err(t->dev, "input_register_device() failed\n");
		ret = -ENOMEM;
		goto err6;
	}

	input_report_abs(t->input_dev, ABS_PRESSURE, 0);
	input_report_key(t->input_dev, BTN_TOUCH, 0);
	input_sync(t->input_dev);

	dev_set_drvdata(t->dev, t);

#if 0
        GPIOLFR1 = 0x0;
        GPIOLFR2 = 0x0;
        GPIOLDATA = 0x0;
        GPIOLDIR = 0x1;
#endif

	/* setup ts controller */
	ts_writel(t, TSI_CR0, TS_CR0_TSI7 | TS_CR0_PYEN | TS_CR0_TWIEN);
	ts_writel(t, TSI_CR1, 0xc5);

	ts_clear_interrupt(t);
	ts_enable_interrupt(t);

	dev_info(t->dev, DRIVER_DESC " ready, fuzz %d\n", tmpa9xx_ts_fuzz);

	return 0;

err6:
	free_irq(t->irq, t);
err5:
	destroy_workqueue(t->wq);
err4:
	input_free_device(t->input_dev);
err3:
	iounmap(t->regs);
err2:
err1:
	kfree(t);
err0:

	return ret;
}

static int __devexit tmpa9xx_ts_remove(struct platform_device *pdev)
{
	struct tmpa9xx_ts_priv *t = dev_get_drvdata(&pdev->dev);

	ts_disable_interrupt(t);

	ts_writel(t, TSI_CR0, 0x00);
	ts_writel(t, TSI_CR1, 0x00);

	flush_workqueue(t->wq);

	destroy_workqueue(t->wq);

	input_unregister_device(t->input_dev);

	free_irq(t->irq, t);

	input_free_device(t->input_dev);

	iounmap(t->regs);

	dev_set_drvdata(t->dev, NULL);

	kfree(t);

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
MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION(DRIVER_DESC);
