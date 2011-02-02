/*
 * ADC functions for TMPA9xx
 * used by the touchscreen and IIO adc driver
 *
 * Copyright (c) 2011 Michael Hunold (michael@mihu.de)
 *
 * Based on code taken from tmpa9xx_adc.c and tmpa9xx_ts.c
 *   Copyright (c) 2010 Thomas Haase (Thomas.Haase@web.de)
 *   Copyright (c) 2010 Florian Boor <florian@kernelconcepts.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <mach/regs.h>
#include <mach/adc.h>

#define DRIVER_NAME "tmpa9xx-adc"

struct tmpa9xx_adc_core
{
	struct device *dev;
	volatile struct tmpa9xx_adc *adc;
	int irq;
	struct mutex lock;
	struct completion complete;
	int value;
};

struct tmpa9xx_adc_core *g_h;

/* fixme: this function does not check who is trying to acquire adc 
values. this should probably be handled through some resource handling
mechanism, which is currently missing. */
int tmpa9xx_adc_read(int num, int delay)
{
	struct tmpa9xx_adc_core *t = g_h;
	int ret;

	/* if the global adc core variable is NULL, this
	means that the module was not correctly loaded */
	BUG_ON(!g_h);

	mutex_lock(&t->lock);

	init_completion(&t->complete);
	ADC_ADMOD1 = (ADC_ADMOD1 & ~0x0f) | num;

	if (delay) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout (msecs_to_jiffies(delay));
	}

 	ADC_ADMOD0 |= (1 << 0);
	wait_for_completion(&t->complete);
	ret = t->value;

	mutex_unlock(&t->lock);

	return ret;
}

irqreturn_t interrupt_handler(int irq, void *dev)
{
	struct tmpa9xx_adc_core *t = dev;
	int num;

	/* clear interrupt */
	ADC_ADIC = (1 << 0);

	num = ADC_ADMOD1 & 0x0f;

	t->value = ((ADC_ADREGxH(num) & 0xff) << 2) | ((ADC_ADREGxL(num) >> 6) & 0x3);
	complete(&t->complete);

	return IRQ_HANDLED;
}

static int tmpa9xx_adc_probe(struct platform_device *pdev)
{
	struct tmpa9xx_adc_core *t;
	struct resource	*mem;
	int ret;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t) {
		return -ENOMEM;
	}

	t->dev = &pdev->dev;

	mutex_init(&t->lock);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(t->dev, "platform_get_resource() @ IORESOURCE_MEM failed\n");
		goto err0;
	}

	t->adc = ioremap(mem->start, resource_size(mem));
	if(!t->adc) {
		dev_err(t->dev, "ioremap() failed\n");
		goto err1;
	}

	/* reset */
	ADC_ADMOD4 = 0x2;
	udelay(10);
	ADC_ADMOD4 = 0x1;

	/* disable adc interrupts */
	ADC_ADIE = 0;

	t->irq = platform_get_irq(pdev, 0);
	if (t->irq <= 0) {
		dev_err(t->dev, "platform_get_irq() failed\n");
		goto err2;
	}

	ret = request_irq(t->irq, interrupt_handler, 0, DRIVER_NAME, t);
	if (ret) {
		dev_err(t->dev, "request_irq() failed\n");
		goto err3;
	}

	platform_set_drvdata(pdev, t);

	ADC_ADMOD1 |= 0x80;

        /* Setup ADCCLK */
        ADC_ADCLK = (0x01 << 7) | 0x02;

	/* Enable IRQ generation */
	ADC_ADIE = (0x01 << 0);

	platform_set_drvdata(pdev, t);

	dev_info(t->dev, "tmpa9xx adc driver registered\n");

	g_h = t;

	return 0;

err3:
err2:
	iounmap(t->adc);
err1:
err0:
	kfree(t);
	return -ENOENT;
}

static int __devexit tmpa9xx_adc_remove(struct platform_device *pdev)
{
	struct tmpa9xx_adc_core *t = platform_get_drvdata(pdev);

	/* Disable IRQ generation */
	ADC_ADIE = 0;

	free_irq(t->irq, t);

	iounmap(t->adc);

	kfree(t);

	platform_set_drvdata(pdev, NULL);

	g_h = NULL;

	return 0;
}

#ifdef CONFIG_PM

static int tmpa9xx_adc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_ts_priv *t = dev_get_drvdata(dev);

	mutex_lock(&t->lock);

	ADC_ADMOD1 &= ~0x80;

	mutex_unlock(&t->lock);

	return 0;
}

static int tmpa9xx_adc_resume(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_ts_priv *t = dev_get_drvdata(dev);

	mutex_lock(&t->lock);

	ADC_ADMOD1 |= 0x80;

	mutex_unlock(&t->lock);

	return 0;
}

#else
#define tmpa9xx_adc_suspend	NULL
#define tmpa9xx_adc_resume	NULL
#endif

static struct platform_driver tmpa9xx_adc_driver = {
	.probe = tmpa9xx_adc_probe,
	.remove = __devexit_p(tmpa9xx_adc_remove),
	.suspend = tmpa9xx_adc_suspend,
	.resume = tmpa9xx_adc_resume,
	.driver = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init tmpa9xx_adc_init(void)
{
	return platform_driver_register(&tmpa9xx_adc_driver);
}

static void __exit tmpa9xx_adc_exit(void)
{
	platform_driver_unregister(&tmpa9xx_adc_driver);
}

module_init(tmpa9xx_adc_init);
module_exit(tmpa9xx_adc_exit);

MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("TMPA9xx ADC driver");
MODULE_LICENSE("GPL v2");

EXPORT_SYMBOL_GPL(tmpa9xx_adc_read);
