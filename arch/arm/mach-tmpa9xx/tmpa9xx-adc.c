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
#include <linux/sched.h>
#include <linux/io.h>

#define DRIVER_NAME "tmpa9xx-adc"

#define ADC_ADREG0L   (0x00) /* A/D conversion result lower-order register 0 */
#define ADC_ADREG0H   (0x04) /* A/D conversion result higher-order register 0 */
#define ADC_ADREG1L   (0x08) /* A/D conversion result lower-order register 1 */
#define ADC_ADREG1H   (0x0c) /* A/D conversion result higher-order register 1 */
#define ADC_ADREG2L   (0x10) /* A/D conversion result lower-order register 2 */
#define ADC_ADREG2H   (0x14) /* A/D conversion result higher-order register 2 */
#define ADC_ADREG3L   (0x18) /* A/D conversion result lower-order register 3 */
#define ADC_ADREG3H   (0x1c) /* A/D conversion result higher-order register 3 */
#define ADC_ADREG4L   (0x20) /* A/D conversion result lower-order register 4 */
#define ADC_ADREG4H   (0x24) /* A/D conversion result higher-order register 4 */
#define ADC_ADREG5L   (0x28) /* A/D conversion result lower-order register 5 */
#define ADC_ADREG5H   (0x2c) /* A/D conversion result higher-order register 5 */
#define ADC_ADREG6L   (0x30) /* A/D conversion result lower-order register 6 */
#define ADC_ADREG6H   (0x34) /* A/D conversion result higher-order register 6 */
#define ADC_ADREG7L   (0x38) /* A/D conversion result lower-order register 7 */
#define ADC_ADREG7H   (0x3c) /* A/D conversion result higher-order register 7 */
#define ADC_ADREGSPL  (0x40) /* Top-priority A/D conversion result lower-order register */
#define ADC_ADREGSPH  (0x44) /* Top-priority A/D conversion result higher-order register */
#define ADC_ADCOMREGL (0x48) /* A/D conversion result comparison lower-order register */
#define ADC_ADCOMREGH (0x4c) /* A/D conversion result comparison lower-order register */
#define ADC_ADMOD0    (0x50) /* A/D mode control register 0 */
#define ADC_ADMOD1    (0x54) /* A/D mode control register 1 */
#define ADC_ADMOD2    (0x58) /* A/D mode control register 2 */
#define ADC_ADMOD3    (0x5c) /* A/D mode control register 3 */
#define ADC_ADMOD4    (0x60) /* A/D mode control register 4 */
#define ADC_ADCLK     (0x70) /* A/D conversion clock setting register */
#define ADC_ADIE      (0x74) /* A/D interrupt enable register */
#define ADC_ADIS      (0x78) /* A/D interrupt status register */
#define ADC_ADIC      (0x7c) /* A/D interrupt clear register */

#define ADC_ADREGxL(x) (x*0x08)
#define ADC_ADREGxH(x) (x*0x08 + 0x04)

#define adc_writel(x, y, z)	writel(z, x->regs + y)
#define adc_readl(x, y)		readl(x->regs + y)

struct tmpa9xx_adc_core
{
	struct device *dev;
	void __iomem *regs;
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
	uint32_t val;
	int ret;

	/* if the global adc core variable is NULL, this
	means that the module was not correctly loaded */
	BUG_ON(!g_h);

	mutex_lock(&t->lock);

	init_completion(&t->complete);

	val = (adc_readl(t, ADC_ADMOD1) & ~0x0f) | num;
	adc_writel(t, ADC_ADMOD1, val);

	if (delay)
		mdelay(delay);

	val = (adc_readl(t, ADC_ADMOD0)) | (1 << 0);
	adc_writel(t, ADC_ADMOD0, val);

	wait_for_completion(&t->complete);
	ret = t->value;

	mutex_unlock(&t->lock);

	return ret;
}

EXPORT_SYMBOL_GPL(tmpa9xx_adc_read);

static irqreturn_t interrupt_handler(int irq, void *dev)
{
	struct tmpa9xx_adc_core *t = dev;
	int num;

	/* clear interrupt */
	adc_writel(t, ADC_ADIC, (1 << 0));

	num = adc_readl(t, ADC_ADMOD1) & 0x0f;

	t->value = ((adc_readl(t, ADC_ADREGxH(num)) & 0xff) << 2) | ((adc_readl(t, ADC_ADREGxL(num)) >> 6) & 0x3);

	complete(&t->complete);

	return IRQ_HANDLED;
}

static int tmpa9xx_adc_probe(struct platform_device *pdev)
{
	struct tmpa9xx_adc_core *t;
	struct resource	*mem;
	uint32_t val;
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

	t->regs = ioremap(mem->start, resource_size(mem));
	if(!t->regs) {
		dev_err(t->dev, "ioremap() failed\n");
		goto err1;
	}

	/* reset */
	adc_writel(t, ADC_ADMOD4, 0x2);
	udelay(10);
	adc_writel(t, ADC_ADMOD4, 0x1);

	/* disable adc interrupts */
	adc_writel(t, ADC_ADIE, 0x0);

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

	val = adc_readl(t, ADC_ADMOD1) | 0x80;
	adc_writel(t, ADC_ADMOD1, 0x80);

        /* setup ADCCLK */
        adc_writel(t, ADC_ADCLK, (0x01 << 7) | 0x02);

	/* enable IRQs */
	adc_writel(t, ADC_ADIE, (0x01 << 0));

	platform_set_drvdata(pdev, t);

	dev_info(t->dev, "tmpa9xx adc driver registered\n");

	g_h = t;

	return 0;

err3:
err2:
	iounmap(t->regs);
err1:
err0:
	kfree(t);
	return -ENOENT;
}

static int __devexit tmpa9xx_adc_remove(struct platform_device *pdev)
{
	struct tmpa9xx_adc_core *t = platform_get_drvdata(pdev);

	/* disable IRQs */
	adc_writel(t, ADC_ADIE, 0x0);

	free_irq(t->irq, t);

	iounmap(t->regs);

	kfree(t);

	platform_set_drvdata(pdev, NULL);

	g_h = NULL;

	return 0;
}

#ifdef CONFIG_PM

static int tmpa9xx_adc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct tmpa9xx_adc_core *t = g_h;

	mutex_lock(&t->lock);

	ADC_ADMOD1 &= ~0x80;

	mutex_unlock(&t->lock);

	return 0;
}

static int tmpa9xx_adc_resume(struct platform_device *pdev)
{
	struct tmpa9xx_adc_core *t = g_h;

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
