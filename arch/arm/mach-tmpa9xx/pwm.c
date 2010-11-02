/*
 * linux/arch/arm/mach-tmpa9xx/pwm.c
 * simple driver for PWM (Pulse Width Modulator) controller
 *
 * (c) 2010 Nils Faerber <nils.faerber@kernelconcepts.de>
 * based on pxa2xx pwm.c by eric miao <eric.miao@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/*
 * The TMPA9xx PWM is implemented in the timer controller.
 * The TMPA9xx has six timers in three blocks. Each block uses
 * one common input clock, so there are only three independant
 * timers available.
 * Only Block-1 and Block-2 habe PWM option in their first parts, Block-3
 * only has pure timers.
 * Block-1 is used as periodic system timer source so we can only use
 * function 1 of Block-2 as PWM output through Port-C4.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>

#include <asm/div64.h>

#include <mach/regs.h>
#include <mach/timer.h>

#define HAS_SECONDARY_PWM	0x10
#define PWM_ID_BASE(d)		((d) & 0xf)

static const struct platform_device_id pwm_id_table[] = {
	/*   PWM    has_secondary_pwm? */
	{ "tmpa9xx-pwm", 0 }, /* TMPA9XX have two PWM channels but only one usable */
	{ },
};
MODULE_DEVICE_TABLE(platform, pwm_id_table);

/* PWM registers and bits definitions */

struct pwm_device {
	struct list_head	node;
	struct pwm_device	*secondary;
	struct platform_device	*pdev;

	const char	*label;
	struct clk	*clk;
	int		clk_enabled;
	void __iomem	*mmio_base;

	unsigned int	use_count;
	unsigned int	pwm_id;
};

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	volatile struct tmpa9xx_hw_timer *hw_timer =
	    (volatile struct tmpa9xx_hw_timer *)TMPA9XX_TIMER2;

	/* printk(KERN_ERR "pwm_config() duty=%d period=%d\n", duty_ns, period_ns); */

	if (pwm == NULL || period_ns == 0 || duty_ns > period_ns)
		return -EINVAL;

	hw_timer->TimerMode = 0x00;		/* disable PWM mode */
	hw_timer->TimerControl = 0x00; /* stop */
	hw_timer->TimerCompare1 = 0xff - (unsigned char)duty_ns;
	hw_timer->TimerMode = 0x40;		/* select PWM mode */
	hw_timer->TimerControl = 0xe2; /* start */

	return 0;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwm)
{
	volatile struct tmpa9xx_hw_timer *hw_timer =
	    (volatile struct tmpa9xx_hw_timer *)TMPA9XX_TIMER2;
	int rc = 0;

	/* printk(KERN_ERR "pwm_enable()\n"); */

	hw_timer->TimerMode = 0x40;		/* select PWM mode */
	hw_timer->TimerControl = 0xe2; /* start */

	return rc;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	volatile struct tmpa9xx_hw_timer *hw_timer =
	    (volatile struct tmpa9xx_hw_timer *)TMPA9XX_TIMER2;

	/* printk(KERN_ERR "pwm_disable()\n"); */

	if (pwm->clk_enabled) {
		pwm->clk_enabled = 0;
	}
	hw_timer->TimerControl = 0x00; /* stop */
}
EXPORT_SYMBOL(pwm_disable);

static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_list);

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	struct pwm_device *pwm;
	int found = 0;

	mutex_lock(&pwm_lock);

	list_for_each_entry(pwm, &pwm_list, node) {
		if (pwm->pwm_id == pwm_id) {
			found = 1;
			break;
		}
	}

	if (found) {
		if (pwm->use_count == 0) {
			pwm->use_count++;
			pwm->label = label;
		} else
			pwm = ERR_PTR(-EBUSY);
	} else
		pwm = ERR_PTR(-ENOENT);

	mutex_unlock(&pwm_lock);
	return pwm;
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwm)
{
	mutex_lock(&pwm_lock);

	if (pwm->use_count) {
		pwm->use_count--;
		pwm->label = NULL;
	} else
		pr_warning("PWM device already freed\n");

	mutex_unlock(&pwm_lock);
}
EXPORT_SYMBOL(pwm_free);

static inline void __add_pwm(struct pwm_device *pwm)
{
	mutex_lock(&pwm_lock);
	list_add_tail(&pwm->node, &pwm_list);
	mutex_unlock(&pwm_lock);
}

static int __devinit pwm_probe(struct platform_device *pdev)
{
	volatile struct tmpa9xx_hw_timer *hw_timer =
	    (volatile struct tmpa9xx_hw_timer *)TMPA9XX_TIMER2;
	struct platform_device_id *id = platform_get_device_id(pdev);
	struct pwm_device *pwm, *secondary = NULL;
	struct resource *r;
	int ret = 0;

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pwm->clk_enabled = 0;

	pwm->use_count = 0;
	pwm->pwm_id = PWM_ID_BASE(id->driver_data) + pdev->id;
	pwm->pdev = pdev;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free;
	}

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free;
	}

	pwm->mmio_base = ioremap(r->start, resource_size(r));
	if (pwm->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	__add_pwm(pwm);

	/* example from TMPA900 handbook */
	hw_timer->TimerControl = 0x00;	/* stop */
	hw_timer->TimerMode = 0x40;		/* select PWM mode */
	hw_timer->TimerCompare1 = 0x37;	/* initial counter */
	hw_timer->TimerCmpEn = 0x01;	/* emable compare */
	hw_timer->TimerControl = 0x62;	/*TimerControl*/;

	platform_set_drvdata(pdev, pwm);

	return 0;

err_free_mem:
	release_mem_region(r->start, resource_size(r));
err_free:
	kfree(pwm);
	return ret;
}

static int __devexit pwm_remove(struct platform_device *pdev)
{
	struct pwm_device *pwm;
	struct resource *r;

	pwm = platform_get_drvdata(pdev);
	if (pwm == NULL)
		return -ENODEV;

	mutex_lock(&pwm_lock);

	if (pwm->secondary) {
		list_del(&pwm->secondary->node);
		kfree(pwm->secondary);
	}

	list_del(&pwm->node);
	mutex_unlock(&pwm_lock);

	iounmap(pwm->mmio_base);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, resource_size(r));

	kfree(pwm);
	return 0;
}

static struct platform_driver pwm_driver = {
	.driver		= {
		.name	= "tmpa9xx-pwm",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_probe,
	.remove		= __devexit_p(pwm_remove),
	.id_table	= pwm_id_table,
};

static int __init pwm_init(void)
{
	/* printk (KERN_ERR "tmpa9xx pwm_init()\n"); */
	return platform_driver_register(&pwm_driver);
}
arch_initcall(pwm_init);

static void __exit pwm_exit(void)
{
	platform_driver_unregister(&pwm_driver);
}
module_exit(pwm_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Nils Faerber <nils.faerber@kernelconcepts.de>");

