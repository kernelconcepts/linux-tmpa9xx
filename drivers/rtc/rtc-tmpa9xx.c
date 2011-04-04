#define DEBUG
/*
 * Real-time clock (RTC) driver for TMPA9xx
 *
 * Copyright (c) 2009 Michael Hasselberg <mh@open-engineering.de>
 * Copyright (c) 2011 Michael Hunold <michael@mihu.de>
 *
 * This program is free software; you may redistribute and/or modify
 * it under the terms of the GNU General Public License Version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/delay.h>

struct tmpa9xx_rtc
{
	void __iomem *regs1;
	void __iomem *regs2;
	struct device *dev;
	int irq;

	struct rtc_device *rtc;
};

#define DATA	     (0x000) /* RTC Data Register */
#define COMP	     (0x004) /* RTC Compare Register */
#define PRST	     (0x008) /* RTC Preset Register */
#define ALMINTCTR    (0x200) /* RTC ALM Interrupt Control Register */
#define ALMMIS       (0x204) /* RTC ALM Interrupt Status Register */

#define ALMINTCLR 	(1 << 7)
#define RTCINTCLR 	(1 << 6)
#define AINTEN1		(1 << 5)
#define AINTEN2		(1 << 4)
#define AINTEN64	(1 << 3)
#define AINTEN512	(1 << 2)
#define AINTEN8192	(1 << 1)
#define RTCINTEN	(1 << 0)

#define rtc_writel(b, o, v)	writel(v, (o < ALMINTCTR ? b->regs1 : (b->regs2 - 0x200)) + o)
#define rtc_readl(b, o)		readl((o < ALMINTCTR ? b->regs1 :(b->regs2 - 0x200)) + o)

static int tmp9xx_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tmpa9xx_rtc *r = platform_get_drvdata(pdev);

	dev_dbg(r->dev, "%s():\n", __func__);

	/* nothing to be done here */

	return 0;
}

static int tmp9xx_rtc_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tmpa9xx_rtc *r = platform_get_drvdata(pdev);
	uint32_t val;

	dev_dbg(r->dev, "%s(): enable %d\n", __func__, enable);

	val = rtc_readl(r, ALMINTCTR);

	if (enable)
		val |= (RTCINTEN | RTCINTCLR);
	else
		val &= ~RTCINTEN;

	rtc_writel(r, ALMINTCTR, val);

	return 0;
}

static int tmp9xx_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tmpa9xx_rtc *r = platform_get_drvdata(pdev);
	unsigned long secs;

	rtc_tm_to_time(&alrm->time, &secs);

	rtc_writel(r, COMP, secs);

	/* wait at least 3/32k s */
	udelay(92);

	tmp9xx_rtc_alarm_irq_enable(dev, alrm->enabled);

	dev_dbg(r->dev, "%s(): enabled %d, pending %d, secs %lu\n", __func__, alrm->enabled, alrm->pending, secs);

	return 0;
}

static int tmp9xx_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tmpa9xx_rtc *r = platform_get_drvdata(pdev);
	uint32_t secs;

	secs = rtc_readl(r, DATA);

	dev_dbg(r->dev, "%s(): secs %d\n", __func__, secs);

	rtc_time_to_tm(secs, tm);

	return 0;
}

static int tmp9xx_rtc_set_mmss(struct device *dev, unsigned long secs)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tmpa9xx_rtc *r = platform_get_drvdata(pdev);
	uint32_t val;

	dev_dbg(r->dev, "%s(): secs %lu\n", __func__, secs);

	rtc_writel(r, PRST, secs);

	while(1) {
		val = rtc_readl(r, DATA);
		if (val == secs)
			break;
	}

	return 0;
}

static const struct rtc_class_ops tmp9xx_rtc_ops = {
	.read_time = tmp9xx_rtc_read_time,
	.read_alarm = tmp9xx_rtc_read_alarm,
	.set_alarm = tmp9xx_rtc_set_alarm,
	.set_mmss = tmp9xx_rtc_set_mmss,
	.alarm_irq_enable = tmp9xx_rtc_alarm_irq_enable,
};

static irqreturn_t tmp9xx_rtc_interrupt(int irq, void *data)
{
	struct tmpa9xx_rtc *r = data;
	uint32_t val;

	val = rtc_readl(r, ALMINTCTR);
	dev_dbg(r->dev, "%s(): ALMINTCTR 0x%08x\n", __func__, val);

	val |= RTCINTCLR;
	rtc_writel(r, ALMINTCTR, val);

	rtc_update_irq(r->rtc, 1, RTC_AF | RTC_IRQF);

	return IRQ_HANDLED;
}

static int __devinit probe(struct platform_device *pdev)
{
	struct tmpa9xx_rtc *r;
	struct resource *res;
	int ret;

	r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (!r) {
		dev_err(r->dev, "kzalloc() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	r->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(r->dev, "platform_get_resource() failed @ IORESOURCE_MEM\n");
		ret = -ENODEV;
		goto err1;
	}

	r->regs1 = ioremap(res->start, resource_size(res));
	if (!r->regs1) {
		dev_err(r->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto err2;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(r->dev, "platform_get_resource() failed @ IORESOURCE_MEM\n");
		ret = -ENODEV;
		goto err3;
	}

	r->regs2 = ioremap(res->start, resource_size(res));
	if (!r->regs2) {
		dev_err(r->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto err4;
	}

	r->irq = platform_get_irq(pdev, 0);
	if (r->irq <= 0) {
		dev_err(r->dev, "platform_get_irq() failed\n");
		ret = -ENODEV;
		goto err5;
	}

	ret = request_irq(r->irq, tmp9xx_rtc_interrupt, IRQF_DISABLED, "tmpa9xx_rtc", r);
	if (ret) {
		dev_err(r->dev, "request_irq() failed\n");
		ret = -ENODEV;
		goto err6;
	}

	platform_set_drvdata(pdev, r);

	r->rtc = rtc_device_register(KBUILD_MODNAME, &pdev->dev, &tmp9xx_rtc_ops, THIS_MODULE);
	if (IS_ERR(r->rtc)) {
		dev_err(r->dev, "rtc_device_register() failed\n");
		ret = PTR_ERR(r->rtc);
		goto err7;
	}

        dev_info(r->dev, "ready, io %p/%p. irq %d\n", r->regs1, r->regs2, r->irq);

	return 0;

err7:
	platform_set_drvdata(pdev, NULL);
	free_irq(r->irq, r);
err6:
err5:
	iounmap(r->regs2);
err4:
err3:
	iounmap(r->regs1);
err2:
err1:
	kfree(r);
err0:
	return ret;
}

static int __devexit remove(struct platform_device *pdev)
{
	struct tmpa9xx_rtc *r = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	rtc_device_unregister(r->rtc);

	disable_irq(r->irq);

	free_irq(r->irq, r);

	iounmap(r->regs2);

	iounmap(r->regs1);

	kfree(r);

	return 0;
}

static struct platform_driver tmpa9xx_rtc_driver = {
	.probe = probe,
	.remove = __devexit_p(remove),
	.driver = {
		.name  = "tmpa9xx-rtc",
		.owner = THIS_MODULE,
	},
};

static int __init tmpa9xx_rtc_init(void)
{
	return platform_driver_register(&tmpa9xx_rtc_driver);
}

static void __exit tmpa9xx_rtc_exit(void)
{
	platform_driver_unregister(&tmpa9xx_rtc_driver);
}

module_init(tmpa9xx_rtc_init);
module_exit(tmpa9xx_rtc_exit);

MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("RTC driver for TMPA9xx");
MODULE_LICENSE("GPL");
