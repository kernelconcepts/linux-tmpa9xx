/*
 *  Driver for Toshiba TMPA910 Real Time Clock unit.
 *  derived from rtc_tmpa910.c
 *  Copyright (C) 2003-2006  Yoichi Yuasa <yoichi_yuasa@tripeaks.co.jp>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/delay.h>


#include <asm/system.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/tmpa910_regs.h>
#include <mach/irqs.h>

#define RTC_BASE		0xF0030000

#define RTCDATA		(RTC_BASE + 0x0000)
#define RTCCOMP		(RTC_BASE + 0x0004)
#define RTCPRST		(RTC_BASE + 0x0008)
#define RTCALMINTCR	(RTC_BASE + 0x0200) 
#define RTCALMMIS	(RTC_BASE + 0x0204)

#define VICINTENABLE	0xF4000010

MODULE_AUTHOR("Michael Hasselberg <mh@open-engineering.de>");
MODULE_DESCRIPTION("Toshiba TMPA910 RTC driver");
MODULE_LICENSE("GPL");

/*
 * This is the initial release of the TMPA910 RTC driver
 * it currently only supports set / read time and set / read alarm
 * TODO:
 * implement RTC_WKLAM_xxx and RTC_PIE_xxx ioctls and functionality
 * RTC_UIE_xxx is not available on TMPA910
 *
 */

static unsigned long epoch = 1970;	/* Jan 1 1970 00:00:00 */

static DEFINE_SPINLOCK(rtc_lock);
static unsigned long rtc_alarm_value;
static unsigned int rtc_irq_enabled;
static int rtc_irq = -1;
/* static int pie_irq = -1; */
static void __iomem *rtc_base;

static inline unsigned long read_elapsed_second(void)
{
	return _in32(RTCDATA);
}

static inline void write_elapsed_second(unsigned long sec)
{
	spin_lock_irq(&rtc_lock);

	_out32(RTCCOMP, 0);
	udelay(100);
	_out32(RTCPRST, sec);
	udelay(100);
	while (_in32(RTCDATA) != sec);

	spin_unlock_irq(&rtc_lock);
}

static void tmpa910_rtc_release(struct device *dev)
{
	uint32_t reg;
//printk("rtc: release\n");
	spin_lock_irq(&rtc_lock);

	if (rtc_irq_enabled) {
//printk("rtc: ioctl alarm disable\n");
		reg = _in32(RTCALMINTCR);
		reg &= 0x3e;
		_out32(RTCALMINTCR, reg);
		disable_irq(rtc_irq);
		rtc_irq_enabled = 0;
	}

	spin_unlock_irq(&rtc_lock);

}

static int tmpa910_rtc_read_time(struct device *dev, struct rtc_time *time)
{
	unsigned long epoch_sec, elapsed_sec;
//printk("rtc: read time\n");
	epoch_sec = mktime(epoch, 1, 1, 0, 0, 0);
	elapsed_sec = read_elapsed_second();

	rtc_time_to_tm(epoch_sec + elapsed_sec, time);

	return 0;
}

static int tmpa910_rtc_set_time(struct device *dev, struct rtc_time *time)
{
	unsigned long epoch_sec, current_sec;
//printk("rtc: set time\n");
	epoch_sec = mktime(epoch, 1, 1, 0, 0, 0);
	current_sec = mktime(time->tm_year + 1900, time->tm_mon + 1, time->tm_mday,
	                     time->tm_hour, time->tm_min, time->tm_sec);

	write_elapsed_second(current_sec - epoch_sec);

	return 0;
}

static int tmpa910_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct rtc_time *time = &wkalrm->time;
//printk("rtc: read alarm\n");
	spin_lock_irq(&rtc_lock);

	wkalrm->enabled = rtc_irq_enabled;

	spin_unlock_irq(&rtc_lock);

	rtc_time_to_tm(rtc_alarm_value, time);

	return 0;
}

static int tmpa910_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	unsigned long alarm_sec;
	uint32_t reg;

	struct rtc_time *time = &wkalrm->time;
//printk("rtc: set alarm 1\n");
	alarm_sec = mktime(time->tm_year + 1900, time->tm_mon + 1, time->tm_mday,
	                   time->tm_hour, time->tm_min, time->tm_sec);

	spin_lock_irq(&rtc_lock);

	rtc_alarm_value = alarm_sec;
//reg = _in32(RTCDATA);
//udelay(100);
//printk("rtc: set alarm 2 rtc: %u, alm: %u\n",reg,alarm_sec);
	_out32(RTCCOMP, alarm_sec);
	udelay(100);

	if (wkalrm->enabled) {
		reg = _in32(RTCALMINTCR);
		reg |= 0x41;
		_out32(RTCALMINTCR, reg);
		enable_irq(rtc_irq);
	}
	rtc_irq_enabled = wkalrm->enabled;
//printk("rtc: enable alarm %u\n", rtc_irq_enabled);
	spin_unlock_irq(&rtc_lock);

	return 0;
}

static int tmpa910_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	uint32_t reg;
/*
	unsigned long count;
*/
//printk("rtc: ioctl\n");
	switch (cmd) {
	case RTC_AIE_ON:
//printk("rtc: ioctl RTC_AIE_ON\n");
		spin_lock_irq(&rtc_lock);

		if (!rtc_irq_enabled) {
//printk("rtc: ioctl alarm enable\n");
			enable_irq(rtc_irq);
//printk("rtc: RTCALMMIS=0x%02x\n",_in32(RTCALMMIS));
			reg = _in32(RTCALMINTCR);
//printk("rtc: RTCALMINTCR=0x%02x\n",reg);
			reg &= 0x3f;
			reg |= 0x41;
			_out32(RTCALMINTCR, reg);
//printk("rtc: RTCALMINTCR=0x%02x\n",reg);
			reg = _in32(VICINTENABLE);
//printk("rtc: VICINTENABLE=0x%04x\n",reg);
			rtc_irq_enabled = 1;
		}
		spin_unlock_irq(&rtc_lock);
		break;
	case RTC_AIE_OFF:
//printk("rtc: ioctl RTC_AIE_OFF\n");
		spin_lock_irq(&rtc_lock);

		if (rtc_irq_enabled) {
//printk("rtc: ioctl alarm disable\n");
			reg = _in32(RTCALMINTCR);
			reg &= 0x3e;
			_out32(RTCALMINTCR, reg);
			disable_irq(rtc_irq);
			rtc_irq_enabled = 0;
		}

		spin_unlock_irq(&rtc_lock);
		break;
#if 0
	case RTC_PIE_ON:
		enable_irq(pie_irq);
		break;
	case RTC_PIE_OFF:
		disable_irq(pie_irq);
		break;
	case RTC_IRQP_READ:
		return put_user(periodic_frequency, (unsigned long __user *)arg);
		break;
	case RTC_IRQP_SET:
		if (arg > MAX_PERIODIC_RATE)
			return -EINVAL;

		periodic_frequency = arg;

		count = RTC_FREQUENCY;
		do_div(count, arg);

		periodic_count = count;

		spin_lock_irq(&rtc_lock);

		rtc1_write(RTCL1LREG, count);
		rtc1_write(RTCL1HREG, count >> 16);

		spin_unlock_irq(&rtc_lock);
		break;
#endif
	case RTC_EPOCH_READ:
		return put_user(epoch, (unsigned long __user *)arg);
	case RTC_EPOCH_SET:
		/* Doesn't support before 1900 */
		if (arg < 1900)
			return -EINVAL;
		epoch = arg;
		break;
	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

static irqreturn_t elapsedtime_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = (struct platform_device *)dev_id;
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	uint32_t reg, reg2;
//printk("rtc: interrupt\n");
	reg2 = _in32(RTCALMMIS);
//printk("rtc: RTCALMMIS=0x%02x\n",reg2);
	if (reg2 & 0x01) {
//printk("rtc: interrupt 1\n");
		reg = _in32(RTCALMINTCR);
//printk("rtc: RTCALMINTCR=0x%02x\n",reg);
		reg &= 0x3f;
		reg |= 0x40;
		_out32(RTCALMINTCR, reg);
//printk("rtc: RTCALMINTCR=0x%02x\n",reg);
		rtc_update_irq(rtc, 1, RTC_AF);
	}
	if (reg2 & 0x02) {
//printk("rtc: interrupt 2\n");
		reg = _in32(RTCALMINTCR);
		reg &= 0x3f;
		reg |= 0x80;
		_out32(RTCALMINTCR, reg);
		printk(KERN_INFO "tmpa910_rtc: GLITCH! ALM irq triggered\n");
	}

	return IRQ_HANDLED;
}

static const struct rtc_class_ops tmpa910_rtc_ops = {
	.release	= tmpa910_rtc_release,
	.ioctl		= tmpa910_rtc_ioctl,
	.read_time	= tmpa910_rtc_read_time,
	.set_time	= tmpa910_rtc_set_time,
	.read_alarm	= tmpa910_rtc_read_alarm,
	.set_alarm	= tmpa910_rtc_set_alarm,
};

static int __devinit tmpa910_rtc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct rtc_device *rtc;
	int retval;
	uint32_t reg;

//printk("rtc: rtc_probe start\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EBUSY;

	rtc_base = ioremap(res->start, res->end - res->start + 1);
	if (!rtc_base)
		return -EBUSY;

	rtc = rtc_device_register("tmpa910", &pdev->dev, &tmpa910_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		retval = PTR_ERR(rtc);
		goto err_iounmap_all;
	}

	spin_lock_irq(&rtc_lock);

	reg = _in32(RTCALMINTCR);
	reg &= 0x3e;
	reg |= 0xc0;
	_out32(RTCALMINTCR, reg);
	udelay(100);
	_out32(RTCCOMP, 0);
	udelay(100);
	_out32(RTCPRST, 0);
	udelay(100);
	while (_in32(RTCDATA) != 0);

	spin_unlock_irq(&rtc_lock);

	rtc_irq = platform_get_irq(pdev, 0);
	if (rtc_irq < 0 || rtc_irq >= NR_IRQS) {
		retval = -EBUSY;
		goto err_device_unregister;
	}

	retval = request_irq(rtc_irq, elapsedtime_interrupt, 0,
	                     "tmpa910-rtc", pdev);
	if (retval < 0)
		goto err_device_unregister;

	platform_set_drvdata(pdev, rtc);

	disable_irq(rtc_irq);

	printk(KERN_INFO "rtc: Toshiba TMPA910 Real Time Clock\n");

	return 0;

err_device_unregister:
	rtc_device_unregister(rtc);

err_iounmap_all:
	iounmap(rtc_base);
	rtc_base = NULL;

	return retval;
}

static int __devexit tmpa910_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc;

	rtc = platform_get_drvdata(pdev);
	if (rtc)
		rtc_device_unregister(rtc);

	platform_set_drvdata(pdev, NULL);

	free_irq(rtc_irq, pdev);
	/* free_irq(pie_irq, pdev); */
	if (rtc_base)
		iounmap(rtc_base);

	return 0;
}
#ifdef CONFIG_PM

/* RTC Power management control */
#define tmpa910_rtc_suspend NULL
#define tmpa910_rtc_resume  NULL

#else
#define tmpa910_rtc_suspend NULL
#define tmpa910_rtc_resume  NULL
#endif


/* work with hotplug and coldplug */
MODULE_ALIAS("platform:RTC");

static struct platform_driver tmpa910_rtc_platform_driver = {
	.probe		= tmpa910_rtc_probe,
	.remove		= __devexit_p(tmpa910_rtc_remove),
	.suspend	= tmpa910_rtc_suspend,
	.resume		= tmpa910_rtc_resume,
	.driver		= {
		.name	= "tmpa910_rtc",
		.owner	= THIS_MODULE
	},

};

static int __init tmpa910_rtc_init(void)
{	
	return platform_driver_register(&tmpa910_rtc_platform_driver);
}

static void __exit tmpa910_rtc_exit(void)
{
	platform_driver_unregister(&tmpa910_rtc_platform_driver);
}

module_init(tmpa910_rtc_init);
module_exit(tmpa910_rtc_exit);

