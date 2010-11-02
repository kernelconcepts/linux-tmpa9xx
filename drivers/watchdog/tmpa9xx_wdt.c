/*
 * Watchdog driver for Toshiba tmpa9xx processors.
 *
 * Copyright (C) 2010, 2010 Florian Boor <florian.boor@kernelconcepts.de> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>
#include <mach/regs.h>

#define DRV_NAME "TMPA9xx Watchdog"

#define SYSCLK 	(192*1000*1000)	// tmpa9xx is clocked with
#define PCLK 	(SYSCLK/2)	

/* TMPA9xx runns 32bit counter @ PCLK */

#define s_to_ticks(t)	(t*PCLK)
#define ticks_to_s(t)	(t/PCLK)

#define WDT_HEARTBEAT 10

static int heartbeat = WDT_HEARTBEAT;
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeats in seconds. "
	"(default = " __MODULE_STRING(WDT_HEARTBEAT) ")");

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
	"(default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

/* ......................................................................... */


/*
 * Enable the watchdog timer.
 */
static inline void tmpa9xx_wdt_enable(void)
{
	WDT_WDOGLOCK	= 0x1ACCE551; 	/* Enable writing to WDT registers */
	WDT_WDOGCONTROL = 0x3;		/* Enables the WDT counter and the WDT interrupt, enables WDT reset output. */
        WDT_WDOGLOCK	= 0;		/* Disable writing to WDT registers */
}

/*
 * Disable the watchdog timer.
 */
static inline void tmpa9xx_wdt_disable(void)
{
	WDT_WDOGLOCK	= 0x1ACCE551; 	/* Enable writing to WDT registers */
	WDT_WDOGCONTROL = 0x0;		/* Enables the WDT counter and the WDT interrupt, enables WDT reset output. */
        WDT_WDOGLOCK	= 0;		/* Disable writing to WDT registers */
}

/*
 * Reload the watchdog timer.  (ie, pat the watchdog)
 */
static inline void tmpa9xx_wdt_reset(void)
{
	WDT_WDOGLOCK	= 0x1ACCE551; 	/* Enable writing to WDT registers */
	WDT_WDOGINTCLR  = 1; 		/* Writing any value resets watchdog */
        WDT_WDOGLOCK	= 0;		/* Disable writing to WDT registers */
}

/*
 * Setup value the watchdog timer.
 */
static inline void tmpa9xx_wdt_set_value(long timeout)
{
	WDT_WDOGLOCK	= 0x1ACCE551;	/* Enable writing to WDT registers */
	WDT_WDOGLOAD 	= timeout;	/* Set WDT value */
	WDT_WDOGLOCK	= 0x0;		/* Disable writing to WDT registers */
}


/*
 * Watchdog device is opened, and watchdog starts running.
 */
static int tmpa9xx_wdt_open(struct inode *inode, struct file *file)
{
	tmpa9xx_wdt_set_value(s_to_ticks(WDT_HEARTBEAT));
        tmpa9xx_wdt_enable();
	return nonseekable_open(inode, file);
}

/*
 * Close the watchdog device.
 */
static int tmpa9xx_wdt_close(struct inode *inode, struct file *file)
{
	if (!nowayout)
        {
		tmpa9xx_wdt_disable();
       	}
        
	return 0;
}

static const struct watchdog_info tmpa9xx_wdt_info = {
	.identity	= DRV_NAME,
	.options	= WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
};

/*
 * Handle commands from user-space.
 */
static long tmpa9xx_wdt_ioctl(struct file *file,unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_value;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &tmpa9xx_wdt_info,sizeof(tmpa9xx_wdt_info)) ? -EFAULT : 0;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);

	case WDIOC_KEEPALIVE:
		tmpa9xx_wdt_reset();
		return 0;

	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;
		tmpa9xx_wdt_set_value(s_to_ticks(new_value));
		return put_user(ticks_to_s(WDT_WDOGLOAD), p);  /* return current value */

	case WDIOC_GETTIMEOUT:
		return put_user(ticks_to_s(WDT_WDOGLOAD), p);
                
	case WDIOC_GETTIMELEFT:    
		return put_user(ticks_to_s(WDT_WDOGVALUE), p);
                
	}
	return -ENOTTY;
}

/*
 * Pat the watchdog whenever device is written to.
 */
static ssize_t tmpa9xx_wdt_write(struct file *file, const char *data, size_t len,loff_t *ppos)
{
	if (!len)
		return 0;

	/* Scan for magic character */
	if (nowayout)
        {
		size_t i;
		for (i = 0; i < len; i++) {
			char c;
			if (get_user(c, data + i))
				return -EFAULT;
			if (c == 'V') {
				tmpa9xx_wdt_disable();
				break;
			}
		}
	}

	tmpa9xx_wdt_reset();
        
	return len;
}

/* ......................................................................... */

static const struct file_operations tmpa9xx_wdt_fops = {
	.owner			= THIS_MODULE,
	.llseek			= no_llseek,
	.unlocked_ioctl		= tmpa9xx_wdt_ioctl,
	.open			= tmpa9xx_wdt_open,
	.release		= tmpa9xx_wdt_close,
	.write			= tmpa9xx_wdt_write,
};

static struct miscdevice tmpa9xx_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &tmpa9xx_wdt_fops,
};

static int __init tmpa9xx_wdt_probe(struct platform_device *pdev)
{
	int res;

	if (tmpa9xx_wdt_miscdev.parent)
		return -EBUSY;
	tmpa9xx_wdt_miscdev.parent = &pdev->dev;

	/* Set watchdog */

	res = misc_register(&tmpa9xx_wdt_miscdev);
	if (res)
		return res;

	printk(KERN_INFO DRV_NAME " enabled (heartbeat=%d sec, nowayout=%d)\n",	heartbeat, nowayout);

	return 0;
}

static int __exit tmpa9xx_wdt_remove(struct platform_device *pdev)
{
	int res;

	res = misc_deregister(&tmpa9xx_wdt_miscdev);
	if (!res)
		tmpa9xx_wdt_miscdev.parent = NULL;

	return res;
}

#ifdef CONFIG_PM

static int tmpa9xx_wdt_suspend(struct platform_device *pdev, pm_message_t message)
{
	return 0;
}

static int tmpa9xx_wdt_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define tmpa9xx_wdt_suspend	NULL
#define tmpa9xx_wdt_resume	NULL
#endif

static struct platform_driver tmpa9xx_wdt_driver = {
	.remove		= __exit_p(tmpa9xx_wdt_remove),
	.suspend	= tmpa9xx_wdt_suspend,
	.resume		= tmpa9xx_wdt_resume,
	.driver		= {
		.name	= "tmpa9xx_wdt",
		.owner	= THIS_MODULE,
	},
};

static int __init tmpa9xx_wdt_init(void)
{
	return platform_driver_probe(&tmpa9xx_wdt_driver, tmpa9xx_wdt_probe);
}

static void __exit tmpa9xx_wdt_exit(void)
{
	platform_driver_unregister(&tmpa9xx_wdt_driver);
}

module_init(tmpa9xx_wdt_init);
module_exit(tmpa9xx_wdt_exit);

MODULE_AUTHOR("Florian Boor <florian.boor@kernelconcepts.de>");
MODULE_DESCRIPTION("Watchdog driver for Toshiba tmpa9xx processors");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
