/*
 * Melody / Alarm driver for Toshiba TMPA9xx processors.
 *
 * Copyright (C) 2010, Thomas Haase <Thomas.Haase@web.de>
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
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>
#include <linux/tmpa9xx_mldalm.h>
#include <mach/regs.h>

#define DRV_NAME "TMPA9xx Melody/Alarm"

/* ......................................................................... */

unsigned char running=0;
unsigned char used_type=0;

/*
 *  Open the Melody/Alarm device
 */
static int tmpa9x0_mldalm_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

/*
 * Close the Melody/Alarm device.
 */
static int tmpa9x0_mldalm_close(struct inode *inode, struct file *file)
{
        
	return 0;
}

static void tmpa9x0_mldalm_settype(struct mldalm_type type)
{
	
        
        switch (type.invert)
        {
        case 0:
        case MLDALM_INVERT:
		MLDALMINV = type.invert;
                break;
        default:
        	printk(KERN_ERR "Impossible invert value\n");
		break;
        }

	switch (type.type)
        {
        case MLDALM_MELODY:
        	MLDALMSEL = type.type;
                used_type = type.type;
                if (type.data <= 0xfff)
			MLDFRQ = type.data;
                else
                {
			MLDFRQ = 0;
                	printk(KERN_ERR "Melody frequency out of range\n");
                }
                break;
                
        
        case MLDALM_ALARM:
        	MLDALMSEL = type.type;
                used_type = type.type;
                switch (type.data)
                {
                case ALMPTSEL_AL0:
                case ALMPTSEL_AL1:
                case ALMPTSEL_AL2:
                case ALMPTSEL_AL3:
                case ALMPTSEL_AL4:
                case ALMPTSEL_AL5:
                case ALMPTSEL_AL6:
                case ALMPTSEL_AL7:
                case ALMPTSEL_AL8:
                	ALMPATERN = type.data;
                        break;
                        
                default:
                 	ALMPATERN = 0;
                 	printk(KERN_ERR "No valid alarm pattern\n");
                        break;
                }
                break;
        default:
        	printk(KERN_ERR "Invalid output signal select\n");
		break;
        }        
        
}

static void tmpa9x0_mldalm_startstop(unsigned char start_stop)
{
	switch (start_stop) {
        
        case MLDALM_START:
        	running=1;
                if (used_type==MLDALM_MELODY)
         		MLDCNTCR = 0x1;       	
		else                        
	                ALMCNTCR = 0x1;
                break;
                
	case MLDALM_STOP:
        	running=0;
                if (used_type==MLDALM_MELODY)
         		MLDCNTCR = 0x0;
		else                        
	                ALMCNTCR = 0x0;
                break;
        default:
        	printk(KERN_ERR "Not start or stop\n");
                break;
        }         
}

/*
 * Handle commands from user-space.
 */
static long tmpa9x0_mldalm_ioctl(struct file *file,unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	struct mldalm_type type;
        unsigned char start_stop;

	switch (cmd) {

	case MLDALM_TYPE_SELECT:
		if (copy_from_user(&type,p,sizeof(struct mldalm_type)))
			return -EFAULT;
                if (!running)
			tmpa9x0_mldalm_settype(type);
                else
                	printk(KERN_ERR "Not stopped\n");
		break;                        
       	
        case MLDALM_START_STOP:
		if (get_user(start_stop,(const unsigned char __user *)argp))
			return -EFAULT;
		tmpa9x0_mldalm_startstop(start_stop);
                break;
                
        default:
        	printk(KERN_ERR "Unkonwn iocl - %d\n",cmd);
                
	}
        
	return -ENOTTY;
}


/* ......................................................................... */

static const struct file_operations tmpa9x0_mldalm_fops = {
	.owner			= THIS_MODULE,
	.llseek			= no_llseek,
	.unlocked_ioctl		= tmpa9x0_mldalm_ioctl,
	.open			= tmpa9x0_mldalm_open,
	.release		= tmpa9x0_mldalm_close,
};

static struct miscdevice tmpa9x0_mldalm_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "mldalm",
	.fops		= &tmpa9x0_mldalm_fops,
};

static int __init tmpa9x0_mldalm_probe(struct platform_device *pdev)
{
	int ret;

	if (tmpa9x0_mldalm_miscdev.parent)
		return -EBUSY;
	tmpa9x0_mldalm_miscdev.parent = &pdev->dev;

	

        ret = misc_register(&tmpa9x0_mldalm_miscdev);

	printk(KERN_INFO DRV_NAME " enabled\n");

	return 0;
}

static int __exit tmpa9x0_mldalm_remove(struct platform_device *pdev)
{
	int res;

	res = misc_deregister(&tmpa9x0_mldalm_miscdev);
	if (!res)
		tmpa9x0_mldalm_miscdev.parent = NULL;

	return res;
}

#ifdef CONFIG_PM

static int tmpa9x0_mldalm_suspend(struct platform_device *pdev, pm_message_t message)
{
	return 0;
}

static int tmpa9x0_mldalm_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define tmpa9x0_mldalm_suspend	NULL
#define tmpa9x0_mldalm_resume	NULL
#endif

static struct platform_driver tmpa9x0_mldalm_driver = {
	.remove		= __exit_p(tmpa9x0_mldalm_remove),
	.suspend	= tmpa9x0_mldalm_suspend,
	.resume		= tmpa9x0_mldalm_resume,
	.driver		= {
		.name	= "tmpa9xx_mldalm",
		.owner	= THIS_MODULE,
	},
};

static int __init tmpa9x0_mldalm_init(void)
{
	return platform_driver_probe(&tmpa9x0_mldalm_driver, tmpa9x0_mldalm_probe);
}

static void __exit tmpa9x0_mldalm_exit(void)
{
	platform_driver_unregister(&tmpa9x0_mldalm_driver);
}

module_init(tmpa9x0_mldalm_init);
module_exit(tmpa9x0_mldalm_exit);

MODULE_AUTHOR("Thomas Haase <Thomas.Haase@web.de>");
MODULE_DESCRIPTION("Melody / Alarm driver for Toshiba TMPA9xx processors");
MODULE_LICENSE("GPL");
