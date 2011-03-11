/*
 * Melody / Alarm driver for Toshiba TMPA9xx processors.
 *
 * Copyright (C) 2010 Thomas Haase <Thomas.Haase@web.de>
 * Copyright (C) 2011 Michael Hunold <michael@mihu.de>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include <linux/tmpa9xx_mldalm.h>

#define DRIVER_DESC "TMPA9xx Melody/Alarm driver"

#define MLDALMINV	(0x00)        /* Melody Alarm Invert Register */
#define MLDALMSEL	(0x04)        /* Melody Alarm signal Select Register */ 
#define ALMCNTCR	(0x08)        /* Alarm Counter Control Register */
#define ALMPATERN	(0x0C)        /* Alarm Pattern Register */
#define MLDCNTCR	(0x10)        /* Melody Counter Control Register */
#define MLDFRQ		(0x14)        /* Melody Frequency Register */

#define ma_writel(b, o, v)	writel(v, b->regs + o)
#define ma_readl(b, o)		readl(b->regs + o)

struct tmpa9xx_mldalm_priv
{
	void __iomem *regs;
	struct device *dev;
	int type;
	bool running;
};

struct tmpa9xx_mldalm_priv *g_ma;

static int tmpa9xx_mldalm_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int tmpa9xx_mldalm_close(struct inode *inode, struct file *file)
{
	return 0;
}

static int tmpa9xx_mldalm_handle_melody(struct tmpa9xx_mldalm_priv *m, int data)
{
	int mldfreqreg;

	ma_writel(m, MLDALMSEL, MLDALM_MELODY);
	m->type = MLDALM_MELODY;

	mldfreqreg = (16384u / data) - 2;
	if (mldfreqreg > 0 && mldfreqreg <= 0xfff) {
		ma_writel(m, MLDFRQ, mldfreqreg);
		return 0;
	}

	dev_err(m->dev, "mldfreqreg %d out of bounds, data %d\n", mldfreqreg, data);
	return -EINVAL;
}

static int tmpa9xx_mldalm_handle_alarm(struct tmpa9xx_mldalm_priv *m, int data)
{
	ma_writel(m, MLDALMSEL, MLDALM_ALARM);
	m->type = MLDALM_ALARM;

	switch (data) {
		case ALMPTSEL_AL0:
		case ALMPTSEL_AL1:
		case ALMPTSEL_AL2:
		case ALMPTSEL_AL3:
		case ALMPTSEL_AL4:
		case ALMPTSEL_AL5:
		case ALMPTSEL_AL6:
		case ALMPTSEL_AL7:
		case ALMPTSEL_AL8:
			ma_writel(m, ALMPATERN, data);
			return 0;
		default:
			break;
	}

	dev_err(m->dev, "data %d out of bounds\n", data);
	return -EINVAL;
}

static int tmpa9xx_mldalm_settype(struct tmpa9xx_mldalm_priv *m, void __user *argp)
{
	struct mldalm_type type;

	if (copy_from_user(&type, argp, sizeof(struct mldalm_type)))
		return -EFAULT;

	if (m->running)
		dev_err(m->dev, "warning, device is running\n");

	if (type.invert != 0 && type.invert != MLDALM_INVERT) {
		dev_err(m->dev, "type.invert %d out of bounds\n", type.invert);
		return -EINVAL;
	}

	ma_writel(m, MLDALMINV, type.invert);

	if (type.type == MLDALM_MELODY)
		return tmpa9xx_mldalm_handle_melody(m, type.data);

	if (type.type == MLDALM_ALARM)
		return tmpa9xx_mldalm_handle_alarm(m, type.data);

	dev_err(m->dev, "type.type %d invalid\n", type.type);
	return -EINVAL;
}

static int tmpa9xx_mldalm_startstop(struct tmpa9xx_mldalm_priv *m, bool running)
{
	if (m->running == running) {
		dev_err(m->dev, "warning, no state change required\n");
		return 0;
	}

	m->running = !!running;

	if (m->type == MLDALM_MELODY)
		ma_writel(m, MLDCNTCR, m->running);
	else
		ma_writel(m, ALMCNTCR, m->running);

	return 0;
}

static long tmpa9xx_mldalm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct tmpa9xx_mldalm_priv *m = g_ma;
	void __user *argp = (void __user *)arg;

	if (cmd == MLDALM_TYPE_SELECT)
		return tmpa9xx_mldalm_settype(m, argp);

	if (cmd == MLDALM_START_STOP) {
		unsigned char running;

		if (get_user(running, (const unsigned char __user *)argp))
			return -EFAULT;

		return tmpa9xx_mldalm_startstop(m, running);
	}

	return -EOPNOTSUPP;
}

static const struct file_operations tmpa9xx_mldalm_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = tmpa9xx_mldalm_ioctl,
	.open = tmpa9xx_mldalm_open,
	.release = tmpa9xx_mldalm_close,
};

static struct miscdevice tmpa9xx_mldalm_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mldalm",
	.fops = &tmpa9xx_mldalm_fops,
};

static int __devinit tmpa9xx_mldalm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_mldalm_priv *m;
	struct resource *res;
	int ret;

	if (tmpa9xx_mldalm_miscdev.parent)
		return -EBUSY;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "platform_get_resource() failed\n");
		return -ENXIO;
	}

	m = kzalloc(sizeof(struct tmpa9xx_mldalm_priv), GFP_KERNEL);
	if (!m) {
		dev_err(dev, "kzalloc() failed\n");
		return -ENOMEM;
	}

	m->regs = ioremap(res->start, resource_size(res));
	if (!m->regs) {
		dev_err(dev, "ioremap() failed\n");
		kfree(m);
		return -ENODEV;
	}

	dev_set_drvdata(dev, m);
	m->dev = dev;

	tmpa9xx_mldalm_miscdev.parent = dev;

	g_ma = m;

	ret = misc_register(&tmpa9xx_mldalm_miscdev);
	if (ret) {
		dev_err(dev, "misc_register() failed\n");
		iounmap(m->regs);
		kfree(m);
		return ret;
	}

	dev_info(dev, DRIVER_DESC " ready\n");

	return 0;
}

static int __devexit tmpa9xx_mldalm_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_mldalm_priv *m = dev_get_drvdata(dev);

	misc_deregister(&tmpa9xx_mldalm_miscdev);

	tmpa9xx_mldalm_startstop(m, false);

	iounmap(m->regs);

	kfree(m);

	dev_set_drvdata(dev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int tmpa9xx_mldalm_suspend(struct platform_device *pdev, pm_message_t message)
{
	return 0;
}

static int tmpa9xx_mldalm_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define tmpa9xx_mldalm_suspend	NULL
#define tmpa9xx_mldalm_resume	NULL
#endif

static struct platform_driver tmpa9xx_mldalm_driver =
{
	.probe	= tmpa9xx_mldalm_probe,
	.remove	= __devexit_p(tmpa9xx_mldalm_remove),
	.suspend = tmpa9xx_mldalm_suspend,
	.resume = tmpa9xx_mldalm_resume,
	.driver = {
		   .name = "tmpa9xx-mldalm",
		   .owner = THIS_MODULE,
		   },
};

static int __init tmpa9xx_mldalm_init(void)
{
	return platform_driver_register(&tmpa9xx_mldalm_driver);
}

static void __exit tmpa9xx_mldalm_exit(void)
{
	platform_driver_unregister(&tmpa9xx_mldalm_driver);
}

module_init(tmpa9xx_mldalm_init);
module_exit(tmpa9xx_mldalm_exit);

MODULE_DESCRIPTION("Melody / Alarm driver for Toshiba TMPA9xx processors");
MODULE_AUTHOR("Thomas Haase <Thomas.Haase@web.de>");
MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
