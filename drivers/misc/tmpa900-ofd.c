/*
 *  Oscillation frequency detection (OFD) driver for TMPA900
 *
 *  Copyright (C) 2010 Thomas Haase <Thomas.Haase@web.de>
 *  Copyright (c) 2011 Michael Hunold <michael@mihu.de>
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
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <mach/regs.h>

struct tmpa900_ofd
{
	void __iomem *regs;
	struct device *dev;
};

#define OFD_CLKSCR1            __REG(OFD_BASE_ADDRESS + 0x0000)
#define OFD_CLKSCR2            __REG(OFD_BASE_ADDRESS + 0x0004)
#define OFD_CLKSCR3            __REG(OFD_BASE_ADDRESS + 0x0008)
#define OFD_CLKSMN             __REG(OFD_BASE_ADDRESS + 0x0010)
#define OFD_CLKSMX             __REG(OFD_BASE_ADDRESS + 0x0020)

#define FOSCH 24000000		/* 24 MHz oscillator clock */

#define CLK_WRITE_ENABLE  0xf9 /*  Enable writing to the clock registers */
#define CLK_WRITE_DISABLE 0x06 /* Disable writing to the clock registers */

#define CLK_S_ENABLE      0xe4 /*  Enable OFD operation */
#define CLK_S_DISABLE     0x00 /* Disable OFD operation */

#define OFD_RESET_ENABLE (1<<1)/* Enable OFD reset */
#define OFD_CLEAR_CLKSF  (1<<0)/* Clear High speed oscillation frequency detection flag */

static int ofd_enable(void)
{
	int i;

	/* Initialize the OFD Module */
	OFD_CLKSCR1 = CLK_WRITE_ENABLE;	/* Enable writing to the OFD registers */
	OFD_CLKSCR2 = CLK_S_DISABLE;	/* Enable OFD operation */
        udelay(1);
        OFD_CLKSCR3 = OFD_RESET_ENABLE 	/* RESEN Enable & Clear OSC Flag */
                      |OFD_CLEAR_CLKSF;

        for (i=0;i<100;i++)
        {
        	if ( (OFD_CLKSCR3 & OFD_RESET_ENABLE) == OFD_RESET_ENABLE)
        		break;
                udelay(1);
	}

        if (i>=99)
        	goto err;

        OFD_CLKSMN = (unsigned long) ((float)FOSCH * 0.9 / 32768.0 / 4.0 +0.5 );
        OFD_CLKSMX = (unsigned long) ((float)FOSCH * 1.1 / 32768.0 / 4.0 +0.5 );

        udelay(1);
	OFD_CLKSCR2 = CLK_S_ENABLE;		/* Enable OFD operation */
	OFD_CLKSCR1 = CLK_WRITE_DISABLE;	/* Disable writing to the OFD registers */
	printk(KERN_INFO "Enable OFD - success\n");

        return 0;
err:
	printk(KERN_INFO "Enable OFD - failed\n");
        return -EFAULT;
}

static int ofd_disable(void)
{
	int i;

	OFD_CLKSCR1 = CLK_WRITE_ENABLE;	/* Enable writing to the OFD registers */
	OFD_CLKSCR2 = CLK_S_DISABLE;	/* Enable OFD operation */
        udelay(1);
        OFD_CLKSCR3 = OFD_CLEAR_CLKSF; 	/* RESEN Enable & Clear OSC Flag */

        for (i=0;i<100;i++)
        {
        	if ( (OFD_CLKSCR3 & OFD_RESET_ENABLE) != OFD_RESET_ENABLE)
        		break;
                udelay(1);
	}

        if (i>=99)
        	goto err;

	OFD_CLKSCR1 = CLK_WRITE_DISABLE;	/* Disable writing to the OFD registers */
	printk(KERN_INFO "Disable OFD - success\n");

        return 0;
err:
	printk(KERN_INFO "Disable OFD - failed\n");
	return -EFAULT;
}

static int __devinit probe(struct platform_device *pdev)
{
	struct tmpa900_ofd *c;
	struct resource *res;
	int ret;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c) {
		dev_err(&pdev->dev, "kzalloc() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	c->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "platform_get_resource() failed @ IORESOURCE_MEM\n");
		ret = -ENODEV;
		goto err1;
	}

	c->regs = ioremap(res->start, resource_size(res));
	if (!c->regs) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto err2;
	}

	platform_set_drvdata(pdev, c);

	ret = ofd_enable();
	if (ret) {
		dev_err(&pdev->dev, "ofd_enable() failed\n");
		ret = -ENODEV;
		goto err3;
	}

        dev_info(&pdev->dev, "ready\n");

	return 0;

err3:
	platform_set_drvdata(pdev, NULL);
	iounmap(c->regs);
err2:
err1:
	kfree(c);
err0:
	return ret;
}

static int __devexit remove(struct platform_device *pdev)
{
	struct tmpa900_ofd *c = platform_get_drvdata(pdev);

	ofd_disable();

	platform_set_drvdata(pdev, NULL);

	iounmap(c->regs);

	kfree(c);

	return 0;
}

static struct platform_driver tmpa900_ofd_driver = {
	.probe = probe,
	.remove = __devexit_p(remove),
	.driver = {
		.name  = "tmpa900-ofd",
		.owner = THIS_MODULE,
	},
};

static int __init tmpa900_ofd_init(void)
{
	return platform_driver_register(&tmpa900_ofd_driver);
}

static void __exit tmpa900_ofd_exit(void)
{
	platform_driver_unregister(&tmpa900_ofd_driver);
}

module_init(tmpa900_ofd_init);
module_exit(tmpa900_ofd_exit);

MODULE_AUTHOR("Thomas Haase <Thomas.Haase@web.de>");
MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("OFD driver for TMPA900");
MODULE_LICENSE("GPL");
