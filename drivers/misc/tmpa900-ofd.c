/*
 *  Oscillation frequency detection (OFD) driver for TMPA900
 *
 *  Copyright (c) 2010 Thomas Haase <Thomas.Haase@web.de>
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
#include <linux/clk.h>
#include <linux/err.h>

struct tmpa900_ofd
{
	void __iomem *regs;
	struct device *dev;
	struct clk *clk;
};

#define CLKSCR1     (0x00)
#define CLKSCR2     (0x04)
#define CLKSCR3     (0x08)
#define CLKSMN      (0x10)
#define CLKSMX      (0x20)

#define ofd_writel(b, o, v)	writel(v, b->regs + o)
#define ofd_readl(b, o)		readl(b->regs + o)

#define CLK_WRITE_ENABLE	0xf9 /* Enable writing to the clock registers */
#define CLK_WRITE_DISABLE	0x06 /* Disable writing to the clock registers */

#define CLK_S_ENABLE	0xe4 /* Enable OFD operation */
#define CLK_S_DISABLE	0x00 /* Disable OFD operation */

#define OFD_RESET_ENABLE	(1<<1) /* Enable OFD reset */
#define OFD_CLEAR_CLKSF		(1<<0) /* Clear High speed oscillation frequency detection flag */

static int ofd_enable(struct tmpa900_ofd *o, int deviation)
{
	uint32_t val;
	uint32_t fosch;
	uint32_t clksmn;
	uint32_t clksmx;

	/* see trm chapter 3.28.5, programming example */

	/* Enable writing to the OFD registers */
	ofd_writel(o, CLKSCR1, CLK_WRITE_ENABLE);
	/* Disable OFD operation */
	ofd_writel(o, CLKSCR2, CLK_S_DISABLE);
	udelay(1);

	ofd_writel(o, CLKSCR3, OFD_RESET_ENABLE | OFD_CLEAR_CLKSF);

	while(1) {
		val = ofd_readl(o, CLKSCR3);
		if ((val & OFD_RESET_ENABLE))
			break;
	}

	fosch = clk_get_rate(o->clk);

#define FIXED_DIVIDER (32768UL*4UL)
	clksmn = (((fosch * (100UL - deviation)) / 100) + FIXED_DIVIDER/2) / FIXED_DIVIDER;
	clksmx = (((fosch * (100UL + deviation)) / 100) + FIXED_DIVIDER/2) / FIXED_DIVIDER;
	ofd_writel(o, CLKSMN, clksmn);
	ofd_writel(o, CLKSMX, clksmx);

	/* Enable OFD operation */
	ofd_writel(o, CLKSCR2, CLK_S_ENABLE);
	udelay(1);

	/* Disable writing to the OFD registers */
	ofd_writel(o, CLKSCR1, CLK_WRITE_DISABLE);

	dev_dbg(o->dev, "%s(): ok, fosch %d, min %d, max %d\n", __func__, fosch, clksmn, clksmx);

	return 0;
}

static int ofd_disable(struct tmpa900_ofd *o)
{
	uint32_t val;

	/* Enable writing to the OFD registers */
	ofd_writel(o, CLKSCR1, CLK_WRITE_ENABLE);
	/* Disable OFD operation */
	ofd_writel(o, CLKSCR2, CLK_S_DISABLE);
	udelay(1);

	ofd_writel(o, CLKSCR3, OFD_CLEAR_CLKSF);

	while(1) {
		val = ofd_readl(o, CLKSCR3);
		if (!(val & OFD_RESET_ENABLE))
			break;
	}

	/* Disable writing to the OFD registers */
	ofd_writel(o, CLKSCR1, CLK_WRITE_DISABLE);

	dev_dbg(o->dev, "%s(): ok\n", __func__);

	return 0;
}

static int __devinit probe(struct platform_device *pdev)
{
	struct tmpa900_ofd *o;
	struct resource *res;
	int ret;

	o = kzalloc(sizeof(*o), GFP_KERNEL);
	if (!o) {
		dev_err(&pdev->dev, "kzalloc() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	o->dev = &pdev->dev;

	o->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(o->clk)) {
		dev_err(o->dev, "clk_get() failed\n");
		ret = -ENOENT;
		goto err1;
	}

	ret = clk_enable(o->clk);
	if (ret) {
		dev_err(o->dev, "clk_enable() failed\n");
		ret = -ENOENT;
		goto err2;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(o->dev, "platform_get_resource() failed @ IORESOURCE_MEM\n");
		ret = -ENODEV;
		goto err3;
	}

	o->regs = ioremap(res->start, resource_size(res));
	if (!o->regs) {
		dev_err(o->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto err4;
	}

	platform_set_drvdata(pdev, o);

	ret = ofd_enable(o, 10);
	if (ret) {
		dev_err(o->dev, "ofd_enable() failed\n");
		ret = -ENODEV;
		goto err5;
	}

	dev_info(o->dev, "ready\n");

	return 0;

err5:
	platform_set_drvdata(pdev, NULL);
	iounmap(o->regs);
err4:
err3:
	clk_disable(o->clk);
err2:
	clk_put(o->clk);
err1:
	kfree(o);
err0:
	return ret;
}

static int __devexit remove(struct platform_device *pdev)
{
	struct tmpa900_ofd *o = platform_get_drvdata(pdev);

	ofd_disable(o);

	platform_set_drvdata(pdev, NULL);

	iounmap(o->regs);

	clk_disable(o->clk);

	clk_put(o->clk);

	kfree(o);

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
