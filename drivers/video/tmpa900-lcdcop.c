/*
 * LCD Controller Option Function (COP) driver for TMPA900
 *
 * Copyright (c) 2011 Michael Hunold (michael@mihu.de)
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

struct tmpa900_lcdcop
{
	void __iomem *regs;
};

#define STN64CR		(0x00)

#define G64_8BIT	(1 << 1)

#define lcdcop_writel(b, o, v)	writel(v, b->regs + o)
#define lcdcop_readl(b, o)	readl(b->regs + o)

static int __devinit probe(struct platform_device *pdev)
{
	struct tmpa900_lcdcop *c;
	struct resource *res;
	int ret;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c) {
		dev_err(&pdev->dev, "kzalloc() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

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

	/* currently this driver just supports one single option, which is
	changing the pin multiplexing of the LCD controller when a 16-bit TFT
	is attached. this is described in chapter 3.19.4 of the technical
	reference manual. when this driver is loaded, this means this option
	needs to be set. */

	lcdcop_writel(c, STN64CR, G64_8BIT);

        dev_info(&pdev->dev, "default configuration activated\n");

	return 0;

err2:
err1:
	kfree(c);
err0:
	return ret;
}

static int __devexit remove(struct platform_device *pdev)
{
	struct tmpa900_lcdcop *c = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	iounmap(c->regs);

	kfree(c);

	return 0;
}

static struct platform_driver tmpa900_lcdcop_driver = {
	.probe = probe,
	.remove = __devexit_p(remove),
	.driver = {
		.name  = "tmpa900-lcdcop",
		.owner = THIS_MODULE,
	},
};

static int __init tmpa900_lcdcop_init(void)
{
	return platform_driver_register(&tmpa900_lcdcop_driver);
}

static void __exit tmpa900_lcdcop_exit(void)
{
	platform_driver_unregister(&tmpa900_lcdcop_driver);
}

module_init(tmpa900_lcdcop_init);
module_exit(tmpa900_lcdcop_exit);

MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("LCD COP driver for TMPA900");
MODULE_LICENSE("GPL");
