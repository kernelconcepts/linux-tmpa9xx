/*
 * TMPA9xx LCDDA driver
 *
 * Copyright (c) 2011 Michael Hunold (michael@mihu.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <asm/uaccess.h>

#include <mach/platform.h>
#include <mach/lcdda.h>

#define BLIT_MODE_NORMAL 0x00
#define BLIT_MODE_BLEND  0x10

#define BLIT_START	(1<<30)
#define BLIT_ISR	((1<<16)|(1<<17))
#define BLIT_ISR_MASK	(~((1<<20)|(1<<21)))
#define BLIT_DMA_MASK	(~0x00008800)
#define BLIT_FIRST_LINE	(0x0800)
#define BLIT_LAST_LINE	(0x8000)
#define BLIT_X_PAD	(0x0200)

#define ISR_ERROR	(1<<21)
#define ISR_LINE	(1<<20)

#define LCDDA_LDACR0		(0x00)
#define LCDDA_LDADRSRC1		(0x04)
#define LCDDA_LDADRSRC0		(0x08)
#define LCDDA_LDAFCPSRC1	(0x0c)
#define LCDDA_LDAEFCPSRC1	(0x10)
#define LCDDA_LDADVSRC1		(0x14)
#define LCDDA_LDACR2		(0x18)
#define LCDDA_LDADXDST		(0x1c)
#define LCDDA_LDADYDST		(0x20)
#define LCDDA_LDASSIZE		(0x24)
#define LCDDA_LDADSIZE		(0x28)
#define LCDDA_LDAS0AD		(0x2c)
#define LCDDA_LDADAD		(0x30)
#define LCDDA_LDACR1		(0x34)
#define LCDDA_LDADVSRC0		(0x38)

#define lcdda_writel(b, o, v)	writel(v, b->regs + o)
#define lcdda_readl(b, o)	readl(b->regs + o)

struct tmpa9xx_lcdda
{
	void __iomem *regs;
	struct device *dev;
	struct mutex mutex;
	int irq;

	int blit_status;
	wait_queue_head_t blit_wait;

	void *mem;
	unsigned long len;
	dma_addr_t dma;
};

static irqreturn_t lcdda_interrupt(int irq, void *data)
{
	struct tmpa9xx_lcdda *l = data;
	unsigned long status;

	if (l->irq != irq)
		return IRQ_NONE;

	status = lcdda_readl(l, LCDDA_LDACR0);

	if (!(status & (ISR_ERROR|ISR_LINE)))
		return IRQ_HANDLED;

	/* clear interrupt flags */
	lcdda_writel(l, LCDDA_LDACR0, (status & ~(ISR_ERROR|ISR_LINE)));

	/* done with operation, reset hardware */
	lcdda_writel(l, LCDDA_LDACR1, 0x80000000);

	l->blit_status = 0;
	wake_up(&l->blit_wait);

	return IRQ_HANDLED;
}

static void lcdda_reset(struct tmpa9xx_lcdda *l)
{
	/* reset hardware */
	lcdda_writel(l, LCDDA_LDACR1, 0x80000000);

	l->blit_status = 0;
}

static int lcdda_blit_fct(struct tmpa9xx_lcdda *l, struct tmpa9xx_blit *params)
{
	int src_diff_pitch = (params->src_pitch - ((params->w-1) * params->bpp));
	int dst_diff_pitch = (params->dst_pitch - ((params->w-1) * params->bpp));
	unsigned long dst = params->dst;
	int dx_dec = 0;
	int dy_dec = 0;
	int mode = BLIT_MODE_NORMAL;
	int bpp;
	int dx_dst = params->bpp;
	int dy_dst = dst_diff_pitch;
	uint32_t val;

	mutex_lock(&l->mutex);

	BUG_ON(l->blit_status);

	if (params->bpp == 2)
		bpp = 1;
	else if (params->bpp == 4)
		bpp = 0;
	else {
		dev_err(l->dev, "bpp %d invalid\n", params->bpp);
		goto out;
	}

	if (params->rotation == 180) {
		dst += (params->w - 1)*params->bpp + (params->dst_pitch * (params->h - 1));
		dx_dec = 1;
		dy_dec = 1;
	} else if (params->rotation == 90) {
		dst += (params->h - 1)*params->bpp;
		dy_dec = 1;
		dx_dst = params->dst_pitch;
		dy_dst = (params->dst_pitch * (params->w-1)) + params->bpp;
	}

	/* bleding is broken atm */
	BUG_ON(params->blend);
	if (params->blend) {
		mode = BLIT_MODE_BLEND;
	}

	lcdda_writel(l, LCDDA_LDACR0, (bpp << 10) | (params->src >> 24));
	lcdda_writel(l, LCDDA_LDAFCPSRC1,  0x0);
	lcdda_writel(l, LCDDA_LDAEFCPSRC1, 0x0);
	if (params->blend) {
		int r0, r1, g0, g1, b0, b1;
		r0 = r1 = g0 = g1 = b0 = b1 = 0x3f;
		lcdda_writel(l, LCDDA_LDADVSRC0, (1 << 31));
		lcdda_writel(l, LCDDA_LDADRSRC1, (b1 << 16) | (g1 << 8) | r1);
		lcdda_writel(l, LCDDA_LDADRSRC0, (b0 << 16) | (g0 << 8) | r0);
	} else {
		lcdda_writel(l, LCDDA_LDADVSRC0, 0x0);
		lcdda_writel(l, LCDDA_LDADRSRC1, 0x0);
		lcdda_writel(l, LCDDA_LDADRSRC0, 0x0);
	}
	lcdda_writel(l, LCDDA_LDADVSRC1, (src_diff_pitch << 6) | (params->bpp << 0));
	lcdda_writel(l, LCDDA_LDACR2,    0x00000000);
	lcdda_writel(l, LCDDA_LDADXDST,  (dx_dec << 24) | dx_dst);
	lcdda_writel(l, LCDDA_LDADYDST,  (dy_dec << 24) | dy_dst);
	lcdda_writel(l, LCDDA_LDASSIZE,  ((params->h-1) << 12) | ((params->w-1) << 0));
	lcdda_writel(l, LCDDA_LDADSIZE,	 ((params->w-1) << 0));
	lcdda_writel(l, LCDDA_LDAS0AD,   0x00000000);
	lcdda_writel(l, LCDDA_LDADAD,	 dst);
	lcdda_writel(l, LCDDA_LDACR1,    (params->src & 0x00ffffff) | (mode << 24));

	l->blit_status = 1;

	/* enable interrupts */
	val = lcdda_readl(l, LCDDA_LDACR0) | BLIT_ISR;
	lcdda_writel(l, LCDDA_LDACR0, val);

	/* start operation */
	val = lcdda_readl(l, LCDDA_LDACR1) | BLIT_START;
	lcdda_writel(l, LCDDA_LDACR1, val);

	wait_event(l->blit_wait, !l->blit_status);

	mutex_unlock(&l->mutex);
 	return 0;

out:
	mutex_unlock(&l->mutex);
 	return -1;
}

static int lcdda_frect_fct(struct tmpa9xx_lcdda *l, struct tmpa9xx_frect *params)
{
	int dst_diff_pitch = (params->pitch - ((params->w-1) * params->bpp));
	int mode = BLIT_MODE_NORMAL;
	uint32_t val;
	int bpp;

	mutex_lock(&l->mutex);

	BUG_ON(l->blit_status);

	if (params->bpp == 2) {
		unsigned short *ptr = l->mem;
		*ptr++ = params->color;
		*ptr++ = params->color;
		*ptr++ = params->color;
		*ptr++ = params->color;
		*ptr++ = params->color;
		*ptr++ = params->color;
		*ptr++ = params->color;
		*ptr++ = params->color;
		bpp = 1;
	} else if (params->bpp == 4) {
		unsigned long *ptr = l->mem;
		*ptr++ = params->color;
		*ptr++ = params->color;
		*ptr++ = params->color;
		*ptr++ = params->color;
		*ptr++ = params->color;
		*ptr++ = params->color;
		*ptr++ = params->color;
		*ptr++ = params->color;
		bpp = 0;
	} else {
		dev_err(l->dev, "bpp %d invalid\n", params->bpp);
		goto out;
	}

	lcdda_writel(l, LCDDA_LDACR0, (bpp << 10) | (l->dma >> 24));
	lcdda_writel(l, LCDDA_LDAFCPSRC1,  0x0);
	lcdda_writel(l, LCDDA_LDAEFCPSRC1, 0x0);
	lcdda_writel(l, LCDDA_LDADVSRC1, 0);
	lcdda_writel(l, LCDDA_LDACR2,    0x00000000);
	lcdda_writel(l, LCDDA_LDADXDST,  (0 << 24) | params->bpp);
	lcdda_writel(l, LCDDA_LDADYDST,  (0 << 24) | dst_diff_pitch);
	lcdda_writel(l, LCDDA_LDASSIZE,  ((params->h-1) << 12) | ((params->w-1) << 0));
	lcdda_writel(l, LCDDA_LDADSIZE,	 ((params->w-1) << 0));
	lcdda_writel(l, LCDDA_LDAS0AD,   0x00000000);
	lcdda_writel(l, LCDDA_LDADAD,	 params->dst);
	lcdda_writel(l, LCDDA_LDACR1,    (l->dma & 0x00ffffff) | (mode << 24));

	l->blit_status = 1;

	/* enable interrupts */
	val = lcdda_readl(l, LCDDA_LDACR0) | BLIT_ISR;
	lcdda_writel(l, LCDDA_LDACR0, val);

	/* start operation */
	val = lcdda_readl(l, LCDDA_LDACR1) | BLIT_START;
	lcdda_writel(l, LCDDA_LDACR1, val);

	wait_event(l->blit_wait, !l->blit_status);

	mutex_unlock(&l->mutex);
 	return 0;

out:
	mutex_unlock(&l->mutex);
 	return -1;
}

static int tmpa9xx_frect_fct(struct tmpa9xx_lcdda *l, unsigned long arg)
{
	struct tmpa9xx_frect params;
	int size;
	int ret;

	if (!access_ok(VERIFY_WRITE, (void *)arg, sizeof(struct tmpa9xx_frect))) {
		dev_err(l->dev, "access_ok() failed\n");
		return -EPROTO;
	}

	size = copy_from_user((void *)&params, (void *)arg, sizeof(struct tmpa9xx_frect));
	if (size) {
		dev_err(l->dev, "copy_from_user() failed\n");
		return -EBADE;
	}

	ret = lcdda_frect_fct(l, &params);
	if (ret) {
		dev_err(l->dev, "lcdda_frect_fct() failed\n");
		return -EBUSY;
	}

	return 0;
}

static int tmpa9xx_blit_fct(struct tmpa9xx_lcdda *l, unsigned long arg)
{
	struct tmpa9xx_blit params;
	int size;
	int ret;

	if (!access_ok(VERIFY_WRITE, (void *)arg, sizeof(struct tmpa9xx_blit))) {
		dev_err(l->dev, "access_ok() failed\n");
		return -EPROTO;
	}

	size = copy_from_user((void *)&params, (void *)arg, sizeof(struct tmpa9xx_blit));
	if (size) {
		dev_err(l->dev, "copy_from_user() failed\n");
		return -EBADE;
	}

	ret = lcdda_blit_fct(l, &params);
	if (ret) {
		dev_err(l->dev, "lcdda_blit_fct() failed\n");
		return -EBUSY;
	}

	return 0;
}


static int tmpa9xx_lcdda_ioctl(void *priv, unsigned int cmd, unsigned long arg)
{
	if (cmd == TMPA9XX_BLIT)
		return tmpa9xx_blit_fct(priv, arg);

	if (cmd == TMPA9XX_FRECT)
		return tmpa9xx_frect_fct(priv, arg);

        return -ENOIOCTLCMD;
}

static int __devinit probe(struct platform_device *pdev)
{
	struct tmpa9xx_lcdda *l;
	struct resource	*res;
	int ret;

	l = kzalloc(sizeof(*l), GFP_KERNEL);
	if (!l) {
		dev_err(&pdev->dev, "kzalloc() failed\n");
		return -ENOMEM;
	}

	mutex_init(&l->mutex);
	init_waitqueue_head(&l->blit_wait);
	l->dev = &pdev->dev;

	l->len = 32;
	l->mem = dma_alloc_writecombine(&pdev->dev, l->len, &l->dma, GFP_KERNEL);
	if (!l->mem) {
		dev_err(&pdev->dev, "dma_alloc_writecombine() @ size %ld failed\n", l->len);
		ret = -ENOMEM;
		goto err0;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "platform_get_resource() failed @ IORESOURCE_MEM\n");
		ret = -ENODEV;
		goto err1;
	}

	l->regs = ioremap(res->start, resource_size(res));
	if (!l->regs) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto err2;
	}

	lcdda_reset(l);

	l->irq = platform_get_irq(pdev, 0);
	if (l->irq <= 0) {
		dev_err(&pdev->dev, "platform_get_irq() failed\n");
		ret = -ENODEV;
		goto err3;
	}

	ret = request_irq(l->irq, lcdda_interrupt, IRQF_DISABLED, "tmpa9xx_lcdda", l);
	if (ret) {
		dev_err(&pdev->dev, "request_irq() failed\n");
		ret = -ENODEV;
		goto err4;
	}

	platform_set_drvdata(pdev, l);

	ret = tmpa9xx_clcd_register_ioctl(tmpa9xx_lcdda_ioctl, l);
	if (ret) {
		dev_err(&pdev->dev, "tmpa9xx_clcd_register_ioctl() failed\n");
		ret = -ENODEV;
		goto err5;
	}

	dev_info(l->dev, "ready\n");

	return 0;

err5:
	platform_set_drvdata(pdev, NULL);
	free_irq(l->irq, l);
err4:
err3:
	iounmap(l->regs);
err2:
err1:
	dma_free_writecombine(&pdev->dev, l->len, l->mem, l->dma);
err0:
	kfree(l);
	return ret;
}

static int __devexit remove(struct platform_device *pdev)
{
	struct tmpa9xx_lcdda *l = platform_get_drvdata(pdev);

	tmpa9xx_clcd_unregister_ioctl(l);

	platform_set_drvdata(pdev, NULL);

	disable_irq(l->irq);

	lcdda_reset(l);

	free_irq(l->irq, l);

	iounmap(l->regs);

	dma_free_writecombine(&pdev->dev, l->len, l->mem, l->dma);

	kfree(l);

	return 0;
}

static struct platform_driver tmpa9xx_lcdda_driver = {
	.probe = probe,
	.remove = __devexit_p(remove),
	.driver = {
		.name  = "tmpa9xx-lcdda",
		.owner = THIS_MODULE,
	},
};

static int __init tmpa9xx_lcdda_init(void)
{
	return platform_driver_register(&tmpa9xx_lcdda_driver);
}

static void __exit tmpa9xx_lcdda_exit(void)
{
	platform_driver_unregister(&tmpa9xx_lcdda_driver);
}

module_init(tmpa9xx_lcdda_init);
module_exit(tmpa9xx_lcdda_exit);

MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("LCDDA driver for TMPA9xx");
MODULE_LICENSE("GPL");
