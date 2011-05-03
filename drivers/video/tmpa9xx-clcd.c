/*
 * TMPA9xx CLCD driver
 *
 * Copyright (c) 2011 Michael Hunold (michael@mihu.de)
 *
 * Initial code extracted from arch/arm/mach-tmpa9xx/tmpa9xx-generic.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>

#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>
#include <linux/amba/clcd.h>

#include <asm/uaccess.h>

#include <mach/platform.h>
#include <mach/lcdda.h>

#define ARM_CLCD_PERIPH_ID 0x00041110

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
	int irq;

	int blit_status;
	wait_queue_head_t blit_wait;

	void *mem;
	unsigned long len;
	dma_addr_t dma;
};

struct tmpa9xx_clcd
{
	struct clcd_panel panel;
	struct mutex mutex;
	int lcd_power;
	int lcd_reset;
};

struct tmpa9xx_lcdda g_tmpa9xx_lcdda;
struct tmpa9xx_clcd *g_tmpa9xx_clcd;

static int tmpa9xx_clcd_setup(struct clcd_fb *fb)
{
	struct tmpa9xx_clcd *c = g_tmpa9xx_clcd;
	unsigned long framesize;
	dma_addr_t dma;
	void *mem;
	unsigned long len;

	/* add additional memory for data and round up to page size */
	framesize = roundup(c->panel.mode.xres * c->panel.mode.yres * c->panel.bpp/8, PAGE_SIZE);
	len = roundup(3 * framesize + CONFIG_FB_TMPA9XX_CLCD_MEM * 1024, PAGE_SIZE);

	if (get_order(len) >= MAX_ORDER) {
		len = PAGE_SIZE * MAX_ORDER_NR_PAGES;
		dev_info(&fb->dev->dev, "limiting framebuffer size to %ld\n", len);
	}

	fb->panel = &c->panel;

	mem = dma_alloc_writecombine(&fb->dev->dev, len, &dma, GFP_KERNEL);
	if (!mem) {
		dev_err(&fb->dev->dev, "dma_alloc_writecombine() @ size %ld failed\n", len);
		return -ENOMEM;
	}

	fb->fb.screen_base    = mem;
	fb->fb.fix.smem_start = dma;
	fb->fb.fix.smem_len   = len;

	return 0;
}

static int tmpa9xx_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
	return dma_mmap_writecombine(&fb->dev->dev, vma,
			fb->fb.screen_base,
 			fb->fb.fix.smem_start,
			fb->fb.fix.smem_len);
}

static void tmpa9xx_clcd_remove(struct clcd_fb *fb)
{
	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len, fb->fb.screen_base, fb->fb.fix.smem_start);
}

int tmpa9xx_clcd_check(struct clcd_fb *fb, struct fb_var_screeninfo *var)
{
#define CHECK(e) (var->e != fb->fb.var.e)
	if (fb->panel->fixedtimings &&
	   (CHECK(xres) 		||
	    CHECK(yres) 		||
	    CHECK(bits_per_pixel)	||
	    CHECK(pixclock)		||
	    CHECK(left_margin)		||
	    CHECK(right_margin)		||
	    CHECK(upper_margin)		||
	    CHECK(lower_margin)		||
	    CHECK(hsync_len)		||
	    CHECK(vsync_len)		||
	    CHECK(sync))) {
	       return -EINVAL;
	}
#undef CHECK

	var->nonstd = 0;
	var->accel_flags = 0;

	return 0;
}

#ifndef CONFIG_FB_TMPA9XX_CLCD_NO_LCDDA
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

static int lcdda_probe(struct platform_device *pdev)
{
	struct tmpa9xx_lcdda *l = &g_tmpa9xx_lcdda;
	struct resource	*res;
	int ret;

	memset(l, 0, sizeof(*l));

	l->len = 32;
	l->mem = dma_alloc_writecombine(&pdev->dev, l->len, &l->dma, GFP_KERNEL);
	if (!l->mem) {
		dev_err(&pdev->dev, "dma_alloc_writecombine() @ size %ld failed\n", l->len);
		ret = -ENOMEM;
		goto err0;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
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

	l->irq = platform_get_irq(pdev, 1);
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

	l->dev = &pdev->dev;

	init_waitqueue_head(&l->blit_wait);

	return 0;

err4:
err3:
	iounmap(l->regs);
err2:
err1:
	dma_free_writecombine(&pdev->dev, l->len, l->mem, l->dma);
err0:
	return ret;
	return 0;
}

static void lcdda_remove(struct platform_device *pdev)
{
	struct tmpa9xx_lcdda *l = &g_tmpa9xx_lcdda;

	disable_irq(l->irq);

	lcdda_reset(l);

	free_irq(l->irq, l);

	iounmap(l->regs);

	dma_free_writecombine(&pdev->dev, l->len, l->mem, l->dma);

}

static int lcdda_blit_fct(struct tmpa9xx_clcd *c, struct tmpa9xx_lcdda *l, struct tmpa9xx_blit *params)
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

	mutex_lock(&c->mutex);

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

	mutex_unlock(&c->mutex);
 	return 0;

out:
	mutex_unlock(&c->mutex);
 	return -1;
}

static int lcdda_frect_fct(struct tmpa9xx_clcd *c, struct tmpa9xx_lcdda *l, struct tmpa9xx_frect *params)
{
	int dst_diff_pitch = (params->pitch - ((params->w-1) * params->bpp));
	int mode = BLIT_MODE_NORMAL;
	uint32_t val;
	int bpp;

	mutex_lock(&c->mutex);

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

	mutex_unlock(&c->mutex);
 	return 0;

out:
	mutex_unlock(&c->mutex);
 	return -1;
}

static int tmpa9xx_frect_fct(struct clcd_fb *fb, unsigned long arg)
{
	struct tmpa9xx_lcdda *l = &g_tmpa9xx_lcdda;
	struct tmpa9xx_clcd *c = g_tmpa9xx_clcd;
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

	ret = lcdda_frect_fct(c, l, &params);
	if (ret) {
		dev_err(l->dev, "lcdda_frect_fct() failed\n");
		return -EBUSY;
	}

	return 0;
}

static int tmpa9xx_blit_fct(struct clcd_fb *fb, unsigned long arg)
{
	struct tmpa9xx_lcdda *l = &g_tmpa9xx_lcdda;
	struct tmpa9xx_clcd *c = g_tmpa9xx_clcd;
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

	ret = lcdda_blit_fct(c, l, &params);
	if (ret) {
		dev_err(l->dev, "lcdda_blit_fct() failed\n");
		return -EBUSY;
	}

	return 0;
}

#else
static int lcdda_probe(struct platform_device *pdev)
{
	return 0;
}

static void lcdda_remove(struct platform_device *pdev)
{
}
#endif

static void tmpa9xx_clcd_decode(struct clcd_fb *fb, struct clcd_regs *regs)
{
	clcdfb_decode(fb, regs);
}

static int tmpa9xx_clcd_ioctl(struct clcd_fb *fb, unsigned int cmd, unsigned long arg)
{
	int ret = -ENOIOCTLCMD;

#ifndef CONFIG_FB_TMPA9XX_CLCD_NO_LCDDA
	if (cmd == TMPA9XX_BLIT)
		return tmpa9xx_blit_fct(fb, arg);

	if (cmd == TMPA9XX_FRECT)
		return tmpa9xx_frect_fct(fb, arg);
#endif

	return ret;
}

static void tmpa9xx_clcd_enable(struct clcd_fb *fb)
{
	struct tmpa9xx_clcd *c = g_tmpa9xx_clcd;

	if (c->lcd_power)
		gpio_set_value(c->lcd_power, 1);

	if (c->lcd_reset)
		gpio_set_value(c->lcd_reset, 1);
}

static void tmpa9xx_clcd_disable(struct clcd_fb *fb)
{
	struct tmpa9xx_clcd *c = g_tmpa9xx_clcd;

	if (c->lcd_reset)
		gpio_set_value(c->lcd_reset, 0);

	if (c->lcd_power)
		gpio_set_value(c->lcd_power, 0);
}

static struct clcd_board clcd_platform_data = {
	.name		= "tmpa9xx fb",
	.enable		= tmpa9xx_clcd_enable,
	.disable	= tmpa9xx_clcd_disable,
	.check		= tmpa9xx_clcd_check,
	.decode		= tmpa9xx_clcd_decode,
	.setup		= tmpa9xx_clcd_setup,
	.mmap		= tmpa9xx_clcd_mmap,
	.ioctl		= tmpa9xx_clcd_ioctl,
	.remove		= tmpa9xx_clcd_remove,
};

static int setup_display(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_clcd *c;
	char *options;
	unsigned int sync   = 0;
	unsigned int timer2 = 0;
	unsigned int videoparams[4];
	int i;
	int ret;

	struct tmpa9xx_panel_ts_info *panels = dev->platform_data;
	BUG_ON(!panels[0].panel);

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c) {
		dev_err(&pdev->dev, "kzalloc() failed\n");
		return -1;
	}
	g_tmpa9xx_clcd = c;

	mutex_init(&c->mutex);

	fb_get_options("tmpa9xxfb", &options);

	if (!options) {
		/* no option on the command line give, use first panel
		defined in platform_data */
		dev_info(&pdev->dev, "using default panel '%s'\n", panels[0].panel->mode.name);
		c->panel = *panels[0].panel;
		tmpa9xx_ts_rate = panels[0].rate;
		tmpa9xx_ts_fuzz = panels[0].fuzz;
		c->lcd_power = panels[0].lcd_power;
		c->lcd_reset = panels[0].lcd_reset;
		goto out;
	}

	/* first try to find panel with string match */
	for (i = 0;;i++) {
		struct tmpa9xx_panel_ts_info *p = &panels[i];

		if (!p->panel)
			break;

		ret = strcmp(p->panel->mode.name, options);
		if (!ret) {
			dev_info(&pdev->dev, "using configured panel '%s'\n", p->panel->mode.name);
			c->panel = *p->panel;
			tmpa9xx_ts_rate = p->rate;
			tmpa9xx_ts_fuzz = p->fuzz;
			c->lcd_power = p->lcd_power;
			c->lcd_reset = p->lcd_reset;
			goto out;
		}
	}

	/* now try old method */
	ret = sscanf(options, "%08x:%08x:%08x:%08x",
		&videoparams[0], &videoparams[1], &videoparams[2], &videoparams[3]);
	if (ret != 4) {
		dev_err(&pdev->dev, "sscanf() @ tmpa9xxfb options failed\n");
		kfree(c);
		return -1;
	}

	dev_info(&pdev->dev, "options from cmdline: \n" \
			 "LCDTiming0: 0x%08x\nLCDTiming1: 0x%08x\nLCDTiming2: 0x%08x\nLCDControl: 0x%08x\n",
			 videoparams[0],videoparams[1],videoparams[2],videoparams[3]);

	/* all panels using the command line passing option are RGB panels,
	so make sure to not to set the BGR flag */
	c->panel.cntl = CNTL_LCDTFT | CNTL_WATERMARK;

	c->panel.mode.name	= "tmpa9xx-panel";
	c->panel.mode.refresh	= 30; /* unused */
	c->panel.mode.pixclock	= 6000000;

	c->panel.mode.xres		= (((videoparams[0]>>2)  & 0x3f)  + 1) * 16;
	c->panel.mode.left_margin	= (  videoparams[0]>>24)	  + 1;
	c->panel.mode.right_margin	= (( videoparams[0]>>16) & 0xff)  + 1;
	c->panel.mode.hsync_len		= (( videoparams[0]>>8)  & 0xff)  + 1;

	c->panel.mode.yres		= ( videoparams[1]       & 0x3ff)  + 1;
	c->panel.mode.upper_margin	= ( videoparams[1]>>24)	       ;
	c->panel.mode.lower_margin	= ((videoparams[1]>>16)  & 0xff)     ;
	c->panel.mode.vsync_len		= ((videoparams[1]>>10)  & 0x1f)  + 1;

	c->panel.bpp       =  1 << ((videoparams[3]>>1) & 0x07);
	c->panel.width     = -1;
	c->panel.height    = -1;
	c->panel.grayscale = 0;

	if (!(videoparams[2] & (1<<11)))
		sync |= FB_SYNC_VERT_HIGH_ACT;
	if (!(videoparams[2] & (1<<12)))
		sync |= FB_SYNC_HOR_HIGH_ACT;
	if ((videoparams[2] & (1<<13)))
		timer2 |= TIM2_IPC;

	timer2 |= (videoparams[2]) & 0x1f;

	c->panel.mode.sync = sync;
	c->panel.tim2      = timer2;

	tmpa9xx_ts_rate = 200;
	tmpa9xx_ts_fuzz = 16;

	c->lcd_power = 97; /* port m1 */
	c->lcd_reset = 96; /* port m1 */

out:
	/* set some default values */
	c->panel.width = -1;
	c->panel.height = -1;
	c->panel.grayscale = 0;
	c->panel.fixedtimings = 1;

#ifdef DEBUG
	dev_info(&pdev->dev, "#define PIX_CLOCK_DIVIDER_xxx %d\n", c->panel.tim2 & ~(TIM2_IPC));
	dev_info(&pdev->dev, "static struct clcd_panel xxx = {\n");
	dev_info(&pdev->dev, "\t.mode = {\n");
	dev_info(&pdev->dev, "\t\t.name\t\t= \"xxx\",\n");
	dev_info(&pdev->dev, "\t\t.xres\t\t= %d,\n", c->panel.mode.xres);
	dev_info(&pdev->dev, "\t\t.yres\t\t= %d,\n", c->panel.mode.yres);
	dev_info(&pdev->dev, "\t\t.pixclock\t= HCLK/PIX_CLOCK_DIVIDER_xxx,\n");
	dev_info(&pdev->dev, "\t\t.left_margin\t= %d,\n", c->panel.mode.left_margin);
	dev_info(&pdev->dev, "\t\t.right_margin\t= %d,\n", c->panel.mode.right_margin);
	dev_info(&pdev->dev, "\t\t.upper_margin\t= %d,\n", c->panel.mode.upper_margin);
	dev_info(&pdev->dev, "\t\t.lower_margin\t= %d,\n", c->panel.mode.lower_margin);
	dev_info(&pdev->dev, "\t\t.hsync_len\t= %d,\n", c->panel.mode.hsync_len);
	dev_info(&pdev->dev, "\t\t.vsync_len\t= %d,\n", c->panel.mode.vsync_len);
	dev_info(&pdev->dev, "\t\t.sync\t\t= %s%s%d,\n",
		 c->panel.mode.sync & FB_SYNC_HOR_HIGH_ACT ? "FB_SYNC_HOR_HIGH_ACT | " : "",
		 c->panel.mode.sync & FB_SYNC_VERT_HIGH_ACT ? "FB_SYNC_VERT_HIGH_ACT | " : "",
		(c->panel.mode.sync & ~(FB_SYNC_HOR_HIGH_ACT|FB_SYNC_VERT_HIGH_ACT))
		);
	dev_info(&pdev->dev, "\t\t.vmode\t\t= %s%d,\n",
		 c->panel.mode.vmode & FB_VMODE_NONINTERLACED ? "FB_VMODE_NONINTERLACED |" : "",
		(c->panel.mode.vmode & ~(FB_VMODE_NONINTERLACED))
		);
	dev_info(&pdev->dev, "\t},\n");
	dev_info(&pdev->dev, "\t.width\t\t= %d,\n", c->panel.width);
	dev_info(&pdev->dev, "\t.height\t\t= %d,\n", c->panel.height);
	dev_info(&pdev->dev, "\t.tim2\t\t= %sPIX_CLOCK_DIVIDER_xxx,\n",
		 c->panel.tim2 & TIM2_IPC ? "TIM2_IPC | " : "");
	dev_info(&pdev->dev, "\t.cntl\t\t= %s%s%s%d,\n",
		 c->panel.cntl & CNTL_BGR ? "CNTL_BGR | " : "",
		 c->panel.cntl & CNTL_LCDTFT ? "CNTL_LCDTFT | " : "",
		 c->panel.cntl & CNTL_WATERMARK ? "CNTL_WATERMARK | " : "",
		(c->panel.cntl & ~(CNTL_BGR|CNTL_LCDTFT|CNTL_WATERMARK))
		);
	dev_info(&pdev->dev, "\t.bpp\t\t= %d,\n", c->panel.bpp);
	dev_info(&pdev->dev, "\t.grayscale\t= %d,\n", c->panel.grayscale);
	dev_info(&pdev->dev, "};\n");
#endif

	return 0;
}

static void shutdown_display(void)
{
	struct tmpa9xx_clcd *c = g_tmpa9xx_clcd;

	kfree(c);
}

static struct amba_device clcd_device =
{
	.dev = {
		.coherent_dma_mask = ~0,
		.init_name = "tmpa9xx-clcd",
		.platform_data = &clcd_platform_data,
	},
	.res = {
		.flags = IORESOURCE_MEM,
	},
	.dma_mask = ~0,
	.periphid = ARM_CLCD_PERIPH_ID,
};

static int lcd_gpio_probe(struct platform_device *pdev)
{
	struct tmpa9xx_clcd *c = g_tmpa9xx_clcd;
	int ret;

	if (c->lcd_power) {
		ret = gpio_request(c->lcd_power, "LCD power");
		if (ret) {
			dev_err(&pdev->dev, "gpio_request() @ power failed, gpio %d\n", c->lcd_power);
			ret = -EINVAL;
			goto err0;
		}
		/* keep disabled */
		gpio_direction_output(c->lcd_power, 0);
	}

	if (c->lcd_reset) {
		ret = gpio_request(c->lcd_reset, "LCD reset");
		if (ret) {
			dev_err(&pdev->dev, "gpio_request() @ reset failed, gpio %d\n", c->lcd_reset);
			ret = -EINVAL;
			goto err1;
		}
		/* keep in reset, low active */
		gpio_direction_output(c->lcd_reset, 0);
	}

	dev_info(&pdev->dev, "lcd_power @ %d, lcd_reset @ %d\n", c->lcd_power, c->lcd_reset);
	return 0;
err1:
err0:
	return ret;
}

static int lcd_gpio_remove(struct platform_device *pdev)
{
	struct tmpa9xx_clcd *c = g_tmpa9xx_clcd;

	if (c->lcd_reset)
		gpio_free(c->lcd_reset);

	if (c->lcd_power)
		gpio_free(c->lcd_power);

	return 0;
}

static int __devinit probe(struct platform_device *pdev)
{
	struct amba_device *d;
	struct resource	*res;
	int ret;

	ret = setup_display(pdev);
	if (ret) {
		dev_err(&pdev->dev, "setup_display() failed\n");
		ret = -ENODEV;
		goto err0;
	}

	ret = lcd_gpio_probe(pdev);
	if (ret) {
		dev_err(&pdev->dev, "lcd_gpio_probe() failed\n");
		ret = -ENODEV;
		goto err1;
	}

	d = kzalloc(sizeof(struct amba_device), GFP_KERNEL);
	if (!d) {
		dev_err(&pdev->dev, "kzalloc() failed\n");
		ret = -ENOMEM;
		goto err3;
	}

	/* copy over template */
	*d = clcd_device;

	/* now fill up the rest */
	d->dev.parent = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "platform_get_resource() failed @ IORESOURCE_MEM\n");
		ret = -ENODEV;
		goto err4;
	}

	d->res.start = res->start;
	d->res.end = res->end;

	d->irq[0] = platform_get_irq(pdev, 0);
	if (d->irq[0] <= 0) {
		dev_err(&pdev->dev, "platform_get_irq() failed\n");
		ret = -ENODEV;
		goto err5;
	}

	ret = lcdda_probe(pdev);
	if (ret) {
		dev_err(&pdev->dev, "lcdda_probe() failed\n");
		ret = -ENODEV;
		goto err6;
	}

	platform_set_drvdata(pdev, d);

	ret = amba_device_register(d, &pdev->resource[0]);
	if (ret) {
		dev_err(&pdev->dev, "amba_device_register() failed\n");
		ret = -ENODEV;
		goto err7;
	}

	return 0;

err7:
	platform_set_drvdata(pdev, NULL);
	lcdda_remove(pdev);
err6:
err5:
err4:
	kfree(d);
err3:
	lcd_gpio_remove(pdev);
err1:
	shutdown_display();
err0:
	return ret;
}

static int __devexit remove(struct platform_device *pdev)
{
	struct amba_device *d = platform_get_drvdata(pdev);

	amba_device_unregister(d);

	lcdda_remove(pdev);

	lcd_gpio_remove(pdev);

	shutdown_display();

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver tmpa9xx_clcd_driver = {
	.probe = probe,
	.remove = __devexit_p(remove),
	.driver = {
		.name  = "tmpa9xx-clcd",
		.owner = THIS_MODULE,
	},
};

static int __init tmpa9xx_clcd_init(void)
{
	return platform_driver_register(&tmpa9xx_clcd_driver);
}

static void __exit tmpa9xx_clcd_exit(void)
{
	platform_driver_unregister(&tmpa9xx_clcd_driver);
}

module_init(tmpa9xx_clcd_init);
module_exit(tmpa9xx_clcd_exit);

MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("CLCD driver for TMPA9xx");
MODULE_LICENSE("GPL");
