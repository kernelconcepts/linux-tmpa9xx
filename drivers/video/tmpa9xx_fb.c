/*
 *  linux/drivers/video/tmpa9xx_fb.c -- TMPA9xx frame buffer driver
 *
 *	Copyright (C) 2008 bplan GmbH
 *      Copyright (C) 2010 Florian Boor <florian@kernelconcepts.de>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <mach/regs.h>

#include <asm/byteorder.h>
#include <linux/screen_info.h>

#include <video/tmpa9xx_fb.h>

/* Supported palette hacks */
enum {
	cmap_unknown,
};

struct hw_tmpa9xx_lcdc
{
	uint32_t LCDTiming0;	//			(LCDC_Base+0x00
	uint32_t LCDTiming1;	//			(LCDC_Base+0x04
	uint32_t LCDTiming2;	//			(LCDC_Base+0x08
	uint32_t LCDTiming3;	//			(LCDC_Base+0x0C
	uint32_t LCDUPBASE;	//			(LCDC_Base+0x10
	uint32_t LCDLPBASE;	//			(LCDC_Base+0x14
	uint32_t LCDIMSC;	//			(LCDC_Base+0x18
	uint32_t LCDControl;	//			(LCDC_Base+0x1C
	uint32_t LCDRIS;	//			(LCDC_Base+0x20
	uint32_t LCDMIS;	//			(LCDC_Base+0x24
	uint32_t LCDICR;	//			(LCDC_Base+0x28
	uint32_t LCDUPCURR;	//			(LCDC_Base+0x2C
	uint32_t LCDLPCURR;	//			(LCDC_Base+0x30
	uint32_t Rsd[0x1c0];	//			Reserved
	uint32_t LCDPalette;	//			(LCDC_Base+0x200
};

struct tmpa9xx_lcdc_par {
	volatile void __iomem *cmap_adr;
	volatile void __iomem *cmap_data;
	int cmap_type;
	int blanked;
	struct hw_tmpa9xx_lcdc *hw_tmpa9xx_lcdc;
};

struct tmpa9xx_lcdc_par default_par;

static void _init_it(struct hw_tmpa9xx_lcdc *hw_tmpa9xx_lcdc, uint32_t *LCDReg, int  width, int height)
{
	LCDDA_LDACR0 = 0x00; /* LCDDA functions off */
	LCDDA_LDACR1 = (1<<31); /* reset LCDDA */
    
	hw_tmpa9xx_lcdc->LCDControl= 0;

	hw_tmpa9xx_lcdc->LCDTiming0	= LCDReg[0];
	hw_tmpa9xx_lcdc->LCDTiming1	= LCDReg[1];
	hw_tmpa9xx_lcdc->LCDTiming2	= LCDReg[2];
	hw_tmpa9xx_lcdc->LCDTiming3	= LCDReg[3];

	hw_tmpa9xx_lcdc->LCDUPBASE	= 0;
	hw_tmpa9xx_lcdc->LCDLPBASE	= 0;

	hw_tmpa9xx_lcdc->LCDIMSC		= 0;
	hw_tmpa9xx_lcdc->LCDControl = LCDReg[4];
	barrier();
}

static void _setup_fb(struct hw_tmpa9xx_lcdc *hw_tmpa9xx_lcdc, uint32_t fb_addr)
{
	hw_tmpa9xx_lcdc->LCDUPBASE	= fb_addr;
	hw_tmpa9xx_lcdc->LCDLPBASE	= 0x00;
	udelay(200);
	hw_tmpa9xx_lcdc->LCDControl |= 0x1;
	barrier();
}

static void _stop_it(struct hw_tmpa9xx_lcdc *hw_tmpa9xx_lcdc)
{
	hw_tmpa9xx_lcdc->LCDControl = 0;
}

static int tmpa9xx_lcdc_setcolreg(unsigned regno, unsigned red, unsigned green,
			  unsigned blue, unsigned transp, struct fb_info *info)
{

	u32 rgba, *pal;
/*
	pr_debug("info=%p,tmpa9xx_lcdc_setcolreg=%p\n", info, tmpa9xx_lcdc_setcolreg);
	pr_debug("info=%p, bpp=%d. tmpa9xx_lcdc_setcolreg=%p, \n", info,
		 info->var.bits_per_pixel, tmpa9xx_lcdc_setcolreg,
		 info->pseudo_palette);
*/
	pal = info->pseudo_palette;

	if (regno >= 16)
		return -EINVAL;

#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)

	red = CNVT_TOHW(red, info->var.red.length);
	green = CNVT_TOHW(green, info->var.green.length);
	blue = CNVT_TOHW(blue, info->var.blue.length);
	transp = CNVT_TOHW(transp, info->var.transp.length);

#undef CNVT_TOHW

	rgba = (red << info->var.red.offset) |
	    (green << info->var.green.offset) |
	    (blue << info->var.blue.offset) |
	    (transp << info->var.transp.offset);

	pal[regno] = rgba;

	return 0;
}

static struct fb_ops tmpa9xx_lcdc_ops = {
	.owner = THIS_MODULE,
	.fb_setcolreg = tmpa9xx_lcdc_setcolreg,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static int __init tmpa9xx_lcdc_init_fb(
	struct platform_device *pdev,	
	const char *name, const char *full_name,
	int width, int height, int depth, 
	int pitch, uint32_t *LCDReg, void *lcdc_base)
{
	struct tmpa9xx_lcdc_par *par = &default_par;
	struct fb_fix_screeninfo *fix;
	struct fb_var_screeninfo *var;
	struct fb_info *info;
	dma_addr_t dma;
	u32 len;
	int ret;

    
	info = framebuffer_alloc(0, &pdev->dev);
	if (info == NULL) {
		return -ENOMEM;
	}

	fix = &info->fix;
	var = &info->var;

	strcpy(fix->id, name);
	strncat(fix->id, name, sizeof(fix->id) - sizeof("tmpa9xx_lcdc "));
	fix->id[sizeof(fix->id) - 1] = '\0';

	var->xres = var->xres_virtual = width;
	var->yres = var->yres_virtual = height;

	len = pitch * height;
	info->screen_base
		= dma_alloc_writecombine (&pdev->dev, len,
					  &dma, GFP_KERNEL);
	if (!info->screen_base) {
		printk(KERN_ERR "tmpa9xx_fb: unable to map framebuffer\n");
		return -ENOMEM;
	}

	fix->smem_start = dma;
	fix->smem_len = len;
    
	fix->line_length = pitch;
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->type_aux = 0;
    
	par->cmap_type = cmap_unknown;

	fix->visual = FB_VISUAL_TRUECOLOR;

	var->xoffset = var->yoffset = 0;
	switch (depth) {

	case 16:		/* RGB 565 */
		var->bits_per_pixel = 16;
		var->blue.offset = 11;
		var->blue.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->red.offset = 0;
		var->red.length = 5;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
		
	case 32:		/* RGB 888 */
		var->bits_per_pixel = 32;
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
		var->transp.offset = 24;
		var->transp.length = 8;

		break;
	}

	// Some defaults, not very important for the overall functionnality
	var->red.msb_right = var->green.msb_right = var->blue.msb_right =
	var->transp.msb_right = 0;
	var->grayscale = 0;
	var->nonstd = 0;
	var->activate = 0;
	var->height = var->width = -1;
	var->pixclock = 10000;
	var->left_margin = var->right_margin = 16;
	var->upper_margin = var->lower_margin = 16;
	var->hsync_len = var->vsync_len = 8;
	var->sync = 0;
	var->vmode = FB_VMODE_NONINTERLACED;

	info->fbops = &tmpa9xx_lcdc_ops;
	
	// Here real LCD hw init
	par->hw_tmpa9xx_lcdc = (void *)(lcdc_base);
	_init_it( par->hw_tmpa9xx_lcdc, LCDReg, width, height);
	_setup_fb( par->hw_tmpa9xx_lcdc, (uint32_t)dma);
	
	// ok
	info->par = par;
	info->pseudo_palette = (void *)(info + 1);
	info->flags = FBINFO_DEFAULT;

	info->pixmap.flags = FB_PIXMAP_IO;

	fb_alloc_cmap(&info->cmap, 256, 0);

	pr_debug("register_framebuffer(%p), var->bits_per_pixel=%d\n", info,
		 info->var.bits_per_pixel);

	// The final touch
	ret = register_framebuffer(info);
	if ( ret< 0) {
		kfree(info);
		return ret;
	}

	// Success :-)
	printk(KERN_INFO "fb%d: Toshiba TMPA9x0 Frame buffer device at 0x%08x (mapped @ %p)\n",
	       info->node, (unsigned int)dma,  info->screen_base);

	return 0;
}

/* parse options in form video=tmpa9xxfb:%08x:%08x:%08x */
int tmpa9xx_lcdc_parse_params(uint32_t *LCDReg, char *options)
{
    	unsigned int r0, r1, r2 , r3;
    	r3=0;

	if (sscanf(options, "%08x:%08x:%08x:%08x", &r0, &r1, &r2, &r3) != 4) {
		if (sscanf(options, "%08x:%08x:%08x", &r0, &r1, &r2) != 3) {
			printk(KERN_WARNING "tmpa9xxfb: Unable to parse options %s\n", options);
			return 0;
                }
	}
	printk(KERN_INFO "tmpa9xxfb: Options from cmdline: \n" \
	                 "LCDTiming0: 0x%08x\nLCDTiming1: 0x%08x\nLCDTiming2: 0x%08x\nLCDControl: 0x%08x\n", r0, r1, r2, r3);
	
	LCDReg[0] = r0;
	LCDReg[1] = r1;
	LCDReg[2] = r2;
        if (r3!=0)
		LCDReg[4] = r3;
    
	return 1;
}

#ifdef CONFIG_PM
static int tmpa9xx_lcdc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int tmpa9xx_lcdc_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define tmpa9xx_lcdc_suspend	NULL
#define tmpa9xx_lcdc_resume	NULL
#endif

static int __init tmpa9xx_lcdc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *regs = NULL;
	struct tmpa9xx_lcdc_platforminfo *platforminfo;
	int ret;
    	char *options = NULL;

	ret = -ENOMEM;

	platforminfo = (struct tmpa9xx_lcdc_platforminfo *) dev->platform_data;
	
	if (!platforminfo) {
		printk(KERN_ERR "no platforminfo\n");
		ret = -ENXIO;
		return ret;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		printk(KERN_ERR "resources unusable\n");
		ret = -ENXIO;
		return ret;
	}

	/* Get command line options */
	fb_get_options("tmpa9xxfb", &options);
	if (options) {
		int ret;
	    
		ret = tmpa9xx_lcdc_parse_params(platforminfo->LCDReg, options);
	    	if (ret) {
			platforminfo->width = (((platforminfo->LCDReg[0] & 0xFF) >> 2) + 1) * 16;
			platforminfo->height = (platforminfo->LCDReg[1] & 0x3FF) + 1; 
			platforminfo->pitch = platforminfo->width * platforminfo->depth / 8; 
			printk(KERN_INFO "tmpa9xxfb: set framebuffer size to: %ix%i \n", 
			       platforminfo->width, platforminfo->height);
		}
	}

	/* initialise */
	ret = tmpa9xx_lcdc_init_fb(pdev,
		"TMPA9XX FB" , "Toshiba TMPA9XX Frame Buffer",
		platforminfo->width, platforminfo->height,
		platforminfo->depth, platforminfo->pitch,
		platforminfo->LCDReg, (void *)regs->start);

	return ret;
}

static int __exit tmpa9xx_lcdc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fb_info *info = dev_get_drvdata(dev);
	struct tmpa9xx_lcdc_par *par = info->par;

	if (!par)
		return 0;

	_stop_it(par->hw_tmpa9xx_lcdc);
	
	unregister_framebuffer(info);
	
	fb_dealloc_cmap(&info->cmap);

	if (info->screen_base)
		iounmap(info->screen_base);
		
	info->screen_base = NULL;
	
	dev_set_drvdata(dev, NULL);

	framebuffer_release(info);

	return 0;
}

static struct platform_driver tmpa9xx_lcdc_driver = {
	.remove		= __exit_p(tmpa9xx_lcdc_remove),
	.suspend	= tmpa9xx_lcdc_suspend,
	.resume		= tmpa9xx_lcdc_resume,

	.driver		= {
		.name	 = "tmpa9xxfb",
		.owner = THIS_MODULE,
	},
};

static int __init tmpa9xx_lcdc_init(void)
{
	return platform_driver_probe(&tmpa9xx_lcdc_driver, tmpa9xx_lcdc_probe);
}

static void __exit tmpa9xx_lcdc_exit(void)
{
	platform_driver_unregister(&tmpa9xx_lcdc_driver);
}

module_init(tmpa9xx_lcdc_init);
module_exit(tmpa9xx_lcdc_exit);
MODULE_LICENSE("GPL")
