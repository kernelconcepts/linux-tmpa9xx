/*
 * linux/include/asm-arm/arch-tmpa9xx/dma.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_ARCH_TMPA9XX_LCDDA_H
#define __ASM_ARCH_TMPA9XX_LCDDA_H

struct tmpa9xx_blit
{
	unsigned long src;
	unsigned long dst;
	int w;
	int h;
	int bpp;
	int src_pitch;
	int dst_pitch;
	int rotation;
	int blend;
};

/* this ioctl definition is provided via the framebuffer fd.
'F' is from include/linux/fb.h */

#define TMPA9XX_BLIT _IOW('F', 0x30, struct tmpa9xx_blit)

#endif /* __ASM_ARCH_TMPA9XX_LCDDA_H */
