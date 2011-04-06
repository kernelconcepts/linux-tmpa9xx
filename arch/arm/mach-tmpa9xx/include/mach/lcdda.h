/*
 *  Copyright (C) 2011 Michael Hunold <michael@mihu.de>
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

#ifndef __TMPA9XX_LCDDA_H__
#define __TMPA9XX_LCDDA_H__

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

struct tmpa9xx_frect
{
	unsigned long dst;
	unsigned long color;
	int w;
	int h;
	int bpp;
	int pitch;
};

/* this ioctl definition is provided via the framebuffer fd.
'F' is from include/linux/fb.h */

#define TMPA9XX_BLIT  _IOW('F', 0x30, struct tmpa9xx_blit)
#define TMPA9XX_FRECT _IOW('F', 0x31, struct tmpa9xx_frect)

#endif
