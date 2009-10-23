/*
 *  Header file for TMPA910 LCD Controller
 *
 *  Data structure and register user interface
 *
 *  Copyright (C) 2008 bplam GmbH
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
#ifndef __TMPA910_LCDC_H__
#define __TMPA910_LCDC_H__

/* register set indices */
#define LCDREG_TIMING0_H   0
#define LCDREG_TIMING1_V   1
#define LCDREG_TIMING2_CLK 2
#define LCDREG_TIMING3_LEC 3
#define LCDREG_LCDCONTROL  4

struct tmpa910_lcdc_platforminfo {
	uint32_t LCDReg[5];
	int width;
	int height;
	int depth;
	int pitch;
};



#endif /* __TMPA910_LCDC_H__ */
