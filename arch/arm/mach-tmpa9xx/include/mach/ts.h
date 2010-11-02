/*
 *  Header file for TMPA9xx TS Controller
 *
 *  Data structure and register user interface
 *
 *  Copyright (C) 2008 bplan GmbH
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
#ifndef __TMPA9XX_TS_H__
#define __TMPA9XX_TS_H__


#define TMPA9XX_TS_DEFAULT_FUZZ 4
#define TMPA9XX_TS_DEFAULT_RATE 25
#define TMPA9XX_TS_DEFAULT_SKIP_COUNT 1

struct tmpa9xx_ts_platforminfo {
	int fuzz;
	int rate;
	int skip_count;
};


/*
 * Controller register
 */
#define TMPA9XX_TS_CR0_TSI7     (1<<7) //  TSI7  R/W 0y0 pull-down resistor(refer to Explanation)
#define TMPA9XX_TS_CR0_INGE     (1<<6) //  INGE  R/W 0y0 Input gate control of Port PD6, PD7
#define TMPA9XX_TS_CR0_PTST     (1<<5) //  PTST  R   0y0 Detection condition
#define TMPA9XX_TS_CR0_TWIEN	(1<<4) //  TWIEN R/W 0y0 INTA interrupt control
#define TMPA9XX_TS_CR0_PYEN     (1<<3) //  PYEN  R/W 0y0 SPY
#define TMPA9XX_TS_CR0_PXEN     (1<<2) //  PXEN  R/W 0y0 SPX
#define TMPA9XX_TS_CR0_MYEN     (1<<1) //  MYEN  R/W 0y0 SMY
#define TMPA9XX_TS_CR0_MXEN     (1<<0) //  MXEN[0] MXEN  R/W 0y0 SMX


struct tmpa9xx_ts
{
	uint32_t tsicr0; // 0x01f0 tsi control register0
	uint32_t tsicr1; // 0x01f4 tsi control register1
};

#endif /* __TMPA9XX_TS_H__ */
