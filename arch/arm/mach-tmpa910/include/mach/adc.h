/*
 *  Header file for TMPA910 TS Controller
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
#ifndef __TMPA910_ADC_H__
#define __TMPA910_ADC_H__


struct tmpa910_adc
{
	uint32_t adreg0l; //   0x0000 a/d conversion result lower-order register 0
	uint32_t adreg0h; //   0x0004 a/d conversion result higher-order register 0
	uint32_t adreg1l; //   0x0008 a/d conversion result lower-order register 1
	uint32_t adreg1h; //   0x000c a/d conversion result higher-order register 1
	uint32_t adreg2l; //   0x0010 a/d conversion result lower-order register 2
	uint32_t adreg2h; //   0x0014 a/d conversion result higher-order register 2
	uint32_t adreg3l; //   0x0018 a/d conversion result lower-order register 3
	uint32_t adreg3h; //   0x001c a/d conversion result higher-order register 3
	uint32_t adreg4l; //   0x0020 a/d conversion result lower-order register 4
	uint32_t adreg4h; //   0x0024 a/d conversion result higher-order register 4
	uint32_t adreg5l; //   0x0028 a/d conversion result lower-order register 5
	uint32_t adreg5h; //   0x002c a/d conversion result higher-order register 5
	uint32_t rsd1;  	//         0x0030 reserved
	uint32_t rsd2;  	//         0x0034 reserved
	uint32_t rsd3;  	//         0x0038 reserved
	uint32_t rsd4;  	//         0x003c reserved
	uint32_t adregspl; //  0x0040 top-priority a/d conversion result lower-order register
	uint32_t adregsph; //  0x0044 top-priority a/d conversion result higher-order register
	uint32_t adcomregl; // 0x0048 a/d conversion result comparison lower-order register
	uint32_t adcomregh; // 0x004c a/d conversion result comparison lower-order register
	uint32_t admod0; //    0x0050 a/d mode control register 0
	uint32_t admod1; //    0x0054 a/d mode control register 1
	uint32_t admod2; //    0x0058 a/d mode control register 2
	uint32_t admod3; //    0x005c a/d mode control register 3
	uint32_t admod4; //    0x0060 a/d mode control register 4
	uint32_t rsd5;  	//         0x0064 reserved
	uint32_t rsd6;  	//         0x0068 reserved
	uint32_t rsd7;  	//         0x006c reserved
	uint32_t adclk; //     0x0070 a/d conversion clock setting register
	uint32_t adie; //      0x0074 a/d interrupt enable register
	uint32_t adis; //      0x0078 a/d interrupt status register
	uint32_t adic; //      0x007c a/d interrupt clear register
};

#endif /* __TMPA910_ADC_H__ */
