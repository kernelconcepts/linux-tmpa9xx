/*
 *  Copyright (C) 2004 Metrowerks Corp.
 *
 *  Based on linux/include/asm-arm/arch-mx1ads/uncompress.h, which is:
 *    Copyright (C) 1999 ARM Limited
 *    Copyright (C) Shane Nay (shane@minirl.com)
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

#ifndef __TMPA9XX_UNCOMPRESS_H__
#define __TMPA9XX_UNCOMPRESS_H__

static void flush(void)
{

}

#define _in32(__ofs)         ( * ( (volatile unsigned long *) (__ofs) ) )
#define _out32(__ofs,__val)  { (* ( (volatile unsigned long *) (__ofs)) ) = __val; }


#define  _UART0DR    0x000
#define  _UART0RSR   0x004
#define  _UART0ECR   0x004
#define  _UART0FR    0x018
#define  _UART0ILPR  0x020
#define  _UART0IBRD  0x024
#define  _UART0FBRD  0x028
#define  _UART0LCR_H 0x02C
#define  _UART0CR    0x030
#define  _UART0IFLS  0x034
#define  _UART0IMSC  0x038
#define  _UART0RIS   0x03C
#define  _UART0MIS   0x040
#define  _UART0ICR   0x044
#define _UART0DMACR 0x048

#define __reg(x) (*((volatile u32 *)(x)))
#define UART0FR __reg(0xf2000000+_UART0FR)
#define UART0DR __reg(0xf2000000+_UART0DR)

#define	UART_FR_TXFE		(1 << 7)

static void putc(int c)
{
	while ((UART0FR & UART_FR_TXFE) == 0);
	UART0DR = (unsigned long)(c & 0xff);
}

/*
 * nothing to setup and wdog currently not used
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()

#endif
