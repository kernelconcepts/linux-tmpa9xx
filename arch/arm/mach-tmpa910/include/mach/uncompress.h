/*
 *  linux/include/asm-arm/arch-tmpa910/uncompress.h
 *
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


#define UART0FR 0xf2000000+_UART0FR 
#define UART0DR 0xf2000000+_UART0DR 


static void putc(int c)
{
	unsigned int reg;
	
	reg = _in32(0xf08013FC);
	reg ++;
	_out32(0xf08013FC, reg);
}

#if 0
static void puthex(unsigned long x) {
        int i;
        char c[10];
        for(i=0;i<8;i++) {
                c[i]=(x>>(28-i*4))&0xf;
                if(c[i]>9) {
                        c[i]+='a'-10;
                } else {
                        c[i]+='0';
                }
        }
        c[i]=0;
        putstr(c);
}
#endif

/*
 * nothing to setup and wdog currently not used
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()
