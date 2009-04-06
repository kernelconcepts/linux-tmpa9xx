/*
 *  arch/arm/mach-tmpa910/include/mach/hardware.h
 *
 *  Based on arch-mx2ads/hardware.h, which is:
 *  Copyright (C) 2004 Metrowerks Corp.
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
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#ifndef __ASSEMBLY__
#include <asm/types.h>
#endif


#define CLK32 32768


// The phys IO [0xf000 0000 -> 0xf400 0000]
// Let's do a simple 1:1 mapping
#define TMPA910_IO_VIRT_BASE 0xf0000000
#define TMPA910_IO_PHYS_BASE 0xf0000000

#define TMPA910_IO_SIZE      0x04400000

#ifndef __ASSEMBLY__

#define _in32(__ofs)         ( * ( (volatile unsigned long *) (__ofs) ) )
#define _out32(__ofs,__val)  { (* ( (volatile unsigned long *) (__ofs)) ) = __val; }


void tmpa910_led_blink(void);
#endif

/* For assembler  the C-type using macros are not useful */
#define IO_ADDRESS(x) ((x)) 


#define io_p2v(x) ( (x) )
#define io_v2p(x) ( (x) )


#endif
