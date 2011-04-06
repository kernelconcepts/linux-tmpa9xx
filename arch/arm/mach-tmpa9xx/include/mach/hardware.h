/*
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

#ifndef __TMPA9XX_HARDWARE_H__
#define __TMPA9XX_HARDWARE_H__

#ifndef __ASSEMBLY__
#include <asm/types.h>
#endif

// The phys IO [0xf000 0000 -> 0xf400 0000]
// Let's do a simple 1:1 mapping
#define TMPA9XX_IO_VIRT_BASE 0xf0000000
#define TMPA9XX_IO_PHYS_BASE 0xf0000000

#define TMPA9XX_IO_SIZE      0x04600000

#ifndef __ASSEMBLY__

#define _in32(__ofs)         ( * ( (volatile unsigned long *) (__ofs) ) )
#define _out32(__ofs,__val)  { (* ( (volatile unsigned long *) (__ofs)) ) = __val; }

#endif

#define __REG(x)	(*((volatile u32 *)io_p2v(x)))
#define __PREG(x)       (io_v2p((u32)&(x)))

/* For assembler  the C-type using macros are not useful */
#define IO_ADDRESS(x) ((x)) 

#define io_p2v(x) ( (x) )
#define io_v2p(x) ( (x) )


#endif
