/* arch/arm/mach-versatile/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <mach/hardware.h>

		.macro	addruart, rp, rv
		mov	\rp,      #0x00000000
		orr	\rv, \rp, #IO_ADDRESS(0xf2000000)	@ virtual base
		orr	\rp, \rp,  #0xf2000000			@ physical base
		.endm

#include <asm/hardware/debug-pl01x.S>
