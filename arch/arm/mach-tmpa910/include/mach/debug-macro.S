/* linux/include/asm-arm/arch-tmpa910/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include "hardware.h"
#define UART0DR 0x00
#define UART0FR 0x18
#define FR_TXFE 0x40
#define FR_BUSY 0x08

		.macro	addruart,rx
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx, #0xf2000000		@ physical
		movne	\rx, #io_p2v(0xf2000000)	@ virtual
		.endm

		.macro	senduart,rd,rx
		strb	\rd, [\rx, #UART0DR]
		.endm

		.macro	busyuart,rd,rx
1002:		ldrb	\rd, [\rx, #UART0FR]
		and	\rd, \rd, #FR_TXFE
		teq	\rd, #FR_TXFE
		bne	1002b
		.endm

		.macro	waituart,rd,rx
1001:		ldrb	\rd, [\rx, #UART0FR]
		and	\rd, \rd, #FR_BUSY
		teq	\rd, #0
		bne	1001b
		.endm

/*
		.macro	addruart,rx
		.endm

		.macro	senduart,rd,rx
		.endm

		.macro	waituart,rd,rx
		.endm

		.macro	busyuart,rd,rx
		.endm
*/
