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
/*
#include "hardware.h"

		.macro	addruart,rx
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx, #0xf2000000		@ physical
		movne	\rx, #io_p2v(0xf2000000)	@ virtual
		.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
*/
		.macro	addruart,rx
		.endm

		.macro	senduart,rd,rx
		.endm

		.macro	waituart,rd,rx
		.endm

		.macro	busyuart,rd,rx
		.endm

