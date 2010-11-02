/*
 *  arch/arm/mach-tmpa9xx/tmpa9xx_timer.h
 *  General definitions for TMPA9xx timers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <mach/regs.h>

/*********/

struct tmpa9xx_hw_timer {
	uint32_t TimerLoad;	// 0x0000
	uint32_t TimerValue;	// 0x0004
	uint32_t TimerControl;	// 0x0008
	uint32_t TimerIntClr;	// 0x000C
	uint32_t TimerRIS;	// 0x0010
	uint32_t TimerMIS;	// 0x0014
	uint32_t TimerBGLoad;	// 0x0018
	uint32_t TimerMode;	// 0x001C
	uint32_t Rsvd[32];	// 0x0020 -> 0x9C
	uint32_t TimerCompare1;	// 0x00A0 Timer0 Compare value
	uint32_t TimerCmpIntClr1;	// 0x00C0 Timer0 Compare Interrupt clear
	uint32_t TimerCmpEn;	// 0x00E0 Timer0 Compare Enable
	uint32_t TimerCmpRIS;	// 0x00E4 Timer0 Compare raw interrupt status
	uint32_t TimerCmpMIS;	// 0x00E8 Timer0 Compare masked int status
	uint32_t TimerBGCmp;	// 0x00EC Background compare value for Timer0
};

#define TIMxEN			(1<<7)
#define TIMxMOD			(1<<6)
#define TIMxINTE		(1<<5)
#define TIMxSIZE_16B		(1<<1)
#define TIMxOSCTL_NORESTART	(1<<0)

#define TIMxPRS_1 		(0x0<<2)
#define TIMxPRS_16 		(0x1<<2)
#define TIMxPRS_256		(0x2<<2)

/*
 * TimerControl
 * [7]   TIM0EN    R/W 0y0       Timer 0 enable bit
 *                                 0: Disable
 *                                 1: Enable
 * [6]   TIM0MOD   R/W 0y0       Timer 0 mode setting
 *                                 0: Free-running mode
 *                                 1: Periodic timer mode
 * [5]   TIM0INTE  R/W 0y0       Timer 0 interrupt control
 *                                 0: Disable inerrupts
 *                                 1: Enable interrupts
 *       -         -
 * [4]                 Undefined Read undefined. Write as zero.
 * [3:2] TIM0PRS   R/W 0y00      Timer 0 prescaler setting
 *                                 00: No division
 *                                 01: Divide by 16
 *                                 10: Divide by 256
 *                                 11: Setting prohibited
 * [1]   TIM0SIZE  R/W 0y0       8-bit/16-bit counter select for Timer 0
 *                                 0: 8-bit counter
 *                                 1: 16-bit counter
 * [0]   TIM0OSCTL R/W 0y0       One-shot/wrapping mode select for Timer 0
 *                                 0: Wrapping mode
 *                                 1: One-shot mode
*/

/* system timer reference clock, in Hz */
#define REFCLK          (32768)

/* timer counter value, to get an interrupt every HZ */
#define TIMER_RELOAD    (REFCLK/HZ)

