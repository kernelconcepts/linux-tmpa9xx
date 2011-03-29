/*
 *  linux/arch/arm/mach-tmpa9xx/time.c
 *
 *  Copyright (C) 2001 Deep Blue Solutions Ltd.
 *  Copyright (C) 2010 Thomas Haase (Thomas.Haase@web.de)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <linux/timex.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/leds.h>
#include <asm/mach/time.h>

#include <mach/hardware.h>
#include <mach/timer.h>
#include <mach/regs.h>

#define TIMER4_LOAD		__REG(TIME4_BASE_ADDRESS + 0x0000)
#define TIMER4_VALUE		__REG(TIME4_BASE_ADDRESS + 0x0004)
#define TIMER4_CONTROL  	__REG(TIME4_BASE_ADDRESS + 0x0008)
#define TIMER4_INTCLR		__REG(TIME4_BASE_ADDRESS + 0x000C)
#define TIMER4_RIS		__REG(TIME4_BASE_ADDRESS + 0x0010)
#define TIMER4_MIS		__REG(TIME4_BASE_ADDRESS + 0x0014)
#define TIMER4_BGLOAD		__REG(TIME4_BASE_ADDRESS + 0x0018)

/*
 * gettimeoffset() returns time since last timer tick, in usecs.
 *
 * 'LATCH' is hwclock ticks (see CLOCK_TICK_RATE in timex.h) per jiffy.
 * 'tick' is usecs per jiffy.
 */
static unsigned long tmpa9xx_gettimeoffset(void)
{
	unsigned long hwticks;
	hwticks = LATCH - (TIMER4_VALUE & 0xffff);	/* since last underflow */
	return (hwticks * (tick_nsec / 1000)) / LATCH;
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t tmpa9xx_timer_interrupt(int irq, void *dev_id)
{
	timer_tick();
        
	/* clear the interrupt */
	if (TIMER4_MIS)
		TIMER4_INTCLR = ~0;
	return IRQ_HANDLED;
}

static struct irqaction tmpa9xx_timer_irq = {
	.name		= "TMPA9xx Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= tmpa9xx_timer_interrupt,
};

static void __init tmpa9xx_timer_init(void)
{
	struct timespec tv;

	CLKCR5 &= ~(1<<2);		/* Select 32kHz Clock for Timer*/
        
	TIMER4_CONTROL = 0;		/* Disable Timer */
        TIMER4_LOAD    = LATCH-1;	/* Write Latch value */
        TIMER4_VALUE   = 0;		/* Setup counter with 0 */
	TIMER4_INTCLR  = 1;		/* Clear pending interrupts */

	setup_irq(INTR_VECT_TIMER45, &tmpa9xx_timer_irq);

	tv.tv_nsec = 0;
	tv.tv_sec = 0;
	do_settimeofday(&tv);

	TIMER4_CONTROL =   TIMxEN 	/* Enable Timer */
			 | TIMxMOD_PER	/* Periodic */
                         | TIMxINTE	/* Enble interrupt */
                         | TIMxPRS_1	/* No prescaler */
                         | TIMxSIZE_16B;/* 16 Bit timer */
}

struct sys_timer tmpa9xx_timer = {
	.init		= tmpa9xx_timer_init,
	.offset		= tmpa9xx_gettimeoffset,
};
