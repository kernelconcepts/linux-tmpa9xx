/*
 *  arch/arm/mach-tmpa910/time.c
 *
 *  Copyright (C) 2000-2001 Deep Blue Solutions
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright (C) 2006 Jochen Karrer (jk06@jkarrer.de) 
 *  Copyright (C) 2008 bplan GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/autoconf.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/time.h>

#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <linux/tick.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/time.h>

#include <mach/tmpa910_regs.h>

/*********/

struct hw_timer {
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

#define TIMxEN							(1<<7)
#define TIMxMOD							(1<<6)
#define TIMxINTE						(1<<5)
#define TIMxSIZE_16B				(1<<1)
#define TIMxOSCTL_NORESTART	(1<<0)

#define TIMxPRS_1 			(0x0<<2)
#define TIMxPRS_16 			(0x1<<2)
#define TIMxPRS_256			(0x2<<2)

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

#define GPT_COUNT 6

/* system timer reference clock, in Hz */
#define REFCLK          (32768)

/* timer counter value, to get an interrupt every HZ */
#define TIMER_RELOAD    (REFCLK/HZ)

/* conversion from timer counter values to microseconds */
#define TICKS2USECS(x)  ((x) * 1000000 / REFCLK)


/*********/
/*********/
static irqreturn_t topas910_timer_interrupt(int irq, void *dev_id)
{
	volatile struct hw_timer *hw_timer =
	    (volatile struct hw_timer *)TMPA910_TIMER0;

	struct clock_event_device *c = dev_id;

	//NPRINTK("-> irq=%d, c =%p, TimerMIS=0x%x, TimerRIS=0x%x, TimerControl=0x%x\n",
	//	irq, c, hw_timer->TimerMIS, hw_timer->TimerRIS,  hw_timer->TimerControl);

	c->event_handler(c);

	/* clear the interrupt */
	if (hw_timer->TimerMIS)
		hw_timer->TimerIntClr = -1;

	return IRQ_HANDLED;
}

/*********/
/*********/
static void
tmpa910_set_mode(enum clock_event_mode mode, struct clock_event_device *dev)
{
	unsigned long irqflags;
	uint32_t TimerControl;
	volatile struct hw_timer *hw_timer =
	    (volatile struct hw_timer *)TMPA910_TIMER0;

	TimerControl = 0;

	raw_local_irq_save(irqflags);


	switch (mode) {

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* initializing, released, or preparing for suspend */
		break;

	case CLOCK_EVT_MODE_RESUME:
		break;
		
	case CLOCK_EVT_MODE_ONESHOT:
		TimerControl |=TIMxOSCTL_NORESTART;
		
	case CLOCK_EVT_MODE_PERIODIC:


		hw_timer->TimerLoad = TIMER_RELOAD;
		hw_timer->TimerValue = 0;

		// Default to 16 bits timers
		TimerControl |= TIMxEN | TIMxSIZE_16B;

		TimerControl |= TIMxMOD;
		TimerControl |= TIMxINTE;

		break;
	}

	hw_timer->TimerControl = TimerControl;

	raw_local_irq_restore(irqflags);

}

/*********/
static int
tmpa910_set_next_event(unsigned long delta, struct clock_event_device *dev)
{
	volatile struct hw_timer *hw_timer =
	    (volatile struct hw_timer *)TMPA910_TIMER0;
	unsigned long flags;

	uint32_t TimerControl;

	//NPRINTK("-> delta=%ld. dev=%p\n", delta, dev);

	raw_local_irq_save(flags);

	hw_timer->TimerLoad = delta;
	hw_timer->TimerValue = 0;

	// Default to 16 bits timers
	TimerControl = TIMxEN | TIMxSIZE_16B;

	TimerControl |= TIMxMOD;
	TimerControl |= TIMxINTE;
	
	hw_timer->TimerControl = TimerControl;

	raw_local_irq_restore(flags);

	//NPRINTK("-> hTimerControl=%pm val=0x%x\n", hw_timer->TimerControl, hw_timer->TimerValue);

	return (signed)0;
}

/*********/
/*********/
static cycle_t tmpa910_readcycle(void)
{
	volatile struct hw_timer *hw_timer =
	    (volatile struct hw_timer *)TMPA910_TIMER0;

	unsigned long ticks = 10;

	ticks = hw_timer->TimerValue;
	
	//NPRINTK("ticks=%ld\n", ticks);

	return (cycle_t) ticks;
}

/*********/
/*********/
static struct clock_event_device clockevent_tmpa910 = {
	.name = "tmpa910_timer",
	.features = 0,
	.shift = 32,
	.set_mode = tmpa910_set_mode,
	.set_next_event = tmpa910_set_next_event,
	.rating = 200,
};

static struct irqaction topas910_timer_irq = {
	.name = "TMPA910/TOPAS910 Timer Tick",
	.flags = IRQF_DISABLED | IRQF_TIMER,
	.handler = topas910_timer_interrupt,
	.dev_id = &clockevent_tmpa910,
};

static struct clocksource cksrc_tmpa910 = {
	.name = "tmpa910",
	.rating = 450,
	.read = tmpa910_readcycle,
};

static int tmpa910_clockevent_init(int clock_tick_rate)
{

	cksrc_tmpa910.mask = CLOCKSOURCE_MASK(32);
	cksrc_tmpa910.shift = 20;
	cksrc_tmpa910.mult =
	    clocksource_hz2mult(clock_tick_rate, cksrc_tmpa910.shift);

	clockevent_tmpa910.mult =
	    div_sc(clock_tick_rate, NSEC_PER_SEC, clockevent_tmpa910.shift);
	clockevent_tmpa910.max_delta_ns =
	    clockevent_delta2ns(0xfffffffe, &clockevent_tmpa910);
	clockevent_tmpa910.min_delta_ns =
	    clockevent_delta2ns(0xf, &clockevent_tmpa910);

	clockevent_tmpa910.features = CLOCK_EVT_FEAT_PERIODIC;

	clockevent_tmpa910.cpumask = cpumask_of(0);

	clocksource_register(&cksrc_tmpa910);
	clockevents_register_device(&clockevent_tmpa910);

	return 0;
}

static void topas910_timer_init(void)
{
	volatile struct hw_timer *hw_timer =
	    (volatile struct hw_timer *)TMPA910_TIMER0;

	hw_timer->TimerControl = 0;

	setup_irq(INTR_VECT_TIMER01, &topas910_timer_irq);

	tmpa910_clockevent_init(REFCLK);
}

struct sys_timer topas910_timer = {
	.init = topas910_timer_init,
};

