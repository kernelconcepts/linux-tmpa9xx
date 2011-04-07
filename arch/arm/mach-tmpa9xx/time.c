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

#define tmr_writel(b, o, v)	writel(v, b->regs + o)
#define tmr_readl(b, o)		readl(b->regs + o)

struct tmpa9xx_time_priv
{
	void __iomem *regs;
};

static struct tmpa9xx_time_priv g_tmpa9xx_time_priv;

/*
 * gettimeoffset() returns time since last timer tick, in usecs.
 *
 * 'LATCH' is hwclock ticks (see CLOCK_TICK_RATE in timex.h) per jiffy.
 * 'tick' is usecs per jiffy.
 */
static unsigned long tmpa9xx_gettimeoffset(void)
{
	struct tmpa9xx_time_priv *t = &g_tmpa9xx_time_priv;
	unsigned long hwticks;
	uint32_t val;

	val = tmr_readl(t, TIMER_VALUE);
	/* since last underflow */
	hwticks = LATCH - (val & 0xffff);

	return (hwticks * (tick_nsec / 1000)) / LATCH;
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t tmpa9xx_timer_interrupt(int irq, void *dev_id)
{
	struct tmpa9xx_time_priv *t = &g_tmpa9xx_time_priv;
	uint32_t val;

	timer_tick();

	val = tmr_readl(t, TIMER_MIS);
	BUG_ON(!val);

	/* clear the interrupt */
	tmr_writel(t, TIMER_INTCLR, 0);

	return IRQ_HANDLED;
}

static struct irqaction tmpa9xx_timer_irq = {
	.name		= "TMPA9xx Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= tmpa9xx_timer_interrupt,
};

static void __init tmpa9xx_timer_init(void)
{
	struct tmpa9xx_time_priv *t = &g_tmpa9xx_time_priv;
	struct timespec tv;

#warning "fix me"
#define CLKCR5                  __REG(PLL_BASE_ADDRESS + 0x054)
	CLKCR5 &= ~(1<<2);		/* Select 32kHz Clock for Timer*/

	memset(t, 0, sizeof(*t));
	t->regs = ioremap(TMPA9XX_TIMER4, SZ_256-1);
	BUG_ON(!t->regs);

	/* disable timer */
	tmr_writel(t, TIMER_CONTROL, 0);
	/* write pre calculated latch value */
	tmr_writel(t, TIMER_LOAD, LATCH-1);
	/* start counting at 0 */
	tmr_writel(t, TIMER_VALUE, 0);
	/* clear pending interrupts */
	tmr_writel(t, TIMER_INTCLR, 1);

	setup_irq(INTR_VECT_TIMER45, &tmpa9xx_timer_irq);

	tv.tv_nsec = 0;
	tv.tv_sec = 0;
	do_settimeofday(&tv);

	/* enable timer, periodic, enable interrupts, no prescaler, 16 bit */
	tmr_writel(t, TIMER_CONTROL, TIMxEN | TIMxMOD_PER | TIMxINTE | TIMxPRS_1 | TIMxSIZE_16B);
}

struct sys_timer tmpa9xx_timer = {
	.init		= tmpa9xx_timer_init,
	.offset		= tmpa9xx_gettimeoffset,
};
