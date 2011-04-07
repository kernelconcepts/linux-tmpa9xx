/*
 * Clock support for TMPA9xx
 *
 * Copyright (C) 2010 Thomas Haase <Thomas.Haase@web.de>
 * Copyright (C) 2011 Michael Hunold <michael@mihu.de>
 *
 * This program is free software; you may redistribute and/or modify
 * it under the terms of the GNU General Public License Version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/clkdev.h>
#include <linux/io.h>

#include <mach/regs.h>

struct clk
{
	unsigned long rate;
	unsigned long (*get_rate)(struct clk *clk);
	int (*set_rate)(struct clk *clk, unsigned long rate);
	int offset;
};

static struct clk ssp_clk = {
	.rate = 96000000,
};

static struct clk uart_clk = {
	.rate = 96000000,
};

static struct clk clcd_clk = {
	.rate = 96000000,
};

static struct clk dummy_apb_pclk = {
	.rate = 96000000,
};

int clk_enable(struct clk *clk)
{
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	if (clk->get_rate)
		return clk->get_rate(clk);

	return clk->rate;
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	/*TODO*/
	return rate;
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	if (clk->set_rate)
		return clk->set_rate(clk, rate);

	clk->rate = rate;
	return 0;
}
EXPORT_SYMBOL(clk_set_rate);

#define SYSCR1 (0x04)
#define SYSCR2 (0x08)
#define SYSCR3 (0x0C)
#define SYSCR4 (0x10)
#define SYSCR5 (0x14)
#define SYSCR6 (0x18)
#define SYSCR7 (0x1C)
#define CLKCR5 (0x54)

#define clk_writel(b, o, v)	writel(v, PLL_BASE_ADDRESS + o)
#define clk_readl(b, o)		readl(PLL_BASE_ADDRESS + o)

static int timer_set_rate(struct clk *clk, unsigned long rate)
{
	uint32_t val;

	if (rate == clk->rate)
		return 0;

	val = clk_readl(clk, CLKCR5);
	val &= ~(1 << clk->offset);

	/* timer can be set to either 32kHz or fHCLK/2 */

	if (rate == clk_get_rate(&dummy_apb_pclk)/2) {
		val |= (1 << clk->offset);
	} else if (rate == 32*1000) {
		val |= (0 << clk->offset);
	} else {
		pr_debug("%s(): rate %lu out of bounds, offset %d\n", __func__, rate, clk->offset);
		return -EINVAL;
	}

	clk_writel(clk, CLKCR5, val);

	clk->rate = rate;

	pr_debug("%s(): offset %d, rate %lu\n", __func__, clk->offset, clk->rate);

	return 0;
}

static unsigned long timer_get_rate(struct clk *clk)
{
	uint32_t val = clk_readl(clk, CLKCR5);

	clk->rate = (val & (1 << clk->offset)) ? clk_get_rate(&dummy_apb_pclk)/2 : 32 * 1000;

	pr_debug("%s(): offset %d, rate %lu\n", __func__, clk->offset, clk->rate);

	return clk->rate;
}

static struct clk tim01_clk =
{
	.set_rate = timer_set_rate,
	.get_rate = timer_get_rate,
	.offset = 0,
};

static struct clk tim23_clk =
{
	.set_rate = timer_set_rate,
	.get_rate = timer_get_rate,
	.offset = 1,
};

static struct clk_lookup lookups[] = {
        {
		/* Bus clock */
                .con_id         = "apb_pclk",
                .clk            = &dummy_apb_pclk,
        }, {
		/* UART0 */
		.dev_id		= "uart0",
		.clk		= &uart_clk,
	}, {	/* UART1 */
		.dev_id		= "uart1",
		.clk		= &uart_clk,
	}, {	/* UART2 */
		.dev_id		= "uart2",
		.clk		= &uart_clk,
	}, {	/* SSP0 */
		.dev_id		= "tmpa9xx-spi0",
		.clk		= &ssp_clk,
	}, {	/* SSP1 */
		.dev_id		= "tmpa9xx-spi1",
		.clk		= &ssp_clk,
	}, {	/* CLCD */
		.dev_id		= "tmpa9xx-clcd",
		.clk		= &clcd_clk,
	}, {	/* PWM */
		.dev_id		= "tmpa9xx-pwm.0",
		.clk		= &tim01_clk,
	}, {	/* PWM */
		.dev_id		= "tmpa9xx-pwm.1",
		.clk		= &tim23_clk,
	}, {	/* WDT */
		.dev_id		= "tmpa9xx-wdt",
		.clk		= &dummy_apb_pclk,
	}
};

int __init clk_init(void)
{
	uint32_t val;
#define fOSCH (24 * 1000 * 1000)
	uint32_t input_to_fcsel = fOSCH;
	uint32_t clock_gear;
	uint32_t fCLK;
	uint32_t fHCLK;

	val = clk_readl(clk, SYSCR3);
	if ((val & (1 << 7))) {
		int multiplier = (val & 0x7) == 0x5 ? 6 : 8;
		val = clk_readl(clk, SYSCR2);
		if (val & 0x1) {
			pr_debug("%s(): PLL output clock is on, input to fcsel is fPLL x%d\n", __func__, multiplier);
			input_to_fcsel = multiplier * fOSCH;
		}
		else {
			pr_debug("%s(): PLL output clock is on, input to fcsel is fOSCH\n", __func__);
		}
	}

	val = clk_readl(clk, SYSCR1);
	BUG_ON(val & 0x4);

	clock_gear = (1 << (val & 0x3));
	pr_debug("%s(): clock gear set to fc/%d\n", __func__, clock_gear);
	fCLK = input_to_fcsel / clock_gear;
	fHCLK = fCLK / 2;
	pr_debug("%s(): fCLK is %d Hz, fHCLK is %d Hz\n", __func__, fCLK, fHCLK);

	/* register the clock lookups */
	clkdev_add_table(lookups, ARRAY_SIZE(lookups));
	return 0;
}
core_initcall(clk_init);
