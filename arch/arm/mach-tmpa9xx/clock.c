/*
 *  Copyright (C) 2010 Thomas Haase
 * 	heavily based on realview platform
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/mutex.h>

#include <asm/clkdev.h>

/* currently the clk structure
 * just supports rate. This would
 * be extended as and when new devices are
 * added - TODO
 */
struct clk {
	unsigned long		rate;
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
	clk->rate = rate;
	return 0;
}
EXPORT_SYMBOL(clk_set_rate);

/* ssp clock */
static struct clk ssp_clk = {
	.rate = 96000000,
};

/* fixed clock */
static struct clk uart_clk = {
	.rate = 96000000,
};

/* fixed clock */
static struct clk clcd_clk = {
	.rate = 96000000,
};

/* fixed clock */
static struct clk tim_clk = {
	.rate = 96000000,
};

static struct clk dummy_apb_pclk;

static struct clk_lookup lookups[] = {
        {      /* Bus clock */
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
		.clk		= &tim_clk,
	}, {	/* PWM */
		.dev_id		= "tmpa9xx-pwm.1",
		.clk		= &tim_clk,
	}
};

int __init clk_init(void)
{
	/* register the clock lookups */
	clkdev_add_table(lookups, ARRAY_SIZE(lookups));
	return 0;
}
core_initcall(clk_init);
