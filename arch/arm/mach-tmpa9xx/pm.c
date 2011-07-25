/*
 * Power management support for TMPA9xx
 *
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

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/spinlock.h>

#include <asm/cacheflush.h>
#include <asm/delay.h>

#include <mach/platform.h>
#include <mach/system.h>

extern unsigned int tmpa9xx_cpu_suspend_sz;
static void (*tmpa9xx_sram_suspend)(void *adr);
extern void tmpa9xx_cpu_suspend(void);

static void tmpa9xx_sram_push(void *dest, void *src, unsigned int size)
{
	memcpy(dest, src, size);

	flush_icache_range((unsigned long)dest, (unsigned long)(dest + size));
}

void tmpa9xx_clock_pll_output_clock_fosch(void);
void tmpa9xx_clock_pll_output_clock_pll(void);

static void tmpa9xx_pm_suspend(void)
{
	void *adr = tmpa9xx_sram_suspend + ((tmpa9xx_cpu_suspend_sz + 31) / 32) * 32;

	tmpa9xx_clock_pll_output_clock_fosch();

	tmpa9xx_sram_suspend(adr);

	tmpa9xx_clock_pll_output_clock_pll();
}

static int tmpa9xx_pm_enter(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		tmpa9xx_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct platform_suspend_ops tmpa9xx_pm_ops = {
	.enter		= tmpa9xx_pm_enter,
	.valid		= suspend_valid_only_mem,
};

static int __init tmpa9xx_pm_probe(struct platform_device *pdev)
{
	/* todo: allocate sram */

	tmpa9xx_sram_suspend = ioremap(0xf8002000, SZ_4K);
	BUG_ON(!tmpa9xx_sram_suspend);

	/* push suspend code to sram */
	tmpa9xx_sram_push(tmpa9xx_sram_suspend, tmpa9xx_cpu_suspend,
						tmpa9xx_cpu_suspend_sz);

	suspend_set_ops(&tmpa9xx_pm_ops);

	dev_info(&pdev->dev, "ready, sz %d\n", tmpa9xx_cpu_suspend_sz);

	return 0;
}

static int __exit tmpa9xx_pm_remove(struct platform_device *pdev)
{
	/* todo: free sram */

	return 0;
}

static struct platform_driver tmpa9xx_pm_driver = {
	.driver = {
		.name	 = "tmpa9xx-pm",
		.owner	 = THIS_MODULE,
	},
	.remove = __exit_p(tmpa9xx_pm_remove),
};

static int __init tmpa9xx_pm_init(void)
{
	return platform_driver_probe(&tmpa9xx_pm_driver, tmpa9xx_pm_probe);
}

late_initcall(tmpa9xx_pm_init);
