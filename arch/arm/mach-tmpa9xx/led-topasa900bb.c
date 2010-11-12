/*
 *  arch/arm/mach-tmpa9xx/led-topasbb.c 
 *
 * Copyright (C) 2010 Thomas Haase < Thomas.Haase@web.de> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 * Toshiba Topasa900, LED driver
 *
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/system.h>
#include <mach/hardware.h>

#include <asm/mach/arch.h>
#include <mach/hardware.h>
#include <mach/regs.h>

#include "tmpa9xx.h"

#ifdef CONFIG_PM
static int saved_state;
#endif

ssize_t led_segment_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%i\n", GPIOBDATA);
}

ssize_t led_segment_store(struct device *pdev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (count) {
		GPIOBDATA = simple_strtol(buf, NULL, 10);
	}
	return count;

}

DEVICE_ATTR(led_segment, 0644, led_segment_show, led_segment_store);

static int __init topas_led_probe(struct platform_device *pdev)
{
	int ret = 0;
    
	platform_set_drvdata(pdev, NULL);
    
	ret = device_create_file(&pdev->dev, &dev_attr_led_segment);
    
	return 0;
}

static int __devexit topas_led_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM
static int topas_led_suspend(struct platform_device *pdev, pm_message_t state)
{
	saved_state = led_segment_store();
    
	return 0;
}

static int topas_led_resume(struct platform_device *pdev)
{
    
	return 0;
}
#else

#define topas_led_suspend  NULL
#define topas_led_resume  NULL

#endif

static struct platform_driver topas_led_driver = {
	.probe		= topas_led_probe,
	.remove		= __exit_p(topas_led_remove),
	.suspend	= topas_led_suspend,
	.resume		= topas_led_resume,
	.driver		= {
		.name	= "led-topas",
	},
};

static int __init topas_led_init(void)
{
	return platform_driver_register(&topas_led_driver);
}

static void __exit topas_led_exit(void)
{
	platform_driver_unregister(&topas_led_driver);
}

module_init(topas_led_init);
module_exit(topas_led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Thomas Haase <Thomas.Haase@web.de>");
MODULE_DESCRIPTION("LED driver for TOPASA 900");
