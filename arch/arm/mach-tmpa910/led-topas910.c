/*
 *  arch/arm/mach-tmpa910/led-topas910.c 
 *
 * Copyright (C) 2009 Florian Boor <florian.boor@kernelconcepts.de>
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
 * Toshiba Topas 910, LED driver, mainly for GPIO testing
 *
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/system.h>
#include <mach/hardware.h>

#include <mach/gpio.h>

#include <asm/mach/arch.h>
#include <mach/hardware.h>
#include <mach/tmpa910_regs.h>

#include "topas910.h"

#define GPIO_LED_SEG_A       8
#define GPIO_LED_SEG_B       9
#define GPIO_LED_SEG_C      10
#define GPIO_LED_SEG_D      11
#define GPIO_LED_SEG_E      12
#define GPIO_LED_SEG_F      13
#define GPIO_LED_SEG_G      14
#define GPIO_LED_SEG_DP     15

/* Pattern for digits from 0 to 9, "L.", clear, all on */
static const unsigned char pattern[] = {0x3F, 0x06/*1*/, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0xB8, 0x00, 0xFF};
static unsigned int num_pattern = ARRAY_SIZE(pattern);
static int saved_state;


static void segments_set(int value)
{
	int i;
    
	if (value >= num_pattern)
		return;
    
	if (value < 0)
		return;

	for (i=0; i<8; i++)
		gpio_set_value(GPIO_LED_SEG_A + i, (pattern[value] & (1 << i)) ? 0 : 1);
}

static int segments_get(void)
{
	int i, p = 0;
    
	for (i=0; i<8; i++)
		p |= (gpio_get_value(GPIO_LED_SEG_A + i) ? 0 : (1 << i));
    
	for (i=0; i<num_pattern; i++)
		if (p == pattern[i])
			return i;
    
	return -1;
}

ssize_t led_segment_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%i\n", segments_get());
}


ssize_t led_segment_store(struct device *pdev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (count) {
		int i = simple_strtol(buf, NULL, 10);
	        
	        segments_set(i);
	}
	return count;
}

DEVICE_ATTR(led_segment, 0644, led_segment_show, led_segment_store);


static int __init topas_led_probe(struct platform_device *pdev)
{
	int ret;
    
	platform_set_drvdata(pdev, NULL);
    
	/* Yes we could have this easier, but I need a customer for the GPIO implementation */
	gpio_request(GPIO_LED_SEG_A, "LED_SEG_A");
	gpio_request(GPIO_LED_SEG_B, "LED_SEG_B");
	gpio_request(GPIO_LED_SEG_C, "LED_SEG_C");
	gpio_request(GPIO_LED_SEG_D, "LED_SEG_D");
	gpio_request(GPIO_LED_SEG_E, "LED_SEG_E");
	gpio_request(GPIO_LED_SEG_F, "LED_SEG_F");
	gpio_request(GPIO_LED_SEG_G, "LED_SEG_G");
	gpio_request(GPIO_LED_SEG_DP, "LED_SEG_DP");

	/* Clear state, bootloader leaves it undefined */
        segments_set(10);
	
	ret = device_create_file(&pdev->dev, &dev_attr_led_segment);
    
	return 0;
}

static int __devexit topas_led_remove(struct platform_device *pdev)
{
	gpio_free(GPIO_LED_SEG_A);
	gpio_free(GPIO_LED_SEG_B);
	gpio_free(GPIO_LED_SEG_C);
	gpio_free(GPIO_LED_SEG_D);
	gpio_free(GPIO_LED_SEG_E);
	gpio_free(GPIO_LED_SEG_F);
	gpio_free(GPIO_LED_SEG_G);
	gpio_free(GPIO_LED_SEG_DP);

	return 0;
}

#ifdef CONFIG_PM

static int topas_led_suspend(struct platform_device *pdev, pm_message_t state)
{
	saved_state = segments_get();
	segments_set(11); /* all led off */
    
	return 0;
}

static int topas_led_resume(struct platform_device *pdev)
{
	segments_set(saved_state);
    
	return 0;
}
#else

#define topas_led_suspend  NULL
#define topas_led_resume  NULL

#endif

static struct platform_driver topas_led_driver = {
	.probe		= topas_led_probe,
	.remove		= __devexit_p(topas_led_remove),
	.suspend	= topas_led_suspend,
	.resume		= topas_led_resume,
	.driver		= {
		.name	= "led-topas",
		.owner	= THIS_MODULE,
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
MODULE_AUTHOR("Florian Boor");
MODULE_DESCRIPTION("LED driver for TOPAS 910");
