/*
 * Copyright (C) 2009 Florian Boor <florian.boor@kernelconcepts.de>
 * Copyright (C) 2011 Michael Hunold <michael@mihu.de>
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
#include <mach/regs.h>

#define GPIO_LED_SEG_START 8

#ifdef CONFIG_MACH_TOPAS910
# define NUM_GPIOS  8
/* Pattern for digits from 0 to 9, "L.", clear, all on, dp on */
static const unsigned char pattern[] = {0x3F, 0x06/*1*/, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F/*9*/, 0xB8, 0x00, 0xFF, 0x08};

#else   /* CONFIG_MACH_TOPASA900 */
# define NUM_GPIOS  4
/* Pattern for digits from 0 to 9, a to f. The a900 can only do this. */
static const unsigned char pattern[] = { 0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, };
#endif

static unsigned int num_pattern = ARRAY_SIZE(pattern);

#ifdef CONFIG_PM
static int saved_state;
#endif

static void segments_set(struct device *dev, int value)
{
	int i;

	dev_info(dev, "%s(): value %d\n", __func__, value);

	if (value >= num_pattern)
		return;

	if (value < 0)
		return;

	for (i=0; i<NUM_GPIOS; i++)
		gpio_set_value(GPIO_LED_SEG_START + i, (pattern[value] & (1 << i)) ? 0 : 1);
}

static int segments_get(struct device *dev)
{
	int i, p = 0;

	for (i=0; i<NUM_GPIOS; i++)
		p |= (gpio_get_value(GPIO_LED_SEG_START + i) ? 0 : (1 << i));

	for (i=0; i<num_pattern; i++)
		if (p == pattern[i])
			return i;

	return -1;
}

ssize_t led_segment_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%i\n", segments_get(dev));
}

ssize_t led_segment_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (count) {
		int i = simple_strtol(buf, NULL, 10);

		segments_set(dev, i);
	}
	return count;
}

DEVICE_ATTR(led_segment, 0644, led_segment_show, led_segment_store);

static struct gpio array[NUM_GPIOS];

static int __devinit topas_led_probe(struct platform_device *pdev)
{
	int i;
	int ret = 0;

	for (i = 0; i < NUM_GPIOS; i++) {
		array[i].gpio = GPIO_LED_SEG_START + i;
		array[i].flags = GPIOF_OUT_INIT_LOW;
		array[i].label = "topas-led";
	}

	platform_set_drvdata(pdev, NULL);

	ret = gpio_request_array(&array[0], NUM_GPIOS);
	if (ret) {
		dev_err(&pdev->dev, "gpio_request_array() failed\n");
		return ret;
	}

	/* Clear state, bootloader leaves it undefined */
	segments_set(&pdev->dev, 10);

	ret = device_create_file(&pdev->dev, &dev_attr_led_segment);

	return 0;
}

static int __devexit topas_led_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_led_segment);

	gpio_free_array(&array[0], NUM_GPIOS);

	return 0;
}

#ifdef CONFIG_PM
static int topas_led_suspend(struct platform_device *pdev, pm_message_t state)
{
	saved_state = segments_get();
#ifdef CONFIG_MACH_TOPAS910
	segments_set(11); /* all led off */
#else
	segments_set(1); /* most power saving value...  */
#endif

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
MODULE_AUTHOR("Florian Boor <florian.boor@kernelconcepts.de>");
MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("LED driver for Topas development boards");
