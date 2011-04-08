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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#define GPIO_LED_SEG_START 8

#ifdef CONFIG_MACH_TOPAS910
# define NUM_GPIOS  8
/* Pattern for digits from 0 to 9, "L.", clear, all on, dp on */
static const unsigned char pattern[] = {0x3F, 0x06/*1*/, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F/*9*/, 0xB8, 0x00, 0xFF, 0x08};
#else
# define NUM_GPIOS  4
/* Pattern for digits from 0 to 9, a to f. The a900 can only do this. */
static const unsigned char pattern[] = { 0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, };
#endif

struct tmpa9xx_led
{
	struct device *dev;
	struct gpio array[NUM_GPIOS];
	int state;
};

static int segments_set(struct tmpa9xx_led *l, int value)
{
	int i;

	if (value >= ARRAY_SIZE(pattern)) {
		dev_err(l->dev, "%s(): value %d too large\n", __func__, value);
		return -1;
	}

	if (value < 0) {
		dev_err(l->dev, "%s(): value %d out of bounds\n", __func__, value);
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(l->array); i++)
		gpio_set_value(GPIO_LED_SEG_START + i, (pattern[value] & (1 << i)) ? 0 : 1);

	l->state = value;

	dev_dbg(l->dev, "%s(): value %d\n", __func__, value);

	return 0;
}

ssize_t led_segment_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmpa9xx_led *l = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%i\n", l->state);
}

ssize_t led_segment_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct tmpa9xx_led *l = dev_get_drvdata(dev);
	int value;
	int ret;

	value = simple_strtol(buf, NULL, 10);

	ret = segments_set(l, value);
	if (ret)
		return -EFAULT;

	return count;
}

DEVICE_ATTR(led_segment, 0644, led_segment_show, led_segment_store);

static int __devinit topas_led_probe(struct platform_device *pdev)
{
	struct tmpa9xx_led *l;
	int i;
	int ret;

	l = kzalloc(sizeof(*l), GFP_KERNEL);
	if (!l) {
		dev_err(&pdev->dev, "kzalloc() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	l->dev = &pdev->dev;

	for (i = 0; i < ARRAY_SIZE(l->array); i++) {
		struct gpio *g = &l->array[i];
		g->gpio = GPIO_LED_SEG_START + i;
		g->flags = GPIOF_OUT_INIT_LOW;
		g->label = "topas-led";
	}


	platform_set_drvdata(pdev, l);

	ret = gpio_request_array(&l->array[0], ARRAY_SIZE(l->array));
	if (ret) {
		dev_err(l->dev, "gpio_request_array() failed\n");
		goto err1;
	}

	/* Clear state, bootloader leaves it undefined */
	segments_set(l, 10);

	ret = device_create_file(l->dev, &dev_attr_led_segment);
	if (ret) {
		dev_err(l->dev, "device_create_file() failed\n");
		ret = -EFAULT;
		goto err2;
	}

        dev_info(l->dev, "ready\n");

	return 0;

err2:
	gpio_free_array(&l->array[0], ARRAY_SIZE(l->array));
err1:
	platform_set_drvdata(pdev, NULL);
	kfree(l);
err0:
	return ret;
}

static int __devexit topas_led_remove(struct platform_device *pdev)
{
	struct tmpa9xx_led *l = platform_get_drvdata(pdev);

	device_remove_file(l->dev, &dev_attr_led_segment);

	gpio_free_array(&l->array[0], ARRAY_SIZE(l->array));

	platform_set_drvdata(pdev, NULL);

	kfree(l);

	return 0;
}

#ifdef CONFIG_PM
static int topas_led_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tmpa9xx_led *l = platform_get_drvdata(pdev);
	int old_state = l->state;

#ifdef CONFIG_MACH_TOPAS910
	segments_set(l, 11); /* all led off */
#else
	segments_set(l, 1); /* most power saving value...  */
#endif
	l->state = old_state;

	return 0;
}

static int topas_led_resume(struct platform_device *pdev)
{
	segments_set(l, l->state);
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
