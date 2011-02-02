/*
 * Industrial I/O - ADC Support for TMPA9xx
 *
 * Copyright (c) 2010 Thomas Haase (Thomas.Haase@web.de)
 * Copyright (c) 2011 Michael Hunold (michael@mihu.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sysfs.h>
#include <linux/semaphore.h>
#include <mach/regs.h>
#include <mach/adc.h>

#include "../iio.h"

#define DRIVER_NAME 	"tmpa9xx-iio-adc"

#if defined CONFIG_TOUCHSCREEN_TMPA9XX || defined CONFIG_TOUCHSCREEN_TMPA9XX_MODULE
#define ADC_MAX_CHANNEL 4
#else
#define ADC_MAX_CHANNEL 8
#endif

#define dprintk(...) // do {printk("%s(): ", __func__); printk(__VA_ARGS__);}  while(0);

struct tmpa9xx_adc_info;

struct tmpa9xx_adc_channel {
	struct iio_dev *indio_dev;
	struct sysfs_dirent *dirent;
	struct tmpa9xx_adc_info *parent;
	int num;
	int value;
	struct completion complete;
	int repeat;
	int remain;
};

struct tmpa9xx_adc_info {
	struct tmpa9xx_adc_channel ch[ADC_MAX_CHANNEL];
	struct device *dev;
	struct semaphore sema;
	wait_queue_head_t wait;
	struct task_struct *tsk;
	struct completion tsk_start;
 	atomic_t active;
	int should_exit;
	int irq;
};

static ssize_t tmpa9xx_show_repeat(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_channel *ch = dev_info->dev_data;
//	struct tmpa9xx_adc_info *chip = ch->parent;

	return sprintf(buf, "%d\n", ch->repeat);
}

static ssize_t tmpa9xx_store_repeat(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_channel *ch = dev_info->dev_data;
	struct tmpa9xx_adc_info *chip = ch->parent;
	unsigned long value;
	int ret;

	ret = strict_strtoul(buf, 10, &value);
	if (ret)
		return -EINVAL;

        ch->repeat = value;

	if (ch->repeat) {
		atomic_inc(&chip->active);
		wake_up(&chip->wait);
	} else {
		atomic_dec(&chip->active);
	}

	return len;
}

static IIO_DEVICE_ATTR(repeat, S_IRUGO | S_IWUSR,
		tmpa9xx_show_repeat,
		tmpa9xx_store_repeat,
		0);
static ssize_t tmpa9xx_show_name(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_channel *ch = dev_info->dev_data;
//	struct tmpa9xx_adc_info *chip = ch->parent;

	return sprintf(buf, "channel %d @ %s\n", ch->num, DRIVER_NAME);
}

static IIO_DEVICE_ATTR(name, S_IRUGO, tmpa9xx_show_name, NULL, 0);

static ssize_t tmpa9xx_show_value(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_channel *ch = dev_info->dev_data;
	struct tmpa9xx_adc_info *chip = ch->parent;
	int delay = 0;
	int ret;

	down(&chip->sema);

	ch->value = tmpa9xx_adc_read(ch->num, delay);

	up(&chip->sema);

	ret = sprintf(buf, "%d\n", ch->value);
	return ret;
}

static IIO_DEVICE_ATTR(value, S_IRUGO, tmpa9xx_show_value, NULL, 0);

static struct attribute *tmpa9xx_attributes[] = {
	&iio_dev_attr_name.dev_attr.attr,
	&iio_dev_attr_value.dev_attr.attr,
	&iio_dev_attr_repeat.dev_attr.attr,
	NULL,
};

static const struct attribute_group tmpa9xx_attribute_group = {
	.attrs = tmpa9xx_attributes,
};

static int adc_thread(void *c)
{
	struct tmpa9xx_adc_info *chip = c;
	int i;

        complete(&chip->tsk_start);

	while(!kthread_should_stop()) {
		wait_event_interruptible(chip->wait, atomic_read(&chip->active) || kthread_should_stop());

		while(!kthread_should_stop()) {
			int wait_ms = INT_MAX;

			for (i = 0; i < ADC_MAX_CHANNEL; i++) {
				struct tmpa9xx_adc_channel *ch = &chip->ch[i];
				if (ch->remain <= 0 && ch->repeat) {
					dprintk("ch %d is due\n", i);
					ch->remain = ch->repeat;
					sysfs_notify_dirent(ch->dirent);
				}
			}

			for (i = 0; i < ADC_MAX_CHANNEL; i++) {
				struct tmpa9xx_adc_channel *ch = &chip->ch[i];
				if (!ch->repeat)
					continue;
				if (ch->remain < wait_ms)
					wait_ms = ch->remain;
				if (ch->repeat)
					dprintk("a repeat %d @ %d, remain %d\n", ch->repeat, i, ch->remain);
			}
			dprintk("wait_ms %d\n", wait_ms);
			if (wait_ms == INT_MAX)
				break;

	                schedule_timeout_interruptible(msecs_to_jiffies(wait_ms));
			for (i = 0; i < ADC_MAX_CHANNEL; i++) {
				struct tmpa9xx_adc_channel *ch = &chip->ch[i];
				if(!ch->repeat)
					continue;
				ch->remain -= wait_ms;
				dprintk("b repeat %d @ %d, remain %d\n", ch->repeat, i, ch->remain);
			}
		}
	}

	return 0;
}

static int tmpa9xx_iio_adc_probe(struct platform_device *pdev)
{
	struct tmpa9xx_adc_info *chip;
	int i;
	int ret;

	chip = kzalloc(sizeof(struct tmpa9xx_adc_info), GFP_KERNEL);
	if (!chip) {
		dev_err(chip->dev, "kzalloc() failed\n");
		return -ENOMEM;
	}

	init_waitqueue_head(&chip->wait);
        init_completion(&chip->tsk_start);
	sema_init(&chip->sema, 1);
	chip->dev = &pdev->dev;

	for (i = 0; i < ADC_MAX_CHANNEL; i++) {
		struct tmpa9xx_adc_channel *ch = &chip->ch[i];

		memset(ch, 0, sizeof(*ch));
		ch->num = i;
		ch->parent = chip;

		ch->indio_dev = iio_allocate_device();
		if (!ch->indio_dev) {
			dev_err(chip->dev, "iio_allocate_device() failed\n");
			ret = -ENOMEM;
			goto err0;
		}
	}

	for (i = 0; i < ADC_MAX_CHANNEL; i++) {
		struct tmpa9xx_adc_channel *ch = &chip->ch[i];

		ch->indio_dev->dev.parent = &pdev->dev;
		ch->indio_dev->attrs = &tmpa9xx_attribute_group;
		ch->indio_dev->dev_data = (void *)ch;
		ch->indio_dev->driver_module = THIS_MODULE;
		ch->indio_dev->modes = INDIO_DIRECT_MODE;

		ret = iio_device_register(ch->indio_dev);
		if (ret) {
			dev_err(chip->dev, "iio_device_register() failed\n");
			ret = -ENOENT;
			goto err1;
		}

	        ch->dirent = sysfs_get_dirent(ch->indio_dev->dev.kobj.sd, NULL, "value");
		BUG_ON(!ch->dirent);
	}

	platform_set_drvdata(pdev, chip);
	dev_info(chip->dev, "tmpa9xx adc driver registered\n");

	chip->tsk = kthread_run(adc_thread, chip, "adc");
	if (IS_ERR(chip->tsk)) {
		dev_err(chip->dev, "kthread_run() failed\n");
		ret = PTR_ERR(chip->tsk);
		goto err2;
	}

	return 0;
/*
err3:
	kthread_stop(chip->tsk);
	i = 1;
*/
err2:
	for (--i; i >= 0; i --) {
		struct tmpa9xx_adc_channel *ch = &chip->ch[i];
		iio_device_unregister(ch->indio_dev);
	}
err1:
	for (i = 0; i < ADC_MAX_CHANNEL; i++) {
		struct tmpa9xx_adc_channel *ch = &chip->ch[i];
		if (ch->indio_dev)
			iio_free_device(ch->indio_dev);
	}
err0:
	kfree(chip);

	return ret;
}

static int __devexit tmpa9xx_iio_adc_remove(struct platform_device *pdev)
{
	struct tmpa9xx_adc_info *chip = platform_get_drvdata(pdev);
	int i;

	kthread_stop(chip->tsk);

	for (i = 0; i < ADC_MAX_CHANNEL; i++) {
		struct tmpa9xx_adc_channel *ch = &chip->ch[i];
		iio_device_unregister(ch->indio_dev);
		iio_free_device(ch->indio_dev);
	}

	kfree(chip);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver tmpa9xx_iio_adc_driver = {
	.probe = tmpa9xx_iio_adc_probe,
	.remove = __devexit_p(tmpa9xx_iio_adc_remove),
	.driver = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init tmpa9xx_iio_adc_init(void)
{
	return platform_driver_register(&tmpa9xx_iio_adc_driver);
}

static void __exit tmpa9xx_iio_adc_exit(void)
{
	platform_driver_unregister(&tmpa9xx_iio_adc_driver);
}

module_init(tmpa9xx_iio_adc_init);
module_exit(tmpa9xx_iio_adc_exit);

MODULE_AUTHOR("Thomas Haase <Thomas.Haase@web.de>, Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("TMPA9xx ADC driver for the iio subsystem");
MODULE_LICENSE("GPL v2");
