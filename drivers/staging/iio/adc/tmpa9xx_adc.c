/*
 * Industrial I/O - ADC Support for TMPA9xx
 *
 * Copyright (c) 2010 Thomas Haase (Thomas.Haase@web.de)
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
#include <mach/regs.h>

#include "../iio.h"

#define DRIVER_NAME 	"tmpa9xx-adc"

#if defined CONFIG_TOUCHSCREEN_TMPA9XX || defined CONFIG_TOUCHSCREEN_TMPA9XX_MODULE
#define TMPA9XX_CHANNEL_MASK 0x0f
#else
#define TMPA9XX_CHANNEL_MASK 0xff
#endif

#define ADC_REPEAT            (1<<2)
#define FIXED_ADC_PRESCALER      0x2
#define ADC_TIMEOUT           0xFFFF
#define ADC_MAX_CHANNEL            8
#define ADC_MAX_AVERAGE_COUNT     10

/**
 * struct tmpa9xx_adc_state - device instance specific data
 * @work_trigger_to_ring: bh for triggered event handling
 * @work_cont_thresh: CLEAN
 * @inter:		used to check if new interrupt has been triggered
 * @last_timestamp:	passing timestamp from th to bh of interrupt handler
 * @indio_dev:		industrial I/O device structure
 * @trig:		data ready trigger registered with iio
 * @tx:			transmit buffer
 * @rx:			recieve buffer
 * @buf_lock:		mutex to protect tx and rx
 **/
struct tmpa9xx_adc_info {
	const char                  *name;
	struct iio_dev              *indio_dev;
	struct iio_trigger          *trig;
        unsigned char               scan_mask;
        int                         irq;
        unsigned char               average_base[ADC_MAX_CHANNEL];
        unsigned char               average_pos[ADC_MAX_CHANNEL];
        unsigned long               values[ADC_MAX_CHANNEL][ADC_MAX_AVERAGE_COUNT];
	struct workqueue_struct     *adc_workq;
	struct delayed_work         scheduled_restart;
	int                         interval;
};

irqreturn_t interrupt_handler(int irq, void *dev)
{
	int i;
	struct tmpa9xx_adc_info *chip = dev;

	ADC_ADIC=(0x01<<0);

	if (chip->adc_workq)
		queue_delayed_work(chip->adc_workq, &chip->scheduled_restart, chip->interval);

	for (i = 0; i < ADC_MAX_CHANNEL; i++) 
		if (((chip->scan_mask) & (1<<i))==(1<<i))
                {
			chip->values[i][chip->average_pos[i]]=((ADC_ADREGxH(i)&0xff)<<2) | ((ADC_ADREGxL(i)>>6) & 0x3);
                        chip->average_pos[i]++;
                        if (chip->average_pos[i]==chip->average_base[i])
                        	chip->average_pos[i]=0;
                }
        
	return IRQ_HANDLED;
}

static void adc_scheduled_restart(struct work_struct *work)
{
//	struct tmpa9xx_ts_priv *tmpa9xx_ts_priv = container_of(work, struct tmpa9xx_ts_priv , scheduled_restart.work);

        /* Start conversion */
       	ADC_ADMOD0 |= 0x01;
	
}

static ssize_t tmpa9xx_show_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_info *chip = dev_info->dev_data;

	if (chip->adc_workq) 
		return sprintf(buf, "repeat\n");
	else
		return sprintf(buf, "single\n");

}

static ssize_t tmpa9xx_store_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_info *chip = dev_info->dev_data;

	if (strcmp(buf, "repeat"))

        {
		chip->adc_workq = create_singlethread_workqueue("tmpa9xx_adc");
		if (chip->adc_workq == NULL) {
			printk(KERN_ERR "Failed to create workqueue\n");
			return -ENOMEM;
		}

		INIT_DELAYED_WORK(&chip->scheduled_restart, adc_scheduled_restart);

        }
	else
        {
		if (chip->adc_workq) {	
			cancel_delayed_work_sync(&chip->scheduled_restart);
			destroy_workqueue(chip->adc_workq);
		}
        }       

	return len;
}

static IIO_DEVICE_ATTR(mode, S_IRUGO | S_IWUSR,
		tmpa9xx_show_mode,
		tmpa9xx_store_mode,
		0);

static ssize_t tmpa9xx_show_average_count(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int i=0, ret,size=0;
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_info *chip = dev_info->dev_data;

	for (i=0;i<ADC_MAX_CHANNEL;i++)
		if (((chip->scan_mask) & (1<<i))==(1<<i))
                {
			ret = sprintf(buf, "channel[%d]=0x%03x\n", i, chip->average_base[i]);
			if (ret < 0)
				break;
			buf  += ret;
			size += ret;
		}

	return size;
}

static ssize_t tmpa9xx_store_average_count(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	unsigned int channel=0,value=0, ret;
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_info *chip = dev_info->dev_data;

	ret=sscanf(buf,"channel[%d]=0x%x\n",&channel,&value);

	if (ret !=2)
        	return -EINVAL;

        chip->average_base[channel]=value;

	return len;
}

static IIO_DEVICE_ATTR(average_count, S_IRUGO | S_IWUSR,
		tmpa9xx_show_average_count,
		tmpa9xx_store_average_count,
		0);
                
static ssize_t tmpa9xx_show_repeat(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_info *chip = dev_info->dev_data;

	return sprintf(buf,"%d\n",chip->interval);
}

static ssize_t tmpa9xx_store_repeat(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	unsigned long value=0, ret;
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_info *chip = dev_info->dev_data;

	ret = strict_strtoul(buf, 10, &value);
	if (ret)
		return -EINVAL;

        chip->interval=value;

	return len;
}

static IIO_DEVICE_ATTR(repeat, S_IRUGO | S_IWUSR,
		tmpa9xx_show_repeat,
		tmpa9xx_store_repeat,
		0);

static ssize_t tmpa9xx_show_available_modes(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "single\nrepeat\n");
}

static IIO_DEVICE_ATTR(available_modes, S_IRUGO, tmpa9xx_show_available_modes, NULL, 0);

static ssize_t tmpa9xx_reset(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{

	/* Initiate Reset */
	ADC_ADMOD4 = 0x2;
        udelay(10);
	ADC_ADMOD4 = 0x1;
        
        /* Resetup ADCCLK */
        ADC_ADCLK = (0x01 << 7) | FIXED_ADC_PRESCALER;

	return len;
}

static IIO_DEVICE_ATTR(reset, S_IWUSR,NULL,tmpa9xx_reset,0);

static ssize_t tmpa9xx_measure(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	int ret;
        unsigned long value;

	ret = strict_strtoul(buf, 10, &value);
        
	if (ret)
		return -EINVAL;
                
	if (value==1)
	        /* Start conversion */
        	ADC_ADMOD0 |= 0x01;
	else
	        /* Start conversion */
        	ADC_ADMOD0 &=~0x01;
                        

	return len;
}

static IIO_DEVICE_ATTR(measure, S_IWUSR,NULL,tmpa9xx_measure,0);

static ssize_t tmpa9xx_show_value(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_info *chip = dev_info->dev_data;
	int i=0, ret,size=0;

	for (i = 0; i < ADC_MAX_CHANNEL; i++)
		if (((chip->scan_mask) & (1<<i))==(1<<i))
                {
			ret = sprintf(buf, "channel[%d]=0x%03x\n", i, (unsigned int)(chip->values[i][0]));
			if (ret < 0)
				break;
			buf  += ret;
			size += ret;
		}

	return size;
}

static IIO_DEVICE_ATTR(value, S_IRUGO, tmpa9xx_show_value, NULL, 0);

static ssize_t tmpa9xx_show_average(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_info *chip = dev_info->dev_data;
	int i=0, ret,size=0;
        
	for (i = 0; i < ADC_MAX_CHANNEL; i++)
		if (((chip->scan_mask) & (1<<i))==(1<<i))
                {
                	unsigned long sum=0;
                        int j;
                	for (j=0;j<chip->average_base[i];j++)
                        	sum+=chip->values[i][j];
                                
			ret = sprintf(buf, "channel[%d]=0x%03x\n", i, (unsigned int)(sum/(chip->average_base[i])));
			if (ret < 0)
				break;
			buf  += ret;
			size += ret;
		}

	return size;
}

static IIO_DEVICE_ATTR(average, S_IRUGO, tmpa9xx_show_average, NULL, 0);

static ssize_t tmpa9xx_show_channel_mask(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_info *chip = dev_info->dev_data;

	return sprintf(buf, "0x%x\n", chip->scan_mask);
}

static unsigned char nr_bits_set(unsigned long mask)
{
	unsigned char count=0;
        int i;
        
        for (i=0;i<ADC_MAX_CHANNEL;i++)
        	if ((mask & (1<<i)) != 0)
                	count ++;
                        
        return count;
}
        
static unsigned char highest_bit(unsigned long mask)
{
	unsigned char pos=0;
        int i;
        
        for (i=0;i<ADC_MAX_CHANNEL;i++)
        	if ((mask & (1<<i)) != 0)
                	pos=i;
        return pos;
}        

static ssize_t tmpa9xx_store_channel_mask(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_info *chip = dev_info->dev_data;
	unsigned long mask;
        unsigned char adc_adch,adc_scan,adc_adscn;
	int ret;

	ret = strict_strtoul(buf, 16, &mask);
        
	if (ret || mask > TMPA9XX_CHANNEL_MASK)
		return -EINVAL;

	if (mask >= 0x20)	/* Above chennel 4 -> 8 channel scan */
                adc_adscn = 0x1;

	if (nr_bits_set(mask) > 1)	/* More than only one channel to scan */
                adc_scan = 0x1;

	adc_adch=highest_bit(mask);

	ADC_ADMOD1  = ((mask!=0)<<7) | (adc_adscn<<5) | adc_adch;
 	ADC_ADMOD0 |= (adc_scan << 1);

	chip->scan_mask=mask;

	/* Setup Port Multiplexing */
        GPIODFR1  =  mask ;
        GPIODFR2  = (0x00);
        GPIODIE   = (0x00);

	return len;
}

static IIO_DEVICE_ATTR(channel_mask, S_IRUGO | S_IWUSR,
		tmpa9xx_show_channel_mask,
		tmpa9xx_store_channel_mask,
		0);

static ssize_t tmpa9xx_show_name(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct tmpa9xx_adc_info *chip = dev_info->dev_data;
        
	return sprintf(buf, "%s\n", chip->name);
}

static IIO_DEVICE_ATTR(name, S_IRUGO, tmpa9xx_show_name, NULL, 0);

static struct attribute *tmpa9xx_attributes[] = {
	&iio_dev_attr_available_modes.dev_attr.attr,
	&iio_dev_attr_mode.dev_attr.attr,
	&iio_dev_attr_average_count.dev_attr.attr,
	&iio_dev_attr_reset.dev_attr.attr,
	&iio_dev_attr_measure.dev_attr.attr,
	&iio_dev_attr_value.dev_attr.attr,
	&iio_dev_attr_average.dev_attr.attr,
	&iio_dev_attr_repeat.dev_attr.attr,
	&iio_dev_attr_channel_mask.dev_attr.attr,
	&iio_dev_attr_name.dev_attr.attr,
	NULL,
};

static const struct attribute_group tmpa9xx_attribute_group = {
	.attrs = tmpa9xx_attributes,
};

static int iio_tmpa9xx_adc_probe(struct platform_device *pdev)
{
	struct tmpa9xx_adc_info *chip;
	int ret = 0;
	struct resource	*res, *mem;

	
	chip = kzalloc(sizeof(struct tmpa9xx_adc_info), GFP_KERNEL);

	if (chip == NULL)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem == NULL) {
		printk(KERN_ERR "no resource definition for memory\n");
		return -ENOENT;
	}

	if (!request_mem_region(mem->start, mem->end - mem->start + 1,
				pdev->name)) {
		printk(KERN_ERR "request_mem_region failed\n");
		return -EBUSY;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		printk(KERN_ERR "platform_get_resource() failed\n");
		ret = -ENOMEM;
		goto error_free_dev;
	}
        
	ret = request_irq(res->start, interrupt_handler, 0, DRIVER_NAME, chip );
	if (ret) {
		printk(KERN_ERR "request_irq() failed\n");
		ret = -ENOMEM;
		goto error_free_dev;
	}

	chip->indio_dev = iio_allocate_device();
	if (chip->indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_free_chip;
	}

	chip->indio_dev->dev.parent = &pdev->dev;
	chip->indio_dev->attrs = &tmpa9xx_attribute_group;
	chip->indio_dev->dev_data = (void *)chip;
	chip->indio_dev->driver_module = THIS_MODULE;
	chip->indio_dev->modes = INDIO_DIRECT_MODE;
	chip->name=DRIVER_NAME;

	chip->interval=10;
        memset(&chip->average_base[0],1,ADC_MAX_CHANNEL);
        memset(&chip->average_pos[0],0,ADC_MAX_CHANNEL);
        memset(&chip->values[0],0,sizeof(unsigned long)*ADC_MAX_CHANNEL*ADC_MAX_AVERAGE_COUNT);
                
	ret = iio_device_register(chip->indio_dev);
	if (ret)
		goto error_free_dev;

	platform_set_drvdata(pdev, chip);
	dev_info(&pdev->dev, "ADC registered.\n");

        /* Setup ADCCLK */
        ADC_ADCLK = (0x01 << 7) | FIXED_ADC_PRESCALER;

	/* Enable IRQ generation */
	ADC_ADIE  = (0x01 << 0);

	return 0;

error_free_dev:
	free_irq(res->start, &pdev->dev);
        release_mem_region(mem->start, resource_size(mem));
	iio_device_unregister(chip->indio_dev);
	iio_free_device(chip->indio_dev);
error_free_chip:
	kfree(chip);

	return ret;
}

static int __devexit iio_tmpa9xx_adc_remove(struct platform_device *pdev)
{
	struct resource *mem;
	struct tmpa9xx_adc_info *chip = platform_get_drvdata(pdev);
	struct iio_dev *indio_dev = chip->indio_dev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem == NULL) {
		printk(KERN_ERR "no resource definition for memory\n");
		return -ENOENT;
	}

        release_mem_region(mem->start, resource_size(mem));
	iio_device_unregister(indio_dev);
	iio_free_device(chip->indio_dev);
	kfree(chip);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver iio_tmpa9xx_adc_driver = {
	.probe = iio_tmpa9xx_adc_probe,
	.remove = __devexit_p(iio_tmpa9xx_adc_remove),
	.driver = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init iio_tmpa9xx_adc_init(void)
{
	return platform_driver_register(&iio_tmpa9xx_adc_driver);
}

static void __exit iio_tmpa9xx_adc_exit(void)
{
	platform_driver_unregister(&iio_tmpa9xx_adc_driver);
}

module_init(iio_tmpa9xx_adc_init);
module_exit(iio_tmpa9xx_adc_exit);

MODULE_AUTHOR("Thomas Haase <Thomas.Haase@web.de>");
MODULE_DESCRIPTION("TMPA9xx ADC for the iio subsystem");
MODULE_LICENSE("GPL v2");
