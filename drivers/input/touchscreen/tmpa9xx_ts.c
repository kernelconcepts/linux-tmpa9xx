
/*
 * TMPA9xx touchscreen driver
 *
 * Copyright (c) 2008 bplan GmbH 
 *           (c) 2010 Florian Boor <florian@kernelconconcepts.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */


#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/workqueue.h>

#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <mach/regs.h>
#include <mach/ts.h>
#include <mach/adc.h>

#define DRIVER_DESC "TMPA9xx touchscreen driver"


struct tmpa9xx_ts_priv
{
	struct tmpa9xx_ts *ts_regs;
	struct tmpa9xx_adc *adc_regs;
	struct input_dev *input_dev;
	int pen_is_down;
	int adc_pass;
	int x;
	int y;
	int ts_irq;
	int adc_irq;
	int interval;
	int start_delay;
	int conv_count;
	int skip_count;
	struct workqueue_struct *ts_workq;
	struct delayed_work scheduled_restart;
};

enum {
	ADC_PASS_NONE,
	ADC_PASS_X,
	ADC_PASS_Y,
};

#define STARTDELAY_FIRST	22
#define STARTDELAY_OTHERS	20


static void ts_init(volatile struct tmpa9xx_ts *ts)
{
	ts->tsicr0 = TMPA9XX_TS_CR0_TSI7 | TMPA9XX_TS_CR0_PYEN;
	ts->tsicr1 = 0xC5;
}

static void ts_end(volatile struct tmpa9xx_ts *ts)
{
	ts->tsicr0 = 0;
	ts->tsicr1 = 0;
}

static inline int ts_pen_is_down(volatile struct tmpa9xx_ts *ts)
{
	return ts->tsicr0 & TMPA9XX_TS_CR0_PTST ? 1 : 0;
}

static inline void ts_clear_interrupt(void)
{
	_out32(PORTD_GPIOIC, _in32(PORTD_GPIOMIS) );
}

static inline void ts_enable_interrupt(volatile struct tmpa9xx_ts *ts)
{
	ts->tsicr0 |= TMPA9XX_TS_CR0_TWIEN;
}

static inline void ts_disable_interrupt(volatile struct tmpa9xx_ts *ts)
{
	ts->tsicr0 &= ~TMPA9XX_TS_CR0_TWIEN;
}

static void enable_interrupt(struct tmpa9xx_ts_priv *tmpa9xx_ts_priv)
{
	struct tmpa9xx_adc *adc;

	adc = tmpa9xx_ts_priv->adc_regs;

	_out32(PORTD_GPIOIE, 0xc0);
	adc->adie = 0x01;
	ts_enable_interrupt(tmpa9xx_ts_priv->ts_regs);
}

static void disable_interrupt(struct tmpa9xx_ts_priv *tmpa9xx_ts_priv)
{
	struct tmpa9xx_adc *adc;

	adc = tmpa9xx_ts_priv->adc_regs;

	_out32(PORTD_GPIOIE, 0x00);
	adc->adie = 0x00;

	ts_disable_interrupt(tmpa9xx_ts_priv->ts_regs);
}

static void adc_startchannel(struct tmpa9xx_ts_priv *tmpa9xx_ts_priv, int chn )
{
	volatile struct tmpa9xx_adc *adc;
	uint32_t admod1;

	adc = tmpa9xx_ts_priv->adc_regs;;

	admod1  = adc->admod1;
	admod1 &= 0xf8;
	admod1 |= chn & 0x7;

	adc->admod1 = admod1;

	/* It seens to be required to obtain aquerate value */
	udelay(tmpa9xx_ts_priv->start_delay);
	
	adc->admod0 = 0x1;
}

/* X in on AN5 */
static int ts_read_x(volatile struct tmpa9xx_ts *ts, volatile struct tmpa9xx_adc *adc )
{
	return	  ( (adc->adreg5l >> 6) & 0x003)
		| ( (adc->adreg5h << 2) & 0x3FC);
}

/* Y in on AN4 */
static int ts_read_y(volatile struct tmpa9xx_ts *ts, volatile struct tmpa9xx_adc *adc )
{
	return	  ( (adc->adreg4l >> 6) & 0x003)
		| ( (adc->adreg4h << 2) & 0x3FC);
}

static void ts_update_pendown(struct tmpa9xx_ts_priv *tmpa9xx_ts_priv, int pen_is_down)
{
	void *input_dev;
	input_dev = tmpa9xx_ts_priv->input_dev;

	input_report_key(input_dev, BTN_TOUCH, pen_is_down );
	input_report_abs(input_dev, ABS_PRESSURE, pen_is_down);
	input_sync(input_dev);

	tmpa9xx_ts_priv->pen_is_down = pen_is_down;
}

static void ts_update_pos(struct tmpa9xx_ts_priv *tmpa9xx_ts_priv)
{
	void *input_dev;
	int skip_count;

	skip_count = tmpa9xx_ts_priv->skip_count;

	if ((skip_count > 0) && (tmpa9xx_ts_priv->conv_count<skip_count)) {
		tmpa9xx_ts_priv->conv_count++;
		return;
	}

	tmpa9xx_ts_priv->conv_count++;

	input_dev = tmpa9xx_ts_priv->input_dev;

	input_report_abs(input_dev, ABS_X, tmpa9xx_ts_priv->x); 
	input_report_abs(input_dev, ABS_Y, tmpa9xx_ts_priv->y); 

	if (tmpa9xx_ts_priv->start_delay == STARTDELAY_FIRST) {
		/* for the first time, report the rpeuse as well */
		ts_update_pendown(tmpa9xx_ts_priv, 1);
		tmpa9xx_ts_priv->start_delay = STARTDELAY_OTHERS;
	}
	else {
		input_sync(input_dev);
	}
}

static void ts_start_convertion(struct tmpa9xx_ts_priv *tmpa9xx_ts_priv)
{
	volatile struct tmpa9xx_adc *adc = tmpa9xx_ts_priv->adc_regs;
	volatile struct tmpa9xx_ts *ts = tmpa9xx_ts_priv->ts_regs;

	int adc_pass;
	void *input_dev;
	
	adc = tmpa9xx_ts_priv->adc_regs;
	ts  = tmpa9xx_ts_priv->ts_regs;

	if (tmpa9xx_ts_priv->pen_is_down == 0) {
		tmpa9xx_ts_priv->adc_pass = ADC_PASS_NONE;

		ts_init(ts);
		ts_enable_interrupt(ts);

		return;
	}
	
	adc_pass = tmpa9xx_ts_priv->adc_pass;
	
	switch(adc_pass) {
		case ADC_PASS_NONE:
			/* no oconvm start x */
			ts->tsicr0 = 0xc5;
			adc_startchannel(tmpa9xx_ts_priv, 5);
			adc_pass = ADC_PASS_X;
			break;
			
		case ADC_PASS_X:
			/* x done, start y */
			ts->tsicr0 = 0xca;
			adc_startchannel(tmpa9xx_ts_priv, 4);
			adc_pass = ADC_PASS_Y;
			break;
			
		case ADC_PASS_Y:
			input_dev = tmpa9xx_ts_priv->input_dev;

			/* Conversion done, report the abs pos and tell input */
			ts_update_pos(tmpa9xx_ts_priv);

			/* Set status NONE because we are done with this conversion */
			adc_pass  = ADC_PASS_NONE;

			/* disable, clear interrupts and restart the ts controller */
			ts_disable_interrupt(ts);
			ts_clear_interrupt();
			ts_init(ts);

			/* Start again pen detection but in a little while */
			queue_delayed_work(tmpa9xx_ts_priv->ts_workq, &tmpa9xx_ts_priv->scheduled_restart, tmpa9xx_ts_priv->interval);
			
			break;
	
	}
	
	tmpa9xx_ts_priv->adc_pass = adc_pass;
}

static void ts_scheduled_restart(struct work_struct *work)
{
	int pen_is_down;
	void *input_dev;
	struct tmpa9xx_ts_priv *tmpa9xx_ts_priv = container_of(work, struct tmpa9xx_ts_priv , scheduled_restart.work);

	input_dev = tmpa9xx_ts_priv->input_dev;

	pen_is_down = ts_pen_is_down(tmpa9xx_ts_priv->ts_regs);
	tmpa9xx_ts_priv->adc_pass = ADC_PASS_NONE;

	ts_enable_interrupt(tmpa9xx_ts_priv->ts_regs);

	if (pen_is_down) {
		ts_start_convertion(tmpa9xx_ts_priv);
	}
	else {
		ts_update_pendown(tmpa9xx_ts_priv, pen_is_down);
	}
}

static irqreturn_t topas910_ts_interrupt(int irq, void *dev_id)
{
	struct tmpa9xx_ts_priv *tmpa9xx_ts_priv = dev_id;
	struct tmpa9xx_ts *ts;
	int pen_is_down;

	ts  = tmpa9xx_ts_priv->ts_regs;
			
	ts_clear_interrupt();
	ts_disable_interrupt(ts);

	tmpa9xx_ts_priv->start_delay = STARTDELAY_FIRST;
	tmpa9xx_ts_priv->conv_count  = 0;
	
	pen_is_down = ts_pen_is_down(ts);

	tmpa9xx_ts_priv->pen_is_down = pen_is_down;

	/* Start the conversion in a little while */
	ts_start_convertion(tmpa9xx_ts_priv);

	return IRQ_HANDLED;
}

static irqreturn_t topas910_adc_interrupt(int irq, void *dev_id)
{
	struct tmpa9xx_ts_priv *tmpa9xx_ts_priv = dev_id;
	struct tmpa9xx_ts *ts;
	struct tmpa9xx_adc *adc;
	uint32_t onereg;
	
	adc = tmpa9xx_ts_priv->adc_regs;
	ts  = tmpa9xx_ts_priv->ts_regs;

	onereg = adc->admod0;
	if ((onereg & 0x80) == 0) {
		printk(KERN_WARNING "Warning! Convertion not finished\n");
	}

	onereg = adc->adis;
	if (onereg == 0) {
		printk(KERN_ERR "Supurious interrupt ? adis=0x%x\n", adc->adis);
		return IRQ_NONE;
	}	

	adc->adic = onereg;
	
	switch(tmpa9xx_ts_priv->adc_pass) {
		case ADC_PASS_NONE:
			/* strange better renable the ctrl */
			ts_init(ts);
			ts_enable_interrupt(ts);
			break;
			
		case ADC_PASS_X:
			/* X done, retreive the pos and continue */
			tmpa9xx_ts_priv->x = ts_read_x(ts, adc);
			ts_start_convertion(tmpa9xx_ts_priv);
			break;
			
		case ADC_PASS_Y:
			/* Y done, continue the adventure */
			tmpa9xx_ts_priv->y = ts_read_y(ts, adc);

			ts_start_convertion(tmpa9xx_ts_priv);
			break;
	}
	
	return IRQ_HANDLED;
}

static void ts_free_priv(struct tmpa9xx_ts_priv *tmpa9xx_ts_priv)
{
	struct tmpa9xx_adc *adc;
	struct tmpa9xx_ts *ts;

	if (!tmpa9xx_ts_priv)
		return;

	adc = tmpa9xx_ts_priv->adc_regs;
	ts  = tmpa9xx_ts_priv->ts_regs;
	if ( ts && adc )

	disable_interrupt(tmpa9xx_ts_priv);

	/* NO VRef */
	if (adc)
		adc->admod1 &= ~0x80;
	if (ts)
		ts_end(ts);
			
	if (tmpa9xx_ts_priv->input_dev)
		input_free_device(tmpa9xx_ts_priv->input_dev);

	if (tmpa9xx_ts_priv->ts_workq) {	
		cancel_delayed_work_sync(&tmpa9xx_ts_priv->scheduled_restart);
		destroy_workqueue(tmpa9xx_ts_priv->ts_workq);
	}

	if (tmpa9xx_ts_priv->adc_irq != NO_IRQ)
		free_irq(tmpa9xx_ts_priv->adc_irq, tmpa9xx_ts_priv);

	if (tmpa9xx_ts_priv->ts_irq != NO_IRQ)
		free_irq(tmpa9xx_ts_priv->ts_irq, tmpa9xx_ts_priv);

	kfree(tmpa9xx_ts_priv);
}

static int __devinit tmpa9xx_ts_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_ts_priv *tmpa9xx_ts_priv = NULL;
	struct input_dev *input_dev = NULL;
	int err;
	struct resource *ts_r = NULL;
	struct resource *adc_r = NULL;
	int ts_irq;
	int adc_irq;
	int ret;
	int rate;
	int skip_count;
	int fuzz;
	struct tmpa9xx_adc *adc;
	struct tmpa9xx_ts_platforminfo *tmpa9xx_ts_platforminfo;

	tmpa9xx_ts_platforminfo = (struct tmpa9xx_ts_platforminfo *) dev->platform_data;
	if (tmpa9xx_ts_platforminfo) {
		fuzz       = tmpa9xx_ts_platforminfo->fuzz;
		rate       = tmpa9xx_ts_platforminfo->rate;
		skip_count = tmpa9xx_ts_platforminfo->skip_count;
	}
	else {
		fuzz       = TMPA9XX_TS_DEFAULT_FUZZ;
		rate       = TMPA9XX_TS_DEFAULT_RATE;
		skip_count = TMPA9XX_TS_DEFAULT_SKIP_COUNT;
	}

	ts_r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!ts_r) {
		printk(KERN_ERR "resources unusable\n");
		ret = -ENXIO;
		return ret;
	}
		
	adc_r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!adc_r) {
		printk(KERN_ERR "resources unusable\n");
		ret = -ENXIO;
		return ret;
	}
	
	ts_irq  = platform_get_irq(pdev, 0);
	if (ts_irq == NO_IRQ) {
		printk(KERN_ERR "platform_get_irq 0 failed\n");
		ret = -ENXIO;
		return ret;
	}

	adc_irq  = platform_get_irq(pdev, 1);
	if (adc_irq == NO_IRQ) {
		printk(KERN_ERR "platform_get_irq 1 failed\n");
		ret = -ENXIO;
		return ret;
	}

	/* Now allocate some memory for our private handle */
	tmpa9xx_ts_priv = kzalloc(sizeof(struct tmpa9xx_ts_priv), GFP_KERNEL);
	if (tmpa9xx_ts_priv==NULL) {
		err = -ENOMEM;
		return err;
	}
	
	/* set the private handle to safe defaults */
	tmpa9xx_ts_priv->pen_is_down = 0;
	tmpa9xx_ts_priv->adc_pass    = ADC_PASS_NONE;
	tmpa9xx_ts_priv->interval    = HZ/rate;
	tmpa9xx_ts_priv->adc_irq     = NO_IRQ;
	tmpa9xx_ts_priv->ts_irq      = NO_IRQ;
	tmpa9xx_ts_priv->skip_count  = skip_count;

	tmpa9xx_ts_priv->ts_workq    = NULL;

	adc                          = (void *) adc_r->start;
	tmpa9xx_ts_priv->adc_regs    = adc;
	tmpa9xx_ts_priv->ts_regs     = (void *) ts_r->start;

	disable_interrupt(tmpa9xx_ts_priv);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		goto fail;
	}
	tmpa9xx_ts_priv->input_dev = input_dev;

	input_dev->name       = DRIVER_DESC;
	input_dev->phys       = (void *) tmpa9xx_ts_priv->ts_regs;
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor  = 0;
	input_dev->id.product = 0;
	input_dev->id.version = 0x0100;
	input_dev->evbit[0]   = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	tmpa9xx_ts_priv->ts_workq = create_singlethread_workqueue("tmpa9xx_ts");
	if (tmpa9xx_ts_priv->ts_workq == NULL) {
		printk(KERN_ERR "Failed to create workqueue\n");
		err = -ENOMEM;
		goto fail;
	}

	INIT_DELAYED_WORK(&tmpa9xx_ts_priv->scheduled_restart, ts_scheduled_restart);
	
	ret = request_irq(ts_irq, topas910_ts_interrupt, IRQF_SHARED, "ts_ts", tmpa9xx_ts_priv);
	if (ret) {
		printk(KERN_ERR "Fail allocate the interrupt (vector=%d), %i\n", ts_irq, ret );
		err = -ENOMEM;
		goto fail;
	}
	tmpa9xx_ts_priv->ts_irq = ts_irq;

	ret = request_irq(adc_irq, topas910_adc_interrupt, IRQF_SHARED, "ts_adc", tmpa9xx_ts_priv);
	if (ret) {
		printk(KERN_ERR "Fail allocate the interrupt (vector=%d)\n", adc_irq );
		err = -ENOMEM;
		goto fail;
	}
	tmpa9xx_ts_priv->adc_irq = adc_irq;


	/* TMPA9xx ADC is 10bit */
	input_set_abs_params(input_dev, ABS_X, 0, 1024, fuzz, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 1024, fuzz, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1, 0, 0);

	err = input_register_device(tmpa9xx_ts_priv->input_dev);
	if (err) {
		printk(KERN_ERR "input_register_device failed (err=%d)\n", err );
		goto fail;
	}

	input_report_abs(input_dev, ABS_PRESSURE, 0);
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);
	
	printk(KERN_INFO DRIVER_DESC " (fuzz=%d, rate=%d Hz, skip=%d) ready\n",
		fuzz, rate, tmpa9xx_ts_priv->skip_count);

	dev_set_drvdata(dev, tmpa9xx_ts_priv);
	ts_init(tmpa9xx_ts_priv->ts_regs);

	adc->adclk  = 0x82;
	adc->admod1 |= 0x80;

	enable_interrupt(tmpa9xx_ts_priv);

	return 0;


 fail:
	ts_free_priv (tmpa9xx_ts_priv);

	return err;
	

}

static int __devexit tmpa9xx_ts_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_ts_priv *tmpa9xx_ts_priv;

	tmpa9xx_ts_priv = (struct tmpa9xx_ts_priv *) dev_get_drvdata(dev);
	
	ts_free_priv(tmpa9xx_ts_priv);

	dev_set_drvdata(dev, NULL);

	return 0;
}


#ifdef CONFIG_PM

static int tmpa9xx_ts_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_ts_priv *tmpa9xx_ts_priv;
	struct tmpa9xx_adc *adc;

	tmpa9xx_ts_priv = (struct tmpa9xx_ts_priv *) dev_get_drvdata(dev);
	adc = tmpa9xx_ts_priv->adc_regs;

	adc->admod1 &= ~0x80;

	return 0;
}

static int tmpa9xx_ts_resume(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_ts_priv *tmpa9xx_ts_priv;
	struct tmpa9xx_adc *adc;

	tmpa9xx_ts_priv = (struct tmpa9xx_ts_priv *) dev_get_drvdata(dev);
	adc = tmpa9xx_ts_priv->adc_regs;

	adc->admod1 |= 0x80;

	return 0;
}

#else
#define tmpa9xx_ts_suspend	NULL
#define tmpa9xx_ts_resume	NULL
#endif

static struct platform_driver tmpa9xx_ts_driver = {
	.probe		= tmpa9xx_ts_probe,
	.remove		= __devexit_p(tmpa9xx_ts_remove),
	.suspend	= tmpa9xx_ts_suspend,
	.resume		= tmpa9xx_ts_resume,

	.driver		= {
		.name	= "tmpa9xx-ts",
		.owner	= THIS_MODULE,
	},
};

static int __init tmpa9xx_ts_init(void)
{
	return platform_driver_register(&tmpa9xx_ts_driver);
}

static void __exit tmpa9xx_ts_exit(void)
{
	platform_driver_unregister(&tmpa9xx_ts_driver);
}

module_init(tmpa9xx_ts_init);
module_exit(tmpa9xx_ts_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florian Boor <florian@kernelconcepts.de>");
MODULE_DESCRIPTION(DRIVER_DESC);

