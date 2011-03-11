/*
 * drivers/i2c/busses/i2c-tmpa9xx.c
 *
 * Provides I2C support for Toshiba TMPA9xx
 *
 * Copyright (c) 2009 bplan GmbH
 * Copyright (c) 2011 Michael Hunold <michael@mihu.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/i2c.h>

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/slab.h>

struct tmpa9xx_i2c_regs {
	uint32_t i2c_cr1;	// 0x0000 I2C Control Register 1
	uint32_t i2c_dbr;	// 0x0004 I2C Data Buffer Register
	uint32_t i2c_ar;	// 0x0008 I2C (Slave) Address Register
	uint32_t i2c_cr2;	// 0x000C I2C Control Register 2(Write)/Status(Read)
	uint32_t i2c_prs;	// 0x0010 I2C Prescaler Clock Set Register
	uint32_t i2c_ie;	// 0x0014 I2C Interrupt Enable Register
	uint32_t i2c_ir;	// 0x0018 I2C Interrupt Register
};

#define i2c_sr i2c_cr2		// reading cr2 reads the status

struct tmpa9xx_i2c_priv {
	struct tmpa9xx_i2c_regs *regs;
	struct device *dev;
	struct i2c_adapter *i2c_adapter;
	int irq;
};

#ifdef DEBUG
static void tmpa9xx_i2c_dump_regs(struct i2c_adapter *adap)
{
	struct tmpa9xx_i2c_priv *priv = adap->algo_data;
	volatile struct tmpa9xx_i2c_regs __iomem *regs = priv->regs;

	printk("I2C controller at %p\n", regs);
	printk(" I2C%dCR1 (0x%02x) = 0x%02x\n", algo->channel, offsetof(struct tmpa9xx_i2c_regs, i2c_cr1), regs->i2c_cr1);
	printk(" I2C%dDBR (0x%02x) = 0xXX\n", algo->channel,   offsetof(struct tmpa9xx_i2c_regs, i2c_dbr));
	printk(" I2C%dAR  (0x%02x) = 0x%02x\n", algo->channel, offsetof(struct tmpa9xx_i2c_regs, i2c_ar), regs->i2c_ar);
	printk(" I2C%dSR  (0x%02x) = 0x%02x\n", algo->channel, offsetof(struct tmpa9xx_i2c_regs, i2c_sr), regs->i2c_sr);
	printk(" I2C%dCR2 (0x%02x) = 0xXX\n", algo->channel,   offsetof(struct tmpa9xx_i2c_regs, i2c_cr2));
	printk(" I2C%dPRS (0x%02x) = 0x%02x\n", algo->channel, offsetof(struct tmpa9xx_i2c_regs, i2c_prs), regs->i2c_prs);
	printk(" I2C%dIE  (0x%02x) = 0x%02x\n", algo->channel, offsetof(struct tmpa9xx_i2c_regs, i2c_ie), regs->i2c_ie);
	printk(" I2C%dIR  (0x%02x) = 0x%02x\n", algo->channel, offsetof(struct tmpa9xx_i2c_regs, i2c_ir), regs->i2c_ir);
}
#else
static void tmpa9xx_i2c_dump_regs(struct i2c_adapter *adap)
{
}
#endif

/* #define USE_UDELAY */

static int tmpa9xx_i2c_wait_status_timeout(struct i2c_adapter *adap,
				    uint32_t mask, uint32_t val)
{
	struct tmpa9xx_i2c_priv *priv = adap->algo_data;
	volatile struct tmpa9xx_i2c_regs __iomem *regs = priv->regs;
#ifdef USE_UDELAY
	volatile int timeout = 1000;
#else
	volatile int timeout = 1000 * 1000;
#endif
	volatile int sr;

	while (((sr = regs->i2c_sr) & mask) != val) {
#ifdef USE_UDELAY
		udelay(10);
#endif
		if (timeout-- < 0) {
			/* tmpa9xx_i2c_dump_regs(adap); */
			return -1;
		}
	}

	return 0;
}

static int tmpa9xx_i2c_wait_free_bus(struct i2c_adapter *adap)
{
	return tmpa9xx_i2c_wait_status_timeout(adap, (1UL << 5), 0);	// bus state == free ?
}

static int tmpa9xx_i2c_wait_done(struct i2c_adapter *adap)
{
	return tmpa9xx_i2c_wait_status_timeout(adap, (1UL << 4), 0);	// SCL line == low ?
}

static int tmpa9xx_i2c_wait_lrb_set(struct i2c_adapter *adap)
{
	return tmpa9xx_i2c_wait_status_timeout(adap, (1UL << 0), 1);	// last received bit == high ?
}

static int tmpa9xx_i2c_restart(struct i2c_adapter *adap)
{
	struct tmpa9xx_i2c_priv *priv = adap->algo_data;
	volatile struct tmpa9xx_i2c_regs __iomem *regs = priv->regs;

	regs->i2c_cr2 = 0
	    | (1UL << 4)		/* clear service request */
	    | (1UL << 3)		/* enable I2C operation */
	    ;

	if (tmpa9xx_i2c_wait_free_bus(adap) < 0) {
		dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_wait_free_bus() failed\n", __func__);
		return -EBUSY;
	}

	if (tmpa9xx_i2c_wait_lrb_set(adap) < 0) {
		dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_wait_lrb_set() failed\n", __func__);
		return -EBUSY;
	}
	
	/* see specification, "Data Transfer Procedure in I2C Bus Mode",
	   section 5 "Restart procedure" */
	udelay(5);

	return 0;
}

static int tmpa9xx_i2c_start(struct i2c_adapter *adap, int slave_adr, int is_read)
{
	struct tmpa9xx_i2c_priv *priv = adap->algo_data;
	volatile struct tmpa9xx_i2c_regs __iomem *regs = priv->regs;

	if (tmpa9xx_i2c_wait_free_bus(adap) < 0) {
		dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_wait_free_bus() failed\n", __func__);
		return -EBUSY;
	}

	regs->i2c_cr1 |= (1UL << 4);	/* enable acknowledge clock */

	regs->i2c_dbr = (slave_adr << 1) | !!is_read;	/* send slave address */

	regs->i2c_cr2 = (1UL << 7)	/* select master mode */
	    | (1UL << 6)		/* transmit operation */
	    | (1UL << 5)		/* generate start condition */
	    | (1UL << 4)		/* clear service request */
	    | (1UL << 3)		/* enable I2C operation */
	    ;

	if (tmpa9xx_i2c_wait_done(adap) < 0) {
		dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_wait_done() failed\n", __func__);
		return -ETIMEDOUT;
	}
	return 0;
}

static int tmpa9xx_i2c_stop(struct i2c_adapter *adap)
{
	struct tmpa9xx_i2c_priv *priv = adap->algo_data;
	volatile struct tmpa9xx_i2c_regs __iomem *regs = priv->regs;

	regs->i2c_cr2 = (1UL << 7)	/* select master mode */
	    | (1UL << 6)	/* transmit operation */
	    | (0UL << 5)	/* generate stop condition */
	    | (1UL << 4)	/* clear service request */
	    | (1UL << 3)	/* enable I2C operation */
	    ;

	if (tmpa9xx_i2c_wait_free_bus(adap) < 0) {
		dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_wait_free_bus() failed\n", __func__);
		return -EBUSY;
	}

	return 0;
}

static int tmpa9xx_i2c_xmit(struct i2c_adapter *adap, struct i2c_msg *msg)
{
	struct tmpa9xx_i2c_priv *priv = adap->algo_data;
	volatile struct tmpa9xx_i2c_regs __iomem *regs = priv->regs;

	int ret;
	unsigned int sr, cr1;
	int i;
	u8 *data;

	data = msg->buf;

	sr = regs->i2c_sr;

	if (sr & (1UL << 0)) {	/* check last received bit (should be low for ACK) */
		dev_dbg(&adap->dev, "%s(): ack check failed\n", __func__);
		tmpa9xx_i2c_dump_regs(adap);
		return -EIO;
	}

	if ((sr & (1UL << 6)) == 0) {	/* check xmit/rcv selection state (should be xmit) */
		dev_dbg(&adap->dev, "%s(): wrong transfer state\n", __func__);
		return -EIO;
	}

	cr1 = regs->i2c_cr1;
	cr1 &= ~(7UL << 5);	/* 8bit transfer */
	cr1 |= (1UL << 4);	/* acknowledge */
	regs->i2c_cr1 = cr1;

	for (i = 0; i < msg->len; i++) {

		dev_dbg(&adap->dev, "%s(): ... %d ...\n", __func__, i);

		regs->i2c_dbr = data[i] & 0xFF;	/* put 8bits into xmit FIFO */

		if (tmpa9xx_i2c_wait_done(adap) < 0) {
			dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_wait_done() failed\n", __func__);
			return -ETIMEDOUT;
		}

	}

	return ret;
}

static int tmpa9xx_i2c_rcv(struct i2c_adapter *adap, struct i2c_msg *msg)
{
	struct tmpa9xx_i2c_priv *priv = adap->algo_data;
	volatile struct tmpa9xx_i2c_regs __iomem *regs = priv->regs;
	int ret;
	unsigned int sr, cr1;
	int i, dummy;
	u8 *data;

	data = msg->buf;

	sr = regs->i2c_sr;

	if (sr & (1UL << 0)) { /* check last received bit (should be low for ACK) */
		dev_dbg(&adap->dev, "%s(): no ack from slave\n", __func__);
		/* tmpa9xx_i2c_dump_regs(adap); */
		return -EIO;
	}

	if (sr & (1UL << 6)) {	/* check xmit/rcv selection state (should be rcv) */
		dev_dbg(&adap->dev, "%s(): wrong transfer state\n", __func__);
		return -EIO;
	}
	/* read receive data */
	dummy = regs->i2c_dbr;

	cr1 = regs->i2c_cr1;
	cr1 &= ~((1UL << 4) | (7UL << 5));	/* 8bit transfer, no ACK */
	if (msg->len > 1) {
		cr1 |= (1UL << 4);      /* do ACK */
	}
	regs->i2c_cr1 = cr1;

	/* write dummy data to set PIN to 1 */
	regs->i2c_dbr = 0x00;

	for (i = 0; i < msg->len; i++) {

		ret = tmpa9xx_i2c_wait_status_timeout(adap, (1UL << 4), (1UL << 4));	// SCL line = free ? ?
		if (ret < 0) {
			dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_wait_status_timeout() @ initial free failed\n", __func__);
			break;
		}

		if (tmpa9xx_i2c_wait_done(adap) < 0) {
			dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_wait_done() failed\n", __func__);
			return -ETIMEDOUT;
		}

		data[i] = regs->i2c_dbr;

		cr1 = regs->i2c_cr1;
		if (i + 2 < msg->len) {
	        	cr1 |= (1UL << 4);                      /* send ack */
		}
		else if (i + 1 < msg->len) {
	        	cr1 &= ~(1UL << 4);                     /* send nack */
		}
		else {
	        	cr1 &= ~((1UL << 4) | (7UL << 5));      /* clear no of xfer bits, no ACK */
	        	cr1 |= (1UL << 5);                      /* xfer bits = 1 */
		}
		regs->i2c_cr1 = cr1;

		/* write dummy data to issue the ack */
		regs->i2c_dbr = 0;

		/* wait until 1bit xfer is complete */
		ret = tmpa9xx_i2c_wait_status_timeout(adap, (1UL << 4), (1UL << 4));	// SCL line = free ? ?
		if (ret < 0) {
			dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_wait_status_timeout() @ wait 1bit xfer failed\n", __func__);
			break;
		}
	}

	return ret;
}

static int tmpa9xx_i2c_setup(struct i2c_adapter *adap);
/*
 * Generic I2C master transfer entrypoint
 */
static int tmpa9xx_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			    int num)
{
	int i;
	int msgcnt = 0;
	int rw_direction = -1;
	int ret;
	
	dev_dbg(&adap->dev, "%s(): num %d\n", __func__, num);

	if (tmpa9xx_i2c_wait_free_bus(adap) < 0) {
		dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_wait_free_bus() failed\n", __func__);
		tmpa9xx_i2c_setup(adap);
	}

	for (i = 0; i < num; i++) {
		struct i2c_msg *msg = &msgs[i];
		int is_read = !!(msg->flags & I2C_M_RD);

		dev_dbg(&adap->dev, "%s(): msg %p, msg->buf 0x%x, msg->len 0x%x, msg->flags 0x%x\n", __func__, msg, (unsigned int)msg->buf, msg->len, msg->flags);

		if (rw_direction != is_read) {
			if (i) {
				dev_dbg(&adap->dev, "%s(): sending restart condition\n", __func__);
	
				ret = tmpa9xx_i2c_restart(adap);
				if (ret < 0) {
					dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_restart() failed\n", __func__);
					/* keep ret */
					goto out;
				}
			}

			ret = tmpa9xx_i2c_start(adap, msg->addr, msg->flags & I2C_M_RD);
			if (ret < 0) {
				dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_start() failed\n", __func__);
				/* keep ret */
				goto out;
			}

			if (tmpa9xx_i2c_wait_done(adap) < 0) {
				dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_wait_done() failed\n", __func__);
				ret = -ETIMEDOUT;
				goto out;
			}
	
			rw_direction = is_read;
		}

		if (is_read)
			ret = tmpa9xx_i2c_rcv(adap, msg);
		else
			ret = tmpa9xx_i2c_xmit(adap, msg);

		if (ret) {
			/* keep ret */
			goto out;
		}

		msgcnt++;
	}

out:
	i = tmpa9xx_i2c_stop(adap);
	if (i < 0) {
		dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_stop() failed\n", __func__);
		return i;
	}

	return ret ? ret : msgcnt;
}

/*
 * serial clock rate = PCLK / (Prescaler*(2**(2+sck)+16))
 *
 * 400khz for PCLK = 96MHz
 * 400 = 96*1000 / ( 12 * (2**(2+0) + 16))
 */
#define PRSCK_400KHZ 12
#define CR1SCK_400KHZ 0
/*
 * 100khz for PCLK = 96MHz
 * 100 = 96*1000 / ( 30 * (2**(2+2) + 16))
 */
#define PRSCK_100KHZ 30
#define CR1SCK_100KHZ 2

static int tmpa9xx_i2c_setup(struct i2c_adapter *adap)
{
	struct tmpa9xx_i2c_priv *priv = adap->algo_data;
	volatile struct tmpa9xx_i2c_regs __iomem *regs = priv->regs;

	/* software reset */
	regs->i2c_cr2 = (1UL << 1) | (0UL << 0);
	regs->i2c_cr2 = (0UL << 1) | (1UL << 0);

	regs->i2c_ar = 0;

	/* setup scalers to 100khz default */
	regs->i2c_prs = PRSCK_100KHZ;
	regs->i2c_cr1 = CR1SCK_100KHZ;

	/* enable i2c operation */
	regs->i2c_cr2 = (1UL << 3);

	/* tmpa9xx_i2c_dump_regs(algo); */

	return 0;
}

static int tmpa9xx_i2c_shutdown(struct i2c_adapter *adap)
{
	struct tmpa9xx_i2c_priv *priv = adap->algo_data;
	volatile struct tmpa9xx_i2c_regs __iomem *regs = priv->regs;

	if(tmpa9xx_i2c_wait_free_bus(adap) < 0) {
		dev_dbg(&adap->dev, "%s(): tmpa9xx_i2c_wait_free_bus() failed\n", __func__);
	}

	regs->i2c_prs = 0;
	regs->i2c_cr1 = 0;

	/* disable i2c operation */
	regs->i2c_cr2 = (0UL << 3);

	return 0;
}

static u32 tmpa9xx_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm tmpa9xx_algorithm = {
	.master_xfer = tmpa9xx_i2c_xfer,
	.functionality = tmpa9xx_i2c_func,
};

static irqreturn_t interrupt_handler(int irq, void *ptr)
{
	struct tmpa9xx_i2c_priv *priv = ptr;
	volatile struct tmpa9xx_i2c_regs __iomem *regs = priv->regs;
	struct device *dev = priv->dev;

/*
	dev_err(dev, "irq\n");
*/

	regs->i2c_ir = 0x1;

	return IRQ_HANDLED;
}

static int __devinit tmpa9xx_i2c_probe(struct platform_device *pdev)
{
	struct tmpa9xx_i2c_priv *priv;
	struct i2c_adapter *adapter;
	struct resource *res;
	int ret;

	priv = kzalloc(sizeof(struct tmpa9xx_i2c_priv), GFP_KERNEL);
	if (!priv) {
		dev_dbg(&pdev->dev, "kzalloc() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	priv->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_dbg(&pdev->dev, "platform_get_resource() failed\n");
		ret = -ENODEV;
		goto err1;
	}

	res = request_mem_region(res->start, res->end - res->start + 1, pdev->name);
	if (!res) {
		dev_dbg(&pdev->dev, "request_mem_region() failed\n");
		ret = -ENODEV;
		goto err2;
	}

	priv->regs = ioremap(res->start, res->end - res->start + 1);
	if (!priv->regs) {
		dev_dbg(&pdev->dev, "ioremap() failed\n");
		ret = -ENOMEM;
		goto err3;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		dev_dbg(&pdev->dev, "platform_get_irq() failed\n");
		ret = -ENODEV;
		goto err4;
	}

	ret = request_irq(priv->irq, interrupt_handler, IRQF_DISABLED, "tmpa9xx-i2c", priv);
	if (ret) {
		dev_dbg(&pdev->dev, "platform_get_irq() failed\n");
		ret = -ENODEV;
		goto err5;
	}

	adapter = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
	if (!adapter) {
		dev_dbg(&pdev->dev, "kzalloc() failed\n");
		ret = -ENODEV;
		goto err6;
	}

	sprintf(adapter->name, "tmpa9xx_i2c%d", pdev->id);
	adapter->algo = &tmpa9xx_algorithm;
	adapter->algo_data = priv;
	adapter->class = I2C_CLASS_HWMON;
	adapter->dev.parent = &pdev->dev;
	adapter->id = 0;
	adapter->nr = pdev->id;
	priv->i2c_adapter = adapter;

	tmpa9xx_i2c_setup(adapter);

	ret = i2c_add_numbered_adapter(priv->i2c_adapter);
	if (ret) {
		dev_dbg(&pdev->dev, "i2c_add_numbered_adapter() failed\n");
		ret = -ENODEV;
		goto err7;
	}

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "channel %d, irq %d, io @ %p\n", pdev->id, priv->irq, priv->regs);

	priv->regs->i2c_ie = 0x1;

	return 0;

err7:
	tmpa9xx_i2c_shutdown(adapter);
	kfree(adapter);
err6:
	free_irq(priv->irq, priv);
err5:
err4:
	iounmap(priv->regs);
err3:
	release_mem_region(res->start, res->end - res->start + 1);
err2:
err1:
	kfree(priv);
err0:
	return ret;
}

static int __devexit tmpa9xx_i2c_remove(struct platform_device *pdev)
{
	struct tmpa9xx_i2c_priv *priv = platform_get_drvdata(pdev);
	struct i2c_adapter *adapter = priv->i2c_adapter;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	BUG_ON(!res);

	priv->regs->i2c_ie = 0x0;

	i2c_del_adapter(adapter);
	tmpa9xx_i2c_shutdown(adapter);
	kfree(adapter);
	iounmap(priv->regs);
	free_irq(priv->irq, priv);
	release_mem_region(res->start, res->end - res->start + 1);
	platform_set_drvdata(pdev, NULL);
	kfree(priv);

	return 0;
}

#ifdef CONFIG_PM
static int tmpa9xx_i2c_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int tmpa9xx_i2c_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define tmpa9xx_i2c_suspend NULL
#define tmpa9xx_i2c_resume  NULL
#endif

static struct platform_driver tmpa9xx_i2c_driver = {
	.probe = tmpa9xx_i2c_probe,
	.remove = __devexit_p(tmpa9xx_i2c_remove),
	.suspend = tmpa9xx_i2c_suspend,
	.resume = tmpa9xx_i2c_resume,
	.driver = {
		   .name = "tmpa9xx-i2c",
		   .owner = THIS_MODULE,
		   },
};

static int __init tmpa9xx_i2c_init(void)
{
	return platform_driver_register(&tmpa9xx_i2c_driver);
}

module_init(tmpa9xx_i2c_init);

static void __exit tmpa9xx_i2c_exit(void)
{
	platform_driver_unregister(&tmpa9xx_i2c_driver);
}

module_exit(tmpa9xx_i2c_exit);

MODULE_DESCRIPTION("Toshiba TMPA9xx I2C Driver");
MODULE_AUTHOR("bplan GmbH, Michael Hunold <michael@mihu.de>");
MODULE_LICENSE("GPL");
