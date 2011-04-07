/*
 * I2C driver for Toshiba TMPA9xx
 *
 * Copyright (c) 2011 Michael Hunold <michael@mihu.de>
 *
 * i2c xfer logic inspired by i2c-algo-bit.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/slab.h>

static int speed_khz_0 = 0;
module_param(speed_khz_0, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(speed_khz_0, "i2c bus 0 speed in khz");

static int speed_khz_1 = 0;
module_param(speed_khz_1, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(speed_khz_1, "i2c bus 1 speed in khz");

#define CR1	(0x00)	/* control register 1 */
#define DBR	(0x04)	/* data buffer register*/
#define AR	(0x08)	/* slave address register */
#define CR2	(0x0C)	/* control register 2 (write) / status (read) */
#define SR	(0x0C)	/* control register 2 (write) / status (read) */
#define PRS	(0x10)	/* prescaler clock set register */
#define IE	(0x14)	/* interrupt enable register*/
#define IR	(0x18)	/* interrupt clear (write) / status (read) register */

#define i2c_writel(b, o, v)	writel(v, b->regs + o)
#define i2c_readl(b, o)		readl(b->regs + o)

struct prescaler_timing
{
	int prs;
	int n;
	int freq;
};

struct tmpa9xx_i2c_priv {
	void __iomem *regs;
	struct device *dev;
	struct clk *pclk;
	struct i2c_adapter *i2c_adapter;
	int irq;
	struct completion c;
	struct prescaler_timing p;
};

#define CR1_ACK  (1<<4)

#define CR2_MST  (1<<7)
#define CR2_TRX  (1<<6)
#define CR2_BB   (1<<5)
#define CR2_PIN  (1<<4)
#define CR2_I2CM (1<<3)

#define SR_LRB (1<<0)
#define SR_AL  (1<<3)

static irqreturn_t interrupt_handler(int irq, void *ptr)
{
	struct tmpa9xx_i2c_priv *i = ptr;

	dev_dbg(i->dev, "irq\n");

	i2c_writel(i, IR, (1 << 0));
	complete(&i->c);

	return IRQ_HANDLED;
}

static int start(struct tmpa9xx_i2c_priv *i, struct i2c_msg *msg)
{
	uint8_t addr = (msg->addr << 1) | !!(msg->flags & I2C_M_RD);
	uint32_t sr;

	dev_dbg(i->dev, "emitting start condition @ addr 0x%02x\n", addr);

	i2c_writel(i, CR1, CR1_ACK | i->p.n);
	i2c_writel(i, DBR, addr);
	init_completion(&i->c);
	i2c_writel(i, CR2, CR2_MST | CR2_TRX | CR2_BB | CR2_PIN | CR2_I2CM);
	wait_for_completion(&i->c);

	sr = i2c_readl(i, SR);
	if ((sr & SR_LRB)) {
		dev_dbg(i->dev, "no ack from slave\n");
		return 1;
	}

	return 0;
}

static void stop(struct tmpa9xx_i2c_priv *i)
{
	uint32_t sr = i2c_readl(i, SR);

	dev_dbg(i->dev, "emitting stop condition, sr 0x%02x\n", sr);
	BUG_ON(!(sr & CR2_BB));

	i2c_writel(i, CR2, CR2_MST | CR2_TRX | CR2_PIN | CR2_I2CM);

	while(1) {
		uint32_t sr = i2c_readl(i, SR);
		if(!(sr & CR2_BB))
			break;
	}
}

static void restart(struct tmpa9xx_i2c_priv *i)
{
	dev_dbg(i->dev, "emitting repeated start condition\n");

	i2c_writel(i, CR2, CR2_PIN | CR2_I2CM);

	while(1) {
		uint32_t sr = i2c_readl(i, SR);
		if(!(sr & CR2_BB))
			break;
	}

	while(1) {
		uint32_t sr = i2c_readl(i, SR);
		if((sr & SR_LRB))
			break;
	}

	udelay(5);

	dev_dbg(i->dev, "done, sr 0x%02x\n", i2c_readl(i, SR));
}

static int xoutb(struct tmpa9xx_i2c_priv *i, unsigned char c)
{
	uint32_t sr;

	dev_dbg(i->dev, "'0x%02x'\n", c);

	init_completion(&i->c);
	i2c_writel(i, DBR, c);
	wait_for_completion(&i->c);

	sr = i2c_readl(i, SR);
	if ((sr & SR_LRB)) {
		dev_dbg(i->dev, "nack\n");
		return 0;
	}

	dev_dbg(i->dev, "ack\n");
	return 1;
}

static int sendbytes(struct tmpa9xx_i2c_priv *i, struct i2c_msg *msg)
{
	const unsigned char *temp = msg->buf;
	int count = msg->len;
	int retval;
	int wrcount = 0;

	dev_dbg(i->dev, "sendbytes\n");

	while (count > 0) {
		retval = xoutb(i, *temp);

		if (retval) {
			count--;
			temp++;
			wrcount++;
			continue;
		}

		/* todo: handle arbitration lost */

		dev_dbg(i->dev, "sendbytes: NAK bailout.\n");
		return -EIO;
	}

	return wrcount;
}

static int xinb(struct tmpa9xx_i2c_priv *i, int ack)
{
	unsigned char indata;

	dev_dbg(i->dev, "ack %d\n", ack);

	if (!ack)
		i2c_writel(i, CR1, i->p.n);

	init_completion(&i->c);
	i2c_writel(i, DBR, 0);
	wait_for_completion(&i->c);
	indata = i2c_readl(i, DBR);

	if (!ack) {
		i2c_writel(i, CR1, (1 << 5) | i->p.n);
		init_completion(&i->c);
		i2c_writel(i, DBR, 0);
		wait_for_completion(&i->c);
	}

	return indata;
}

static int readbytes(struct tmpa9xx_i2c_priv *i, struct i2c_msg *msg)
{
	int inval;
	int rdcount = 0;	/* counts bytes read */
	unsigned char *temp = msg->buf;
	int count = msg->len;

	while (count > 0) {

		count--;

		inval = xinb(i, count);
		if (inval >= 0) {
			*temp = inval;
			rdcount++;
		} else {   /* read timed out */
			break;
		}

		temp++;

		dev_dbg(i->dev, "readbytes: 0x%02x %s\n",
			inval, (count ? "A" : "NA"));
	}
	return rdcount;
}

static int xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct tmpa9xx_i2c_priv *priv = adap->algo_data;
	struct device *dev = priv->dev;
	struct i2c_msg *pmsg;
	uint32_t sr;
	int i;
	int ret;

	sr = i2c_readl(priv, SR);
	if ((sr & CR2_BB)) {
		dev_dbg(priv->dev, "bus is busy\n");
		return -EREMOTEIO;
	}

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];

		if (i)
			restart(priv);

		ret = start(priv, pmsg);
		if (ret) {
			dev_dbg(dev, "NAK from device addr 0x%02x msg #%d\n", msgs[i].addr, i);
			ret = -EREMOTEIO;
			goto bailout;
		}

		if (pmsg->flags & I2C_M_RD) {
			/* read bytes into buffer */
			ret = readbytes(priv, pmsg);
			if (ret >= 1)
				dev_dbg(dev, "read %d byte%s\n",
					ret, ret == 1 ? "" : "s");
			if (ret < pmsg->len) {
				if (ret >= 0)
					ret = -EREMOTEIO;
				goto bailout;
			}
		} else {
			/* write bytes from buffer */
			ret = sendbytes(priv, pmsg);
			if (ret >= 1)
				dev_dbg(dev, "wrote %d byte%s\n",
					ret, ret == 1 ? "" : "s");
			if (ret < pmsg->len) {
				if (ret >= 0)
					ret = -EREMOTEIO;
				goto bailout;
			}
		}
	}
	ret = i;

bailout:
	stop(priv);
	return ret;
}

static void calculate_prescaler_timing(int pclk_khz, int fscl_khz, struct prescaler_timing *t)
{
	int tprsck;
	struct prescaler_timing cur;

	t->freq = 0;

	for(cur.prs = 1; cur.prs <= 32; cur.prs++) {
		tprsck = (cur.prs * 1000 * 1000) / pclk_khz;
		if (tprsck < 50)
			continue;
		if (tprsck > 150)
			continue;
		for(cur.n = 0; cur.n <= 7; cur.n++) {
			cur.freq = pclk_khz / (cur.prs * ((1 << (2+cur.n)) + 16));
			if (cur.freq > t->freq && cur.freq <= fscl_khz)
				*t = cur;
		}
	}
}

static u32 func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm tmpa9xx_algorithm = {
	.master_xfer	= xfer,
	.functionality	= func,
};

static int __devinit tmpa9xx_i2c_probe(struct platform_device *pdev)
{
	struct tmpa9xx_i2c_priv *priv;
	struct i2c_adapter *adapter;
	struct resource *res;
	int speed_khz;
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
		dev_dbg(&pdev->dev, "request_irq() failed\n");
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

	/* software reset */
	i2c_writel(priv, CR2, (1 << 1) | (0 << 0));
	i2c_writel(priv, CR2, (0 << 1) | (1 << 0));

	i2c_writel(priv, AR, 0);

	priv->pclk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->pclk)) {
		dev_dbg(&pdev->dev, "clk_get() failed\n");
		ret = PTR_ERR(priv->pclk);
		goto err7;
	}

	/* check module parameter for bus speed first */
	speed_khz = pdev->id ? speed_khz_1 : speed_khz_0;
	/* otherwise try platform specifiy bus */
	if (!speed_khz)
		speed_khz = (int)pdev->dev.platform_data;
	/* if nothing is specified, use 100kHz as default */
	if (!speed_khz)
		speed_khz = 100;

	calculate_prescaler_timing(clk_get_rate(priv->pclk)/1000, speed_khz, &priv->p);
	dev_dbg(&pdev->dev, "prs %d, n %d, freq %d\n", priv->p.prs, priv->p.n, priv->p.freq);

	/* setup scalers to 100khz */
	i2c_writel(priv, PRS, priv->p.prs);
	i2c_writel(priv, CR1, CR1_ACK | priv->p.n);

	/* enable i2c operation, clear any requests */
	i2c_writel(priv, CR2, CR2_I2CM | CR2_PIN);

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "channel %d, irq %d, io @ %p, speed %d kHz\n", pdev->id, priv->irq, priv->regs, priv->p.freq);

	i2c_writel(priv, IE, (1 << 0));

	ret = i2c_add_numbered_adapter(priv->i2c_adapter);
	if (ret) {
		dev_dbg(&pdev->dev, "i2c_add_numbered_adapter() failed\n");
		ret = -ENODEV;
		goto err8;
	}

	return 0;

err8:
	i2c_writel(priv, IE, 0);
	clk_put(priv->pclk);
err7:
	/* disable i2c operation */
	i2c_writel(priv, CR2, (0 << 3));

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

	/* disable i2c operation */
	i2c_writel(priv, IE, 0);
	i2c_writel(priv, CR2, (0 << 3));

	i2c_del_adapter(adapter);
	kfree(adapter);
	iounmap(priv->regs);
	free_irq(priv->irq, priv);
	release_mem_region(res->start, res->end - res->start + 1);
	clk_put(priv->pclk);
	platform_set_drvdata(pdev, NULL);
	kfree(priv);

	return 0;
}

static struct platform_driver tmpa9xx_i2c_driver = {
	.probe = tmpa9xx_i2c_probe,
	.remove = __devexit_p(tmpa9xx_i2c_remove),
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

MODULE_DESCRIPTION("Toshiba TMPA9xx I2C driver");
MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_LICENSE("GPL");
