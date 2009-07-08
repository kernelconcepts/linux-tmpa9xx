/*
 * drivers/i2c/busses/i2c-tmpa910.c
 *
 * Provides I2C support for Toshiba TMPA910
 *
 * Copyright © 2009 bplan GmbH
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


struct tmpa910_i2c_regs
{
    uint32_t i2c_cr1;     // 0x0000 I2C Control Register 1
    uint32_t i2c_dbr;     // 0x0004 I2C Data Buffer Register
    uint32_t i2c_ar;      // 0x0008 I2C (Slave) Address Register
    uint32_t i2c_cr2;     // 0x000C I2C Control Register 2(Write)/Status(Read)
    uint32_t i2c_prs;     // 0x0010 I2C Prescaler Clock Set Register
    uint32_t i2c_ie;      // 0x0014 I2C Interrupt Enable Register
    uint32_t i2c_ir;      // 0x0018 I2C Interrupt Register
};

#define i2c_sr i2c_cr2      // reading cr2 reads the status

struct tmpa910_i2c_algo_data
{
	struct tmpa910_i2c_regs *regs;
	int	  channel;

	int	  irq;
	spinlock_t  lock;
};

struct tmpa910_i2c_priv
{
    struct platform_device *pdev;
    struct i2c_adapter *i2c_adapter[2];
    struct tmpa910_i2c_algo_data	i2c_algo_data[2];
		unsigned long io_start[2];
		unsigned long io_lenght[2];
};

void tmpa910_i2c_dump_regs(struct tmpa910_i2c_algo_data *algo)
{
	volatile struct tmpa910_i2c_regs __iomem *regs=algo->regs;

	printk("I2C controller at %p\n",regs);
	printk(" I2C%dCR1 (0x%02x) = 0x%02x\n", algo->channel,offsetof(struct tmpa910_i2c_regs,i2c_cr1), regs->i2c_cr1);
	printk(" I2C%dDBR (0x%02x) = 0xXX\n", algo->channel,offsetof(struct tmpa910_i2c_regs,i2c_dbr));
	printk(" I2C%dAR  (0x%02x) = 0x%02x\n",algo->channel, offsetof(struct tmpa910_i2c_regs,i2c_ar), regs->i2c_ar);
	printk(" I2C%dSR  (0x%02x) = 0x%02x\n",algo->channel, offsetof(struct tmpa910_i2c_regs,i2c_sr), regs->i2c_sr);
	printk(" I2C%dCR2 (0x%02x) = 0xXX\n",algo->channel, offsetof(struct tmpa910_i2c_regs,i2c_cr2));
	printk(" I2C%dPRS (0x%02x) = 0x%02x\n",algo->channel, offsetof(struct tmpa910_i2c_regs,i2c_prs), regs->i2c_prs);
	printk(" I2C%dIE  (0x%02x) = 0x%02x\n",algo->channel, offsetof(struct tmpa910_i2c_regs,i2c_ie), regs->i2c_ie);
	printk(" I2C%dIR  (0x%02x) = 0x%02x\n",algo->channel, offsetof(struct tmpa910_i2c_regs,i2c_ir), regs->i2c_ir);
}

//#define USE_UDELAY

int tmpa910_i2c_wait_status_timeout(struct tmpa910_i2c_algo_data *algo,
								uint32_t mask,uint32_t val)
{
	volatile struct tmpa910_i2c_regs __iomem *regs=algo->regs;
#ifdef USE_UDELAY
	volatile int timeout=1000;	
#else
	volatile int timeout=1000*1000;	
#endif
	volatile int sr;
	
	while(((sr = regs->i2c_sr) & mask) != val)
	{
		#ifdef USE_UDELAY
		udelay(10);
		#endif
		if(timeout-- < 0)	{
			printk(" status 0x%x mask 0x%x val 0x%x timeout %d\n",sr,mask,val,timeout);
			//tmpa910_i2c_dump_regs(algo);
			return -1;
		}
	}
	
	return 0;
}

int tmpa910_i2c_wait_free_bus(struct tmpa910_i2c_algo_data *algo)
{
	return tmpa910_i2c_wait_status_timeout(algo,
								(1UL << 5),0);	// bus state == free ?
}

int tmpa910_i2c_wait_done(struct tmpa910_i2c_algo_data *algo)
{
	return tmpa910_i2c_wait_status_timeout(algo,
								(1UL << 4),0);	// SCL line == low ?
}


int tmpa910_i2c_start(struct tmpa910_i2c_algo_data *algo,int slave_adr)
{
	volatile struct tmpa910_i2c_regs __iomem *regs=algo->regs;

	if(tmpa910_i2c_wait_free_bus(algo) < 0)	{
		printk("bus not free (slave_adr 0x%02x)!\n",slave_adr);
		return -EBUSY;
	}
	
	regs->i2c_cr1	|=	(1UL << 4);	// enable acknowledge clock

	regs->i2c_dbr	=	slave_adr;		// send slave address

	regs->i2c_cr2	=	(1UL << 7)	// select master mode
				|	(1UL << 6)	// transmit operation
				|	(1UL << 5)	// generate start condition
				|	(1UL << 4)	// clear service request
				|	(1UL << 3)	// enable I2C operation
				;
	
	if(tmpa910_i2c_wait_done(algo) < 0)	{
		printk("timeout on slave adr 0x%x!\n",slave_adr);
		return -ETIMEDOUT;
	}
	return 0;
}

int tmpa910_i2c_stop(struct tmpa910_i2c_algo_data *algo)
{
	volatile struct tmpa910_i2c_regs __iomem *regs=algo->regs;

	regs->i2c_cr2	=	(1UL << 7)	// select master mode
				|	(1UL << 6)	// transmit operation
				|	(0UL << 5)	// generate stop condition
				|	(1UL << 4)	// clear service request
				|	(1UL << 3)	// enable I2C operation
				;

	if(tmpa910_i2c_wait_free_bus(algo) < 0)	{
		printk("bus not free !\n");
		return -EBUSY;
	}
	
	return 0;
}

int tmpa910_i2c_xmit(struct i2c_adapter *adap,
							struct i2c_msg *msg)
{
	struct tmpa910_i2c_algo_data *algo=adap->algo_data;
	volatile struct tmpa910_i2c_regs __iomem *regs=algo->regs;
	int ret;
	unsigned int sr,cr1;
	int i;
	u8 *data;
	
	data = msg->buf;
	
	if((ret=tmpa910_i2c_start(algo,(msg->addr << 1))) < 0)
	{
		printk("failed to generate start!\n");		
		return ret;
	}
	
	sr = regs->i2c_sr;
	
	if(sr & (1UL << 0))	// check last received bit (should be low for ACK)
	{
		printk("no ack!\n");
		return -EIO;
	}

	if((sr & (1UL << 6)) == 0)	// check xmit/rcv selection state (should be xmit)
	{
		printk("wrong transfer state!\n");
		return -EIO;
	}
	
	for(i=0;i < msg->len;i++)	{
		cr1 = regs->i2c_cr1;
		cr1 &=  ~(7UL << 5);	// 8bit transfer
		cr1 |= (1UL << 4);	// acknowledge
		
		if(tmpa910_i2c_wait_done(algo) < 0)	{
			printk("timeout !\n");
			return -ETIMEDOUT;
		}
		
		regs->i2c_cr1 = cr1;
		
		regs->i2c_dbr = data[i] & 0xFF;	// put 8bits into xmit FIFO
		
		if(tmpa910_i2c_wait_done(algo) < 0)	{
			printk("timeout !\n");
			return -ETIMEDOUT;
		}
		
	}
	
	if((ret=tmpa910_i2c_stop(algo)) < 0)
	{
		printk("failed to generate stop!\n");		
		return ret;
	}
	
	return ret;
}

int tmpa910_i2c_rcv(struct i2c_adapter *adap,
							struct i2c_msg *msg)
{
	struct tmpa910_i2c_algo_data *algo=adap->algo_data;
	volatile struct tmpa910_i2c_regs __iomem *regs=algo->regs;
	int ret;
	unsigned int sr,cr1;
	int i,dummy;
	u8 *data;
	
	data = msg->buf;
	
	if((ret=tmpa910_i2c_start(algo,(msg->addr << 1)| 1)) < 0)
	{
		printk(KERN_ERR "failed to generate start!\n");		
		return ret;
	}
	
	if(tmpa910_i2c_wait_done(algo) < 0)	{
		printk(KERN_ERR "i2c timeout !\n");
		return -ETIMEDOUT;
	}

	sr = regs->i2c_sr;
	
	if(sr & (1UL << 0))	// check last received bit (should be low for ACK)
	{
		printk(KERN_ERR "no ack!\n");
		return -EIO;
	}

	if(sr & (1UL << 6))	// check xmit/rcv selection state (should be rcv)
	{
		printk(KERN_ERR "wrong transfer state!\n");
		return -EIO;
	}

		// read receive data 
	dummy = regs->i2c_dbr;

	cr1 = regs->i2c_cr1;
	cr1 &=  ~((1UL << 4) | (7UL << 5));	// 8bit transfer, no ACK
	regs->i2c_cr1 = cr1;
	
		// write dummy data to set PIN to 1
	regs->i2c_dbr = 0x00;
	
	for(i=0;i < msg->len;i++)	{

		ret=tmpa910_i2c_wait_status_timeout(algo,
								(1UL << 4),(1UL << 4));	// SCL line = free ? ?
		if(ret < 0)
		{
			printk(KERN_ERR "initial SCL line free status failed!\n");
			break;
		}
		
		if(tmpa910_i2c_wait_done(algo) < 0)	{
			printk(KERN_ERR "i2c timeout !\n");
			return -ETIMEDOUT;
		}
		
		data[i] = regs->i2c_dbr;
		
		// generate NACK, clear ACK
		cr1 = regs->i2c_cr1;
		cr1 &=  ~((1UL << 4) | (7UL << 5));	// clear no of xfer bits, no ACK
		cr1 |= (1UL << 5);				// xfer bits = 1
		regs->i2c_cr1 = cr1;

		// write dummy data to issue the ack
		regs->i2c_dbr = 0;
		
		// wait until 1bit xfer is complete
		ret=tmpa910_i2c_wait_status_timeout(algo,
								(1UL << 4),(1UL << 4));	// SCL line = free ? ?
		if(ret < 0) {
			printk("wait 1bit xfer failed!\n");
			break;
		}
	}
		
	if((ret=tmpa910_i2c_stop(algo)) < 0)
	{
		printk("failed to generate stop!\n");		
		return ret;
	}

	return ret;
}

int tmpa910_i2c_setup(struct i2c_adapter *adap);
/*
 * Generic I2C master transfer entrypoint
 */
static int tmpa910_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,int num)
{
    int i;
	struct i2c_msg *msg;
	int err=0,msgcnt=0;
	struct tmpa910_i2c_algo_data *algo=adap->algo_data;

	printk(KERN_DEBUG "tmpa910_i2c_xfer(adap %p,msgs %p,num %d)\n",adap,msgs,num);


	if(tmpa910_i2c_wait_free_bus(algo) < 0)	{
		printk("bus not free ! Reset!\n");
		tmpa910_i2c_setup(adap);
	}

    dev_dbg(&adap->dev, "tmpa910_i2c_xfer: processing %d messages:\n", num);

    for(i = 0; i < num; i++) {
	msg = &msgs[i];
	    
		printk(KERN_DEBUG " msg %p msg->buf 0x%x msg->len 0x%x msg->flags 0x%x\n",msg,msg->buf,msg->len,msg->flags);

        dev_dbg(&adap->dev, " #%d : %sing %d byte%s %s 0x%02x\n",
            i,
            msg->flags & I2C_M_RD ? "Read" : "Writ",
            msg->len,
            msg->len > 1 ? "s" : "",
            msg->flags & I2C_M_RD ? "from" : "to",
            msg->addr);

	if(msg->len == 0)
		continue;
	
	if(msg->flags & I2C_M_RD)
		err = tmpa910_i2c_rcv(adap,msg);
	else
		err = tmpa910_i2c_xmit(adap,msg);

	if(err)
		break;
	
        //dev_dbg(&adap->dev, "transfer complete\n");

	
        msgcnt++;
    };    

    if(err == 0)
	    return msgcnt;
    else
	return err;
}

// serial clock rate = PCLK / (Prescaler*(2**(2+sck)+16))
//
// 400khz for PCLK = 96MHz
// 400 = 96*1000 / ( 12 * (2**(2+0) + 16))
#define PRSCK_400KHZ 12
#define CR1SCK_400KHZ 0
// 100khz for PCLK = 96MHz
// 100 = 96*1000 / ( 30 * (2**(2+2) + 16))
#define PRSCK_100KHZ 30
#define CR1SCK_100KHZ 2

int tmpa910_i2c_setup(struct i2c_adapter *adap)
{
	struct tmpa910_i2c_algo_data *algo=adap->algo_data;
	volatile struct tmpa910_i2c_regs __iomem *regs=algo->regs;

	// software reset
	regs->i2c_cr2 = (1UL << 1) | (0UL << 0);
	regs->i2c_cr2 = (0UL << 1) | (1UL << 0);

	regs->i2c_ar = 0;
	
	// setup scalers to 100khz default
	regs->i2c_prs = PRSCK_100KHZ;
	regs->i2c_cr1 = CR1SCK_100KHZ;
	
	// enable i2c operation
	regs->i2c_cr2 = (1UL << 3);

	//tmpa910_i2c_dump_regs(algo);
	
	return 0;
}

int tmpa910_i2c_shutdown(struct i2c_adapter *adap)
{
	struct tmpa910_i2c_algo_data *algo=adap->algo_data;
	volatile struct tmpa910_i2c_regs __iomem *regs=algo->regs;

	if(tmpa910_i2c_wait_free_bus(algo) < 0)	{
		printk(KERN_ERR "bus not free !\n");
		//return -EBUSY;
	}
	
	regs->i2c_prs = 0;
	regs->i2c_cr1 = 0;
	
	// disable i2c operation
	regs->i2c_cr2 = (0UL << 3);
	
	return 0;
}

static u32 tmpa910_i2c_func(struct i2c_adapter *adapter)
{
	printk(KERN_INFO "tmpa910_i2c_func adapter %p\n",adapter);

	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm tmpa910_algorithm =
{
    .master_xfer = tmpa910_i2c_xfer,
    .functionality = tmpa910_i2c_func,
};

static int __init tmpa910_i2c_probe(struct platform_device *pdev)
{
    struct resource *r;
    int ret = 0;
    int irq;
    struct i2c_adapter *adapter;
    struct tmpa910_i2c_priv *priv;
	    
	printk(KERN_INFO "probe i2c dev!\n");

    r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if(r == NULL) {
        dev_err(&pdev->dev, "No IO memory resource for I2C0\n");
    	ret = -ENODEV;
        goto fail;
    }

    priv = kzalloc(sizeof(struct tmpa910_i2c_priv), GFP_KERNEL);
	if(priv == 0)
	{
		ret = -ENOMEM;
		goto fail;
	}

    priv->pdev = pdev;
    priv->i2c_algo_data[0].irq  = NO_IRQ;
    priv->i2c_algo_data[1].irq  = NO_IRQ;
	

    r = request_mem_region(r->start, r->end - r->start + 1, pdev->name);
    if(r == NULL) {
        dev_err(&pdev->dev, "failed to request memory resource\n");
        ret = -EBUSY;
        goto fail;
    }

		priv->io_start[0]  = r->start;
		priv->io_lenght[0] = r->end - r->start + 1;

    priv->i2c_algo_data[0].regs = ioremap(r->start, r->end - r->start +1);
    if(priv->i2c_algo_data[0].regs == NULL) {
        dev_err(&pdev->dev, "ioremap() failed\n");
        ret = -ENODEV;
        goto fail;
    }

    r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    if(r == NULL) {
        dev_err(&pdev->dev, "No IO memory resource for I2C1\n");
    	ret = -ENODEV;
        goto fail;
    }

    r = request_mem_region(r->start, r->end - r->start + 1, pdev->name);
    if(r == NULL) {
        dev_err(&pdev->dev, "failed to request memory resource\n");
        ret = -EBUSY;
        goto fail;
    }

		priv->io_start[1]  = r->start;
		priv->io_lenght[1] = r->end - r->start + 1;

    priv->i2c_algo_data[1].regs = ioremap(r->start, r->end - r->start +1);
    if(priv->i2c_algo_data[1].regs == NULL) {
        dev_err(&pdev->dev, "ioremap() failed\n");
        ret = -ENODEV;
        goto fail;
    }


    irq = platform_get_irq(pdev,0);
    if(irq < 0) {
        dev_err(&pdev->dev, "no IRQ resource defined\n");
        ret = -ENXIO;
        goto fail;
    }

    priv->i2c_algo_data[0].irq = irq;


    irq = platform_get_irq(pdev,1);
    if(irq < 0) {
        dev_err(&pdev->dev, "no IRQ resource defined\n");
        ret = -ENXIO;
        goto fail;
    }

	priv->i2c_algo_data[1].irq = irq;
    
    adapter = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
    if(adapter == NULL) {
        dev_err(&pdev->dev,"can't allocate adapter 0\n");
        ret = -ENOMEM;
        goto fail;
    }
        
	printk(KERN_INFO "tmpa910_i2c_func adapter %p\n",adapter);

    sprintf(adapter->name, "TMPA910_I2C0");
    adapter->algo = &tmpa910_algorithm;
    adapter->algo_data = &priv->i2c_algo_data[0];
    priv->i2c_algo_data[0].channel = 0;    
    adapter->class = I2C_CLASS_HWMON;
    adapter->dev.parent = &pdev->dev;
    adapter->id = 0;
	adapter->nr = 0;
    priv->i2c_adapter[0] = adapter;


    platform_set_drvdata(pdev, priv);

    ret = i2c_add_numbered_adapter(priv->i2c_adapter[0]);
    if(ret) {
        dev_err(&pdev->dev, "Adapter %s registration failed\n", priv->i2c_adapter[0]->name);
        goto fail;
    }
    
    tmpa910_i2c_setup(adapter);    

    adapter = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
    if(adapter == NULL) {
        dev_err(&pdev->dev,"can't allocate adapter 1\n");
        ret = -ENOMEM;
        goto fail;
    }
        
    sprintf(adapter->name, "TMPA910_I2C1");
    adapter->algo = &tmpa910_algorithm;
    adapter->algo_data = &priv->i2c_algo_data[1];   
    priv->i2c_algo_data[1].channel = 1;
    adapter->class = I2C_CLASS_HWMON;
    adapter->dev.parent = &pdev->dev;
    adapter->id = 0;		// new style drivers don't need those
	adapter->nr = 1;
    priv->i2c_adapter[1] = adapter;

    ret = i2c_add_numbered_adapter(priv->i2c_adapter[1]);
    if(ret) {
        dev_err(&pdev->dev, "Adapter %s registration failed\n", priv->i2c_adapter[1]->name);
        goto fail;
    }

    tmpa910_i2c_setup(adapter);
    
    printk( "driver ready (ch0: irq=%d IO@%p ch1: irq=%d IO@%p)\n",
	priv->i2c_algo_data[0].irq,priv->i2c_algo_data[0].regs,
	priv->i2c_algo_data[1].irq,priv->i2c_algo_data[1].regs);

    return 0;
fail:
    return ret;
}

static int tmpa910_i2c_remove(struct platform_device *pdev)
{
	struct tmpa910_i2c_priv *priv = platform_get_drvdata(pdev);
	struct i2c_adapter *adap;

	platform_set_drvdata(pdev, NULL);


	if((adap=priv->i2c_adapter[1]) != NULL) {
		tmpa910_i2c_shutdown(adap);
		i2c_del_adapter(adap);
		iounmap(priv->i2c_algo_data[1].regs);	

		release_mem_region(priv->io_start[0], priv->io_lenght[0]);

		kfree(adap);
	}
	
	if((adap=priv->i2c_adapter[0]) != NULL) {
		tmpa910_i2c_shutdown(adap);
		i2c_del_adapter(adap);
		iounmap(priv->i2c_algo_data[0].regs);		

		release_mem_region(priv->io_start[1], priv->io_lenght[1]);

		kfree(adap);
	}
	

	kfree(priv);
	return 0;
}

#ifdef CONFIG_PM

static int tmpa910_i2c_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int tmpa910_i2c_resume(struct platform_device *pdev)
{
	return 0;
}

#else

#define tmpa910_i2c_suspend NULL
#define tmpa910_i2c_resume  NULL
#endif

static struct platform_driver tmpa910_i2c_driver = {
	.probe		= tmpa910_i2c_probe,
	.remove		= tmpa910_i2c_remove,
    .suspend    = tmpa910_i2c_suspend,
    .resume     = tmpa910_i2c_resume,
	.driver		= {
		.name 	= "tmpa910-i2c",
		.owner	= THIS_MODULE,
	},
};

static int __init tmpa910_i2c_init(void)
{
	return platform_driver_register(&tmpa910_i2c_driver);
}

module_init(tmpa910_i2c_init);

static void __exit tmpa910_i2c_exit(void)
{
	platform_driver_unregister(&tmpa910_i2c_driver);
}

module_exit(tmpa910_i2c_exit);

MODULE_DESCRIPTION("Toshiba TMPA910 I2C Driver");
MODULE_AUTHOR("bplan GmbH");
MODULE_LICENSE("GPL");

