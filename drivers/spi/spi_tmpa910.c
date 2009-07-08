/*
 * drivers/spi/spi_tmpa910.c
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

#include <linux/workqueue.h>
#include <linux/completion.h>

#include <linux/spi/spi.h>

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include <linux/time.h>

struct tmpa910_spi_regs
{
	uint32_t cr0;		// 0x0000 SSP Control register 0
	uint32_t cr1;		// 0x0004 SSP Control register 1
	uint32_t dr;		// 0x0008 SSP Receive FIFO register (read) and transmit FIFO data register (write)
	uint32_t sr;		// 0x000C SSP Status register
	uint32_t cpsr;		// 0x0010 SSP Clock prescale register
	uint32_t imsc;		// 0x0014 SSP Interrupt enable/disable register
	uint32_t ris;		// 0x0018 SSP Interrupt status prior to enable gate register
	uint32_t mis;		// 0x001C SSP Interrupt status after enable gate register
	uint32_t icr;		// 0x0020 SSP Interrupt clear register
};

struct tmpa910_spi_priv
{
	struct tmpa910_spi_regs *regs;
	struct platform_device *pdev;
	spinlock_t	lock;
	int channel;
	int	irq;

	int bits_per_word;

	struct workqueue_struct *workqueue;
	struct work_struct work;
	
	struct list_head queue;
	struct completion done;
		unsigned long io_start;
		unsigned long io_lenght;
};

struct tmpa910_spi_cs {
	int speed_hz;
	int bits_per_word;
	int cpsr;	// CPSDVSR Clock prescale divisor (2 to 254)
	int scr;	// serial clock rate (0 to 255)


};

#define TMPA910_SPI_QUEUE

int tmpa910_spi_transfer_setup(struct spi_device *spi,struct spi_transfer *t)
{
	struct tmpa910_spi_cs *cs = spi->controller_state;

	if(t == 0)
	{
		// reset values

		cs->speed_hz = spi->max_speed_hz;
		cs->bits_per_word = spi->bits_per_word;
	}
	else
	{
		cs->speed_hz = (t->speed_hz ? t->speed_hz : spi->max_speed_hz);
		cs->bits_per_word = (t->bits_per_word ? t->bits_per_word : spi->bits_per_word);
	}
	
	//cs->bits_per_word += 7;
	//cs->bits_per_word >>=3;
	//cs->bits_per_word <<=3;

	return 0;
}

void tmpa910_spi_activate_cs(struct spi_device *spi)
{
	struct tmpa910_spi_priv *tsp = spi_master_get_devdata(spi->master);
	struct tmpa910_spi_cs *cs = spi->controller_state;
	struct tmpa910_spi_regs __iomem *regs = tsp->regs;

	tsp->bits_per_word = cs->bits_per_word;

	regs = regs; // dummy

//	if(tsp->activate_cs)	{
//		tsp->activate_cs(spi->chip_select, (spi->mode & SPI_CS_HIGH) ? 1 : 0);
//	};
}

void tmpa910_spi_deactivate_cs(struct spi_device *spi)
{
	struct tmpa910_spi_priv *tsp = spi_master_get_devdata(spi->master);
	struct tmpa910_spi_cs *cs = spi->controller_state;
	struct tmpa910_spi_regs __iomem *regs = tsp->regs;

	tsp->bits_per_word = cs->bits_per_word;

	regs = regs; // dummy

//	if(tsp->deactivate_cs)	{
//		tsp->deactivate_cs(spi->chip_select, (spi->mode & SPI_CS_HIGH) ? 1 : 0);
//	};
}

void tmpa910_spi_dump_regs(struct tmpa910_spi_priv *tsp)
{
	volatile struct tmpa910_spi_regs __iomem *regs = tsp->regs;

	printk("tmpa910_spi_dump_regs tsp %p regs %p\n",tsp,regs);

	printk(" SSP%dCR0  %08x\n",tsp->channel,regs->cr0);
	printk(" SSP%dCR1  %08x\n",tsp->channel,regs->cr1);
//	printk(" SSP%dDR   %08x\n",tsp->channel,regs->dr);
	printk(" SSP%dSR   %08x (RO)\n",tsp->channel,regs->sr);
	printk(" SSP%dCPSR %08x\n",tsp->channel,regs->cpsr);
	printk(" SSP%dIMSC %08x\n",tsp->channel,regs->imsc);
	printk(" SSP%dRIS  %08x (RO)\n",tsp->channel,regs->ris);
	printk(" SSP%dMIS  %08x (RO)\n",tsp->channel,regs->mis);
//	printk(" SSP%dICR  %08x (WO)\n",tsp->channel,regs->icr);
}

static int tmpa910_spi_transfer_rxtx(struct spi_device *spi,struct spi_transfer *t)
{
	struct tmpa910_spi_priv *tsp = spi_master_get_devdata(spi->master);
	volatile struct tmpa910_spi_regs __iomem *regs = tsp->regs;
    unsigned char *rx_buf = (unsigned char *)t->rx_buf;
    unsigned char *tx_buf = (unsigned char *)t->tx_buf;
	int tx_cnt=0,rx_cnt=0;
	volatile int dat=0;
	int ris,mis;
	volatile int	sr;
	int timeout;
	struct tmpa910_spi_cs *cs=(struct tmpa910_spi_cs*)spi->controller_state;

#define SSPSR_RXFIFO_FULL	(1UL << 3)	// 1 fifo is full, 0 fifo is not full
#define SSPSR_RXFIFO_NOTEMPTY	(1UL << 2)	// 1 fifo is not empty, 0 fifo is empty
#define SSPSR_RXFIFO_EMPTY	(0UL)
#define SSPSR_TXFIFO_NOTFULL	(1UL << 1)	// 1 txfifo is not full, 0 fifo is full
#define SSPSR_TXFIFO_EMPTY	(1UL << 0)	// 1 txfifo is empty, 0 fifo is not empty


	if(!t->tx_buf && !t->rx_buf && t->len)
		return -EINVAL;

	while(rx_cnt < t->len)
	{

		ris = regs->ris;
		mis = regs->mis;

		while(tx_cnt < t->len)
		{
			if((sr = regs->sr) & SSPSR_TXFIFO_NOTFULL)	// only send if Transmit FIFO is not full
			{
				if(tx_buf != NULL)
				{
					regs->dr = *tx_buf++;
					//udelay(1);
				}
				else
				{
					regs->dr = 0;	// 0x55;
				}
				tx_cnt++;
				//tx_cnt++;
			}
			else
			{
				break;
			}
		}

		timeout=100000;
		while((((sr = regs->sr) & SSPSR_RXFIFO_NOTEMPTY) == SSPSR_RXFIFO_EMPTY) && timeout--) {	// wait for data to arrive (non empty Receive FIFO)
			ris = regs->ris; mis = regs->mis;

			if((tx_cnt < t->len) && ((sr = regs->sr) & SSPSR_TXFIFO_NOTFULL))
				continue;	// send more data ....
		};

		if(timeout <= 0)
		{
			printk(KERN_ERR "timeout! rx_cnt=%d. tx_cnt=%d, regs->sr=0x%x\n", rx_cnt,tx_cnt,  regs->sr);
			break;
		}
		else
		{
			if((((sr = regs->sr) & SSPSR_RXFIFO_NOTEMPTY)) && (rx_cnt < t->len))
			{
				if(rx_buf != NULL)
				{
					*rx_buf++ = regs->dr;	
				}
				else
				{
					dat = regs->dr;
				}
				rx_cnt++;
				//rx_cnt++;
			}
		}
	}

	return 0;
}

#define FRAME_FMT_MOTOSPI	(0UL)
#define FRAME_FMT_TI		(1UL)
#define FRAME_FMT_MICROWIRE	(2UL)	

#define DEFAULT_FRAME_FMT	FRAME_FMT_MOTOSPI	// we only support Microwire frame format

//#define PORTL_BASE  			0xF080A000
//#define PORTL_GPIODATA	 (PORTL_BASE + 0x4)	// 0x03fc)

static void tmpa910_spi_work(struct work_struct *work)
{
	struct tmpa910_spi_priv *tsp =container_of(work, struct tmpa910_spi_priv, work);
	volatile struct tmpa910_spi_regs __iomem *regs = tsp->regs;
	volatile int dat=0;
	int cnt=0;
	int first=1;
	//unsigned long *GPIO=PORTL_GPIODATA;

	//tmpa910_spi_dump_regs(tsp);

	spin_lock_irq(&tsp->lock);

	while(!list_empty(&tsp->queue)) {
		struct spi_message *m;
		struct spi_device *spi;
		struct spi_transfer *t = NULL;
		int status=0;
		struct tmpa910_spi_cs *cs;

		m = container_of(tsp->queue.next, struct spi_message, queue);
		
		list_del_init(&m->queue);

		spin_unlock_irq(&tsp->lock);

		spi = m->spi;

		cs=(struct tmpa910_spi_cs*)spi->controller_state;

		if(first)
		{
			regs->icr = 3;			// kill pending ints

			regs->cpsr= cs->cpsr;	

			regs->cr0 = (cs->scr << 8)	// set SCR to 0
				|		(((spi->mode & SPI_CPHA) ? 1UL : 0UL) << 7)	// SPCLK phase 
				|		(((spi->mode & SPI_CPOL) ? 1UL : 0UL) << 6)	// SPCLK polarity
				|		(DEFAULT_FRAME_FMT << 4)	// we only support Microwire frame format
				|		((cs->bits_per_word - 1)<< 0);	// data-width (7==8bit,15==16bit)

			regs->cr1 = (0UL << 3)	// Slave mode SP0D0 output disable
				|		(0UL << 2)	// Master (0)/slave (1) mode select
				|		(0UL << 1)	// Synchronous serial port enable
				|		(((spi->mode & SPI_LOOP) ? 1UL : 0UL) << 0);	// Loop back mode enable
	
			#if 0
			regs->imsc = (1UL << 3)	// Transmit FIFO interrupt enable
				|		 (1UL << 2)	// Receive FIFO interrupt enable
				|		 (1UL << 1)	// Receive FIFO timeout interrupt enable
				|		 (1UL << 0);// Receive overrun interrupt enable
			#endif

			#if 1
			while((regs->sr & 4) != 0)	{
				dat=regs->dr;
				cnt++;
			}

			if(cnt)
			{
				printk(" read %d bytes from receive FIFO\n",cnt);
			}

			if((regs->sr & 1) == 0)
			{
				printk(" transmit fifo is not empty !!\n");	
			}
			#endif

			//printk("current sr 0x%x\n",regs->sr);

			first=0;

		//GPIO[0] = 0;

		}

		regs->cr0 = (cs->scr << 8)	// set SCR to 0
			|		(((spi->mode & SPI_CPHA) ? 1UL : 0UL) << 7)	// SPCLK phase 
			|		(((spi->mode & SPI_CPOL) ? 1UL : 0UL) << 6)	// SPCLK polarity
			|		(DEFAULT_FRAME_FMT << 4)	// we only support Microwire frame format
			|		((cs->bits_per_word - 1)<< 0);	// data-width (7==8bit,15==16bit)

		regs->cr1 =	(0UL << 3)	// Slave mode SP0D0 output disable
			|		(0UL << 2)	// Master (0)/slave (1) mode select
			|		(1UL << 1)	// Synchronous serial port enable
			|		(((spi->mode & SPI_LOOP) ? 1UL : 0UL) << 0);	// Loop back mode enable

		list_for_each_entry(t, &m->transfers, transfer_list) {

				/* check if setup is modifed */
			if(t->bits_per_word || t->speed_hz) {
				status = tmpa910_spi_transfer_setup(spi, t);

				if(status < 0)
					break;
				printk("update bits_per_word or speed_hz\n");
			}

			// activate chip select
			
			// get cs_change flag from t->cs_change

			// do the actual transfer
			status = tmpa910_spi_transfer_rxtx(spi, t);

			if(status)
				break;

			m->actual_length += t->len;


			if(t->delay_usecs)
			{
				printk("delay_usec %d\n",t->delay_usecs);
				udelay(t->delay_usecs);
			}

			// deactive chip select again if transfer has cs_change flag set
		}

		m->status = status;

#if 0
		list_for_each_entry(t, &m->transfers, transfer_list) {

			printk("transfer %p len %d tx_buf %p rx_buf %p\n",t,t->len,t->rx_buf,t->tx_buf);
			if(t->len)
			{
			    unsigned char *rx_buf = (unsigned char *)t->rx_buf;
			    unsigned char *tx_buf = (unsigned char *)t->tx_buf;

				if(tx_buf)
				{
					int i;

					printk("TXbuf[%d]: ",t->len);
					for(i=0;i < t->len;i++)
					{
						printk(" %02X",tx_buf[i]);
					}
					printk("\n");
				}
				else
				{
				}

				if(t->rx_buf)
				{
					int i;

#if 0
					if(t->len == 1)
					{
						rx_buf[0] = 0xFF;
					}
#endif
					printk("RXbuf[%d]: ",t->len);
					for(i=0;i < t->len;i++)
					{
						printk(" %02X",rx_buf[i]);
					}
					printk("\n");
				}
				else
				{
				}

			}
		}
#endif
		m->complete(m->context);

		tmpa910_spi_transfer_setup(spi, NULL);	// reset to default setup

		spin_lock_irq(&tsp->lock);
	}

	if(!first)
	{
		regs->cr1 =	(0UL << 3)	// Slave mode SP0D0 output disable
			|		(0UL << 2)	// Master (0)/slave (1) mode select
			|		(0UL << 1)	// Synchronous serial port disable
			|		(0UL << 0);	// Loop back mode enable
		//GPIO[0] = 1;
	}

	spin_unlock_irq(&tsp->lock);
}

#define MODEBITS (SPI_CPOL | SPI_CPHA | SPI_LOOP)

#define PCLK_MHZ (96UL)

/*
 * Bit rate = f_pclk/(CPSDVSR x (1+SCR)
 *
 * CPSDVSR is an even value from 2 to 254
 * SCR os a value from 0 to 255
 *
 */

static int tmpa910_spi_setup(struct spi_device *spi)
{
	struct tmpa910_spi_priv *tsp = spi_master_get_devdata(spi->master);
	struct tmpa910_spi_cs *cs = spi->controller_state;
	unsigned long flags;
	unsigned long ref_pclk_hz;
	int cpsr = 96;	// default to 1MHz
	int scr = 0;	//

	printk(KERN_INFO "tmpa910_spi_setup spi_device %p\n",spi);

		// if bits_per_word is 0, default to 8bits per word
	if((spi->bits_per_word  != 0) && ((spi->bits_per_word < 4) || (spi->bits_per_word > 16))) {
		dev_dbg(&spi->dev, "setup: unsupported number of bits per word (%x)\n",spi->bits_per_word);
		return -EINVAL;
	}

	if(spi->mode & (~MODEBITS)) {
		dev_dbg(&spi->dev, "setup: unsupported mode bits (%x)\n",spi->mode & (~MODEBITS));
		return -EINVAL;
	}

	if(cs == NULL) {
		cs = kzalloc(sizeof(struct tmpa910_spi_cs), GFP_KERNEL);
		if(cs == NULL)	{
			return -ENOMEM;
		}
		spi->controller_state = cs;
	}

	cs->bits_per_word = spi->bits_per_word;
	cs->speed_hz = spi->max_speed_hz;

	if(cs->speed_hz != 0)
	{
		ref_pclk_hz = PCLK_MHZ * 1000000;

		if((cs->speed_hz * 2) > ref_pclk_hz) {	// we can't go higher than half PCLK
			kfree(cs);
			return -EINVAL;
		}

		cpsr = ref_pclk_hz / cs->speed_hz;
		cpsr += 1;
		cpsr &= ~1;	// multiple of 2

		scr = 0;

		if((ref_pclk_hz / cpsr) > cs->speed_hz)
		{
			cpsr+=2;
		}

		while(cpsr > 254)
		{
			cpsr >>= 1;
			scr <<= 1;
		}

		cs->cpsr = cpsr;
		cs->scr = scr;

		
	}
	else
	{
		// default to 1MHZ
		cs->cpsr = 96;
		cs->scr = 0;
	}

	
	spin_lock_irqsave(&tsp->lock, flags);
	tmpa910_spi_deactivate_cs(spi);
	spin_unlock_irqrestore(&tsp->lock, flags);

	return 0;
}

static int tmpa910_spi_transfer(struct spi_device *spi,struct spi_message *m)
{
	struct tmpa910_spi_priv *tsp= spi_master_get_devdata(spi->master);
	unsigned long flags;

	m->actual_length = 0;
	m->status = -EINPROGRESS;

#ifdef TMPA910_SPI_QUEUE
	spin_lock_irqsave(&tsp->lock, flags);
	list_add_tail(&m->queue, &tsp->queue);
	queue_work(tsp->workqueue, &tsp->work);
	spin_unlock_irqrestore(&tsp->lock, flags);
#endif
	return 0;
}

static void tmpa910_spi_cleanup(struct spi_device *spi)
{
	kfree(spi->controller_state);
}

int tmpa910_spi_port_config(int id, struct tmpa910_spi_priv *tsp)
{
//	struct tmpa910_spi_regs __iomem *regs=tsp->regs;

	tsp->bits_per_word = 8;

	return 0;
}


static irqreturn_t tmpa910_spi_isr(int irq, void *dev_id)
{
	struct tmpa910_spi_priv *tsp = (struct tmpa910_spi_priv *)dev_id;
	struct tmpa910_spi_regs __iomem *regs=tsp->regs;
	int sr[8];
	int ris[8];
	int mis[8];
	int i;
	unsigned char data[8];

	i=0;

	while(i < 7)
	{
		ris[i] = regs->ris;
		mis[i] = regs->mis;

		while(((sr[i] = regs->sr) & 4) == 0);	// wait for data to arrive

		data[i] = regs->dr;

		i++;
	};


//	for(i=0;i < 8;i++)
//	{
//		printk("%s.l%d_%ld: SR %08x RIS %08x MIS %08x IMSC %08x DR %08x\n",__FUNCTION__,__LINE__,tv.tv_usec,sr[i],ris[i],mis[i],regs->imsc,data[i]);
//	}

	ris[0] = 0;

	if((ris[0] & 8) || (mis[0] & 8))
	{
		printk("disabling Transmit FIFO interrupt\n");

		regs->imsc = regs->imsc & ~8;	// disable Transmit FIFO interrupt

		return IRQ_HANDLED;
	}

	if((ris[0] & 4) || (mis[0] & 4))
	{
		printk("disabling Receive FIFO interrupt\n");

		regs->imsc = regs->imsc & ~4;	// disable related interrupt

		return IRQ_HANDLED;
	}

	if((ris[0] & 2) || (mis[0] & 2))
	{
		printk("disabling Receive overrun interrupt\n");

		regs->imsc = regs->imsc & ~2;	// disable related interrupt

		return IRQ_HANDLED;
	}

	if((ris[0] & 1) || (mis[0] & 1))
	{
		printk("disabling Receive overrun interrupt\n");

		regs->imsc = regs->imsc & ~1;	// disable related interrupt

		return IRQ_HANDLED;
	}
	
	return IRQ_NONE;	
}

static int __init tmpa910_spi_probe(struct platform_device *pdev)
{

	struct device *dev=&pdev->dev;
	struct tmpa910_spi_priv *tsp;
	struct spi_master *master;
    struct resource *r;
	int irq;
	int ret;
	
	dev_dbg(&pdev->dev,"tmpa910_spi_probe (pdev %p)",pdev);

	switch(pdev->id) {
		case 0:
		case 1:
			break;
		default:
			return -EINVAL;
	}

	master = spi_alloc_master(dev, sizeof(struct tmpa910_spi_priv));

	if(master == NULL) {
		dev_err(&pdev->dev," No memory for spi_master !\n");
		return -ENOMEM;
	}

	master = spi_master_get(master);

	dev_set_drvdata(dev, master);

	tsp = spi_master_get_devdata(master);
	tsp->channel = pdev->id;

	master->bus_num = pdev->id;
	master->num_chipselect = 1;	// there is only a single chip select
	master->setup = tmpa910_spi_setup;
	master->transfer = tmpa910_spi_transfer;
	master->cleanup = tmpa910_spi_cleanup;

	printk("spi probe master bus_num %d\n",master->bus_num);

    tsp->pdev = pdev;
    tsp->irq  = NO_IRQ;


    r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if(r == NULL) {
        dev_err(&pdev->dev, "No IO memory resource for SPI\n");
    	ret = -ENODEV;
        goto fail;
    }

	printk("spi probe resource start %x end %x\n",r->start,r->end);

    r = request_mem_region(r->start, r->end - r->start + 1, pdev->name);
    if(r == NULL) {
        dev_err(&pdev->dev, "failed to request memory resource\n");
        ret = -EBUSY;
        goto fail;
    }

	tsp->io_start  = r->start;
		tsp->io_lenght = r->end - r->start + 1;

	printk("spi probe mem region requested start %x end %x\n",r->start,r->end);


    tsp->regs = ioremap(r->start, r->end - r->start +1);

	printk("spi probe spi_regs %p\n",tsp->regs);

    if(tsp->regs == NULL) {
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

	printk("spi probe irq %d\n",irq);

	tsp->irq = irq;

	ret = request_irq(tsp->irq, tmpa910_spi_isr, 0, "tmpa910-spi", tsp);

	if(ret) {
		printk("failed to request irq!\n");
		tsp->irq = -1;
		goto fail;
	}

	spin_lock_init(&tsp->lock);
	init_completion(&tsp->done);
	INIT_WORK(&tsp->work, tmpa910_spi_work);
	INIT_LIST_HEAD(&tsp->queue);

	tsp->workqueue = create_singlethread_workqueue(dev_name(master->dev.parent));

	if(tsp->workqueue == NULL) {
		ret = -EBUSY;
		printk("spi workqueue creation failed\n");
		goto fail;
	}

	ret = spi_register_master(master);

	if(ret) {
		printk("spi_register_master returned %d\n",ret);
	}
	printk("tmpa910_spi_probe master 0x%p tsp 0x%p\n",master,tsp);

	return 0;
fail:
	dev_err(&pdev->dev, "failed to probe SPI%d\n", pdev->id);
	return ret;
}

static int tmpa910_spi_remove(struct platform_device *pdev)
{
	struct driver_data *drv_data = platform_get_drvdata(pdev);
	struct spi_master *master = (struct spi_master *) drv_data;


	drv_data = drv_data;	// DUMMY


	if (master)
	{
		struct tmpa910_spi_priv *tsp = spi_master_get_devdata( (void *) drv_data);

		spi_unregister_master(master);

		if (tsp)
		{

			if (tsp->regs)
			{
				volatile struct tmpa910_spi_regs __iomem *regs = tsp->regs;
				// make sure to disable controller and interrupt
				regs->cr1  = 0;
				regs->imsc = 0;
				iounmap(tsp->regs);
			}

			if (tsp->io_start)
			{
				release_mem_region(tsp->io_start, tsp->io_lenght);
			}

			if (tsp->irq>=0)
				free_irq(tsp->irq, tsp);
		}


		spi_master_put(master);
	}

	return 0;
}

#ifdef CONFIG_PM

static int tmpa910_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int tmpa910_spi_resume(struct platform_device *pdev)
{
	return 0;
}

#else

#define tmpa910_spi_suspend NULL
#define tmpa910_spi_resume  NULL
#endif

static struct platform_driver tmpa910_spi_driver = {
	.probe		= tmpa910_spi_probe,
	.remove		= tmpa910_spi_remove,
	.suspend	= tmpa910_spi_suspend,
	.resume		= tmpa910_spi_resume,
	.driver		= {
		.name 	= "tmpa910-spi",
		.owner	= THIS_MODULE,
	},
};

static int __init tmpa910_spi_init(void)
{
	return platform_driver_register(&tmpa910_spi_driver);
}

module_init(tmpa910_spi_init);

static void __exit tmpa910_spi_exit(void)
{
	platform_driver_unregister(&tmpa910_spi_driver);
}

module_exit(tmpa910_spi_exit);

MODULE_DESCRIPTION("Toshiba TMPA910 SPI Driver");
MODULE_AUTHOR("bplan GmbH");
MODULE_LICENSE("GPL");
