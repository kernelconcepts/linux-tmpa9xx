/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2005 David Brownell
 * (C) Copyright 2002 Hewlett-Packard Company
 * (C) Copyright 2008 Magnus Damm
 * (C) Copyright 2009 bPlan GmbH
 * 
 * Toshiba TMPA9xx Bus Glue - based on ohci-tmpa9xx.c
 *
 * This file is licenced under the GPL.
 */

#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/sysrq.h>

#define MEM_DEBUG

#ifdef MEM_DEBUG
#define mprintk printk
#else
#define mprintk(...)
#endif

static void *g_virt;
static void *g_phys;

unsigned long tmpa9xx_sram_to_phys(void *virt_sram)
{
	return ((unsigned int)virt_sram - (unsigned int)g_virt  ) + (unsigned int)g_phys;
}

struct memory_item
{
	unsigned int size;
	void *virt;
	unsigned char in_use;
};

#define MAX_NUM_ITEMS 256 /* 64 */

static struct memory_item memory_map[MAX_NUM_ITEMS];
static spinlock_t sram_lock;

static void tmpa9xx_sram_print_stats(void)
{
	unsigned long flags;
	int i;
	void *adr;

	spin_lock_irqsave (&sram_lock, flags);

	adr = g_virt;
	for (i = 0; i < MAX_NUM_ITEMS; i++) {
		struct memory_item *m = &memory_map[i];
		if (!m->size)
			break;
		printk("%3d: %4d|%d @ %p\n", i, m->size, m->in_use, m->virt);
		adr += m->size;
	}	
	printk("%d bytes free in sram\n", (g_virt + 8192) - adr);

	spin_unlock_irqrestore (&sram_lock, flags);
}

static int tmpa9xx_sram_init(void *virt, void *phys)
{
	g_virt = virt;
	g_phys = phys;

	spin_lock_init(&sram_lock);
		
	return 0;
}

void *tmpa9xx_sram_alloc(int o_size)
{
	unsigned long flags;
	struct memory_item *m;
	void *virt = NULL;
	int i;
	int n_size;

	n_size = (((o_size - 1) / 32) + 1) * 32;

	spin_lock_irqsave (&sram_lock, flags);	

	for (i = 0; i < MAX_NUM_ITEMS; i++) {
		m = &memory_map[i];
	
		if (!m->size)
		    break;
		if (m->in_use)
		    continue;
		if (m->size != n_size)
		    continue;

		m->in_use = 1;
		virt = m->virt;
		break;
	}
	if (i == MAX_NUM_ITEMS)
	{
		printk("increase MAX_NUM_ITEMS\n");
		virt = NULL;
		goto out;
	}
	
	/* try allocate new item */
	if (!m->size) {
		void *adr = g_virt;
		if (i) {
			adr = memory_map[i-1].virt + memory_map[i-1].size;
		}
		if (adr + n_size > g_virt + 8192) {
			/* out of memory */
			printk("out of memory for size %d\n", n_size);
			virt = NULL;
			goto out;
		}

		mprintk("allocating new item @ %2d with size %d @ adr %p (original size %d)\n", i, n_size, adr, o_size);
		m = &memory_map[i];
		m->virt = adr;
		m->size = n_size;
		m->in_use = 1;
		virt = m->virt;

	}

	mprintk("using item %2d, size %4d @ virt %p\n", i, memory_map[i].size, virt);

out:
	spin_unlock_irqrestore (&sram_lock, flags);
	return virt;
}

void tmpa9xx_sram_free(void *virt)
{
	unsigned long flags;
	int i, j;
    int last_in_use;

	spin_lock_irqsave (&sram_lock, flags);

	for (i = 0; i < MAX_NUM_ITEMS; i++) {
		struct memory_item *m = &memory_map[i];

		if (m->virt != virt)
			continue;

		mprintk("clr'g item %2d, size %4d @ virt %p\n", i, m->size, virt);
		
		m->in_use = 0;
		break;
	}
	BUG_ON(i == MAX_NUM_ITEMS);

    /* erase all unused items after the last item in use */
    last_in_use = -1;
    for (i = 0; i < MAX_NUM_ITEMS; i++) {
        struct memory_item *m = &memory_map[i];
        if(m->in_use)
            last_in_use = i;
        if (!m->size)
            break;
    }
    last_in_use++;
    if(last_in_use != i) {
        mprintk("flushing items %d to %d\n", last_in_use, i);
        for(j = last_in_use; j < i; j++) {
            struct memory_item *m = &memory_map[j];
            m->size = 0;
        }
    }

	spin_unlock_irqrestore (&sram_lock, flags);
}

#ifdef CONFIG_MAGIC_SYSRQ
static void sysrq_handle_tmpa(int key, struct tty_struct *tty)
{
	tmpa9xx_sram_print_stats();
}

static struct sysrq_key_op sysrq_tmpa_op = {
	.handler	= sysrq_handle_tmpa,
	.help_msg	= "tmpa(A)",
	.action_msg	= "DEBUG",
};
#endif

static int ohci_tmpa9xx_init(struct usb_hcd *hcd)
{
	struct device *dev = hcd->self.controller;
	int ret;

	ret = ohci_init(hcd_to_ohci(hcd));
	if (ret) {
		dev_err(dev, "can't init %s", hcd->self.bus_name);
	}

	return ret;
}

static int ohci_tmpa9xx_start(struct usb_hcd *hcd)
{
	struct device *dev = hcd->self.controller;
	int ret;

	ret = ohci_run(hcd_to_ohci(hcd));
	if (ret < 0) {
		dev_err(dev, "can't start %s", hcd->self.bus_name);
		ohci_stop(hcd);
	}

	return ret;
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_tmpa9xx_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"tmpa9xx OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY | HCD_LOCAL_MEM,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ohci_tmpa9xx_init,
	.start =		ohci_tmpa9xx_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,

#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};
/*-------------------------------------------------------------------------*/

static int ohci_hcd_tmpa9xx_drv_probe(struct platform_device *pdev)
{
	const struct hc_driver *driver = &ohci_tmpa9xx_hc_driver;
	struct device *dev = &pdev->dev;
	struct resource	*res, *mem;
	int retval, irq;
	struct usb_hcd *hcd = NULL;
	void *sram_virt;

	irq = retval = platform_get_irq(pdev, 0);
	if (retval < 0)
		goto err0;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (mem == NULL) {
		dev_err(dev, "no resource definition for memory\n");
		retval = -ENOENT;
		goto err0;
	}

	if (!request_mem_region(mem->start, mem->end - mem->start + 1,
				pdev->name)) {
		dev_err(dev, "request_mem_region failed\n");
		retval = -EBUSY;
		goto err0;
	}

	sram_virt = ioremap(mem->start, mem->end - mem->start + 1);
	if (!sram_virt) {
		dev_err(dev, "cannot remap sram\n");
		retval = -ENXIO;
		goto err1;
	}

	tmpa9xx_sram_init(sram_virt, (void *)mem->start);

#ifdef CONFIG_MAGIC_SYSRQ
	register_sysrq_key('a', &sysrq_tmpa_op);
#endif

	/* allocate, reserve and remap resources for registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "no resource definition for registers\n");
		retval = -ENOENT;
		goto err2;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, "tmpa9xx");
	if (!hcd) {
		retval = -ENOMEM;
		goto err2;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,	pdev->name)) {
		dev_err(dev, "request_mem_region failed\n");
		retval = -EBUSY;
		goto err3;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (hcd->regs == NULL) {
		dev_err(dev, "cannot remap registers\n");
		retval = -ENXIO;
		goto err4;
	}

	ohci_hcd_init(hcd_to_ohci(hcd));

	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED);
	if (retval)
		goto err5;

	return 0;

err5:
	iounmap(hcd->regs);
err4:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err3:
	usb_put_hcd(hcd);
err2:
	iounmap(sram_virt);
err1:
	release_mem_region(mem->start, mem->end - mem->start + 1);
err0:
	return retval;
}

static int ohci_hcd_tmpa9xx_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct resource	*mem;

	usb_remove_hcd(hcd);

    iounmap(hcd->regs);

    release_mem_region(hcd->rsrc_start, hcd->rsrc_len);

    usb_put_hcd(hcd);

    mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    if (mem)
        release_mem_region(mem->start, mem->end - mem->start + 1);

	platform_set_drvdata(pdev, NULL);

#ifdef CONFIG_MAGIC_SYSRQ
		unregister_sysrq_key('a', &sysrq_tmpa_op);
#endif
	return 0;
}

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_PM
static int ohci_tmpa9xx_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(platform_get_drvdata(pdev));

	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;


	ohci_to_hcd(ohci)->state = HC_STATE_SUSPENDED;
	return 0;
}

static int ohci_tmpa9xx_resume(struct platform_device *pdev)
{
	struct usb_hcd	*hcd = platform_get_drvdata(pdev);
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);

	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;


	ohci_finish_controller_resume(hcd);
	return 0;
}
#else
#define ohci_tmpa9xx_suspend NULL
#define ohci_tmpa9xx_resume NULL
#endif

/*-------------------------------------------------------------------------*/

/*
 * Driver definition to register with the tmpa9xx bus
 */
static struct platform_driver ohci_hcd_tmpa9xx_driver = {
	.probe		= ohci_hcd_tmpa9xx_drv_probe,
	.remove		= ohci_hcd_tmpa9xx_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.suspend	= ohci_tmpa9xx_suspend,
	.resume		= ohci_tmpa9xx_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tmpa9xx-usb",
	},
};

MODULE_ALIAS("platform:tmpa9xx-usb");
