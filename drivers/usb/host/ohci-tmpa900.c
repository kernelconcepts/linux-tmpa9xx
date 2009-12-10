/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2005 David Brownell
 * (C) Copyright 2002 Hewlett-Packard Company
 * (C) Copyright 2008 Magnus Damm
 * (C) Copyright 2009 bPlan GmbH
 * 
 * Toshiba TMPA900 Bus Glue - based on ohci-tmpa900.c
 *
 * This file is licenced under the GPL.
 */

#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>

#include <asm/cacheflush.h>

#include "ohci-tmpa900_sram.c"


static int _ohci_init (struct ohci_hcd *ohci)
{
	int ret;
	struct usb_hcd *hcd = ohci_to_hcd(ohci);

	disable (ohci);
	ohci->regs = hcd->regs;

	/* REVISIT this BIOS handshake is now moved into PCI "quirks", and
	 * was never needed for most non-PCI systems ... remove the code?
	 */

	/* Disable HC interrupts */
	ohci_writel (ohci, OHCI_INTR_MIE, &ohci->regs->intrdisable);

	/* flush the writes, and save key bits like RWC */
	if (ohci_readl (ohci, &ohci->regs->control) & OHCI_CTRL_RWC)
		ohci->hc_control |= OHCI_CTRL_RWC;

	/* Read the number of ports unless overridden */
	if (ohci->num_ports == 0)
		ohci->num_ports = roothub_a(ohci) & RH_A_NDP;

	if (ohci->hcca)
		return 0;

	ohci->hcca = (struct ohci_hcca*) tmpa9x0_sram_alloc(sizeof *ohci->hcca, 256);
	if (!ohci->hcca)
		return -ENOMEM;

	ohci->hcca_dma = tmpa9x0_sram_to_phys(ohci->hcca);

	if ((ret = ohci_mem_init (ohci)) < 0)
		ohci_stop (hcd);
	else {
		create_debug_files (ohci);
	}

	return ret;
}



static void tmpa9x0_ohci_stop (struct usb_hcd *hcd)
{
	struct ohci_hcd		*ohci = hcd_to_ohci (hcd);

	ohci_dump (ohci, 1);

	flush_scheduled_work();

	ohci_usb_reset (ohci);
	ohci_writel (ohci, OHCI_INTR_MIE, &ohci->regs->intrdisable);
	free_irq(hcd->irq, hcd);
	hcd->irq = -1;

	if (quirk_zfmicro(ohci))
		del_timer(&ohci->unlink_watchdog);

	remove_debug_files (ohci);
	ohci_mem_cleanup (ohci);
	if (ohci->hcca) {
		tmpa9x0_sram_free ( (a32) ohci->hcca);
		ohci->hcca = NULL;
		ohci->hcca_dma = 0;
	}
}


static int ohci_tmpa900_init(struct usb_hcd *hcd)
{
	int ret;

	ret = _ohci_init(hcd_to_ohci(hcd));

	return ret;
}

static int ohci_tmpa900_start(struct usb_hcd *hcd)
{
	struct device *dev = hcd->self.controller;
	int ret;

	ret = ohci_run(hcd_to_ohci(hcd));
	if (ret < 0) {
		dev_err(dev, "can't start %s", hcd->self.bus_name);
		tmpa9x0_ohci_stop(hcd);
	}

	return ret;
}

/*-------------------------------------------------------------------------*/

/* an interrupt happens */

static irqreturn_t tmpa9x0_ohci_irq (struct usb_hcd *hcd)
{
	struct ohci_hcd		*ohci = hcd_to_ohci (hcd);
	struct ohci_regs __iomem *regs = ohci->regs;
	int			ints;

	/* Read interrupt status (and flush pending writes).  We ignore the
	 * optimization of checking the LSB of hcca->done_head; it doesn't
	 * work on all systems (edge triggering for OHCI can be a factor).
	 */

	ints = ohci_readl(ohci, &regs->intrstatus);

	/* Check for an all 1's result which is a typical consequence
	 * of dead, unclocked, or unplugged (CardBus...) devices
	 */
	if (ints == ~(u32)0) {
		disable (ohci);
		ohci_dbg (ohci, "device removed!\n");
		return IRQ_HANDLED;
	}

	/* We only care about interrupts that are enabled */
	ints &= ohci_readl(ohci, &regs->intrenable);

	/* interrupt for some other device? */
	if (ints == 0)
		return IRQ_NOTMINE;

	if (ints & OHCI_INTR_UE) {
		// e.g. due to PCI Master/Target Abort
		if (quirk_nec(ohci)) {
			/* Workaround for a silicon bug in some NEC chips used
			 * in Apple's PowerBooks. Adapted from Darwin code.
			 */
			ohci_err (ohci, "OHCI Unrecoverable Error, scheduling NEC chip restart\n");

			ohci_writel (ohci, OHCI_INTR_UE, &regs->intrdisable);

			schedule_work (&ohci->nec_work);
		} else {
			disable (ohci);
			ohci_err (ohci, "OHCI Unrecoverable Error, disabled\n");
		}

		ohci_dump (ohci, 1);
		ohci_usb_reset (ohci);
	}

	if (ints & OHCI_INTR_RHSC) {
		ohci->next_statechange = jiffies + STATECHANGE_DELAY;
		ohci_writel(ohci, OHCI_INTR_RD | OHCI_INTR_RHSC,
				&regs->intrstatus);

		/* NOTE: Vendors didn't always make the same implementation
		 * choices for RHSC.  Many followed the spec; RHSC triggers
		 * on an edge, like setting and maybe clearing a port status
		 * change bit.  With others it's level-triggered, active
		 * until khubd clears all the port status change bits.  We'll
		 * always disable it here and rely on polling until khubd
		 * re-enables it.
		 */
		ohci_writel(ohci, OHCI_INTR_RHSC, &regs->intrdisable);
		usb_hcd_poll_rh_status(hcd);
	}

	/* For connect and disconnect events, we expect the controller
	 * to turn on RHSC along with RD.  But for remote wakeup events
	 * this might not happen.
	 */
	else if (ints & OHCI_INTR_RD) {
		ohci_writel(ohci, OHCI_INTR_RD, &regs->intrstatus);
		hcd->poll_rh = 1;
		if (ohci->autostop) {
			spin_lock (&ohci->lock);
			ohci_rh_resume (ohci);
			spin_unlock (&ohci->lock);
		} else
			usb_hcd_resume_root_hub(hcd);
	}

	if (ints & OHCI_INTR_WDH) {
		spin_lock (&ohci->lock);
		dl_done_list (ohci);
		spin_unlock (&ohci->lock);
	}

	if (quirk_zfmicro(ohci) && (ints & OHCI_INTR_SF)) {
		spin_lock(&ohci->lock);
		if (ohci->ed_to_check) {
			struct ed *ed = ohci->ed_to_check;

			if (check_ed(ohci, ed)) {
				/* HC thinks the TD list is empty; HCD knows
				 * at least one TD is outstanding
				 */
				if (--ohci->zf_delay == 0) {
					struct td *td = list_entry(
						ed->td_list.next,
						struct td, td_list);
					ohci_warn(ohci,
						  "Reclaiming orphan TD %p\n",
						  td);
					takeback_td(ohci, td);
					ohci->ed_to_check = NULL;
				}
			} else
				ohci->ed_to_check = NULL;
		}
		spin_unlock(&ohci->lock);
	}

	/* could track INTR_SO to reduce available PCI/... bandwidth */

	/* handle any pending URB/ED unlinks, leaving INTR_SF enabled
	 * when there's still unlinking to be done (next frame).
	 */
	spin_lock (&ohci->lock);
	if (ohci->ed_rm_list)
	{
		finish_unlinks (ohci, ohci_frame_no(ohci));
	}
	if ((ints & OHCI_INTR_SF) != 0
			&& !ohci->ed_rm_list
			&& !ohci->ed_to_check
			&& HC_IS_RUNNING(hcd->state))
		ohci_writel (ohci, OHCI_INTR_SF, &regs->intrdisable);
	spin_unlock (&ohci->lock);

	if (HC_IS_RUNNING(hcd->state)) {
		ohci_writel (ohci, ints, &regs->intrstatus);
		ohci_writel (ohci, OHCI_INTR_MIE, &regs->intrenable);
		// flush those writes
		(void) ohci_readl (ohci, &ohci->regs->control);
	}

	return IRQ_HANDLED;
}

/*
 * queue up an urb for anything except the root hub
 */
static int tmpa9x0_urb_enqueue (
	struct usb_hcd	*hcd,
	struct urb	*urb,
	gfp_t		mem_flags)
{
	int transfer_buffer_length;


	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	struct ed	*ed;
	urb_priv_t	*urb_priv;
	unsigned int	pipe = urb->pipe;
	int		i, size = 0;
	unsigned long	flags;
	int		retval = 0;

#ifdef OHCI_VERBOSE_DEBUG
	urb_print(urb, "SUB", usb_pipein(pipe), -EINPROGRESS);
#endif

	/* every endpoint has a ed, locate and maybe (re)initialize it */
	if (! (ed = ed_get (ohci, urb->ep, urb->dev, pipe, urb->interval)))
		return -ENOMEM;

	/* for the private part of the URB we need the number of TDs (size) */
	switch (ed->type) {
		case PIPE_CONTROL:
			/* td_submit_urb() doesn't yet handle these */
			if (urb->transfer_buffer_length > 4096)
				return -EMSGSIZE;

			/* 1 TD for setup, 1 for ACK, plus ... */
			size = 2;
			/* FALLTHROUGH */
		// case PIPE_INTERRUPT:
		// case PIPE_BULK:
		default:
			/* one TD for every 4096 Bytes (can be upto 8K) */
			size += urb->transfer_buffer_length / 4096;
			/* ... and for any remaining bytes ... */
			if ((urb->transfer_buffer_length % 4096) != 0)
				size++;
			/* ... and maybe a zero length packet to wrap it up */
			if (size == 0)
				size++;
			else if ((urb->transfer_flags & URB_ZERO_PACKET) != 0
				&& (urb->transfer_buffer_length
					% usb_maxpacket (urb->dev, pipe,
						usb_pipeout (pipe))) == 0)
				size++;
			break;
		case PIPE_ISOCHRONOUS: /* number of packets from URB */
			size = urb->number_of_packets;
			break;
	}

	/* allocate the private part of the URB */
	urb_priv = kzalloc (sizeof (urb_priv_t) + size * sizeof (struct td *),
			mem_flags);
	if (!urb_priv)
		return -ENOMEM;

	INIT_LIST_HEAD (&urb_priv->pending);
	urb_priv->length = size;
	urb_priv->ed = ed;

	// Changes against orignal ohci func
	// we allocate SRAM for setup and data
	// replace the original urb pointer
	// and backup then in our priv urb
	if ( usb_endpoint_xfer_control(&urb->ep->desc) )
	{
		void *setup_in_sram;
		unsigned int setup_in_sram_phys;

		printk(KERN_DEBUG "Allcoate 8 bytes for setup in SRAM\n" );
		setup_in_sram = (void *) tmpa9x0_sram_alloc(8, 32 );
		if (setup_in_sram == NULL)
		{
			printk("Out of SRAM! setup\n");
			return -ENOMEM;
		}

		setup_in_sram_phys = tmpa9x0_sram_to_phys(setup_in_sram);

		memcpy(setup_in_sram, urb->setup_packet, 8);
		dmac_clean_range(setup_in_sram, (uint8_t *) setup_in_sram );

		urb_priv->urb_setup     = urb->setup_packet;
		urb_priv->urb_setup_dma = urb->setup_dma;

		urb->setup_packet = setup_in_sram;
		urb->setup_dma    = setup_in_sram_phys;

		// bitte enqueue me in one list
		printk(KERN_DEBUG "Ok setup in SRAM at 0x%p/0x%8x\n", urb->setup_packet, urb->setup_dma);
	}
	else
	{
		urb_priv->urb_setup     = NULL;
		urb_priv->urb_setup_dma = 0;
	}

	transfer_buffer_length = urb->transfer_buffer_length;

	if (transfer_buffer_length>0)
	{
		void *data_in_sram;
		unsigned int data_in_sram_phys;

		printk(KERN_DEBUG "Allcoate %d bytes for data in SRAM\n", transfer_buffer_length );
		data_in_sram = (void *) tmpa9x0_sram_alloc(transfer_buffer_length,32 );
		if (data_in_sram == NULL)
		{
			printk(KERN_ERR "Out of SRAM! data (%d)\n", transfer_buffer_length);
			return -ENOMEM;
		}

		data_in_sram_phys = tmpa9x0_sram_to_phys(data_in_sram);

		if (usb_pipeout(urb->pipe) )
		{
			memcpy(data_in_sram, urb->transfer_buffer, transfer_buffer_length);
			dmac_clean_range(data_in_sram, (uint8_t *) data_in_sram + transfer_buffer_length );
		}


		urb_priv->urb_data     = urb->transfer_buffer;
		urb_priv->urb_data_dma = urb->transfer_dma;

		urb->transfer_buffer = data_in_sram;
		urb->transfer_dma    = data_in_sram_phys;

		// bitte enqueue me in one list
		printk(KERN_DEBUG "Ok data in SRAM at 0x%p/0x%8x. old 0x%p / priv 0x%p adr 0x%p\n",
			data_in_sram, data_in_sram_phys, urb_priv->urb_data, urb_priv, &urb_priv->urb_data);
	}
	else
	{
		urb_priv->urb_data     = NULL;
		urb_priv->urb_data_dma = 0;
	}

	/* allocate the TDs (deferring hash chain updates) */
	for (i = 0; i < size; i++) {
		urb_priv->td [i] = td_alloc (ohci, mem_flags);
		if (!urb_priv->td [i]) {
			urb_priv->length = i;
			urb_free_priv (ohci, urb_priv);
			return -ENOMEM;
		}
	}

	spin_lock_irqsave (&ohci->lock, flags);

	/* don't submit to a dead HC */
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		retval = -ENODEV;
		goto fail;
	}
	if (!HC_IS_RUNNING(hcd->state)) {
		retval = -ENODEV;
		goto fail;
	}
	retval = usb_hcd_link_urb_to_ep(hcd, urb);
	if (retval)
		goto fail;

	/* schedule the ed if needed */
	if (ed->state == ED_IDLE) {
		retval = ed_schedule (ohci, ed);
		if (retval < 0) {
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			goto fail;
		}
		if (ed->type == PIPE_ISOCHRONOUS) {
			u16	frame = ohci_frame_no(ohci);

			/* delay a few frames before the first TD */
			frame += max_t (u16, 8, ed->interval);
			frame &= ~(ed->interval - 1);
			frame |= ed->branch;
			urb->start_frame = frame;

			/* yes, only URB_ISO_ASAP is supported, and
			 * urb->start_frame is never used as input.
			 */
		}
	} else if (ed->type == PIPE_ISOCHRONOUS)
		urb->start_frame = ed->last_iso + ed->interval;

	/* fill the TDs and link them to the ed; and
	 * enable that part of the schedule, if needed
	 * and update count of queued periodic urbs
	 */

	urb->hcpriv = urb_priv;
	td_submit_urb (ohci, urb);


#if 0
	if(1)
	{
		struct td *td;
		uint32_t *ptr;
		printk("ed %p / 0x%8x\n", ed, ed->dma);
		printk("  hwINFO   0x%8x\n",   ed->hwINFO);
		printk("  hwHeadP  0x%8x\n",  ed->hwHeadP);
		printk("  hwNextED 0x%8x\n", ed->hwNextED);

		for (i = 0; i < size; i++)
		{
			td = urb_priv->td[i];
			printk("  td %p / 0x%8x\n", td, td->td_dma);
			printk("    hwINFO   0x%8x\n", td->hwINFO);
			printk("    hwCBP    0x%8x\n", td->hwCBP);
			printk("    hwNextTD 0x%8x\n", td->hwNextTD);
			printk("    hwBE     0x%8x\n", td->hwBE);
		}

		ptr  = (uint32_t *) hcd->regs;

			printk("ctrl at 0x%p\n", ptr);
		for (i = 0; i < 0x60; i+=4)
		{
			printk("  [0x%2x] 0x%8x\n", i, ptr[i/4]);
		}


	}
#endif

		/* maybe kickstart control list */
		wmb ();
		ohci_writel (ohci, OHCI_CLF, &ohci->regs->cmdstatus);

fail:
	if (retval)
		urb_free_priv (ohci, urb_priv);
	spin_unlock_irqrestore (&ohci->lock, flags);
	return retval;

}


static void _free_urb_priv_sram(struct urb *urb)
{
	urb_priv_t  *urb_priv;
	urb_priv   = urb->hcpriv;

	if (urb_priv == NULL)
	{
		printk(KERN_DEBUG "Potential memory leak\n");
		return ;
	}

	if (urb_priv->urb_data)
	{
		if (usb_pipein(urb->pipe) )
		{
			
			dmac_flush_range(urb->transfer_buffer, (uint8_t *) urb->transfer_buffer + urb->actual_length );
			memcpy(urb_priv->urb_data, urb->transfer_buffer, urb->actual_length);
		}

		tmpa9x0_sram_free( (a32) urb->transfer_buffer );

		urb->transfer_buffer = urb_priv->urb_data;
		urb->transfer_dma    = urb_priv->urb_data_dma;
	}

	if (urb_priv->urb_setup)
	{
		urb->setup_packet = urb_priv->urb_setup;
		urb->setup_dma    = urb_priv->urb_setup_dma;
		tmpa9x0_sram_free( (a32) urb->setup_packet);
	}
}

/*
 * decouple the URB from the HC queues (TDs, urb_priv).
 * reporting is always done
 * asynchronously, and we might be dealing with an urb that's
 * partially transferred, or an ED with other urbs being unlinked.
 */
static int tmpa9x0_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	int ret;

	printk(KERN_DEBUG "data at 0x%p / phys 0x%8x, len %d\n", urb->transfer_buffer, urb->transfer_dma, urb->transfer_buffer_length);

	_free_urb_priv_sram(urb);

	ret = ohci_urb_dequeue(hcd, urb, status);

	return ret; 
}


void tmpa9x0_usb_hcd_giveback_urb(struct usb_hcd *hcd, struct urb *urb, int status)
{
	_free_urb_priv_sram(urb);

	usb_hcd_giveback_urb(hcd, urb, status);
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_tmpa900_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"tmpa900 OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			tmpa9x0_ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ohci_tmpa900_init,
	.start =		ohci_tmpa900_start,
	.stop =			tmpa9x0_ohci_stop,
	.shutdown =		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		tmpa9x0_urb_enqueue,
	.urb_dequeue =		tmpa9x0_urb_dequeue,
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
#warning check this, renamed?
//	.hub_irq_enable =	ohci_rhsc_enable,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};



/* Check for NEC chip and apply quirk for allegedly lost interrupts.
 */

static void ohci_quirk_nec_worker(struct work_struct *work)
{
	struct ohci_hcd *ohci = container_of(work, struct ohci_hcd, nec_work);
	int status;

	status = ohci_init(ohci);
	if (status != 0) {
		ohci_err(ohci, "Restarting NEC controller failed in %s, %d\n",
			 "ohci_init", status);
		return;
	}

	status = ohci_restart(ohci);
	if (status != 0)
		ohci_err(ohci, "Restarting NEC controller failed in %s, %d\n",
			 "ohci_restart", status);

}

static int ohci_quirk_nec(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);

	ohci->flags |= OHCI_QUIRK_NEC;
	INIT_WORK(&ohci->nec_work, ohci_quirk_nec_worker);
	ohci_dbg (ohci, "enabled NEC chipset lost interrupt quirk\n");

	return 0;
}


/*-------------------------------------------------------------------------*/

static int ohci_hcd_tmpa900_drv_probe(struct platform_device *pdev)
{
	const struct hc_driver *driver = &ohci_tmpa900_hc_driver;
	struct device *dev = &pdev->dev;
	struct resource	*res, *mem;
	int retval, irq;
	struct usb_hcd *hcd = NULL;
	void *sram_virt;
	irq = retval = platform_get_irq(pdev, 0);
	if (retval < 0)
		goto err0;

	ohci_usb_hcd_giveback_urb = tmpa9x0_usb_hcd_giveback_urb;

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

	printk(KERN_DEBUG "SRAM at  0x%x 0x%x 0x%x\n",
		mem->start,
		mem->start - mem->parent->start,
		(mem->end - mem->start) + 1);

	sram_virt = ioremap(mem->start, SRAMSIZE);
	if (!sram_virt) {
		dev_err(dev, "cannot remap sram\n");
		retval = -ENXIO;
		goto err1;
	}

	printk(KERN_DEBUG "SRAM at 0x%x / 0x%p\n",mem->start, sram_virt);

	tmpa9x0_sram_init( (a32) sram_virt, mem->start);

	/* allocate, reserve and remap resources for registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "no resource definition for registers\n");
		retval = -ENOENT;
		goto err2;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
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

	//ohci_quirk_nec(hcd);

	hcd_to_ohci(hcd)->flags |= OHCI_QUIRK_SUPERIO;
	//hcd_to_ohci(hcd)->flags |= OHCI_QUIRK_NEC;

	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (retval)
		goto err4;

	/* enable power and unmask interrupts */


	return 0;
err4:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err3:
	usb_put_hcd(hcd);
err2:
	dma_release_declared_memory(dev);
err1:
	release_mem_region(mem->start, mem->end - mem->start + 1);
err0:
	return retval;
}

static int ohci_hcd_tmpa900_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct resource	*mem;

	usb_remove_hcd(hcd);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	dma_release_declared_memory(&pdev->dev);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (mem)
		release_mem_region(mem->start, mem->end - mem->start + 1);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_PM
static int ohci_tmpa900_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(platform_get_drvdata(pdev));

	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;


	ohci_to_hcd(ohci)->state = HC_STATE_SUSPENDED;
	return 0;
}

static int ohci_tmpa900_resume(struct platform_device *pdev)
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
#define ohci_tmpa900_suspend NULL
#define ohci_tmpa900_resume NULL
#endif

/*-------------------------------------------------------------------------*/

/*
 * Driver definition to register with the tmpa900 bus
 */
static struct platform_driver ohci_hcd_tmpa900_driver = {
	.probe		= ohci_hcd_tmpa900_drv_probe,
	.remove		= ohci_hcd_tmpa900_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.suspend	= ohci_tmpa900_suspend,
	.resume		= ohci_tmpa900_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tmpa900-usb",
	},
};
MODULE_ALIAS("platform:tmpa900-usb");
