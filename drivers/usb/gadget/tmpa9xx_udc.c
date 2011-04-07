/*
 * tmpa9xx_udc -- driver for tmpa9xx-series USB peripheral controller
 *
 * Copyright (C) 2008 
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
 * along with this program; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA  02111-1307, USA.
 */

#undef	DEBUG
#undef	VERBOSE
#undef	PACKET_TRACE

#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/clk.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach-types.h>

#include <linux/dma-mapping.h>
#include <mach/dma.h>
#include <mach/regs.h>
#include "tmpa9xx_udc.h"

#define DRIVER_VERSION  "10 Sep 2010"

static const char driver_name[] = "tmpa9xx-udc";
static const char *const ep_name[] = {
	"ep0",
	"ep1in-bulk",
	"ep2out-bulk",
	"ep3in-int"
};

//#define __tmpa9xx_UDC_DEBUG
#ifdef __tmpa9xx_UDC_DEBUG
#define _D(x...) do {printk(KERN_INFO "Debug:"); printk(x); } while (0);
#define _E(x...) do {printk(KERN_ERR "Error:"); printk(x); } while (0);
#define _ND(x...) do {printk(x); } while (0);
#define FN_B  printk(KERN_INFO "<%s> start, line %d\n", __func__, __LINE__)
#define FN_IN  printk(KERN_INFO "<%s> start, line %d\n", __func__, __LINE__)
#else
#define _D(x...)
#define _E(x...)
#define _ND(x...)
#define FN_B
#define FN_IN
#endif

/*-------------------------------------------------------------------------*/
#include <linux/seq_file.h>

static const char debug_filename[] = "driver/udc";

#define FOURBITS "%s%s%s%s"
#define EIGHTBITS FOURBITS FOURBITS

#define tmpa9xx_ud2ab_read(dev, reg) \
	__raw_readl((dev)->udp_baseaddr + (reg))
#define tmpa9xx_ud2ab_write(dev, reg, val) \
	__raw_writel((val), (dev)->udp_baseaddr + (reg))
#define CLKCR4          __REG(0xf0050050)
#define	USB_ENABLE			0x00000001

static void
udc2_reg_read(struct tmpa9xx_udc *udc, const u32 reqdadr, u16 * data_p)
{
	u32 read_addr;
	volatile u32 reg_data;

	read_addr = (reqdadr & UDC2AB_READ_ADDRESS) | UDC2AB_READ_RQ;
	tmpa9xx_ud2ab_write(udc, UD2AB_UDC2RDREQ, read_addr);
	do {
		reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_UDC2RDREQ);
	}
	while ((reg_data & UDC2AB_READ_RQ) == UDC2AB_READ_RQ);

	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_UDC2REG_RD);

	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_UDC2RDVL);
	*data_p = (u16) (reg_data & MASK_UINT32_LOWER_16BIT);

	return;

}

static void
udc2_reg_write(struct tmpa9xx_udc *udc, const u32 reqAddr, const u16 data)
{

	tmpa9xx_ud2ab_write(udc, reqAddr, (u32) data);
	return;
}

static void usb_bulk_in(struct tmpa9xx_ep *ep, unsigned char *buf, int length)
{
	volatile u32 reg_data;
	struct tmpa9xx_udc *udc = ep->udc;

	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_INTSTS);
	if ((reg_data & INT_MR_AHBERR) == INT_MR_AHBERR) {
		tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MR_RESET);
	} else {
		memcpy((char *)udc->buf, buf, length);
		tmpa9xx_ud2ab_write(udc, UD2AB_MRSADR, udc->phy_buf);
		tmpa9xx_ud2ab_write(udc, UD2AB_MREADR,
				    (udc->phy_buf + length - 1));
		tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MR_ENABLE);

	}
	return;
}

static void usb_bulk_out(struct tmpa9xx_ep *ep)
{
	u32 reg_data;
	struct tmpa9xx_udc *udc = ep->udc;

	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_INTSTS);
	if (((reg_data & INT_MW_AHBERR) == INT_MW_AHBERR)
	    || ((reg_data & INT_MW_RD_ERR) == INT_MW_RD_ERR)) {
		tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MW_RESET);
	} else {
		udc->dma_status = DMA_READ_START;
		tmpa9xx_ud2ab_write(udc, UD2AB_MWSADR, udc->phy_buf);
		tmpa9xx_ud2ab_write(udc, UD2AB_MWEADR,
				    (int)(udc->phy_buf + ep->datasize - 1));
		tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MW_ENABLE);
	}

	return;
}

/*-------------------------------------------------------------------------*/

static void done(struct tmpa9xx_ep *ep, struct tmpa9xx_request *req, int status)
{
	unsigned stopped = ep->stopped;

	list_del_init(&req->queue);
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;
	if (status && status != -ESHUTDOWN)
		VDBG("%s done %p, status %d\n", ep->ep.name, req, status);

	ep->stopped = 1;
	req->req.complete(&ep->ep, &req->req);
	ep->stopped = stopped;
}

/*
 * write to an IN endpoint fifo, as many packets as possible.
 * irqs will use this to write the rest later.
 * caller guarantees at least one packet buffer is ready (or a zlp).
 */
static int write_ep0_fifo(struct tmpa9xx_ep *ep, struct tmpa9xx_request *req)
{
	u16 interrupt_status;
	u8 chardata;
	u8 *data_p;
	u16 PacketSize;
	u16 *buf;
	unsigned length, is_last;
	struct tmpa9xx_udc *udc = ep->udc;
	FN_B;
	buf = req->req.buf + req->req.actual;
	//prefetch(buf);

	/* how big will this packet be? */
	length = req->req.length - req->req.actual;

	if (length > ep->ep.maxpacket) {
                udelay(20);
		/* STATUS NAK Interrupt Enable. */
		udc2_reg_read(udc, UD2INT, &interrupt_status);
		interrupt_status &= STATUS_NAK_E;
                udelay(40);
		udc2_reg_write(udc, UD2INT, interrupt_status);
                udelay(40);
		length = ep->ep.maxpacket;
		req->req.actual += length;
                
		while (length > 0) {	/* write transmit data to endpoint0's Fifo */
                        udelay(20);
			udc2_reg_write(udc, UD2EP0_FIFO, *buf);
                       	udelay(40);
			buf++;
			length -= WORD_SIZE;
		}
		is_last = 0;
	} else {
		req->req.actual += length;
		while (length > 0) {	/* write transmit data to endpoint0's Fifo */
			if (length == 1) {
				udc2_reg_read(udc, UD2EP0_MaxPacketSize,
					      &PacketSize);
				udelay(30);
				udc2_reg_write(udc, UD2EP0_MaxPacketSize, 1);	/* process of EOP */
				udelay(50);	//never kill
				chardata = (u8) (*buf & MASK_UINT16_LOWER_8BIT);
				data_p =
				    (u8 *) (UD2EP0_FIFO + udc->udp_baseaddr);
				*data_p = chardata;
				length = 0;
				udelay(50);	//never kill
				udc2_reg_write(udc, UD2EP0_MaxPacketSize, PacketSize);	/* process of EOP */
				udelay(50);	//never kill
			} else {
				udc2_reg_write(udc, UD2EP0_FIFO, *buf);
				udelay(40);	//never kill
				buf++;
				length -= WORD_SIZE;
			}
		}
		udc->stage = STATUS_STAGE;
		udc2_reg_write(udc, UD2CMD, EP0_EOP);	/* process of EOP */
		/* STATUS NAK Interrupt Disable. */
		udelay(20);	//never kill
		udc2_reg_read(udc, UD2INT, &interrupt_status);
		interrupt_status |= STATUS_NAK_E;
		udelay(20);	//never kill
		udc2_reg_write(udc, UD2INT, interrupt_status);
		udelay(20);
		udelay(20);//never kill
		is_last = 1;
	}
	if (is_last)
		done(ep, req, 0);
	return is_last;
}

/*
 * special ep0 version of the above.  no UBCR0 or double buffering; status
 * handshaking is magic.  most device protocols don't need control-OUT.
 * CDC vendor commands (and RNDIS), mass storage CB/CBI, and some other
 * protocols do use them.
 */
static int read_ep0_fifo(struct tmpa9xx_ep *ep, struct tmpa9xx_request *req)
{

	u16 PacketSize;
	u16 length;
	u16 interrupt_status;
	u16 *buf;
	unsigned is_last;
	struct tmpa9xx_udc *udc = ep->udc;
	FN_B;
	buf = req->req.buf + req->req.actual;
	prefetch(buf);
	length = req->req.length - req->req.actual;

	if (length > ep->ep.maxpacket) {
		length = ep->ep.maxpacket;
		req->req.actual += length;

		/* STATUS NAK Interrupt Enable. */
		udc2_reg_read(udc, UD2INT, &interrupt_status);
		interrupt_status &= STATUS_NAK_E;
		udc2_reg_write(udc, UD2INT, interrupt_status);

		while (length > 0) {	/* write transmit data to endpoint0's Fifo */
			udc2_reg_read(udc, UD2EP0_FIFO, buf);
			buf++;
			length -= WORD_SIZE;
		}
		is_last = 0;
	} else {
		req->req.actual += length;
		while (length != 0) {	/* write transmit data to endpoint0's Fifo */
			if (length == 1) {
				udc2_reg_read(udc, UD2EP0_MaxPacketSize,
					      &PacketSize);
				udc2_reg_write(udc, UD2EP0_MaxPacketSize, 1);	/* process of EOP */
				udc2_reg_write(udc, UD2EP0_MaxPacketSize, PacketSize);	/* process of EOP */
			} else {
				udc2_reg_read(udc, UD2EP0_FIFO, buf);
				buf++;
				length -= WORD_SIZE;
			}
		}
		udc->stage = STATUS_STAGE;
		/* STATUS NAK Interrupt Disable. */
		udc2_reg_read(udc, UD2INT, &interrupt_status);
		interrupt_status |= STATUS_NAK_D;
		udc2_reg_write(udc, UD2INT, interrupt_status);
		is_last = 1;
	}

	if (is_last)
		done(ep, req, 0);
	return is_last;

}

/* pull OUT packet data from the endpoint's fifo */
static int read_fifo(struct tmpa9xx_ep *ep, struct tmpa9xx_request *req)
{
	u8 *buf;
	unsigned int bufferspace = 0, is_done = 0;
	struct tmpa9xx_udc *udc = ep->udc;
	u32 intsts;
	u16 count = 0;
	FN_IN;

	if (req == NULL) {
		_ND("req null\n");
		if (udc->dma_status == DMA_READ_IDLE) {
			ep->datasize = 0;
			udc2_reg_read(udc, UD2EP2_DataSize, &count);
			count &= UD2EP_DATASIZE_MASK;
			if (count > 0) {
				if (count > ep->ep.maxpacket)
					count = ep->ep.maxpacket;
				ep->datasize = count;
				usb_bulk_out(ep);
			} else
				count = 0;
		}
		return 0;
	}
	if (udc->dma_status == DMA_READ_START) {
		return 0;	//busy
	}
	_ND("1req->req.length=%x,%x\n", req->req.length, udc->dma_status);
	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;

	if (udc->dma_status == DMA_READ_COMPLETE) {
		udc->dma_status = DMA_READ_IDLE;
		memcpy(buf, udc->buf, ep->datasize);	//GCH

		//Check for last write error
		intsts = tmpa9xx_ud2ab_read(udc, UD2AB_INTSTS);
		if ((intsts & INT_MW_AHBERR) == INT_MW_AHBERR) {
			tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_MW_AHBERR);
			tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET,
					    UDC2AB_MW_RESET);
			return 0;
		}
		if ((intsts & INT_MW_RD_ERR) == INT_MW_RD_ERR) {
			tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_MW_RD_ERR);
			tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET,
					    UDC2AB_MW_RESET);
			return 0;
		}

		req->req.actual += ep->datasize;
		is_done = (ep->datasize < ep->ep.maxpacket);
		if (ep->datasize == bufferspace)
			is_done = 1;

		PACKET("%s %p out/%d%s\n", ep->ep.name, &req->req, count,
		       is_done ? " (done)" : "");
		//      }
	}

	/*
	 * avoid extra trips through IRQ logic for packets already in
	 * the fifo ... maybe preventing an extra (expensive) OUT-NAK
	 */
	if (is_done)
		done(ep, req, 0);
	else {
//              bufferspace -= count;
//              buf += count;
		_ND("is_done=%x\n", is_done);
		ep->datasize = 0;
		udc2_reg_read(udc, UD2EP2_DataSize, &count);
		count &= UD2EP_DATASIZE_MASK;
		if (udc->dma_status == 1)
			printk("count=%x,%x\n", count, udc->dma_status);
		if (count > 0) {
			if (count > ep->ep.maxpacket)
				count = ep->ep.maxpacket;
			if (count > bufferspace) {
				DBG("%s buffer overflow\n", ep->ep.name);
				req->req.status = -EOVERFLOW;
				count = bufferspace;
			}
			ep->datasize = count;
			usb_bulk_out(ep);
			_ND("bulk out size=%x\n", ep->datasize);
		} else
			count = 0;
		udelay(10);
//              goto rescan;
	}
	return is_done;
}

/* load fifo for an IN packet */
static int write_fifo(struct tmpa9xx_ep *ep, struct tmpa9xx_request *req)
{
	int bufferspace, count;
	unsigned is_last = 0;
	unsigned char *buf;
	FN_IN;
	/*
	 * TODO: allow for writing two packets to the fifo ... that'll
	 * reduce the amount of IN-NAKing, but probably won't affect
	 * throughput much.  (Unlike preventing OUT-NAKing!)
	 */

	/*
	 * If ep_queue() calls us, the queue is empty and possibly in
	 * odd states like TXCOMP not yet cleared (we do it, saving at
	 * least one IRQ) or the fifo not yet being free.  Those aren't
	 * issues normally (IRQ handler fast path).
	 */

//      if (status ==0 && is_last ==0){
	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;
	_ND("req->req.length=%x,req->req.actual=%x\n", req->req.length,
	    req->req.actual);
	if (ep->ep.maxpacket < bufferspace) {
		count = ep->ep.maxpacket;
		is_last = 0;
	} else {
		count = bufferspace;
//                      is_last = (count < ep->ep.maxpacket) || !req->req.zero;
	}
	if (count > 0) {

		/*
		 * Write the packet, maybe it's a ZLP.
		 *
		 * NOTE:  incrementing req->actual before we receive the ACK means
		 * gadget driver IN bytecounts can be wrong in fault cases.  That's
		 * fixable with PIO drivers like this one (save "count" here, and
		 * do the increment later on TX irq), but not for most DMA hardware.
		 *
		 * So all gadget drivers must accept that potential error.  Some
		 * hardware supports precise fifo status reporting, letting them
		 * recover when the actual bytecount matters (e.g. for USB Test
		 * and Measurement Class devices).
		 */

		usb_bulk_in(ep, buf, count);
//                      ep->datasize = count;
		req->req.actual += count;

		PACKET("%s %p in/%d%s\n", ep->ep.name, &req->req, count,
		       is_last ? " (done)" : "");
		udelay(10);
	} else {
//              if (is_last)
		is_last = 1;
		done(ep, req, 0);
	}
	return is_last;
}

static void usb_ctl_init(struct tmpa9xx_udc *udc)
{
	u32 reg_data;
	/*------------------------------*/
	/* Stamdard Class initialize                    */
	/*------------------------------*/
	udc->state = DEFAULT;
	udc->config = USB_INIT;
	udc->interface = USB_INIT;
	udc->config_bak = USB_INIT;
	udc->interface_bak = USB_INIT;
	udc->stage = IDLE_STAGE;

	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_PWCTL);

	reg_data &= PWCTL_PHY_POWER_RESET_ON;	/* [5][1] <= 0 */
	reg_data |= PWCTL_PHY_SUSPEND_ON;	/* [3] <= 1 */

	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);
	mdelay(1);

	//     reg_data = UD2AB_PWCTL;                                 /* UDPWCTL */
	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_PWCTL);

	reg_data |= PWCTL_PHY_RESET_OFF;	/* [5][3] <= 1 */

//      UD2AB_PWCTL = reg_data;                                 /* PHY ???Z?b?g??e?? */
	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);

	mdelay(1);

	//     reg_data = UD2AB_PWCTL;                                 /* UDPWCTL */
	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_PWCTL);

	reg_data &= PWCTL_PHY_SUSPEND_OFF;	/* [3] <= 0 */

	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);

	//    mdelay(1);

/* ----------------------------------------------------------------------
  wait: 1ms
----------------------------------------------------------------------*/
	mdelay(1);

	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_PWCTL);

	reg_data |= PWCTL_POWER_RESET_OFF;	/* [2] <= 1 */

	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);

	mdelay(1);

	udc2_reg_write(udc, UD2INT, UDC2_INT_MASK);	/*INT EPx MASK & Refresh; */

	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, UDC2AB_INT_ALL_CLEAR);

	tmpa9xx_ud2ab_write(udc, UD2AB_INTENB, UDC2AB_INT_MASK);

	tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET,
			    UDC2AB_MR_RESET | UDC2AB_MW_RESET);

	mdelay(1);
	//return TRUE;
}

/*
 * this is a PIO-only driver, so there's nothing
 * interesting for request or buffer allocation.
 */

static struct usb_request *tmpa9xx_ep_alloc_request(struct usb_ep *_ep,
						    unsigned int gfp_flags)
{
	struct tmpa9xx_request *req;

	req = kzalloc(sizeof(struct tmpa9xx_request), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void
tmpa9xx_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct tmpa9xx_request *req;

	req = container_of(_req, struct tmpa9xx_request, req);
//      BUG_ON(!list_empty(&req->queue));  gch
	kfree(req);
}

static int tmpa9xx_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct tmpa9xx_ep *ep = container_of(_ep, struct tmpa9xx_ep, ep);
	struct tmpa9xx_udc *udc = ep->udc;

	unsigned long flags;
	int status = 0;

	local_irq_save(flags);

	if (strncmp(_ep->name, ep_name[1], sizeof(ep_name[1])) == 0) {
		udc2_reg_write(udc, UD2CMD, EP1_STALL);	/* EP1 STALL */
	} else if (strncmp(_ep->name, ep_name[2], sizeof(ep_name[2])) == 0) {
		udc2_reg_write(udc, UD2CMD, EP2_STALL);	/* EP2 STALL */
	} else if (strncmp(_ep->name, ep_name[3], sizeof(ep_name[3])) == 0) {
		udc2_reg_write(udc, UD2CMD, EP3_STALL);	/* EP3 STALL */
	} else
		udc2_reg_write(udc, UD2CMD, EP0_STALL);	/* EP0 STALL */
	udelay(500);		//never kill

	local_irq_restore(flags);
	return status;
}

static int
tmpa9xx_ep_enable(struct usb_ep *_ep,
		  const struct usb_endpoint_descriptor *desc)
{
	struct tmpa9xx_ep *ep = container_of(_ep, struct tmpa9xx_ep, ep);
	struct tmpa9xx_udc *dev = ep->udc;
	u16 maxpacket;
	u32 tmp;
	unsigned long flags;
	FN_IN;

	maxpacket = le16_to_cpu(desc->wMaxPacketSize);
	if (!_ep || !ep || !desc || ep->desc || _ep->name == ep_name[0]
	    || desc->bDescriptorType != USB_DT_ENDPOINT
	    || maxpacket == 0 || maxpacket > ep->maxpacket) {

		DBG("bad ep or descriptor\n");
		return -EINVAL;
	}

	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DBG("bogus device state\n");
		return -ESHUTDOWN;
	}
        
	tmp = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	switch (tmp) {
	case USB_ENDPOINT_XFER_CONTROL:
		DBG("only one control endpoint\n");
		return -EINVAL;
	case USB_ENDPOINT_XFER_INT:
		if (maxpacket > 64)
			goto bogus_max;
		break;
	case USB_ENDPOINT_XFER_BULK:
		switch (maxpacket) {
		case 8:
		case 16:
		case 32:
		case 64:
		case 512:
			goto ok;
		}
bogus_max:
		DBG("bogus maxpacket %d\n", maxpacket);
		return -EINVAL;
	case USB_ENDPOINT_XFER_ISOC:
		if (!ep->is_pingpong) {
			DBG("iso requires double buffering\n");
			return -EINVAL;
		}
		break;
	}

ok:
	local_irq_save(flags);

	/* initialize endpoint to match this descriptor */
	ep->is_in = (desc->bEndpointAddress & USB_DIR_IN) != 0;
	ep->is_iso = (tmp == USB_ENDPOINT_XFER_ISOC);
	ep->stopped = 0;
	ep->desc = desc;
	ep->ep.maxpacket = maxpacket;

	local_irq_restore(flags);
	return 0;
}

static int tmpa9xx_ep_disable(struct usb_ep *_ep)
{
	return 0;
}

/* reinit == restore inital software state */
static void udc_reinit(struct tmpa9xx_udc *udc)
{
	u32 i;

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);

	for (i = 0; i < NUM_ENDPOINTS; i++) {
		struct tmpa9xx_ep *ep = &udc->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
		ep->desc = NULL;
		ep->stopped = 0;
		ep->fifo_bank = 0;
		ep->ep.maxpacket = ep->maxpacket;
//              ep->creg = (void __iomem *) udc->udp_baseaddr + tmpa9xx_UDP_CSR(i);
		// initialiser une queue par endpoint
		INIT_LIST_HEAD(&ep->queue);
		if (list_empty(&ep->queue))
			_ND("linit list ep->queue=%x\n", ep->queue);
	}
}

union setup {
	u8 raw[8];
	struct usb_ctrlrequest r;
};
static int handle_ep(struct tmpa9xx_ep *ep)
{
	struct tmpa9xx_request *req;
	if (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct tmpa9xx_request, queue);
		_ND("handle req=%p\n", req);
	} else
		req = NULL;

	if (ep->is_in) {
		if (req)
			return write_fifo(ep, req);
	} else {
		return read_fifo(ep, req);
	}
	return 0;
}

static void
handle_setup(struct tmpa9xx_udc *udc, struct tmpa9xx_ep *ep, u32 csr)
{
	u16 request_reg;
	u32 tmp;
	union setup pkt;
	u8 index;
	int status = 0;
	FN_IN;
	status = 1;
	udelay(10);
	udc2_reg_read(udc, UD2BRQ, &request_reg);

	pkt.r.bRequestType = (u8) (request_reg & MASK_UINT16_LOWER_8BIT);
	pkt.r.bRequest =
	    (u8) ((request_reg >> SHIFT_8BIT) & MASK_UINT16_LOWER_8BIT);

	udc2_reg_read(udc, UD2VAL, &pkt.r.wValue);
	udc2_reg_read(udc, UD2IDX, &pkt.r.wIndex);
	udc2_reg_read(udc, UD2LEN, &pkt.r.wLength);

	ep->stopped = 0;

#define w_index		le16_to_cpu(pkt.r.wIndex)
#define w_value		le16_to_cpu(pkt.r.wValue)
#define w_length		le16_to_cpu(pkt.r.wLength)

	VDBG("SETUP %02x.%02x v%04x i%04x l%04x\n",
	     pkt.r.bRequestType, pkt.r.bRequest, w_value, w_index, w_length);
	/*
	 * A few standard requests get handled here, ones that touch
	 * hardware ... notably for device and endpoint features.
	 */
	udc->req_pending = 1;
	if (pkt.r.bRequestType & USB_DIR_IN) {
		ep->is_in = 1;
	} else {
		ep->is_in = 0;
	}

	udc2_reg_write(udc, UD2CMD, SETUP_RECEIVED);	/* Recieved Device Request. */
	switch ((pkt.r.bRequestType << 8) | pkt.r.bRequest) {
	case ((USB_TYPE_STANDARD | USB_RECIP_DEVICE) << 8) | USB_REQ_SET_ADDRESS:
		udc->addr =
		    w_value;

		udc->req_pending = 0;
		if (udc->addr > USB_ADDRESS_MAX) {
			goto stall;
		}

		switch (udc->state & CURRENT_STATUS_CHECK) {
		case DEFAULT:
		case ADDRESSED:
			if (udc->addr == 0) {
				udc->state_bak = DEFAULT;
			} else {
				udc->state_bak = ADDRESSED;
			}
			break;
		case CONFIGURED:
			if (udc->addr == 0) {
				udc->state_bak = DEFAULT;
				goto stall;;
			} else {
				udc->state_bak = CONFIGURED;
			}
			break;
		default:
			goto stall;;
			/* FALL THROUGH */
		}
		udc->wait_for_addr_ack = 1;
		udc->stage = STATUS_STAGE;
		/* FADDR is set later, when we ack host STATUS */
		return;

	case ((USB_TYPE_STANDARD | USB_RECIP_DEVICE) << 8) | USB_REQ_SET_CONFIGURATION:
//              if (udc->wait_for_config_ack)
		VDBG("wait for config\n");
		/* CONFG is toggled later, if gadget driver succeeds */

		index = (u8) (pkt.r.wValue & MASK_UINT16_LOWER_8BIT);

		if (index > NUM_CONFIG) {
			goto stall;
		}

		if (udc->addr == 0) {
			goto stall;
		}

		if (index == 0) {	/* Config 0 */
			udc2_reg_write(udc, UD2CMD, All_EP_INVALID);	/*  INVALID */
#if 1				//
			udc->config_bak = index;
			udc->state_bak = ADDRESSED;
			udc->interface_bak = 0;

#endif
		} else {
			udc2_reg_write(udc, UD2CMD, All_EP_INVALID);	/*  INVALID */
			udelay(50);
			if (index == 1) {
				udc2_reg_write(udc, UD2EP1_MaxPacketSize,
					       udc->ep[1].maxpacket);
				udelay(50);
				udc2_reg_write(udc, UD2EP1_Status,
					       EP_DUAL_BULK_IN);
				udelay(50);
				udc2_reg_write(udc, UD2EP2_MaxPacketSize,
					       udc->ep[2].maxpacket);
				udelay(50);
				udc2_reg_write(udc, UD2EP2_Status,
					       EP_DUAL_BULK_OUT);
				udelay(50);
				udc2_reg_write(udc, UD2CMD, EP1_RESET);	/*EP1 Reset */
				udelay(50);
				udc2_reg_write(udc, UD2CMD, EP2_RESET);	/*EP2 Reset */
			} else {
				goto stall;
			}
#if 1				//

			udc->state_bak = CONFIGURED;
			udc->config_bak = index;
			udc->interface_bak = 0;
			udc->wait_for_config_ack = 1;
#endif
		}

		udc->stage = STATUS_STAGE;
		break;

		/*
		 * Hosts may set or clear remote wakeup status, and
		 * devices may report they're VBUS powered.
		 */
	case ((USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE) << 8) | USB_REQ_GET_STATUS:
#if 0
		switch (g_Current_State &
			CURRENT_STATUS_CHECK) {
		case DEFAULT:
			goto stall;
			/* FALL THROUGH */
		case ADDRESSED:
			break;
		case CONFIGURED:
			break;
		default:
			goto stall;
			/* FALL THROUGH */
		}
#endif
		request_reg = (udc->selfpowered << USB_DEVICE_SELF_POWERED);
		PACKET("get device status\n");

		udc2_reg_write(udc, UD2EP0_FIFO, request_reg);
		goto write_in;

		/* then STATUS starts later, automatically */
	case ((USB_TYPE_STANDARD | USB_RECIP_DEVICE) << 8) | USB_REQ_SET_FEATURE:

		if (w_value !=
		    USB_DEVICE_REMOTE_WAKEUP)
			goto stall;
#if 0
		switch (g_Current_State & CURRENT_STATUS_CHECK) {
		case DEFAULT:
			goto stall;
			/* FALL THROUGH */
		case ADDRESSED:
			goto stall;
			/* FALL THROUGH */
		case CONFIGURED:
			break;
		default:
			/* DO NOTHING */
			break;
		}
#endif
		udc->stage = STATUS_STAGE;
		goto succeed;
	case ((USB_TYPE_STANDARD | USB_RECIP_DEVICE) << 8) | USB_REQ_CLEAR_FEATURE:
		//              Rq_Clear_Feature();
		if (w_value !=
		    USB_DEVICE_REMOTE_WAKEUP)
			goto stall;
#if 0
		index = (UCHAR_t) (w_index & INDEX_CHECK);
		switch (g_Current_State & CURRENT_STATUS_CHECK) {
		case DEFAULT:
			goto stall;
			/* FALL THROUGH */
		case ADDRESSED:
			goto stall;
			/* FALL THROUGH */
		case CONFIGURED:
			break;
		default:
			/* DO NOTHING */
			break;
		}
#endif
		udc->stage = STATUS_STAGE;
		udc2_reg_write(udc, UD2CMD, SETUP_FIN);
		goto succeed;

		/*
		 * Interfaces have no feature settings; this is pretty useless.
		 * we won't even insist the interface exists...
		 */
	case ((USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_INTERFACE) << 8) | USB_REQ_GET_STATUS:
		switch (udc->state & CURRENT_STATUS_CHECK)
		{
		case DEFAULT:
		case CONFIGURED:
			break;
		case ADDRESSED:
			goto stall;
			/* FALL THROUGH */
		default:
			goto stall;
			/* FALL THROUGH */
		}

		index = (u8) (w_index & INDEX_CHECK);

		if (index > NUM_CONFIG1_INTERFACE) {
			goto stall;
		}
		/* DO NOTHING */

		udc2_reg_write(udc, UD2EP0_FIFO, 0);

		udc->stage = STATUS_STAGE;
		udc2_reg_write(udc, UD2CMD, EP0_EOP);	/* process of EOP */
		PACKET("get interface status\n");

		goto write_in;
		/* then STATUS starts later, automatically */
	case ((USB_TYPE_STANDARD | USB_RECIP_INTERFACE) << 8) | USB_REQ_SET_FEATURE:
	case ((USB_TYPE_STANDARD | USB_RECIP_INTERFACE) << 8) | USB_REQ_CLEAR_FEATURE:
		goto stall;

		/*
		 * Hosts may clear bulk/intr endpoint halt after the gadget
		 * driver sets it (not widely used); or set it (for testing)
		 */
	case ((USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT) << 8) | USB_REQ_GET_STATUS:
#if 0
		switch (g_Current_State &
			CURRENT_STATUS_CHECK) {
		case DEFAULT:
			goto stall;
			/* FALL THROUGH */
		case ADDRESSED:
			if (index == EP0) {
				break;
			} else {
				goto stall;
			}
			/* FALL THROUGH */
		case CONFIGURED:
			break;
		default:
			goto stall;
			/* FALL THROUGH */
		}
#endif
#if 0
		index = (UCHAR_t) (g_wIndex & INDEX_CHECK);
		if (((g_Current_Config == 0) && (index > 0)) ||
		    ((g_Current_Config == 1) && (index > NUM_TOTAL_ENDPOINTS)))
		{
			return FALSE;
		} else {
			if (ST_Feature > 0) {
				udc2_reg_write(udc, UD2EP0_FIFO, STALL_FEATURE);	//GCH
			} else {
				udc2_reg_write(udc, UD2EP0_FIFO, 0);
			}
		}
#endif
		tmp = w_index & USB_ENDPOINT_NUMBER_MASK;
		ep = &udc->ep[tmp];
		if (tmp > NUM_ENDPOINTS || (tmp && !ep->desc))
			goto stall;

		if (tmp) {
			if ((w_index & USB_DIR_IN)) {
				if (!ep->is_in)
					goto stall;
			} else if (ep->is_in)
				goto stall;
		}
		PACKET("get %s status\n", ep->ep.name);

		goto write_in;
		/* then STATUS starts later, automatically */
	case ((USB_TYPE_STANDARD | USB_RECIP_ENDPOINT) << 8) | USB_REQ_SET_FEATURE:
		tmp =
		    w_index & USB_ENDPOINT_NUMBER_MASK;
		ep = &udc->ep[tmp];
		if (w_value != USB_ENDPOINT_HALT || tmp > NUM_ENDPOINTS)
			goto stall;
		if (!ep->desc || ep->is_iso)
			goto stall;
		if ((w_index & USB_DIR_IN)) {
			if (!ep->is_in)
				goto stall;
		} else if (ep->is_in)
			goto stall;
#if 0
		switch (g_Current_State & CURRENT_STATUS_CHECK) {
		case DEFAULT:
			goto stall;
			/* FALL THROUGH */
		case ADDRESSED:
			if (index != EP0) {
				goto stall;
			}
			/* DO NOTHING */
			/* FALL THROUGH */
		case CONFIGURED:
			break;
		default:
			/* DO NOTHING */
			break;
		}

		if ((g_Current_Config == 1) && (w_index > NUM_TOTAL_ENDPOINTS)) {
			goto stall;
		}

		/* DO NOTHING */

		if ((w_value != 0) || (w_index >= NUM_BREQRUEST_MAX)) {
			goto stall;
		} else {
			switch (w_index) {
			case EP0:
				fEP0_Stall_Feature = FLAG_ON;
				break;
			case EP1:
				fEP1_Stall_Feature = FLAG_ON;
				udc2_reg_write(udc, UD2CMD, EP1_STALL);
				break;
			case EP2:
				fEP2_Stall_Feature = FLAG_ON;
				udc2_reg_write(udc, UD2CMD, EP2_STALL);
				break;
			default:
				goto stall;
				/* FALL THROUGH */
			}
		}
#endif
		if (ep->is_in)
			udc2_reg_write(udc, UD2CMD, EP1_STALL);
		else {
			udc2_reg_write(udc, UD2CMD, EP2_STALL);
		}

		goto succeed;

	case ((USB_TYPE_STANDARD | USB_RECIP_ENDPOINT) << 8) | USB_REQ_CLEAR_FEATURE:
		_ND("Clear feature\n");
		tmp = w_index & USB_ENDPOINT_NUMBER_MASK;
		ep = &udc->ep[tmp];
		if (w_value != USB_ENDPOINT_HALT || tmp > NUM_ENDPOINTS)
			goto stall;
		if (tmp == 0)
			goto succeed;
		if (!ep->desc || ep->is_iso)
			goto stall;
		if ((w_index & USB_DIR_IN)) {
			if (!ep->is_in)
				goto stall;
		} else if (ep->is_in)
			goto stall;
#if 0
		switch (g_Current_State & CURRENT_STATUS_CHECK) {
		case DEFAULT:
			goto stall;
			/* FALL THROUGH */
		case ADDRESSED:
			if (w_index != EP0) {
				goto stall;
			}
			/* DO NOTHING */
			break;
		case CONFIGURED:
			break;
		default:
			/* DO NOTHING */
			break;
		}

		if ((g_Current_Config == 1) && (w_index > NUM_TOTAL_ENDPOINTS)) {
			goto stall;
		}
#endif
		_ND("w_value=%x,%x\n", w_value, index);
		/* DO NOTHING */
		if (ep->is_in)
			udc2_reg_write(udc, UD2CMD, EP1_RESET);
		else {
			udc2_reg_write(udc, UD2CMD, EP2_RESET);
			/* FALL THROUGH */
		}

		udelay(50);
		udc->stage = STATUS_STAGE;
		udc2_reg_write(udc, UD2CMD, SETUP_FIN);
#if 0
		at91_udp_write(udc, AT91_UDP_RST_EP, ep->int_mask);
		at91_udp_write(udc, AT91_UDP_RST_EP, 0);
		tmp = __raw_readl(ep->creg);
		tmp |= CLR_FX;
		tmp &= ~(SET_FX | AT91_UDP_FORCESTALL);
		__raw_writel(tmp, ep->creg);
		if (!list_empty(&ep->queue))	//GCH
			handle_ep(ep);
#endif
		goto succeed;
	}

#undef w_value
#undef w_index
#undef w_length

	/* pass request up to the gadget driver */
	if (udc->driver)
		status = udc->driver->setup(&udc->gadget, &pkt.r);
	else
		status = -ENODEV;
	if (status < 0) {
stall:
		VDBG("req %02x.%02x protocol STALL; stat %d\n",
		     pkt.r.bRequestType, pkt.r.bRequest, status);
		udc->req_pending = 0;
	}
	return;

succeed:
	/* immediate successful (IN) STATUS after zero length DATA */
	PACKET("ep0 in/status\n");
write_in:

	udc->req_pending = 0;
	return;

}

static void handle_ep0(struct tmpa9xx_udc *udc)
{
	struct tmpa9xx_ep *ep0 = &udc->ep[0];
	struct tmpa9xx_request *req;
	//      FN_IN;//0421;
	udc2_reg_write(udc, UD2INT, INT_EP0_CLEAR);
	if (list_empty(&ep0->queue))
		req = NULL;
	else
		req =
		    list_entry(ep0->queue.next, struct tmpa9xx_request, queue);

	/* host ACKed an IN packet that we sent */
	if (ep0->is_in) {	/*Write */
		/* write more IN DATA? */
		if (req && ep0->is_in) {
//                      if (handle_ep(ep0))
			write_ep0_fifo(ep0, req);
//                              udc->req_pending = 0;

			/*
			 * Ack after:
			 *  - last IN DATA packet (including GET_STATUS)
			 *  - IN/STATUS for OUT DATA
			 *  - IN/STATUS for any zero-length DATA stage
			 * except for the IN DATA case, the host should send
			 * an OUT status later, which we'll ack.
			 */
		} else {
			udc->req_pending = 0;
//                      __raw_writel(csr, creg);

			/*
			 * SET_ADDRESS takes effect only after the STATUS
			 * (to the original address) gets acked.
			 */
		}
	}

	/* OUT packet arrived ... */
	else {

		/* OUT DATA stage */
		if (!ep0->is_in) {
			if (req) {
//                              if (handle_ep(ep0)) {
				if (read_ep0_fifo(ep0, req)) {
					/* send IN/STATUS */
					PACKET("ep0 in/status\n");
					udc->req_pending = 0;
				}
			} else if (udc->req_pending) {
				DBG("no control-OUT deferred responses!\n");
				udc->req_pending = 0;
			}

			/* STATUS stage for control-IN; ack.  */
		} else {
			PACKET("ep0 out/status ACK\n");

			/* "early" status stage */
			if (req)
				done(ep0, req, 0);
		}
	}
}

static int
tmpa9xx_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct tmpa9xx_request *req;
	struct tmpa9xx_ep *ep;
	struct tmpa9xx_udc *dev;
	int status;
	unsigned long flags;
	FN_IN;			//0421;
	req = container_of(_req, struct tmpa9xx_request, req);
	ep = container_of(_ep, struct tmpa9xx_ep, ep);

	if (!_req || !_req->complete || !_req->buf || !list_empty(&req->queue)) {
		DBG("invalid request\n");
		return -EINVAL;
	}

	if (!_ep || (!ep->desc && ep->ep.name != ep_name[0])) {
		DBG("invalid ep\n");
		return -EINVAL;
	}

	dev = ep->udc;

	if (!dev || !dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DBG("invalid device\n");
		return -EINVAL;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	local_irq_save(flags);

	/* try to kickstart any empty and idle queue */
	if (list_empty(&ep->queue) && !ep->stopped) {
		int is_ep0;

		/*
		 * If this control request has a non-empty DATA stage, this
		 * will start that stage.  It works just like a non-control
		 * request (until the status stage starts, maybe early).
		 *
		 * If the data stage is empty, then this starts a successful
		 * IN/STATUS stage.  (Unsuccessful ones use set_halt.)
		 */
		is_ep0 = (ep->ep.name == ep_name[0]);
		if (is_ep0) {

			if (!dev->req_pending) {
				status = -EINVAL;
				goto done;
			}

			/*
			 * defer changing CONFG until after the gadget driver
			 * reconfigures the endpoints.
			 */

			if (req->req.length == 0) {
ep0_in_status:
				PACKET("ep0 in/status\n");
				status = 0;
				dev->req_pending = 0;
				goto done;
			} else {
				_ND("ep0 len=%d, in=%x\n", req->req.length,
				    ep->is_in);
				if (ep->is_in)
					status = write_ep0_fifo(ep, req);
				else {
					status = read_ep0_fifo(ep, req);
					if (status)
						goto ep0_in_status;
				}
			}
		} else {
			if (req->req.length > 0) {
				if (ep->is_in)
					status = write_fifo(ep, req);
				else {
					status = read_fifo(ep, req);

					/* IN/STATUS stage is otherwise triggered by irq */
					if (status && is_ep0)
						goto ep0_in_status;
				}
			} else {
				status = -EINVAL;
				goto done;
				_ND("%s reqlen=0\n", ep->ep.name);
			}
		}
	} else
		status = 0;

	if (req && !status) {
		list_add_tail(&req->queue, &ep->queue);
	}
done:
	local_irq_restore(flags);
	return (status < 0) ? status : 0;

}

static void nop_release(struct device *dev)
{
	/* nothing to free */
}

static const struct usb_gadget_ops tmpa9xx_udc_ops = {
//      .get_frame              = tmpa9xx_get_frame,
//      .wakeup                 = tmpa9xx_wakeup,
//      .set_selfpowered        = tmpa9xx_set_selfpowered,
//      .vbus_session           = tmpa9xx_vbus_session,
//      .pullup                 = tmpa9xx_pullup,

	/*
	 * VBUS-powered devices may also also want to support bigger
	 * power budgets after an appropriate SET_CONFIGURATION.
	 */
//      .vbus_power             = tmpa9xx_vbus_power,
};

static int tmpa9xx_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct tmpa9xx_ep *ep;
	struct tmpa9xx_request *req;

	req = container_of(_req, struct tmpa9xx_request, req);
	ep = container_of(_ep, struct tmpa9xx_ep, ep);
	req->req.complete(&ep->ep, &req->req);
	return 0;
}

static const struct usb_ep_ops tmpa9xx_ep_ops = {
	.enable = tmpa9xx_ep_enable,
	.disable = tmpa9xx_ep_disable,
	.alloc_request = tmpa9xx_ep_alloc_request,
	.free_request = tmpa9xx_ep_free_request,
	.queue = tmpa9xx_ep_queue,
	.dequeue = tmpa9xx_ep_dequeue,
	.set_halt = tmpa9xx_ep_set_halt,
	// there's only imprecise fifo status reporting
};

static struct tmpa9xx_udc controller = {
	.gadget = {
		   .ops = &tmpa9xx_udc_ops,
		   .ep0 = &controller.ep[0].ep,
		   .name = driver_name,
		   .dev = {
			   .release = nop_release,
			   }
		   },
	.ep[0] = {
		  .ep = {
			 .name = "ep0",
			 .ops = &tmpa9xx_ep_ops,
			 },
		  .udc = &controller,
		  .maxpacket = 64,
		  .int_mask = 1 << 0,
		  },
	.ep[1] = {
		  .ep = {
			 .name = "ep1in-bulk",
			 .ops = &tmpa9xx_ep_ops,
			 },
		  .udc = &controller,
		  .is_pingpong = 1,
		  .maxpacket = 512,
		  .int_mask = 1 << 1,
		  },
	.ep[2] = {
		  .ep = {
			 .name = "ep2out-bulk",
			 .ops = &tmpa9xx_ep_ops,
			 },
		  .udc = &controller,
		  .is_pingpong = 1,
		  .maxpacket = 512,
		  .int_mask = 1 << 2,
		  },
	.ep[3] = {
		  .ep = {
			 .name = "ep3in-int",
			 .ops = &tmpa9xx_ep_ops,
			 },
		  .udc = &controller,
		  .is_pingpong = 1,
		  .maxpacket = 64,
		  .int_mask = 1 << 3,
		  },
};

static void stop_activity(struct tmpa9xx_udc *udc)
{
	struct usb_gadget_driver *driver = udc->driver;
	int i;

	if (udc->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->suspended = 0;

	for (i = 0; i < NUM_ENDPOINTS; i++) {
		struct tmpa9xx_ep *ep = &udc->ep[i];
		ep->stopped = 1;
//              nuke(ep, -ESHUTDOWN);//GCH
	}
//      if (driver)
//              driver->disconnect(&udc->gadget); //GCH

	udc_reinit(udc);
}

static irqreturn_t tmpa9xx_udc_irq(int irq, void *_udc)
{
	struct tmpa9xx_udc *udc = _udc;
	u32 rescans = 1;

	while (rescans--) {
		u32 status;

		status = tmpa9xx_ud2ab_read(udc, UD2AB_INTSTS);
		if (!status)
			break;

		/* USB reset irq:  not maskable */
		if ((status & INT_RESET) == INT_RESET) {
			tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_RESET);
			VDBG("end bus reset\n");
			udc->addr = 0;
			stop_activity(udc);	//GCH
			/* enable ep0 */
			udc->gadget.speed = USB_SPEED_HIGH;
			udc->suspended = 0;

		} else if ((status & INT_RESET_END) == INT_RESET_END) {
			u16 state;
			tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_RESET_END);
			udc2_reg_read(udc, UD2ADR, &state);
			state &= CURRENT_SPEED_CHECK;	/* Current_Speed check */
			udc2_reg_write(udc, UD2INT, UDC2_INT_MASK);	/*INT EPx MASK & Refresh; */
			udc->suspended = 0;
			udc->wait_for_addr_ack = 0;
			udc->wait_for_config_ack = 0;
			if (state == HIGH_SPEED) {
				_ND("high speed\n");
				udc->gadget.speed = USB_SPEED_HIGH;
				udc->ep[1].maxpacket = EP_MAX_PACKET_SIZE_HS;
				udc->ep[2].maxpacket = EP_MAX_PACKET_SIZE_HS;
			} else {
				udc->gadget.speed = USB_SPEED_FULL;
				udc->ep[1].maxpacket = EP_MAX_PACKET_SIZE_FS;
				udc->ep[2].maxpacket = EP_MAX_PACKET_SIZE_FS;
			}
			tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET,
					    UDC2AB_MR_RESET | UDC2AB_MW_RESET);
			/* host initiated suspend (3+ms bus idle) */
		} else if ((status & INT_SUSPEND) == INT_SUSPEND) {
			u32 state;
			tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_SUSPEND);
			state = tmpa9xx_ud2ab_read(udc, UD2AB_PWCTL);
		} else if ((status & INT_SETUP) == INT_SETUP) {
			struct tmpa9xx_ep *ep0 = &udc->ep[0];
			udc2_reg_write(udc, UD2INT, INT_SETUP_CLEAR);
			udc->req_pending = 0;
			handle_setup(udc, ep0, 0);

		} else if ((status & INT_DATA) == INT_DATA) {
//                      USB_Int_Rx_Zero(udc);  //GCH
		} else if ((status & INT_EP0) == INT_EP0) {
			handle_ep0(udc);
		} else if ((status & INT_MW_END_ADD) == INT_MW_END_ADD) {
			struct tmpa9xx_ep *ep = &udc->ep[2];
			tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_MW_END_ADD);
			udc->dma_status = DMA_READ_COMPLETE;
			handle_ep(ep);
		} else if ((status & INT_MR_END_ADD) == INT_MR_END_ADD) {
			struct tmpa9xx_ep *ep = &udc->ep[1];
			tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_MR_END_ADD);
			handle_ep(ep);
		} else if ((status & INT_EP) == INT_EP) {
			u16 reg_data;
			struct tmpa9xx_ep *ep2 = &udc->ep[2];
			struct tmpa9xx_ep *ep1 = &udc->ep[1];

			udc2_reg_write(udc, UD2INT, INT_EP_CLEAR);
			udelay(10);	//0410
			udc2_reg_read(udc, UD2EP1_MaxPacketSize, &reg_data);

			if ((reg_data & UD2EP_DSET) == UD2EP_DSET) {	/*dset? */
				handle_ep(ep1);
			}
			/* DO NOTHING */
			udelay(10);
			udc2_reg_read(udc, UD2EP2_MaxPacketSize, &reg_data);
			if ((reg_data & UD2EP_DSET) == UD2EP_DSET) {	/*dset? */
				handle_ep(ep2);
			}
		} else if ((status & INT_SOF) == INT_SOF) {
			udc2_reg_write(udc, UD2INT, INT_SOF_CLEAR);
		} else if ((status & INT_STATUS) == INT_STATUS) {
			u16 status;
			udc2_reg_write(udc, UD2INT, INT_STATUS_CLEAR);
			udelay(100);
			if (udc->stage == STATUS_STAGE) {
				/* STATUS NAK Interrupt Enable. */
				udc2_reg_read(udc, UD2INT, &status);
				status |= INT_STATUSNAK_MASK;
				udelay(100);
				udc2_reg_write(udc, UD2INT, status);
				udelay(50);
				if (udc->wait_for_config_ack
				    || udc->wait_for_addr_ack) {
					udc->state = udc->state_bak;
					udc->state |= udc->addr;
					udc2_reg_write(udc, UD2ADR, udc->state);
					if (udc->wait_for_config_ack) {
						udc->config = udc->config_bak;
						udc->wait_for_config_ack = 0;
					}
					if (udc->wait_for_addr_ack) {
						udc->interface =
						    udc->interface_bak;
						udc->wait_for_addr_ack = 0;
					}
				}
				/* DO NOTHING */
				udc->stage = IDLE_STAGE;
			}

		} else if ((status & INT_STATUSNAK) == INT_STATUSNAK) {
			udelay(500);	//never kill
			udc2_reg_write(udc, UD2INT, INT_STATUSNAK_CLEAR);
			udelay(20);
			udc2_reg_write(udc, UD2CMD, SETUP_FIN);

			/* endpoint IRQs are cleared by handling them */
		} else {

		}
	}

	return IRQ_HANDLED;

}

int usb_gadget_probe_driver(struct usb_gadget_driver *driver, int (*bind)(struct usb_gadget *))
{
	struct tmpa9xx_udc *udc;
	int retval;

	DBG("usb_gadget_register_driver\n");

	udc = &controller;
	if (!driver || driver->speed < USB_SPEED_FULL
	    || !bind || !driver->setup) {
		DBG("bad parameter.\n");
		return -EINVAL;
	}

	if (udc->driver) {
		DBG("UDC already has a gadget driver\n");
		return -EBUSY;
	}

	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	dev_set_drvdata(&udc->gadget.dev, &driver->driver);
	udc->enabled = 1;
	udc->selfpowered = 1;

	retval = bind(&udc->gadget);
	if (retval) {
		DBG("driver->bind() returned %d\n", retval);
		udc->driver = NULL;
		udc->gadget.dev.driver = NULL;
		dev_set_drvdata(&udc->gadget.dev, NULL);
		udc->enabled = 0;
		udc->selfpowered = 0;
		return retval;
	}
	udc2_reg_write(udc, UD2CMD, USB_READY);	//GCH
	enable_irq(udc->udp_irq);

	DBG("bound to %s\n", driver->driver.name);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_probe_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct tmpa9xx_udc *udc;
	u32 reg_data;

	DBG("usb_gadget_unregister_driver\n");

	udc = &controller;
	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;
	udc->enabled = 0;
	disable_irq(udc->udp_irq);
	udc2_reg_write(udc, UD2CMD, All_EP_INVALID);
	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_PWCTL);

	reg_data &= PWCTL_POWER_RESET_ON;	/* [2] <= 0 */

	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);

	mdelay(1);
	reg_data |= PWCTL_POWER_RESET_OFF;	/* [2] <= 0 */
	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);
	mdelay(1);
	driver->unbind(&udc->gadget);
	udc->driver = NULL;
        udc->gadget.dev.driver=NULL;
	DBG("unbound from %s\n", driver->driver.name);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);
/*-------------------------------------------------------------------------*/

static void tmpa9xx_udc_dma_handler(int dma_ch, void *data)
{
	struct tmpa9xx_udc *udc;

	udc = (struct tmpa9xx_udc *)data;
	complete(&udc->dma_completion);
	return;
}

static void tmpa9xx_udc_dma_error_handler(int dma_ch, void *data)
{
	struct tmpa9xx_udc *udc;

	udc = (struct tmpa9xx_udc *)data;
	complete(&udc->dma_completion);
	printk("DMA Error happens at DMA channel %d\n", dma_ch);
	return;
}

static int __devinit tmpa9xx_udc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_udc *udc;
	int retval;
	struct resource *res;

	if (pdev->num_resources != 2) {
		DBG("invalid num_resources");
		return -ENODEV;
	}
	if ((pdev->resource[0].flags != IORESOURCE_MEM)
	    || (pdev->resource[1].flags != IORESOURCE_IRQ)) {
		DBG("invalid resource type");
		return -ENODEV;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	if (!request_mem_region(res->start,
				res->end - res->start + 1, driver_name)) {
		DBG("someone's using UDC memory\n");
		return -EBUSY;
	}

	/* init software state */
	udc = &controller;

	udc->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(udc->clk)) {
		dev_err(&pdev->dev, "clk_get() failed\n");
		release_mem_region(res->start, res->end - res->start + 1);
		return -ENOENT;
	}

	retval = clk_enable(udc->clk);
	if (retval) {
		dev_err(&pdev->dev, "clk_enable() failed\n");
		clk_put(udc->clk);
		release_mem_region(res->start, res->end - res->start + 1);
		return -ENOENT;
	}

	udc->gadget.dev.parent = dev;
	udc->pdev = pdev;
	udc->enabled = 0;
	dev_set_name(&udc->gadget.dev, "gadget");
	udc->udp_baseaddr = ioremap(res->start, res->end - res->start + 1);
	if (!udc->udp_baseaddr) {
		clk_disable(udc->clk);
		clk_put(udc->clk);
		release_mem_region(res->start, res->end - res->start + 1);
		return -ENOMEM;
	}
	udc_reinit(udc);

	retval = device_register(&udc->gadget.dev);
	if (retval < 0)
		goto fail0;

	usb_ctl_init(udc);

	/* request DMA channel */
	init_completion(&udc->dma_completion);
	udc->dma_ch =
	    tmpa9xx_dma_request(tmpa9xx_udc_dma_handler,
				tmpa9xx_udc_dma_error_handler, NULL);
	if (udc->dma_ch < 0) {
		printk("Cannot allocate dma channel.");
		retval = -EBUSY;
		goto fail1;
	}

	udc->buf =
	    (unsigned char *)dma_alloc_coherent(NULL, 2112, &udc->phy_buf,
						GFP_KERNEL);
	if (udc->buf == NULL) {
		retval = -ENOMEM;
		printk("Cannot allocate dma buf.");
		goto fail1;
	}

	/* request UDC irqs */
	udc->udp_irq = platform_get_irq(pdev, 0);
	if (request_irq(udc->udp_irq, tmpa9xx_udc_irq,
			IRQF_DISABLED, driver_name, udc)) {
		DBG("request irq %d failed\n", udc->udp_irq);
		retval = -EBUSY;
		goto fail2;
	}

	disable_irq(udc->udp_irq);
	udc2_reg_write(udc, UD2CMD, All_EP_INVALID);
	udelay(10);

	dev_set_drvdata(dev, udc);
	device_init_wakeup(dev, 1);

	printk(KERN_INFO "%s version %s\n", driver_name, DRIVER_VERSION);

	return 0;
fail2:
	tmpa9xx_dma_free(udc->dma_ch);
	dma_free_coherent(&pdev->dev, 2112, udc->buf, udc->phy_buf);

fail1:
	device_unregister(&udc->gadget.dev);
fail0:
	iounmap(udc->udp_baseaddr);
	release_mem_region(res->start, res->end - res->start + 1);
	clk_disable(udc->clk);
	clk_put(udc->clk);
	return retval;
}

static int __devexit tmpa9xx_udc_remove(struct platform_device *pdev)
{
	struct tmpa9xx_udc *udc = platform_get_drvdata(pdev);
	struct resource *res;

	DBG("remove\n");
	if (udc->driver)
		return -EBUSY;

	device_init_wakeup(&pdev->dev, 0);

	free_irq(udc->udp_irq, udc);
	device_unregister(&udc->gadget.dev);

	iounmap(udc->udp_baseaddr);

	tmpa9xx_dma_free(udc->dma_ch);
	dma_free_coherent(&pdev->dev, 2112, udc->buf, udc->phy_buf);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);

	clk_disable(udc->clk);
	clk_put(udc->clk);

	return 0;
}

#ifdef CONFIG_PM
static int tmpa9xx_udc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct tmpa9xx_udc *udc = platform_get_drvdata(pdev);
	int wake = udc->driver && device_may_wakeup(&pdev->dev);

	/* Unless we can act normally to the host (letting it wake us up
	 * whenever it has work for us) force disconnect.  Wakeup requires
	 * PLLB for USB events (signaling for reset, wakeup, or incoming
	 * tokens) and VBUS irqs (on systems which support them).
	 */
	if ((!udc->suspended && udc->addr) || !wake
#if 0				/* missing? */
	    || tmpa9xx_suspend_entering_slow_clock()
#endif
	    ) {
		wake = 0;
	} else
		enable_irq_wake(udc->udp_irq);

	udc->active_suspend = wake;
#if 0				/* API change, needs update */
	if (udc->board.vbus_pin > 0 && wake)
		enable_irq_wake(udc->board.vbus_pin);
#endif
	return 0;
}

static int tmpa9xx_udc_resume(struct platform_device *pdev)
{
	struct tmpa9xx_udc *udc = platform_get_drvdata(pdev);

#if 0				/* API change, needs update */
	if (udc->board.vbus_pin > 0 && udc->active_suspend)
		disable_irq_wake(udc->board.vbus_pin);
#endif

	/* maybe reconnect to host; if so, clocks on */
	if (udc->active_suspend)
		disable_irq_wake(udc->udp_irq);
	//else
	//      pullup(udc, 1);
	return 0;
}
#else
#define	tmpa9xx_udc_suspend	NULL
#define	tmpa9xx_udc_resume	NULL
#endif

static struct platform_driver tmpa9xx_udc_driver = {
	.probe = tmpa9xx_udc_probe,
	.remove = __devexit_p(tmpa9xx_udc_remove),
	.suspend = tmpa9xx_udc_suspend,
	.resume = tmpa9xx_udc_resume,
	.driver = {
		   .name = (char *)driver_name,
		   .owner = THIS_MODULE,
		   },
};

static int __init udc_init_module(void)
{
	return platform_driver_register(&tmpa9xx_udc_driver);
}

module_init(udc_init_module);

static void __exit udc_exit_module(void)
{
	platform_driver_unregister(&tmpa9xx_udc_driver);
}

module_exit(udc_exit_module);

MODULE_DESCRIPTION("TMPA9xx udc driver");
MODULE_AUTHOR("Thomas Haase");
MODULE_LICENSE("GPL");
