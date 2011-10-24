/*
 * TMPA9xx USB peripheral controller driver
 *
 * Copyright (c) 2008 
 *           (c) 2011 Michael Hunold <michael@mihu.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#if 0
#define DEBUG
#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <asm/io.h>

#include "tmpa9xx_udc.h"

#define PACKET(...)	dev_dbg(udc->dev, __VA_ARGS__);

static const char driver_name[] = "tmpa9xx-udc";

static const char *const ep_name[] = {
	"ep0",
	"ep1in-bulk",
	"ep2out-bulk",
	"ep3in-int"
};

#define tmpa9xx_ud2ab_read(dev, reg) \
	__raw_readl((dev)->udp_baseaddr + (reg))

#define tmpa9xx_ud2ab_write(dev, reg, val) \
	__raw_writel((val), (dev)->udp_baseaddr + (reg))

static void udc2_reg_read(struct tmpa9xx_udc *udc, const u32 addr, u16 *data)
{
	u32 reg;

	down(&udc->sem);

	BUG_ON(tmpa9xx_ud2ab_read(udc, UD2AB_UDC2RDREQ) & UDC2AB_READ_RQ);

	tmpa9xx_ud2ab_write(udc, UD2AB_UDC2RDREQ, (addr & UDC2AB_READ_ADDRESS) | UDC2AB_READ_RQ);

	do {
		reg = tmpa9xx_ud2ab_read(udc, UD2AB_UDC2RDREQ);
	} while ((reg & UDC2AB_READ_RQ));
	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_UDC2REG_RD);

	up(&udc->sem);

	*data = tmpa9xx_ud2ab_read(udc, UD2AB_UDC2RDVL);
}

static void udc2_reg_write(struct tmpa9xx_udc *udc, const u32 addr, const u16 data)
{
	u32 reg;

	down(&udc->sem);

	BUG_ON(tmpa9xx_ud2ab_read(udc, UD2AB_UDC2RDREQ) & UDC2AB_READ_RQ);

	tmpa9xx_ud2ab_write(udc, addr, data);

	do {
		reg = tmpa9xx_ud2ab_read(udc, UD2AB_UDC2RDREQ);
	} while ((reg & UDC2AB_READ_RQ));
	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_UDC2REG_RD);

	up(&udc->sem);
}

static void usb_bulk_in(struct tmpa9xx_ep *ep, unsigned char *buf, int length)
{
	struct tmpa9xx_udc *udc = ep->udc;
	u32 reg;

	reg = tmpa9xx_ud2ab_read(udc, UD2AB_INTSTS);

	BUG_ON((reg & INT_MR_AHBERR));

	udc->dma_ep = ep;

	memcpy(udc->buf, buf, length);
	tmpa9xx_ud2ab_write(udc, UD2AB_MRSADR, udc->phy_buf);
	tmpa9xx_ud2ab_write(udc, UD2AB_MREADR, (udc->phy_buf + length - 1));
	tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MR_ENABLE);

	dev_dbg(udc->dev, "%s(): '%s', length %d\n", __func__, ep->ep.name, length);
}

static void usb_bulk_out(struct tmpa9xx_ep *ep)
{
	struct tmpa9xx_udc *udc = ep->udc;
	u32 reg;

	reg = tmpa9xx_ud2ab_read(udc, UD2AB_INTSTS);

	BUG_ON((reg & INT_MW_AHBERR));
	BUG_ON((reg & INT_MW_RD_ERR));

	udc->dma_status = DMA_READ_START;
	tmpa9xx_ud2ab_write(udc, UD2AB_MWSADR, udc->phy_buf);
	tmpa9xx_ud2ab_write(udc, UD2AB_MWEADR, (int)(udc->phy_buf + ep->datasize - 1));
	tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MW_ENABLE);

	dev_dbg(udc->dev, "%s(): '%s', datasize %d\n", __func__, ep->ep.name, ep->datasize - 1);
}

static void done(struct tmpa9xx_ep *ep, struct tmpa9xx_request *req, int status)
{
	struct tmpa9xx_udc *udc = ep->udc;
	unsigned stopped = ep->stopped;

	list_del_init(&req->queue);
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	dev_dbg(udc->dev, "%s(): '%s', status %d\n", __func__, ep->ep.name, status);

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
	u16 *buf;
	int length;
	struct tmpa9xx_udc *udc = ep->udc;
	buf = req->req.buf + req->req.actual;

	/* how big will this packet be? */
	length = req->req.length - req->req.actual;

	dev_dbg(udc->dev, "%s(): '%s', length %d\n", __func__, ep->ep.name, length);

	if (length > ep->ep.maxpacket) {
		/* STATUS NAK Interrupt Enable. */
		udc2_reg_read(udc, UD2INT, &interrupt_status);
		interrupt_status &= STATUS_NAK_E;
		udc2_reg_write(udc, UD2INT, interrupt_status);
		length = ep->ep.maxpacket;
		req->req.actual += length;

		while (length > 0) {	/* write transmit data to endpoint0's Fifo */
			udc2_reg_write(udc, UD2EP0_FIFO, *buf);
			buf++;
			length -= WORD_SIZE;
		}
		return 0;
	}

	req->req.actual += length;

	while (length > 0) {	/* write transmit data to endpoint0's Fifo */
		if (length == 1) {
			chardata = (u8) (*buf & MASK_UINT16_LOWER_8BIT);
			data_p = (u8 *) (UD2EP0_FIFO + udc->udp_baseaddr);
			*data_p = chardata;
			length = 0;
		} else {
			udc2_reg_write(udc, UD2EP0_FIFO, *buf);
			buf++;
			length -= WORD_SIZE;
		}
	}

	udc->stage = STATUS_STAGE;
	udc2_reg_write(udc, UD2CMD, EP0_EOP);	/* process of EOP */
	/* STATUS NAK Interrupt Disable. */
	udc2_reg_read(udc, UD2INT, &interrupt_status);
	interrupt_status |= STATUS_NAK_E;
	udc2_reg_write(udc, UD2INT, interrupt_status);

	done(ep, req, 0);

	return 1;
}

/*
 * special ep0 version of the above.  no UBCR0 or double buffering; status
 * handshaking is magic.  most device protocols don't need control-OUT.
 * CDC vendor commands (and RNDIS), mass storage CB/CBI, and some other
 * protocols do use them.
 */
static int read_ep0_fifo(struct tmpa9xx_ep *ep, struct tmpa9xx_request *req)
{
	u16 length;
	u16 interrupt_status;
	u16 *buf;
	struct tmpa9xx_udc *udc = ep->udc;

	buf = req->req.buf + req->req.actual;
	prefetch(buf);
	length = req->req.length - req->req.actual;

	dev_dbg(udc->dev, "%s(): '%s', length %d\n", __func__, ep->ep.name, length);

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
		return 0;
	}

	req->req.actual += length;
	while (length != 0) {	/* write transmit data to endpoint0's Fifo */
		if (length == 1) {
			u16 tmp;
			u8 *ptr = (u8 *) buf;
			udc2_reg_read(udc, UD2EP0_FIFO, &tmp);
			*ptr = tmp & 0xff;
			break;
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

	done(ep, req, 0);
	return 1;
}

/* pull OUT packet data from the endpoint's fifo */
static int read_fifo(struct tmpa9xx_ep *ep, struct tmpa9xx_request *req)
{
	struct tmpa9xx_udc *udc = ep->udc;
	u8 *buf;
	unsigned int bufferspace = 0, is_done = 0;
	u32 intsts;
	u16 count = 0;

	dev_dbg(udc->dev, "%s(): '%s'\n", __func__, ep->ep.name);

	BUG_ON(!req);

	if (udc->dma_status == DMA_READ_START) {
		dev_dbg(udc->dev, "%s(): '%s', busy\n", __func__, ep->ep.name);
		return 0;	//busy
	}

	dev_dbg(udc->dev, "%s(): req.length %d, dma status %d\n", __func__, req->req.length, udc->dma_status);

	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;

	if (udc->dma_status == DMA_READ_COMPLETE) {
		udc->dma_status = DMA_READ_IDLE;
		memcpy(buf, udc->buf, ep->datasize);	//GCH

		//Check for last write error
		intsts = tmpa9xx_ud2ab_read(udc, UD2AB_INTSTS);
		if ((intsts & INT_MW_AHBERR) == INT_MW_AHBERR) {
			tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_MW_AHBERR);
			tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MW_RESET);
			dev_dbg(udc->dev, "%s(): '%s', crazy condition 1\n", __func__, ep->ep.name);
			return 0;
		}
		if ((intsts & INT_MW_RD_ERR) == INT_MW_RD_ERR) {
			tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_MW_RD_ERR);
			tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MW_RESET);
			dev_dbg(udc->dev, "%s(): '%s', crazy condition 2\n", __func__, ep->ep.name);
			return 0;
		}

		req->req.actual += ep->datasize;
		is_done = (ep->datasize < ep->ep.maxpacket);
		if (ep->datasize == bufferspace)
			is_done = 1;

		PACKET("%s %p out/%d%s\n", ep->ep.name, &req->req, count, is_done ? " (done)" : "");
	}

	/*
	 * avoid extra trips through IRQ logic for packets already in
	 * the fifo ... maybe preventing an extra (expensive) OUT-NAK
	 */
	if (is_done)
		done(ep, req, 0);
	else {
		dev_dbg(udc->dev, "%s(): is_done=%x\n", __func__, is_done);
		ep->datasize = 0;
		udc2_reg_read(udc, UD2EP2_DataSize, &count);
		count &= UD2EP_DATASIZE_MASK;
		if (udc->dma_status == 1)
			dev_dbg(udc->dev, "%s(): count=%x,%x\n", __func__, count, udc->dma_status);
		if (count > 0) {
			if (count > ep->ep.maxpacket)
				count = ep->ep.maxpacket;
			if (count > bufferspace) {
				dev_dbg(udc->dev, "%s(): '%s' buffer overflow\n", __func__, ep->ep.name);
				req->req.status = -EOVERFLOW;
				count = bufferspace;
			}
			ep->datasize = count;
			usb_bulk_out(ep);
			dev_dbg(udc->dev, "%s(): bulk out size=%x\n", __func__, ep->datasize);
		} else
			count = 0;
	}
	return is_done;
}

/* load fifo for an IN packet */
static int write_fifo(struct tmpa9xx_ep *ep, struct tmpa9xx_request *req)
{
	struct tmpa9xx_udc *udc = ep->udc;
	int bufferspace, count;
	unsigned char *buf;

	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;

	if (!bufferspace) {
		dev_dbg(udc->dev, "%s(): %p @ '%s' -- done\n", __func__, &req->req, ep->ep.name);
		done(ep, req, 0);
		return 1;
	}

	if (ep->ep.maxpacket < bufferspace)
		count = ep->ep.maxpacket;
	else
		count = bufferspace;

	usb_bulk_in(ep, buf, count);
	req->req.actual += count;
	dev_dbg(udc->dev, "%s(): %p @ '%s', count %d\n", __func__, &req->req, ep->ep.name, count);
	return 0;
}

static void usb_ctl_init(struct tmpa9xx_udc *udc)
{
	u32 reg_data;

	/* standdard Class initialize    */
	udc->state = DEFAULT;
	udc->config = USB_INIT;
	udc->config_bak = USB_INIT;
	udc->stage = IDLE_STAGE;

	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_PWCTL);

	reg_data &= PWCTL_PHY_POWER_RESET_ON;	/* [5][1] <= 0 */
	reg_data |= PWCTL_PHY_SUSPEND_ON;	/* [3] <= 1 */

	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);

	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_PWCTL);
	reg_data |= PWCTL_PHY_RESET_OFF;	/* [5][3] <= 1 */

	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);

	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_PWCTL);

	reg_data &= PWCTL_PHY_SUSPEND_OFF;	/* [3] <= 0 */

	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);

	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_PWCTL);

	reg_data |= PWCTL_POWER_RESET_OFF;	/* [2] <= 1 */

	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);

	udc2_reg_write(udc, UD2INT, UDC2_INT_MASK);	/*INT EPx MASK & Refresh; */

	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, UDC2AB_INT_ALL_CLEAR);

	tmpa9xx_ud2ab_write(udc, UD2AB_INTENB, UDC2AB_INT_MASK);

	tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MR_RESET | UDC2AB_MW_RESET);
}

/*
 * this is a PIO-only driver, so there's nothing
 * interesting for request or buffer allocation.
 */

static struct usb_request *tmpa9xx_ep_alloc_request(struct usb_ep *_ep, unsigned int gfp_flags)
{
	struct tmpa9xx_ep *ep = container_of(_ep, struct tmpa9xx_ep, ep);
	struct tmpa9xx_udc *udc = ep->udc;
	struct tmpa9xx_request *req;

	req = kzalloc(sizeof(struct tmpa9xx_request), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);

	dev_dbg(udc->dev, "%s(): %p\n", __func__, &req->req);

	return &req->req;
}

static void tmpa9xx_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct tmpa9xx_ep *ep = container_of(_ep, struct tmpa9xx_ep, ep);
	struct tmpa9xx_udc *udc = ep->udc;
	struct tmpa9xx_request *req;

	req = container_of(_req, struct tmpa9xx_request, req);
	kfree(req);

	dev_dbg(udc->dev, "%s(): %p\n", __func__, _req);
}

static int tmpa9xx_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct tmpa9xx_ep *ep = container_of(_ep, struct tmpa9xx_ep, ep);
	struct tmpa9xx_udc *udc = ep->udc;

	unsigned long flags;
	int status = 0;

	dev_dbg(udc->dev, "%s(): '%s'\n", __func__, ep->ep.name);

	local_irq_save(flags);

	if (strncmp(_ep->name, ep_name[1], sizeof(ep_name[1])) == 0) {
		udc2_reg_write(udc, UD2CMD, EP1_STALL);	/* EP1 STALL */
	} else if (strncmp(_ep->name, ep_name[2], sizeof(ep_name[2])) == 0) {
		udc2_reg_write(udc, UD2CMD, EP2_STALL);	/* EP2 STALL */
	} else if (strncmp(_ep->name, ep_name[3], sizeof(ep_name[3])) == 0) {
		udc2_reg_write(udc, UD2CMD, EP3_STALL);	/* EP3 STALL */
	} else
		udc2_reg_write(udc, UD2CMD, EP0_STALL);	/* EP0 STALL */

	local_irq_restore(flags);
	return status;
}

static int tmpa9xx_ep_enable(struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
	struct tmpa9xx_ep *ep = container_of(_ep, struct tmpa9xx_ep, ep);
	struct tmpa9xx_udc *udc = ep->udc;
	u16 maxpacket;
	u32 tmp;
	unsigned long flags;

	maxpacket = le16_to_cpu(desc->wMaxPacketSize);
	if (!_ep || !ep || !desc || ep->desc || _ep->name == ep_name[0]
	    || desc->bDescriptorType != USB_DT_ENDPOINT || maxpacket == 0 || maxpacket > ep->maxpacket) {
		dev_err(udc->dev, "%s(): bad ep or descriptor\n", __func__);
		return -EINVAL;
	}

	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		dev_err(udc->dev, "%s(): bogus device state\n", __func__);
		return -ESHUTDOWN;
	}

	tmp = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	switch (tmp) {
	case USB_ENDPOINT_XFER_CONTROL:
		dev_err(udc->dev, "%s(): only one control endpoint\n", __func__);
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
		dev_err(udc->dev, "%s(): bogus maxpacket %d\n", __func__, maxpacket);
		return -EINVAL;
	case USB_ENDPOINT_XFER_ISOC:
		if (!ep->is_pingpong) {
			dev_err(udc->dev, "%s(): iso requires double buffering\n", __func__);
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
	struct tmpa9xx_ep *ep = container_of(_ep, struct tmpa9xx_ep, ep);
	struct tmpa9xx_udc *udc = ep->udc;

	dev_dbg(udc->dev, "%s(): '%s'\n", __func__, ep->ep.name);

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
		INIT_LIST_HEAD(&ep->queue);
	}
}

union setup {
	u8 raw[8];
	struct usb_ctrlrequest r;
};

static int handle_ep(struct tmpa9xx_ep *ep)
{
	struct tmpa9xx_udc *udc = ep->udc;
	struct tmpa9xx_request *req;
	int ret;

again:
	if (list_empty(&ep->queue)) {
		dev_dbg(udc->dev, "%s(): '%s', list is empty\n", __func__, ep->ep.name);
		return 0;
	}

	req = list_entry(ep->queue.next, struct tmpa9xx_request, queue);
	dev_dbg(udc->dev, "%s(): '%s', request %p\n", __func__, ep->ep.name, req);

	if (ep->is_in) {
		ret = write_fifo(ep, req);
		if (ret) {
			dev_dbg(udc->dev, "%s(): finished write, next request\n", __func__);
			goto again;
		}
	}

	return read_fifo(ep, req);
}

static void handle_setup(struct tmpa9xx_udc *udc, struct tmpa9xx_ep *ep, u32 csr)
{
	u16 request_reg;
	union setup pkt;
	u8 index;
	int status = 0;

	udc2_reg_read(udc, UD2BRQ, &request_reg);

	pkt.r.bRequestType = (u8) (request_reg & MASK_UINT16_LOWER_8BIT);
	pkt.r.bRequest = (u8) ((request_reg >> SHIFT_8BIT) & MASK_UINT16_LOWER_8BIT);

	udc2_reg_read(udc, UD2VAL, &pkt.r.wValue);
	udc2_reg_read(udc, UD2IDX, &pkt.r.wIndex);
	udc2_reg_read(udc, UD2LEN, &pkt.r.wLength);

	ep->stopped = 0;

#define w_index		le16_to_cpu(pkt.r.wIndex)
#define w_value		le16_to_cpu(pkt.r.wValue)
#define w_length	le16_to_cpu(pkt.r.wLength)

	dev_dbg(udc->dev, "SETUP %02x.%02x v%04x i%04x l%04x\n", pkt.r.bRequestType, pkt.r.bRequest, w_value, w_index, w_length);
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

		dev_dbg(udc->dev, "%s(): '%s', (USB_TYPE_STANDARD | USB_RECIP_DEVICE) << 8) | USB_REQ_SET_ADDRESS\n", __func__, ep->ep.name);

		udc->addr = w_value;

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
		/* CONFG is toggled later, if gadget driver succeeds */

		dev_dbg(udc->dev, "%s(): '%s', (USB_TYPE_STANDARD | USB_RECIP_DEVICE) << 8) | USB_REQ_SET_CONFIGURATION:\n", __func__, ep->ep.name);

		index = (u8) (pkt.r.wValue & MASK_UINT16_LOWER_8BIT);

		if (udc->addr == 0) {
			goto stall;
		}

		if (index == 0) {	/* Config 0 */
			udc->config_bak = index;
			udc->state_bak = ADDRESSED;
		} else {
			udc2_reg_write(udc, UD2CMD, All_EP_INVALID);	/*  INVALID */

			udc2_reg_write(udc, UD2EP1_MaxPacketSize, udc->ep[1].maxpacket);
			udc2_reg_write(udc, UD2EP1_Status, EP_DUAL_BULK_IN);
			udc2_reg_write(udc, UD2EP2_MaxPacketSize, udc->ep[2].maxpacket);
			udc2_reg_write(udc, UD2EP2_Status, EP_DUAL_BULK_OUT);
			udc2_reg_write(udc, UD2EP3_MaxPacketSize, udc->ep[3].maxpacket);
			udc2_reg_write(udc, UD2EP3_Status, 0xc08c);

			udc2_reg_write(udc, UD2CMD, EP1_RESET);	/*EP1 Reset */
			udc2_reg_write(udc, UD2CMD, EP2_RESET);	/*EP2 Reset */
			udc2_reg_write(udc, UD2CMD, EP3_RESET);	/*EP3 Reset */

			udc->state_bak = CONFIGURED;
			udc->config_bak = index;
			udc->wait_for_config_ack = 1;
		}

		udc->stage = STATUS_STAGE;
		break;

		/*
		 * Hosts may set or clear remote wakeup status, and
		 * devices may report they're VBUS powered.
		 */
	default:
		dev_dbg(udc->dev, "%s(): '%s', request unhandled by udc driver\n", __func__, ep->ep.name);
		break;
	}

#undef w_value
#undef w_index
#undef w_length

	/* pass request up to the gadget driver */
	if (udc->driver)
		status = udc->driver->setup(&udc->gadget, &pkt.r);
	else
		status = -ENODEV;

	dev_dbg(udc->dev, "%s(): '%s', gadget driver status %d\n", __func__, ep->ep.name, status);

	if (status < 0) {
stall:
		dev_dbg(udc->dev, "req %02x.%02x protocol STALL; stat %d\n", pkt.r.bRequestType, pkt.r.bRequest, status);
		udc->req_pending = 0;
	}

	return;
}

static void handle_ep0(struct tmpa9xx_udc *udc)
{
	struct tmpa9xx_ep *ep0 = &udc->ep[0];
	struct tmpa9xx_request *req;

	udc2_reg_write(udc, UD2INT, INT_EP0_CLEAR);
	if (list_empty(&ep0->queue))
		req = NULL;
	else
		req = list_entry(ep0->queue.next, struct tmpa9xx_request, queue);

	dev_dbg(udc->dev, "%s(): '%s', req %p\n", __func__, ep0->ep.name, req);

	/* host ACKed an IN packet that we sent */
	if (ep0->is_in) {	/*Write */
		/* write more IN DATA? */
		if (req && ep0->is_in) {
			write_ep0_fifo(ep0, req);

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
				if (read_ep0_fifo(ep0, req)) {
					/* send IN/STATUS */
					PACKET("ep0 in/status\n");
					udc->req_pending = 0;
				}
			} else if (udc->req_pending) {
				dev_dbg(udc->dev, "%s(): no control-OUT deferred responses!\n", __func__);
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

static int tmpa9xx_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct tmpa9xx_request *req;
	struct tmpa9xx_ep *ep;
	struct tmpa9xx_udc *udc;
	int status;
	unsigned long flags;

	BUG_ON(!_ep);

	req = container_of(_req, struct tmpa9xx_request, req);
	ep = container_of(_ep, struct tmpa9xx_ep, ep);

	udc = ep->udc;

	if (!_req || !_req->complete || !_req->buf || !list_empty(&req->queue)) {
		dev_err(udc->dev, "%s(): invalid request\n", __func__);
		return -EINVAL;
	}

	if (!ep->desc && ep->ep.name != ep_name[0]) {
		dev_err(udc->dev, "%s(): invalid ep\n", __func__);
		return -EINVAL;
	}

	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		dev_err(udc->dev, "%s(): invalid device\n", __func__);
		return -EINVAL;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	local_irq_save(flags);

	dev_dbg(udc->dev, "%s(): '%s', stopped %d, req %p, empty %d\n", __func__, ep->ep.name, ep->stopped, _req, list_empty(&ep->queue));

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

			if (!udc->req_pending) {
				dev_dbg(udc->dev, "%s(): '%s', pending\n", __func__, ep->ep.name);
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
				udc->req_pending = 0;
				dev_dbg(udc->dev, "%s(): '%s', req.length == 0\n", __func__, ep->ep.name);
				goto done;
			} else {
				dev_dbg(udc->dev, "%s(): ep0 len=%d, in=%x\n", __func__, req->req.length, ep->is_in);
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
				dev_dbg(udc->dev, "%s(): '%s', req.length == 0 (2)\n", __func__, ep->ep.name);
				goto done;
			}
		}
	} else {
		dev_dbg(udc->dev, "%s(): '%s', no kickstart\n", __func__, ep->ep.name);
		status = 0;
	}

	if (req && !status) {
		list_add_tail(&req->queue, &ep->queue);
	}
done:
	local_irq_restore(flags);

	dev_dbg(udc->dev, "%s(): '%s', status %d\n", __func__, ep->ep.name, status);

	return (status < 0) ? status : 0;

}

static void nop_release(struct device *dev)
{
	/* nothing to free */
}

static const struct usb_gadget_ops tmpa9xx_udc_ops = {
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

	dev_dbg(udc->dev, "%s():\n", __func__);

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

static irqreturn_t tmpa9xx_udc_irq(int irq, void *priv)
{
	struct tmpa9xx_udc *udc = priv;

	disable_irq_nosync(udc->udp_irq);
	queue_work(udc->wqs, &udc->ws);

	return IRQ_HANDLED;
}

static void backend_irq_work(struct work_struct *work)
{
	struct tmpa9xx_udc *udc = container_of(work, struct tmpa9xx_udc, ws);
	u32 status;

	status = tmpa9xx_ud2ab_read(udc, UD2AB_INTSTS);
	if (!status)
		goto out;

	/* USB reset irq:  not maskable */
	if ((status & INT_RESET) == INT_RESET) {
		tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_RESET);
		dev_dbg(udc->dev, "%s(): end bus reset\n", __func__);
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
			dev_dbg(udc->dev, "%s(): high speed\n", __func__);
			udc->gadget.speed = USB_SPEED_HIGH;
			udc->ep[1].maxpacket = EP_MAX_PACKET_SIZE_HS;
			udc->ep[2].maxpacket = EP_MAX_PACKET_SIZE_HS;
		} else {
			udc->gadget.speed = USB_SPEED_FULL;
			udc->ep[1].maxpacket = EP_MAX_PACKET_SIZE_FS;
			udc->ep[2].maxpacket = EP_MAX_PACKET_SIZE_FS;
		}
		tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MR_RESET | UDC2AB_MW_RESET);
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
		dev_dbg(udc->dev, "%s(): read complete\n", __func__);
		handle_ep(ep);
	} else if ((status & INT_MR_END_ADD) == INT_MR_END_ADD) {
		tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_MR_END_ADD);
		dev_dbg(udc->dev, "%s(): write complete\n", __func__);
		handle_ep(udc->dma_ep);
	} else if ((status & INT_EP) == INT_EP) {
		u16 reg_data;
		struct tmpa9xx_ep *ep3 = &udc->ep[3];
		struct tmpa9xx_ep *ep2 = &udc->ep[2];
		struct tmpa9xx_ep *ep1 = &udc->ep[1];
		udc2_reg_write(udc, UD2INT, INT_EP_CLEAR);
		udc2_reg_read(udc, UD2EP1_MaxPacketSize, &reg_data);
		if ((reg_data & UD2EP_DSET) == UD2EP_DSET) {	/*dset? */
			dev_dbg(udc->dev, "%s(): '%s', irq\n", __func__, ep1->ep.name);
			handle_ep(ep1);
		}
		udc2_reg_read(udc, UD2EP2_MaxPacketSize, &reg_data);
		if ((reg_data & UD2EP_DSET) == UD2EP_DSET) {	/*dset? */
			dev_dbg(udc->dev, "%s(): '%s', irq\n", __func__, ep2->ep.name);
			handle_ep(ep2);
		}
		udc2_reg_read(udc, UD2EP3_MaxPacketSize, &reg_data);
		if ((reg_data & UD2EP_DSET) == UD2EP_DSET) {	/*dset? */
		dev_dbg(udc->dev, "%s(): '%s', irq\n", __func__, ep3->ep.name);
			handle_ep(ep3);
		}
	} else if ((status & INT_SOF) == INT_SOF) {
		udc2_reg_write(udc, UD2INT, INT_SOF_CLEAR);
	} else if ((status & INT_STATUS) == INT_STATUS) {
		u16 status;
		udc2_reg_write(udc, UD2INT, INT_STATUS_CLEAR);
		if (udc->stage == STATUS_STAGE) {
			/* STATUS NAK Interrupt Enable. */
			udc2_reg_read(udc, UD2INT, &status);
			status |= INT_STATUSNAK_MASK;
			udc2_reg_write(udc, UD2INT, status);
			if (udc->wait_for_config_ack || udc->wait_for_addr_ack) {
				udc->state = udc->state_bak;
				udc->state |= udc->addr;
				udc2_reg_write(udc, UD2ADR, udc->state);
				if (udc->wait_for_config_ack) {
					udc->config = udc->config_bak;
					udc->wait_for_config_ack = 0;
				}
				if (udc->wait_for_addr_ack) {
					udc->wait_for_addr_ack = 0;
				}
			}
			/* DO NOTHING */
			udc->stage = IDLE_STAGE;
		}
		} else if ((status & INT_STATUSNAK) == INT_STATUSNAK) {
		udc2_reg_write(udc, UD2INT, INT_STATUSNAK_CLEAR);
		udc2_reg_write(udc, UD2CMD, SETUP_FIN);
		/* endpoint IRQs are cleared by handling them */
	}

out:
	enable_irq(udc->udp_irq);
}

int usb_gadget_probe_driver(struct usb_gadget_driver *driver, int (*bind) (struct usb_gadget *))
{
	struct tmpa9xx_udc *udc = &controller;
	int retval;

	dev_dbg(udc->dev, "%s(): usb_gadget_register_driver\n", __func__);

	if (!driver || driver->speed < USB_SPEED_FULL || !bind || !driver->setup) {
		dev_err(udc->dev, "bad parameter.\n");
		return -EINVAL;
	}

	if (udc->driver) {
		dev_err(udc->dev, "%s(): UDC already has a gadget driver\n", __func__);
		return -EBUSY;
	}

	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	dev_set_drvdata(&udc->gadget.dev, &driver->driver);
	udc->enabled = 1;
	udc->selfpowered = 1;

	retval = bind(&udc->gadget);
	if (retval) {
		dev_dbg(udc->dev, "%s(): driver->bind() returned %d\n", __func__, retval);
		udc->driver = NULL;
		udc->gadget.dev.driver = NULL;
		dev_set_drvdata(&udc->gadget.dev, NULL);
		udc->enabled = 0;
		udc->selfpowered = 0;
		return retval;
	}
	udc2_reg_write(udc, UD2CMD, USB_READY);	//GCH
	enable_irq(udc->udp_irq);

	dev_dbg(udc->dev, "%s(): bound to %s\n", __func__, driver->driver.name);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_probe_driver);

static void pwctl_power_reset(struct tmpa9xx_udc *udc)
{
	u32 reg_data;

	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_PWCTL);
	reg_data &= PWCTL_POWER_RESET_ON;
	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);
	reg_data |= PWCTL_POWER_RESET_OFF;
	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);
}

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct tmpa9xx_udc *udc = &controller;

	dev_dbg(udc->dev, "%s(): usb_gadget_unregister_driver\n", __func__);

	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;

	udc->enabled = 0;
	disable_irq(udc->udp_irq);
	udc2_reg_write(udc, UD2CMD, All_EP_INVALID);

        driver->disconnect(&udc->gadget);
	driver->unbind(&udc->gadget);

	udc->driver = NULL;
	udc->gadget.dev.driver = NULL;

	pwctl_power_reset(udc);

	dev_dbg(udc->dev, "%s(): unbound from %s\n", __func__, driver->driver.name);

	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

static int __devinit tmpa9xx_udc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tmpa9xx_udc *udc;
	int retval;
	struct resource *res;

	if (pdev->num_resources != 2) {
		dev_err(&pdev->dev, "invalid num_resources");
		return -ENODEV;
	}

	if ((pdev->resource[0].flags != IORESOURCE_MEM)
	    || (pdev->resource[1].flags != IORESOURCE_IRQ)) {
		dev_err(&pdev->dev, "invalid resource type");
		return -ENODEV;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	if (!request_mem_region(res->start, res->end - res->start + 1, driver_name)) {
		dev_err(&pdev->dev, "someone's using UDC memory\n");
		return -EBUSY;
	}

	/* init software state */
	udc = &controller;

	udc->wqs = create_workqueue("tmpa9xx-udc-wq");
	if (!udc->wqs) {
		dev_err(udc->dev, "%s(): create_workqueue() failed\n", __func__);
		/* ah, fixme error handling */
		return -EBUSY;
	}

	INIT_WORK(&udc->ws, backend_irq_work);

	sema_init(&udc->sem, 1);

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
	udc->dev = dev;
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

	udc->buf = (unsigned char *)dma_alloc_coherent(NULL, 2112, &udc->phy_buf, GFP_KERNEL);
	if (udc->buf == NULL) {
		dev_err(udc->dev, "%s(): dma_alloc_coherent() failed\n", __func__);
		retval = -ENOMEM;
		goto fail1;
	}

	/* request UDC irqs */
	udc->udp_irq = platform_get_irq(pdev, 0);
	if (request_irq(udc->udp_irq, tmpa9xx_udc_irq, IRQF_DISABLED, driver_name, udc)) {
		dev_err(udc->dev, "%s(): request_irq() failed\n", __func__);
		retval = -EBUSY;
		goto fail2;
	}

	disable_irq(udc->udp_irq);
	udc2_reg_write(udc, UD2CMD, All_EP_INVALID);

	dev_set_drvdata(dev, udc);
	device_init_wakeup(dev, 1);

	dev_info(udc->dev, "%s ready\n", driver_name);

	return 0;
fail2:
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

	if (udc->driver)
		return -EBUSY;

	device_init_wakeup(&pdev->dev, 0);

	free_irq(udc->udp_irq, udc);
	device_unregister(&udc->gadget.dev);

	iounmap(udc->udp_baseaddr);

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
MODULE_AUTHOR("Michael Hunold");
MODULE_LICENSE("GPL");
