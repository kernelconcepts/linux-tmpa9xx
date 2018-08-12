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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "tmpa9xx-udc.h"

#define	DMA_ADDR_INVALID (~(dma_addr_t)0)

static const char driver_name[] = "tmpa9xx-udc";

static const char *const ep_name[] = {
	"ep0",
	"ep1in-bulk",
	"ep2out-bulk",
	"ep3in-int"
};

#define tmpa9xx_ud2ab_read(dev, reg) \
	__raw_readl((dev)->base + (reg))

#define tmpa9xx_ud2ab_write(dev, reg, val) \
	__raw_writel((val), (dev)->base + (reg))

static void udc2_reg_read(struct tmpa9xx_udc *udc, const u32 addr, u16 *data)
{
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&udc->reg_slock, flags);

	/* policy is that UDC access is alwas done, if not, it's a bug */
	BUG_ON(tmpa9xx_ud2ab_read(udc, UD2AB_UDC2RDREQ) & UDC2AB_READ_RQ);

	tmpa9xx_ud2ab_write(udc, UD2AB_UDC2RDREQ, (addr & UDC2AB_READ_ADDRESS) | UDC2AB_READ_RQ);

	/* wait for UDC access to be done */
	do {
		reg = tmpa9xx_ud2ab_read(udc, UD2AB_UDC2RDREQ);
	} while ((reg & UDC2AB_READ_RQ));
	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_UDC2REG_RD);

	spin_unlock_irqrestore(&udc->reg_slock, flags);

	*data = tmpa9xx_ud2ab_read(udc, UD2AB_UDC2RDVL);
}

static void udc2_reg_write(struct tmpa9xx_udc *udc, const u32 addr, const u16 data)
{
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&udc->reg_slock, flags);

	/* policy is that UDC access is alwas done, if not, it's a bug */
	BUG_ON(tmpa9xx_ud2ab_read(udc, UD2AB_UDC2RDREQ) & UDC2AB_READ_RQ);

	tmpa9xx_ud2ab_write(udc, addr, data);

	/* wait for UDC access to be done */
	do {
		reg = tmpa9xx_ud2ab_read(udc, UD2AB_UDC2RDREQ);
	} while ((reg & UDC2AB_READ_RQ));
	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_UDC2REG_RD);

	spin_unlock_irqrestore(&udc->reg_slock, flags);
}

static void done(struct tmpa9xx_ep *ep, struct tmpa9xx_request *req, int status)
{
	struct tmpa9xx_udc *udc = ep->udc;
	int stopped = ep->stopped;

	dev_dbg(udc->dev, "%s(): '%s', req %p\n", __func__, ep->ep.name, req);

	BUG_ON(req->req.status != -EINPROGRESS);

	if (ep->has_dma && req->req.dma != DMA_ADDR_INVALID) {
		dev_dbg(udc->dev, "%s(): unmap dma 0x%08x @ buf %p\n", __func__, req->req.dma, req);
		dma_unmap_single(udc->gadget.dev.parent,
			req->req.dma, req->req.length,
			ep->is_in ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		req->req.dma = DMA_ADDR_INVALID;
	}


	ep->stopped = 1;
	req->req.status = status;
	req->req.complete(&ep->ep, &req->req);
	ep->stopped = stopped;
}

static struct usb_request *tmpa9xx_ep_alloc_request(struct usb_ep *_ep, unsigned int gfp_flags)
{
	struct tmpa9xx_ep *ep = container_of(_ep, struct tmpa9xx_ep, ep);
	struct tmpa9xx_udc *udc = ep->udc;
	struct tmpa9xx_request *req;

	req = kzalloc(sizeof(struct tmpa9xx_request), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);
	req->req.dma = DMA_ADDR_INVALID;

	dev_dbg(udc->dev, "%s(): %p @ '%s'\n", __func__, &req->req, ep->ep.name);

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

static inline void udc2_cmd_ep(struct tmpa9xx_ep *ep, int cmd)
{
	struct tmpa9xx_udc *udc = ep->udc;

	udc2_reg_write(udc, UD2CMD, (ep->num << 4) | cmd);
}

static int __ep_set_halt(struct tmpa9xx_udc *udc, struct tmpa9xx_ep *ep)
{
	dev_dbg(udc->dev, "%s(): '%s'\n", __func__, ep->ep.name);

	if (!ep->num)
		return 0;
#if 0
	udc2_cmd_ep(ep, EP_STALL);
#endif
	ep->is_halted = 1;

	return 0;
}

static int tmpa9xx_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct tmpa9xx_ep *ep = container_of(_ep, struct tmpa9xx_ep, ep);
	struct tmpa9xx_udc *udc = ep->udc;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ep->s, flags);

	ret = __ep_set_halt(udc, ep);

	spin_unlock_irqrestore(&ep->s, flags);

	return ret;
}

static int tmpa9xx_ep_enable(struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
	struct tmpa9xx_ep *ep = container_of(_ep, struct tmpa9xx_ep, ep);
	struct tmpa9xx_udc *udc = ep->udc;
	u16 maxpacket;
	u32 tmp;

	maxpacket = le16_to_cpu(desc->wMaxPacketSize);

	if (!_ep || !ep) {
		dev_err(udc->dev, "%s(): bad ep\n", __func__);
		return -EINVAL;
	}

	if (!desc || ep->desc) {
		dev_err(udc->dev, "%s(): bad descriptor\n", __func__);
		return -EINVAL;
	}

	if (!ep->num) {
		dev_err(udc->dev, "%s(): ep[0]\n", __func__);
		return -EINVAL;
	}

	if (desc->bDescriptorType != USB_DT_ENDPOINT) {
		dev_err(udc->dev, "%s():not USB_DT_ENDPOINT\n", __func__);
		return -EINVAL;
	}

	if (!maxpacket || maxpacket > ep->maxpacket) {
		dev_err(udc->dev, "%s(): maxpacket %d\n", __func__, maxpacket);
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
		if (maxpacket > 64) {
			dev_err(udc->dev, "%s(): '%s', bogus maxpacket %d for XFER_INT\n", __func__, ep->ep.name, maxpacket);
			return -EINVAL;
		}
		break;
	case USB_ENDPOINT_XFER_BULK:
		switch (maxpacket) {
		case 8:
		case 16:
		case 32:
		case 64:
		case 512:
			break;
		default: {
			dev_err(udc->dev, "%s(): '%s', bogus maxpacket %d for XFER_BULK\n", __func__, ep->ep.name, maxpacket);
			return -EINVAL;
		}
		}
		break;
	case USB_ENDPOINT_XFER_ISOC:
		dev_err(udc->dev, "%s(): USB_ENDPOINT_XFER_ISOC not supported yet.\n", __func__);
		return -EINVAL;
	}

	/* initialize endpoint to match this descriptor */
	ep->is_in = (desc->bEndpointAddress & USB_DIR_IN) != 0;
	ep->is_iso = (tmp == USB_ENDPOINT_XFER_ISOC);
	ep->is_halted = 0;
	ep->desc = desc;
	ep->ep.maxpacket = maxpacket;
	ep->stopped = 0;
	spin_lock_init(&ep->s);

	dev_dbg(udc->dev, "%s(): '%s', is_in %d, is_iso %d, maxpacket %d\n", __func__, ep->ep.name, ep->is_in, ep->is_iso, maxpacket);

	udc2_reg_write(udc, UD2EPx_MAXPACKETSIZE(ep->num), maxpacket);

	if (ep->num == 1) {
		udc2_reg_write(udc, UD2EPx_STATUS(ep->num), EP_DUAL_BULK_IN);
	} else if (ep->num == 2) {
		udc2_reg_write(udc, UD2EPx_STATUS(ep->num), EP_DUAL_BULK_OUT);
	} else {
		BUG_ON(ep->num != 3);
		udc2_reg_write(udc, UD2EPx_STATUS(ep->num), (1 << 7) | (1 << 3) | (1 << 2));
	}

	udc2_cmd_ep(ep, EP_RESET);

	return 0;
}

static void __nuke(struct tmpa9xx_ep *ep, int status)
{
	struct tmpa9xx_udc *udc = ep->udc;
	struct tmpa9xx_request *req;

	if (ep->cur) {
		done(ep, ep->cur, status);
		dev_dbg(udc->dev, "%s(): nuked %p\n", __func__, ep->cur);
		ep->cur = NULL;
	}

	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct tmpa9xx_request, queue);
		list_del_init(&req->queue);
		done(ep, req, status);
		dev_dbg(udc->dev, "%s(): nuked %p\n", __func__, req);
	}

	dev_dbg(udc->dev, "%s(): '%s'\n", __func__, ep->ep.name);
}

static int tmpa9xx_ep_disable(struct usb_ep *_ep)
{
	struct tmpa9xx_ep *ep = container_of(_ep, struct tmpa9xx_ep, ep);
	struct tmpa9xx_udc *udc = ep->udc;
	uint16_t reg;
	unsigned long flags;

	dev_dbg(udc->dev, "%s(): '%s'\n", __func__, ep->ep.name);

	BUG_ON(!ep->num);

	spin_lock_irqsave(&ep->s, flags);

	ep->desc = NULL;
	__nuke(ep, -ESHUTDOWN);
	ep->cur = NULL;
	ep->ep.maxpacket = ep->maxpacket;
	INIT_LIST_HEAD(&ep->queue);

	udc2_cmd_ep(ep, EP_DISABLE);
	/* wait until disable command is successful */
	while (1) {
		udc2_reg_read(udc, UD2EPx_STATUS(ep->num), &reg);
		if (reg & (1 << 8))
			break;
	}

	/* abort dma if necessary */
	if (ep->num == 1) {
		/* master read */
		tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MR_ABORT);
		while(1) {
			uint32_t stset = tmpa9xx_ud2ab_read(udc, UD2AB_UDMSTSET);
			if (!(stset & UDC2AB_MR_ENABLE))
			 	break;
		}
		tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MR_RESET);
	} else if (ep->num == 2) {
		/* master write */
		tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MW_ABORT);
		while(1) {
			uint32_t stset = tmpa9xx_ud2ab_read(udc, UD2AB_UDMSTSET);
			if (!(stset & UDC2AB_MW_ENABLE))
			 	break;
		}
		tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MW_RESET);
	}

	udc2_cmd_ep(ep, EP_FIFO_CLEAR);

	spin_unlock_irqrestore(&ep->s, flags);
	return 0;
}

static int __write_ep0_fifo(struct tmpa9xx_udc *udc, uint16_t *buf, int length)
{
	struct tmpa9xx_ep *ep = &udc->ep[0];
	int i;

	BUG_ON(!length);

	if (length > ep->ep.maxpacket) {
		length = ep->ep.maxpacket;

		for (i = length; i > 0 ; i -= 2)
			udc2_reg_write(udc, UD2EPx_FIFO(ep->num), *buf++);

		dev_dbg(udc->dev, "%s(): partial write\n", __func__);
		return length;
	}

	for (i = length; i > 0 ; i -= 2) {
		if (i == 1) {
			u8 chardata = (u8) (*buf & 0xff);
			u8 *data_p = (u8 *) (UD2EPx_FIFO(ep->num) + udc->base);
			*data_p = chardata;
			break;
		}
		udc2_reg_write(udc, UD2EPx_FIFO(ep->num), *buf++);
	}

	if (length < ep->ep.maxpacket)
		udc2_cmd_ep(ep, EP_EOP);

	dev_dbg(udc->dev, "%s(): write done\n", __func__);

	return length;
}

static int write_ep0_fifo(struct tmpa9xx_udc *udc, struct tmpa9xx_request *req)
{
	uint16_t *buf = req->req.buf + req->req.actual;
	int length = req->req.length - req->req.actual;

	dev_dbg(udc->dev, "%s(): length %d\n", __func__, length);

	req->req.actual += __write_ep0_fifo(udc, buf, length);

	return 0;
}

static void write_ep3_fifo(struct tmpa9xx_udc *udc, struct tmpa9xx_request *req)
{
	struct tmpa9xx_ep *ep = &udc->ep[3];
	int length = req->req.length - req->req.actual;
	u16 *buf = req->req.buf + req->req.actual;
	int send_eop = 0;

	if (length < ep->ep.maxpacket)
		send_eop = 1;

	dev_dbg(udc->dev, "%s(): length %d\n", __func__, length);

	BUG_ON(length > ep->ep.maxpacket);

	req->req.actual += length;
	while (length > 0) {
		if (length == 1) {
			u8 chardata = (u8) (*buf & 0xff);
			u8 *data_p = (u8 *) (UD2EPx_FIFO(ep->num) + udc->base);
			*data_p = chardata;
			break;
		}
		udc2_reg_write(udc, UD2EPx_FIFO(ep->num), *buf);
		buf++;
		length -= 2;
	}

	if (send_eop)
		udc2_cmd_ep(ep, EP_EOP);

	dev_dbg(udc->dev, "%s(): write done, send_eop %d\n", __func__, send_eop);
}

static int read_ep0_fifo(struct tmpa9xx_udc *udc, struct tmpa9xx_request *req)
{
	struct tmpa9xx_ep *ep = &udc->ep[0];
	int length = req->req.length - req->req.actual;
	u16 *buf = req->req.buf + req->req.actual;

	BUG_ON(!length);

	dev_dbg(udc->dev, "%s(): length %d\n", __func__, length);

	if (length > ep->ep.maxpacket) {

		length = ep->ep.maxpacket;
		req->req.actual += length;

		while (length > 0) {
			udc2_reg_read(udc, UD2EPx_FIFO(ep->num), buf);
			buf++;
			length -= 2;
		}
		return 0;
	}

	req->req.actual += length;
	while (length > 0) {
		if (length == 1) {
			u16 tmp;
			u8 *ptr = (u8 *) buf;
			udc2_reg_read(udc, UD2EPx_FIFO(ep->num), &tmp);
			*ptr = tmp & 0xff;
			break;
		}
		udc2_reg_read(udc, UD2EPx_FIFO(ep->num), buf);
		buf++;
		length -= 2;
	}

	return 1;
}

static void finish_ep0_no_data_stage(struct tmpa9xx_udc *udc)
{
	udc->ackwait = 1;
	udc2_reg_write(udc, UD2CMD, EP_SETUP_FIN);
	udc->ignore_status_ack = 0;
}

static int ep0_queue(struct tmpa9xx_udc *udc, struct tmpa9xx_request *req)
{
	struct tmpa9xx_ep *ep = &udc->ep[0];
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ep->s, flags);

	BUG_ON(!list_empty(&ep->queue));
	BUG_ON(ep->cur);

	ep->cur = req;

	if (!req->req.length) {
		dev_dbg(udc->dev, "%s(): req %p, is_in %d, length == 0. done.\n", __func__, req, ep->is_in);

		finish_ep0_no_data_stage(udc);

		goto out;
	}

	if (ep->stopped || udc->ackwait) {
		dev_dbg(udc->dev, "%s(): stopped %d, ackwait %d\n", __func__, ep->stopped, udc->ackwait);
		goto out;
	}

	if (ep->is_in) {
		dev_dbg(udc->dev, "%s(): is_in %d, do write req\n", __func__, ep->is_in);
		ret = write_ep0_fifo(udc, req);
		goto out;
	}

	dev_dbg(udc->dev, "%s(): is_in %d, queue read req\n", __func__, ep->is_in);

	/* will be handled with ep0 interrupt */
out:
	spin_unlock_irqrestore(&ep->s, flags);
	return 0;
}

static int ep1_dma_helper(struct tmpa9xx_udc *udc, struct tmpa9xx_request *req)
{
	struct tmpa9xx_ep *ep = &udc->ep[1];
	int len;

	len = req->req.length - req->req.actual;
	if (len > ep->ep.maxpacket)
		len = ep->ep.maxpacket;

	/* transmit a zero-length packet, no DMA necessary */
	if (!len) {
		dev_dbg(udc->dev, "%s(): req %p, todo %d, len %d, (zlp)\n", __func__, req, req->req.length - req->req.actual, len);
		udc2_cmd_ep(ep, EP_TX_0DATA);
		return 1;
	}

	dev_dbg(udc->dev, "%s(): req %p, todo %d, len %d\n", __func__, req, req->req.length - req->req.actual, len);

	udc->r_len = len;
	tmpa9xx_ud2ab_write(udc, UD2AB_MRSADR, req->req.dma + req->req.actual);
	tmpa9xx_ud2ab_write(udc, UD2AB_MREADR, req->req.dma + req->req.actual + len - 1);
	tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MR_ENABLE);

	return 0;
}

static void prepare_req_for_dma(struct tmpa9xx_udc *udc, struct tmpa9xx_ep *ep, struct tmpa9xx_request *req)
{
	int ret;

	BUG_ON(req->req.dma != DMA_ADDR_INVALID);

	req->req.dma = dma_map_single(
		udc->gadget.dev.parent,
		req->req.buf,
		req->req.length,
		ep->is_in ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	ret = dma_mapping_error(udc->gadget.dev.parent, req->req.dma);
	BUG_ON(ret);

	dev_dbg(udc->dev, "%s(): map dma 0x%08x @ buf %p\n", __func__, req->req.dma, req);
}

static int handle_ep1_dma_done(struct tmpa9xx_udc *udc)
{
	struct tmpa9xx_ep *ep = &udc->ep[1];
	struct tmpa9xx_request *req;
	unsigned long flags;
	int len;
	int ret;

	spin_lock_irqsave(&ep->s, flags);

	req = ep->cur;
	BUG_ON(!req);

	req->req.actual += udc->r_len;
	len = req->req.length - req->req.actual;
	dev_dbg(udc->dev, "%s(): req %p, finished %d, todo %d\n", __func__, req, udc->r_len, len);
	udc->r_len = 0;

	if (len) {
		ret = ep1_dma_helper(udc, req);
		BUG_ON(ret);
		goto out;
	}

again:

	done(ep, req, 0);
	ep->cur = NULL;

	if (list_empty(&ep->queue)) {
		dev_dbg(udc->dev, "%s(): no more reqs to setup\n", __func__);
		goto out;
	}

	req = list_entry(ep->queue.next, struct tmpa9xx_request, queue);
	list_del_init(&req->queue);
	ep->cur = req;
	prepare_req_for_dma(udc, ep, req);
	dev_dbg(udc->dev, "%s(): new cur is %p\n", __func__, req);

	ret = ep1_dma_helper(udc, ep->cur);
	if (!ret)
		goto out;

	goto again;

out:
	spin_unlock_irqrestore(&ep->s, flags);
	return 0;
}

static int ep1_queue(struct tmpa9xx_udc *udc, struct tmpa9xx_request *req)
{
	struct tmpa9xx_ep *ep = &udc->ep[1];
	unsigned long flags;
	int ret;

	BUG_ON(!ep->is_in);

	spin_lock_irqsave(&ep->s, flags);

	if (ep->stopped) {
		dev_dbg(udc->dev, "%s(): stopped %d, adding req %p to queue\n", __func__, ep->stopped, req);
		list_add_tail(&req->queue, &ep->queue);
		goto out;
	}

	if (ep->cur) {
		dev_dbg(udc->dev, "%s(): busy, req %p, active %p\n", __func__, req, ep->cur);
		list_add_tail(&req->queue, &ep->queue);
		goto out;
	}

	ep->cur = req;
	dev_dbg(udc->dev, "%s(): new cur is %p\n", __func__, req);
	prepare_req_for_dma(udc, ep, req);

	ret = ep1_dma_helper(udc, ep->cur);
	if (ret) {
		done(ep, ep->cur, 0);
		ep->cur = NULL;
	}

out:
	spin_unlock_irqrestore(&ep->s, flags);
	return 0;
}



static int __handle_ep2_rcv(struct tmpa9xx_udc *udc, struct tmpa9xx_request *req)
{
	struct tmpa9xx_ep *ep = &udc->ep[2];
	uint16_t len;

	if (udc->w_len) {
		dev_dbg(udc->dev, "%s(): in progress\n", __func__);
		return 0;
	}

	udc2_reg_read(udc, UD2EPx_DATASIZE(ep->num), &len);
	dev_dbg(udc->dev, "%s(): len %d\n", __func__, len);

	BUG_ON(udc->w_len);
	udc->w_len = len;
	tmpa9xx_ud2ab_write(udc, UD2AB_MWSADR, req->req.dma + req->req.actual);
	tmpa9xx_ud2ab_write(udc, UD2AB_MWEADR, req->req.dma + req->req.actual + len - 1);
	tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MW_ENABLE);

	return 0;
}

static int __ep2_dma_copy(struct tmpa9xx_udc *udc, struct tmpa9xx_request *req)
{
	struct tmpa9xx_ep *ep = &udc->ep[2];

	int len = req->req.length - req->req.actual;
	int ret = 0;

	dev_dbg(udc->dev, "%s(): req %p, len %d, free %d\n", __func__, req, udc->w_len, len);

	BUG_ON(udc->w_len > len);

	req->req.actual += udc->w_len;
	len = req->req.length - req->req.actual;

	if (udc->w_len < ep->ep.maxpacket || !len) {
		ret = 1;
	}

	udc->w_len = 0;
	return ret;
}

static int ep2_xfer_pending(struct tmpa9xx_udc *udc)
{
	uint16_t mststs;
	int ret;

	mststs = tmpa9xx_ud2ab_read(udc, UD2AB_MSTSTS);

	ret = !!(mststs & 0x1);
	dev_dbg(udc->dev, "%s(): state %d\n", __func__, ret);

	return ret;
}

static int handle_ep2_dma_done(struct tmpa9xx_udc *udc)
{
	struct tmpa9xx_ep *ep = &udc->ep[2];
	struct tmpa9xx_request *req;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ep->s, flags);

	BUG_ON(!ep->cur);

	dev_dbg(udc->dev, "%s(): req %p\n", __func__, ep->cur);

	ret = __ep2_dma_copy(udc, ep->cur);
	if (!ret) {
		if (ep2_xfer_pending(udc))
			__handle_ep2_rcv(udc, ep->cur);
		goto out;
	}

	done(ep, ep->cur, 0);
	ep->cur = NULL;

	/* start another transfer right away if necessary */
	if (!ep2_xfer_pending(udc)) {
		dev_dbg(udc->dev, "%s(): no pending request\n", __func__);
		goto out;
	}

	if (list_empty(&ep->queue)) {
		dev_dbg(udc->dev, "%s(): pending request, but no buffer\n", __func__);
		goto out;
	}

	req = list_entry(ep->queue.next, struct tmpa9xx_request, queue);
	list_del_init(&req->queue);
	ep->cur = req;
	prepare_req_for_dma(udc, ep, req);

	dev_dbg(udc->dev, "%s(): new cur %p\n", __func__, ep->cur);
	__handle_ep2_rcv(udc, ep->cur);

out:
	spin_unlock_irqrestore(&ep->s, flags);
	return 0;
}

static int ep2_queue(struct tmpa9xx_udc *udc, struct tmpa9xx_request *req)
{
	struct tmpa9xx_ep *ep = &udc->ep[2];
	unsigned long flags;

	spin_lock_irqsave(&ep->s, flags);

	if (ep->cur || !ep2_xfer_pending(udc)) {
		list_add_tail(&req->queue, &ep->queue);
		dev_dbg(udc->dev, "%s(): req %p added to queue\n", __func__, req);
		goto out;
	}

	ep->cur = req;
	prepare_req_for_dma(udc, ep, req);

	__handle_ep2_rcv(udc, ep->cur);

out:
	spin_unlock_irqrestore(&ep->s, flags);
	return 0;
}
static int ep3_queue(struct tmpa9xx_udc *udc, struct tmpa9xx_request *req)
{
	struct tmpa9xx_ep *ep = &udc->ep[3];
	unsigned long flags;

	BUG_ON(!ep->is_in);

	spin_lock_irqsave(&ep->s, flags);

	if (ep->stopped) {
		dev_dbg(udc->dev, "%s(): stopped %d, adding req %p to queue\n", __func__, ep->stopped, req);
		list_add_tail(&req->queue, &ep->queue);
		goto out;
	}

	list_add_tail(&req->queue, &ep->queue);

	if (ep->cur) {
		dev_dbg(udc->dev, "%s(): req %p active, do nothing with req %p\n", __func__, ep->cur, req);
		goto out;
	}

	req = list_entry(ep->queue.next, struct tmpa9xx_request, queue);
	list_del_init(&req->queue);
	ep->cur = req;

	dev_dbg(udc->dev, "%s(): new cur %p\n", __func__, req);

	write_ep3_fifo(udc, req);

out:
	spin_unlock_irqrestore(&ep->s, flags);
	return 0;
}

static int handle_ep3_snd(struct tmpa9xx_udc *udc)
{
	struct tmpa9xx_ep *ep = &udc->ep[3];
	struct tmpa9xx_request *req;
	unsigned long flags;

	spin_lock_irqsave(&ep->s, flags);

	BUG_ON(!ep->cur);
	req = ep->cur;

	done(ep, req, 0);
	ep->cur = NULL;

	if (list_empty(&ep->queue)) {
		dev_dbg(udc->dev, "%s(): no more requests\n", __func__);
		goto out;
	}

	req = list_entry(ep->queue.next, struct tmpa9xx_request, queue);
	list_del_init(&req->queue);
	ep->cur = req;

	dev_dbg(udc->dev, "%s(): handling next request %p\n", __func__, req);
	write_ep3_fifo(udc, req);

out:
	spin_unlock_irqrestore(&ep->s, flags);
	return 0;
}

static int tmpa9xx_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct tmpa9xx_request *req;
	struct tmpa9xx_ep *ep;
	struct tmpa9xx_udc *udc;
	int ret = 0;

	BUG_ON(!_ep);

	req = container_of(_req, struct tmpa9xx_request, req);
	ep = container_of(_ep, struct tmpa9xx_ep, ep);

	udc = ep->udc;

	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		dev_err(udc->dev, "%s(): invalid device\n", __func__);
		return -EINVAL;
	}

	if (!_req || !_req->complete || !_req->buf || !list_empty(&req->queue)) {
		dev_err(udc->dev, "%s(): invalid request %p\n", __func__, req);
		return -EINVAL;
	}

	if (!ep->desc && ep->num) {
		dev_err(udc->dev, "%s(): invalid ep\n", __func__);
		return -EINVAL;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	dev_dbg(udc->dev, "%s(): '%s', req %p, is_in %d, empty %d\n", __func__, ep->ep.name, _req, ep->is_in, list_empty(&ep->queue));

	if (!ep->num) {
		ret = ep0_queue(udc, req);
	} else if (ep->num == 1) {
		ret = ep1_queue(udc, req);
	} else if (ep->num == 2) {
		ret = ep2_queue(udc, req);
	} else {
		BUG_ON(ep->num != 3);
		ret = ep3_queue(udc, req);
	}

	return ret;
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
		.has_dma = 0,
		.num = 0,
	},
	.ep[1] = {
		.ep = {
			.name = "ep1in-bulk",
			.ops = &tmpa9xx_ep_ops,
		},
		.udc = &controller,
		.maxpacket = 512,
		.has_dma = 1,
		.num = 1,
	},
	.ep[2] = {
		.ep = {
			.name = "ep2out-bulk",
			.ops = &tmpa9xx_ep_ops,
		},
		.udc = &controller,
		.maxpacket = 512,
		.has_dma = 1,
		.num = 2,
	},
	.ep[3] = {
		.ep = {
			.name = "ep3in-int",
			.ops = &tmpa9xx_ep_ops,
		},
		.udc = &controller,
		.maxpacket = 64,
		.has_dma = 0,
		.num = 3,
	},
};

/* helper function to tweak the lower bits of the UD2INT register
for acknowleding interrupts */
static void ud2_int(struct tmpa9xx_udc *udc, uint16_t val)
{
	udc2_reg_write(udc, UD2INT, udc->ud2int | val);
}

static void stop_activity(struct tmpa9xx_udc *udc)
{
	struct tmpa9xx_ep *ep = &udc->ep[0];
	uint32_t intenb;
	int i;

	dev_dbg(udc->dev, "%s():\n", __func__);

	udc->state = DEFAULT;
	udc2_reg_write(udc, UD2CMD, EP_ALL_INVALID);
	udc2_reg_write(udc, UD2INT_EP_MASK, (1 << 2) | (1 << 1));

	__nuke(ep, -ESHUTDOWN);
	udc2_cmd_ep(ep, EP_FIFO_CLEAR);
	udc->ackwait = 0;

	for (i = 1; i < NUM_ENDPOINTS; i++)
		tmpa9xx_ep_disable(&udc->ep[i].ep);

	/* clear all interrupts */
	intenb = tmpa9xx_ud2ab_read(udc, UD2AB_INTENB);
	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, intenb);
}

static int udc_set_address(struct tmpa9xx_udc *udc, int addr)
{
	dev_dbg(udc->dev, "%s(): addr 0x%04x", __func__, addr);

	/* check max. address */
	BUG_ON(addr > 0x7f);

	udc->addr = addr;
	udc->state = ADDRESSED;

	finish_ep0_no_data_stage(udc);

	return 0;
}

static int udc_set_feature(struct tmpa9xx_udc *udc, int type, int feature, int index)
{
	dev_dbg(udc->dev, "%s(): type %d, feature %d, index %d\n", __func__, type, feature, index);

	/* fixme: we only handle endpoints atm */
	BUG_ON(type != USB_RECIP_ENDPOINT);

	/* fixme: we only handle feature endpoint halt atm */
	BUG_ON(feature != USB_ENDPOINT_HALT);

	/* fixme: we only handle stalling ep != 0 atm */
	BUG_ON(index == 0);

	__ep_set_halt(udc, &udc->ep[index]);

	finish_ep0_no_data_stage(udc);

	return 0;
}

static int udc_clear_feature(struct tmpa9xx_udc *udc, int type, int feature, int index)
{
	struct tmpa9xx_ep *ep;

	dev_dbg(udc->dev, "%s(): type %d, feature %d, index %d\n", __func__, type, feature, index);

	/* fixme: we only handle endpoints atm */
	BUG_ON(type != USB_RECIP_ENDPOINT);

	/* fixme: we only handle feature endpoint halt atm */
	BUG_ON(feature != USB_ENDPOINT_HALT);

	ep = &udc->ep[index];

	if (!ep->num) {
		/* no reset command necessary */
	}
#if 0
	udc2_cmd_ep(ep, EP_RESET);
#endif
	ep->is_halted = 0;

	finish_ep0_no_data_stage(udc);

	return 0;
}

static int udc_set_config(struct tmpa9xx_udc *udc, int configuration_value)
{
	dev_dbg(udc->dev, "%s(): configuration_value %d", __func__, configuration_value);

	udc->state = CONFIGURED;

	udc->ackwait = 1;
	/* enable STATUS interrupt to make transition to CONFIGURED state happen */
	udc->ignore_status_ack = 0;

	return 0;
}

static int udc_get_status(struct tmpa9xx_udc *udc, int type, int index)
{
	uint16_t status;

	BUG_ON(index < 0 || index > 2);

	if (type == USB_RECIP_DEVICE) {
		status = 0;
		dev_dbg(udc->dev, "%s(): device status 0x%04x", __func__, status);
	} else if (type == USB_RECIP_INTERFACE) {
		status = 0;
		dev_dbg(udc->dev, "%s(): interface %d status 0x%04x", __func__,index, status);
	} else if (type == USB_RECIP_ENDPOINT) {
		struct tmpa9xx_ep *ep = &udc->ep[index];
		status = !!ep->is_halted;
		dev_dbg(udc->dev, "%s(): endpoint %d status 0x%04x", __func__,index, status);
	} else {
		dev_err(udc->dev, "%s(): type %d, index %d", __func__, type, index);
		BUG();
	}

	__write_ep0_fifo(udc, &status, sizeof(status));

	return 0;
}

static irqreturn_t tmpa9xx_udc_irq(int irq, void *priv)
{
	struct tmpa9xx_udc *udc = priv;

	disable_irq_nosync(udc->irq);
	queue_work(udc->wq, &udc->ws);

	return IRQ_HANDLED;
}

static void int_suspend(struct tmpa9xx_udc *udc)
{
	dev_dbg(udc->dev, "%s():\n", __func__);
	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_SUSPEND);
}

static void int_reset(struct tmpa9xx_udc *udc)
{
	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_RESET);
}

static void int_reset_end(struct tmpa9xx_udc *udc)
{
	u16 state;

	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_RESET_END);

	stop_activity(udc);

	udc->ignore_status_nak = 1;
	udc->ignore_status_ack = 1;

	udc->ud2int = (INT_SOF << 8) | (INT_NAK << 8);
	ud2_int(udc, 0xff);

	udc2_reg_read(udc, UD2ADR, &state);

	if (state & HIGH_SPEED) {
		dev_dbg(udc->dev, "%s(): high speed\n", __func__);
		udc->gadget.speed = USB_SPEED_HIGH;
		udc->ep[1].maxpacket = EP_MAX_PACKET_SIZE_HS;
		udc->ep[2].maxpacket = EP_MAX_PACKET_SIZE_HS;
	} else {
		dev_dbg(udc->dev, "%s(): full speed\n", __func__);
		udc->gadget.speed = USB_SPEED_FULL;
		udc->ep[1].maxpacket = EP_MAX_PACKET_SIZE_FS;
		udc->ep[2].maxpacket = EP_MAX_PACKET_SIZE_FS;
	}
}

union setup {
	u8 raw[8];
	struct usb_ctrlrequest r;
};

static void int_setup(struct tmpa9xx_udc *udc)
{
	struct tmpa9xx_ep *ep = &udc->ep[0];
	uint16_t request_reg;
	union setup pkt;
	int ret;

	ud2_int(udc, INT_SETUP);

	udc2_reg_read(udc, UD2BRQ, &request_reg);
	pkt.r.bRequestType = (u8) (request_reg & 0xff);
	pkt.r.bRequest = (u8) ((request_reg >> 8) & 0xff);
	udc2_reg_read(udc, UD2VAL, &pkt.r.wValue);
	udc2_reg_read(udc, UD2IDX, &pkt.r.wIndex);
	udc2_reg_read(udc, UD2LEN, &pkt.r.wLength);

#define w_index		le16_to_cpu(pkt.r.wIndex)
#define w_value		le16_to_cpu(pkt.r.wValue)
#define w_length	le16_to_cpu(pkt.r.wLength)

	if (ep->cur) {
		dev_dbg(udc->dev, "warning: setup packet handling in progress\n");
		__nuke(ep, 0);
		udc2_cmd_ep(ep, EP_FIFO_CLEAR);
		udc->ackwait = 0;
	}

	dev_dbg(udc->dev, "SETUP %02x.%02x v%04x i%04x l%04x, in %d\n", pkt.r.bRequestType, pkt.r.bRequest, w_value, w_index, w_length, pkt.r.bRequestType & USB_DIR_IN);

	udc2_reg_write(udc, UD2CMD, EP_SETUP_RECEIVED);

	/*
	 * A few standard requests get handled here, ones that touch
	 * hardware ... notably for device and endpoint features.
	 */
	if (pkt.r.bRequestType & USB_DIR_IN)
		ep->is_in = 1;
	else
		ep->is_in = 0;

	switch (pkt.r.bRequest) {
	case USB_REQ_SET_ADDRESS:
		ret = udc_set_address(udc, w_value);
		return;
	case USB_REQ_SET_CONFIGURATION:
		ret = udc_set_config(udc, w_value);
		break;
	case USB_REQ_GET_STATUS:
		ret = udc_get_status(udc, pkt.r.bRequestType & 0xf, w_index & 0xf);
		return;
	case USB_REQ_CLEAR_FEATURE:
		ret = udc_clear_feature(udc, pkt.r.bRequestType & 0xf, w_value, w_index & 0xf);
		return;
	case USB_REQ_SET_FEATURE:
		ret = udc_set_feature(udc, pkt.r.bRequestType & 0xf, w_value, w_index & 0xf);
		return;
	case USB_REQ_GET_DESCRIPTOR:
		dev_dbg(udc->dev, "%s(): USB_REQ_GET_DESCRIPTOR\n", __func__);
		break;
	case USB_REQ_SET_DESCRIPTOR:
		dev_dbg(udc->dev, "%s(): USB_REQ_SET_DESCRIPTOR\n", __func__);
		break;
	case USB_REQ_GET_CONFIGURATION:
		dev_dbg(udc->dev, "%s(): USB_REQ_GET_CONFIGURATION\n", __func__);
		break;
	case USB_REQ_GET_INTERFACE:
		dev_dbg(udc->dev, "%s(): USB_REQ_GET_INTERFACE\n", __func__);
		break;
	case USB_REQ_SET_INTERFACE:
		dev_dbg(udc->dev, "%s(): USB_REQ_SET_INTERFACE\n", __func__);
		break;
	case USB_REQ_SYNCH_FRAME:
		dev_dbg(udc->dev, "%s(): USB_REQ_SYNCH_FRAME\n", __func__);
		break;
	default:
		dev_dbg(udc->dev, "%s(): no special handling by udc\n", __func__);
		break;
	}

#undef w_value
#undef w_index
#undef w_length

	ret = udc->driver->setup(&udc->gadget, &pkt.r);
	if (ret < 0) {
		dev_dbg(udc->dev, "req %02x.%02x protocol STALL; ret %d\n", pkt.r.bRequestType, pkt.r.bRequest, ret);
		udc2_cmd_ep(ep, EP_STALL);
	}

	dev_dbg(udc->dev, "%s(): end\n", __func__);
}

static void int_status(struct tmpa9xx_udc *udc)
{
	struct tmpa9xx_ep *ep = &udc->ep[0];
	struct tmpa9xx_request *req;
	unsigned long flags;

	ud2_int(udc, INT_STATUS);

	if (udc->ignore_status_ack) {
		dev_dbg(udc->dev, "%s():\n", __func__);
		return;
	}

	if (udc->state != CONFIGURED_DONE) {
		dev_dbg(udc->dev, "%s(): state %d, addr set to 0x%04x\n", __func__, udc->state, udc->addr);
		udc2_reg_write(udc, UD2ADR, udc->state | udc->addr);
		if (udc->state == CONFIGURED)
			udc->state = CONFIGURED_DONE;
	}

	spin_lock_irqsave(&ep->s, flags);

	udc->ackwait = 0;
	udc->ignore_status_ack = 1;

	req = ep->cur;
	/* handle any pending ep0 stuff */
	if (req) {
		dev_dbg(udc->dev, "%s(): finished pending ep0 req %p\n", __func__, ep->cur);
		BUG_ON(req->req.length);
		done(ep, req, 0);
		ep->cur = NULL;
	}

	spin_unlock_irqrestore(&ep->s, flags);
}

static void int_statusnak(struct tmpa9xx_udc *udc)
{
	ud2_int(udc, INT_STATUSNAK);

	if (udc->ignore_status_nak)
		return;

	udc2_reg_write(udc, UD2CMD, EP_SETUP_FIN);

	udc->ignore_status_nak = 1;
}

static void int_ep(struct tmpa9xx_udc *udc)
{
	udc2_reg_write(udc, UD2INT_EP, 0x08);

	dev_dbg(udc->dev, "%s():\n", __func__);

	handle_ep3_snd(udc);
}

static void int_ep0(struct tmpa9xx_udc *udc)
{
	struct tmpa9xx_ep *ep = &udc->ep[0];
	struct tmpa9xx_request *req;
	int length;
	uint16_t avail;
	unsigned long flags;
	int ret;

	ud2_int(udc, INT_EP0);

	spin_lock_irqsave(&ep->s, flags);

	req = ep->cur;

	if (!req) {
		dev_dbg(udc->dev, "%s(): manual setup handling. done\n", __func__);
		udc2_reg_write(udc, UD2CMD, EP_SETUP_FIN);
		goto out;
	}

	length = req->req.length - req->req.actual;

	if (ep->is_in) {
		if (length) {
			dev_dbg(udc->dev, "%s(): continuing control msg, req %p\n", __func__, req);
			write_ep0_fifo(udc, req);
			goto out;
		}

		done(ep, req, 0);
		ep->cur = NULL;
		udc2_reg_write(udc, UD2CMD, EP_SETUP_FIN);
		dev_dbg(udc->dev, "%s(): finished control msg, req %p\n", __func__, req);

		goto out;
	}

	udc2_reg_read(udc, UD2EPx_DATASIZE(ep->num), &avail);
	dev_dbg(udc->dev, "%s(): avail %d, length %d\n", __func__, avail, length);

	ret = read_ep0_fifo(udc, req);
	if (ret) {
		done(ep, req, 0);
		ep->cur = NULL;
		udc2_reg_write(udc, UD2CMD, EP_SETUP_FIN);
		dev_dbg(udc->dev, "%s(): received control msg, req %p\n", __func__, req);
	}

out:
	spin_unlock_irqrestore(&ep->s, flags);
}

static void int_mr_end_add(struct tmpa9xx_udc *udc)
{
	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_MR_END_ADD);

	dev_dbg(udc->dev, "%s():\n", __func__);

	handle_ep1_dma_done(udc);
}

static void int_mw_set_add(struct tmpa9xx_udc *udc)
{
	struct tmpa9xx_ep *ep = &udc->ep[2];
	struct tmpa9xx_request *req;
	unsigned long flags;

	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_MW_SET_ADD);

	spin_lock_irqsave(&ep->s, flags);

	if (ep->cur) {
		dev_dbg(udc->dev, "%s(): continue buf %p\n", __func__, ep->cur);
		__handle_ep2_rcv(udc, ep->cur);
		goto out;
	}

	if (list_empty(&ep->queue)) {
		dev_dbg(udc->dev, "%s(): no buf available (yet)\n", __func__);
		goto out;
	}

	req = list_entry(ep->queue.next, struct tmpa9xx_request, queue);
	list_del_init(&req->queue);
	ep->cur = req;
	prepare_req_for_dma(udc, ep, req);
	dev_dbg(udc->dev, "%s(): receiving to buf %p\n", __func__, ep->cur);

	__handle_ep2_rcv(udc, ep->cur);

out:
	spin_unlock_irqrestore(&ep->s, flags);
}

static void int_mw_end_add(struct tmpa9xx_udc *udc)
{
	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, INT_MW_END_ADD);

	dev_dbg(udc->dev, "%s():\n", __func__);

	handle_ep2_dma_done(udc);
}

static void backend_irq_work(struct work_struct *work)
{
	struct tmpa9xx_udc *udc = container_of(work, struct tmpa9xx_udc, ws);
	uint32_t intsts;
	uint32_t intenb;
	uint32_t status;

again:
	intsts = tmpa9xx_ud2ab_read(udc, UD2AB_INTSTS);
	intenb = tmpa9xx_ud2ab_read(udc, UD2AB_INTENB);

	status = intsts & (intenb | ((~(udc->ud2int >> 8)) & 0xff));
	if (!status)
		goto out;

	dev_dbg(udc->dev, "%s(): raw 0x%08x, en 0x%08x, masked 0x%08x\n", __func__, intsts, intenb, status);

	if ((status & INT_SUSPEND))
		int_suspend(udc);
	else if ((status & INT_RESET))
		int_reset(udc);
	else if ((status & INT_RESET_END)) {
		int_reset_end(udc);
		/* make sure that no pending interrupts are handled */
		goto again;
	} else if ((status & INT_MR_END_ADD))
		int_mr_end_add(udc);
	else if ((status & INT_MW_SET_ADD))
		int_mw_set_add(udc);
	else if ((status & INT_MW_END_ADD))
		int_mw_end_add(udc);
	else if ((status & INT_EP))
		int_ep(udc);
	else if ((status & INT_EP0))
		int_ep0(udc);
	/* must be last */
	else if ((status & INT_STATUS))
		int_status(udc);
	else if ((status & INT_STATUSNAK))
		int_statusnak(udc);
	else if ((status & INT_SETUP))
		int_setup(udc);
	else
		dev_err(udc->dev, "%s(): unhandled irq, status 0x%04x\n", __func__, status);

	goto again;

out:
	enable_irq(udc->irq);
}
#if 0
static int usb_gadget_probe_driver(struct usb_gadget_driver *driver, int (*bind) (struct usb_gadget *))
{
	struct tmpa9xx_udc *udc = &controller;
	int ret;

	dev_dbg(udc->dev, "%s():\n", __func__);

	if (!driver || driver->speed < USB_SPEED_FULL || !bind || !driver->setup) {
		dev_dbg(udc->dev, "%s(): invalid arguments\n", __func__);
		return -EINVAL;
	}

	if (udc->driver) {
		dev_dbg(udc->dev, "%s(): already in use\n", __func__);
		return -EINVAL;
	}

	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	dev_set_drvdata(&udc->gadget.dev, &driver->driver);

	ret = bind(&udc->gadget);
	if (ret) {
		dev_dbg(udc->dev, "%s(): bind() failed, ret %d\n", __func__, ret);
		udc->driver = NULL;
		udc->gadget.dev.driver = NULL;
		dev_set_drvdata(&udc->gadget.dev, NULL);
		return ret;
	}

	udc2_reg_write(udc, UD2CMD, EP_USB_READY);
	enable_irq(udc->irq);

	dev_dbg(udc->dev, "%s(): bound to %s\n", __func__, driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_probe_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct tmpa9xx_udc *udc = &controller;

	BUG_ON(!udc->driver);

	if (!driver || driver != udc->driver || !driver->unbind) {
		dev_dbg(udc->dev, "%s(): invalid arguments\n", __func__);
		return -EINVAL;
	}

	disable_irq(udc->irq);
	cancel_work_sync(&udc->ws);
	flush_workqueue(udc->wq);
	stop_activity(udc);

	driver->disconnect(&udc->gadget);
	driver->unbind(&udc->gadget);

	udc->driver = NULL;
	udc->gadget.dev.driver = NULL;
	dev_set_drvdata(&udc->gadget.dev, NULL);

	dev_dbg(udc->dev, "%s(): unbound from %s\n", __func__, driver->driver.name);

	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);
#endif
static void pwctl_power_reset(struct tmpa9xx_udc *udc)
{
	u32 reg_data;

	reg_data = tmpa9xx_ud2ab_read(udc, UD2AB_PWCTL);

	reg_data &= ~PWCTL_POWER_RESET;
	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);

	reg_data |= PWCTL_POWER_RESET;
	tmpa9xx_ud2ab_write(udc, UD2AB_PWCTL, reg_data);
}

static int __devinit tmpa9xx_udc_probe(struct platform_device *pdev)
{
	struct tmpa9xx_udc *udc = &controller;
	struct resource *res;
	int i;
	int ret;

	/* sanity check */
	BUG_ON(pdev->num_resources != 2);

	udc->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(udc->dev, "%s(): platform_get_resource() @ IORESOURCE_MEM failed\n", __func__);
		ret = -ENXIO;
		goto err0;
	}

	if (!request_mem_region(res->start, res->end - res->start + 1, driver_name)) {
		dev_err(udc->dev, "%s(): request_mem_region() failed\n", __func__);
		ret = -EBUSY;
		goto err0;
	}

	udc->base = ioremap(res->start, res->end - res->start + 1);
	if (!udc->base) {
		dev_err(udc->dev, "%s(): ioremap() failed\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

	udc->clk = clk_get(udc->dev, NULL);
	if (IS_ERR(udc->clk)) {
		dev_err(udc->dev, "%s(): clk_get() failed\n", __func__);
		ret = -ENOENT;
		goto err2;
	}

	ret = clk_enable(udc->clk);
	if (ret) {
		dev_err(udc->dev, "%s(): clk_enable() failed\n", __func__);
		ret = -ENOENT;
		goto err3;
	}

	pwctl_power_reset(udc);

	udc->irq = platform_get_irq(pdev, 0);
	if (request_irq(udc->irq, tmpa9xx_udc_irq, IRQF_DISABLED, driver_name, udc)) {
		dev_err(udc->dev, "%s(): request_irq() failed\n", __func__);
		ret = -EBUSY;
		goto err4;
	}
	disable_irq(udc->irq);

	udc->wq = create_workqueue("tmpa9xx-udc-wq");
	if (!udc->wq) {
		dev_err(udc->dev, "%s(): create_workqueue() failed\n", __func__);
		ret = -EBUSY;
		goto err5;
	}

	/* init software state */
	udc->gadget.dev.parent = udc->dev;
	dev_set_name(&udc->gadget.dev, "gadget");

	INIT_WORK(&udc->ws, backend_irq_work);

	/* this spinlock protects the access to the UDC registers */
	spin_lock_init(&udc->reg_slock);

	/* ep0 init handling */
	spin_lock_init(&udc->ep[0].s);
	INIT_LIST_HEAD(&udc->ep[0].queue);
	udc->ep[0].ep.maxpacket = udc->ep[0].maxpacket;

	/* other ep init handling */
	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);
	for (i = 1; i < NUM_ENDPOINTS; i++) {
		struct tmpa9xx_ep *ep = &udc->ep[i];
		INIT_LIST_HEAD(&ep->queue);
		list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
	}

	udc->ud2int = (INT_SOF << 8) | (INT_NAK << 8);
	ud2_int(udc, 0xff);

	/* clear all interrupts */
	tmpa9xx_ud2ab_write(udc, UD2AB_INTSTS, 0x33fe07ff);

	/* enable our interrupts */
	tmpa9xx_ud2ab_write(udc, UD2AB_INTENB,
		INT_SUSPEND | INT_RESET | INT_RESET_END |
		INT_MR_END_ADD | INT_MW_END_ADD | INT_MW_SET_ADD);

	/* reset dma engine */
	tmpa9xx_ud2ab_write(udc, UD2AB_UDMSTSET, UDC2AB_MR_RESET | UDC2AB_MW_RESET);

	udc->w_len = 0;
	udc->r_len = 0;

	stop_activity(udc);

	ret = device_register(&udc->gadget.dev);
	if (ret < 0) {
		dev_err(udc->dev, "%s(): device_register() failed\n", __func__);
		ret = -EBUSY;
		goto err6;
	}

	dev_set_drvdata(&pdev->dev, udc);

	dev_info(udc->dev, "%s ready\n", driver_name);

	return 0;

err6:
	destroy_workqueue(udc->wq);
err5:
	free_irq(udc->irq, udc);
err4:
	clk_disable(udc->clk);
err3:
	clk_put(udc->clk);
err2:
	iounmap(udc->base);
err1:
	release_mem_region(res->start, res->end - res->start + 1);
err0:
	return ret;
}

static int __devexit tmpa9xx_udc_remove(struct platform_device *pdev)
{
	struct tmpa9xx_udc *udc = platform_get_drvdata(pdev);
	struct resource *res;

	BUG_ON(udc->driver);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	BUG_ON(!res);

	disable_irq(udc->irq);

	cancel_work_sync(&udc->ws);

	flush_workqueue(udc->wq);

	device_unregister(&udc->gadget.dev);

	destroy_workqueue(udc->wq);

	free_irq(udc->irq, udc);

	clk_disable(udc->clk);

	clk_put(udc->clk);

	iounmap(udc->base);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int tmpa9xx_udc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int tmpa9xx_udc_resume(struct platform_device *pdev)
{
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
