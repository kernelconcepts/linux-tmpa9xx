/*
 *  linux/arch/arm/mach-tmpa/dma.c
 *
 *  tmpa910 DMA registration and IRQ dispatching
 *  based on arch/arm/mach-imx/dma.c
 *  Copyright (C) Yin, Fengwei (fengwei.yin@gmail.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#undef DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/errno.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <mach/tmpa910_regs.h>
#include <mach/dma.h>

struct tmpa910_dma_channel tmpa910_dma_channels[TMPA910_DMA_CHANNELS];

/**
 * tmpa910_dma_enable - function to start TMPA910 DMA channel operation
 */
void tmpa910_dma_enable(int dma_ch)
{
	struct tmpa910_dma_channel *dma = &tmpa910_dma_channels[dma_ch];
	unsigned long flags;

	pr_debug("tmpa910 dma%d: tmpa910_dma_enable\n", dma_ch);

	if (!dma->name) {
		printk(KERN_CRIT "%s: called for  not allocated channel %d\n",
		       __FUNCTION__, dma_ch);
		return;
	}

	local_irq_save(flags);
	DMA_CONFIG(dma_ch) = DMA_CONFIG(dma_ch) | DMA_CONFIG_EN;
	local_irq_restore(flags);
}

/**
 * tmpa910_dma_disable - disable TMPA910 DMA channel operatin
 */
void tmpa910_dma_disable(int dma_ch)
{
	unsigned long flags;

	local_irq_save(flags);
	DMA_CONFIG(dma_ch) = DMA_CONFIG(dma_ch) & ~DMA_CONFIG_EN;
	local_irq_restore(flags);
}

long tmpa910_dma_get_size(int dma_ch)
{
	long size = DMA_CONTROL(dma_ch) & 0x0FFF;
	long  width;
	width = (size >> 18) & 0x0007;
	switch(width)
	{
	case 0:
		size *= 1;
		break;
	case 1:
		size *= 2;
		break;
	case 2:
		size *= 4;
		break;
	}
	return size;
}

/**
 * tmpa910_dma_request - request/allocate specified channel number
 */
int tmpa910_dma_request(const char *name,
		int prio,
		void (*irq_handler)(int, void *),
		void (*err_handler)(int, void *),
		void *data)
{
	unsigned long flags;
	int i, found = 0;

	/* basic sanity checks */
	if (!name)
		return -EINVAL;

	local_irq_save(flags);

	for (i = prio; i > 0; i--) {
		if (tmpa910_dma_channels[i].name == NULL) {
			found = 1;
			break;
		}
	}

	if (!found) {
		for (i = prio; i < TMPA910_DMA_CHANNELS; i ++) {
			if (tmpa910_dma_channels[i].name == NULL) {
				found = 1;
				break;
			}
		}
	}
	
	if (!found) {
		local_irq_restore(flags);
		return -ENODEV;
	}

	if (tmpa910_dma_channels[i].name != NULL) {
		local_irq_restore(flags);
		return -ENODEV;
	}
	tmpa910_dma_channels[i].name = name;
	tmpa910_dma_channels[i].err_handler = err_handler;
	tmpa910_dma_channels[i].irq_handler = irq_handler;
	tmpa910_dma_channels[i].data = data;

	local_irq_restore(flags);
	return i;
}

/**
 * tmpa910_dma_free - release previously acquired channel
 */
void tmpa910_dma_free(int dma_ch)
{
	unsigned long flags;
	struct tmpa910_dma_channel *dma = &tmpa910_dma_channels[dma_ch];

	if (!dma->name) {
		printk(KERN_CRIT
		       "%s: trying to free channel %d which is already freed\n",
		       __FUNCTION__, dma_ch);
		return;
	}

	local_irq_save(flags);
	/* Disable interrupts */
	DMA_CONFIG(dma_ch) &= ~DMA_CONFIG_EN;
	dma->name = NULL;
	dma->err_handler = NULL;
	dma->irq_handler = NULL;
	local_irq_restore(flags);
}

static irqreturn_t dma_err_handler(int irq, void *dev_id)
{
	struct tmpa910_dma_channel *channel;
	unsigned int err_status = DMA_ERR_STATUS;
	int i;

	DMA_ERR_CLEAR = err_status;

	for (i = 0; i < TMPA910_DMA_CHANNELS; i++) {
		if(!(err_status & (1 << i)))
			continue;

		channel = &tmpa910_dma_channels[i];
		if (channel->name && channel->err_handler) {
			channel->err_handler(i, channel->data);
			continue;
		} else {
			printk(KERN_WARNING "spurous DMA IRQ: %d\n", i);
			tmpa910_dma_disable(i);
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t dma_irq_handler(int irq, void *dev_id)
{
	struct tmpa910_dma_channel *channel;
	unsigned int tc_status = DMA_TC_STATUS;
	int i;

	DMA_TC_CLEAR = tc_status;

	for (i = 0; i < TMPA910_DMA_CHANNELS; i++) {
		if(!(tc_status & (1 << i)))
			continue;

		channel = &tmpa910_dma_channels[i];
		if (channel->name && channel->irq_handler) {
			channel->irq_handler(i, channel->data);
			continue;
		} else {
			printk(KERN_WARNING "spurous DMA IRQ: %d\n", i);
			tmpa910_dma_disable(i);
		}
	}
	return IRQ_HANDLED;
}

static int __init tmpa910_dma_init(void)
{
	int ret;
	int i;

	/* Initialize DMA module */
	DMA_CONFIGURE = 0x0001;	/* DMA1/2: little endian, Active DMA */
	DMA_ERR_CLEAR = 0xff;	/* Clear DMA error interrupt */
	DMA_TC_CLEAR = 0xff;	/* Clear DMA TC interrupt */


	ret = request_irq(DMA_END_INT, dma_irq_handler, 0, "DMA", NULL);
	if (ret) {
		printk(KERN_CRIT "Wow!  Can't register IRQ for DMA\n");
		return ret;
	}

	ret = request_irq(DMA_ERR_INT, dma_err_handler, 0, "DMA", NULL);
	if (ret) {
		printk(KERN_CRIT "Wow!  Can't register ERRIRQ for DMA\n");
		free_irq(DMA_END_INT, NULL);
		return ret;
	}

	for (i = 0; i < TMPA910_DMA_CHANNELS; i++) {
		tmpa910_dma_channels[i].name = NULL;
		tmpa910_dma_channels[i].dma_num = i;
	}

	return ret;
}

arch_initcall(tmpa910_dma_init);

EXPORT_SYMBOL(tmpa910_dma_enable);
EXPORT_SYMBOL(tmpa910_dma_disable);
EXPORT_SYMBOL(tmpa910_dma_get_size);
EXPORT_SYMBOL(tmpa910_dma_request);
EXPORT_SYMBOL(tmpa910_dma_free);
