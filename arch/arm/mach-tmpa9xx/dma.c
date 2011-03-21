/*
 * TMPA9xx DMA driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/errno.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <mach/regs.h>
#include <mach/dma.h>

#define DMAC_PERIPHERAL_UART0TX        0  // UART0 transmit
#define DMAC_PERIPHERAL_UART0RX        1  // UART0 receive
#define DMAC_PERIPHERAL_SSP0_TX	       2  // SSP0 transmit
#define DMAC_PERIPHERAL_SSP0_RX	       3  // SSP0 receive
#define DMAC_PERIPHERAL_NANDC0         4  // NANDC0
#define DMAC_PERIPHERAL_CMSI           5  // CMSI
#define DMAC_PERIPHERAL_SDHC_SD_WRITE  8  // SDHC SD buffer write request
#define DMAC_PERIPHERAL_SDHC_SD_READ   9  // SDHC SD buffer read request
#define DMAC_PERIPHERAL_I2S0           10 // I2S0   10
#define DMAC_PERIPHERAL_I2S1           11 // I2S1   11 
#define DMAC_PERIPHERAL_SSP1_TX	       12 // SSP1 transmit
#define DMAC_PERIPHERAL_SSP1_RX	       13 // SSP1 receive
#define DMAC_PERIPHERAL_LCDDA14 LCDDA  14 //

#define DMAC_CH_CTRL_TRANSFERSIZE_MASK (0xfff) // [11:0]  TransferSize R/W 0x000 Set the total transfer count

#define	TMPA9XX_DMA_CHANNELS	8

struct tmpa9xx_dma_channel {
	void (*irq_handler)(int, void *);
	void (*err_handler)(int, void *);
	void *data;
	bool in_use;
};

struct tmpa9xx_dma_channel tmpa9xx_dma_channels[TMPA9XX_DMA_CHANNELS];

void tmpa9xx_dma_enable(int dma_ch)
{
	struct tmpa9xx_dma_channel *dma = &tmpa9xx_dma_channels[dma_ch];

	pr_debug("%s(): channel %d\n", __func__, dma_ch);

	BUG_ON(!dma->in_use);

	DMA_CONFIG(dma_ch) = DMA_CONFIG(dma_ch) | DMA_CONFIG_EN;
}

void tmpa9xx_dma_disable(int dma_ch)
{
	struct tmpa9xx_dma_channel *dma = &tmpa9xx_dma_channels[dma_ch];

	pr_debug("%s(): channel %d\n", __func__, dma_ch);

	BUG_ON(!dma->in_use);

	DMA_CONFIG(dma_ch) = DMA_CONFIG(dma_ch) & ~DMA_CONFIG_EN;
}

int tmpa9xx_dma_request(void (*irq_handler)(int, void *),
			void (*err_handler)(int, void *),
			void *data)
{
	struct tmpa9xx_dma_channel *dma;
	int i;

	for (i = 0; i < TMPA9XX_DMA_CHANNELS; i++) {
		dma = &tmpa9xx_dma_channels[i];
		if (!dma->in_use)
			break;
	}

	if (i == TMPA9XX_DMA_CHANNELS) {
		pr_err("%s(): no more free dma channels\n", __func__);
		return -EINVAL;
	}

	dma->in_use = true;
	dma->err_handler = err_handler;
	dma->irq_handler = irq_handler;
	dma->data = data;

	pr_debug("%s(): channel %d\n", __func__, i);

	return i;
}

void tmpa9xx_dma_free(int dma_ch)
{
	struct tmpa9xx_dma_channel *dma = &tmpa9xx_dma_channels[dma_ch];

	pr_debug("%s(): channel %d\n", __func__, dma_ch);

	BUG_ON(!dma->in_use);

	DMA_CONFIG(dma_ch) = DMA_CONFIG(dma_ch) & ~DMA_CONFIG_EN;

	dma->in_use = false;
	dma->err_handler = NULL;
	dma->irq_handler = NULL;
}

static irqreturn_t dma_err_handler(int irq, void *dev_id)
{
	struct tmpa9xx_dma_channel *dma;
	uint32_t err_status;
	int i;

	err_status = DMA_ERR_STATUS;
	DMA_ERR_CLEAR = err_status;

	for (i = 0; i < TMPA9XX_DMA_CHANNELS; i++) {
		if(!(err_status & (1 << i)))
			continue;

		dma = &tmpa9xx_dma_channels[i];

		if (!dma->in_use)
			continue;

		BUG_ON(!dma->err_handler);

		dma->err_handler(i, dma->data);
	}

	return IRQ_HANDLED;
}

static irqreturn_t dma_irq_handler(int irq, void *dev_id)
{
	struct tmpa9xx_dma_channel *dma;
	uint32_t tc_status;
	int i;

	tc_status = DMA_TC_STATUS;
	DMA_TC_CLEAR = tc_status;

	for (i = 0; i < TMPA9XX_DMA_CHANNELS; i++) {
		if (!(tc_status & (1 << i)))
			continue;

		dma = &tmpa9xx_dma_channels[i];
		if (!dma->in_use)
			continue;

		BUG_ON(!dma->irq_handler);

		dma->irq_handler(i, dma->data);
	}

	return IRQ_HANDLED;
}

static int __init tmpa9xx_dma_init(void)
{
	int ret;

	/* Initialize DMA module */
	DMA_CONFIGURE = 0x0001;	/* DMA1/2: little endian, Active DMA */
	DMA_ERR_CLEAR = 0xff;	/* Clear DMA error interrupt */
	DMA_TC_CLEAR = 0xff;	/* Clear DMA TC interrupt */

	ret = request_irq(DMA_END_INT, dma_irq_handler, 0, "DMA", NULL);
	if (ret) {
		pr_err("%s(): request_irq() @ DMA_END_INT failed\n", __func__);
		return ret;
	}

	ret = request_irq(DMA_ERR_INT, dma_err_handler, 0, "DMA", NULL);
	if (ret) {
		pr_err("%s(): request_irq() @ DMA_ERR_INT failed\n", __func__);
		free_irq(DMA_END_INT, NULL);
		return ret;
	}

	pr_debug("%s():\n", __func__);

	return 0;
}

arch_initcall(tmpa9xx_dma_init);

EXPORT_SYMBOL(tmpa9xx_dma_enable);
EXPORT_SYMBOL(tmpa9xx_dma_disable);
EXPORT_SYMBOL(tmpa9xx_dma_request);
EXPORT_SYMBOL(tmpa9xx_dma_free);
