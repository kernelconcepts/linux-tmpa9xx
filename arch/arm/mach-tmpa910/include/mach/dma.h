/*
 *  linux/include/asm-arm/arch-tmpa910/dma.h
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_ARCH_TMPA910_DMA_H
#define __ASM_ARCH_TMPA910_DMA_H

#define	TMPA910_DMA_CHANNELS	8

enum {
	DMA_UART0_TX = 0,
	DMA_UART0_RX = 1,
	DMA_NANDC0 = 4,
	DMA_CMSI = 5,
	I2S0 = 10,
	I2S1 = 11,
	LCDDA = 14,
};

/*
 * Descriptor structure for TMPA910 DMA engine
 */
typedef struct tmpa910_dma_desc {
	volatile u32 src_addr;	/* DMA Channel source address */
	volatile u32 dest_addr;	/* DMA Channel dest address */
	volatile u32 dma_lli;	/* DMA linked list item */
	volatile u32 control;	/* DMA channel control */
	volatile u32 config;	/* DMA channel configuration */
} tmpa910_dma_desc;

typedef enum {
	DMA_PRIO_HIGH = 1,
	DMA_PRIO_MEDIUM = 4,
	DMA_PRIO_LOW = 8, 
} tmpa910_dma_prio;

struct tmpa910_dma_channel {
	const char *name;
	void (*irq_handler)(int, void *);
	void (*err_handler)(int, void *);
	void *data;
	int dma_num;
};

void tmpa910_dma_enable(int dma_ch);
void tmpa910_dma_disable(int dma_ch);
long tmpa910_dma_get_size(int dma_ch);


/*
 * DMA registration
 */

int tmpa910_dma_request (const char *name,
			 int prio,
			 void (*irq_handler)(int, void *),
			 void (*err_handler)(int, void *),
			 void *data);

void tmpa910_dma_free (int dma_ch);

#endif /* _ASM_ARCH_DMA_H */

