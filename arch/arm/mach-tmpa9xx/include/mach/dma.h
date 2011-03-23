/*
 *  linux/include/asm-arm/arch-tmpa9xx/dma.h
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_ARCH_TMPA9XX_DMA_H
#define __ASM_ARCH_TMPA9XX_DMA_H

#define	TMPA9XX_DMA_CHANNELS	8

enum {
	DMA_UART0_TX = 0,
	DMA_UART0_RX = 1,
	DMA_NANDC0 = 4,
	DMA_CMSI = 5,
	SDHC_WRITE = 8,
	SDHC_READ = 9,
	I2S0 = 10,
	I2S1 = 11,
	LCDDA = 14,
};

/*
 * Descriptor structure for TMPA9xx DMA engine
 */
typedef struct tmpa9xx_dma_desc {
	u32 src_addr;  /* DMA Channel source address */
	u32 dest_addr; /* DMA Channel dest address */
	u32 dma_lli;   /* DMA linked list item */
	u32 control;   /* DMA channel control */
	u32 config;    /* DMA channel configuration */
} tmpa9xx_dma_desc;

typedef enum {
	DMA_PRIO_HIGH = 1,
	DMA_PRIO_MEDIUM = 4,
	DMA_PRIO_LOW = 8, 
} tmpa9xx_dma_prio;

struct tmpa9xx_dma_channel {
	const char *name;
	void (*irq_handler)(int, void *);
	void (*err_handler)(int, void *);
	void *data;
	int dma_num;
};

void tmpa9xx_dma_enable(int dma_ch);
void tmpa9xx_dma_disable(int dma_ch);
long tmpa9xx_dma_get_size(int dma_ch);

/*
 * DMA registration
 */

int tmpa9xx_dma_request (const char *name,
			 int prio,
			 void (*irq_handler)(int, void *),
			 void (*err_handler)(int, void *),
			 void *data);

void tmpa9xx_dma_free (int dma_ch);

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

#define DMAC_CH_CONF_HALT      (1<<18)   // Halt           R/W 0y0       0y0: Enable DMA requests
#define DMAC_CH_CONF_ACTIVE    (1<<17)   // Active         RO  0y0       0y0: No data in the FIFO
#define DMAC_CH_CONF_LOCK      (1<<16)   // Lock           R/W 0y0       0y0: Disable locked transfers
#define DMAC_CH_CONF_ITC       (1<<15)   // ITC            R/W 0y0       Terminal count interrupt enable register
#define DMAC_CH_CONF_IE        (1<<14)   // IE             R/W 0y0       Error interrupt enable register

#define DMAC_CH_CONF_FLOWCNTRL_MEM2MEM (0<<11) // 0y000              Memory to Memory
#define DMAC_CH_CONF_FLOWCNTRL_MEM2PER (1<<11) // 0y001              Memory to Peripheral
#define DMAC_CH_CONF_FLOWCNTRL_PER2MEM (2<<11) // 0y010              Peripheral to Memory
#define DMAC_CH_CONF_FLOWCNTRL_PER2PER (3<<11) // 0y011              Peripheral to Peripheral

#define DMAC_CH_CONF_DEST_OFS (6)	//
#define DMAC_CH_CONF_SRC_OFS  (1)	//

#define DMAC_CH_CONF_E        (1<<0)	//   E              R/W 0y0       Channel enable

#define DMAC_CH_CTRL_I               (1<<31) // Terminal count interrupt enable register when u
#define DMAC_CH_CTRL_PROT3           (1<<30) // Control cache permission HPROT[3]
#define DMAC_CH_CTRL_PROT2           (1<<29) // Control buffer permission HPROT[2]
#define DMAC_CH_CTRL_PROT1           (1<<28) // Control privileged mode HPROT[1]
#define DMAC_CH_CTRL_DI              (1<<27) // Increment the transfer destination address
#define DMAC_CH_CTRL_SI              (1<<26) // Increment the transfer source address
#define DMAC_CH_CTRL_D               (1<<25) // Transfer destination AHB Master. 0 DMA1, 1 DMA2
#define DMAC_CH_CTRL_S               (1<<24) //  Transfer source AHB Master

#define DMAC_CH_CTRL_DWIDTH_BYTE     (0<<21) // 0y000: Byte (8 bits)
#define DMAC_CH_CTRL_DWIDTH_HWORD    (1<<21) // 0y001: Half-word (16 bits)
#define DMAC_CH_CTRL_DWIDTH_WORD     (2<<21) // 0y010: Word (32 bits)

#define DMAC_CH_CTRL_SWIDTH_BYTE     (0<<18) // 0y000: Byte (8 bits)
#define DMAC_CH_CTRL_SWIDTH_HWORD    (1<<18) // 0y001: Half-word (16 bits)
#define DMAC_CH_CTRL_SWIDTH_WORD     (2<<18) // 0y010: Word (32 bits)

                                         // [17:15] DBSize[2:0]  R/W 0y000
#define DMAC_CH_CTRL_DBSIZE_1B       (0<<15) // 0y000: 1 beat
#define DMAC_CH_CTRL_DBSIZE_4B       (1<<15) // 0y001: 4 beats
#define DMAC_CH_CTRL_DBSIZE_8B       (2<<15) // 0y010 : 8 beats
#define DMAC_CH_CTRL_DBSIZE_16B      (3<<15) // 0y011: 16 beats
#define DMAC_CH_CTRL_DBSIZE_32B      (4<<15) // 0y100: 32 beats
#define DMAC_CH_CTRL_DBSIZE_64B      (5<<15) // 0y101: 64 beats
#define DMAC_CH_CTRL_DBSIZE_128B     (6<<15) // 0y110: 128 beats
#define DMAC_CH_CTRL_DBSIZE_256B     (7<<15) // 0y111: 256 beats

                                          //  [14:12] SBSize[2:0]  R/W 0y000 Transfer source burst size:
#define DMAC_CH_CTRL_SBSIZE_1B       (0<<12) // 0y000: 1 beat
#define DMAC_CH_CTRL_SBSIZE_4B       (1<<12) // 0y001: 4 beats
#define DMAC_CH_CTRL_SBSIZE_8B       (2<<12) // 0y010: 8 beats
#define DMAC_CH_CTRL_SBSIZE_16B      (3<<12) // 0y011: 16 beats
#define DMAC_CH_CTRL_SBSIZE_32B      (4<<12) // 0y100: 32 beats
#define DMAC_CH_CTRL_SBSIZE_64B      (5<<12) // 0y101: 64 beats
#define DMAC_CH_CTRL_SBSIZE_128B     (6<<12) // 0y110: 128 beats
#define DMAC_CH_CTRL_SBSIZE_256B     (7<<12) // 0y111: 256 beats

#define DMAC_CH_CTRL_TRANSFERSIZE_MASK (0xfff) // [11:0]  TransferSize R/W 0x000 Set the total transfer count

#endif /* _ASM_ARCH_DMA_H */
