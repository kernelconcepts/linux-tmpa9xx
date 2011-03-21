#ifndef __TMPA9XX_I2S_H__
#define __TMPA9XX_I2S_H__

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <asm/dma.h>

/* I2S Interface   */
#define I2S_BASE                  (0xF2040000)
#define I2STCON                   __REG(I2S_BASE + 0x000)
#define I2STSLVON                 __REG(I2S_BASE + 0x004)
#define I2STFCLR                  __REG(I2S_BASE + 0x008)
#define I2STMS                    __REG(I2S_BASE + 0x00C)
#define I2STMCON                  __REG(I2S_BASE + 0x010)
#define I2STMSTP                  __REG(I2S_BASE + 0x014)
#define I2STDMA1                  __REG(I2S_BASE + 0x018)
#define I2SRCON                   __REG(I2S_BASE + 0x020)
#define I2SRSLVON                 __REG(I2S_BASE + 0x024)
#define I2SFRFCLR                 __REG(I2S_BASE + 0x028)
#define I2SRMS                    __REG(I2S_BASE + 0x02C)
#define I2SRMCON                  __REG(I2S_BASE + 0x030)
#define I2SRMSTP                  __REG(I2S_BASE + 0x034)
#define I2SRDMA1                  __REG(I2S_BASE + 0x038)
#define I2SCOMMON                 __REG(I2S_BASE + 0x044) 
#define I2STST                    __REG(I2S_BASE + 0x048)
#define I2SRST                    __REG(I2S_BASE + 0x04C)
#define I2SINT                    __REG(I2S_BASE + 0x050)
#define I2SINTMSK                 __REG(I2S_BASE + 0x054)
#define I2STDAT                   __REG(I2S_BASE + 0x1000)
#define I2SRDAT                   __REG(I2S_BASE + 0x2000)
#define I2STDAT_ADR              (I2S_BASE + 0x1000)
#define I2SRDAT_ADR              (I2S_BASE + 0x2000)

#define DESC_ELEMENT_COUNT  9

struct scatter_dma_t {
	unsigned long srcaddr;
	unsigned long dstaddr;
	unsigned long lli;
	unsigned long control;
};

struct tmpa9xx_i2s {
	int i2s_num;
	int err_irq;
	
	int dma_tx_ch;
	int dma_rx_ch;

	/* DMA descriptor ring head of current audio stream*/
	struct scatter_dma_t *dma_tx_desc;
	unsigned int tx_desc_bytes;
	unsigned int dma_tx_phydesc;
	unsigned int dma_tx_buf;
	unsigned int tx_run; /* tx is running */
	struct scatter_dma_t *curr_tx_desc;

	struct scatter_dma_t *dma_rx_desc;
	unsigned int rx_desc_bytes;
	unsigned int dma_rx_phydesc;
	unsigned int dma_rx_buf;
	unsigned int rx_run; /* tx is running */
	struct scatter_dma_t *curr_rx_desc;

	void (*rx_callback)(void *data);
	void (*tx_callback)(void *data);
	void (*err_callback)(void *data);
	void *data;
};

struct tmpa9xx_i2s* tmpa9xx_i2s_init(
		int dma_rx, void (*rx_callback)(void *),
		int dma_tx, void (*tx_callback)(void *),
		int err_irq, void (*err_callback)(void *),
		void *data);

void tmpa9xx_i2s_free(struct tmpa9xx_i2s* i2s);

/* buffer size (in bytes) == fragcount * fragsize_bytes */

/* this is not a very general api, it sets the dma to 2d autobuffer mode */

int tmpa9xx_i2s_config_tx_dma(struct tmpa9xx_i2s *i2s,
        unsigned char *cpu_buf, unsigned int phy_buf,
		int fragcount, size_t fragsize, size_t size);

int tmpa9xx_i2s_config_rx_dma(struct tmpa9xx_i2s *i2s, 
        unsigned char *cpu_buf, unsigned int phy_buf,
		int fragcount, size_t fragsize, size_t size);

int tmpa9xx_i2s_tx_start(struct tmpa9xx_i2s* i2s);
int tmpa9xx_i2s_tx_stop(struct tmpa9xx_i2s* i2s);
int tmpa9xx_i2s_rx_start(struct tmpa9xx_i2s* i2s);
int tmpa9xx_i2s_rx_stop(struct tmpa9xx_i2s* i2s);

/* for use in interrupt handler */
unsigned int tmpa9xx_i2s_curr_offset_rx(struct tmpa9xx_i2s* i2s);
unsigned int tmpa9xx_i2s_curr_offset_tx(struct tmpa9xx_i2s* i2s);

#endif /* TMPA9XX_I2S_H */
