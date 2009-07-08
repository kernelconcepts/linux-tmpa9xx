#ifndef __TMPA910_I2S_H__
#define __TMPA910_I2S_H__

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <asm/dma.h>

#define DESC_ELEMENT_COUNT  9

struct scatter_dma_t {
	unsigned long srcaddr;
	unsigned long dstaddr;
	unsigned long lli;
	unsigned long control;
};

struct tmpa910_i2s {
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

struct tmpa910_i2s* tmpa910_i2s_init(
		int dma_rx, void (*rx_callback)(void *),
		int dma_tx, void (*tx_callback)(void *),
		int err_irq, void (*err_callback)(void *),
		void *data);

void tmpa910_i2s_free(struct tmpa910_i2s* i2s);

/* first use these ...*/

int tmpa910_i2s_config_rx(struct tmpa910_i2s* i2s);

int tmpa910_i2s_config_tx(struct tmpa910_i2s* i2s);

/* ... then these: */

/* buffer size (in bytes) == fragcount * fragsize_bytes */

/* this is not a very general api, it sets the dma to 2d autobuffer mode */

int tmpa910_i2s_config_tx_dma(struct tmpa910_i2s *i2s,
        unsigned char *cpu_buf, unsigned int phy_buf,
		int fragcount, size_t fragsize, size_t size);

int tmpa910_i2s_config_rx_dma(struct tmpa910_i2s *i2s, 
        unsigned char *cpu_buf, unsigned int phy_buf,
		int fragcount, size_t fragsize, size_t size);

int tmpa910_i2s_tx_start(struct tmpa910_i2s* i2s);
int tmpa910_i2s_tx_stop(struct tmpa910_i2s* i2s);
int tmpa910_i2s_rx_start(struct tmpa910_i2s* i2s);
int tmpa910_i2s_rx_stop(struct tmpa910_i2s* i2s);

/* for use in interrupt handler */
unsigned int tmpa910_i2s_curr_offset_rx(struct tmpa910_i2s* i2s);
unsigned int tmpa910_i2s_curr_offset_tx(struct tmpa910_i2s* i2s);

#endif /* TMPA910_I2S_H */
