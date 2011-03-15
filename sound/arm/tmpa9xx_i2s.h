#ifndef __TMPA9XX_I2S_H__
#define __TMPA9XX_I2S_H__

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <asm/dma.h>

int tmpa9xx_i2s_init(
		void (*rx_callback)(void *),
		void (*tx_callback)(void *),
		void *data);

void tmpa9xx_i2s_free(void);

/* buffer size (in bytes) == fragcount * fragsize_bytes */

/* this is not a very general api, it sets the dma to 2d autobuffer mode */

int tmpa9xx_i2s_config_tx_dma(
        unsigned char *cpu_buf, unsigned int phy_buf,
		int fragcount, size_t fragsize, size_t size);

int tmpa9xx_i2s_config_rx_dma(
        unsigned char *cpu_buf, unsigned int phy_buf,
		int fragcount, size_t fragsize, size_t size);

int tmpa9xx_i2s_tx_start(void);
int tmpa9xx_i2s_tx_stop(void);
int tmpa9xx_i2s_rx_start(void);
int tmpa9xx_i2s_rx_stop(void);

/* for use in interrupt handler */
unsigned int tmpa9xx_i2s_curr_offset_rx(void);
unsigned int tmpa9xx_i2s_curr_offset_tx(void);

#endif /* TMPA9XX_I2S_H */
