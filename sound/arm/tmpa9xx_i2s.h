#ifndef __TMPA9XX_I2S_H__
#define __TMPA9XX_I2S_H__

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <asm/dma.h>

struct tmpa9xx_i2s_config
{
	unsigned char *cpu_buf;
	unsigned int phy_buf;
	int fragcount;
	size_t fragsize;
	size_t size;
	void (*callback)(void *);
	void *data;
};

int tmpa9xx_i2s_tx_setup(struct tmpa9xx_i2s_config *c);
int tmpa9xx_i2s_tx_start(void);
int tmpa9xx_i2s_tx_stop(void);
unsigned int tmpa9xx_i2s_curr_offset_tx(void);

int tmpa9xx_i2s_rx_setup(struct tmpa9xx_i2s_config *c);
int tmpa9xx_i2s_rx_start(void);
int tmpa9xx_i2s_rx_stop(void);
unsigned int tmpa9xx_i2s_curr_offset_rx(void);

#endif /* TMPA9XX_I2S_H */
