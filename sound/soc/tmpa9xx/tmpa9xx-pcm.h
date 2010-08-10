/*
 * linux/sound/arm/tmpa9xx-pcm.h -- ALSA PCM interface for the Toshiba TMPA9xx SOC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _TMPA9XX_PCM_H
#define _TMPA9XX_PCM_H

/* platform data */
extern struct snd_soc_platform tmpa9xx_soc_platform;


struct tmpa9xx_runtime_data {
	int dma_ch;
	dma_addr_t dma_desc_array_phys;
};

#endif
