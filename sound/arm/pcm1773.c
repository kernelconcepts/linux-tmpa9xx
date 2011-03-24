/*
 * PCM1773 codec driver for Toshiba TMPA9xx SoC
 *
 * Copyright (c) Toshiba
 * Copyright (c) 2009 OPEN-engineering.de <info@open-engineering.de
 * Copyright (c) 2011 Michael Hunold (michael@mihu.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <sound/core.h>
#include <sound/info.h>
#include <sound/control.h>
#include <sound/pcm.h>
#define SNDRV_GET_ID
#include <sound/initval.h>

#include <mach/dma.h>

#include "snd-tmpa9xx-i2s.h"

#define snd_printk_marker(...) \
	dev_dbg(chip->dev, __VA_ARGS__);

#undef CONFIG_SND_DEBUG_CURRPTR  /* causes output every frame! */

#undef NOCONTROLS  /* define this to omit all the ALSA controls */

#define DRIVER_NAME	"PCM1733-I2S"
#define CHIP_NAME	"PCM1733"
#define PCM_NAME	"PCM1733_PCM"

/* Chip level */
#define PCM1733_BUF_SZ 	0x10000  /* 64kb */
#define PCM_BUFFER_MAX	(PCM1733_BUF_SZ / 2)

#define FRAGMENTS_MIN	2
#define FRAGMENTS_MAX	32

struct snd_pcm1773
{
	struct device *dev;

	struct snd_card *card;
	struct snd_pcm *pcm;

	/* if non-null, current subtream running */
	struct snd_pcm_substream *tx_substream;
};

/* pcm methods */
static struct snd_pcm_hardware snd_pcm1773_playback_hw = {
	.info = ( SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER ),
	.formats =      SNDRV_PCM_FMTBIT_S16_LE,
	.rates =            (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
				   SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |\
				   SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
				   SNDRV_PCM_RATE_KNOT),
	.rate_min =	    8000,
	.rate_max =	    48000,
	.channels_min =	    2,
	.channels_max =     2,
	.buffer_bytes_max = PCM_BUFFER_MAX,
	.period_bytes_min = 0x1000,     /* 4KB */
	.period_bytes_max = 0x3000,     /* 8KB */
	.periods_min =      FRAGMENTS_MIN,
	.periods_max =      FRAGMENTS_MAX,
};

static int snd_pcm1773_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm1773 *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker("%s():\n", __func__);
	chip->tx_substream = substream;
	substream->runtime->hw = snd_pcm1773_playback_hw;

	return 0;
}

static int snd_pcm1773_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm1773 *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker("%s():\n", __func__);
	chip->tx_substream = NULL;

	return 0;
}

/* I2S in following */
static int snd_pcm1773_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *hwparams)
{
	struct snd_pcm1773 *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker("%s():\n", __func__);

	/*
	*  Allocate all available memory for our DMA buffer.
	*  Necessary because we get a 4x increase in bytes for the 2 channel mode.
	*  (we lie to the ALSA midlayer through the hwparams data)
	*  We're relying on the driver not supporting full duplex mode
	*  to allow us to grab all the memory.
	*/
	if (snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hwparams)) < 0 )
		return -ENOMEM;

	return 0;
}

static int snd_pcm1773_hw_free(struct snd_pcm_substream * substream)
{
	struct snd_pcm1773 *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker("%s():\n", __func__);

	snd_pcm_lib_free_pages(substream);

	return 0;
}

static void snd_pcm1773_dma_tx(void *data);

static int snd_pcm1773_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm1773 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct tmpa9xx_i2s_config config;
	int fragsize_bytes = frames_to_bytes(runtime, runtime->period_size);
	int word_len = 4;
	int ret;

	if (substream != chip->tx_substream)
		return -EINVAL;

	config.cpu_buf = runtime->dma_area;
	config.phy_buf = runtime->dma_addr;
	config.fragcount = runtime->periods;
	config.fragsize = fragsize_bytes;
	config.size = word_len;
	config.callback = snd_pcm1773_dma_tx;
	config.data = chip;

	ret = tmpa9xx_i2s_tx_setup(&config);
	if (ret)
		return -EINVAL;

	dev_dbg(chip->dev, "sample rate %d, channels %d, period bytes %d, periods %d\n", runtime->rate, runtime->channels, frames_to_bytes(runtime, runtime->period_size), runtime->periods);

	return 0;
}

static int snd_pcm1773_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm1773 *chip = snd_pcm_substream_chip(substream);
	int ret;

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			snd_printk_marker("%s(): SNDRV_PCM_TRIGGER_START\n", __func__);
			ret = tmpa9xx_i2s_tx_start();
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			snd_printk_marker("%s(): SNDRV_PCM_TRIGGER_STOP\n", __func__);
			ret = tmpa9xx_i2s_tx_stop();
			break;
		default:
			return -EINVAL;
			break;
	}

	return 0;
}

static snd_pcm_uframes_t snd_pcm1773_playback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm1773 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int offset;

	snd_printk_marker("%s():\n", __func__);

	offset = tmpa9xx_i2s_curr_offset_tx();

	offset = bytes_to_frames(runtime, offset);
	if (offset >= runtime->buffer_size)
		offset = 0;

	return offset;
}

/* pcm method tables */
static struct snd_pcm_ops snd_pcm1773_playback_ops = {
	.open      = snd_pcm1773_playback_open,
	.close     = snd_pcm1773_playback_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.hw_params = snd_pcm1773_hw_params,
	.hw_free   = snd_pcm1773_hw_free,
	.prepare   = snd_pcm1773_playback_prepare,
	.trigger   = snd_pcm1773_playback_trigger,
	.pointer   = snd_pcm1773_playback_pointer,
};

/* card and device */
static int snd_pcm1773_stop(struct snd_pcm1773 *chip)
{
	snd_printk_marker("%s():\n", __func__);

	return 0;
}

static int snd_pcm1773_dev_free(struct snd_device *device)
{
	struct snd_pcm1773 *chip = (struct snd_pcm1773 *)device->device_data;

	snd_printk_marker("%s():\n", __func__);

	return snd_pcm1773_stop(chip);
}

static struct snd_device_ops snd_pcm1773_ops = {
	.dev_free = snd_pcm1773_dev_free,
};

static void snd_pcm1773_dma_tx(void *data)
{
	struct snd_pcm1773 *chip = data;

	snd_printk_marker("%s():\n", __func__);

	if (chip->tx_substream)
		snd_pcm_period_elapsed(chip->tx_substream);
}

static int __devinit snd_pcm1773_pcm(struct snd_pcm1773 *chip)
{
	struct snd_pcm *pcm;
	int ret;

	snd_printk_marker("%s():\n", __func__);

	/* 1 playback of 2-8 channels each */
	ret = snd_pcm_new(chip->card, PCM_NAME, 0, 1, 0, &pcm);
	if (ret < 0) {
		dev_err(chip->dev, "snd_pcm_new() failed\n");
		return ret;
	}

	/*
	 * this sets up our initial buffers and sets the dma_type to isa.
	 * isa works but I'm not sure why (or if) it's the right choice
	 * this may be too large, trying it for now
	 */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
			snd_dma_isa_data(), PCM1733_BUF_SZ, PCM1733_BUF_SZ);

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_pcm1773_playback_ops);
	chip->pcm = pcm;
	pcm->info_flags = 0;

	strcpy(pcm->name, PCM_NAME);
	pcm->private_data = chip;

	return 0;
}

static int __devinit tmpa9xx_pcm1773_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_pcm1773 *pcm1773;
	struct snd_card *card;
	char *id = "ID string for TMPA9XX + PCM1773 soundcard.";
	int ret = 0;

	snd_card_create(-1, id, THIS_MODULE, sizeof(struct snd_pcm1773), &card);
	if (!card) {
		dev_err(dev, "snd_card_create() failed\n");
		return -ENOMEM;
	}

	strcpy(card->driver, DRIVER_NAME);
	strcpy(card->shortname, CHIP_NAME);
	sprintf(card->longname, "%s at I2S", card->shortname);

	pcm1773 = card->private_data;
	pcm1773->card = card;
	pcm1773->dev = dev;

	ret = snd_device_new(card, SNDRV_DEV_LOWLEVEL, pcm1773, &snd_pcm1773_ops);
	if (ret) {
		dev_err(dev, "snd_device_new() failed\n");
		ret = -ENODEV;
		goto err1;
	}

	ret = snd_pcm1773_pcm(pcm1773);
	if (ret) {
		dev_err(dev, "snd_pcm1773_pcm() failed\n");
		ret = -ENODEV;
		goto err2;
	}

	ret = snd_card_register(card);
	if (ret) {
		dev_err(dev, "snd_card_register() failed\n");
		ret = -ENODEV;
		goto err3;
	}

	dev_set_drvdata(dev, pcm1773);
	snd_card_set_dev(card, dev);

	return 0;

err3:
err2:
err1:
	snd_card_free(card);
	return ret;

	return 0;
}

static int __devexit tmpa9xx_pcm1773_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_pcm1773 *pcm1773 = dev_get_drvdata(dev);

	snd_card_free(pcm1773->card);

	return 0;
}

static struct platform_driver tmpa9xx_pcm1773_driver =
{
	.probe	= tmpa9xx_pcm1773_probe,
	.remove	= __devexit_p(tmpa9xx_pcm1773_remove),
	.driver = {
		.name = "tmpa9xx-pcm1773",
		.owner = THIS_MODULE,
	},
};

static int __init tmpa9xx_pcm1773_init(void)
{
	return platform_driver_register(&tmpa9xx_pcm1773_driver);
}

static void __exit tmpa9xx_pcm1773_exit(void)
{
	platform_driver_unregister(&tmpa9xx_pcm1773_driver);
}

module_init(tmpa9xx_pcm1773_init);
module_exit(tmpa9xx_pcm1773_exit);

MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("PCM1773 driver for TMPA910");
MODULE_LICENSE("GPL");

