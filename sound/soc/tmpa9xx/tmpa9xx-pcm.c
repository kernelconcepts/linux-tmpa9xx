/*
 * linux/sound/arm/tmpa9xx-pcm.c -- ALSA PCM interface for the Toshiba TMPA9xx SOC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/soc.h>

#include "tmpa9xx-pcm.h"


static int tmpa9xx_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	//struct tmpa9xx_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tmpa9xx_pcm_dma_params *dma;

	dma = snd_soc_dai_get_dma_data(rtd->dai->cpu_dai, substream);

	/* return if this is a bufferless transfer e.g. */
	if (!dma)
		return 0;

	return 0;
}

static int tmpa9xx_pcm_open(struct snd_pcm_substream *substream)
{
	printk(KERN_ERR "%s\n", __FUNCTION__);
	return 0;
}

static int tmpa9xx_pcm_close(struct snd_pcm_substream *substream)
{
	printk(KERN_ERR "%s\n", __FUNCTION__);
	return 0;
}

static int tmpa9xx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct tmpa9xx_runtime_data *prtd = substream->runtime->private_data;

	printk(KERN_ERR "%s\n", __FUNCTION__);
	if (prtd->dma_ch >= 0) {
		prtd->dma_ch = -1;
	}

	return 0;
}

static int tmpa9xx_pcm_prepare(struct snd_pcm_substream *substream)
{
	printk(KERN_ERR "%s\n", __FUNCTION__);
	return 0;
}

static int tmpa9xx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	printk(KERN_ERR "%s\n", __FUNCTION__);
	return 0;
}

static snd_pcm_uframes_t
tmpa9xx_pcm_pointer(struct snd_pcm_substream *substream)
{
	printk(KERN_ERR "%s\n", __FUNCTION__);
	return 0;
}

static int tmpa9xx_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	printk(KERN_ERR "%s\n", __FUNCTION__);
	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
									runtime->dma_area,
									runtime->dma_addr,
									runtime->dma_bytes);
}

static struct snd_pcm_ops tmpa9xx_pcm_ops = {
	.open		= tmpa9xx_pcm_open,
	.close		= tmpa9xx_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= tmpa9xx_pcm_hw_params,
	.hw_free	= tmpa9xx_pcm_hw_free,
	.prepare	= tmpa9xx_pcm_prepare,
	.trigger	= tmpa9xx_pcm_trigger,
	.pointer	= tmpa9xx_pcm_pointer,
	.mmap		= tmpa9xx_pcm_mmap,
};

static u64 tmpa9xx_pcm_dmamask = DMA_BIT_MASK(32);

static int tmpa9xx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = 64 * 1024; /* 64k */

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
										&buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;

	buf->bytes = size;

	return 0;
}

static void tmpa9xx_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	printk(KERN_ERR "%s\n", __FUNCTION__);
	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;
		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		dma_free_writecombine(pcm->card->dev, buf->bytes,
							buf->area, buf->addr);
		buf->area = NULL;
		}
}

static int tmpa9xx_soc_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;

	printk(KERN_ERR "%s\n", __FUNCTION__);

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &tmpa9xx_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (dai->playback.channels_min) {
		ret = tmpa9xx_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		ret = tmpa9xx_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

struct snd_soc_platform tmpa9xx_soc_platform = {
	.name		= "tmpa9xx-audio",
	.pcm_ops 	= &tmpa9xx_pcm_ops,
	.pcm_new	= tmpa9xx_soc_pcm_new,
	.pcm_free	= tmpa9xx_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(tmpa9xx_soc_platform);

static int __init tmpa9xx_soc_platform_init(void)
{
	int ret = snd_soc_register_platform(&tmpa9xx_soc_platform);
	printk(KERN_ERR "%s snd_soc_register_platform()=%d\n", __FUNCTION__, ret);

	return ret;
}
module_init(tmpa9xx_soc_platform_init);

static void __exit tmpa9xx_soc_platform_exit(void)
{
	printk(KERN_ERR "%s\n", __FUNCTION__);
	snd_soc_unregister_platform(&tmpa9xx_soc_platform);
}
module_exit(tmpa9xx_soc_platform_exit);

MODULE_AUTHOR("Nils Faerber");
MODULE_DESCRIPTION("Toshiba tmpa9xx PCM DMA module");
MODULE_LICENSE("GPL");

