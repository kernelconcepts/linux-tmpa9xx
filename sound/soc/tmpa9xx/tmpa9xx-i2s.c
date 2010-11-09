/*
 * tmpa9xx-i2s.c  --  ALSA Soc Audio Layer
 *
 * Copyright 2010 Nils Faerber <nils.faerber@kernelconcepts.de>
 * based on tmpa9xx-i2s by
 *  Liam Girdwood
 *         lrg@slimlogic.co.uk
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <mach/hardware.h>
#include <mach/dma.h>
/*#include <mach/audio.h>*/

#include <mach/regs.h>

#include "tmpa9xx-pcm.h"
#include "tmpa9xx-i2s.h"


static int tmpa9xx_i2s_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;

	printk(KERN_ERR "%s\n", __FUNCTION__);
	return 0;
}

static void tmpa9xx_i2s_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	printk(KERN_ERR "%s\n", __FUNCTION__);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
	} else {
	}

}

static int tmpa9xx_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			      struct snd_soc_dai *dai)
{
	int ret = 0;

	printk(KERN_ERR "%s\n", __FUNCTION__);
	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				ret=0;
			else
				ret=0;
			break;
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			break;
		default:
			ret = -EINVAL;
	}

	return ret;
}

static int tmpa9xx_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct tmpa9xx_pcm_dma_params *dma_data;

	printk(KERN_ERR "%s\n", __FUNCTION__);
#if 0
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		NOP;
	else
		NOP;
#endif

	snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data);

	switch (params_rate(params)) {
	case 8000:
		break;
	case 11025:
		break;
	case 16000:
		break;
	case 22050:
		break;
	case 44100:
		break;
	case 48000:
		break;
	case 96000: /* not in manual and possibly slightly inaccurate */
		break;
	}

	return 0;
}

static int tmpa9xx_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	printk(KERN_ERR "%s\n", __FUNCTION__);
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			break;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBS_CFS:
			break;
		case SND_SOC_DAIFMT_CBM_CFS:
			break;
		default:
			break;
	}

	return 0;
}

static int tmpa9xx_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
		int clk_id, unsigned int freq, int dir)
{

	return 0;
}

#ifdef CONFIG_PM
static int tmpa9xx_i2s_suspend(struct snd_soc_dai *dai)
{
	printk(KERN_ERR "%s\n", __FUNCTION__);
	return 0;
}

static int tmpa9xx_i2s_resume(struct snd_soc_dai *dai)
{
	printk(KERN_ERR "%s\n", __FUNCTION__);
	return 0;
}
#else
#define tmpa9xx_i2s_suspend	NULL
#define tmpa9xx_i2s_resume	NULL
#endif

#define TMPA9XX_I2S_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 )


static struct snd_soc_dai_ops tmpa_i2s_dai_ops = {
	.startup	= tmpa9xx_i2s_startup,
	.shutdown	= tmpa9xx_i2s_shutdown,
	.trigger	= tmpa9xx_i2s_trigger,
	.hw_params	= tmpa9xx_i2s_hw_params,
	.set_fmt	= tmpa9xx_i2s_set_dai_fmt,
	.set_sysclk	= tmpa9xx_i2s_set_dai_sysclk,
};

struct snd_soc_dai tmpa_i2s_dai = {
	.name = "tmpa9xx-i2s",
	.id = 0,
	.suspend = tmpa9xx_i2s_suspend,
	.resume = tmpa9xx_i2s_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = TMPA9XX_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = TMPA9XX_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &tmpa_i2s_dai_ops,
	.symmetric_rates = 1,
};

EXPORT_SYMBOL_GPL(tmpa_i2s_dai);

static int __init tmpa9xx_i2s_probe(struct platform_device *dev)
{
	int ret;

	tmpa_i2s_dai.dev = &dev->dev;
	tmpa_i2s_dai.private_data = NULL;
	ret = snd_soc_register_dai(&tmpa_i2s_dai);

	printk(KERN_ERR "%s snd_soc_register_dai()=%d\n", __FUNCTION__, ret);

	return ret;
}

static int __devexit tmpa9xx_i2s_remove(struct platform_device *dev)
{
	snd_soc_unregister_dai(&tmpa_i2s_dai);

	printk(KERN_ERR "%s\n", __FUNCTION__);
	return 0;
}

static struct platform_driver tmpa9xx_i2s_driver = {
	.probe = tmpa9xx_i2s_probe,
	.remove = __devexit_p(tmpa9xx_i2s_remove),

	.driver = {
		.name = "tmpa9xx-i2s",
		.owner = THIS_MODULE,
	},
};

static int __init tmpa9xx_i2s_init(void)
{
	int ret = platform_driver_register(&tmpa9xx_i2s_driver);

	printk(KERN_ERR "%s platform_driver_register()=%d\n", __FUNCTION__, ret);

	return ret;
}

static void __exit tmpa9xx_i2s_exit(void)
{
	platform_driver_unregister(&tmpa9xx_i2s_driver);
}

module_init(tmpa9xx_i2s_init);
module_exit(tmpa9xx_i2s_exit);

/* Module information */
MODULE_AUTHOR("Nils Faerber <nils.faerber@kernelconcepts.de>");
MODULE_DESCRIPTION("tmpa9xx I2S SoC Interface");
MODULE_LICENSE("GPL");

