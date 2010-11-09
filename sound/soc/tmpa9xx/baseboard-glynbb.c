/*
 * tmpa9xx-glynbb -  SoC audio for TMPA900 Glyn baseboard
 *
 * Copyright 2010 kernel concepts
 *
 * Authors: Nils Faerber <nils.faerber@kernelconcepts.de>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>

#include "tmpa9xx-i2s.h"
#include "tmpa9xx-pcm.h"
#include "../codecs/wm8983.h"


static int glynbb_startup(struct snd_pcm_substream *substream)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_codec *codec = rtd->socdev->card->codec;

        return 0;
}


static int glynbb_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
        struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
        unsigned int clk = 0;
        int ret = 0;

        switch (params_rate(params)) {
        case 8000:
        case 16000:
        case 48000:
        case 96000:
                clk = 12288000;
                break;
        case 11025:
        case 22050:
        case 44100:
                clk = 11289600;
                break;
        }

        /* set codec DAI configuration */
        ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
        if (ret < 0)
                return ret;

        /* set cpu DAI configuration */
        ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
        if (ret < 0)
                return ret;
#if 0
        /* set the codec system clock for DAC and ADC */
        ret = snd_soc_dai_set_sysclk(codec_dai, WM8983_SYSCLK, clk,
                SND_SOC_CLOCK_IN);
        if (ret < 0)
                return ret;
#endif
#if 0
        /* set the I2S system clock as input (unused) */
        ret = snd_soc_dai_set_sysclk(cpu_dai, PXA2XX_I2S_SYSCLK, 0,
                SND_SOC_CLOCK_IN);
        if (ret < 0)
                return ret;
#endif
        return 0;
}


static struct snd_soc_ops glynbb_ops = {
	.startup = glynbb_startup,
	.hw_params = glynbb_hw_params,
};

/* GlynBB machine audio_map */
/* we only have line-out and line-in */
static const struct snd_soc_dapm_route audio_map[] = {

        /* headphone connected to LOUT1, ROUT1 */
        {"Left Headphone Out", NULL, "LHP"},
        {"Right Headphone Out", NULL, "RHP"},

        /* line is connected to input 1 - no bias */
        {"LAUX", NULL, "Left Boost Mixer"},
        {"RAUX", NULL, "Right Boost Mixer"},
};

static int glynbb_wm8983_init(struct snd_soc_codec *codec)
{
	int err;

	// printk(KERN_ERR "glynbb_wm8983_init()\n");
	/* NC codec pins */
	snd_soc_dapm_nc_pin(codec, "LSPK");
	snd_soc_dapm_nc_pin(codec, "RSPK");
	snd_soc_dapm_nc_pin(codec, "R2");
	snd_soc_dapm_nc_pin(codec, "L2");
	snd_soc_dapm_nc_pin(codec, "LMICN");
	snd_soc_dapm_nc_pin(codec, "LMICP");
	snd_soc_dapm_nc_pin(codec, "RMICN");
	snd_soc_dapm_nc_pin(codec, "RMICP");

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	return 0;
}


/* GlynBB digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link glynbb_dai = {
	.name = "wm8983",
	.stream_name = "WM8983",
	.cpu_dai = &tmpa_i2s_dai,
	.codec_dai = &wm8983_dai,
	.init = glynbb_wm8983_init,
	.ops = &glynbb_ops,
};

/* GlynBB audio machine driver */
static struct snd_soc_card snd_soc_glynbb = {
	.name = "GlynBB",
	.platform = &tmpa9xx_soc_platform,
	.dai_link = &glynbb_dai,
	.num_links = 1,
};

static struct snd_soc_device glynbb_snd_devdata = {
	.card = &snd_soc_glynbb,
	.codec_dev = &soc_codec_dev_wm8983,
};

static struct platform_device *glynbb_snd_device;

static int __init glynbb_init(void)
{
	int ret;

	glynbb_snd_device = platform_device_alloc("soc-audio", -1);
	if (!glynbb_snd_device) {
		printk(KERN_ERR "Alsa SOC glynbb platform_device_alloc failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(glynbb_snd_device, &glynbb_snd_devdata);
	glynbb_snd_devdata.dev = &glynbb_snd_device->dev;
	ret = platform_device_add(glynbb_snd_device);
	if (ret) {
		printk(KERN_ERR "Alsa SOC glynbb platform_device_add failed\n");
		platform_device_put(glynbb_snd_device);
	}

	// printk(KERN_ERR "%s =%d\n", __FUNCTION__, ret);

	return ret;
}

static void __exit glynbb_exit(void)
{
	platform_device_unregister(glynbb_snd_device);
}

module_init(glynbb_init);
module_exit(glynbb_exit);

/* Module information */
MODULE_AUTHOR("Nils Faerber");
MODULE_DESCRIPTION("ALSA SoC Glyn Tonga baseboard");
MODULE_LICENSE("GPL");

