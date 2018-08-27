/*
 * Wolfson wm89xx codec driver for Toshiba TMPA9xx SoC
 *
 * Copyright (c) Toshiba
 * Copyright (c) 2011 Michael Hunold (michael@mihu.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
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

#define DRIVER_NAME	"WM89XX-I2S"
#define CHIP_NAME	"Wolfson WM89XX"
#define PCM_NAME	"WM89XX_PCM"

/* Chip level */
#define WM89XX_BUF_SZ 	0x10000  /* 64kb */
#define PCM_BUFFER_MAX	(WM89XX_BUF_SZ / 2)

#define FRAGMENTS_MIN	2
#define FRAGMENTS_MAX	32

static unsigned char wm89xx_for_samplerate[7][6] =
{
	{0x00,0x08,0x0C,0x93,0xE9,0x49}, 	/* 48000 Hz */
	{0x00,0x07,0x21,0x61,0x27,0x49}, 	/* 44100 Hz */
	{0x02,0x08,0x0C,0x93,0xE9,0x69}, 	/* 32000 Hz */
	{0x04,0x07,0x21,0x61,0x27,0x89}, 	/* 22050 Hz */
	{0x05,0x07,0x21,0x61,0x27,0xa9}, 	/* 16000 Hz */
	{0x08,0x07,0x21,0x61,0x27,0xC9}, 	/* 12000 Hz */
	{0x0a,0x08,0x0C,0x93,0xE9,0xE9}, 	/* 8000 Hz */
};

struct snd_wm89xx
{
	struct device *dev;

	struct snd_card *card;
	struct i2c_client *i2c_client;
	struct snd_pcm *pcm;

	/* if non-null, current subtream running */
	struct snd_pcm_substream *rx_substream;
	/* if non-null, current subtream running */
	struct snd_pcm_substream *tx_substream;

	int snd_speaker_volume;
};

#define i2c_packet_send(reg, val) \
	{ \
	int ret = i2c_smbus_write_byte_data(i2c_client, reg, val); \
	if (ret) \
		dev_err(&i2c_client->dev, "i2c_smbus_write_byte_data() failed, ret %d\n", ret); \
	}

// define as function because return value is required
static s32 i2c_packet_read(struct i2c_client* i2c_client, u8 reg) 
	{ 
	s32 ret = i2c_smbus_read_byte_data(i2c_client, reg); 
	if (ret) 
	{
		dev_err(&i2c_client->dev, "i2c_smbus_read_byte_data() failed, ret %d\n", ret); 
		return 0; // something needs to be returned -> send 0
	}
	else
	{
		printk("read ret: %d\n", ret);
		return ret; // if not failed this contains the data
	}
	//printk("If you read this something is really fucked up");
	//return 0; // for compiler - should not occur
	}

#ifdef CONFIG_MACH_TOPASA900
static void init_wm89xx_i2c(struct i2c_client *i2c_client)
{
	i2c_packet_send(0x00,0x00);	/* R0  0x000 */
	i2c_packet_send(0x02,0x3d);	/* R1  0x02D */
	i2c_packet_send(0x05,0x95);	/* R2  0x195 */
	i2c_packet_send(0x06,0x6F);	/* R3  0x00F */
	i2c_packet_send(0x08,0x10);	/* R4  0X010 */
	i2c_packet_send(0x0a,0x00);	/* R5  0X000 */
	i2c_packet_send(0x14, 0x80);	/* R10 = 0x080 */
	i2c_packet_send(0x17,0xff);	/* R11 0X1ff */
	i2c_packet_send(0x19,0xff);	/* R12 0X1ff */
	i2c_packet_send(0x1c,0x00);	/* R14 = 0x1c01 */
	i2c_packet_send(0x1F,0xff);	/* R15 0X1FF */
	i2c_packet_send(0x30, 0x32);	/* R24 = 0x032 */
	i2c_packet_send(0x5b,0x3f);	/* R45 0X000 */
	i2c_packet_send(0x56,0x10);	/* R43 0X010   add for wm89xx 8 ohm speaker */
	i2c_packet_send(0x5f,0x55);	/* R47 0X005 */
	i2c_packet_send(47<<1 | 1,0xff);	/* R47 0X1ff */
	i2c_packet_send(0x62,0x02);	/* R49 0X002 */
	i2c_packet_send(0x64,0x01);	/* R50 0X001 */
	i2c_packet_send(0x66,0x01);	/* R51 0X001 */
	i2c_packet_send(0x69,0x3f);	/* R52 0X13f - volume left speaker - 0x3f seems to be max*/
	i2c_packet_send(0x6b,0x3f);	/* R53 0X13f - volume right speaker*/
}
#endif



#ifdef CONFIG_MACH_TONGA2_TFTTIMER
static void init_wm89xx_i2c(struct i2c_client *i2c_client)
{
	i2c_packet_send(0x00,0x00);	/* R0  0x000 - Reset OFF*/
	i2c_packet_send((0x02),0x2f);	/* R1  0x03D - PLL Enable, fasted startup*/
	i2c_packet_send(0x04,0x00);	/* R2  0x015 - no ADC*/
	i2c_packet_send(0x06,0xed);	/* R3  0x0ed - SPKP/N enable, spk mix dac enable*/
	i2c_packet_send(0x08,0x10);	/* R4  0X010 - i2s format*/
	i2c_packet_send(0x0a,0x00);	/* R5  0X000 - reset value */
	i2c_packet_send(0x14,0x00);	/* R10 0x080 */
	i2c_packet_send(0x16,0xff);	/* R11 0X0ff - max volume */
	i2c_packet_send(0x1c,0x00);	/* R14 0x1c01 */
	i2c_packet_send(0x1e,0x00);	/* R15 0X000 - ADC off */
	i2c_packet_send(0x62,0x02);	/* R49 - no boost  */
	i2c_packet_send(0x64,0x01);	/* R50 0X001 */
	i2c_packet_send(54<<1,0x39);	/* R54 0dB */
	i2c_packet_send(56<<1,0x01);	/* R56 DAC to mono mix */
}
#endif

#ifdef CONFIG_MACH_TONGA
static void init_wm89xx_i2c(struct i2c_client *i2c_client)
{
	i2c_packet_send(0x00,0x00);	/* R0  0x000 */
	i2c_packet_send(0x02,0x3d);	/* R1  0x02D */
	i2c_packet_send(0x05,0x95);	/* R2  0x195 */
	i2c_packet_send(0x06,0x6F);	/* R3  0x00F */
	i2c_packet_send(0x08,0x10);	/* R4  0X010 */
	i2c_packet_send(0x0a,0x00);	/* R5  0X000 */
	i2c_packet_send(0x14, 0x80);	/* R10 = 0x080 */
	i2c_packet_send(0x17,0xff);	/* R11 0X1ff */
	i2c_packet_send(0x19,0xff);	/* R12 0X1ff */
	i2c_packet_send(0x1c,0x00);	/* R14 = 0x1c01 */
	i2c_packet_send(0x1F,0xff);	/* R15 0X1FF */
	i2c_packet_send(0x30, 0x32);	/* R24 = 0x032 */
	i2c_packet_send(0x5b,0x3f);	/* R45 0X000 */
	i2c_packet_send(0x56,0x10);	/* R43 0X010   add for WM8983 8 ohm speaker */
	i2c_packet_send(0x5f,0x55);	/* R47 0X005 */
	i2c_packet_send(47<<1 | 1,0xff);	/* R47 0X1ff */
	i2c_packet_send(0x62,0x02);	/* R49 0X002 */
	i2c_packet_send(0x64,0x01);	/* R50 0X001 */
	i2c_packet_send(0x66,0x01);	/* R51 0X001 */
	i2c_packet_send(0x69,0x3f);	/* R52 0X13f */
	i2c_packet_send(0x6b,0x3f);	/* R53 0X13f */
}
#endif

static void snd_wm89xx_set_samplerate(struct i2c_client *i2c_client, long rate)
{
	/* wait for any frame to complete */
	udelay(125);

	if (rate >= 48000)
		rate = 0;
	else if (rate >= 44100)
		rate = 1;
	else if (rate >= 32000)
		rate = 2;
	else if (rate >= 22050)
		rate = 3;
	else if (rate >= 16000)
		rate = 4;
	else if (rate >= 12000)
		rate = 5;
	else
		rate = 6;

	i2c_packet_send(0x0d, wm89xx_for_samplerate[rate][5]);    /* R6  */
	i2c_packet_send(0x0e, wm89xx_for_samplerate[rate][0]);    /* R7  */
	i2c_packet_send(0x48, wm89xx_for_samplerate[rate][1]);    /* R36 */
	i2c_packet_send(0x4a, wm89xx_for_samplerate[rate][2]);    /* R37 */
	i2c_packet_send(0x4d, wm89xx_for_samplerate[rate][3]);    /* R38 */
	i2c_packet_send(0x4e, wm89xx_for_samplerate[rate][4]);    /* R39 */
}

/* pcm methods */
static struct snd_pcm_hardware snd_wm89xx_playback_hw = {
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

static struct snd_pcm_hardware snd_wm89xx_capture_hw = {
	.info = ( SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER ),
	.formats =          SNDRV_PCM_FMTBIT_S16_LE,
	.rates =            (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
				   SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |\
				   SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
				   SNDRV_PCM_RATE_KNOT),
	.rate_min =         8000,
	.rate_max =         48000,
	.channels_min =     1,
	.channels_max =     2,
	.buffer_bytes_max = PCM_BUFFER_MAX,
	.period_bytes_min = 0x1000,     /* 4KB */
	.period_bytes_max = 0x3000,     /* 8KB */
	.periods_min =      FRAGMENTS_MIN,
	.periods_max =      FRAGMENTS_MAX,
};

static int snd_wm89xx_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_wm89xx *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker("%s():\n", __func__);
	chip->tx_substream = substream;
	substream->runtime->hw = snd_wm89xx_playback_hw;

	return 0;
}

static int snd_wm89xx_capture_open(struct snd_pcm_substream *substream)
{
	struct snd_wm89xx *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker("%s():\n", __func__);
	substream->runtime->hw = snd_wm89xx_capture_hw;
	chip->rx_substream = substream;

	return 0;
}

static int snd_wm89xx_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_wm89xx *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker("%s():\n", __func__);
	chip->tx_substream = NULL;

	return 0;
}

static int snd_wm89xx_capture_close(struct snd_pcm_substream *substream)
{
	struct snd_wm89xx *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker("%s():\n", __func__);
	chip->rx_substream = NULL;
	return 0;
}

/* I2S in following */
static int snd_wm89xx_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *hwparams)
{
	struct snd_wm89xx *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker("%s():\n", __func__);

	/*
	*  Allocate all available memory for our DMA buffer.
	*  Necessary because we get a 4x increase in bytes for the 2 channel mode.
	*  (we lie to the ALSA midlayer through the hwparams data)
	*  We're relying on the driver not supporting full duplex mode
	*  to allow us to grab all the memory.
	*/
	//printk("VOR DEM IF IN snd_wm89xx_hw_params");
	if (snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hwparams)) < 0 )
		return -ENOMEM;

	return 0;
}

static int snd_wm89xx_hw_free(struct snd_pcm_substream * substream)
{
	struct snd_wm89xx *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker("%s():\n", __func__);

	snd_pcm_lib_free_pages(substream);

	return 0;
}

static void snd_wm89xx_dma_tx(void *data);

static int snd_wm89xx_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_wm89xx *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct tmpa9xx_i2s_config config;
	int fragsize_bytes = frames_to_bytes(runtime, runtime->period_size);
	int word_len = 4;
	int ret;

	if (substream != chip->tx_substream)
		return -EINVAL;

	/* set requested samplerate */
	snd_wm89xx_set_samplerate(chip->i2c_client, runtime->rate);

	config.cpu_buf = runtime->dma_area;
	config.phy_buf = runtime->dma_addr;
	config.fragcount = runtime->periods;
	config.fragsize = fragsize_bytes;
	config.size = word_len;
	config.callback = snd_wm89xx_dma_tx;
	config.data = chip;

	ret = tmpa9xx_i2s_tx_setup(&config);
	if (ret)
		return -EINVAL;

	dev_dbg(chip->dev, "sample rate %d, channels %d, period bytes %d, periods %d\n", runtime->rate, runtime->channels, frames_to_bytes(runtime, runtime->period_size), runtime->periods);

	return 0;
}

static void snd_wm89xx_dma_rx(void *data);

static int snd_wm89xx_capture_prepare(struct snd_pcm_substream *substream)
{
	struct snd_wm89xx *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct tmpa9xx_i2s_config config;
	int fragsize_bytes = frames_to_bytes(runtime, runtime->period_size);
	int word_len = 4;
	int ret;

	if (substream != chip->tx_substream)
		return -EINVAL;

	/* set requested samplerate */
	snd_wm89xx_set_samplerate(chip->i2c_client, runtime->rate);

	config.cpu_buf = runtime->dma_area;
	config.phy_buf = runtime->dma_addr;
	config.fragcount = runtime->periods;
	config.fragsize = fragsize_bytes;
	config.size = word_len;
	config.callback = snd_wm89xx_dma_rx;
	config.data = chip;

	ret = tmpa9xx_i2s_rx_setup(&config);
	if (ret)
		return -EINVAL;

	dev_dbg(chip->dev, "sample rate %d, channels %d, period bytes %d, periods %d\n", runtime->rate, runtime->channels, frames_to_bytes(runtime, runtime->period_size), runtime->periods);

	return 0;
}

static int snd_wm89xx_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_wm89xx *chip = snd_pcm_substream_chip(substream);
	int ret;

	snd_printk_marker("%s():\n", __func__);

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

	snd_printd(KERN_INFO "playback cmd:%s. ret=%d\n", cmd?"start":"stop", ret);

	return 0;
}

static int snd_wm89xx_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_wm89xx *chip = snd_pcm_substream_chip(substream);
	int ret;

	snd_printk_marker("%s():\n", __func__);

	if (substream != chip->rx_substream)
		return -EINVAL;

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			snd_printk_marker("%s(): SNDRV_PCM_TRIGGER_START\n", __func__);
			ret = tmpa9xx_i2s_rx_start();
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			snd_printk_marker("%s(): SNDRV_PCM_TRIGGER_STOP\n", __func__);
			tmpa9xx_i2s_rx_stop();
			break;
		default:
			return -EINVAL;
			break;
	}

	snd_printd(KERN_ERR"capture cmd:%s\n", cmd ? "start" : "stop");

	return 0;
}

static snd_pcm_uframes_t snd_wm89xx_playback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int offset;

	offset = tmpa9xx_i2s_curr_offset_tx();

	offset = bytes_to_frames(runtime, offset);
	if (offset >= runtime->buffer_size)
		offset = 0;

	return offset;
}

static snd_pcm_uframes_t snd_wm89xx_capture_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int offset;

	offset = tmpa9xx_i2s_curr_offset_rx();

	offset = bytes_to_frames(runtime, offset);

	if (offset >= runtime->buffer_size)
		offset = 0;

	return offset;
}

/* pcm method tables */
static struct snd_pcm_ops snd_wm89xx_playback_ops = {
	.open      = snd_wm89xx_playback_open,
	.close     = snd_wm89xx_playback_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.hw_params = snd_wm89xx_hw_params,
	.hw_free   = snd_wm89xx_hw_free,
	.prepare   = snd_wm89xx_playback_prepare,
	.trigger   = snd_wm89xx_playback_trigger,
	.pointer   = snd_wm89xx_playback_pointer,
};

static struct snd_pcm_ops snd_wm89xx_capture_ops = {
	.open  = snd_wm89xx_capture_open,
	.close = snd_wm89xx_capture_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_wm89xx_hw_params,
	.hw_free   = snd_wm89xx_hw_free,
	.prepare   = snd_wm89xx_capture_prepare,
	.trigger   = snd_wm89xx_capture_trigger,
	.pointer   = snd_wm89xx_capture_pointer,
};

/* card and device */
static int snd_wm89xx_stop(struct snd_wm89xx *chip)
{
	snd_printk_marker("%s():\n", __func__);

	return 0;
}

static int snd_wm89xx_dev_free(struct snd_device *device)
{
	struct snd_wm89xx *chip = (struct snd_wm89xx *)device->device_data;

	snd_printk_marker("%s():\n", __func__);

	return snd_wm89xx_stop(chip);
}

static struct snd_device_ops snd_wm89xx_ops = {
	.dev_free = snd_wm89xx_dev_free,
};

static void snd_wm89xx_dma_rx(void *data)
{
	struct snd_wm89xx *chip = data;

        snd_printk_marker("%s():\n", __func__);

	if (chip->rx_substream)
		snd_pcm_period_elapsed(chip->rx_substream);
}

static void snd_wm89xx_dma_tx(void *data)
{
	struct snd_wm89xx *chip = data;

	snd_printk_marker("%s():\n", __func__);

	if (chip->tx_substream)
		snd_pcm_period_elapsed(chip->tx_substream);
}

static int __devinit snd_wm89xx_pcm(struct snd_wm89xx *chip)
{
	struct snd_pcm *pcm;
	int ret;

	snd_printk_marker("%s():\n", __func__);

	/* 1 playback and 1 capture substream, of 2-8 channels each */
	ret = snd_pcm_new(chip->card, PCM_NAME, 0, 1, 1, &pcm);
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
			snd_dma_isa_data(), WM89XX_BUF_SZ, WM89XX_BUF_SZ);

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_wm89xx_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_wm89xx_capture_ops);
	chip->pcm = pcm;
	pcm->info_flags = 0;

	strcpy(pcm->name, PCM_NAME);
	pcm->private_data = chip;

	return 0;
}

// ALSA CONTROLS FOR TOPASA900 

#ifdef CONFIG_MACH_TOPASA900


static int snd_volumeCtl_mono_info(struct snd_kcontrol *kcontrol, //
		struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0x3f;
	//printk("snd_volumeCtl_mono_info was called\n"); // only for debug purposes
	return 0;
}

static s32 snd_volumeCtl_get(struct snd_kcontrol* kcontrol, //
		struct snd_ctl_elem_value *ucontrol)
{
	//create a "chip struct" (wm89xx struct) that holds the snd_speaker_volume variable
	struct snd_wm89xx *wm89xx = snd_kcontrol_chip(kcontrol);
	struct i2c_client *i2c_client = wm89xx->i2c_client;
	//printk("snd_volumeCtl_get was called(before i2cPacketRead)\n"); // only for debug purposes
	ucontrol->value.integer.value[0] = wm89xx->snd_speaker_volume;
	//printk("snd_volumeCtl_get was called(after i2cPacketRead). Result: %ld\n", ucontrol->value.integer.value[0]); // only for debug purposes
	return 0;
}

static int snd_volumeCtl_put(struct snd_kcontrol *kcontrol, //
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_wm89xx *wm89xx = snd_kcontrol_chip(kcontrol);
	struct i2c_client *i2c_client = wm89xx->i2c_client;
	int changed = 0;
	if (wm89xx->snd_speaker_volume != ucontrol->value.integer.value[0])
	{
		u8 volumeToWrite = ucontrol->value.integer.value[0];
		// write the volume to both speakers
		i2c_packet_send(0x69, volumeToWrite);
		i2c_packet_send(0x6b, volumeToWrite);
		changed = 1;
		wm89xx->snd_speaker_volume = volumeToWrite;
		//printk("Volume was set to %d\n", wm89xx->snd_speaker_volume);
	}
	//printk("snd_volumeCtl_put was called\n"); // only for debug purposes
	return changed;
}


static struct snd_kcontrol_new volumeControl /*__devinitdata*/ = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Playback Volume",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0xffff,
	.info = snd_volumeCtl_mono_info,
	.get = snd_volumeCtl_get,
	.put = snd_volumeCtl_put
};



#endif

// constructor

static int wm89xx_i2c_probe(struct i2c_client *i2c_client, const struct i2c_device_id *iid)
{
	struct snd_wm89xx *wm89xx;
	struct snd_card *card;
	char *id = "ID string for TMPA9XX + wm89xx soundcard.";
	int ret = 0;

	init_wm89xx_i2c(i2c_client);

	//create card (old version of snd_card_new)
	snd_card_create(-1, id, THIS_MODULE, sizeof(struct snd_wm89xx), &card);
	if (!card) {
		dev_err(&i2c_client->dev, "snd_card_create() failed\n");
		return -ENOMEM;
	}

	strcpy(card->driver, DRIVER_NAME);
	strcpy(card->shortname, CHIP_NAME);
	sprintf(card->longname, "%s at I2S",
		  card->shortname);

	wm89xx = card->private_data;
	wm89xx->dev = &i2c_client->dev;
	wm89xx->card = card;
	wm89xx->i2c_client = i2c_client;
	wm89xx->snd_speaker_volume = 0x3f; // this is (probably ?) max vol -> 63

	ret = snd_device_new(card, SNDRV_DEV_LOWLEVEL, wm89xx, &snd_wm89xx_ops);
	if (ret) {
		dev_err(&i2c_client->dev, "snd_device_new() failed\n");
		ret = -ENODEV;
		goto err1;
	}

	//now create the new alsa control

	ret = snd_ctl_add(card, snd_ctl_new1(&volumeControl, wm89xx));
	if (ret < 0){
		dev_err(&i2c_client->dev, "failed to initialise volume control\n");
		ret = -ENODEV;
		goto err1;
	}


	ret = snd_wm89xx_pcm(wm89xx);
	if (ret) {
		dev_err(&i2c_client->dev, "snd_wm89xx_pcm() failed\n");
		ret = -ENODEV;
		goto err2;
	}

	ret = snd_card_register(card);
	if (ret) {
		dev_err(&i2c_client->dev, "snd_card_register() failed\n");
		ret = -ENODEV;
		goto err3;
	}

	snd_card_set_dev(card, &i2c_client->dev);

	i2c_set_clientdata(i2c_client, card);

	dev_info(&i2c_client->dev, "wm89xx driver ready\n");

	return 0;

err3:
err2:
err1:
	snd_card_free(card);
	return ret;

	return 0;
}

//destructor

static int wm89xx_i2c_remove(struct i2c_client *i2c_client)
{
	struct snd_card *card = i2c_get_clientdata(i2c_client);

	snd_card_free(card);

	i2c_set_clientdata(i2c_client, NULL);

	return 0;
}
static struct i2c_device_id wm89xx_id[] = {
	{"wm89xx", 0},
	{}, /* mandatory */
};
MODULE_DEVICE_TABLE(i2c, wm89xx_id);

static struct i2c_driver wm89xx_i2c_driver = {
	.driver = {
		.name = "wm89xx",
		.owner = THIS_MODULE,
	},
	.id_table = wm89xx_id,
	.probe = wm89xx_i2c_probe,
	.remove = wm89xx_i2c_remove,
};

static int __init wm89xx_init(void)
{
	return i2c_add_driver(&wm89xx_i2c_driver);
}
module_init(wm89xx_init);

static void __exit wm89xx_exit(void)
{
	i2c_del_driver(&wm89xx_i2c_driver);
}
module_exit(wm89xx_exit);

MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("wm89xx driver for TMPA900");
MODULE_LICENSE("GPL");
