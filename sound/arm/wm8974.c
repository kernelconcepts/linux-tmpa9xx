/*
 * Wolfson wm8974 codec driver for Toshiba TMPA900 SoC
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
#include <mach/irqs.h>
#include <mach/regs.h>

#include "tmpa9xx_i2s.h"

#define I2S_DMA_RX   I2S0
#define I2S_DMA_TX   I2S1
#define I2S_IRQ_ERR  I2S_INT

#undef WM8974_DEBUG

#ifdef WM8974_DEBUG
#define wm_printd(level, format, arg...) \
	printk(level "i2s: " format, ## arg)
#define snd_printk_marker() \
	printk(KERN_DEBUG "->\n");
#else
#define wm_printd(level, format, arg...)
#define snd_printk_marker()
#endif

#undef CONFIG_SND_DEBUG_CURRPTR  /* causes output every frame! */

#undef NOCONTROLS  /* define this to omit all the ALSA controls */

#define DRIVER_NAME	"WM8974-I2S"
#define CHIP_NAME	"Wolfson WM8974"
#define PCM_NAME	"WM8974_PCM"

/* Chip level */
#define WM8974_BUF_SZ 	0x10000  /* 64kb */
#define PCM_BUFFER_MAX	(WM8974_BUF_SZ / 2)

#define FRAGMENTS_MIN	2
#define FRAGMENTS_MAX	32

static unsigned char wm8974_for_samplerate[7][6] =
{
	{0x00,0x08,0x0C,0x93,0xE9,0x49}, 	/* 48000 Hz */
	{0x00,0x07,0x21,0x61,0x27,0x49}, 	/* 44100 Hz */
	{0x02,0x08,0x0C,0x93,0xE9,0x69}, 	/* 32000 Hz */
	{0x04,0x07,0x21,0x61,0x27,0x89}, 	/* 22050 Hz */
	{0x05,0x07,0x21,0x61,0x27,0xa9}, 	/* 16000 Hz */
	{0x08,0x07,0x21,0x61,0x27,0xC9}, 	/* 12000 Hz */
	{0x0a,0x08,0x0C,0x93,0xE9,0xE9}, 	/* 8000 Hz */
};

struct snd_wm8974
{
	struct snd_card *card;
	struct tmpa9xx_i2s *i2s;
	spinlock_t wm8974_lock;
	struct i2c_client *i2c_client;
	struct snd_pcm *pcm;

	/* if non-null, current subtream running */
	struct snd_pcm_substream *rx_substream;
	/* if non-null, current subtream running */
	struct snd_pcm_substream *tx_substream;
};

/* this must be moved into tmpa9xx_i2s.c */
static void init_wm8974_i2c_hw(void)
{
	I2SCOMMON = 0x19;		/* IISSCLK = Fosch(X1),       Set SCK/WS/CLKO of Tx and Rx as Common */
	I2STMCON = 0x04;		/* I2SMCLK = Fosch/4 = 11.2896M Hz */
	I2SRMCON = 0x04;
	I2STCON = 0x00;			/* IIS Standard Format */
	I2STFCLR = 0x01;		/* Clear FIFO */
	I2SRMS = 0x00;			/* Slave */
	I2STMS = 0x00;			/* Slave */
}

#define i2c_packet_send(reg, val) \
	{ \
	int ret = i2c_smbus_write_byte_data(i2c_client, reg, val); \
	if (ret) \
		dev_err(&i2c_client->dev, "i2c_smbus_write_byte_data() failed, ret %d\n", ret); \
	}

static void init_wm8974_i2c(struct i2c_client *i2c_client)
{
	init_wm8974_i2c_hw();

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
	i2c_packet_send(0x56,0x10);	/* R43 0X010   add for wm8974 8 ohm speaker */
	i2c_packet_send(0x5f,0x55);	/* R47 0X005 */
	i2c_packet_send(47<<1 | 1,0xff);	/* R47 0X1ff */
	i2c_packet_send(0x62,0x02);	/* R49 0X002 */
	i2c_packet_send(0x64,0x01);	/* R50 0X001 */
	i2c_packet_send(0x66,0x01);	/* R51 0X001 */
	i2c_packet_send(0x69,0x3f);	/* R52 0X13f */
	i2c_packet_send(0x6b,0x3f);	/* R53 0X13f */
}

static void snd_wm8974_set_samplerate(struct i2c_client *i2c_client, long rate)
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

	i2c_packet_send(0x0d, wm8974_for_samplerate[rate][5]);    /* R6  */
	i2c_packet_send(0x0e, wm8974_for_samplerate[rate][0]);    /* R7  */
	i2c_packet_send(0x48, wm8974_for_samplerate[rate][1]);    /* R36 */
	i2c_packet_send(0x4a, wm8974_for_samplerate[rate][2]);    /* R37 */
	i2c_packet_send(0x4d, wm8974_for_samplerate[rate][3]);    /* R38 */
	i2c_packet_send(0x4e, wm8974_for_samplerate[rate][4]);    /* R39 */
}

/* pcm methods */
static struct snd_pcm_hardware snd_wm8974_playback_hw = {
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

static struct snd_pcm_hardware snd_wm8974_capture_hw = {
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

static int snd_wm8974_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_wm8974 *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker();
	chip->tx_substream = substream;
	substream->runtime->hw = snd_wm8974_playback_hw;

	return 0;
}

static int snd_wm8974_capture_open(struct snd_pcm_substream *substream)
{
	struct snd_wm8974 *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker();
	substream->runtime->hw = snd_wm8974_capture_hw;
	chip->rx_substream = substream;

	return 0;
}

static int snd_wm8974_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_wm8974 *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker();
	chip->tx_substream = NULL;

	return 0;
}

static int snd_wm8974_capture_close(struct snd_pcm_substream *substream)
{
	struct snd_wm8974 *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker();
	chip->rx_substream = NULL;
	return 0;
}

/* I2S in following */
static int snd_wm8974_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *hwparams)
{
	snd_printk_marker();

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

static int snd_wm8974_hw_free(struct snd_pcm_substream * substream)
{
	snd_printk_marker();
	snd_pcm_lib_free_pages(substream);

	return 0;
}

static int snd_wm8974_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_wm8974 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
        int word_len = 4;
	int ret;

	int fragsize_bytes = frames_to_bytes(runtime, runtime->period_size);
	wm_printd(KERN_ERR, "Playback sample rate = %d.\n",runtime->rate);

	/* set requested samplerate */
	snd_wm8974_set_samplerate(chip->i2c_client, runtime->rate);

	if (substream != chip->tx_substream)
		return -EINVAL;

	snd_printd(KERN_INFO "%s channels:%d, period_bytes:0x%lx, periods:%d\n",
			__FUNCTION__, runtime->channels,
			(unsigned long)frames_to_bytes(runtime, runtime->period_size),
			runtime->periods);

	ret = tmpa9xx_i2s_config_tx_dma(chip->i2s, runtime->dma_area, runtime->dma_addr,
			runtime->periods, fragsize_bytes, word_len);

	return ret;
}

static int snd_wm8974_capture_prepare(struct snd_pcm_substream *substream)
{
	struct snd_wm8974 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
        int word_len = 4;
	int ret;

	int fragsize_bytes = frames_to_bytes(runtime, runtime->period_size);

	wm_printd(KERN_ERR, "Record sample rate = %d.\n",runtime->rate);

	/* set requested samplerate */
	snd_wm8974_set_samplerate(chip->i2c_client, runtime->rate);

	if (substream != chip->tx_substream)
		return -EINVAL;

	snd_printd(KERN_INFO "%s channels:%d, period_bytes:0x%lx, periods:%d\n",
			__FUNCTION__, runtime->channels,
			(unsigned long)frames_to_bytes(runtime, runtime->period_size),
			runtime->periods);

	ret = tmpa9xx_i2s_config_rx_dma(chip->i2s, runtime->dma_area, runtime->dma_addr,
			runtime->periods, fragsize_bytes, word_len);

	return ret;
}

static int snd_wm8974_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_wm8974 *chip = snd_pcm_substream_chip(substream);
	int ret;

	snd_printk_marker();

	spin_lock(&chip->wm8974_lock);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			wm_printd(KERN_ERR, "  SNDRV_PCM_TRIGGER_START\n");
			ret = tmpa9xx_i2s_tx_start(chip->i2s);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			wm_printd(KERN_ERR, "  SNDRV_PCM_TRIGGER_STOP\n");
			ret = tmpa9xx_i2s_tx_stop(chip->i2s);
			break;
		default:
			spin_unlock(&chip->wm8974_lock);
			return -EINVAL;
	}
	spin_unlock(&chip->wm8974_lock);

	snd_printd(KERN_INFO "playback cmd:%s. ret=%d\n", cmd?"start":"stop", ret);

	return 0;
}

static int snd_wm8974_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_wm8974 *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker();

	spin_lock(&chip->wm8974_lock);

	if (substream != chip->rx_substream)
		return -EINVAL;

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			wm_printd(KERN_ERR, "  SNDRV_PCM_TRIGGER_START\n");
			tmpa9xx_i2s_rx_start(chip->i2s);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			wm_printd(KERN_ERR, "  SNDRV_PCM_TRIGGER_STOP\n");
			tmpa9xx_i2s_rx_stop(chip->i2s);
			break;
		default:
			spin_unlock(&chip->wm8974_lock);
			return -EINVAL;
			break;
	}
	spin_unlock(&chip->wm8974_lock);

	snd_printd(KERN_ERR"capture cmd:%s\n", cmd ? "start" : "stop");

	return 0;
}

static snd_pcm_uframes_t snd_wm8974_playback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_wm8974 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int offset;

	offset = tmpa9xx_i2s_curr_offset_tx(chip->i2s);

	offset = bytes_to_frames(runtime, offset);
	if (offset >= runtime->buffer_size)
		offset = 0;

	return offset;
}

static snd_pcm_uframes_t snd_wm8974_capture_pointer(struct snd_pcm_substream *substream)
{
	struct snd_wm8974 *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int offset;

	offset = tmpa9xx_i2s_curr_offset_rx(chip->i2s);

	offset = bytes_to_frames(runtime, offset);

	wm_printd(KERN_ERR, "offset=0x%02x\n",offset);		/* wym */
	wm_printd(KERN_ERR, "handler srcaddr = 0x%02x\n", DMA_SRC_ADDR(2));
	wm_printd(KERN_ERR, "handler dest addr = 0x%02x\n", DMA_DEST_ADDR(2));
	wm_printd(KERN_ERR, "handler lli addr = 0x%02x\n", DMA_LLI(2));
	wm_printd(KERN_ERR, "handler control = 0x%02x\n", DMA_CONTROL(2));

	if (offset >= runtime->buffer_size)
		offset = 0;

	return offset;
}

/* pcm method tables */
static struct snd_pcm_ops snd_wm8974_playback_ops = {
	.open      = snd_wm8974_playback_open,
	.close     = snd_wm8974_playback_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.hw_params = snd_wm8974_hw_params,
	.hw_free   = snd_wm8974_hw_free,
	.prepare   = snd_wm8974_playback_prepare,
	.trigger   = snd_wm8974_playback_trigger,
	.pointer   = snd_wm8974_playback_pointer,
};

static struct snd_pcm_ops snd_wm8974_capture_ops = {
	.open  = snd_wm8974_capture_open,
	.close = snd_wm8974_capture_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_wm8974_hw_params,
	.hw_free   = snd_wm8974_hw_free,
	.prepare   = snd_wm8974_capture_prepare,
	.trigger   = snd_wm8974_capture_trigger,
	.pointer   = snd_wm8974_capture_pointer,
};

/* card and device */
static int snd_wm8974_stop(struct snd_wm8974 *chip)
{
	snd_printk_marker();

	return 0;
}

static int snd_wm8974_dev_free(struct snd_device *device)
{
	struct snd_wm8974 *chip = (struct snd_wm8974 *)device->device_data;

	snd_printk_marker();

	return snd_wm8974_stop(chip);
}

static struct snd_device_ops snd_wm8974_ops = {
	.dev_free = snd_wm8974_dev_free,
};

static void snd_wm8974_dma_rx(void *data)
{
	struct snd_wm8974 *wm8974 = data;

        snd_printk_marker();

	if (wm8974->rx_substream) {
		snd_pcm_period_elapsed(wm8974->rx_substream);

		wm_printd(KERN_ERR, "WM8974->rx_substream=0x%02x\n", (unsigned int)wm8974->rx_substream);
		wm_printd(KERN_ERR, "handler srcaddr = 0x%02x\n", DMA_SRC_ADDR(2));
	       	wm_printd(KERN_ERR, "handler dest addr = 0x%02x\n", DMA_DEST_ADDR(2));
		wm_printd(KERN_ERR, "handler lli addr = 0x%02x\n", DMA_LLI(2));
	       	wm_printd(KERN_ERR, "handler control = 0x%02x\n", DMA_CONTROL(2));
	}
}

static void snd_wm8974_dma_tx(void *data)
{
	struct snd_wm8974 *wm8974 = data;

	snd_printk_marker();

	if (wm8974->tx_substream) {
		snd_pcm_period_elapsed(wm8974->tx_substream);

                wm_printd(KERN_ERR, "handler srcaddr = 0x%02x\n", DMA_SRC_ADDR(1));
                wm_printd(KERN_ERR, "handler dest addr = 0x%02x\n", DMA_DEST_ADDR(1));
                wm_printd(KERN_ERR, "handler lli addr = 0x%02x\n", DMA_LLI(1));
                wm_printd(KERN_ERR, "handler control = 0x%02x\n", DMA_CONTROL(1));
	}
}

static void snd_wm8974_i2s_err(void *data)
{
	struct snd_wm8974 *wm8974 = data;
	dev_err(&wm8974->i2c_client->dev, "I2S error\n");
}

static int __devinit snd_wm8974_pcm(struct snd_wm8974 *wm8974)
{
	struct snd_pcm *pcm;
	int ret;

	snd_printk_marker();

	/* 1 playback and 1 capture substream, of 2-8 channels each */
	ret = snd_pcm_new(wm8974->card, PCM_NAME, 0, 1, 1, &pcm);
	if (ret < 0) {
		dev_err(&wm8974->i2c_client->dev, "snd_pcm_new() failed\n");
		return ret;
	}

	/*
	 * this sets up our initial buffers and sets the dma_type to isa.
	 * isa works but I'm not sure why (or if) it's the right choice
	 * this may be too large, trying it for now
	 */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
			snd_dma_isa_data(), WM8974_BUF_SZ, WM8974_BUF_SZ);

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_wm8974_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_wm8974_capture_ops);
	wm8974->pcm = pcm;
	pcm->info_flags = 0;

	strcpy(pcm->name, PCM_NAME);
	pcm->private_data = wm8974;

	return 0;
}

static int wm8974_i2c_probe(struct i2c_client *i2c_client, const struct i2c_device_id *iid)
{
	struct snd_wm8974 *wm8974;
	struct snd_card *card;
	char *id = "ID string for TMPA9XX + wm8974 soundcard.";
	int ret = 0;

	init_wm8974_i2c(i2c_client);

	snd_printk_marker();

	snd_card_create(-1, id, THIS_MODULE, sizeof(struct snd_wm8974), &card);
	if (!card) {
		dev_err(&i2c_client->dev, "snd_card_create() failed\n");
		return -ENOMEM;
	}

	strcpy(card->driver, DRIVER_NAME);
	strcpy(card->shortname, CHIP_NAME);
	sprintf(card->longname, "%s at I2S rx/tx dma %d/%d err irq %d",
		  card->shortname,
		  I2S_DMA_RX, I2S_DMA_TX, I2S_IRQ_ERR);

	wm8974 = card->private_data;

	wm8974->card = card;
	wm8974->i2c_client = i2c_client;

	wm8974->i2s = tmpa9xx_i2s_init(I2S_DMA_RX, snd_wm8974_dma_rx, I2S_DMA_TX, snd_wm8974_dma_tx,
			    I2S_IRQ_ERR, snd_wm8974_i2s_err, wm8974);
	if (!wm8974->i2s) {
		dev_err(&i2c_client->dev, "tmpa9xx_i2s_init() failed\n");
		ret = -ENODEV;
		goto err0;
	}

	ret = snd_device_new(card, SNDRV_DEV_LOWLEVEL, wm8974, &snd_wm8974_ops);
	if (ret) {
		dev_err(&i2c_client->dev, "snd_device_new() failed\n");
		ret = -ENODEV;
		goto err1;
	}

	ret = snd_wm8974_pcm(wm8974);
	if (ret) {
		dev_err(&i2c_client->dev, "snd_wm8974_pcm() failed\n");
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

	return 0;

err3:
err2:
err1:
	tmpa9xx_i2s_free(wm8974->i2s);
err0:
	snd_card_free(card);
	return ret;

	return 0;
}

static int wm8974_i2c_remove(struct i2c_client *i2c_client)
{
	struct snd_card *card = i2c_get_clientdata(i2c_client);
	struct snd_wm8974 *wm8974 = card->private_data;

	tmpa9xx_i2s_free(wm8974->i2s);

	snd_card_free(card);

	i2c_set_clientdata(i2c_client, NULL);

	return 0;
}
static struct i2c_device_id wm8974_id[] = {
	{"wm8974", 0},
	{}, /* mandatory */
};
MODULE_DEVICE_TABLE(i2c, wm8974_id);

static struct i2c_driver wm8974_i2c_driver = {
	.driver = {
		.name  = "wm8974",
		.owner = THIS_MODULE,
	},
	.id_table = wm8974_id,
	.probe    = wm8974_i2c_probe,
	.remove   = wm8974_i2c_remove,
};

static int __init wm8974_init(void)
{
	return i2c_add_driver(&wm8974_i2c_driver);
}
module_init(wm8974_init);

static void __exit wm8974_exit(void)
{
	i2c_del_driver(&wm8974_i2c_driver);
}
module_exit(wm8974_exit);

MODULE_AUTHOR("Thomas Haase <Thomas.Haase@web.de>");
MODULE_DESCRIPTION("wm8974 driver for TMPA900");
MODULE_LICENSE("GPL");