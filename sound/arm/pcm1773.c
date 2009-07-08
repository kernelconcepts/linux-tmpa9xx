#include <linux/init.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/delay.h>

#include <sound/core.h>
#include <sound/info.h>
#include <sound/control.h>
#include <sound/pcm.h>
#define SNDRV_GET_ID
#include <sound/initval.h>

#include <mach/dma.h>
#include <mach/irqs.h>
#include <mach/tmpa910_regs.h>

#include "tmpa910_i2s.h"

#define I2S_DMA_RX   I2S0
#define I2S_DMA_TX   I2S1
#define I2S_IRQ_ERR  I2S_INT


/***********/
/***********/

#undef CONFIG_SND_DEBUG_CURRPTR  /* causes output every frame! */
//#define CONFIG_SND_DEBUG_CURRPTR

#undef NOCONTROLS  /* define this to omit all the ALSA controls */

#define DRIVER_NAME	"PCM1773-I2S"
#define CHIP_NAME	"BB PCM1773"
#define PCM_NAME	"PCM1773_PCM"

/* Only one PCM1773 soundcard is supported */
static struct platform_device *g_device = NULL;


/* Chip level */
#define PCM1773_BUF_SZ 	0x10000  /* 64kb */
#define PCM_BUFFER_MAX	(PCM1773_BUF_SZ / 2)

#define CHANNELS_OUTPUT	2
#define FRAGMENTS_MIN	2
#define FRAGMENTS_MAX	32

#define AUDIO_RATE_DEFAULT  32000

typedef struct snd_pcm1773 pcm1773_t;
typedef struct snd_pcm_substream snd_pcm_substream_t;
typedef struct snd_pcm_hardware snd_pcm_hardware_t;
typedef struct snd_pcm_hw_params snd_pcm_hw_params_t;
typedef struct snd_pcm_runtime snd_pcm_runtime_t;
typedef struct snd_pcm_ops snd_pcm_ops_t;

struct snd_pcm1773 {

	struct snd_card    *card;
	struct tmpa910_i2s *i2s;
	spinlock_t    pcm1773_lock;

	struct snd_pcm *pcm;

	/* if non-null, current subtream running */
	snd_pcm_substream_t *tx_substream;
};

static void init_pcm1773(void)
{
    unsigned long flags;

	local_irq_save(flags);

	/* I2S Register Set */
	I2SCOMMON = 0x18;     // IISSCLK = Fosch(X1),       Set SCK/WS/CLKO of Tx and Rx as Common

	// The codec hardware officially supports
	// only 41.1 Khz freq but it works also
	// well with others. We use 31.25 Khz because
	// it is pretty close to 32Khz. The audiable resut is
	// ok
	I2STMCON  = 0x05;     // I2SMCLK = Fosch/2 = 12MHz
                                  // I2SSCLK = 12MHz/12 = 1000KHz
                                  // I2SWS = 1000KHz/32 = 31.25KHz

	I2SRMCON = 0x04;
	I2STCON  = 0x00;      // IIS Standard Format
	I2STFCLR = 0x01;      // Clear FIFO
	I2STMS   = 0x01;      // MasterTx

    local_irq_restore(flags);
}

/*======================================*/
/* AUDIO CLOCK INTERFACE                */
static void enable_audio_sysclk(void)
{
}

static void disable_audio_sysclk(void)
{
}


/*************************************************************
 *                pcm methods
 *************************************************************/

static snd_pcm_hardware_t snd_pcm1773_playback_hw = {
	.info = ( SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER ),
	.formats =      SNDRV_PCM_FMTBIT_S16_LE,
	.rates =	    SNDRV_PCM_RATE_32000,
	.rate_min =	    AUDIO_RATE_DEFAULT,
	.rate_max =	    AUDIO_RATE_DEFAULT,
	.channels_min =	    2,
	.channels_max =     2,
	.buffer_bytes_max = PCM_BUFFER_MAX,
	.period_bytes_min = 0x1000,     //4KB
	.period_bytes_max = 0x1000,     //8KB
	.periods_min =      FRAGMENTS_MIN,
	.periods_max =      FRAGMENTS_MAX,
};

static int snd_pcm1773_playback_open(snd_pcm_substream_t *substream)
{
	pcm1773_t *chip = snd_pcm_substream_chip(substream);

	chip->tx_substream = substream;
	substream->runtime->hw = snd_pcm1773_playback_hw;

	return 0;
}

static int snd_pcm1773_playback_close(snd_pcm_substream_t *substream)
{
	pcm1773_t *chip = snd_pcm_substream_chip(substream);

	chip->tx_substream = NULL;

	return 0;
}

//I2S in following
static int snd_pcm1773_hw_params(snd_pcm_substream_t *substream,
					snd_pcm_hw_params_t *hwparams)
{
	/*
	*  Allocate all available memory for our DMA buffer.
	*  Necessary because we get a 4x increase in bytes for the 2 channel mode.
	*  (we lie to the ALSA midlayer through the hwparams data)
	*  We're relying on the driver not supporting full duplex mode
	*  to allow us to grab all the memory.
	*/
	//printk("params_buffer_bytes return %d\n", params_buffer_bytes(hwparams));
	if( snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hwparams)) < 0 )
		return -ENOMEM;
	return 0;
}

static int snd_pcm1773_hw_free(snd_pcm_substream_t * substream)
{
	snd_pcm_lib_free_pages(substream);
	return 0;
}

static int snd_pcm1773_playback_prepare(snd_pcm_substream_t *substream)
{

	pcm1773_t *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;

	int fragsize_bytes = frames_to_bytes(runtime, runtime->period_size);

	int err = 0;
	int word_len = 4;

//	snd_assert((substream == chip->tx_substream), return -EINVAL);

	snd_printd(KERN_INFO "channels:%d, period_bytes:0x%lx, periods:%d\n",
			 runtime->channels,
			frames_to_bytes(runtime, runtime->period_size),
			runtime->periods);

	err = tmpa910_i2s_config_tx_dma(chip->i2s, runtime->dma_area, runtime->dma_addr,
			runtime->periods, fragsize_bytes, word_len);

	return err;
}

static int snd_pcm1773_playback_trigger(snd_pcm_substream_t *substream, int cmd)
{
	pcm1773_t *chip = snd_pcm_substream_chip(substream);

	spin_lock(&chip->pcm1773_lock);
	switch (cmd)
	{
	case SNDRV_PCM_TRIGGER_START:
	    //printk("  SNDRV_PCM_TRIGGER_START\n");
		tmpa910_i2s_tx_start(chip->i2s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	    tmpa910_i2s_tx_stop(chip->i2s);
	    //printk("  SNDRV_PCM_TRIGGER_STOP\n");
		break;
	default:
		spin_unlock(&chip->pcm1773_lock);
		return -EINVAL;
	}
	spin_unlock(&chip->pcm1773_lock);

	snd_printd(KERN_INFO "playback cmd:%s\n", cmd?"start":"stop");

	return 0;
}

static snd_pcm_uframes_t snd_pcm1773_playback_pointer(snd_pcm_substream_t *substream)
{
	pcm1773_t *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;
	unsigned int offset;

	// snd_printk_marker();

	offset = tmpa910_i2s_curr_offset_tx(chip->i2s);
	
	offset = bytes_to_frames(runtime, offset);
	if (offset >= runtime->buffer_size)
		offset = 0;
		
	// printk("runtime: 0x%p, offset:0x%x\n", runtime, offset);
	// tmpa910_i2s_hw_dump();
	// tmpa910_i2s_dma_dump();

	return offset;
}

/* pcm method tables */
static snd_pcm_ops_t snd_pcm1773_playback_ops = {
	.open      = snd_pcm1773_playback_open,
	.close     = snd_pcm1773_playback_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.hw_params = snd_pcm1773_hw_params,
	.hw_free   = snd_pcm1773_hw_free,
	.prepare   = snd_pcm1773_playback_prepare,
	.trigger   = snd_pcm1773_playback_trigger,
	.pointer   = snd_pcm1773_playback_pointer,
};

/*************************************************************
 *      card and device
 *************************************************************/
static int snd_pcm1773_stop(struct snd_pcm1773 *chip)
{
	return 0;
}

static int snd_pcm1773_dev_free(struct snd_device *device)
{
	struct snd_pcm1773 *chip = (pcm1773_t *)device->device_data;

	return snd_pcm1773_stop(chip);
}

static struct snd_device_ops snd_pcm1773_ops = {
	.dev_free = snd_pcm1773_dev_free,
};

static int snd_pcm1773_configure(pcm1773_t *chip)
{
	int err = 0;
	struct tmpa910_i2s *i2s= chip->i2s;

	err = err | tmpa910_i2s_config_tx(i2s);
	
	if (err)
	{
		snd_printk(KERN_ERR "Unable to set i2s configuration\n");
	}

	return err;
}

static void snd_pcm1773_dma_tx(void *data)
{
	struct snd_pcm1773 *pcm1773 = data;
	unsigned tmp;

	 // tmpa910_i2s_hw_dump();
	 // tmpa910_i2s_dma_dump();

	if (pcm1773->tx_substream)
	{
		snd_pcm_period_elapsed(pcm1773->tx_substream);

		tmp = I2SINT;
		// printk("I2SINT: 0x%x\n", tmp);
		//printk("DMACSoftBReq: 0x%03x\n", DMACSoftBReq);
		if (tmp)
		{
			I2SINT = tmp;
			I2STFCLR = 0x01;      // Clear FIFO
			//printk("I2SINT: 0x%x\n", tmp);
		}	
	}
	//printk("DMACSoftBReq: 0x%03x\n", DMACSoftBReq);
}

static void snd_pcm1773_i2s_err(void *data)
{
	printk(KERN_ERR DRIVER_NAME ":%s: err happened on i2s\n", __FUNCTION__);
}

static int __devinit snd_pcm1773_pcm(struct snd_pcm1773 *pcm1773)
{
	struct snd_pcm *pcm;
	int err = 0;

	/* 1 playback substream, of 2-8 channels each */
	if((err = snd_pcm_new(pcm1773->card, PCM_NAME, 0, 1, 0, &pcm)) < 0)
	{
		return err;
	}

	/*
	 * this sets up our initial buffers and sets the dma_type to isa.
	 * isa works but I'm not sure why (or if) it's the right choice
	 * this may be too large, trying it for now
	 */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
			snd_dma_isa_data(), PCM1773_BUF_SZ, 0);


	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_pcm1773_playback_ops);
	pcm1773->pcm = pcm;
	pcm->info_flags = 0;

	strcpy(pcm->name, PCM_NAME);
	pcm->private_data = pcm1773;

	return 0;
}

static int __devinit snd_pcm1773_probe(struct platform_device *pdev)
{
	int err = 0;
	struct snd_card *card = NULL;
	struct snd_pcm1773 *pcm1773;
	struct tmpa910_i2s *i2s;
	char * id = "TMPA910 + PCM1773";
	
	
	if (g_device != NULL)
		return -ENOENT;

	snd_card_create(-1, id, THIS_MODULE, sizeof(struct snd_pcm1773), &card);
	if (card == NULL) {
		snd_printdd(KERN_DEBUG "%s: snd_card_new() failed\n", __FUNCTION__);
		printk("snd_card_new faild.\n");
		return -ENOMEM;
	}

	pcm1773 = card->private_data;
	pcm1773->card = card;
	if ((i2s = tmpa910_i2s_init(0, NULL, I2S_DMA_TX, snd_pcm1773_dma_tx,
			    I2S_IRQ_ERR, snd_pcm1773_i2s_err, pcm1773)) == NULL)
	{
		printk(KERN_ERR DRIVER_NAME ": Failed to find device on i2s\n");
		err = -ENODEV;
		goto __i2s_err;
	}

	pcm1773->i2s = i2s;

	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, pcm1773, &snd_pcm1773_ops);
	if (err)
	{
		printk(KERN_ERR "snd_device_new faild.\n");
		goto __nodev;
	}

	if ((err = snd_pcm1773_pcm(pcm1773)) < 0)
	{
		printk(KERN_ERR "snd_pcm1773_pcm faild.\n");
		goto __nodev;
	}

	if ((err = snd_pcm1773_configure(pcm1773)) < 0)
	{
	    printk(KERN_ERR "snd_pcm1773_configure faild.\n");
		goto __nodev;
	}
	
	strcpy(card->driver, DRIVER_NAME);
	strcpy(card->shortname, CHIP_NAME);
	sprintf(card->longname, "%s at I2S tx dma %d err irq %d",
		  card->shortname,
		  I2S_DMA_TX, I2S_IRQ_ERR);
		  
	snd_card_set_dev(card, &pdev->dev);

	if ((err = snd_card_register(card)) < 0)
	{
		printk(KERN_ERR "snd_card_register faild.\n");
		goto __nodev;
	}

	platform_set_drvdata(pdev, card);

	return 0;

__nodev:
	tmpa910_i2s_free(i2s);
__i2s_err:
	snd_card_free(card);

	return err;
}

static int __devexit snd_pcm1773_remove(struct platform_device *pdev)
{
	struct snd_card *card;
	struct snd_pcm1773 *pcm1773;

	card = platform_get_drvdata(pdev);
	pcm1773 = card->private_data;

	snd_pcm1773_stop(pcm1773);
	tmpa910_i2s_free(pcm1773->i2s);

	snd_card_free(card);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#define TMPA910_PCM1773_DRIVER	"tmpa910_pcm1773"
static struct platform_driver snd_pcm1773_driver = {
	.probe		= snd_pcm1773_probe,
	.remove		= __devexit_p(snd_pcm1773_remove),
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __init snd_pcm1773_init(void)
{
	int err;

	init_pcm1773();
	enable_audio_sysclk();

	if ((err = platform_driver_register(&snd_pcm1773_driver)) < 0)	
	{
		printk(KERN_ERR "platform_driver_register failed. ret=%d\n", platform_driver_register);
		return err;
	}

	g_device = platform_device_register_simple(DRIVER_NAME,
							 0, NULL, 0);

	if (g_device==NULL)
	{
		printk(KERN_ERR "platform_device_register_simple failed\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit snd_pcm1773_exit(void)
{
	if (g_device)
	{
		platform_device_unregister(g_device);
		platform_driver_unregister(&snd_pcm1773_driver);
		disable_audio_sysclk();
	}
}

MODULE_AUTHOR("OPEN-engineering.de <info@open-engineering.de>");
MODULE_DESCRIPTION("TMPA910/PCM1773");
MODULE_LICENSE("GPL");

module_init(snd_pcm1773_init);
module_exit(snd_pcm1773_exit);

