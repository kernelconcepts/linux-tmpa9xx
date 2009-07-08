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

#include <asm/arch/dma.h>
#include <asm/arch/irqs.h>
#include <asm/arch/tmpa910_regs.h>

#include "tmpa910_i2s.h"
#include "wm8976.h"

#define I2S_DMA_RX   I2S0
#define I2S_DMA_TX   I2S1
#define I2S_IRQ_ERR  I2S_INT



/***********/
/***********/
// Useful debug
//#define __DEBUG__
#include <linux/debug.h>

#define snd_printk_marker() NPRINTK("->\n");
#define i2s_printd(level, format, arg...) NPRINTK(format, arg)

#undef snd_printd
#define snd_printd NPRINTK
#undef CONFIG_SND_DEBUG_CURRPTR  /* causes output every frame! */
//#define CONFIG_SND_DEBUG_CURRPTR

#undef NOCONTROLS  /* define this to omit all the ALSA controls */

#define DRIVER_NAME	"WM8976-I2S"
#define CHIP_NAME	"Wolfson WM8976"
#define PCM_NAME	"WM8976_PCM"

/* Only one WM8976 soundcard is supported */
static struct platform_device *g_device = NULL;


/* Chip level */
#define WM8976_BUF_SZ 	0x10000  /* 64kb */
#define PCM_BUFFER_MAX	(WM8976_BUF_SZ / 2)

#define CHANNELS_OUTPUT	2
#define CHANNELS_INPUT	2
#define FRAGMENTS_MIN	2
#define FRAGMENTS_MAX	32

#define AUDIO_RATE_DEFAULT  44100


static unsigned char wm8976_for_samplerate[7][6]={
	{0x00,0x08,0x0C,0x93,0xE9,0x49}, 	//48000HZ
	{0x00,0x07,0x21,0x61,0x27,0x49}, 	//44100HZ
	{0x02,0x08,0x0C,0x93,0xE9,0x69}, 	//32000HZ	
	{0x04,0x07,0x21,0x61,0x27,0x89}, 	//22050HZ
	{0x05,0x07,0x21,0x61,0x27,0xa9}, 	//16000HZ
	{0x08,0x07,0x21,0x61,0x27,0xC9}, 	//12000HZ
	{0x0a,0x08,0x0C,0x93,0xE9,0xE9}, 	//8000HZ						
};
	

typedef struct snd_wm8976 wm8976_t;
typedef struct snd_pcm_substream snd_pcm_substream_t;
typedef struct snd_pcm_hardware snd_pcm_hardware_t;
typedef struct snd_pcm_hw_params snd_pcm_hw_params_t;
typedef struct snd_pcm_runtime snd_pcm_runtime_t;
typedef struct snd_pcm_ops snd_pcm_ops_t;

struct snd_wm8976 {

	struct snd_card    *card;
	struct tmpa910_i2s *i2s;
	spinlock_t    wm8976_lock;

	struct snd_pcm *pcm;

	int poll_reg;  /* index of the wm8976 register last queried */

	/* if non-null, current subtream running */
	snd_pcm_substream_t *rx_substream;
	/* if non-null, current subtream running */
	snd_pcm_substream_t *tx_substream;
};

/*======================================================*/
/*     WM8976 I2C INTERFACE DEFINE                      */
/*======================================================*/
#define I2C1SR_BUS_CHECK	0x00000020
#define I2C1SR_BUS_FREE		0x00000000

#define I2C1SR_ACK_CHECK	0x00000010
#define I2C1SR_ACK_ENABLE	0x00000000
static void snd_wm8976_set_samplerate(long rate);

static void i2c_bus_free_chk(void)
{
	u32 reg_data;
	do
	{
		reg_data = I2C1SR;
	}while( (reg_data & I2C1SR_BUS_CHECK) != I2C1SR_BUS_FREE );
}

static void i2c_ack_wait(void)
{
	u32 reg_data;
	do
	{
		reg_data = I2C1SR;
	}while( (reg_data & I2C1SR_ACK_CHECK) != I2C1SR_ACK_ENABLE );
}

static void i2c_packet_send(u8 sub_addr, u8 data)
{
	//printk("[0x%x] <- 0x%3x\n", sub_addr >> 1, data | ((sub_addr&1) << 8) );

	I2C1DBR = 0x34;
	I2C1CR2 = 0xf8;

	i2c_ack_wait();

	I2C1DBR = sub_addr;
	i2c_ack_wait();

	I2C1DBR = data;
	i2c_ack_wait();

	I2C1CR2 = 0xd8;
	i2c_bus_free_chk();
}

static void init_wm8976_i2c(void)
{
    unsigned long flags;

	local_irq_save(flags);

 //   GPIOMFR1 = 0x0F;           // Config IIS Function Port: I2S1DATO
    I2SCOMMON = 0x19;           // IISSCLK = Fosch(X1),       Set SCK/WS/CLKO of Tx and Rx as Common
    I2STMCON = 0x04;           // I2SMCLK = Fosch/4 = 11.2896MHz
                                                      
    I2SRMCON = 0x04;
		
    I2STCON = 0x00;         // IIS Standard Format
    I2STFCLR = 0x01;           // Clear FIFO
	I2SRMS = 0x00;           // Slave
	I2STMS = 0x00;           // Slave
/*
	GPIORDIR |= 0X02;
	GPIORFR1 &= 0Xfd;
	GPIORFR2 &= 0Xfd;
	GPIORDATA &= 0Xfd;
	GPIOCODE = 0xc0;
	GPIOCFR1 = 0xc0;
	GPIOCFR2 = 0x00;
	GPIOCIE = 0x00;

	GPIOFODE = 0xc0;
	GPIOFFR1 = 0xc0;
	GPIOFFR2 = 0x00;
	GPIOFIE = 0x00;
*/
	local_irq_restore(flags);	
	i2c_bus_free_chk();
	I2C1CR2 = 0xc8;
	I2C1PRS = 0x19;
	I2C1CR1 = 0x13;

    i2c_packet_send(0x00,0x00);//R0  0X000             
    i2c_packet_send(0x02,0x3d);//R1  0X02D  
//    i2c_packet_send(0x05,0x91);//R2  0X191 
    i2c_packet_send(0x05,0x95);//R2  0X195 
    i2c_packet_send(0x06,0x6F);//R3  0X00F
    i2c_packet_send(0x08,0x10);//R4  0X010
    i2c_packet_send(0x0a,0x00);//R5  0X000
//    i2c_packet_send(0x0d,0x49);//R6  0X14d
 
 
//    i2c_packet_send(0x0e,0x00);//R7  0X00A     //set sample rate 48khz
        
    i2c_packet_send(0x14, 0x80);    //R10 = 0x080        
    i2c_packet_send(0x17,0xff);//R11 0X1ff
    i2c_packet_send(0x19,0xff);//R12 0X1ff
    i2c_packet_send(0x1c,0x00);    //R14 = 0x1c01   
    i2c_packet_send(0x1F,0xff);//R15 0X1FF
    i2c_packet_send(0x30, 0x32);    //R24 = 0x032        
//    i2c_packet_send(0x48,0x07);//R36 0X017
//    i2c_packet_send(0x4A,0x21);//R37 0X012
//    i2c_packet_send(0x4C,0x61);//R38 0X011
//    i2c_packet_send(0x4E,0x27);//R39 0X096  //set pll 12.288Mhz
//    i2c_packet_send(0x58,0x00);//R44 0X000
    i2c_packet_send(0x5b,0x3f);//R45 0X000   
    i2c_packet_send(0x56,0x10);//R43 0X010  //add for WM8976 8 ohm speaker                              
    
    i2c_packet_send(0x5f,0x55);//R47 0X005

    i2c_packet_send(47<<1 | 1,0xff);//R47 0X1ff

    i2c_packet_send(0x62,0x02);//R49 0X002
    i2c_packet_send(0x64,0x01);//R50 0X001
    i2c_packet_send(0x66,0x01);//R51 0X001
    i2c_packet_send(0x69,0x3f);//R52 0X13f
    i2c_packet_send(0x6b,0x3f);//R53 0X13f
                                
}

static void release_wm8976_i2c(void)
{
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

static snd_pcm_hardware_t snd_wm8976_playback_hw = {
	.info = ( SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER ),
	.formats =      SNDRV_PCM_FMTBIT_S16_LE,
//	.rates =	    SNDRV_PCM_RATE_44100,
	.rates =            (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
				   SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |\
				   SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
				   SNDRV_PCM_RATE_KNOT),
	.rate_min =	    8000,
	.rate_max =	    48000,
	.channels_min =	    2,
	.channels_max =     2,
	.buffer_bytes_max = PCM_BUFFER_MAX,
	.period_bytes_min = 0x1000,     //4KB
	.period_bytes_max = 0x3000,     //8KB
	.periods_min =      FRAGMENTS_MIN,
	.periods_max =      FRAGMENTS_MAX,
};

static snd_pcm_hardware_t snd_wm8976_capture_hw = {
	.info = ( SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER ),
	.formats =          SNDRV_PCM_FMTBIT_S16_LE,
//	.rates =            SNDRV_PCM_RATE_44100,
	.rates =            (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
				   SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |\
				   SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
				   SNDRV_PCM_RATE_KNOT),
	.rate_min =         8000,
	.rate_max =         48000,
	.channels_min =     1,
	.channels_max =     2,
	.buffer_bytes_max = PCM_BUFFER_MAX,
	.period_bytes_min = 0x1000,     //4KB
	.period_bytes_max = 0x3000,     //8KB
	.periods_min =      FRAGMENTS_MIN,
	.periods_max =      FRAGMENTS_MAX,
/*
	.rate_min		= 44100,
	.rate_max		= 44100,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= 64*1024,
	.period_bytes_min	= 64,
	.period_bytes_max	= 4096,
	.periods_min		= 2,
	.periods_max		= 255,
*/	

};

static int snd_wm8976_playback_open(snd_pcm_substream_t *substream)
{
	wm8976_t *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker();
	chip->tx_substream = substream;
	substream->runtime->hw = snd_wm8976_playback_hw;

	return 0;
}

static int snd_wm8976_capture_open(snd_pcm_substream_t *substream)
{
	wm8976_t *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker();
	substream->runtime->hw = snd_wm8976_capture_hw;
	chip->rx_substream = substream;

	return 0;
}

static int snd_wm8976_playback_close(snd_pcm_substream_t *substream)
{
	wm8976_t *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker();
	chip->tx_substream = NULL;

	return 0;
}

static int snd_wm8976_capture_close(snd_pcm_substream_t *substream)
{
	wm8976_t *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker();
	chip->rx_substream = NULL;
	return 0;
}

//I2S in following
static int snd_wm8976_hw_params(snd_pcm_substream_t *substream,
					snd_pcm_hw_params_t *hwparams)
{
	snd_printk_marker();

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

static int snd_wm8976_hw_free(snd_pcm_substream_t * substream)
{
	snd_printk_marker();
	snd_pcm_lib_free_pages(substream);
	return 0;
}

static int snd_wm8976_playback_prepare(snd_pcm_substream_t *substream)
{
        int err = 0;
        int word_len = 4;

	wm8976_t *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;

	int fragsize_bytes = frames_to_bytes(runtime, runtime->period_size);
        //printk("Playback sample rate = %d.\n",runtime->rate);

	/* set requested samplerate */
	snd_wm8976_set_samplerate(runtime->rate);


	snd_assert((substream == chip->tx_substream), return -EINVAL);

	snd_printd(KERN_INFO "%s channels:%d, period_bytes:0x%lx, periods:%d\n",
			__FUNCTION__, runtime->channels,
			frames_to_bytes(runtime, runtime->period_size),
			runtime->periods);

	err = tmpa910_i2s_config_tx_dma(chip->i2s, runtime->dma_area, runtime->dma_addr,
			runtime->periods, fragsize_bytes, word_len);

	return err;
}

static int snd_wm8976_capture_prepare(snd_pcm_substream_t *substream)
{
        int err = 0;
        int word_len = 4;

	wm8976_t *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;

	int fragsize_bytes = frames_to_bytes(runtime, runtime->period_size);

        //printk("Record sample rate = %d.\n",runtime->rate);

	/* set requested samplerate */
	snd_wm8976_set_samplerate(runtime->rate);	

	snd_assert((substream == chip->tx_substream), return -EINVAL);

	snd_printd(KERN_INFO "%s channels:%d, period_bytes:0x%lx, periods:%d\n",
			__FUNCTION__, runtime->channels,
			frames_to_bytes(runtime, runtime->period_size),
			runtime->periods);

	err = tmpa910_i2s_config_rx_dma(chip->i2s, runtime->dma_area, runtime->dma_addr,
			runtime->periods, fragsize_bytes, word_len);

	return err;
}

static int snd_wm8976_playback_trigger(snd_pcm_substream_t *substream, int cmd)
{
	int ret;
	wm8976_t *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker();

	spin_lock(&chip->wm8976_lock);
	switch (cmd)
	{
	case SNDRV_PCM_TRIGGER_START:
	    //printk("  SNDRV_PCM_TRIGGER_START\n");
		ret = tmpa910_i2s_tx_start(chip->i2s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	    //printk("  SNDRV_PCM_TRIGGER_STOP\n");
	    ret = tmpa910_i2s_tx_stop(chip->i2s);
		break;
	default:
		ret = -1;
		spin_unlock(&chip->wm8976_lock);
		return -EINVAL;
	}
	spin_unlock(&chip->wm8976_lock);

	snd_printd(KERN_INFO"playback cmd:%s. ret=%d\n", cmd?"start":"stop", ret);

	return 0;
}

static int snd_wm8976_capture_trigger(snd_pcm_substream_t *substream, int cmd)
{
	wm8976_t *chip = snd_pcm_substream_chip(substream);

	snd_printk_marker();

	spin_lock(&chip->wm8976_lock);
	snd_assert(substream == chip->rx_substream, return -EINVAL);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		//printk("  SNDRV_PCM_TRIGGER_START\n");
		tmpa910_i2s_rx_start(chip->i2s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	    //printk("  SNDRV_PCM_TRIGGER_STOP\n");
	    tmpa910_i2s_rx_stop(chip->i2s);
		break;
	default:
		spin_unlock(&chip->wm8976_lock);
		return -EINVAL;
	}
	spin_unlock(&chip->wm8976_lock);

	//snd_printd(KERN_ERR"capture cmd:%s\n", cmd ? "start" : "stop");

	return 0;
}

static snd_pcm_uframes_t snd_wm8976_playback_pointer(snd_pcm_substream_t *substream)
{
	wm8976_t *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;
	unsigned int offset;

	offset = tmpa910_i2s_curr_offset_tx(chip->i2s);
	
	offset = bytes_to_frames(runtime, offset);
	if (offset >= runtime->buffer_size)
		offset = 0;
		
	return offset;
}

static snd_pcm_uframes_t snd_wm8976_capture_pointer(snd_pcm_substream_t *substream)
{
	wm8976_t *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;
	unsigned int offset;

	offset = tmpa910_i2s_curr_offset_rx(chip->i2s);
	
	offset = bytes_to_frames(runtime, offset);

//	printk("offset=0x%02x\n",offset);		//wym
//        printk("handler srcaddr = 0x%02x\n", DMA_SRC_ADDR(2));
//        printk("handler dest addr = 0x%02x\n", DMA_DEST_ADDR(2));
//        printk("handler lli addr = 0x%02x\n", DMA_LLI(2));
//        printk("handler control = 0x%02x\n", DMA_CONTROL(2));

//                printk("VICIRQSTATUS is 0x%02x.\n",VICIRQSTATUS);
//                printk("VICINTENABLE is 0x%02x.\n",VICINTENABLE);
	if (offset >= runtime->buffer_size)
		offset = 0;
		
	return offset;
}

/* pcm method tables */
static snd_pcm_ops_t snd_wm8976_playback_ops = {
	.open      = snd_wm8976_playback_open,
	.close     = snd_wm8976_playback_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.hw_params = snd_wm8976_hw_params,
	.hw_free   = snd_wm8976_hw_free,
	.prepare   = snd_wm8976_playback_prepare,
	.trigger   = snd_wm8976_playback_trigger,
	.pointer   = snd_wm8976_playback_pointer,
};

static snd_pcm_ops_t snd_wm8976_capture_ops = {
	.open  = snd_wm8976_capture_open,
	.close = snd_wm8976_capture_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_wm8976_hw_params,
	.hw_free   = snd_wm8976_hw_free,
	.prepare   = snd_wm8976_capture_prepare,
	.trigger   = snd_wm8976_capture_trigger,
	.pointer   = snd_wm8976_capture_pointer,
};

/*************************************************************
 *      card and device
 *************************************************************/
static int snd_wm8976_stop(struct snd_wm8976 *chip)
{
	snd_printk_marker();

	return 0;
}

static int snd_wm8976_dev_free(struct snd_device *device)
{
	struct snd_wm8976 *chip = (wm8976_t *)device->device_data;

	snd_printk_marker();

	return snd_wm8976_stop(chip);
}

static struct snd_device_ops snd_wm8976_ops = {
	.dev_free = snd_wm8976_dev_free,
};

static int snd_bf53x_wm8976_reset(wm8976_t *chip)
{
	return 0;
}

static int snd_wm8976_configure(wm8976_t *chip)
{
	int err = 0;
	struct tmpa910_i2s *i2s= chip->i2s;

	snd_printk_marker();

	snd_bf53x_wm8976_reset(chip);

	err = err || tmpa910_i2s_config_tx(i2s);
	
	if (err)
	{
		snd_printk(KERN_ERR "Unable to set i2s configuration\n");
	}

	return err;
}

static void snd_wm8976_dma_rx(void *data)
{
	struct snd_wm8976 *wm8976 = data;
   	
        snd_printk_marker();


	if (wm8976->rx_substream) {
		snd_pcm_period_elapsed(wm8976->rx_substream);
	        
/*		printk("WM8976->rx_substream=0x%02x\n",wm8976->rx_substream);               //wym
		printk("handler srcaddr = 0x%02x\n", DMA_SRC_ADDR(2));
        	printk("handler dest addr = 0x%02x\n", DMA_DEST_ADDR(2));
		printk("handler lli addr = 0x%02x\n", DMA_LLI(2));
        	printk("handler control = 0x%02x\n", DMA_CONTROL(2));
		printk("VICIRQSTATUS is 0x%02x.\n",VICIRQSTATUS);
		printk("VICINTENABLE is 0x%02x.\n",VICINTENABLE);
*/	}
//	VICINTENABLE |= 0x20000;
}

static void snd_wm8976_dma_tx(void *data)
{
	struct snd_wm8976 *wm8976 = data;
	
	snd_printk_marker();

	if (wm8976->tx_substream) {
		snd_pcm_period_elapsed(wm8976->tx_substream);
/*
                printk("handler srcaddr = 0x%02x\n", DMA_SRC_ADDR(1));
                printk("handler dest addr = 0x%02x\n", DMA_DEST_ADDR(1));
                printk("handler lli addr = 0x%02x\n", DMA_LLI(1));
                printk("handler control = 0x%02x\n", DMA_CONTROL(1));
                printk("VICIRQSTATUS is 0x%02x.\n",VICIRQSTATUS);
                printk("VICINTENABLE is 0x%02x.\n",VICINTENABLE);
*/	}
}

static void snd_wm8976_i2s_err(void *data)
{
	printk(KERN_ERR DRIVER_NAME ":%s: err happened on i2s\n", __FUNCTION__);
}

static int __devinit snd_wm8976_pcm(struct snd_wm8976 *wm8976)
{
	struct snd_pcm *pcm;
	int err = 0;

	snd_printk_marker();

	/* 1 playback and 1 capture substream, of 2-8 channels each */
	if((err = snd_pcm_new(wm8976->card, PCM_NAME, 0, 1, 1, &pcm)) < 0)
	{
		return err;
	}

	/*
	 * this sets up our initial buffers and sets the dma_type to isa.
	 * isa works but I'm not sure why (or if) it's the right choice
	 * this may be too large, trying it for now
	 */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
			snd_dma_isa_data(), WM8976_BUF_SZ, WM8976_BUF_SZ);


	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_wm8976_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_wm8976_capture_ops);
	wm8976->pcm = pcm;
	pcm->info_flags = 0;

	strcpy(pcm->name, PCM_NAME);
	pcm->private_data = wm8976;

	return 0;
}

static int __devinit snd_wm8976_probe(struct platform_device *pdev)
{
	int err = 0;
	struct snd_card *card;
	struct snd_wm8976 *wm8976;
	struct tmpa910_i2s *i2s;
	char * id = "ID string for TMPA910 + WM8976 soundcard.";
	
	snd_printk_marker();
	
	if (g_device != NULL)
		return -ENOENT;

	card = snd_card_new(-1, id, THIS_MODULE, sizeof(struct snd_wm8976));
	if (card == NULL) {
		snd_printdd(KERN_DEBUG "%s: snd_card_new() failed\n", __FUNCTION__);
		return -ENOMEM;
	}

	wm8976 = card->private_data;
	wm8976->card = card;
	//wm8976->samplerate = AUDIO_RATE_DEFAULT;
	if ((i2s = tmpa910_i2s_init(I2S_DMA_RX, snd_wm8976_dma_rx, I2S_DMA_TX, snd_wm8976_dma_tx,
			    I2S_IRQ_ERR, snd_wm8976_i2s_err, wm8976)) == NULL)
	{
		printk(KERN_ERR DRIVER_NAME ": Failed to find device on i2s\n");
		err = -ENODEV;
		goto __i2s_err;
	}

	wm8976->i2s = i2s;

	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, wm8976, &snd_wm8976_ops);
	if (err)
	{
		goto __nodev;
	}

	if ((err = snd_wm8976_pcm(wm8976)) < 0)
	{
		goto __nodev;
	}

	if ((err = snd_wm8976_configure(wm8976)) < 0)
	{
	    printk("snd_wm8976_configure faild.\n");
		goto __nodev;
	}
	
	strcpy(card->driver, DRIVER_NAME);
	strcpy(card->shortname, CHIP_NAME);
	sprintf(card->longname, "%s at I2S rx/tx dma %d/%d err irq %d",
		  card->shortname,
		  I2S_DMA_RX, I2S_DMA_TX, I2S_IRQ_ERR);
		  
	snd_card_set_dev(card, &pdev->dev);

	if ((err = snd_card_register(card)) < 0)
	{
	    printk("snd_card_register faild.\n");
		goto __nodev;
	}

    //printk("platform_set_drvdata card=%p\n", card);
	platform_set_drvdata(pdev, card);

	g_device = pdev;

	return 0;

__nodev:
	tmpa910_i2s_free(i2s);
__i2s_err:
	snd_card_free(card);
	return err;
}

static int __devexit snd_wm8976_remove(struct platform_device *pdev)
{
	struct snd_card *card;
	struct snd_wm8976 *wm8976;
	
	card = platform_get_drvdata(pdev);
	wm8976 = card->private_data;

	snd_wm8976_stop(wm8976);
	tmpa910_i2s_free(wm8976->i2s);

	snd_card_free(card);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#define TMPA910_WM8976_DRIVER	"tmpa910_wm8976"
static struct platform_driver snd_wm8976_driver = {
	.probe		= snd_wm8976_probe,
	.remove		= __devexit_p(snd_wm8976_remove),
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __init snd_wm8976_init(void)
{
	int err;

	init_wm8976_i2c();
	enable_audio_sysclk();

	if ((err = platform_driver_register(&snd_wm8976_driver)) < 0)
	{
		NPRINTK("platform_driver_register failed. ret=%d\n", platform_driver_register);
		return err;
	}

	return 0;
}

static void __exit snd_wm8976_exit(void)
{
	if (g_device)
	{
		platform_device_unregister(g_device);
		platform_driver_unregister(&snd_wm8976_driver);
		release_wm8976_i2c();
		disable_audio_sysclk();
	}

	g_device = NULL;
}

	
static void snd_wm8976_set_samplerate(long rate)
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

  i2c_packet_send(0x0d, wm8976_for_samplerate[rate][5]);    //R6
  i2c_packet_send(0x0e, wm8976_for_samplerate[rate][0]);    //R7
  i2c_packet_send(0x48, wm8976_for_samplerate[rate][1]);    //R36 
  i2c_packet_send(0x4a, wm8976_for_samplerate[rate][2]);    //R37 
  i2c_packet_send(0x4d, wm8976_for_samplerate[rate][3]);    //R38
  i2c_packet_send(0x4e, wm8976_for_samplerate[rate][4]);    //R39	
}	


MODULE_AUTHOR("TOSHIBA <tmpa910@toshiba.com>");
MODULE_DESCRIPTION("TMPA910/WM8976");
MODULE_LICENSE("GPL");

module_init(snd_wm8976_init);
module_exit(snd_wm8976_exit);
