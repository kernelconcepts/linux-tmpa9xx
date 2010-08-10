/*
 *  arch/arm/mach-tmpa910/tonga.c 
 *
 * Copyright (C) 2010 Florian Boor <florian.boor@kernelconcepts.de>
 * Based on topasa900.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 * Glyn ARM9 / Tonga machine definition, multi purpose Toshiba TMPA900 based SoM 
 *
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/mmc_spi.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/pwm_backlight.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/irq.h>

#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <video/tmpa910_fb.h>
#include <mach/gpio.h>
#include <linux/mmc/host.h>
#include <asm/mach/arch.h>
#include <mach/hardware.h>
#include <mach/ts.h>
#include <mach/tmpa910_regs.h>
#include <linux/smsc911x.h>


#ifdef CONFIG_SPI_TMPA910
#include <linux/spi/spi.h>
#endif

#include "topas910.h"

/* I/O Mapping related, might want to be moved to a CPU specific file */

static struct map_desc tmpa900_io_desc[] __initdata = {
	{
		.virtual = TMPA910_IO_VIRT_BASE,
		.pfn = __phys_to_pfn(TMPA910_IO_PHYS_BASE),
		.length	= TMPA910_IO_SIZE,
		.type = MT_DEVICE,
	}
};


void __init tonga_map_io(void)
{
	iotable_init(tmpa900_io_desc, ARRAY_SIZE(tmpa900_io_desc));
}


static u64 topas910_dmamask = 0xffffffffUL;

/* 
 * Ethernet 
 */ 
#if defined CONFIG_NET_ETHERNET || defined CONFIG_NET_ETHERNET_MODULE
static struct resource smsc911x_resources[] = {
        [0] = {
                .start  = 0x60000000,
                .end    = 0x60000003,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                .start  = 0x60001000,
                .end    = 0x60001003,
                .flags  = IORESOURCE_MEM,
        },
        [2] = {
                .start  = TONGA_INT_SMSC911X,
                .end    = TONGA_INT_SMSC911X,
                .flags  = IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
        },
};

static struct smsc911x_platform_config tonga_smsc911x_pdata = {
	.irq_polarity  = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type      = SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags         = SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS,
	.phy_interface = PHY_INTERFACE_MODE_MII,
        .mac           = "deadaa",
};


static struct platform_device tonga_smsc911x_device = {
        .name           = "smsc911x",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(smsc911x_resources),
        .resource       = smsc911x_resources,
	.dev = {
		.platform_data = &tonga_smsc911x_pdata,
	},
};
#endif

/*
 * Serial UARTs
 */ 
#if defined CONFIG_SERIAL_TMPA910 || defined CONFIG_SERIAL_TMPA910_MODULE
#define CONFIG_UART0	/* enable UART0 */
#define CONFIG_UART1	/* enable UART1 */

#ifdef CONFIG_UART0
static struct resource tmpa910_resource_uart0[] = {
	{
		.start	= 0xf2000000,
		.end	= 0xf2000000 + 0x100,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 10,
		.end	= 10,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};

struct platform_device tmpa910_device_uart0 = {
	.name		= "tmpa910-uart",
	.id		= 0,
	.resource	= tmpa910_resource_uart0,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_uart0),
};
#endif

#ifdef CONFIG_UART1
static struct resource tmpa910_resource_uart1[] = {
	{
		.start	= 0xf2001000,
		.end	= 0xf2001000 + 0x100,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 11,
		.end	= 11,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};

struct platform_device tmpa910_device_uart1 = {
	.name		= "tmpa910-uart",
	.id		= 1,
	.resource	= tmpa910_resource_uart1,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_uart1),
};
#endif

#ifdef CONFIG_UART2
static struct resource tmpa910_resource_uart2[] = {
	{
		.start	= 0xf2004000,
		.end	= 0xf2004000 + 0x100,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 9,
		.end	= 9,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};

struct platform_device tmpa910_device_uart2 = {
	.name		= "tmpa910-uart",
	.id		= 2,
	.resource	= tmpa910_resource_uart2,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_uart2),
};
#endif
#endif

/*
 * I2C
 */ 
#if defined CONFIG_I2C_TMPA910 || defined CONFIG_I2C_TMPA910_MODULE
static struct resource tmpa910_resource_i2c[] = {
	{
		.start	= I2C0_BASE,
		.end	= I2C0_BASE+0x1F,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= I2C1_BASE,
		.end	= I2C1_BASE+0x1F,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= INTR_VECT_I2C_CH0,
		.end	= INTR_VECT_I2C_CH0,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}, {
		.start	= INTR_VECT_I2C_CH1,
		.end	= INTR_VECT_I2C_CH1,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};

struct platform_device tmpa910_device_i2c = {
	.name		 = "tmpa910-i2c",
	.id = 0,
	.dev = {
		.platform_data = NULL,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource	= tmpa910_resource_i2c,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_i2c),
};
#endif

/*
 * SDHC
 */ 
#if defined CONFIG_MMC_TMPA910_SDHC || defined CONFIG_MMC_TMPA910_SDHC_MODULE
static struct resource tmpa910_resource_sdhc[] = {
{
		.start	= INTR_VECT_SDHC,
		.end	= INTR_VECT_SDHC,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	},
};

struct platform_device tmpa910_device_sdhc = {
	.name		= "tmpa910-sdhc",
	.id		= 0,
	.dev =
	{
		.coherent_dma_mask = 0xffffffff,
	},
	.resource	= tmpa910_resource_sdhc,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_sdhc),
};
#endif

/*
 * SPI
 */
#if defined CONFIG_SPI_TMPA910 || defined CONFIG_SPI_TMPA910_MODULE
#define CONFIG_SPI_CHANNEL0	/* enable SPI channel 0 */

#ifdef CONFIG_SPI_CHANNEL0
static struct resource tmpa910_resource_spi0[] = {
	{
		.start	= 0xF2002000,
		.end	= 0xF2002000+0x27,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= INTR_VECT_SSP_CH0,
		.end	= INTR_VECT_SSP_CH0,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};
#endif /* CONFIG_SPI_CHANNEL0 */

#ifdef CONFIG_SPI_CHANNEL1
static struct resource tmpa910_resource_spi1[] = {
	{
		.start	= 0xF2003000,
		.end	= 0xF2003000+0x27,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= INTR_VECT_SSP_CH1,
		.end	= INTR_VECT_SSP_CH1,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};
#endif /* CONFIG_SPI_CHANNEL1 */

#ifdef CONFIG_SPI_CHANNEL0
struct platform_device tmpa910_device_spi0 = {
	.name		 = "tmpa910-spi",
	.id = 0,
	.dev = {
		.platform_data = NULL,
	},
	.resource	= tmpa910_resource_spi0,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_spi0),
};
#endif /* CONFIG_SPI_CHANNEL0 */

#ifdef CONFIG_SPI_CHANNEL1
struct platform_device tmpa910_device_spi1 = {
	.name		 = "tmpa910-spi",
	.id = 1,
	.dev = {
		.platform_data = NULL,
	},
	.resource	= tmpa910_resource_spi1,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_spi1),
};
#endif /* CONFIG_SPI_CHANNEL1 */

static struct mmc_spi_platform_data mmc_spi_info = {
	.caps = MMC_CAP_NEEDS_POLL | MMC_CAP_SPI,
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3.3V only */
};

#ifdef CONFIG_MMC_SPI
static struct spi_board_info spi_board_info[] = 
{
{
	.modalias = "mmc_spi",
	.platform_data = &mmc_spi_info,
	.mode = SPI_MODE_0,
	.chip_select = 0,
	.max_speed_hz = 1000000,
	.bus_num = 0,

}
};
#elif defined(CONFIG_SPI_SPIDEV)
static struct spi_board_info spi_board_info[] = {
{
	.modalias = "spidev",
	.platform_data = &mmc_spi_info,
	.mode = SPI_MODE_0,
	.chip_select = 0,
	.max_speed_hz = 10000000,
	.bus_num = 0,
},
{
	.modalias = "spidev",
	.platform_data = NULL,
	.mode = SPI_MODE_0,
	.chip_select = 0,
	.max_speed_hz = 10000000,
	.bus_num = 1,
}
};
#endif

#endif /* CONFIG_SPI_TMPA910 */

/*
 * Touchscreen
 */
#if defined CONFIG_TOUCHSCREEN_TMPA910 || defined CONFIG_TOUCHSCREEN_TMPA910_MODULE
static struct tmpa910_ts_platforminfo tmpa910_info_ts = {
		.fuzz       = 0,
		.rate       = 36,
		.skip_count = 4,
};

static struct resource tmpa910_resource_ts[] = {
	{
		.start	= TS_BASE,
		.end	= TS_BASE + 0x40,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= ADC_BASE,
		.end	= ADC_BASE + 0x100,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= INTR_VECT_GPIOD,
		.end	= INTR_VECT_GPIOD,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}, {
		.start	= INTR_VECT_ADC,
		.end	= INTR_VECT_ADC,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};

struct platform_device tmpa910_device_ts = {
	.name		= "tmpa910_ts",
	.id		= 0,
	.dev = {
		.platform_data = &tmpa910_info_ts,
	},
	.resource	= tmpa910_resource_ts,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_ts),
};
#endif

/* 
 * LCD controller device 
 */
#if defined CONFIG_FB_TMPA910 || defined CONFIG_FB_TMPA910_MODULE
static struct resource tmpa9xx_resource_lcdc[] = {
	{
		.start	= LCDC_BASE,
		.end	= LCDC_BASE + 0x400,
		.flags	= IORESOURCE_MEM,
	},{
		.start	= INTR_VECT_LCDC,
		.end	= INTR_VECT_LCDC,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};

static struct tmpa910_lcdc_platforminfo topas910_v1_lcdc_platforminfo;

struct platform_device tmpa9xx_device_lcdc= {
	.name		= "tmpa9xxfb",
	.id		= 0,
	.resource	= tmpa9xx_resource_lcdc,
	.num_resources	= ARRAY_SIZE(tmpa9xx_resource_lcdc),
        .dev = {
		.coherent_dma_mask = 0xffffffff,		
        },
};
#endif

/*
 * NAND Flash Controller
 */
#if defined CONFIG_MTD_NAND_TMPA910 || defined CONFIG_MTD_NAND_TMPA910_MODULE
static struct resource tmpa910_nand_resources[] = {
	[0] = {
		.start	=  NANDF_BASE,
		.end	=  NANDF_BASE + 0x200,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device tmpa910_nand_device = {
	.name		= "tmpa9x0-nand",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(tmpa910_nand_resources),
	.resource	= tmpa910_nand_resources,
};
#endif

/*
 * Real Time Clock
 */
#if defined CONFIG_RTC_DRV_TMPA910 || defined CONFIG_RTC_DRV_TMPA910_MODULE
static struct resource tmpa910_resource_rtc[] = {
	{
		.start = RTC_BASE,
		.end   = RTC_BASE + 0x3ff,
		.flags = IORESOURCE_MEM
	},{
		.start = INTR_VECT_RTC,
		.end   = INTR_VECT_RTC,
		.flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH
	}
};

static struct platform_device tmpa910_device_rtc = {
	.name           = "tmpa910_rtc",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(tmpa910_resource_rtc),
	.resource       = tmpa910_resource_rtc
	}
;
#endif

/*
 * USB Host Controller
 */
#if defined CONFIG_USB_OHCI_HCD_TMPA900 || defined CONFIG_USB_OHCI_HCD_TMPA900_MODULE
static struct resource tmpa900_ohci_resources[] = {
        [0] = {
                .start  = 0xf4500000,
                .end    = 0xf4500000 + 0x100,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                .start  = 0xF8008000,
                .end    = 0xF8009fff,
                .flags  = IORESOURCE_MEM,
        },
        [2] = {
                .start  = 27,
                .end    = 27,
                .flags  = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
        },
};

static struct platform_device tmpa900_ohci_device = {
        .name           = "tmpa900-usb",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(tmpa900_ohci_resources),
        .resource       = tmpa900_ohci_resources,
        .dev = {
		.coherent_dma_mask = 0xffffffff,		
        },
};
#endif /* CONFIG_USB_OHCI_HCD_TMPA900 */


/*
 * USB Device Controller
 */
#if defined CONFIG_USB_GADGET_TMPA910 || defined CONFIG_USB_GADGET_TMPA910_MODULE
static struct resource tmpa910_udc_resource[] = {
        [0] = {
                .start = 0xf4400000,
                .end   = 0xf44003ff,
                .flags = IORESOURCE_MEM
        },
        [1] = {
                .start = USB_INT,
                .end   = USB_INT,
                .flags = IORESOURCE_IRQ
        }
};

static struct platform_device tmpa910_udc_device = {
        .name           = "tmpa9xx-udc",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(tmpa910_udc_resource),
        .resource       = tmpa910_udc_resource,
        .dev            = {
        .platform_data  = NULL,
        }
};
#endif

/*
 * Watchdog
 */
#if defined CONFIG_TMPA9X0_WATCHDOG || defined CONFIG_TMPA9X0_WATCHDOG_MODULE
static struct resource tmpa9x0_wdt_resource[] = {
        [0] = {
                .start = 0xf0010000,
                .end   = 0xf0010c04,
                .flags = IORESOURCE_MEM
        },
};

static struct platform_device tmpa910_wdt_device = {
        .name           = "tmpa9x0_wdt",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(tmpa9x0_wdt_resource),
        .resource       = tmpa9x0_wdt_resource,
        .dev            = {
        .platform_data  = NULL,
        }
};
#endif

static struct resource tmpa9xx_pwm_resource[] = {
	[0] = {
		.start = TMPA910_TIMER2,
		.end   = TMPA910_TIMER2 + 0x0fff,
		.flags = IORESOURCE_MEM
	},
};

static struct platform_device tmpa9xx_pwm_device = {
	.name		= "tmpa9xx-pwm",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(tmpa9xx_pwm_resource),
	.resource	= tmpa9xx_pwm_resource,
	.dev		= {
	.platform_data	= NULL,
	}
};

#if defined CONFIG_BACKLIGHT_PWM
static int tonga_backlight_init(struct device *dev)
{
	int ret=0;

	return ret;
}

static int tonga_backlight_notify(struct device *dev, int brightness)
{
/*
	gpio_set_value(VIPER_LCD_EN_GPIO, !!brightness);
	gpio_set_value(VIPER_BCKLIGHT_EN_GPIO, !!brightness);
*/
	printk(KERN_ERR "tonga_backlight_notify() brightness=%d (-> %d)\n", brightness, !brightness);
	/* Backlight is on pin port C4 */
	//gpio_set_value(20, !brightness);

	return brightness;
}

static void tonga_backlight_exit(struct device *dev)
{
/*
	gpio_free(VIPER_LCD_EN_GPIO);
	gpio_free(VIPER_BCKLIGHT_EN_GPIO);
*/
}

static struct platform_pwm_backlight_data tonga_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 100,
	.dft_brightness	= 100,
	.pwm_period_ns	= 255,
	.init		= tonga_backlight_init,
	.notify		= tonga_backlight_notify,
	.exit		= tonga_backlight_exit,
};

static struct platform_device tonga_backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.platform_data	= &tonga_backlight_data,
	},
};
#endif

#if defined CONFIG_SND_TMPA910_WM8983 || defined CONFIG_SND_TMPA910_WM8983_MODULE
static struct platform_device tmpa910_i2s_device = {
	.name = "WM8983-I2S",
	.id   = -1,
};
#endif

static struct platform_device *devices[] __initdata = {
#if defined CONFIG_NET_ETHERNET || defined CONFIG_NET_ETHERNET_MODULE
	&tonga_smsc911x_device,
#endif

#if defined CONFIG_SERIAL_TMPA910 || defined CONFIG_SERIAL_TMPA910_MODULE
#ifdef CONFIG_UART0
	&tmpa910_device_uart0,
#endif
#ifdef CONFIG_UART1
	&tmpa910_device_uart1,
#endif
#ifdef CONFIG_UART2
	&tmpa910_device_uart2,
#endif
#endif /* CONFIG_SERIAL_TMPA910 */

#if defined CONFIG_I2C_TMPA910 || defined CONFIG_I2C_TMPA910_MODULE
	&tmpa910_device_i2c,
#endif

#if defined CONFIG_MMC_TMPA910_SDHC || defined CONFIG_MMC_TMPA910_SDHC_MODULE
 	&tmpa910_device_sdhc,
#endif

#if defined CONFIG_SPI_TMPA910 || defined CONFIG_SPI_TMPA910_MODULE
#ifdef CONFIG_SPI_CHANNEL0
	&tmpa910_device_spi0,
#endif
#ifdef CONFIG_SPI_CHANNEL1
	&tmpa910_device_spi1,
#endif
#endif /* CONFIG_SPI_TMPA910 */

#if defined CONFIG_TOUCHSCREEN_TMPA910 || CONFIG_TOUCHSCREEN_TMPA910_MODULE
	&tmpa910_device_ts,
#endif

#if defined CONFIG_FB_TMPA910 || CONFIG_FB_TMPA910_MODULE
	&tmpa9xx_device_lcdc,
#endif

#if defined CONFIG_MTD_NAND_TMPA910 || defined CONFIG_MTD_NAND_TMPA910_MODULE
 	&tmpa910_nand_device,
#endif

#if defined CONFIG_RTC_DRV_TMPA910 || defined CONFIG_RTC_DRV_TMPA910_MODULE
	&tmpa910_device_rtc,
#endif

#if defined CONFIG_USB_OHCI_HCD_TMPA900 || defined CONFIG_USB_OHCI_HCD_TMPA900_MODULE
	&tmpa900_ohci_device,
#endif       

#if defined CONFIG_USB_GADGET_TMPA910 || defined CONFIG_USB_GADGET_TMPA910_MODULE
	&tmpa910_udc_device,
#endif
#if defined CONFIG_TMPA9X0_WATCHDOG || defined CONFIG_TMPA9X0_WATCHDOG_MODULE
	&tmpa910_wdt_device,
#endif
#if defined CONFIG_SND_TMPA910_WM8983 || defined CONFIG_SND_TMPA910_WM8983_MODULE
 	&tmpa910_i2s_device,	
#endif
	&tmpa9xx_pwm_device,
#if defined CONFIG_BACKLIGHT_PWM
	&tonga_backlight_device,
#endif
};


static void __init setup_lcdc_device(void)
{
	uint32_t *LCDReg;
	LCDReg = topas910_v1_lcdc_platforminfo.LCDReg;
	
 	/* ET0350G0DH6 Display */
	#define XSIZE_PHYS 320
	#define YSIZE_PHYS 240
	topas910_v1_lcdc_platforminfo.width  = XSIZE_PHYS;
	topas910_v1_lcdc_platforminfo.height = YSIZE_PHYS;
	topas910_v1_lcdc_platforminfo.depth  = 16;
	topas910_v1_lcdc_platforminfo.pitch  = XSIZE_PHYS * 2;


	/* Horizontal timing, LCDTiming0 */
	#define HBP                       (68)                    /* Horizontal back porch  0..255 */
	#define HFP                       (20)                    /* Horizontal front porch 0..255 */
	#define HSW                       (30)                    /* Horizontal sync pulse width 0..255 */
	#define PPL                       ((XSIZE_PHYS / 16) - 1) /* Pixel per line value 0..255 */

	/* Vertical timing, LCDTiming1 */
	#define VBP                       (28)             /* Vertical back porch  0..255 */
	#define VFP                       (4)              /* Vertical front porch 0..255 */
	#define VSW                       (3)              /* Vertical sync pulse lines value 0..63 */
	#define LPP                       (YSIZE_PHYS - 1) /* Lines per panel value 0..1023 */

	/* Clock timing, LCDTiming2 */
	#define PCD_HI                    (0)            /* PCD value, upper 5 bits */
	#define PCD_LO                    ((13) & 0x1F)  /* PCD value, lower 5 bits */
	#define IPC                       1
	#define IHS			  1
	#define IVS			  1
	#define IOE			  0
	#define CPL                       ((XSIZE_PHYS-1) & 0x3FF)

	LCDReg[0] = 		  ( (PPL << 2)	    /* pixel per line */
				| ( (HSW) << 8 )    /* tHSW. Horizontal sync pulse */
				| ( (HFP) << 16 )   /* tHFP, Horizontal front porch */
				| ( (HBP) << 24 )); /* tHBP, Horizontal back porch */

	LCDReg[1] =  		 (( VBP << 24)  /* tVBP */
				| ( VFP << 16) 	/* tVFP */
				| ( VSW << 10)  /* tVSP */
				|   LPP);

	LCDReg[2] =               ((PCD_HI << 27)
	                        | (CPL << 16)
			        | (IOE << 14)
			        | (IPC << 13)
				| (IHS << 12)
				| (IVS << 11)
				| (PCD_LO << 0));
				
	LCDReg[3] = 0;
    
	/* LCDControl */
	LCDReg[4] = (0x4 << 1) | (1 << 5) | (1 << 11) | (1 << 16);

	tmpa9xx_device_lcdc.dev.platform_data = &topas910_v1_lcdc_platforminfo;

	/* Configure Pins and reset LCD */
	gpio_request(96, "LCD Reset");
	gpio_request(97, "LCD Enable");
	gpio_request(20, "Light");
	gpio_direction_output(96, 1);
	gpio_direction_output(97, 1);
	gpio_direction_output(20, 1);

	/* Reset */
  	gpio_set_value(96, 0);
    	udelay(1000);
	gpio_set_value(96, 1);

	/* Enable */
	gpio_set_value(97, 1);

	/* Light */
	gpio_set_value(20, 0);
}


void __init tonga_init_irq(void) {
	tmpa910_init_irq();
}

#define ETHERNET_MAC_ASCII_LENGTH 17
#define MAC_OFFSET		   8

static void parse_enetaddr(char *addr, unsigned char *enetaddr)
{
	char *end;
	int i;

	for (i = 0; i < 6; ++i) {
		enetaddr[i] = addr ? simple_strtoul(addr, &end, 16) : 0;
		if (addr)
			addr = (*end) ? end + 1 : end;
	}
}

/* 
 * Tonga2 device initialisation
 */
static void __init tonga_init(void)
{
	char *p;
	char eth_mac_ascii[ETHERNET_MAC_ASCII_LENGTH+2];

	/* get mac address from the command line */
	memset(eth_mac_ascii,0,sizeof(eth_mac_ascii));

	p = strstr(boot_command_line, "ethaddr=");

	if (p != NULL && (p == boot_command_line || p[-1] == ' ')) {
		printk("U-BOOT Ethernet Address: %s\n",p+8);
		memcpy(&eth_mac_ascii,p+MAC_OFFSET,ETHERNET_MAC_ASCII_LENGTH);
		parse_enetaddr (eth_mac_ascii,tonga_smsc911x_pdata.mac);
	}

	/* Memory controller - for SMSC Ethernet */
	SMC_TIMEOUT = 0x01;
    
	/* DMA setup */
	platform_bus.coherent_dma_mask = 0xffffffff;
	platform_bus.dma_mask=&topas910_dmamask;
	
	/* Pin configuration */
        
	/* Port A can be used not only as a general-purpose input pin with pull up but also as key input pin. */
	TMPA910_CFG_PORT_GPIO(PORTA); /* All useable for GPIO */
        
	/* Port B can be used not only as general-purpose output pins but also as key output pins. */
	TMPA910_CFG_PORT_GPIO(PORTB); 
	GPIOBODE = 0x00; /* Disable Open Drain */
        
	/* Port C
	   The upper 2 bits (bits [7:6]) of Port C can be used as general-purpose input/output pins
	   and the lower 3 bits (bits [4:2]) can be used as general-purpose output pins.
	   Port C can also be used as interrupt (INT9), I2C (I2C0DA, I2C0CL), low-frequency clock
	   output (FSOUT), melody output (MLDALM), PWM output function (PWM0OUT,
	   PWM2OUT), and USB Host power supply control function (USBOCn, USBPON). */

#if defined CONFIG_USB_OHCI_HCD_TMPA900 || defined CONFIG_USB_OHCI_HCD_TMPA900_MODULE
	/* prepare Port C for USB Host */
        GPIOCDATA  = 0xcf;
	GPIOCDIR  &= ~(0xc0);
	GPIOCFR1  &= ~(0xc0);
	GPIOCFR2  |= 0xc0;
	GPIOCODE  &= ~(0xc0);
	GPIOCIE   &= ~(0xc0);
	/* Enable USB Host Controller Clock Domain */
	CLKCR5    |= (1<<4);
	/* Set appropriate clock seting for USB in SYSCR8.
	   For USB device, 24Mhz directly from quartz: [5:4]  11 / 0x3
	   For USB host  , 48Mhz from F PPL / 4      : [3:0] 100 / 0x4 */
	SYSCR8    |= ((0x3<<4)|(0x4<<0));
	/* Enable overcurrent */
	HCBCR0     = 0;
#else
	TMPA910_CFG_PORT_GPIO(PORTC);
	GPIOCODE = 0x00; 
	GPIOCDIR = 0xFF;
	GPIOCFR1 = 0;
	GPIOCFR2 = 0;
	GPIOCDATA = 0x00;
#endif 

#if defined CONFIG_BACKLIGHT_PWM
	GPIOCFR1 &= ~0x10;	/* enable PWM2OUT */
	GPIOCFR2 |=  0x10;
#endif

	/* Port D can be used as general-purpose input.
	   Port D can also be used as interrupt (INTB, INTA), ADC (AN7-AN0), and touch screen
	   control (PX, PY, MX, MY) pins. */
#if defined CONFIG_TOUCHSCREEN_TMPA910 || CONFIG_TOUCHSCREEN_TMPA910_MODULE
	GPIODFR1 = 0x0f;
	GPIODFR2 = 0xf0;
	GPIODIE = 0x00;
#endif

	/* Port F
	   The upper 2 bits (bits [7:6]) of Port F can be used as general-purpose input/output pins.
	   Port F can also be used as interrupt (INTC), UART (U2RXD, U2TXD) and I2C (I2C1DA,
	   I2C1CL) pins. */
#ifdef CONFIG_UART2
	GPIOFFR1 &= ~0xC0;  /* UART 2 */
	GPIOFFR2 |= 0xC0;
	GPIOFIE  &= ~0xC0;
	GPIOFODE &= ~0xC0;
#endif    
   
	/* Port G can be used as general-purpose input/output pins.
	   Port G can also be used as SD host controller function pins (SDC0CLK, SDC0CD,
	   SDC0WP, SDC0CMD, SDC0DAT3, SDC0DAT2, SDC0DAT1 and SDC0DAT0). */
#if defined CONFIG_MMC_TMPA910_SDHC || !defined CONFIG_MMC_TMPA910_SDHC_MODULE
	GPIOGFR1 = 0xFF;
#else
	TMPA910_CFG_PORT_GPIO(PORTG); /* SDIO0 or GPIO */
#endif

	/* Port J can be used as general-purpose input/output pins.
	   Port J can also be used as LCD cotroller function pins (LD15-LD8) and CMOS image
	   sensor control (CMSVSY, CMSHBK, CMSHSY and CMSPCK) pins. */
	/* Port K can be used as general-purpose input/output pins.
	   Port K can also be used as LCD controller function pins (LD23 to LD16) and CMOS image
	   sensor control (CMSD7 toCMSD0) pins. */
#if defined CONFIG_FB_TMPA910 || CONFIG_FB_TMPA910_MODULE
	LCDCOP_STN64CR |= LCDCOP_STN64CR_G64_8bit;
	GPIOJFR2 = 0x00;
	GPIOJFR1 = 0xFF;
	GPIOKFR2 = 0x00;
	GPIOKFR1 = 0xFF;
	PMCCTL &= ~PMCCTL_PMCPWE;
	PMCWV1 |= PMCWV1_PMCCTLV;
	udelay(200);
#endif

	/* Port L can be used as general-purpose input/output pins. (Bits [7:5] are not used.)
	   In addition, Port L can also be used as I2S function (I2SSCLK, I2S0MCLK, I2S0DATI,
	   I2S0CLK and I2S0WS) and SPI function (SP1DI, SP1DO, SP1CLK and SP1FSS) pins.
	   TMPA910_CFG_PORT_GPIO(PORTR) */
	GPIOLFR2 = 0x00;
	GPIOLFR1 = 0x1f; /* bits 4:0 for I2S */

	/* Port M can be used as general-purpose input/output pins. (Bits [7:4] are not used.)
	   Port M can also be used as I2S function pins (I2S1MCLK, I2S1DATO, I2S1CLK and
	   I2S1WS).*/
	GPIOMDIR |= 0x03; /* M0, MI GPIO OUT */
	GPIOMFR1 &= ~0x03;
	GPIOMFR1 |= 0x04; /* M2 I2S1DAT0 */
	/* GPIOMFR2 &= ~0x03; */ /* there is no FR2 for port M */
           
           
	/* Port N can be used as general-purpose input/output pins.
	   Port N can also be used as UART/IrDA function (U0RTSn, U0DTRn, U0RIn, U0DSRn,
	   U0DCDn, U0CTSn, U0RXD, U0TXD, SIR0IN, SIR0OUT) and interrupt function (INTD,
	   INTE, INTF, INTG) pins. */
           
	/* already set by bootloader */
           
	/* Port R
	   Bit 2 of Port R can be used as a general-purpose input/output pin and bits [1:0] can be
	   used as general-purpose output pins. (Bits [7:3] are not used.)
	   Port R can also be used as reset output (RESETOUTn), high-frequency clock output
	   (FCOUT), interrupt function (INTH) and Oscillation Frequency Detection (OFDOUTn). */
#if defined CONFIG_NET_ETHERNET || defined CONFIG_NET_ETHERNET_MODULE
	GPIORDIR &= ~(1 << 2); /* Eth IRQ */
#endif
    
	/* Port T can be used as general-purpose input/output pins.
	   Port T can also be used as USB external clock input (X1USB), UART function (U1CTSn,
	   U1RXD, U1TXD), and SPI function (SP0DI, SP0DO, SP0CLK, SP0FSS) and pins. */
#ifdef CONFIG_UART1
	GPIOTFR1 = 0xFF;
#endif        
    
	/* Configure LCD interface */
	setup_lcdc_device();
    
	/* NAND Controller */
	NDFMCR0 = 0x00000010; // NDCE0n pin = 0, ECC-disable
	NDFMCR1 = 0x00000000; // ECC = Hamming
	NDFMCR2 = 0x00003343; // NDWEn L = 3clks,H =3clks,
              	              // NDREn L = 4clks,H = 3clks
	NDFINTC = 0x00000000; // ALL Interrupt Disable

	/* Add devices */
	platform_add_devices(devices, ARRAY_SIZE(devices));
  
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_MMC_SPI)
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif
}


MACHINE_START(TONGA, "Tonga 2")
        /* Maintainer:  Florian Boor <florian.boor@kernelconcepts.de> */
        .phys_io        = TMPA910_IO_PHYS_BASE,
        .boot_params    = 0,
        .io_pg_offst    = (io_p2v(TMPA910_IO_PHYS_BASE) >> 18) & 0xfffc,
        .map_io         = tonga_map_io,
        .init_irq       = tonga_init_irq,
        .timer          = &tmpa910_timer,
        .init_machine   = tonga_init,
MACHINE_END

