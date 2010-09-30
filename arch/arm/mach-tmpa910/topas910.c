/*
 * arch/arm/mach-tmpa910/topas910.c -- Topas 910 machine 
 *
 * Copyright (C) 2008 bplan GmbH. All rights reserved.
 * Copyright (C) 2009, 2010 Florian Boor <florian.boor@kernelconcepts.de>
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
 * Toshiba Topas 910 machine, reference design for the TMPA910CRAXBG SoC 
 *
 * TODO: MMC, ADC, power manager
 * TODO: separate SoC and board code
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/spi/spi.h>
#include <linux/spi/mmc_spi.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/page.h>

#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <video/tmpa910_fb.h>
#include <mach/gpio.h>

#include <asm/mach/arch.h>
#include <mach/hardware.h>
#include <mach/ts.h>
#include <mach/tmpa910_regs.h>
#include <linux/mmc/host.h>
#include <asm/serial.h>

#ifdef CONFIG_SPI_TMPA910
#include <linux/spi/spi.h>
#endif

#ifdef CONFIG_SPI_AT25
#include <linux/spi/eeprom.h>
#endif

#include "topas910.h"

#define CONFIG_SPI_CHANNEL0


/* I/O Mapping related, might want to be moved to a CPU specific file */

static struct map_desc tmpa910_io_desc[] __initdata = {
	{
		.virtual = TMPA910_IO_VIRT_BASE,
		.pfn = __phys_to_pfn(TMPA910_IO_PHYS_BASE),
		.length	= TMPA910_IO_SIZE,
		.type = MT_DEVICE,
	}
};


void __init topas910_map_io(void)
{
	iotable_init(tmpa910_io_desc, ARRAY_SIZE(tmpa910_io_desc));
}


static void dummy_release(struct device *dev)
{
        /* normally not freed */
}

static u64  topas910_dmamask = 0xffffffffUL;



/* Ethernet */
#if defined CONFIG_NET_ETHERNET || defined CONFIG_NET_ETHERNET_MODULE
static struct resource dm9000_resources[] = {
        [0] = {
                .start  = 0x60000002,
                .end    = 0x60000003,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                .start  = 0x60001002,
                .end    = 0x60001003,
                .flags  = IORESOURCE_MEM,
        },
        [2] = {
                .start  = TOPAS910_INT_DM9000,
                .end    = TOPAS910_INT_DM9000,
                .flags  = IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
        },
};


static struct platform_device topas910_dm9000_device = {
        .name           = "dm9000",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(dm9000_resources),
        .resource       = dm9000_resources,
        .dev = {
		.release        = dummy_release,
		.coherent_dma_mask = 0xffffffff,		
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
static struct tmpa9xx_ts_platforminfo tmpa9xx_info_ts = {
		.fuzz       = 0,
		.rate       = 36,
		.skip_count = 4,
};

static struct resource tmpa9xx_resource_ts[] = {
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
	.name		= "tmpa9xx_ts",
	.id		= 0,
	.dev = {
		.platform_data = &tmpa9xx_info_ts,
	},
	.resource	= tmpa9xx_resource_ts,
	.num_resources	= ARRAY_SIZE(tmpa9xx_resource_ts),
};
#endif

/* 
 * 7 segment LED display
 */
static struct platform_device topas910_led_device = {
	.name = "led-topas",
	.id = -1,
};


/*
 * Joystick 
 */
static struct gpio_keys_button topas910_buttons[] = {
	{
		.code	= KEY_LEFT,
		.gpio	= 0,
		.active_low = 0,
		.desc	= "Joystick left",
		.type	= EV_KEY,
		.wakeup = 0,
	},
	{
		.code	= KEY_DOWN,
		.gpio	= 1,
		.active_low = 0,
		.desc	= "Joystick down",
		.type	= EV_KEY,
		.wakeup = 0,
	},
	{
		.code	= KEY_UP,
		.gpio	= 2,
		.active_low = 0,
		.desc	= "Joystick up",
		.type	= EV_KEY,
		.wakeup = 0,
	},
	{
		.code	= KEY_RIGHT,
		.gpio	= 3,
		.active_low = 0,
		.desc	= "Joystick right",
		.type	= EV_KEY,
		.wakeup = 0,
	},
	{
		.code	= KEY_ENTER,
		.gpio	= 4,
		.active_low = 0,
		.desc	= "Joystick center",
		.type	= EV_KEY,
		.wakeup = 1,
	},
};

static struct gpio_keys_platform_data topas910_keys_data = {
	.buttons	= topas910_buttons,
	.nbuttons	= ARRAY_SIZE(topas910_buttons),
};

static struct platform_device topas910_keys_device = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data = &topas910_keys_data,
	},
};

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

static struct platform_device tmpa910_i2s_device = {
	.name = "WM8976-I2S",
	.id   = -1,
};


static struct platform_device *devices[] __initdata = {
#if defined CONFIG_NET_ETHERNET || defined CONFIG_NET_ETHERNET_MODULE
	&topas910_dm9000_device,
#endif

#if defined CONFIG_SERIAL_TMPA910 || defined CONFIG_SERIAL_TMPA910_MODULE
#ifdef CONFIG_UART0
	&tmpa910_device_uart0,
#endif
#ifdef CONFIG_UART1
	&tmpa910_device_uart1,
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
	&topas910_keys_device,
 	&tmpa910_i2s_device,	
	&topas910_led_device,
};


static void __init setup_lcdc_device(void)
{
	uint32_t *LCDReg;
	int width  = 320;
	int height = 240;
	
	LCDReg = topas910_v1_lcdc_platforminfo.LCDReg;
#ifdef CONFIG_DISPLAY_GLYN_640_480
 // ET057007DHU Display

#define XSIZE_PHYS 640
#define YSIZE_PHYS 480
	topas910_v1_lcdc_platforminfo.width  = XSIZE_PHYS;
	topas910_v1_lcdc_platforminfo.height = YSIZE_PHYS;
	topas910_v1_lcdc_platforminfo.depth  = 32;
	topas910_v1_lcdc_platforminfo.pitch  = XSIZE_PHYS * 4;


//      Horizontal timing, LCDTiming0
#define HBP                       (90)                      // Horizontal back porch  0..255
#define HFP                       (6)                      // Horizontal front porch 0..255
#define HSW                       (10)                      // Horizontal sync pulse width 0..255
#define PPL                       ((XSIZE_PHYS / 16) - 1)     // Pixel per line value 0..255

//      Vertical timing, LCDTiming1
#define VBP                       (8)                      // Vertical back porch  0..255
#define VFP                       (8)                      // Vertical front porch 0..255
#define VSW                       (2)                      // Vertical sync pulse lines value 0..63
#define LPP                       (YSIZE_PHYS - 1)            // Lines per panel value 0..1023

//      Clock timing, LCDTiming2
#define PCD_HI                    (0)            // PCD value, upper 5 bits
#define PCD_LO                    ((2) & 0x1F)          // PCD value, lower 5 bits
#define IPC                       1
#define IHS			  1
#define IVS			  1
#define CPL                       ((XSIZE_PHYS-1)&0x3FF)

	LCDReg[0] = 
				  ( (PPL << 2)	// pixel per line
				| ( (HSW) << 8 ) 			// tHSW. Horizontal sync pulse
				| ( (HFP) << 16 ) 			// tHFP, Horizontal front porch
				| ( (HBP) << 24 )); 			// tHBP, Horizontal back porch


	LCDReg[1] =  		  (( VBP << 24) 		// tVBP		
				| ( VFP << 16) 		// tVFP
				| ( VSW << 10) 		// tVSP
				| ( LPP));

	LCDReg[2] =               ((PCD_HI << 27)
	                        | (CPL << 16)
			        | (IPC<<13)
				| (IHS<<12)
				| (IVS<<11)
				| (PCD_LO << 0));
				
	LCDReg[3] = 0;
	LCDReg[4] = (0x5<<1)  | (1<<5)  | (1<<11) | (1<<16); /* LCDControl */
#else

	topas910_v1_lcdc_platforminfo.width  = width;
	topas910_v1_lcdc_platforminfo.height = height;
	topas910_v1_lcdc_platforminfo.depth  = 32;
	topas910_v1_lcdc_platforminfo.pitch  = width*4;

	LCDReg[0] = 
				  ( ((width/16)-1) << 2)	// pixel per line
				| ( (8-1) << 8 ) 				// tHSW. Horizontal sync pulse
				| ( (8-1) << 16 ) 			// tHFP, Horizontal front porch
				| ( (8-1) << 24 ) 			// tHBP, Horizontal back porch
				;

	LCDReg[1] = 
				(2 << 24) 		// tVBP		
				| (2 << 16) 		// tVFP
				| ((2-1) << 10) 		// tVSP
				| (height-1);

	LCDReg[2] = ((width-1)<<16) | 0x0000e | 1<<13 | 0<<12 | 0<<11;
	LCDReg[3] = 0;
	LCDReg[4]	= (0x5<<1)  | (1<<5) | (1<<11);
#endif
	tmpa9xx_device_lcdc.dev.platform_data = &topas910_v1_lcdc_platforminfo;
}


void __init topas910_init_irq(void) {
	tmpa910_init_irq();
}


/* Topas910 device initialisation */

static void __init topas910_init(void)
{
	/* Memory controller - for DM9000 */
	SMC_SET_CYCLES_3 = 0x0004AFAA;
	SMC_SET_OPMODE_3 = 0x00000002;
	SMC_DIRECT_CMD_3 = 0x00C00000;
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
	   PWM2OUT). */

	TMPA910_CFG_PORT_GPIO(PORTC);
	GPIOCODE = 0x00; 
	GPIOCDIR = 0xFF;
	GPIOCFR1 = 0;
	GPIOCFR2 = 0;
	GPIOCDATA = 0x00;

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

	/* Port P */
	TMPA910_CFG_PORT_GPIO(PORTP); /* GPIO routed to CM605 left */

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
  
#if defined(CONFIG_SPI_AT25) || defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_MMC_SPI)
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif
}


MACHINE_START(TOPAS910, "Toshiba Topas910")
        /* Maintainer:  Florian Boor <florian.boor@kernelconcepts.de> */
        .phys_io        = TMPA910_IO_PHYS_BASE,
        .boot_params    = 0,
        .io_pg_offst    = (io_p2v(TMPA910_IO_PHYS_BASE) >> 18) & 0xfffc,
        .map_io         = topas910_map_io,
        .init_irq       = topas910_init_irq,
        .timer          = &tmpa910_timer,
        .init_machine   = topas910_init,
MACHINE_END
