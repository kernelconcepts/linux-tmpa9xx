/*
 *  arch/arm/mach-tmpa910/topasa900.c 
 *
 * Copyright (C) 2008 bplan GmbH. All rights reserved.
 * Copyright (C) 2009 Florian Boor <florian.boor@kernelconcepts.de>
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
 * Toshiba Topas A900 machine, reference design for the TMPA900 SoC 
 *
 * TODO: MMC, ADC, power manager
 * TODO: separate SoC and board code
 */

#include <linux/device.h>
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
#include <asm/serial.h>
#include <asm/dma.h>

#if defined(CONFIG_USB_ISP1362_HCD)
#include <linux/usb/isp1362.h>
#endif

#ifdef CONFIG_SPI_TMPA910
#include <linux/spi/spi.h>
#endif

#ifdef CONFIG_SPI_AT25
#include <linux/spi/eeprom.h>
#endif

#include "topas910.h"

#define CONFIG_SPI_CHANNEL0


/* I/O Mapping related, might want to be moved to a CPU specific file */

static struct map_desc tmpa900_io_desc[] __initdata = {
	{
		.virtual = TMPA910_IO_VIRT_BASE,
		.pfn = __phys_to_pfn(TMPA910_IO_PHYS_BASE),
		.length	= TMPA910_IO_SIZE,
		.type = MT_DEVICE,
	}
};


void __init topasa900_map_io(void)
{
	iotable_init(tmpa900_io_desc, ARRAY_SIZE(tmpa900_io_desc));
}


static void dummy_release(struct device *dev)
{
        /* normally not freed */
}

static u64  topas910_dmamask = 0xffffffffUL;

/* Ethernet */
 
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


/*
 * Serial UART
 */ 

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


/*
 * DMA
*/
static struct resource tmpa910_resource_dmac[] = {
	{
		.start	= DMAC_BASE,
		.end	= DMAC_BASE+0x200,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= INTR_VECT_DMA_END,
		.end	= INTR_VECT_DMA_END,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}, {
		.start	= INTR_VECT_DMA_ERROR,
		.end	= INTR_VECT_DMA_ERROR,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};

struct platform_device tmpa910_device_dmac = {
	.name		= "tmpa910-dmac",
	.id		= 0,
	.dev = {
		.platform_data = NULL
	},
	.resource	= tmpa910_resource_dmac,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_dmac),
};


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
#endif

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
#endif

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
#endif

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
#endif



/*
 * Touchscreen
 */

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
		.platform_data = NULL
	},
	.resource	= tmpa910_resource_ts,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_ts),
};


/* LCD controller device */

static struct resource tmpa910_resource_lcdc[] = {
	{
		.start	= LCDC_BASE,
		.end	= LCDC_BASE + 0x400,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= FB_OFFSET,
		.end	= FB_OFFSET+FB_SIZE,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= INTR_VECT_LCDC,
		.end	= INTR_VECT_LCDC,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};


static struct tmpa910_lcdc_platforminfo topas910_v1_lcdc_platforminfo;


struct platform_device tmpa910_device_lcdc= {
	.name		= "tmpa910_lcdc",
	.id		= 0,
	.resource	= tmpa910_resource_lcdc,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_lcdc),
};


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
 * NAND Flash Controller
 */

#ifdef CONFIG_MTD_NAND_TMPA910
static struct resource tmpa910_nand_resources[] = {
	[0] = {
		.start	=  NANDF_BASE ,
		.end	=  NANDF_BASE + 0x200,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device tmpa910_nand_device = {
	.name		= "tmpa910-nand",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(tmpa910_nand_resources),
	.resource	= tmpa910_nand_resources,
};

#endif


static struct platform_device tmpa910_i2s_device = {
	.name = "WM8976-I2S",
	.id   = -1,
};


#ifdef CONFIG_MMC_SPI
static struct spi_board_info spi_board_info[] = 
{
{
	.modalias = "mmc_spi",
	.platform_data = NULL,
	.mode = SPI_MODE_0,
	.chip_select = 0,
	.max_speed_hz = 1000000,
	.bus_num = 0,

}
};

#elif defined(CONFIG_SPI_AT25)
static struct spi_eeprom spi_eeprom_info = {
	.page_size = 256,
	.name = "AT25F512",
	.flags = EE_ADDR3|EE_READONLY
};
 
static struct spi_board_info spi_board_info[] = {
{
	.modalias = "at25",
	.platform_data = &spi_eeprom_info,
	.mode = SPI_MODE_0,
	.chip_select = 0,
	.max_speed_hz = 20000000,
	.bus_num = 0,
},
{
	.modalias = "at25",
	.platform_data = &spi_eeprom_info,
	.mode = SPI_MODE_0,
	.chip_select = 0,
	.max_speed_hz = 20000000,
	.bus_num = 1,
}
};
#elif defined(CONFIG_SPI_SPIDEV)
static struct spi_board_info spi_board_info[] = {
{
	.modalias = "spidev",
	.platform_data = NULL,
	.mode = SPI_MODE_0,
	.chip_select = 0,
	.max_speed_hz = 20000000,
	.bus_num = 0,
},
{
	.modalias = "spidev",
	.platform_data = NULL,
	.mode = SPI_MODE_0,
	.chip_select = 0,
	.max_speed_hz = 20000000,
	.bus_num = 1,
}
};

#endif


#define RTC_BASE		0xF0030000

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


#ifdef CONFIG_USB_OHCI_HCD_TMPA900
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
        .id             = 0,
        .num_resources  = ARRAY_SIZE(tmpa900_ohci_resources),
        .resource       = tmpa900_ohci_resources,
        .dev = {
		.release        = dummy_release, // not needed
		.coherent_dma_mask = 0xffffffff,		
        },
};

#endif /* CONFIG_USB_OHCI_HCD_TMPA900 */


#ifdef CONFIG_USB_GADGET_TMPA910
/* USB Device Controller */
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
        .name           = "tmpa910-usb",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(tmpa910_udc_resource),
        .resource       = tmpa910_udc_resource,
        .dev            = {
        .platform_data  = NULL,
        }
};
#endif


static struct platform_device *devices[] __initdata = {
	&tmpa910_device_dmac,
	&tmpa910_device_ts,
	&topas910_led_device,
	&topas910_dm9000_device,
	&tmpa910_device_uart0,
#ifdef CONFIG_MTD_NAND_TMPA910
 	&tmpa910_nand_device,
#endif
	&topas910_keys_device,
	&tmpa910_device_lcdc,
	&tmpa910_device_i2c,
#ifdef CONFIG_USB_GADGET_TMPA910
	&tmpa910_udc_device,
#endif
#ifdef CONFIG_USB_OHCI_HCD_TMPA900
	&tmpa900_ohci_device,
#endif
 	&tmpa910_i2s_device,	
#ifdef CONFIG_SPI_CHANNEL0
	&tmpa910_device_spi0,
#endif
#ifdef CONFIG_SPI_CHANNEL1
	&tmpa910_device_spi1,
#endif
	&tmpa910_device_rtc
};


static void __init setup_lcdc_device(void)
{
	uint32_t *LCDReg;
	int width  = 320;
	int height = 240;

	topas910_v1_lcdc_platforminfo.width  = width;
	topas910_v1_lcdc_platforminfo.height = height;
	topas910_v1_lcdc_platforminfo.depth  = 32;
	topas910_v1_lcdc_platforminfo.pitch  = width*4;
	
	LCDReg = topas910_v1_lcdc_platforminfo.LCDReg;
	LCDReg[LCDREG_TIMING0_H] =	( ((height/16)-1) << 2)	// pixel per line
			| ( (8-1) << 8 ) 				// tHSW. Horizontal sync pulse
			| ( (8-1) << 16 ) 			// tHFP, Horizontal front porch
			| ( (8-1) << 24 ); 			// tHBP, Horizontal back porch

	LCDReg[LCDREG_TIMING1_V] =     (2 << 24) 		// tVBP		
			| (2 << 16) 		// tVFP
			| ((2-1) << 10) 		// tVSP
			| (width-1);

	LCDReg[LCDREG_TIMING2_CLK] = ((width-1)<<16) | 0x0000e | 1<<13 | 0<<12 | 0<<11;
	LCDReg[LCDREG_TIMING3_LEC] = 0;
	LCDReg[LCDREG_LCDCONTROL]	= (0x5<<1)  | (1<<5) | (1<<11);
	tmpa910_device_lcdc.dev.platform_data = &topas910_v1_lcdc_platforminfo;
}


void __init topasa900_init_irq(void) {
	tmpa910_init_irq();
}


/* TopasA900 device initialisation */

static void __init topasa900_init(void)
{
        
	/* Memory controller - for DM9000 */
	SMC_SET_CYCLES_3 = 0x0004AFAA;
	SMC_SET_OPMODE_3 = 0x00000002;
	SMC_DIRECT_CMD_3 = 0x00C00000;
    
	/* DMA setup */
	platform_bus.coherent_dma_mask = 0xffffffff;
	platform_bus.dma_mask=&topas910_dmamask;
	
	/* Pin configuration */
	TMPA910_CFG_PORT_GPIO(PORTA); /* Keypad */
	TMPA910_CFG_PORT_GPIO(PORTB); /* 7 segment LED */
	TMPA910_CFG_PORT_GPIO(PORTG); /* SDIO0, for SPI MMC */
	TMPA910_CFG_PORT_GPIO(PORTP); /* GPIO routed to CM605 left */
	TMPA910_PORT_T_FR1 = 0x00F0; /* Enable USB function pin */

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


MACHINE_START(TOPASA900, "Toshiba TopasA900")
        /* Maintainer:  Florian Boor <florian.boor@kernelconcepts.de> */
        .phys_io        = TMPA910_IO_PHYS_BASE,
        .boot_params    = 0,
        .io_pg_offst    = (io_p2v(TMPA910_IO_PHYS_BASE) >> 18) & 0xfffc,
        .map_io         = topasa900_map_io,
        .init_irq       = topasa900_init_irq,
        .timer          = &topas910_timer,
        .init_machine   = topasa900_init,
MACHINE_END

