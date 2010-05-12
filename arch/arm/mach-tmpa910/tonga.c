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

#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/irq.h>

#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <video/tmpa910_fb.h>
#include <mach/gpio.h>

#include <asm/mach/arch.h>
#include <mach/hardware.h>
#include <mach/ts.h>
#include <mach/tmpa910_regs.h>
#include <asm/dma.h>
#include <linux/smsc911x.h>


#ifdef CONFIG_SPI_TMPA910
#include <linux/spi/spi.h>
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


void __init tonga_map_io(void)
{
	iotable_init(tmpa900_io_desc, ARRAY_SIZE(tmpa900_io_desc));
}


static u64 topas910_dmamask = 0xffffffffUL;

/* 
 * Ethernet 
 */ 
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
	.flags         = SMSC911X_USE_32BIT | SMSC911X_FORCE_INTERNAL_PHY,
	.phy_interface = PHY_INTERFACE_MODE_MII,
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


/* 
 * LCD controller device 
 */
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



/*
 * NAND Flash Controller
 */
#ifdef CONFIG_MTD_NAND_TMPA910
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
		.coherent_dma_mask = 0xffffffff,		
        },
};

#endif /* CONFIG_USB_OHCI_HCD_TMPA900 */


/*
 * USB Device Controller
 */
#ifdef CONFIG_USB_GADGET_TMPA910
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
	&tonga_smsc911x_device,
	&tmpa910_device_uart0,
#ifdef CONFIG_MTD_NAND_TMPA910
 	&tmpa910_nand_device,
#endif
	&tmpa9xx_device_lcdc,
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


/* 
 * Tonga2 device initialisation
 */
static void __init tonga_init(void)
{
	/* Memory controller - for SMSC Ethernet */
    	SMC_TIMEOUT = 0x01;
    
	/* DMA setup */
	platform_bus.coherent_dma_mask = 0xffffffff;
	platform_bus.dma_mask=&topas910_dmamask;
	
	/* Pin configuration */
	TMPA910_CFG_PORT_GPIO(PORTA); /* Keypad */
	TMPA910_CFG_PORT_GPIO(PORTB); /* 7 segment LED */
	TMPA910_CFG_PORT_GPIO(PORTC); /* TEST display */
	TMPA910_CFG_PORT_GPIO(PORTG); /* SDIO0, for SPI MMC */
	TMPA910_CFG_PORT_GPIO(PORTR); /*  */
	GPIOBODE = 0x00; /* Disable Open Drain */
	GPIOCODE = 0x00; /* Disable Open Drain */

	TMPA910_PORT_T_FR1 = 0x00F0; /* Enable USB function pin */
    
	GPIORDIR &= ~(1 << 2); /* Eth IRQ */
    
	GPIOCDIR = 0xFF;
	GPIOCFR1 = 0;
	GPIOCFR2 = 0;
	GPIOCDATA = 0x00;
    
	GPIOCIE &= ~0xC0; /* USB Host */
	GPIOCFR1 &= ~0xC0;
	GPIOCFR2 |=  0xC0;
    
	GPIOMDIR |= 0x03; /* M0, MI GPIO OUT */
	GPIOMFR1 &= ~0x03;
	GPIOMFR2 &= ~0x03;

	PMCCTL &= ~PMCCTL_PMCPWE;
	PMCWV1 |= PMCWV1_PMCCTLV;
    	udelay(200);
	LCDCOP_STN64CR |= LCDCOP_STN64CR_G64_8bit;
	GPIOJFR2 = 0x00;
	GPIOJFR1 = 0xFF;
	GPIOKFR2 = 0x00;
	GPIOKFR1 = 0xFF;
    
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
        .timer          = &topas910_timer,
        .init_machine   = tonga_init,
MACHINE_END
