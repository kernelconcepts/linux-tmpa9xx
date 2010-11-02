/*
 *  arch/arm/mach-tmpa9xx/topasa900.c 
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
 * Toshiba Topas A900 machine, reference design for the TMPA900 SoC 
 *
 * TODO: ADC, power manager
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
#include <linux/mmc/host.h>
#include <linux/i2c.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>

#include <video/tmpa9xx_fb.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/ts.h>
#include <mach/regs.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "topas910.h"

/* I/O Mapping related, might want to be moved to a CPU specific file */

static struct map_desc tmpa9xx_io_desc[] __initdata = {
	{
		.virtual = TMPA9XX_IO_VIRT_BASE,
		.pfn = __phys_to_pfn(TMPA9XX_IO_PHYS_BASE),
		.length	= TMPA9XX_IO_SIZE,
		.type = MT_DEVICE,
	}
};


void __init topasa900_map_io(void)
{
	iotable_init(tmpa9xx_io_desc, ARRAY_SIZE(tmpa9xx_io_desc));
}


/* 
 * Ethernet 
 */ 
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
		.coherent_dma_mask = 0xffffffff,		
        },
};
#endif

/*
 * Serial UARTs
 */
#if defined CONFIG_SERIAL_AMBA_PL011 || defined CONFIG_SERIAL_AMBA_PL011_MODULE

#ifdef CONFIG_SERIAL_AMBA_PL011_CHANNEL_0
struct amba_device pl011_device0 = {
	.dev = {
		.init_name = "uart0",
		.platform_data = NULL,
	},
	.res = {
        	 .start = 0xf2000000,
                 .end   = 0xf2000000 + SZ_4K -1,
                 .flags = IORESOURCE_MEM
               },
	.irq = {10, NO_IRQ},
	.periphid = 0x00041011,
};
#endif

#ifdef CONFIG_SERIAL_AMBA_PL011_CHANNEL_1
struct amba_device pl011_device1 = {
	.dev = {
		.init_name = "uart1",
		.platform_data = NULL,
	},
	.res = {
        	 .start = 0xf2001000,
                 .end   = 0xf2001000 + SZ_4K -1,
                 .flags = IORESOURCE_MEM
               },
	.irq = {11, NO_IRQ},
	.periphid = 0x00041011,
};

#endif

#ifdef CONFIG_SERIAL_AMBA_PL011_CHANNEL_2
struct amba_device pl011_device2 = {
	.dev = {
		.init_name = "uart2",
		.platform_data = NULL,
	},
	.res = {
        	 .start = 0xf2004000,
                 .end   = 0xf2004000 + SZ_4K -1,
                 .flags = IORESOURCE_MEM
               },
	.irq = {9, NO_IRQ},
	.periphid = 0x00041011,
};
#endif

#endif // defined SERIAL_AMBA_PL011 || defined SERIAL_AMBA_PL011_MODULE
 
/*
 * I2C
 */ 
#if defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE
static struct resource tmpa9xx_resource_i2c[] = {
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

struct platform_device tmpa9xx_device_i2c = {
	.name		 = "tmpa9xx-i2c",
	.id = 0,
	.dev = {
		.platform_data = NULL,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= tmpa9xx_resource_i2c,
	.num_resources	= ARRAY_SIZE(tmpa9xx_resource_i2c),
};

static struct i2c_board_info topasa900_i2c_0_devices[] = {
	{
	},
};

static struct i2c_board_info topasa900_i2c_1_devices[] = {
	{
	},
};
#endif

/*
 * SDHC
 */ 
#if defined CONFIG_MMC_TMPA9XX_SDHC || defined CONFIG_MMC_TMPA9XX_SDHC_MODULE
static struct resource tmpa9xx_resource_sdhc[] = {
{
		.start	= INTR_VECT_SDHC,
		.end	= INTR_VECT_SDHC,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	},
};

struct platform_device tmpa9xx_device_sdhc = {
	.name		= "tmpa9xx-sdhc",
	.id		= -1,
	.dev =
	{
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= tmpa9xx_resource_sdhc,
	.num_resources	= ARRAY_SIZE(tmpa9xx_resource_sdhc),
};
#endif

/*
 * SPI
 */
#if defined CONFIG_SPI_PL022 || defined CONFIG_SPI_PL022_MODULE
 
#ifdef CONFIG_SPI_PL022_CHANNEL_0
static void tmpa9xx_spi0_cs_control(u32 command)
{

}
#endif

#ifdef CONFIG_SPI_PL022_CHANNEL_1
static void tmpa9xx_spi1_cs_control(u32 command)
{

}
#endif

#if defined CONFIG_SPI_PL022_CHANNEL_0
struct pl022_config_chip mmc_info = {
	.lbm = LOOPBACK_DISABLED,
	.com_mode = INTERRUPT_TRANSFER,
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	/* we can act as master only */
	.hierarchy = SSP_MASTER,
	.slave_tx_disable = 0,
	.endian_rx = SSP_RX_MSB,
	.endian_tx = SSP_TX_MSB,
	.data_size = SSP_DATA_BITS_8,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.clk_phase = SSP_CLK_SECOND_EDGE,
	.clk_pol = SSP_CLK_POL_IDLE_HIGH,
	.cs_control = tmpa9xx_spi0_cs_control,
};
#endif

#if defined CONFIG_SPI_PL022_CHANNEL_0
struct pl022_config_chip spidev0_info = {
	.lbm = LOOPBACK_DISABLED,
	.com_mode = INTERRUPT_TRANSFER,
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	/* we can act as master only */
	.hierarchy = SSP_MASTER,
	.slave_tx_disable = 0,
	.endian_rx = SSP_RX_MSB,
	.endian_tx = SSP_TX_MSB,
	.data_size = SSP_DATA_BITS_8,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.clk_phase = SSP_CLK_SECOND_EDGE,
	.clk_pol = SSP_CLK_POL_IDLE_HIGH,
	.cs_control = tmpa9xx_spi0_cs_control,
};
#endif

#if defined CONFIG_SPI_PL022_CHANNEL_1
struct pl022_config_chip spidev1_info = {
	.lbm = LOOPBACK_DISABLED,
	.com_mode = INTERRUPT_TRANSFER,
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	/* we can act as master only */
	.hierarchy = SSP_MASTER,
	.slave_tx_disable = 0,
	.endian_rx = SSP_RX_MSB,
	.endian_tx = SSP_TX_MSB,
	.data_size = SSP_DATA_BITS_8,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.clk_phase = SSP_CLK_SECOND_EDGE,
	.clk_pol = SSP_CLK_POL_IDLE_HIGH,
	.cs_control = tmpa9xx_spi1_cs_control,
};
#endif

#if defined CONFIG_MMC_SPI && defined CONFIG_SPI_PL022_CHANNEL_0
static struct mmc_spi_platform_data mmc_spi_info = {
	.caps = MMC_CAP_NEEDS_POLL | MMC_CAP_SPI,
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3.3V only */
};

static struct spi_board_info spi_board_info[] = {
#ifdef CONFIG_SPI_PL022_CHANNEL_0
{
	.modalias = "mmc_spi",
	.controller_data = &mmc_info,
	.platform_data = &mmc_spi_info,
	.mode = SPI_MODE_0,
	.chip_select = 0,
	.max_speed_hz = 24000000,
	.bus_num = 0,

},
#endif
#ifdef CONFIG_SPI_PL022_CHANNEL_1
{
	.modalias = "spidev",
        .controller_data = &spidev1_info,
	.platform_data = NULL,
	.mode = SPI_MODE_0,
	.chip_select = 0,
	.max_speed_hz = 10000000,
	.bus_num = 1,
}
#endif
};
#elif defined(CONFIG_SPI_SPIDEV)
static struct spi_board_info spi_board_info[] = {
#ifdef CONFIG_SPI_PL022_CHANNEL_0
{
	.modalias = "spidev",
        .controller_data = &spidev0_info,
	.platform_data = NULL,
	.mode = SPI_MODE_0,
	.chip_select = 0,
	.max_speed_hz = 10000000,
	.bus_num = 0,
},
#endif
#ifdef CONFIG_SPI_PL022_CHANNEL_1
{
	.modalias = "spidev",
        .controller_data = &spidev1_info,
	.platform_data = NULL,
	.mode = SPI_MODE_0,
	.chip_select = 0,
	.max_speed_hz = 10000000,
	.bus_num = 1,
}
#endif
};
#endif

#ifdef CONFIG_SPI_PL022_CHANNEL_0
static struct pl022_ssp_controller ssp0_platform_data = {
	.bus_id = 0,
	/* pl022 not yet supports dma */
	.enable_dma = 0,
	.num_chipselect = 1,
};
#endif

#ifdef CONFIG_SPI_PL022_CHANNEL_1
static struct pl022_ssp_controller ssp1_platform_data = {
	.bus_id = 1,
	/* pl022 not yet supports dma */
	.enable_dma = 0,
	.num_chipselect = 1,
};
#endif


#ifdef CONFIG_SPI_PL022_CHANNEL_0
static struct amba_device pl022_device0 = {
	.dev = {
		.coherent_dma_mask = ~0,
		.init_name = "tmpa9xx-spi0",
		.platform_data = &ssp0_platform_data,
	},
	.res = {
		.start = 0xF2002000,
		.end   = 0xF2002027,
		.flags = IORESOURCE_MEM,
	},
	.irq = {INTR_VECT_SSP_CH0, NO_IRQ },
	.periphid = 0x00041022,
};
#endif

#ifdef CONFIG_SPI_PL022_CHANNEL_1
static struct amba_device pl022_device1 = {
	.dev = {
		.coherent_dma_mask = ~0,
		.init_name = "tmpa9xx-spi1",
		.platform_data = &ssp1_platform_data,
	},
	.res = {
		.start = 0xF2003000,
		.end   = 0xF2003027,
		.flags = IORESOURCE_MEM,
	},
	.irq = {INTR_VECT_SSP_CH1, NO_IRQ },
	.periphid = 0x00041022,
};
#endif
#endif //defined CONFIG_SPI_PL022 || defined CONFIG_SPI_PL022_MODULE

#ifdef CONFIG_ARM_AMBA
static struct amba_device *amba_devs[] __initdata = {
#ifdef CONFIG_SPI_PL022_CHANNEL_0
	&pl022_device0,
#endif        
#ifdef CONFIG_SPI_PL022_CHANNEL_1
	&pl022_device1,
#endif
#ifdef CONFIG_SERIAL_AMBA_PL011_CHANNEL_0
	&pl011_device0,
#endif
#ifdef CONFIG_SERIAL_AMBA_PL011_CHANNEL_1
	&pl011_device1,
#endif        
#ifdef CONFIG_SERIAL_AMBA_PL011_CHANNEL_2
	&pl011_device2,
#endif        
};
#endif

/*
 * Touchscreen
 */
#if defined CONFIG_TOUCHSCREEN_TMPA9XX || defined CONFIG_TOUCHSCREEN_TMPA9XX_MODULE
static struct tmpa9xx_ts_platforminfo tmpa9xx_info_ts = {
		.fuzz       = 0,
		.rate       = 100,
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

struct platform_device tmpa9xx_device_ts = {
	.name		= "tmpa9xx-ts",
	.id		= -1,
	.dev = {
		.platform_data = &tmpa9xx_info_ts,
	},
	.resource	= tmpa9xx_resource_ts,
	.num_resources	= ARRAY_SIZE(tmpa9xx_resource_ts),
};
#endif

/* 
 * LCD controller device 
 */
#if defined CONFIG_FB_TMPA9XX || defined CONFIG_FB_TMPA9XX_MODULE
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

static struct tmpa9xx_lcdc_platforminfo topas910_v1_lcdc_platforminfo;

struct platform_device tmpa9xx_device_lcdc= {
	.name		= "tmpa9xx-fb",
	.id		= -1,
	.resource	= tmpa9xx_resource_lcdc,
	.num_resources	= ARRAY_SIZE(tmpa9xx_resource_lcdc),
        .dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
        },
};
#endif

static u64 tmpa9xx_device_lcdda_dmamask = 0xffffffffUL;
static struct resource tmpa9xx_lcdda_resource[] = {
	[0] = {
		.start = 0xF2050000,
		.end   = 0xF2050000 + 0x4000,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INTR_VECT_LCDDA,
		.end   = INTR_VECT_LCDDA,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tmpa9xx_device_lcdda = {
	.name = "tmpa9xx-lcdda",
	.id = -1,
	.num_resources = ARRAY_SIZE(tmpa9xx_lcdda_resource),
	.resource = tmpa9xx_lcdda_resource,
	.dev              = {
		.dma_mask		= &tmpa9xx_device_lcdda_dmamask,
		.coherent_dma_mask	= 0xffffffffUL
	}
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
#if defined CONFIG_MTD_NAND_TMPA9XX || defined CONFIG_MTD_NAND_TMPA9XX_MODULE
static struct resource tmpa9xx_nand_resources[] = {
	[0] = {
		.start	=  NANDF_BASE,
		.end	=  NANDF_BASE + 0x200,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device tmpa9xx_nand_device = {
	.name		= "tmpa9xx-nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(tmpa9xx_nand_resources),
	.resource	= tmpa9xx_nand_resources,
};
#endif

/*
 * Real Time Clock
 */
#if defined CONFIG_RTC_DRV_TMPA9XX || defined CONFIG_RTC_DRV_TMPA9XX_MODULE
static struct resource tmpa9xx_resource_rtc[] = {
	{
		.start = RTC_BASE,
		.end   = RTC_BASE + 0xff,
		.flags = IORESOURCE_MEM
	},{
		.start = RTC_BASE + 0x200,
		.end   = RTC_BASE + 0x2ff,
		.flags = IORESOURCE_MEM
	},{
		.start = INTR_VECT_RTC,
		.end   = INTR_VECT_RTC,
		.flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH
	}
};

static struct platform_device tmpa9xx_device_rtc = {
	.name           = "tmpa9xx-rtc",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(tmpa9xx_resource_rtc),
	.resource       = tmpa9xx_resource_rtc
	}
;
#endif

/*
 * Melody / Arlarm
 */
#if defined CONFIG_TMPA9XX_MLDALM || defined CONFIG_TMPA9XX_MLDALM_MODULE
static struct resource tmpa9xx_resource_mldalm[] = {
	{
		.start = RTC_BASE + 0x100,
		.end   = RTC_BASE + 0x1ff,
		.flags = IORESOURCE_MEM
	}
};

static struct platform_device tmpa9xx_device_mldalm = {
	.name           = "tmpa9xx-mldalm",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(tmpa9xx_resource_mldalm),
	.resource       = tmpa9xx_resource_mldalm
	}
;
#endif

/*
 * USB Host Controller
 */
#if defined CONFIG_USB_OHCI_HCD_TMPA9XX || defined CONFIG_USB_OHCI_HCD_TMPA9XX_MODULE
static struct resource tmpa9xx_ohci_resources[] = {
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

static struct platform_device tmpa9xx_ohci_device = {
        .name           = "tmpa9xx-usb",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(tmpa9xx_ohci_resources),
        .resource       = tmpa9xx_ohci_resources,
        .dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
        },
};
#endif /* CONFIG_USB_OHCI_HCD_TMPA9XX */


/*
 * USB Device Controller
 */
#if defined CONFIG_USB_GADGET_TMPA9XX || defined CONFIG_USB_GADGET_TMPA9XX_MODULE
static struct resource tmpa9xx_udc_resource[] = {
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

static struct platform_device tmpa9xx_udc_device = {
        .name           = "tmpa9xx-udc",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(tmpa9xx_udc_resource),
        .resource       = tmpa9xx_udc_resource,
        .dev            = {
        .platform_data  = NULL,
        }
};
#endif

/*
 * Watchdog
 */
#if defined CONFIG_TMPA9XX_WATCHDOG || defined CONFIG_TMPA9XX_WATCHDOG_MODULE
static struct resource tmpa9xx_wdt_resource[] = {
        [0] = {
                .start = 0xf0010000,
                .end   = 0xf0010c04,
                .flags = IORESOURCE_MEM
        },
};

static struct platform_device tmpa9xx_wdt_device = {
        .name           = "tmpa9xx-wdt",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(tmpa9xx_wdt_resource),
        .resource       = tmpa9xx_wdt_resource,
        .dev            = {
        .platform_data  = NULL,
        }
};
#endif

static struct resource tmpa9xx_pwm_resource[] = {
	[0] = {
		.start = TMPA9XX_TIMER2,
		.end   = TMPA9XX_TIMER2 + 0x0fff,
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

static struct platform_device tmpa9xx_i2s_device = {
	.name = "WM8976-I2S",
	.id   = -1,
};


static struct platform_device *devices[] __initdata = {
#if defined CONFIG_NET_ETHERNET || defined CONFIG_NET_ETHERNET_MODULE
	&topas910_dm9000_device,
#endif

#if defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE
	&tmpa9xx_device_i2c,
#endif

#if defined CONFIG_MMC_TMPA9XX_SDHC || defined CONFIG_MMC_TMPA9XX_SDHC_MODULE
 	&tmpa9xx_device_sdhc,
#endif

#if defined CONFIG_TOUCHSCREEN_TMPA9XX || defined CONFIG_TOUCHSCREEN_TMPA9XX_MODULE
	&tmpa9xx_device_ts,
#endif

#if defined CONFIG_FB_TMPA9XX || defined CONFIG_FB_TMPA9XX_MODULE
	&tmpa9xx_device_lcdc,
#endif

#if defined CONFIG_MTD_NAND_TMPA9XX || defined CONFIG_MTD_NAND_TMPA9XX_MODULE
 	&tmpa9xx_nand_device,
#endif

#if defined CONFIG_RTC_DRV_TMPA9XX || defined CONFIG_RTC_DRV_TMPA9XX_MODULE
	&tmpa9xx_device_rtc,
#endif

#if defined CONFIG_TMPA9XX_MLDALM || defined CONFIG_TMPA9XX_MLDALM_MODULE
	&tmpa9xx_device_mldalm,
#endif

#if defined CONFIG_USB_OHCI_HCD_TMPA9XX || defined CONFIG_USB_OHCI_HCD_TMPA9XX_MODULE
	&tmpa9xx_ohci_device,
#endif       

#if defined CONFIG_USB_GADGET_TMPA9XX || defined CONFIG_USB_GADGET_TMPA9XX_MODULE
	&tmpa9xx_udc_device,
#endif
#if defined CONFIG_TMPA9XX_WATCHDOG || defined CONFIG_TMPA9XX_WATCHDOG_MODULE
	&tmpa9xx_wdt_device,
#endif
#if defined CONFIG_SND_TMPA9XX_WM8983 || defined CONFIG_SND_TMPA9XX_WM8983_MODULE || defined CONFIG_SND_SOC_TMPA9XX_I2S
 	&tmpa9xx_i2s_device,	
#endif
	&tmpa9xx_pwm_device,
#if defined CONFIG_FB_TMPA9XX || defined CONFIG_FB_TMPA9XX_MODULE
	&tmpa9xx_device_lcdda,
#endif
	&topas910_led_device,
    &topas910_keys_device,

};

static void __init setup_lcdc_device(void)
{
	uint32_t *LCDReg;
	int width  = 320;
	int height = 240;
	
	LCDReg = topas910_v1_lcdc_platforminfo.LCDReg;
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

	tmpa9xx_device_lcdc.dev.platform_data = &topas910_v1_lcdc_platforminfo;
}


void __init topasa900_init_irq(void) {
	tmpa9xx_init_irq();
}

/*
 * TopasA900 device initialisation
 */
static void __init topasa900_init(void)
{
#ifdef CONFIG_ARM_AMBA
	int i;
#endif        

	/* DMA setup */
	platform_bus.coherent_dma_mask = DMA_BIT_MASK(32);
	platform_bus.dma_mask=DMA_BIT_MASK(32);
	
	/* Pin configuration */
        
	/* Port A can be used not only as a general-purpose input pin with pull up but also as key input pin. */
	TMPA9XX_CFG_PORT_GPIO(PORTA); /* All useable for GPIO */
        
	/* Port B can be used not only as general-purpose output pins but also as key output pins. */
	TMPA9XX_CFG_PORT_GPIO(PORTB); 
	GPIOBODE = 0x00; /* Disable Open Drain */
        
	/* Port C
	   The upper 2 bits (bits [7:6]) of Port C can be used as general-purpose input/output pins
	   and the lower 3 bits (bits [4:2]) can be used as general-purpose output pins.
	   Port C can also be used as interrupt (INT9), I2C (I2C0DA, I2C0CL), low-frequency clock
	   output (FSOUT), melody output (MLDALM), PWM output function (PWM0OUT,
	   PWM2OUT), and USB Host power supply control function (USBOCn, USBPON). */

#if defined CONFIG_USB_OHCI_HCD_TMPA9XX || defined CONFIG_USB_OHCI_HCD_TMPA9XX_MODULE
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
#endif
#if defined CONFIG_TMPA9XX_MLDALM || defined CONFIG_TMPA9XX_MLDALM_MODULE
	GPIOCFR1 |=  (0x1<<3);
	GPIOCFR2 &= ~(0x1<<3);
	GPIOCIE  &= ~(0x1<<3);
	GPIOCODE &= ~(0x1<<3);
#endif
        
#if !defined CONFIG_USB_OHCI_HCD_TMPA9XX && !defined CONFIG_USB_OHCI_HCD_TMPA9XX_MODULE \
 && !defined CONFIG_TMPA9XX_MLDALM       && !defined CONFIG_TMPA9XX_MLDALM_MODULE
	TMPA9XX_CFG_PORT_GPIO(PORTC);
	GPIOCODE = 0x00; 
	GPIOCDIR = 0xFF;
	GPIOCFR1 = 0;
	GPIOCFR2 = 0;
	GPIOCDATA = 0x00;
#endif

#if defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE
	/* set PORT-C 6,7 to I2C and enable open drain */
	GPIOCFR1 |= 0xc0;
	GPIOCFR2 &= ~(0xc0);
	GPIOCODE |= 0xc0;
#endif


       	GPIOTFR1 = 0xFF;  /* USB, SPI0 and UART 1 */

	/* Port D can be used as general-purpose input.
	   Port D can also be used as interrupt (INTB, INTA), ADC (AN7-AN0), and touch screen
	   control (PX, PY, MX, MY) pins. */
#if defined CONFIG_TOUCHSCREEN_TMPA9XX || defined CONFIG_TOUCHSCREEN_TMPA9XX_MODULE
	GPIODFR1 |= 0x0;
	GPIODFR2 |= 0xf0;
	GPIODIE = 0x00;
#endif

#if defined CONFIG_TMPA9XX_ADC || defined CONFIG_TMPA9XX_ADC_MODULE
	GPIODFR1 |= 0x0f;
	GPIODFR2 |= 0x00;
	GPIODIE = 0x00;
#endif

#if  ( defined CONFIG_TMPA9XX_ADC         ||  defined CONFIG_TMPA9XX_ADC_MODULE) \
   &&(!defined CONFIG_TOUCHSCREEN_TMPA9XX && !defined CONFIG_TOUCHSCREEN_TMPA9XX_MODULE)
	GPIODFR1 |= 0xf0;
	GPIODFR2 |= 0x00;
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
#if defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE
	/* set PORT-C 6,7 to I2C and enable open drain */
	GPIOFDIR  &= ~(0xc0);
	GPIOFFR1 |= 0xc0;
	GPIOFFR2 &= ~(0xc0);
	GPIOFIE  &= ~0xC0;
	GPIOFODE |= 0xc0;
#endif

	/* Port G can be used as general-purpose input/output pins.
	   Port G can also be used as SD host controller function pins (SDC0CLK, SDC0CD,
	   SDC0WP, SDC0CMD, SDC0DAT3, SDC0DAT2, SDC0DAT1 and SDC0DAT0). */
#if defined CONFIG_MMC_TMPA9XX_SDHC || !defined CONFIG_MMC_TMPA9XX_SDHC_MODULE
	GPIOGFR1 = 0xFF;
#else
	TMPA9XX_CFG_PORT_GPIO(PORTG); /* SDIO0 or GPIO */
#endif

	/* Port J can be used as general-purpose input/output pins.
	   Port J can also be used as LCD cotroller function pins (LD15-LD8) and CMOS image
	   sensor control (CMSVSY, CMSHBK, CMSHSY and CMSPCK) pins. */
	/* Port K can be used as general-purpose input/output pins.
	   Port K can also be used as LCD controller function pins (LD23 to LD16) and CMOS image
	   sensor control (CMSD7 toCMSD0) pins. */
#if defined CONFIG_FB_TMPA9XX || defined CONFIG_FB_TMPA9XX_MODULE
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
	   I2S0CLK and I2S0WS) and SPI function (SP1DI, SP1DO, SP1CLK and SP1FSS) pins. */
#if defined CONFIG_SND_TMPA9XX_WM8983 || defined CONFIG_SND_TMPA9XX_WM8983_MODULE || defined CONFIG_SND_SOC_TMPA9XX_I2S
	GPIOLFR1 |= 0x1f; /* bits 4:0 for I2S */
#endif        
#ifdef CONFIG_SPI_PL022_CHANNEL_1
	GPIOLFR2 |= 0x0F;
#endif
	/* Port M can be used as general-purpose input/output pins. (Bits [7:4] are not used.)
	   Port M can also be used as I2S function pins (I2S1MCLK, I2S1DATO, I2S1CLK and
	   I2S1WS).*/
	GPIOMDIR |= 0x03; /* M0, MI GPIO OUT */
#if defined CONFIG_SND_TMPA9XX_WM8983 || defined CONFIG_SND_TMPA9XX_WM8983_MODULE || defined CONFIG_SND_SOC_TMPA9XX_I2S
	GPIOMFR1 &= ~0x03;
	GPIOMFR1 |= 0x04; /* M2 I2S1DAT0 */
#endif
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
	GPIOTFR1 |= (0x07 << 4);
#endif
#ifdef CONFIG_SPI_PL022_CHANNEL_0
	GPIOTFR1 |= 0x0F;
#endif
   
#if defined CONFIG_FB_TMPA9XX || defined CONFIG_FB_TMPA9XX_MODULE
	/* Configure LCD interface */
	setup_lcdc_device();
#endif

	/* NAND Controller */
	NDFMCR0 = 0x00000010; // NDCE0n pin = 0, ECC-disable
	NDFMCR1 = 0x00000000; // ECC = Hamming
	NDFMCR2 = 0x00003343; // NDWEn L = 3clks,H =3clks,
							// NDREn L = 4clks,H = 3clks
	NDFINTC = 0x00000000; // ALL Interrupt Disable

	/* Register the active AMBA devices on this board */
#ifdef CONFIG_ARM_AMBA
	for (i= 0; i < ARRAY_SIZE(amba_devs); i++)
        {
		amba_device_register(amba_devs[i], &iomem_resource);
	}
#endif        
	/* Add devices */
	platform_add_devices(devices, ARRAY_SIZE(devices));

#if defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE
	i2c_register_board_info(0, topasa900_i2c_0_devices,
			ARRAY_SIZE(topasa900_i2c_0_devices));

	i2c_register_board_info(1, topasa900_i2c_1_devices,
			ARRAY_SIZE(topasa900_i2c_1_devices));
#endif
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_MMC_SPI)
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif
}

extern struct sys_timer tmpa9xx_timer;

MACHINE_START(TOPASA900, "Toshiba TopasA900")
        /* Maintainer:  Florian Boor <florian.boor@kernelconcepts.de> */
        .phys_io        = TMPA9XX_IO_PHYS_BASE,
        .boot_params    = 0,
        .io_pg_offst    = (io_p2v(TMPA9XX_IO_PHYS_BASE) >> 18) & 0xfffc,
        .map_io         = topas910_map_io,
        .init_irq       = topas910_init_irq,
        .timer          = &tmpa9xx_timer,
        .init_machine   = topasa900_init,
MACHINE_END

