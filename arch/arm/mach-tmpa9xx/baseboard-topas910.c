/*
 *  arch/arm/mach-tmpa9xx/baseboard-topas910.c 
 *
 * Copyright (C) 2010 Thomas Haase <Thomas.Haase@web.de>
 * Based on former topa910.c
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
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <linux/gpio_keys.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/amba/pl022.h>

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
                .start  = TOPAS_INT_DM9000,
                .end    = TOPAS_INT_DM9000,
                .flags  = IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
        },
};


static struct platform_device topas_dm9000_device = {
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
 * I2C
 */
#if defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE
static struct i2c_board_info baseboard_i2c_0_devices[] = {
	/* no devices */
};

static struct i2c_board_info baseboard_i2c_1_devices[] = {
	/* no devices */
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
struct pl022_config_chip spidev0_info = {
        .com_mode         = INTERRUPT_TRANSFER,
        .iface            = SSP_INTERFACE_MOTOROLA_SPI,
        /* we can act as master only */
        .hierarchy        = SSP_MASTER,
        .slave_tx_disable = 0,
        .rx_lev_trig      = SSP_RX_1_OR_MORE_ELEM,
        .tx_lev_trig      = SSP_TX_1_OR_MORE_EMPTY_LOC,
        .cs_control       = tmpa9xx_spi0_cs_control,
};
#endif

#if defined CONFIG_SPI_PL022_CHANNEL_1
struct pl022_config_chip spidev1_info = {
        .com_mode         = INTERRUPT_TRANSFER,
        .iface            = SSP_INTERFACE_MOTOROLA_SPI,
        /* we can act as master only */
        .hierarchy        = SSP_MASTER,
        .slave_tx_disable = 0,
        .rx_lev_trig      = SSP_RX_1_OR_MORE_ELEM,
        .tx_lev_trig      = SSP_TX_1_OR_MORE_EMPTY_LOC,
        .cs_control       = tmpa9xx_spi1_cs_control,
};
#endif

#if defined CONFIG_SPI_SPIDEV || defined CONFIG_SPI_SPIDEV_MODULE
static struct spi_board_info spi_board_info[] = {
#ifdef CONFIG_SPI_PL022_CHANNEL_0
{
        .modalias        = "spidev",
        .controller_data = &spidev0_info,
        .platform_data   = NULL,
        .mode            = SPI_MODE_0,
        .chip_select     = 0,
        .max_speed_hz    = 10000000,
        .bus_num         = 0,
},
#endif
#ifdef CONFIG_SPI_PL022_CHANNEL_1
{
        .modalias        = "spidev",
        .controller_data = &spidev1_info,
        .platform_data   = NULL,
        .mode            = SPI_MODE_0,
        .chip_select     = 0,
        .max_speed_hz    = 10000000,
        .bus_num         = 1,
}
#endif
};
#endif

#endif //defined CONFIG_SPI_PL022 || defined CONFIG_SPI_PL022_MODULE

/* 
 * 7 segment LED display
 */

static struct platform_device topas_led_device = {
        .name = "led-topas",
        .id   = -1,
};

/*
 * Joystick 
 */

static struct gpio_keys_button topas_buttons[] = {
        {
                .code       = KEY_LEFT,
                .gpio       = 0,
                .active_low = 0,
                .desc       = "Joystick left",
                .type       = EV_KEY,
                .wakeup     = 0,
        },
        {
                .code       = KEY_DOWN,
                .gpio       = 1,
                .active_low = 0,
                .desc       = "Joystick down",
                .type       = EV_KEY,
                .wakeup     = 0,
        },
        {
                .code        = KEY_UP,
                .gpio        = 2,
                .active_low  = 0,
                .desc        = "Joystick up",
                .type        = EV_KEY,
                .wakeup      = 0,
        },
        {
                .code        = KEY_RIGHT,
                .gpio        = 3,
                .active_low  = 0,
                .desc        = "Joystick right",
                .type        = EV_KEY,
                .wakeup      = 0,
        },
        {
                .code        = KEY_ENTER,
                .gpio        = 4,
                .active_low  = 0,
                .desc        = "Joystick center",
                .type        = EV_KEY,
                .wakeup      = 1,
        },
};


static struct gpio_keys_platform_data topas_keys_data = {
        .buttons  = topas_buttons,
        .nbuttons = ARRAY_SIZE(topas_buttons),
};

static struct platform_device topas_keys_device = {
        .name = "gpio-keys",
        .id   = -1,
        .dev  = {
                 .platform_data = &topas_keys_data,
        },
};

static struct platform_device baseboard_i2s_device = {
        .name = "WM8976-I2S",
        .id   = -1,
};

static struct platform_device *devices_baseboard[] __initdata = {
#if defined CONFIG_NET_ETHERNET || defined CONFIG_NET_ETHERNET_MODULE
        &topas_dm9000_device,
#endif
        &topas_led_device,
        &topas_keys_device,
        &baseboard_i2s_device,    
};

#define HCLK 			96000000
#define PIX_CLOCK_DIVIDER	16
#define PIX_CLOCK		(HCLK/PIX_CLOCK_DIVIDER)

struct clcd_panel tmpa9xx_panel = {
	.mode = {
		.name		= "TMPA9xx panel",
		.refresh	= 30, /* unused */
		.xres		= 320,
		.yres		= 240,
		.pixclock	= PIX_CLOCK,
		.left_margin	= 8,
		.right_margin	= 8,
		.upper_margin	= 2,
		.lower_margin	= 2,
		.hsync_len	= 16,
		.vsync_len	= 8,
		.sync		= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_IPC | PIX_CLOCK_DIVIDER,
	.cntl		= CNTL_BGR | CNTL_LCDTFT | CNTL_WATERMARK,
	.bpp		= 32,
	.grayscale	= 0,
};

struct tmpa9xx_panel_ts_info tmpa9xx_panels[] = {
	{
		.panel = &tmpa9xx_panel,
		.fuzz = 0,
		.rate = 200,
	},
	{
		.panel = NULL,
	},
};

void __init baseboard_init(void)
{
#if defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE
        i2c_register_board_info(0, baseboard_i2c_0_devices,
                        ARRAY_SIZE(baseboard_i2c_0_devices));

        i2c_register_board_info(1, baseboard_i2c_1_devices,
                        ARRAY_SIZE(baseboard_i2c_1_devices));
#endif
#if defined CONFIG_SPI_SPIDEV || defined CONFIG_SPI_SPIDEV_MODULE
        spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif

        /* Add devices */ 
        platform_add_devices(devices_baseboard, ARRAY_SIZE(devices_baseboard));
}
