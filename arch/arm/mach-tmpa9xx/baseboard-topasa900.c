/*
 *  arch/arm/mach-tmpa9xx/baseboard-topasa900.c
 *
 * Copyright (C) 2010 Thomas Haase <Thomas.Haase@web.de>
 * Based on former topas910.c
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
#include <linux/spi/mmc_spi.h>
#include <linux/mmc/host.h>
#include <linux/gpio_keys.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/amba/pl022.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/regs.h>
#include <mach/platform.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

/*
 * Ethernet
 */
#if defined CONFIG_NET_ETHERNET || defined CONFIG_NET_ETHERNET_MODULE
static struct resource dm9000_resources[] = {
	{
		.start	= 0x60000002,
		.end	= 0x60000003,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 0x60001002,
		.end	= 0x60001003,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= INT_GPIO_INTH,
		.end	= INT_GPIO_INTH,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
	},
};


static struct platform_device topas_dm9000_device = {
	.name		= "dm9000",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(dm9000_resources),
	.resource	= dm9000_resources,
	.dev = {
		.coherent_dma_mask = 0xffffffff,
	},
};
#endif

/*
 * I2C
 */
#if defined CONFIG_I2C_TMPA9XX_CHANNEL_0
static struct i2c_board_info baseboard_i2c_0_devices[] = {
	/* no devices */
};
#endif

#if defined CONFIG_I2C_TMPA9XX_CHANNEL_1
static struct i2c_board_info baseboard_i2c_1_devices[] = {
#if defined CONFIG_SND_TMPA9XX_I2S || defined CONFIG_SND_TMPA9XX_I2S_MODULE
	{
		I2C_BOARD_INFO("wm89xx", 0x1a),
	},
#endif
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
	.com_mode	= INTERRUPT_TRANSFER,
	.iface		= SSP_INTERFACE_MOTOROLA_SPI,
	/* we can act as master only */
	.hierarchy	  = SSP_MASTER,
	.slave_tx_disable = 0,
	.rx_lev_trig	  = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig	  = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.cs_control	  = tmpa9xx_spi0_cs_control,
};
#endif

#if defined CONFIG_SPI_PL022_CHANNEL_0
struct pl022_config_chip spidev0_info = {
	.com_mode	= INTERRUPT_TRANSFER,
	.iface		= SSP_INTERFACE_MOTOROLA_SPI,
	/* we can act as master only */
	.hierarchy	  = SSP_MASTER,
	.slave_tx_disable = 0,
	.rx_lev_trig	  = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig	  = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.cs_control	  = tmpa9xx_spi0_cs_control,
};
#endif

#if defined CONFIG_SPI_PL022_CHANNEL_1
struct pl022_config_chip spidev1_info = {
	.com_mode	= INTERRUPT_TRANSFER,
	.iface		= SSP_INTERFACE_MOTOROLA_SPI,
	/* we can act as master only */
	.hierarchy	  = SSP_MASTER,
	.slave_tx_disable = 0,
	.rx_lev_trig	  = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig	  = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.cs_control	  = tmpa9xx_spi1_cs_control,
};
#endif

#if (defined CONFIG_MMC_SPI || defined CONFIG_MMC_SPI_MODULE) && defined CONFIG_SPI_PL022_CHANNEL_0
static struct mmc_spi_platform_data mmc_spi_info = {
	.caps		= MMC_CAP_NEEDS_POLL | MMC_CAP_SPI,
	.ocr_mask	= MMC_VDD_32_33	| MMC_VDD_33_34, /* 3.3V only */
};

static struct spi_board_info spi_board_info[] = {
#ifdef CONFIG_SPI_PL022_CHANNEL_0
{
	.modalias	= "mmc_spi",
	.controller_data = &mmc_info,
	.platform_data	= &mmc_spi_info,
	.mode		= SPI_MODE_0,
	.chip_select	= 0,
	.max_speed_hz	= 20000000,
	.bus_num	= 0,

},
#endif
#ifdef CONFIG_SPI_PL022_CHANNEL_1
{
	.modalias	= "spidev",
	.controller_data = &spidev1_info,
	.platform_data	= NULL,
	.mode		= SPI_MODE_0,
	.chip_select	= 0,
	.max_speed_hz 	= 10000000,
	.bus_num	= 1,
}
#endif
};
#elif defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
static struct spi_board_info spi_board_info[] = {
#ifdef CONFIG_SPI_PL022_CHANNEL_0
{
	.modalias	= "spidev",
	.controller_data = &spidev0_info,
	.platform_data	= NULL,
	.mode		= SPI_MODE_0,
	.chip_select	= 0,
	.max_speed_hz	= 10000000,
	.bus_num	= 0,
},
#endif
#ifdef CONFIG_SPI_PL022_CHANNEL_1
{
	.modalias	= "spidev",
	.controller_data = &spidev1_info,
	.platform_data	= NULL,
	.mode		= SPI_MODE_0,
	.chip_select	= 0,
	.max_speed_hz	= 10000000,
	.bus_num	= 1,
}
#endif
};
#endif

#endif

/*
 * 7 segment LED display
 */

static struct platform_device topas_led_device = {
	.name	= "led-topas",
	.id	= -1,
};

/*
 * Joystick
 */

static struct gpio_keys_button topas_buttons[] = {
	{
		.code		= KEY_LEFTSHIFT,
		.gpio		= 0,
		.active_low	= 1,
		.desc		= "Joystick Bit 0",
		.type		= EV_KEY,
		.wakeup		= 1,
	}, {
		.code		= KEY_LEFTCTRL,
		.gpio		= 1,
		.active_low	= 1,
		.desc		= "Joystick Bit 1",
		.type		= EV_KEY,
		.wakeup		= 1,
	}, {
		.code		= KEY_LEFTALT,
		.gpio		= 2,
		.active_low 	= 1,
		.desc		= "Joystick Bit 2",
		.type		= EV_KEY,
		.wakeup		= 1,
	}, {
		.code		= KEY_SELECT,
		.gpio		= 3,
		.active_low	= 1,
		.desc		= "Joystick strobe",
		.type		= EV_KEY,
		.wakeup		= 1,
	},
};


static struct gpio_keys_platform_data topas_keys_data = {
	.buttons	= topas_buttons,
	.nbuttons	= ARRAY_SIZE(topas_buttons),
};

static struct platform_device topas_keys_device = {
	.name	= "gpio-keys",
	.id 	= -1,
	.dev 	= {
		 .platform_data = &topas_keys_data,
	},
};

static struct platform_device *devices_baseboard[] __initdata = {
#if defined CONFIG_NET_ETHERNET || defined CONFIG_NET_ETHERNET_MODULE
	&topas_dm9000_device,
#endif
	&topas_led_device,
	&topas_keys_device,
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
	.tim2		= TIM2_IPC | PIX_CLOCK_DIVIDER,
	.cntl		= CNTL_BGR | CNTL_LCDTFT | CNTL_WATERMARK,
	.bpp		= 32,
};

struct tmpa9xx_panel_ts_info tmpa9xx_panels[] = {
	{
		.panel = &tmpa9xx_panel,
		.fuzz = 16,
		.rate = 200,
	}, {
		.panel = NULL,
	},
};

struct tmpa9xx_i2s_cfg tmpa9xx_i2s_cfg =
{
	.is_master_rx = false,
	.is_master_tx = false,
	.common_rx_tx_clock = true,
};

#if defined CONFIG_I2C_TMPA9XX_CHANNEL_0
extern struct platform_device tmpa9xx_device_i2c_channel_0;
#endif
#if defined CONFIG_I2C_TMPA9XX_CHANNEL_1
extern struct platform_device tmpa9xx_device_i2c_channel_1;
#endif

static int setup_port_l(void)
{
	/* Port L can be used as general-purpose input/output pins. (Bits [7:5] are not used.)
	   In addition, Port L can also be used as I2S function (I2SSCLK, I2S0MCLK, I2S0DATI,
	   I2S0CLK and I2S0WS) and SPI function (SP1DI, SP1DO, SP1CLK and SP1FSS) pins. */
#if (defined CONFIG_SND_TMPA9XX_I2S || defined CONFIG_SND_TMPA9XX_I2S_MODULE) && defined CONFIG_SPI_PL022_CHANNEL_1
#error "port l configuration mismatch. spi channel 1 vs. i2s channel 0"
#endif

#if defined CONFIG_SND_TMPA9XX_I2S || defined CONFIG_SND_TMPA9XX_I2S_MODULE
#if defined CONFIG_SPI_PL022_CHANNEL_1
#error "port l configuration mismatch. spi channel 1 vs. i2s channel 0"
#endif
	GPIOLFR1 |= (0x1f); /* bits 4:0 for I2S */
#endif

#if defined CONFIG_SPI_PL022_CHANNEL_1
#if defined CONFIG_SND_TMPA9XX_I2S || defined CONFIG_SND_TMPA9XX_I2S_MODULE
#error "port l configuration mismatch. spi channel 1 vs. i2s channel 0"
#endif
	GPIOLFR2 |= (0x0f);
#endif
	return 0;
}

static int setup_port_m(void)
{
	/* Port M can be used as general-purpose input/output pins. (Bits [7:4] are not used.)
	   Port M can also be used as I2S function pins (I2S1MCLK, I2S1DATO, I2S1CLK and
	   I2S1WS).*/
#if defined CONFIG_SND_TMPA9XX_I2S || defined CONFIG_SND_TMPA9XX_I2S_MODULE
	GPIOMFR1 |= (0x04); /* M2 I2S1DAT0 */
#endif
	return 0;
}

void __init baseboard_init(void)
{
	setup_port_l();
	setup_port_m();

#if defined CONFIG_I2C_TMPA9XX_CHANNEL_0
	tmpa9xx_device_i2c_channel_0.dev.platform_data = (void *)400;
	i2c_register_board_info(0, baseboard_i2c_0_devices, ARRAY_SIZE(baseboard_i2c_0_devices));
#endif
#if defined CONFIG_I2C_TMPA9XX_CHANNEL_1
	tmpa9xx_device_i2c_channel_1.dev.platform_data = (void *)400;
	i2c_register_board_info(1, baseboard_i2c_1_devices, ARRAY_SIZE(baseboard_i2c_1_devices));
#endif
#if defined CONFIG_SPI_SPIDEV || defined CONFIG_SPI_SPIDEV_MODULE \
 || defined CONFIG_MMC_SPI    || defined CONFIG_MMC_SPI_MODULE
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif

	/* Add devices */
	platform_add_devices(devices_baseboard, ARRAY_SIZE(devices_baseboard));
}
