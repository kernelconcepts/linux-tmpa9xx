/*
 *  arch/arm/mach-tmpa9xx/baseboard-glyn.c 
 *
 * Copyright (C) 2010 Thomas Haase <Thomas.Haase@web.de>
 * Based on former tonga.c
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
#include <linux/pwm_backlight.h>
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

#if defined CONFIG_I2C_TMPA9XX_CHANNEL_0
static struct i2c_board_info baseboard_i2c_0_devices[] = {
#if defined CONFIG_SND_TMPA9XX_I2S || defined CONFIG_SND_TMPA9XX_I2S_MODULE
        {
                I2C_BOARD_INFO("wm89xx", 0x1a),
        },
#endif
};
#endif

#if defined CONFIG_I2C_TMPA9XX_CHANNEL_1
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
        .iface = SSP_INTERFACE_MOTOROLA_SPI,
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

#endif //defined CONFIG_SPI_PL022 || defined CONFIG_SPI_PL022_MODULE

#if 0
/* this is for the old alsa/arm/ sound driver */
#if defined CONFIG_SND_TMPA9XX_WM8974 || defined CONFIG_SND_TMPA9XX_WM8974_MODULE
static struct platform_device baseboard_i2s_device = {
        .name = "WM8974-I2S",
        .id   = -1,
};
#endif
#endif

/* new Alsa SOC driver */
#if defined CONFIG_SND_SOC_TMPA9XX_I2S
static struct platform_device baseboard_i2s_device = {
        .name = "tmpa9xx-i2s",
        .id   = -1,
};
#endif

static struct platform_device *devices_baseboard[] __initdata = {
#if defined CONFIG_BACKLIGHT_PWM
        &tonga_backlight_device,
#endif
#if defined CONFIG_SND_TMPA910_WM8974 || defined CONFIG_SND_TMPA910_WM8974_MODULE || defined CONFIG_SND_SOC_TMPA9XX_I2S
        &baseboard_i2s_device,    
#endif
};

#define HCLK 			96000000
#define PIX_CLOCK_DIVIDER	11
#define PIX_CLOCK		(HCLK/PIX_CLOCK_DIVIDER)

struct clcd_panel default_panel = {
	.mode = {
		.name		= "tfttimer",
		.refresh	= 30, /* unused */
		.xres           = 480,
		.yres           = 272,
		.pixclock       = PIX_CLOCK,
		.left_margin    = 41,
		.right_margin   = 6,
		.upper_margin   = 8,
		.lower_margin   = 8,
		.hsync_len      = 11,
		.vsync_len      = 11,
		.sync           = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT | 0,
		.vmode          = 0,
	},
	.width          = -1,
	.height         = -1,
	.tim2           = PIX_CLOCK_DIVIDER,
	.cntl           = CNTL_LCDTFT | CNTL_WATERMARK,
	.bpp            = 16,
	.grayscale      = 0,
};

struct tmpa9xx_panel_ts_info tmpa9xx_panels[] = {
	{
		.panel = &default_panel,
		.fuzz = 0,
		.rate = 200,
	},
	{
		.panel = NULL,
	},
};

void __init baseboard_init(void)
{
#if defined CONFIG_I2C_TMPA9XX_CHANNEL_0
        i2c_register_board_info(0, baseboard_i2c_0_devices,
                        ARRAY_SIZE(baseboard_i2c_0_devices));
#endif

#if defined CONFIG_I2C_TMPA9XX_CHANNEL_1
        i2c_register_board_info(1, baseboard_i2c_1_devices,
                        ARRAY_SIZE(baseboard_i2c_1_devices));
#endif

#if defined CONFIG_SPI_SPIDEV || defined CONFIG_SPI_SPIDEV_MODULE
        spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif

        /* Add devices */
        platform_add_devices(devices_baseboard, ARRAY_SIZE(devices_baseboard));

	/* Reset LCD*/
	GPIOMDATA=0;
    	udelay(1000);
	GPIOMDATA|=(1<<0);
	/* Enable */
	GPIOMDATA|=(1<<1);
	/* Light */
	GPIOCDATA=0;

        /* DE (Display Enable) Pin on Tonga2 board enable */
        GPIOVDIR =(1<<7);
        GPIOVDATA=(1<<7);

	LCDCOP_STN64CR |= LCDCOP_STN64CR_G64_8bit;
	PMCCTL &= ~PMCCTL_PMCPWE;
	PMCWV1 |= PMCWV1_PMCCTLV;
	udelay(200);

	/* Free Port N for other usage than TxD RxD */
        /* Switching N2/N3 to high for mlad_alm output enable */
        GPIONDIR  = (0x0c);
        GPIONDATA = (0x0c);
        GPIONFR1  = (0x01);
        GPIONFR2  = (0x02);
        GPIONIE   = (0x00);
        
}
