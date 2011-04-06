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
#include <mach/regs.h>
#include <mach/platform.h>
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

static struct resource tmpa900_resource_lcdcop[] = {
        {
		.start = LCDCOP_BASE,
		.end   = LCDCOP_BASE + SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device tmpa900_lcdcop_device = {
        .name          = "tmpa900-lcdcop",
        .id            = -1,
        .resource      = tmpa900_resource_lcdcop,
        .num_resources = ARRAY_SIZE(tmpa900_resource_lcdcop),
        .dev           = {
                .coherent_dma_mask 	= ~0,
		.platform_data		= NULL,
        }
};

#if defined CONFIG_BACKLIGHT_PWM || defined CONFIG_BACKLIGHT_PWM_MODULE
static struct platform_pwm_backlight_data backlight_data = {
	.name            = "tmpa9xx-pwm:1",
	.max_brightness  = 16368, /* usable range is below 1023, needs investigation */
	.dft_brightness  = 16368,
	.pwm_period_ns   = 341000, /* fixme: depends on the clock of the pwm */
};

static struct platform_device backlight_device = {
	.name = "pwm-backlight",
	.dev  = {
		.platform_data  = &backlight_data,
	},
};
#endif

static struct platform_device *devices_baseboard[] __initdata = {
#if defined CONFIG_BACKLIGHT_PWM || defined CONFIG_BACKLIGHT_PWM_MODULE
        &backlight_device,
#endif
#if defined CONFIG_SND_TMPA910_WM8974 || defined CONFIG_SND_TMPA910_WM8974_MODULE || defined CONFIG_SND_SOC_TMPA9XX_I2S
        &baseboard_i2s_device,    
#endif
	&tmpa900_lcdcop_device,
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

struct tmpa9xx_i2s_cfg tmpa9xx_i2s_cfg =
{
	.is_master_rx = false,
	.is_master_tx = false,
	.common_rx_tx_clock = true,
};

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
        GPIOMFR1 |=  (0x04); /* M2 I2S1DAT0 */
#endif
	return 0;
}

void __init baseboard_init(void)
{
	setup_port_l();
	setup_port_m();

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
