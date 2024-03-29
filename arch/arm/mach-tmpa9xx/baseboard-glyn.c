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
#include <linux/spi/mmc_spi.h>
#include <linux/mmc/host.h>
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

#if (defined CONFIG_MMC_SPI || defined CONFIG_MMC_SPI_MODULE) && defined CONFIG_SPI_PL022_CHANNEL_0
static struct mmc_spi_platform_data mmc_spi_info = {
        .caps     = MMC_CAP_NEEDS_POLL | MMC_CAP_SPI,
        .ocr_mask = MMC_VDD_32_33      | MMC_VDD_33_34, /* 3.3V only */
};

static struct spi_board_info spi_board_info[] = {
#ifdef CONFIG_SPI_PL022_CHANNEL_0
{
        .modalias        = "mmc_spi",
        .controller_data = &mmc_info,
        .platform_data   = &mmc_spi_info,
        .mode            = SPI_MODE_0,
        .chip_select     = 0,
        .max_speed_hz    = 16000000,
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
#elif defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
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

#endif /* defined CONFIG_SPI_PL022 || defined CONFIG_SPI_PL022_MODULE */

#if defined CONFIG_BACKLIGHT_PWM || defined CONFIG_BACKLIGHT_PWM_MODULE
static int tonga_backlight_init(struct device *dev)
{
	pr_debug("%s():\n", __func__);
	return 0;
}

static int tonga_backlight_notify(struct device *dev, int brightness)
{
	/* values below 62 result in a voltage > 2.5V. according
	to the datasheet this is not allowed, so we just cap it here */
	if (brightness < 62)
		brightness = 62;

	pr_debug("%s(): brightness %d\n", __func__, brightness);
	return brightness;
}

static void tonga_backlight_exit(struct device *dev)
{
	pr_debug("%s():\n", __func__);
}

static struct platform_pwm_backlight_data tonga_backlight_data = {
	.name            = "tmpa9xx-pwm:1",
	.max_brightness  = 255,
	.dft_brightness  = 255,
	.pwm_period_ns   = 7968750, /* fixme: depends on the clock of the pwm */
	.inverted_polarity = true,
	.init            = tonga_backlight_init,
	.notify          = tonga_backlight_notify,
	.exit            = tonga_backlight_exit,
};

static struct platform_device tonga_backlight_device = {
	.name = "pwm-backlight",
	.dev  = {
		.platform_data  = &tonga_backlight_data,
	},
};
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

static struct platform_device *devices_baseboard[] __initdata = {
#if defined CONFIG_BACKLIGHT_PWM || defined CONFIG_BACKLIGHT_PWM_MODULE
        &tonga_backlight_device,
#endif
#if defined CONFIG_SND_SOC_TMPA9XX_I2S
        &baseboard_i2s_device,    
#endif
	&tmpa900_lcdcop_device,
};

#define HCLK 			96000000
#define PIX_CLOCK		(HCLK/PIX_CLOCK_DIVIDER)

#define PIX_CLOCK_DIVIDER_QVGA 13
static struct clcd_panel qvga = {
	.mode = {
		.name		= "qvga",
		.xres		= 320,
		.yres		= 240,
		.pixclock	= HCLK/PIX_CLOCK_DIVIDER_QVGA,
		.left_margin	= 26,
		.right_margin	= 34,
		.upper_margin	= 16,
		.lower_margin	= 4,
		.hsync_len	= 31,
		.vsync_len	= 4,
		.sync		= 0,
		.vmode  	= 0,
	},
	.tim2		= TIM2_IPC | PIX_CLOCK_DIVIDER_QVGA,
	.cntl		= CNTL_LCDTFT | CNTL_WATERMARK | 0,
	.bpp		= 16,
};

#define PIX_CLOCK_DIVIDER_WQVGA 11
static struct clcd_panel wqvga = {
	.mode = {
		.name		= "wqvga",
		.xres		= 480,
		.yres		= 272,
		.pixclock	= HCLK/PIX_CLOCK_DIVIDER_WQVGA,
		.left_margin	= 11,
		.right_margin	= 16,
		.upper_margin	= 2,
		.lower_margin	= 2,
		.hsync_len	= 21,
		.vsync_len	= 11,
		.sync		= 0,
		.vmode  	= 0,
	},
	.tim2		= TIM2_IPC | PIX_CLOCK_DIVIDER_WQVGA,
	.cntl		= CNTL_LCDTFT | CNTL_WATERMARK | 0,
	.bpp		= 16,
};

#define PIX_CLOCK_DIVIDER_VGA 4
static struct clcd_panel vga = {
	.mode = {
		.name		= "vga",
		.xres		= 640,
		.yres		= 480,
		.pixclock	= HCLK/PIX_CLOCK_DIVIDER_VGA,
		.left_margin	= 102,
		.right_margin	= 30,
		.upper_margin	= 33,
		.lower_margin	= 10,
		.hsync_len	= 31,
		.vsync_len	= 3,
		.sync		= 0,
		.vmode  	= 0,
	},
	.tim2		= TIM2_IPC | PIX_CLOCK_DIVIDER_VGA,
	.cntl		= CNTL_LCDTFT | CNTL_WATERMARK | 0,
	.bpp		= 16,
};

#define PIX_CLOCK_DIVIDER_WVGA 2
static struct clcd_panel wvga = {
	.mode = {
		.name		= "wvga",
		.xres		= 800,
		.yres		= 480,
		.pixclock	= HCLK/PIX_CLOCK_DIVIDER_WVGA,
		.left_margin	= 76,
		.right_margin	= 54,
		.upper_margin	= 33,
		.lower_margin	= 10,
		.hsync_len	= 129,
		.vsync_len	= 3,
		.sync		= FB_SYNC_HOR_HIGH_ACT | 0,
		.vmode  	= 0,
	},
	.tim2		= TIM2_IPC | PIX_CLOCK_DIVIDER_WVGA,
	.cntl		= CNTL_LCDTFT | CNTL_WATERMARK | 0,
	.bpp		= 16,
};

struct tmpa9xx_panel_ts_info tmpa9xx_panels[] = {
	{
		.panel = &qvga,
		.fuzz = 0,
		.rate = 200,
		.lcd_power = 97, /* port m1 */
		.lcd_reset = 96, /* port m0 */
	},
	{
		.panel = &wqvga,
		.fuzz = 0,
		.rate = 200,
		.lcd_power = 97, /* port m1 */
		.lcd_reset = 96, /* port m0 */
	},
	{
		.panel = &vga,
		.fuzz = 0,
		.rate = 200,
		.lcd_power = 97, /* port m1 */
		.lcd_reset = 96, /* port m0 */
	},
	{
		.panel = &wvga,
		.fuzz = 0,
		.rate = 200,
		.lcd_power = 97, /* port m1 */
		.lcd_reset = 96, /* port m0 */
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

#if defined CONFIG_SPI_SPIDEV || defined CONFIG_SPI_SPIDEV_MODULE \
 || defined CONFIG_MMC_SPI    || defined CONFIG_MMC_SPI_MODULE
        spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif

        /* Add devices */
        platform_add_devices(devices_baseboard, ARRAY_SIZE(devices_baseboard));

        PMCCTL &= ~PMCCTL_PMCPWE;
        PMCWV1 |= PMCWV1_PMCCTLV;

        udelay(200);
}
