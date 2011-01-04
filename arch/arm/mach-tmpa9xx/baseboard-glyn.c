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
#include <linux/amba/pl022.h>
#include <linux/spi/mmc_spi.h>
#include <linux/mmc/host.h>
#include <linux/pwm_backlight.h>

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
 * I2C
 */
#if defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE
static struct i2c_board_info baseboard_i2c_0_devices[] = {
        {
        },
};

#if defined CONFIG_SND_SOC_TMPA9XX_I2S
static struct i2c_board_info baseboard_i2c_1_devices[] = {
        {
                I2C_BOARD_INFO("wm8983", 0x1a),
        },
};
#else
static struct i2c_board_info baseboard_i2c_1_devices[] = {
        {
        },
};
#endif
#endif // CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE

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
        .lbm              = LOOPBACK_DISABLED,
        .com_mode         = INTERRUPT_TRANSFER,
        .iface            = SSP_INTERFACE_MOTOROLA_SPI,
        /* we can act as master only */
        .hierarchy        = SSP_MASTER,
        .slave_tx_disable = 0,
        .endian_rx        = SSP_RX_MSB,
        .endian_tx        = SSP_TX_MSB,
        .data_size        = SSP_DATA_BITS_8,
        .rx_lev_trig      = SSP_RX_1_OR_MORE_ELEM,
        .tx_lev_trig      = SSP_TX_1_OR_MORE_EMPTY_LOC,
        .clk_phase        = SSP_CLK_SECOND_EDGE,
        .clk_pol          = SSP_CLK_POL_IDLE_HIGH,
        .cs_control       = tmpa9xx_spi0_cs_control,
};
#endif

#if defined CONFIG_SPI_PL022_CHANNEL_0
struct pl022_config_chip spidev0_info = {
        .lbm              = LOOPBACK_DISABLED,
        .com_mode         = INTERRUPT_TRANSFER,
        .iface = SSP_INTERFACE_MOTOROLA_SPI,
        /* we can act as master only */
        .hierarchy        = SSP_MASTER,
        .slave_tx_disable = 0,
        .endian_rx        = SSP_RX_MSB,
        .endian_tx        = SSP_TX_MSB,
        .data_size        = SSP_DATA_BITS_8,
        .rx_lev_trig      = SSP_RX_1_OR_MORE_ELEM,
        .tx_lev_trig      = SSP_TX_1_OR_MORE_EMPTY_LOC,
        .clk_phase        = SSP_CLK_SECOND_EDGE,
        .clk_pol          = SSP_CLK_POL_IDLE_HIGH,
        .cs_control       = tmpa9xx_spi0_cs_control,
};
#endif

#if defined CONFIG_SPI_PL022_CHANNEL_1
struct pl022_config_chip spidev1_info = {
        .lbm              = LOOPBACK_DISABLED,
        .com_mode         = INTERRUPT_TRANSFER,
        .iface            = SSP_INTERFACE_MOTOROLA_SPI,
        /* we can act as master only */
        .hierarchy        = SSP_MASTER,
        .slave_tx_disable = 0,
        .endian_rx        = SSP_RX_MSB,
        .endian_tx        = SSP_TX_MSB,
        .data_size        = SSP_DATA_BITS_8,
        .rx_lev_trig      = SSP_RX_1_OR_MORE_ELEM,
        .tx_lev_trig      = SSP_TX_1_OR_MORE_EMPTY_LOC,
        .clk_phase        = SSP_CLK_SECOND_EDGE,
        .clk_pol          = SSP_CLK_POL_IDLE_HIGH,
        .cs_control       = tmpa9xx_spi1_cs_control,
};
#endif

#if defined CONFIG_MMC_SPI && defined CONFIG_SPI_PL022_CHANNEL_0
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
        .max_speed_hz    = 24000000,
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
#elif defined(CONFIG_SPI_SPIDEV)
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

#if defined CONFIG_BACKLIGHT_PWM
static int tonga_backlight_init(struct device *dev)
{
        int ret=0;
        return ret;
}

static int tonga_backlight_notify(struct device *dev, int brightness)
{
        /* printk(KERN_ERR "tonga_backlight_notify() brightness=%d (-> %d)\n", brightness, !brightness); */
        /* Backlight is on pin port C4 */
        /* we could also power down the LCD or do other things here... */

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
        .pwm_id          = 0,
        .max_brightness  = 100,
        .dft_brightness  = 100,
        .pwm_period_ns   = 255,
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

/* this is for the old alsa/arm/ sound driver */
#if defined CONFIG_SND_TMPA9XX_WM8983 || defined CONFIG_SND_TMPA9XX_WM8983_MODULE
static struct platform_device baseboard_i2s_device = {
        .name = "WM8983-I2S",
        .id   = -1,
};
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
#if defined CONFIG_SND_TMPA910_WM8983 || defined CONFIG_SND_TMPA910_WM8983_MODULE || defined CONFIG_SND_SOC_TMPA9XX_I2S
        &baseboard_i2s_device,    
#endif
};

void __init baseboard_init(void)
{
#if defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE
        i2c_register_board_info(0, baseboard_i2c_0_devices,
                        ARRAY_SIZE(baseboard_i2c_0_devices));

        i2c_register_board_info(1, baseboard_i2c_1_devices,
                        ARRAY_SIZE(baseboard_i2c_1_devices));
#endif
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_MMC_SPI)
        spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif

        /* Add devices */
        platform_add_devices(devices_baseboard, ARRAY_SIZE(devices_baseboard));

	/* Configure Pins and reset LCD */
        GPIOMDIR=3;
        GPIOMFR1=0;
	/* Reset */
	GPIOMDATA=0;
    	udelay(1000);
	GPIOMDATA|=(1<<0);
	/* Enable */
	GPIOMDATA|=(1<<1);
	/* Light */
	GPIOCDATA=0;

        LCDCOP_STN64CR |= LCDCOP_STN64CR_G64_8bit;
        PMCCTL &= ~PMCCTL_PMCPWE;
        PMCWV1 |= PMCWV1_PMCCTLV;
        udelay(200);

}
