/*
 *  arch/arm/mach-tmpa9xx/tmpa9xx-generic.c 
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/mmc_spi.h>
#include <linux/mmc/host.h>
#include <linux/i2c.h>
#include <linux/pwm_backlight.h>
#include <linux/dma-mapping.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>
#include <linux/amba/clcd.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/regs.h>
#include <mach/dma.h>
#include <mach/platform.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#define ARM_UART_PERIPH_ID  0x00041011
#define ARM_SSP_PERIPH_ID   0x00041022
#define ARM_WDT_PERIPH_ID   0x00141805
/*
 * Memory Map Description
 */
static struct map_desc tmpa9xx_io_desc[] __initdata = {
        {
         .pfn     = __phys_to_pfn(TMPA9XX_IO_PHYS_BASE),
         .virtual = TMPA9XX_IO_VIRT_BASE,
         .length  = TMPA9XX_IO_SIZE,
         .type    = MT_DEVICE,
        }
};

void __init tmpa9xx_map_io(void)
{
        iotable_init(tmpa9xx_io_desc, ARRAY_SIZE(tmpa9xx_io_desc));
}

/********************
 * AMBA Bus Devices * 
 ********************/

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
                .start = UART0_BASE_ADDRESS,
                .end   = UART0_BASE_ADDRESS + SZ_4K -1,
                .flags = IORESOURCE_MEM
               },
        .irq = {INTR_VECT_UART_CH0, NO_IRQ},
        .periphid = ARM_UART_PERIPH_ID,
};
#endif

#ifdef CONFIG_SERIAL_AMBA_PL011_CHANNEL_1
struct amba_device pl011_device1 = {
        .dev = {
                .init_name = "uart1",
                .platform_data = NULL,
        },
        .res = {
                .start = UART1_BASE_ADDRESS,
                .end   = UART1_BASE_ADDRESS + SZ_4K -1,
                .flags = IORESOURCE_MEM
               },
        .irq = {INTR_VECT_UART_CH1, NO_IRQ},
        .periphid = ARM_UART_PERIPH_ID,
};

#endif

#ifdef CONFIG_SERIAL_AMBA_PL011_CHANNEL_2
struct amba_device pl011_device2 = {
        .dev = {
                .init_name = "uart2",
                .platform_data = NULL,
        },
        .res = {
                .start = UART2_BASE_ADDRESS,
                .end   = UART2_BASE_ADDRESS + SZ_4K -1,
                .flags = IORESOURCE_MEM
               },
        .irq = {INTR_VECT_UART_CH2, NO_IRQ},
        .periphid = ARM_UART_PERIPH_ID,
};
#endif

#endif // defined SERIAL_AMBA_PL011 || defined SERIAL_AMBA_PL011_MODULE

/*
 * SPI
 */
#if defined CONFIG_SPI_PL022 || defined CONFIG_SPI_PL022_MODULE

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
                .start = SSP0_BASE_ADDRESS,
                .end   = SSP0_BASE_ADDRESS + 0x27,
                .flags = IORESOURCE_MEM,
        },
        .irq = {INTR_VECT_SSP_CH0, NO_IRQ },
        .periphid = ARM_SSP_PERIPH_ID,
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
                .start = SSP1_BASE_ADDRESS,
                .end   = SSP1_BASE_ADDRESS + 0x27,
                .flags = IORESOURCE_MEM,
        },
        .irq = {INTR_VECT_SSP_CH1, NO_IRQ },
        .periphid = ARM_SSP_PERIPH_ID,
};
#endif
#endif //defined CONFIG_SPI_PL022 || defined CONFIG_SPI_PL022_MODULE

static struct amba_device watchdog = {
	.dev = {
		.coherent_dma_mask = ~0,
		.init_name = "tmpa9xx-wdt",
		.platform_data = NULL,
	},
	.res = {
		.start = WDT_BASE_ADDRESS,
		.end   = WDT_BASE_ADDRESS + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = { NO_IRQ, NO_IRQ },
	.periphid = ARM_WDT_PERIPH_ID,
};

#if defined CONFIG_SND_TMPA9XX_I2S || defined CONFIG_SND_TMPA9XX_I2S_MODULE
extern struct tmpa9xx_i2s_cfg tmpa9xx_i2s_cfg;

static struct resource tmpa9xx_resource_i2s[] = {
        {
		.start = I2S_BASE,
		.end   = I2S_BASE + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
        },
};

static struct platform_device tmpa9xx_i2s_device = {
        .name          = "tmpa9xx-i2s",
        .id            = -1,
        .resource      = tmpa9xx_resource_i2s,
        .num_resources = ARRAY_SIZE(tmpa9xx_resource_i2s),
        .dev           = {
                .coherent_dma_mask 	= ~0,
		.platform_data		= &tmpa9xx_i2s_cfg,
        }
};
#endif

#if defined CONFIG_FB_ARMCLCD || defined CONFIG_FB_ARMCLCD_MODULE
static struct resource tmpa9xx_resource_clcd[] = {
        {
		.start = LCDC_BASE,
		.end   = LCDC_BASE + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
        }, {
		.start = LCDDA_BASE,
		.end   = LCDDA_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
        }, {
		.start = INTR_VECT_LCDC,
		.end   = INTR_VECT_LCDC,
		.flags = IORESOURCE_IRQ
        }, {
		.start = INTR_VECT_LCDDA,
		.end   = INTR_VECT_LCDDA,
		.flags = IORESOURCE_IRQ
        }
};

/* defined by baseboard file */
extern struct tmpa9xx_panel_ts_info tmpa9xx_panels[];

static struct platform_device tmpa9xx_clcd_device = {
        .name          = "tmpa9xx-clcd",
        .id            = -1,
        .resource      = tmpa9xx_resource_clcd,
        .num_resources = ARRAY_SIZE(tmpa9xx_resource_clcd),
        .dev           = {
                .coherent_dma_mask 	= ~0,
		.platform_data		= &tmpa9xx_panels,
        }
};
#endif

/*
 * AMBA Devices
 */
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
	&watchdog,
};
#endif



/********************
 *    Chip Devices  * 
 ********************/

/*
 * I2C
 */ 
#if defined CONFIG_I2C_TMPA9XX_CHANNEL_0
static struct resource tmpa9xx_resource_i2c_channel_0[] = {
        {
            .start = I2C0_BASE,
            .end   = I2C0_BASE+0x1F,
            .flags = IORESOURCE_MEM,
        }, {
            .start = INTR_VECT_I2C_CH0,
            .end   = INTR_VECT_I2C_CH0,
            .flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
        },
};

struct platform_device tmpa9xx_device_i2c_channel_0 = {
        .name = "tmpa9xx-i2c",
        .id   = 0,
        .dev  = {
                .platform_data     = NULL,
                .coherent_dma_mask = DMA_BIT_MASK(32),
        },
        .resource      = tmpa9xx_resource_i2c_channel_0,
        .num_resources = ARRAY_SIZE(tmpa9xx_resource_i2c_channel_0),
};
#endif

#if defined CONFIG_I2C_TMPA9XX_CHANNEL_1
static struct resource tmpa9xx_resource_i2c_channel_1[] = {
	{
            .start = I2C1_BASE,
            .end   = I2C1_BASE+0x1F,
            .flags = IORESOURCE_MEM,
        }, {
            .start = INTR_VECT_I2C_CH1,
            .end   = INTR_VECT_I2C_CH1,
            .flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
        },
};

struct platform_device tmpa9xx_device_i2c_channel_1 = {
        .name = "tmpa9xx-i2c",
        .id   = 1,
        .dev  = {
                .platform_data     = NULL,
                .coherent_dma_mask = DMA_BIT_MASK(32),
        },
        .resource      = tmpa9xx_resource_i2c_channel_1,
        .num_resources = ARRAY_SIZE(tmpa9xx_resource_i2c_channel_1),
};
#endif

/*
 * SDHC
 */ 
#if defined CONFIG_MMC_TMPA9XX_SDHC || defined CONFIG_MMC_TMPA9XX_SDHC_MODULE
static struct resource tmpa9xx_resource_sdhc[] = {
        {
         .start = INTR_VECT_SDHC,
         .end   = INTR_VECT_SDHC,
         .flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
        },
};

struct platform_device tmpa9xx_device_sdhc = {
        .name= "tmpa9xx-sdhc",
        .id  = -1,
        .dev = {
                .coherent_dma_mask = DMA_BIT_MASK(32),
        },
        .resource      = tmpa9xx_resource_sdhc,
        .num_resources = ARRAY_SIZE(tmpa9xx_resource_sdhc),
};
#endif

/*
 * Touchscreen
 */
#if defined CONFIG_TOUCHSCREEN_TMPA9XX || defined CONFIG_TOUCHSCREEN_TMPA9XX_MODULE
static struct resource tmpa9xx_resource_ts[] = {
        {
         .start = TS_BASE,
         .end   = TS_BASE + 0x40,
         .flags = IORESOURCE_MEM,
        }, {
         .start = INTR_VECT_GPIOD,
         .end   = INTR_VECT_GPIOD,
         .flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
        },
};

struct platform_device tmpa9xx_device_ts = {
        .name= "tmpa9xx-ts",
        .id  = -1,
        .dev = {
                .platform_data = NULL,
        },
        .resource      = tmpa9xx_resource_ts,
        .num_resources = ARRAY_SIZE(tmpa9xx_resource_ts),
};
#endif

int tmpa9xx_ts_fuzz = 0;
int tmpa9xx_ts_rate = 100;

EXPORT_SYMBOL(tmpa9xx_ts_fuzz);
EXPORT_SYMBOL(tmpa9xx_ts_rate);

/*
 * NAND Flash Controller
 */
#if defined CONFIG_MTD_NAND_TMPA9XX || defined CONFIG_MTD_NAND_TMPA9XX_MODULE
static struct resource tmpa9xx_nand_resources[] = {
        [0] = {
               .start =  NANDF_BASE,
               .end   =  NANDF_BASE + 0x200,
               .flags = IORESOURCE_MEM,
        },
};

static struct platform_device tmpa9xx_nand_device = {
        .name          = "tmpa9xx-nand",
        .id            = -1,
        .num_resources = ARRAY_SIZE(tmpa9xx_nand_resources),
        .resource      = tmpa9xx_nand_resources,
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
 * Melody / Alarm
 */
#if defined CONFIG_TMPA9XX_MLDALM || defined CONFIG_TMPA9XX_MLDALM_MODULE
static struct resource tmpa9xx_resource_mldalm[] = {
        {
         .start = MA_BASE,
         .end   = MA_BASE + 0xff,
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
         .start = USB_HOST_BASE_ADDRESS,
         .end   = USB_HOST_BASE_ADDRESS + SZ_128 - 1,
         .flags = IORESOURCE_MEM,
        },
        [2] = {
         .start = INTR_VECT_USB_HOST,
         .end   = INTR_VECT_USB_HOST,
         .flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
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
               .start = USB_DEVICE_BASE_ADDRESS,
               .end   = USB_DEVICE_BASE_ADDRESS + SZ_1K - 1,
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

#if defined CONFIG_TMPA9XX_ADC || defined CONFIG_TMPA9XX_ADC_MODULE
static struct resource tmpa9xx_adc_resource[] = {
        [0] = {
               .start = ADC_BASE,
               .end   = ADC_BASE + SZ_128 - 1,
               .flags = IORESOURCE_MEM
        },
        [1] = {
               .start = INTR_VECT_ADC,
               .end   = INTR_VECT_ADC,
               .flags = IORESOURCE_IRQ
        }
};

static struct platform_device tmpa9xx_adc_device = {
        .name          = "tmpa9xx-adc",
        .id            = -1,
        .num_resources = ARRAY_SIZE(tmpa9xx_adc_resource),
        .resource      = tmpa9xx_adc_resource,
        .dev           = {
                          .platform_data        = NULL,
        }
};
#endif

#if defined CONFIG_TMPA9XX_IIO_ADC || defined CONFIG_TMPA9XX_IIO_ADC_MODULE
static struct platform_device tmpa9xx_iio_adc_device = {
        .name          = "tmpa9xx-iio-adc",
        .id            = -1,
        .num_resources = 0,
        .dev           = {
                          .platform_data        = NULL,
        }
};
#endif

#if defined CONFIG_FB_ACCELERATOR_ALTIA || defined CONFIG_FB_ACCELERATOR_ALTIA_MODULE
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

static u64 tmpa9xx_lcdda_device_dmamask = 0xffffffffUL;
static struct platform_device tmpa9xx_lcdda_device = {
        .name = "tmpa9xx-lcdda",
        .id = -1,
        .num_resources = ARRAY_SIZE(tmpa9xx_lcdda_resource),
        .resource = tmpa9xx_lcdda_resource,
        .dev		  = {
                .dma_mask                = &tmpa9xx_lcdda_device_dmamask,
                .coherent_dma_mask       = 0xffffffffUL
        }
};
#endif

#if defined CONFIG_TMPA9XX_PWM_CHANNEL_0 || defined CONFIG_TMPA9XX_PWM_CHANNEL_0_MODULE
static struct resource tmpa9xx_pwm0_resource[] = {
        [0] = {
                .start = TMPA9XX_TIMER0,
                .end   = TMPA9XX_TIMER0 + SZ_4K - 1,
                .flags = IORESOURCE_MEM
        },
};

static struct platform_device tmpa9xx_pwm0_device = {
        .name          = "tmpa9xx-pwm",
        .id            = 0,
        .num_resources = ARRAY_SIZE(tmpa9xx_pwm0_resource),
        .resource      = tmpa9xx_pwm0_resource,
        .dev           = {
                          .platform_data = NULL,
        }
};
#endif

#if defined CONFIG_TMPA9XX_PWM_CHANNEL_1 || defined CONFIG_TMPA9XX_PWM_CHANNEL_1_MODULE
static struct resource tmpa9xx_pwm2_resource[] = {
        [0] = {
                .start = TMPA9XX_TIMER2,
                .end   = TMPA9XX_TIMER2 + SZ_4K - 1,
                .flags = IORESOURCE_MEM
        },
};

static struct platform_device tmpa9xx_pwm1_device = {
        .name          = "tmpa9xx-pwm",
        .id            = 1,
        .num_resources = ARRAY_SIZE(tmpa9xx_pwm2_resource),
        .resource      = tmpa9xx_pwm2_resource,
        .dev           = {
                          .platform_data = NULL,
        }
};
#endif

static struct platform_device *devices_tmpa9xx[] __initdata = {
#if defined CONFIG_I2C_TMPA9XX_CHANNEL_0
        &tmpa9xx_device_i2c_channel_0,
#endif
#if defined CONFIG_I2C_TMPA9XX_CHANNEL_1
        &tmpa9xx_device_i2c_channel_1,
#endif

#if defined CONFIG_MMC_TMPA9XX_SDHC || defined CONFIG_MMC_TMPA9XX_SDHC_MODULE
         &tmpa9xx_device_sdhc,
#endif

#if defined CONFIG_TOUCHSCREEN_TMPA9XX || defined CONFIG_TOUCHSCREEN_TMPA9XX_MODULE
        &tmpa9xx_device_ts,
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

#if defined CONFIG_TMPA9XX_ADC || defined CONFIG_TMPA9XX_ADC_MODULE
        &tmpa9xx_adc_device,
#endif
#if defined CONFIG_TMPA9XX_IIO_ADC || defined CONFIG_TMPA9XX_IIO_ADC_MODULE
        &tmpa9xx_iio_adc_device,
#endif

#if defined CONFIG_FB_ARMCLCD || defined CONFIG_FB_ARMCLCD_MODULE
       &tmpa9xx_clcd_device,
#endif

#if defined CONFIG_FB_ACCELERATOR_ALTIA || defined CONFIG_FB_ACCELERATOR_ALTIA_MODULE
       &tmpa9xx_lcdda_device,
#endif

#if defined CONFIG_SND_TMPA9XX_I2S || defined CONFIG_SND_TMPA9XX_I2S_MODULE
       &tmpa9xx_i2s_device,
#endif

#if defined CONFIG_TMPA9XX_PWM_CHANNEL_0 || defined CONFIG_TMPA9XX_PWM_CHANNEL_0_MODULE
        &tmpa9xx_pwm0_device,
#endif
#if defined CONFIG_TMPA9XX_PWM_CHANNEL_1 || defined CONFIG_TMPA9XX_PWM_CHANNEL_1_MODULE
        &tmpa9xx_pwm1_device,
#endif
};

static u64 tmpa9xx_dma_mask = DMA_BIT_MASK(32);

static int setup_port_a(void)
{
        /* Port A can be used not only as a general-purpose input pin with pull up but also as key input pin. */

	/* All useable for GPIO */
        TMPA9XX_CFG_PORT_GPIO(PORTA);
        GPIOADIR  = 0x00;
        GPIOAFR1  = 0x00;
        GPIOAFR2  = 0x00;
        GPIOADATA = 0x00;

	return 0;
}

static int setup_port_b(void)
{
        /* Port B can be used not only as general-purpose output pins but also as key output pins. */

        TMPA9XX_CFG_PORT_GPIO(PORTB);
        GPIOBODE  = 0x00;
        GPIOBDIR  = 0x00;
        GPIOBFR1  = 0x00;
        GPIOBFR2  = 0x00;
        GPIOBDATA = 0x00;

	return 0;
}

static int setup_port_c(void)
{
        /* Port C
           The upper 2 bits (bits [7:6]) of Port C can be used as general-purpose input/output pins
           and the lower 3 bits (bits [4:2]) can be used as general-purpose output pins.
           Port C can also be used as interrupt (INT9), I2C (I2C0DA, I2C0CL), low-frequency clock
           output (FSOUT), melody output (MLDALM), PWM output function (PWM0OUT,
           PWM2OUT), and USB Host power supply control function (USBOCn, USBPON). */

        GPIOCODE  = 0x00;
        GPIOCDIR  = 0x00;
        GPIOCFR1  = 0x00;
        GPIOCFR2  = 0x00;
        GPIOCDATA = 0x00;

#if defined CONFIG_USB_OHCI_HCD_TMPA9XX || defined CONFIG_USB_OHCI_HCD_TMPA9XX_MODULE
        /* prepare Port C for USB Host */
        GPIOCDATA  =  (0xcf);
        GPIOCDIR  &= ~(0xc0);
        GPIOCFR1  &= ~(0xc0);
        GPIOCFR2  |=  (0xc0);
        GPIOCODE  &= ~(0xc0);
        GPIOCIE   &= ~(0xc0);

        /* Enable USB Host Controller Clock Domain */
        CLKCR5    |= (1<<4);

        /* Set appropriate clock seting for USB in SYSCR8.
           For USB device, 24Mhz directly from quartz: [5:4]  11 / 0x3
           For USB host  , 48Mhz from F PPL / 4      : [3:0] 100 / 0x4 */
        SYSCR8    |= ((0x3<<4)|(0x4<<0));

        /* disable suspend */
        HCBCR0     = 0;
#endif
#if defined CONFIG_TMPA9XX_MLDALM || defined CONFIG_TMPA9XX_MLDALM_MODULE
#if defined CONFIG_TMPA9XX_PWM_CHANNEL_0 || defined CONFIG_TMPA9XX_PWM_CHANNEL_0_MODULE
#error "port c configuration mismatch. melody/alarm vs. pwm channel 0"
#endif
        GPIOCFR1 |=  (0x1<<3);
        GPIOCFR2 &= ~(0x1<<3);
        GPIOCIE  &= ~(0x1<<3);
        GPIOCODE &= ~(0x1<<3);
#endif

#if defined CONFIG_TMPA9XX_PWM_CHANNEL_0 || defined CONFIG_TMPA9XX_PWM_CHANNEL_0_MODULE
#if defined CONFIG_TMPA9XX_MLDALM || defined CONFIG_TMPA9XX_MLDALM_MODULE
#error "port c configuration mismatch. melody/alarm vs. pwm channel 0"
#endif
        GPIOCFR1 &= ~(0x1<<3);
        GPIOCFR2 |=  (0x1<<3);
        GPIOCIE  &= ~(0x1<<3);
        GPIOCODE &= ~(0x1<<3);
#endif

#if (defined CONFIG_TMPA9XX_PWM_CHANNEL_1 || defined CONFIG_TMPA9XX_PWM_CHANNEL_1_MODULE)
        GPIOCFR1 &= ~(0x1<<4);
        GPIOCFR2 |=  (0x1<<4);
        GPIOCIE  &= ~(0x1<<4);
        GPIOCODE &= ~(0x1<<4);
#endif

#if defined CONFIG_I2C_TMPA9XX_CHANNEL_0 \
 && (!defined CONFIG_USB_OHCI_HCD_TMPA9XX && !defined CONFIG_USB_OHCI_HCD_TMPA9XX_MODULE) \
        /* set PORT-C 6,7 to I2C and enable open drain */
        GPIOCFR1 |=  (0xc0);
        GPIOCFR2 &= ~(0xc0);
        GPIOCODE |=  (0xc0);
#endif

#if !defined CONFIG_USB_OHCI_HCD_TMPA9XX && !defined CONFIG_USB_OHCI_HCD_TMPA9XX_MODULE \
 && !defined CONFIG_TMPA9XX_MLDALM       && !defined CONFIG_TMPA9XX_MLDALM_MODULE \
 && !defined CONFIG_I2C_TMPA9XX          && !defined CONFIG_I2C_TMPA9XX_MODULE
        TMPA9XX_CFG_PORT_GPIO(PORTC);
#endif

	return 0;
}

static int setup_port_d(void)
{
        /* Port D can be used as general-purpose input.
           Port D can also be used as interrupt (INTB, INTA), ADC (AN7-AN0), and touch screen
           control (PX, PY, MX, MY) pins. */
        GPIODDIR  = 0x00;
        GPIODDATA = 0x00;
        GPIODIE   = 0x00;

        TMPA9XX_CFG_PORT_GPIO(PORTD);

#if defined CONFIG_TOUCHSCREEN_TMPA9XX || defined CONFIG_TOUCHSCREEN_TMPA9XX_MODULE
        GPIODFR2 |= 0xf0;
#endif

#if defined CONFIG_TMPA9XX_ADC || defined CONFIG_TMPA9XX_ADC_MODULE
#if defined CONFIG_TOUCHSCREEN_TMPA9XX || defined CONFIG_TOUCHSCREEN_TMPA9XX_MODULE
        GPIODFR1 |= 0x0f;
#else
        GPIODFR2  = 0x00;
        GPIODFR1 |= 0xff;
#endif
#endif

	return 0;
}

static int setup_port_e(void)
{
#if defined CONFIG_TMPA9XX_CMSI || defined CONFIG_TMPA9XX_CMSI_MODULE
        GPIOEFR1 |=  (0xff);
#endif
	return 0;
}

static int setup_port_f(void)
{
        /* Port F
           The upper 2 bits (bits [7:6]) of Port F can be used as general-purpose input/output pins.
           Port F can also be used as interrupt (INTC), UART (U2RXD, U2TXD) and I2C (I2C1DA,
           I2C1CL) pins. */

#if defined CONFIG_SERIAL_AMBA_PL011_CHANNEL_2 && !defined CONFIG_I2C_TMPA9XX_CHANNEL_1
        GPIOFFR1 &= ~(0xc0);  /* UART 2 */
        GPIOFFR2 |=  (0xc0);
        GPIOFIE  &= ~(0xc0);
        GPIOFODE &= ~(0xc0);
#endif    

#if defined CONFIG_I2C_TMPA9XX_CHANNEL_1
        /* set PORT-C 6,7 to I2C and enable open drain */
        GPIOFDIR &= ~(0xc0);
        GPIOFFR1 |=  (0xc0);
        GPIOFFR2 &= ~(0xc0);
        GPIOFIE  &= ~(0xc0);
        GPIOFODE |=  (0xc0);
#endif

#if defined CONFIG_TMPA9XX_CMSI || defined CONFIG_TMPA9XX_CMSI_MODULE
        GPIOFFR1 |=  (0x0f);
#endif

	return 0;
}

static int setup_port_g(void)
{
        /* Port G can be used as general-purpose input/output pins.
           Port G can also be used as SD host controller function pins (SDC0CLK, SDC0CD,
           SDC0WP, SDC0CMD, SDC0DAT3, SDC0DAT2, SDC0DAT1 and SDC0DAT0). */

        GPIOGDIR  = (0x00);
        GPIOGDATA = (0x00);
        GPIOGFR1  = (0x00);

#if defined CONFIG_MMC_TMPA9XX_SDHC || defined CONFIG_MMC_TMPA9XX_SDHC_MODULE
        GPIOGFR1 = (0xff);
#else
        TMPA9XX_CFG_PORT_GPIO(PORTG)
#endif

	return 0;
}

static int setup_port_j_and_k(void)
{
        /* Port J can be used as general-purpose input/output pins.
           Port J can also be used as LCD cotroller function pins (LD15-LD8) and CMOS image
           sensor control (CMSVSY, CMSHBK, CMSHSY and CMSPCK) pins. */

        /* Port K can be used as general-purpose input/output pins.
           Port K can also be used as LCD controller function pins (LD23 to LD16) and CMOS image
           sensor control (CMSD7 toCMSD0) pins. */

        GPIOJDIR  = 0x00;
        GPIOJDATA = 0x00;
        GPIOJFR1  = 0x00;
        GPIOJFR2  = 0x00;

        GPIOKDIR  = 0x00;
        GPIOKDATA = 0x00;
        GPIOKFR1  = 0x00;
        GPIOKFR2  = 0x00;

#if defined CONFIG_FB_ARMCLCD || defined CONFIG_FB_ARMCLCD_MODULE
        GPIOJFR2 = 0x00;
        GPIOJFR1 = 0xff;
        GPIOKFR2 = 0x00;
        GPIOKFR1 = 0xff;
#else
        TMPA9XX_CFG_PORT_GPIO(PORTJ);
        TMPA9XX_CFG_PORT_GPIO(PORTK);
#endif

	return 0;
}

static int setup_port_n(void)
{
        /* Port N can be used as general-purpose input/output pins.
           Port N can also be used as UART/IrDA function (U0RTSn, U0DTRn, U0RIn, U0DSRn,
           U0DCDn, U0CTSn, U0RXD, U0TXD, SIR0IN, SIR0OUT) and interrupt function (INTD,
           INTE, INTF, INTG) pins. */

        GPIONDIR  = 0x00;
        GPIONDATA = 0x00;
        GPIONFR1  = 0x00;
        GPIONFR2  = 0x00;
        GPIONIE   = 0x00;

#ifdef CONFIG_SERIAL_AMBA_PL011_CHANNEL_0
	GPIONFR1 = 0x000000fd;
	GPIONFR2 = 0x00000002;
#else
        TMPA9XX_CFG_PORT_GPIO(PORTN)
#endif           

	return 0;
}

static int setup_port_r(void)
{
        /* Port R
           Bit 2 of Port R can be used as a general-purpose input/output pin and bits [1:0] can be
           used as general-purpose output pins. (Bits [7:3] are not used.)
           Port R can also be used as reset output (RESETOUTn), high-frequency clock output
           (FCOUT), interrupt function (INTH) and Oscillation Frequency Detection (OFDOUTn). */

        GPIORDIR  = 0x00;
        GPIORDATA = 0x00;
        GPIORFR1  = 0x00;
        GPIORFR2  = 0x00;
        GPIORIE   = 0x00;

#if defined CONFIG_NET_ETHERNET || defined CONFIG_NET_ETHERNET_MODULE
        GPIORDIR &= ~(1 << 2); /* Eth IRQ */
#else
        TMPA9XX_CFG_PORT_GPIO(PORTR)
#endif

	return 0;
}

static int setup_port_t(void)
{
        /* Port T can be used as general-purpose input/output pins.
           Port T can also be used as USB external clock input (X1USB), UART function (U1CTSn,
           U1RXD, U1TXD), and SPI function (SP0DI, SP0DO, SP0CLK, SP0FSS) and pins. */

        GPIOTDIR  = 0x00;
        GPIOTDATA = 0x00;
        GPIOTFR1  = 0x00;
        GPIOTFR2  = 0x00;

#ifdef CONFIG_SERIAL_AMBA_PL011_CHANNEL_1
        GPIOTFR1 |= (0x07 << 4);
#endif
#ifdef CONFIG_SPI_PL022_CHANNEL_0
        GPIOTFR1 |= (0x0f);
#endif

	return 0;
}

void __init tmpa9xx_init(void)
{
#ifdef CONFIG_ARM_AMBA
        int i;
#endif        
        /* DMA setup */
        platform_bus.coherent_dma_mask = DMA_BIT_MASK(32);
        platform_bus.dma_mask          = &tmpa9xx_dma_mask;

        /* Pin configuration */
	setup_port_a();
	setup_port_b();
	setup_port_c();
	setup_port_d();
	setup_port_e();
	setup_port_f();
	setup_port_g();
	setup_port_j_and_k();
	/* port l and m are highly board specific */
	setup_port_n();
	setup_port_r();
	setup_port_t();

        /* Register the active AMBA devices on this chip */
#ifdef CONFIG_ARM_AMBA
        for (i= 0; i < ARRAY_SIZE(amba_devs); i++)
        {
                amba_device_register(amba_devs[i], &iomem_resource);
        }
#endif

        /* Add chip devices */
        platform_add_devices(devices_tmpa9xx, ARRAY_SIZE(devices_tmpa9xx));
}
