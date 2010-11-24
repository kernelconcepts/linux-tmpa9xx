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
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/pwm_backlight.h>
#include <linux/dma-mapping.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>
#include <linux/amba/clcd.h>

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


#define ARM_UART_PERIPH_ID  0x00041011
#define ARM_SSP_PERIPH_ID   0x00041022
#define ARM_CLCD_PERIPH_ID  0x00041110
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

#if defined CONFIG_FB_ARMCLCD || defined CONFIG_FB_ARMCLCD_MODULE

/*
 * CLCD support
 */
#define CLOCK_TO_DIV(e,c)       (((c) + (e) - 1)/(e))

#define HCLK 96000000
#define PIX_CLOCK_TARGET        (9600000) /* -/6.3/7 MHz */
#define PIX_CLOCK_DIVIDER       CLOCK_TO_DIV (PIX_CLOCK_TARGET, HCLK)
#define PIX_CLOCK               (HCLK/PIX_CLOCK_DIVIDER)

struct clcd_panel tmpa9xx_panel = {
	.mode		= {
		.name		= "TMPA9xx Panel",
		.refresh	= 30,
		.xres		= 320,
		.yres		= 240,
		.pixclock	= PIX_CLOCK,
		.left_margin	= 8,
		.right_margin	= 8,
		.upper_margin	= 2,
		.lower_margin	= 2,
		.hsync_len	= 8,
		.vsync_len	= 2,
		.sync		= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_IPC | PIX_CLOCK_DIVIDER,
	.cntl		= CNTL_LCDTFT | CNTL_WATERMARK,
	.bpp		= 32,
	.grayscale	= 0,
};

static void tmpa9xx_clcd_enable(struct clcd_fb *fb)
{
}

static unsigned long framesize = SZ_1M;

static int tmpa9xx_clcd_setup(struct clcd_fb *fb)
{
	dma_addr_t dma;

	fb->panel = &tmpa9xx_panel;

	fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev, framesize,
						    &dma, GFP_KERNEL);
	if (!fb->fb.screen_base) {
		printk(KERN_ERR "CLCD: unable to map framebuffer\n");
		return -ENOMEM;
	}

	fb->fb.fix.smem_start	= dma;
	fb->fb.fix.smem_len	= framesize;

	return 0;
}

static int tmpa9xx_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
	return dma_mmap_writecombine(&fb->dev->dev, vma,
				     fb->fb.screen_base,
				     fb->fb.fix.smem_start,
				     fb->fb.fix.smem_len);
}

static void tmpa9xx_clcd_remove(struct clcd_fb *fb)
{
	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
			      fb->fb.screen_base, fb->fb.fix.smem_start);
}

static struct clcd_board clcd_platform_data = {
	.name		= "tmpa9xx FB",
	.check		= clcdfb_check,
	.decode		= clcdfb_decode,
	.enable		= tmpa9xx_clcd_enable,
	.setup		= tmpa9xx_clcd_setup,
	.mmap		= tmpa9xx_clcd_mmap,
	.remove		= tmpa9xx_clcd_remove,
};

static struct amba_device clcd_device = {
        .dev = {
                .coherent_dma_mask = ~0,
                .init_name = "tmpa9xx-clcd",
                .platform_data = &clcd_platform_data,
                },
        .res = {
                .start        = LCDC_BASE,
                .end          = LCDC_BASE+ (4*1024) - 1,
                .flags        = IORESOURCE_MEM,
                },
        .dma_mask = ~0,
        .irq      = { INTR_VECT_LCDC, },
        .periphid = ARM_CLCD_PERIPH_ID,
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
#if defined CONFIG_FB_ARMCLCD || defined CONFIG_FB_ARMCLCD_MODULE
        &clcd_device,
#endif        
};
#endif



/********************
 *    Chip Devices  * 
 ********************/

/*
 * I2C
 */ 
#if defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE
static struct resource tmpa9xx_resource_i2c[] = {
        {
            .start = I2C0_BASE,
            .end   = I2C0_BASE+0x1F,
            .flags = IORESOURCE_MEM,
        }, {
            .start = I2C1_BASE,
            .end   = I2C1_BASE+0x1F,
            .flags = IORESOURCE_MEM,
        }, {
            .start = INTR_VECT_I2C_CH0,
            .end   = INTR_VECT_I2C_CH0,
            .flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
        }, {
            .start = INTR_VECT_I2C_CH1,
            .end   = INTR_VECT_I2C_CH1,
            .flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
        }
};

struct platform_device tmpa9xx_device_i2c = {
        .name= "tmpa9xx-i2c",
        .id  = 0,
        .dev = {
                .platform_data     = NULL,
                .coherent_dma_mask = DMA_BIT_MASK(32),
        },
        .resource      = tmpa9xx_resource_i2c,
        .num_resources = ARRAY_SIZE(tmpa9xx_resource_i2c),
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
static struct tmpa9xx_ts_platforminfo tmpa9xx_info_ts = {
        .fuzz       = 0,
        .rate       = 100,
        .skip_count = 4,
};

static struct resource tmpa9xx_resource_ts[] = {
        {
         .start = TS_BASE,
         .end   = TS_BASE + 0x40,
         .flags = IORESOURCE_MEM,
        }, {
         .start = ADC_BASE,
         .end   = ADC_BASE + 0x100,
         .flags = IORESOURCE_MEM,
        }, {
         .start = INTR_VECT_GPIOD,
         .end   = INTR_VECT_GPIOD,
         .flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
        }, {
         .start = INTR_VECT_ADC,
         .end   = INTR_VECT_ADC,
         .flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
        }
};

struct platform_device tmpa9xx_device_ts = {
        .name= "tmpa9xx-ts",
        .id  = -1,
        .dev = {
                .platform_data = &tmpa9xx_info_ts,
        },
        .resource      = tmpa9xx_resource_ts,
        .num_resources = ARRAY_SIZE(tmpa9xx_resource_ts),
};
#endif

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
         .start = USB_HOST_BASE_ADDRESS,
         .end   = USB_HOST_BASE_ADDRESS + 0x100,
         .flags = IORESOURCE_MEM,
        },
        [1] = {
         .start = 0xF8008000,
         .end   = 0xF8009fff,
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
               .end   = USB_DEVICE_BASE_ADDRESS + 0x3ff,
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
               .start = WDT_BASE_ADDRESS,
               .end   = WDT_BASE_ADDRESS + 0xc04,
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

#if defined CONFIG_TMPA9XX_ADC || defined CONFIG_TMPA9XX_ADC_MODULE
static struct resource tmpa9xx_adc_resource[] = {
        [0] = {
               .start = ADC_BASE,
               .end   = ADC_BASE + 0x0FFF,
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

static u64 tmpa9xx_device_lcdda_dmamask = 0xffffffffUL;
static struct platform_device tmpa9xx_device_lcdda = {
        .name = "tmpa9xx-lcdda",
        .id = -1,
        .num_resources = ARRAY_SIZE(tmpa9xx_lcdda_resource),
        .resource = tmpa9xx_lcdda_resource,
        .dev              = {
                .dma_mask                = &tmpa9xx_device_lcdda_dmamask,
                .coherent_dma_mask        = 0xffffffffUL
        }
};

static struct resource tmpa9xx_pwm_resource[] = {
        [0] = {
                .start = TMPA9XX_TIMER2,
                .end   = TMPA9XX_TIMER2 + 0x0fff,
                .flags = IORESOURCE_MEM
        },
};

static struct platform_device tmpa9xx_pwm_device = {
        .name          = "tmpa9xx-pwm",
        .id            = 0,
        .num_resources = ARRAY_SIZE(tmpa9xx_pwm_resource),
        .resource      = tmpa9xx_pwm_resource,
        .dev           = {
                          .platform_data = NULL,
        }
};


static struct platform_device *devices_tmpa9xx[] __initdata = {
#if defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE
        &tmpa9xx_device_i2c,
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
#if defined CONFIG_TMPA9XX_WATCHDOG || defined CONFIG_TMPA9XX_WATCHDOG_MODULE
        &tmpa9xx_wdt_device,
#endif

#if defined CONFIG_TMPA9XX_ADC || defined CONFIG_TMPA9XX_ADC_MODULE
        &tmpa9xx_adc_device,
#endif

        &tmpa9xx_device_lcdda,
        &tmpa9xx_pwm_device,
};

/*
 * LCD Parameter Parsing for compatibility
 */ 
#if defined CONFIG_FB_ARMCLCD || defined CONFIG_FB_ARMCLCD_MODULE

#if defined CONFIG_MACH_TONGA
static unsigned int videoparams[4]={0x19211e4c,0x10040cef,0x013f380d,0x00010828};
#else
static unsigned int videoparams[4]={0x0707074c,0x020204ef,0x013f200e,0x0001082A};
#endif

static void setup_display(void)
{
    	char *options = NULL;
        unsigned int sync   = 0;
        unsigned int timer2 = 0;
        
	fb_get_options("tmpa9xxfb", &options);
	if (options) {
	    	unsigned int r0, r1, r2 , r3;
    		r3=0;

		if (sscanf(options, "%08x:%08x:%08x:%08x", &r0, &r1, &r2, &r3) != 4) {
			if (sscanf(options, "%08x:%08x:%08x", &r0, &r1, &r2) != 3) {
				return;
                	}
		}
		videoparams[0] = r0;
		videoparams[1] = r1;
		videoparams[2] = r2;
	        if (r3!=0)
			videoparams[3] = r3;

		printk(KERN_INFO "tmpa9xxfb: Options from cmdline: \n" \
	        	         "LCDTiming0: 0x%08x\nLCDTiming1: 0x%08x\nLCDTiming2: 0x%08x\nLCDControl: 0x%08x\n"
                                 , videoparams[0],videoparams[1],videoparams[2],videoparams[3]);
        }
	
        tmpa9xx_panel.mode.xres         = (((videoparams[0]>>2)  &  0x3f)  + 1) *16;
        tmpa9xx_panel.mode.left_margin  = (  videoparams[0]>>24)           + 1;
        tmpa9xx_panel.mode.right_margin = (( videoparams[0]>>16) &  0xff)  + 1;
        tmpa9xx_panel.mode.hsync_len    = (( videoparams[0]>>8)  &  0xff)  + 1;

        tmpa9xx_panel.mode.yres         = ( videoparams[1]&        0x3ff)  + 1;
        tmpa9xx_panel.mode.upper_margin = ( videoparams[1]>>24)               ;
        tmpa9xx_panel.mode.lower_margin = ((videoparams[1]>>16)  &  0xff)     ;
        tmpa9xx_panel.mode.vsync_len    = ((videoparams[1]>>10)  &  0x1f)  + 1;

        tmpa9xx_panel.bpp               =  1 << ((videoparams[3]>>1)&0x07);

        if (!(videoparams[2] & (1<<13)))
		sync |= FB_SYNC_HOR_HIGH_ACT;
        if (!(videoparams[2] & (1<<12)))
		sync |= FB_SYNC_VERT_HIGH_ACT;
        if (videoparams[2] && (1<<11))
		timer2 |= TIM2_IPC;

	timer2 |= (videoparams[2])&0x1f;
        
        tmpa9xx_panel.mode.sync = sync;
	tmpa9xx_panel.tim2      = timer2;
}
#endif

static u64 tmpa9xx_dma_mask = DMA_BIT_MASK(32);

void __init tmpa9xx_init(void)
{
#ifdef CONFIG_ARM_AMBA
        int i;
#endif        
#if defined CONFIG_FB_ARMCLCD || defined CONFIG_FB_ARMCLCD_MODULE
	setup_display();
#endif
        /* DMA setup */
        platform_bus.coherent_dma_mask = DMA_BIT_MASK(32);
        platform_bus.dma_mask          = &tmpa9xx_dma_mask;
        
        /* Pin configuration */
        
        /* Port A can be used not only as a general-purpose input pin with pull up but also as key input pin. */
        TMPA9XX_CFG_PORT_GPIO(PORTA); /* All useable for GPIO */
        
        /* Port B can be used not only as general-purpose output pins but also as key output pins. */
        TMPA9XX_CFG_PORT_GPIO(PORTB); 
        GPIOBODE = (0x00); /* Disable Open Drain */
        
        /* Port C
           The upper 2 bits (bits [7:6]) of Port C can be used as general-purpose input/output pins
           and the lower 3 bits (bits [4:2]) can be used as general-purpose output pins.
           Port C can also be used as interrupt (INT9), I2C (I2C0DA, I2C0CL), low-frequency clock
           output (FSOUT), melody output (MLDALM), PWM output function (PWM0OUT,
           PWM2OUT), and USB Host power supply control function (USBOCn, USBPON). */

        GPIOCODE  = (0x00); 
        GPIOCDIR  = (0x00);
        GPIOCFR1  = (0x00);
        GPIOCFR2  = (0x00);
        GPIOCDATA = (0x00);
        
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
 && !defined CONFIG_TMPA9XX_MLDALM       && !defined CONFIG_TMPA9XX_MLDALM_MODULE \
 && !defined CONFIG_I2C_TMPA9XX          && !defined CONFIG_I2C_TMPA9XX_MODULE
       TMPA9XX_CFG_PORT_GPIO(PORTC);
        GPIOCODE  = (0x00); 
        GPIOCDIR  = (0xff);
        GPIOCFR1  = (0x00);
        GPIOCFR2  = (0x00);
        GPIOCDATA = (0x00);
#endif

#if (defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE) \
 && (!defined CONFIG_USB_OHCI_HCD_TMPA9XX && !defined CONFIG_USB_OHCI_HCD_TMPA9XX_MODULE) \
 && defined CONFIG_I2C_TMPA9XX_CHANNEL_0
        /* set PORT-C 6,7 to I2C and enable open drain */
        GPIOCFR1 |=  (0xc0);
        GPIOCFR2 &= ~(0xc0);
        GPIOCODE |=  (0xc0);
#endif

        /* Port D can be used as general-purpose input.
           Port D can also be used as interrupt (INTB, INTA), ADC (AN7-AN0), and touch screen
           control (PX, PY, MX, MY) pins. */
        GPIODFR1  = (0x00);
        GPIODFR2  = (0x00);
        GPIODIE   = (0x00);
        
#if defined CONFIG_TOUCHSCREEN_TMPA9XX || defined CONFIG_TOUCHSCREEN_TMPA9XX_MODULE
        GPIODFR1 |= (0x00);
        GPIODFR2 |= (0xf0);
        GPIODIE   = (0x00);
#endif

	/* ADC Multiplex Handling will be done in driver directly */

        /* Port F
           The upper 2 bits (bits [7:6]) of Port F can be used as general-purpose input/output pins.
           Port F can also be used as interrupt (INTC), UART (U2RXD, U2TXD) and I2C (I2C1DA,
           I2C1CL) pins. */
#if defined CONFIG_UART2 && !defined CONFIG_I2C_TMPA9XX_CHANNEL_1
        GPIOFFR1 &= ~(0xc0);  /* UART 2 */
        GPIOFFR2 |=  (0xc0);
        GPIOFIE  &= ~(0xc0);
        GPIOFODE &= ~(0xc0);
#endif    
#if (defined CONFIG_I2C_TMPA9XX || defined CONFIG_I2C_TMPA9XX_MODULE) && defined CONFIG_I2C_TMPA9XX_CHANNEL_1
        /* set PORT-C 6,7 to I2C and enable open drain */
        GPIOFDIR &= ~(0xc0);
        GPIOFFR1 |=  (0xc0);
        GPIOFFR2 &= ~(0xc0);
        GPIOFIE  &= ~(0xc0);
        GPIOFODE |=  (0xc0);
#endif

        /* Port G can be used as general-purpose input/output pins.
           Port G can also be used as SD host controller function pins (SDC0CLK, SDC0CD,
           SDC0WP, SDC0CMD, SDC0DAT3, SDC0DAT2, SDC0DAT1 and SDC0DAT0). */
#if defined CONFIG_MMC_TMPA9XX_SDHC || !defined CONFIG_MMC_TMPA9XX_SDHC_MODULE
        GPIOGFR1 = (0xff);
#else
        TMPA9XX_CFG_PORT_GPIO(PORTG); /* SDIO0 or GPIO */
#endif

        /* Port J can be used as general-purpose input/output pins.
           Port J can also be used as LCD cotroller function pins (LD15-LD8) and CMOS image
           sensor control (CMSVSY, CMSHBK, CMSHSY and CMSPCK) pins. */
        /* Port K can be used as general-purpose input/output pins.
           Port K can also be used as LCD controller function pins (LD23 to LD16) and CMOS image
           sensor control (CMSD7 toCMSD0) pins. */
#if defined CONFIG_FB_ARMCLCD || defined CONFIG_FB_ARMCLCD_MODULE
        GPIOJFR2 = (0x00);
        GPIOJFR1 = (0xff);
        GPIOKFR2 = (0x00);
        GPIOKFR1 = (0xff);
#endif

        GPIOLFR1 = (0x00);
        GPIOLFR2 = (0x00);
        /* Port L can be used as general-purpose input/output pins. (Bits [7:5] are not used.)
           In addition, Port L can also be used as I2S function (I2SSCLK, I2S0MCLK, I2S0DATI,
           I2S0CLK and I2S0WS) and SPI function (SP1DI, SP1DO, SP1CLK and SP1FSS) pins. */
#if defined CONFIG_SND_TMPA9XX_WM8983 || defined CONFIG_SND_TMPA9XX_WM8983_MODULE || defined CONFIG_SND_SOC_TMPA9XX_I2S
        GPIOLFR1 |= (0x1f); /* bits 4:0 for I2S */
#endif        
#ifdef CONFIG_SPI_PL022_CHANNEL_1
        GPIOLFR2 |= (0x0f);
#endif
        /* Port M can be used as general-purpose input/output pins. (Bits [7:4] are not used.)
           Port M can also be used as I2S function pins (I2S1MCLK, I2S1DATO, I2S1CLK and
           I2S1WS).*/
        GPIOMDIR |=  (0x03); /* M0, MI GPIO OUT */
#if defined CONFIG_SND_TMPA9XX_WM8983 || defined CONFIG_SND_TMPA9XX_WM8983_MODULE || defined CONFIG_SND_SOC_TMPA9XX_I2S
        GPIOMFR1 &= ~(0x03);
        GPIOMFR1 |=  (0x04); /* M2 I2S1DAT0 */
#endif
           
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
        GPIOTFR1 |= (0x0f);
#endif
   
        /* NAND Controller */
        NDFMCR0 = 0x00000010; // NDCE0n pin = 0, ECC-disable
        NDFMCR1 = 0x00000000; // ECC = Hamming
        NDFMCR2 = 0x00003343; // NDWEn L = 3clks,H =3clks,
                              // NDREn L = 4clks,H = 3clks
        NDFINTC = 0x00000000; // ALL Interrupt Disable

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
