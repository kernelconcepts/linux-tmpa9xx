/*
 *  arch/arm/mach-tmpa9xx/tonga.c 
 *
 * Copyright (C) 2010 Thomas Haase <Thomas.Haase@web.de>
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
 * TFTTimer / Tonga machine definition, multi purpose Toshiba TMPA900 based SoM 
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <linux/smsc911x.h>
#include <linux/interrupt.h>

#include "tmpa9xx.h"

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/dma-mapping.h>
#include <linux/amba/pl022.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/ts.h>
#include <mach/regs.h>
#include <asm/mach/map.h>

/* 
 * Ethernet 
 */ 
#if defined CONFIG_NET_ETHERNET || defined CONFIG_NET_ETHERNET_MODULE

#define ETHERNET_MAC_ASCII_LENGTH 17
#define MAC_OFFSET                 8

static struct resource smsc911x_resources[] = {
        [0] = {
                .start = 0x60000000,
                .end   = 0x60000003,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = 0x60001000,
                .end   = 0x60001003,
                .flags = IORESOURCE_MEM,
        },
        [2] = {
                .start = TONGA_INT_SMSC911X,
                .end   = TONGA_INT_SMSC911X,
                .flags = IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
        },
};

static struct smsc911x_platform_config tonga_smsc911x_pdata = {
        .irq_polarity  = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
        .irq_type      = SMSC911X_IRQ_TYPE_PUSH_PULL,
        .flags         = SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS,
        .phy_interface = PHY_INTERFACE_MODE_MII,
        .mac           = "deadaa",
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

static struct platform_device *devices[] __initdata = {
        &tonga_smsc911x_device,
};

static void parse_enetaddr(char *addr, unsigned char *enetaddr)
{
        char *end;
        int i;

        for (i = 0; i < 6; ++i) {
                enetaddr[i] = addr ? simple_strtoul(addr, &end, 16) : 0;
                if (addr)
                        addr = (*end) ? end + 1 : end;
        }
}
#endif

/* 
 * Tonga2 device initialisation
 */
static void __init tonga2_tfttimer_init(void)
{
#if defined CONFIG_NET_ETHERNET || defined CONFIG_NET_ETHERNET_MODULE
        char *p;
        char eth_mac_ascii[ETHERNET_MAC_ASCII_LENGTH+2];

        /* get mac address from the command line */
        memset(eth_mac_ascii,0,sizeof(eth_mac_ascii));

        p = strstr(boot_command_line, "ethaddr=");

        if (p != NULL && (p == boot_command_line || p[-1] == ' ')) {
                printk(KERN_DEBUG "U-BOOT Ethernet Address: %s\n", p+8);
                memcpy(&eth_mac_ascii,p + MAC_OFFSET, ETHERNET_MAC_ASCII_LENGTH);
                parse_enetaddr (eth_mac_ascii, tonga_smsc911x_pdata.mac);
        }
#endif
        /* Add devices */
        platform_add_devices(devices, ARRAY_SIZE(devices));

	baseboard_init();
	tmpa9xx_init();
}

MACHINE_START(TONGA2_TFTTIMER, "Tonga 2 TFTTimer")
        /* Maintainer:  Thomas Haase <Thomas.Haase@web.de> */
        .phys_io        = TMPA9XX_IO_PHYS_BASE,
        .boot_params    = 0,
        .io_pg_offst    = (io_p2v(TMPA9XX_IO_PHYS_BASE) >> 18) & 0xfffc,
        .map_io         = tmpa9xx_map_io,
        .init_irq       = tmpa9xx_init_irq,
        .timer          = &tmpa9xx_timer,
        .init_machine   = tonga2_tfttimer_init,
MACHINE_END
