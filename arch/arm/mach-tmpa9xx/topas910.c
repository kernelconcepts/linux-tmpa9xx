/*
 * arch/arm/mach-tmpa9xx/topas910.c -- Topas 910 machine 
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
 * Toshiba Topas 910 machine, reference design for the TMPA910CRAXBG SoC 
 */

#include <linux/init.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/regs.h>

#include "tmpa9xx.h"

static int setup_port_e(void)
{
#if defined CONFIG_TMPA9XX_CMSI || defined CONFIG_TMPA9XX_CMSI_MODULE
        GPIOEFR1 |=  (0xff);
#endif
	return 0;
}

static int setup_port_f(void)
{
#if defined CONFIG_I2C_TMPA9XX_CHANNEL_1
        /* set PORT-C 6,7 to I2C and enable open drain */
        GPIOFDIR &= ~(0xc0);
        GPIOFFR1 |=  (0xc0);
        GPIOFIE  &= ~(0xc0);
        GPIOFODE |=  (0xc0);
#endif
#if defined CONFIG_TMPA9XX_CMSI || defined CONFIG_TMPA9XX_CMSI_MODULE
        GPIOFFR1 |=  (0x0f);
#endif
	return 0;
}

/*
 * Topas910 device initialisation
 */
static void __init topas910_init(void)
{
	baseboard_init();
	tmpa9xx_init();

	setup_port_e();
	setup_port_f();
}

MACHINE_START(TOPAS910, "Toshiba Topas910")
        /* Maintainer:  Florian Boor <florian.boor@kernelconcepts.de> */
        .boot_params    = 0,
        .map_io         = tmpa9xx_map_io,
        .init_irq       = tmpa9xx_init_irq,
        .timer          = &tmpa9xx_timer,
        .init_machine   = topas910_init,
MACHINE_END
