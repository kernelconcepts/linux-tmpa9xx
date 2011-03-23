/*
 *  arch/arm/mach-tmpa9xx/topasa900.c 
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
 * Toshiba Topas A900 machine, reference design for the TMPA900 SoC 
 */

#include <linux/init.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/regs.h>

#include "tmpa9xx.h"

static int setup_port_f(void)
{
        /* Port F
           The upper 2 bits (bits [7:6]) of Port F can be used as general-purpose input/output pins.
           Port F can also be used as interrupt (INTC), UART (U2RXD, U2TXD) and I2C (I2C1DA,
           I2C1CL) pins. */

        GPIOFDIR  = 0x00;
        GPIOFDATA = 0x00;
        GPIOFFR1  = 0x00;
        GPIOFFR2  = 0x00;
        GPIOFIE   = 0x00;

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

#if (!defined CONFIG_SERIAL_AMBA_PL011_CHANNEL_2 && !defined CONFIG_I2C_TMPA9XX_CHANNEL_1 ) \
 && (!defined CONFIG_I2C_TMPA9XX && !defined CONFIG_I2C_TMPA9XX_MODULE) && !defined CONFIG_I2C_TMPA9XX_CHANNEL_1
        TMPA9XX_CFG_PORT_GPIO(PORTF);
#endif

	return 0;
}

/*
 * TopasA900 device initialisation
 */
static void __init topasa900_init(void)
{
	baseboard_init();
	tmpa9xx_init();

	setup_port_f();
}

MACHINE_START(TOPASA900, "Toshiba TopasA900")
        /* Maintainer:  Florian Boor <florian.boor@kernelconcepts.de> */
        .boot_params    = 0,
        .map_io         = tmpa9xx_map_io,
        .init_irq       = tmpa9xx_init_irq,
        .timer          = &tmpa9xx_timer,
        .init_machine   = topasa900_init,
MACHINE_END
