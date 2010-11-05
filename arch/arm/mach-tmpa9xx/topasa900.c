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

#include "tmpa9xx.h"

/*
 * TopasA900 device initialisation
 */
static void __init topasa900_init(void)
{
	baseboard_init();
	tmpa9xx_init();
}

MACHINE_START(TOPASA900, "Toshiba TopasA900")
        /* Maintainer:  Florian Boor <florian.boor@kernelconcepts.de> */
        .phys_io        = TMPA9XX_IO_PHYS_BASE,
        .boot_params    = 0,
        .io_pg_offst    = (io_p2v(TMPA9XX_IO_PHYS_BASE) >> 18) & 0xfffc,
        .map_io         = tmpa9xx_map_io,
        .init_irq       = tmpa9xx_init_irq,
        .timer          = &tmpa9xx_timer,
        .init_machine   = topasa900_init,
MACHINE_END
