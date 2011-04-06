/*
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
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
 */

#ifndef __TMPA9XX_SYSTEM_H__
#define __TMPA9XX_SYSTEM_H__

#include <asm/io.h>

static void
arch_idle(void)
{
	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks
	 */
	cpu_do_idle();
}

static inline void
arch_reset(char mode,const char * cmd)
{
  uint8_t *wdt_base;

  printk("Issue reset\n");

  wdt_base = (char *) (0xf0010000);

  if (wdt_base==NULL )
  {
          wdt_base = (uint8_t *) 0xf0010000;
  }

  outl( 0x1, (uint32_t *) (wdt_base + 0));
  outl( 0x3, (uint32_t *) (wdt_base + 8));

  /* Bye ! */
}

#endif
