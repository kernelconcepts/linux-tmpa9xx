/*
 * arch/arm/mach-topas9xx/tmpa9xx.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __TOPAS9XX_H__
#define __TOPAS9XX_H__

extern void __init tmpa9xx_init_irq(void);
struct sys_timer;
extern struct sys_timer tmpa9xx_timer;
extern void __init tmpa9xx_init(void);
extern void __init tmpa9xx_map_io(void);
extern void __init baseboard_init(void);


#endif
