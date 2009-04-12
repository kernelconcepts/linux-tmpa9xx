/*
 * arch/arm/mach-topas910/topas910.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __TOPAS910_H__
#define __TOPAS910_H__

extern void __init tmpa910_init_irq(void);

struct sys_timer;
extern struct sys_timer topas910_timer;

#endif
