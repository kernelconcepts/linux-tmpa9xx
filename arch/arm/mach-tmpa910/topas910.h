/*
 *  linux/arch/arm/mach-topas910/topas910.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


extern void __init topas910_init_irq(void);

struct sys_timer;
extern struct sys_timer topas910_timer;
