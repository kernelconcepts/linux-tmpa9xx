/*
 *  linux/include/asm-arm/arch-topas910/memory.h
 *
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright (C) 2006 Jochen Karrer (jk06@jkarrer.de)
 *  Copyright (C) 2008 bplan GmbH
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
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#define FB_SIZE   UL(2048*1024)
#define FB_OFFSET PHYS_OFFSET + MEM_SIZE 

#define PHYS_OFFSET     UL(0x40000000)
#define MEM_SIZE        ( UL(0x04000000) - FB_SIZE)
/*
 * Virtual view <-> DMA view memory address translations
 * virt_to_bus: Used to translate the virtual address to an
 *              address suitable to be passed to set_dma_addr
 * bus_to_virt: Used to convert an address for DMA operations
 *              to an address that the kernel can use.
 */
#define __virt_to_bus__is_a_macro
#define __virt_to_bus(x)        (x - PAGE_OFFSET +  PHYS_OFFSET)
#define __bus_to_virt__is_a_macro
#define __bus_to_virt(x)        (x -  PHYS_OFFSET + PAGE_OFFSET)

#endif

