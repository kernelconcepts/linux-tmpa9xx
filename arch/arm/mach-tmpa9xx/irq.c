/*
 *  arch/arm/mach-tmpa9xx/irq.c
 *
 *  Copyright (C) 2008 bplan GmbH
 *  Copyright (C) 2009, 2010 Florian Boor <florian.boor@kernelconcepts.de>
 *
 *  Based on mach-mx1ads/irq.c, which is:
 *    Copyright (C) 1999 ARM Limited
 *    Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  TMPA9XX main interrupt handling
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <asm/mach/irq.h>
#include <mach/gpio.h>
#include <mach/regs.h>


struct hw_ictl
{
        uint32_t vicirqstatus;              // 0x0000
        uint32_t vicfiqstatus;              // 0x0004
        uint32_t vicrawintr;                // 0x0008
        uint32_t vicintselect;              // 0x000c
        uint32_t vicintenable;              // 0x0010
        uint32_t vicintenclear;             // 0x0014
        uint32_t vicsoftint;                // 0x0018
        uint32_t vicsoftintclear;           // 0x001c
        uint32_t vicprotection;             // 0x0020
        uint32_t vicswprioritymask;         // 0x0024
        uint32_t rsd[54];                   // 0x0028 - 0x00fC
        uint32_t vicvectaddr[32];           // 0x0100 - 0x017c
        uint32_t rsd2[32];                  // 0x0180 - 0x01fc
        uint32_t vicvectpriority[32];       // 0x0200 - 0x027c
        uint32_t rsd3[32 + 0xc00/4];        // 0x0280 - 0x0dfc
        uint32_t vicaddress;                // 0x0f00
};


static void tmpa9xx_ena_irq(unsigned int irq) {

        volatile struct hw_ictl *hw_ictl = (volatile struct hw_ictl *) INTR_BASE;
    
        hw_ictl->vicintenable = 1 << irq;
}


static void tmpa9xx_dis_irq(unsigned int irq) {

        volatile struct hw_ictl *hw_ictl = (volatile struct hw_ictl *) INTR_BASE;

        hw_ictl->vicintenclear = 1 << irq;
}


static void tmpa9xx_ack_irq(unsigned int irq) {

        volatile struct hw_ictl *hw_ictl = (volatile struct hw_ictl *) INTR_BASE;

        hw_ictl->vicintenclear = 1 << irq;
        hw_ictl->vicaddress = 0x12345678;
}


static void tmpa9xx_end_irq(unsigned int irq) {

        volatile struct hw_ictl *hw_ictl = (volatile struct hw_ictl *) INTR_BASE;
        
        if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS))) {
                hw_ictl->vicintenable = 1 << irq;
        }
}


static struct irq_chip  tmpa9xx_chip = {
        .typename  = "tmpa9xx",

        .ack       = tmpa9xx_ack_irq,
        .end       = tmpa9xx_end_irq,

        .mask             = tmpa9xx_dis_irq,
        .unmask    = tmpa9xx_ena_irq,
};


void __init tmpa9xx_init_irq(void)
{
        volatile struct hw_ictl *hw_ictl = (volatile struct hw_ictl *) INTR_BASE;
        int i;

        /* Every interrupt to the IRQ execpetion */
        hw_ictl->vicintselect = 0;

        /* make sure every pri unmasked */
        hw_ictl->vicswprioritymask  = 0xffffffff;

        /* this help to obtain the interrupt vector in the service call */
        for(i=0; i < 32; i++)
                hw_ictl->vicvectaddr[i] = i;

        for (i = 0; i < TMPA9XX_NUM_IRQS; i++) {
                set_irq_chip(i, &tmpa9xx_chip);
                set_irq_handler(i, handle_level_irq);
                set_irq_flags(i, IRQF_VALID );
        }

            /* Now register GPIO Pins and IRQs */
        tmpa9xx_gpio_init();
}