/*
 *  arch/arm/mach-tmpa9xx/ofd.c
 *
 *  Copyright (C) 2010 Thomas Haase <Thomas.Haase@web.de>
 *
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
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <mach/regs.h>

#define OFD_CLKSCR1            __REG(OFD_BASE_ADDRESS + 0x0000)
#define OFD_CLKSCR2            __REG(OFD_BASE_ADDRESS + 0x0004)
#define OFD_CLKSCR3            __REG(OFD_BASE_ADDRESS + 0x0008)
#define OFD_CLKSMN             __REG(OFD_BASE_ADDRESS + 0x0010)
#define OFD_CLKSMX             __REG(OFD_BASE_ADDRESS + 0x0020)

#define FOSCH 24000000		/* 24 MHz oscillator clock */

#define CLK_WRITE_ENABLE  0xf9 /*  Enable writing to the clock registers */
#define CLK_WRITE_DISABLE 0x06 /* Disable writing to the clock registers */

#define CLK_S_ENABLE      0xe4 /*  Enable OFD operation */
#define CLK_S_DISABLE     0x00 /* Disable OFD operation */

#define OFD_RESET_ENABLE (1<<1)/* Enable OFD reset */
#define OFD_CLEAR_CLKSF  (1<<0)/* Clear High speed oscillation frequency detection flag */

static int tmpa9xx_ofd_enable(void)
{
	int i;

	/* Initialize the OFD Module */
	OFD_CLKSCR1 = CLK_WRITE_ENABLE;	/* Enable writing to the OFD registers */
	OFD_CLKSCR2 = CLK_S_DISABLE;	/* Enable OFD operation */
        udelay(1);
        OFD_CLKSCR3 = OFD_RESET_ENABLE 	/* RESEN Enable & Clear OSC Flag */
                      |OFD_CLEAR_CLKSF;

        for (i=0;i<100;i++)
        {
        	if ( (OFD_CLKSCR3 & OFD_RESET_ENABLE) == OFD_RESET_ENABLE)
        		break;
                udelay(1);
	}

        if (i>=99)
        	goto err;

        OFD_CLKSMN = (unsigned long) ((float)FOSCH * 0.9 / 32768.0 / 4.0 +0.5 );
        OFD_CLKSMX = (unsigned long) ((float)FOSCH * 1.1 / 32768.0 / 4.0 +0.5 );

        udelay(1);
	OFD_CLKSCR2 = CLK_S_ENABLE;		/* Enable OFD operation */
	OFD_CLKSCR1 = CLK_WRITE_DISABLE;	/* Disable writing to the OFD registers */
	printk(KERN_INFO "Enable OFD - success\n");

        return 0;
err:
	printk(KERN_INFO "Enable OFD - failed\n");
        return -EFAULT;
}

void tmpa9xx_ofd_disable(void)
{
	int i;

	OFD_CLKSCR1 = CLK_WRITE_ENABLE;	/* Enable writing to the OFD registers */
	OFD_CLKSCR2 = CLK_S_DISABLE;	/* Enable OFD operation */
        udelay(1);
        OFD_CLKSCR3 = OFD_CLEAR_CLKSF; 	/* RESEN Enable & Clear OSC Flag */

        for (i=0;i<100;i++)
        {
        	if ( (OFD_CLKSCR3 & OFD_RESET_ENABLE) != OFD_RESET_ENABLE)
        		break;
                udelay(1);
	}

        if (i>=99)
        	goto err;

	OFD_CLKSCR1 = CLK_WRITE_DISABLE;	/* Disable writing to the OFD registers */
	printk(KERN_INFO "Disable OFD - success\n");

        return;
err:
	printk(KERN_INFO "Disable OFD - failed\n");
	return;
}

static int __init tmpa9xx_ofd_init(void)
{
	return tmpa9xx_ofd_enable();
}

arch_initcall(tmpa9xx_ofd_init);
