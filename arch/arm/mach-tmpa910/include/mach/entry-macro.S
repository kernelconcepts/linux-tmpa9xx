/*
 * include/asm-arm/arch-tmpa910/entry-macro.S
 *
 * Low-level IRQ helper macros for TMPA910 platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <mach/hardware.h>


                .macro  get_irqnr_preamble, base, tmp
                .endm

                .macro  arch_ret_to_user, tmp1, tmp2
                .endm


                .macro  disable_fiq
                .endm

                .macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
                ldr     \irqstat, =IO_ADDRESS(0xf4000000) @ int ctrl
		
                ldr     \base, =0                  @ invalid interrupt
		ldr     \tmp, [\irqstat, #0x000]        @ 0xf00 <- vic addr
		cmp     \tmp, \base
		beq	1002f
		
                ldr     \irqnr, [\irqstat, #0xf00]        @ 0xf00 <- vic addr

		# hack it here?
		#str 	\irqnr, [\irqstat, #0xf00]        @ 0xf00 <- vic addr

                ldr     \irqstat, =1                    @ dummy compare
                ldr     \base, =32                  @ invalid interrupt
                cmp     \irqnr, \base
                bne     1001f
1002:
                ldr     \irqstat, =0
1001:
                tst     \irqstat, #1                    @ to make the condition code = TRUE
                .endm

                .macro  irq_prio_table
                .endm


