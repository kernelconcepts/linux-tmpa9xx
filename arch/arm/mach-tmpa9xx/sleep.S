/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR /PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

.macro  mov32   rd, val

	mov	\rd, #(\val) & 0xff000000
	orr	\rd, \rd, #(\val) & 0x00ff0000
	orr	\rd, \rd, #(\val) & 0x0000ff00
	orr	\rd, \rd, #(\val) & 0x000000ff
.endm

	.text
/*
 * Move tmp9xx into deep sleep state
 *
 */
ENTRY(tmpa9xx_cpu_suspend)
	stmfd	sp!, {r0-r12, lr}		@ save registers on stack

	ldr 	ip, CACHE_FLUSH
	blx	ip

	ldr 	ip, DO_IDLE
	blx	ip
	
	/* restore registers and return */
	ldmfd   sp!, {r0-r12, pc}

ENDPROC(tmpa9xx_cpu_suspend)

CACHE_FLUSH:
	.word	arm926_flush_kern_cache_all
DO_IDLE:
	.word	cpu_arm926_do_idle

ENTRY(tmpa9xx_cpu_suspend_sz)
	.word	. - tmpa9xx_cpu_suspend
ENDPROC(tmpa9xx_cpu_suspend_sz)
