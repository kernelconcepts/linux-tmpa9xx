/* 
 * The head file for Toshiba TMPA910
 * by Yin, Fengwei (fengwei.yin@gmail.com)
 * TMPA910 sepific. This is merged int head.S by the linker.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/mach-types.h>

		.section	".start", "ax"
tmpa910_start:
		@ Preserve r8/r7 i.e. kernel entry values

		@ Data cache might be active.
		@ Be sure to flush kernel binary out of the cache,
		@ whatever state it is, before it is turned off.
		@ This is done by fetching through currently executed
		@ memory to be sure we hit the same cache.
		bic	r2, pc, #0x1f
		add	r3, r2, #0x10000	@ 64 kb is quite enough...
1:		ldr	r0, [r2], #32
		teq	r2, r3
		bne	1b
		mcr	p15, 0, r0, c7, c10, 4	@ drain WB
		mcr	p15, 0, r0, c7, c7, 0	@ flush I & D caches

		@ disabling MMU and caches
		mrc	p15, 0, r0, c1, c0, 0	@ read control reg
		bic	r0, r0, #0x05		@ clear DC, MMU
		bic	r0, r0, #0x1000		@ clear Icache
		mcr	p15, 0, r0, c1, c0, 0
