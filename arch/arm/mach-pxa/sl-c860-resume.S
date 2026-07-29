/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Qualified low-level suspend/resume support for the Sharp SL-C860.
 *
 * The factory NOR resume dispatcher branches through PSPR before SDRAM and
 * the normal Linux execution environment are available. The first routine
 * below is copied to the first physical SDRAM page and entered MMU-off,
 * stackless, and with no relocation-dependent data accesses.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

#include "pxa2xx-regs.h"
#include "smemc.h"

	.text
	.arm
	.align	5

/*
 * Follow the same saved-context chain as cpu_resume:
 *
 *   sleep_save_sp.save_ptr_stash_phys
 *     -> per-CPU physical context pointer
 *     -> { physical pgd, virtual sp, physical cpu_do_resume }
 *
 * Every physical pointer must lie in the retained 64 MiB SDRAM window and be
 * word-aligned. The saved stack must be in the kernel half, and the final
 * resume function must exactly match the value staged while awake.
 */
ENTRY(sl_c860_resume_trampoline)
	adr	r4, sl_c860_resume_trampoline_context_cell
	ldmia	r4, {r4-r5}

	mov	r6, r4, lsr #26
	cmp	r6, #0x28
	bne	1f
	tst	r4, #3
	bne	1f

	ldr	r0, [r4]
	mov	r6, r0, lsr #26
	cmp	r6, #0x28
	bne	1f
	tst	r0, #3
	bne	1f

	ldr	r0, [r0]
	mov	r6, r0, lsr #26
	cmp	r6, #0x28
	bne	1f
	tst	r0, #3
	bne	1f

	ldr	r1, [r0]
	mov	r6, r1, lsr #26
	cmp	r6, #0x28
	bne	1f
	tst	r1, #3
	bne	1f

	ldr	r2, [r0, #4]
	mov	r6, r2, lsr #30
	cmp	r6, #3
	bne	1f
	tst	r2, #3
	bne	1f

	ldr	r3, [r0, #8]
	cmp	r3, r5
	bne	1f
	mov	r6, r3, lsr #26
	cmp	r6, #0x28
	bne	1f
	tst	r3, #3
	bne	1f

	/*
	 * Match cpu_resume's final ARM register contract. cpu_do_resume does
	 * not use the stack before it restores CP15 state and enables the MMU.
	 */
	msr	CPSR_c, #0xd3
	add	r0, r0, #12
	mov	sp, r2
	mov	pc, r3

1:	b	1b
ENDPROC(sl_c860_resume_trampoline)

	.global	sl_c860_resume_trampoline_context_cell
sl_c860_resume_trampoline_context_cell:
	.word	0
	.global	sl_c860_resume_trampoline_cpu_do_resume
sl_c860_resume_trampoline_cpu_do_resume:
	.word	0
	.global	sl_c860_resume_trampoline_end
sl_c860_resume_trampoline_end:

/*
 * The SL-C860-qualified PXA255 sleep path intentionally does not perform the
 * generic late frequency transition. Generic suspend has already cleaned the
 * saved context; the loop leaves no valid D-cache lines, matching Sharp's
 * board kernel. The three final operations occupy one 32-byte I-cache line.
 */
ENTRY(sl_c860_pxa25x_finish_suspend)
	mov	r1, r0
	mov	r2, #UNCACHED_PHYS_0
	ldr	r4, =MDREFR
	ldr	r5, [r4]
	orr	r5, r5, #MDREFR_SLFRSH
	b	2f
	.ltorg
2:
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4		@ drain write/fill buffer
	mov	r0, #0xf6000000
	mov	r6, #1024
3:
	mcr	p15, 0, r0, c7, c2, 5		@ allocate D-cache line
	mcr	p15, 0, r0, c7, c6, 1		@ invalidate same line
	add	r0, r0, #32
	subs	r6, r6, #1
	bne	3b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4		@ drain write/fill buffer
	b	4f
	.align	5
4:
	str	r5, [r4]			@ enter SDRAM self-refresh
	ldr	r3, [r2]			@ force address lines low
	mcr	p14, 0, r1, c7, c0, 0		@ enter PWRMODE sleep
5:	b	5b
ENDPROC(sl_c860_pxa25x_finish_suspend)
