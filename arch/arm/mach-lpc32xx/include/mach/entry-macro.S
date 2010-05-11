/*
 * asm-arm/arch-lpc32xx/entry-macro.S
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2008 NXP Semiconductors
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

#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/irqs.h>

/*
 * Return IRQ number in irqnr. Also return processor Z flag status in CPSR
 * as set if an interrupt is pending.
 */
	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
	/* Get MIC status first */
	ldr	\base, =io_p2v(MIC_BASE)
	ldr	\irqstat, [\base, #INTC_STAT]
	and	\irqstat, \irqstat, #0xFFFFFFFC
	mov	\tmp, #0

	/* If the masked IRQ status is 0, then the interrupt was due to
	   SIC1 or SIC2 */
	cmp	\irqstat, #0
	bne	1000f

	/* Check SIC1 to see if an interrupt is pending */
	ldr	\base, =io_p2v(SIC1_BASE)
	ldr	\irqstat, [\base, #INTC_STAT]
	mov	\tmp, #32

	/* If the SIC1 status is 0, then the interrupt falls to SIC2 */
	cmp	\irqstat, #0
	bne	1000f

	/* Check SIC2 to see if an interrupt is pending */
	ldr	\base, =io_p2v(SIC2_BASE)
	ldr	\irqstat, [\base, #INTC_STAT]
	mov	\tmp, #64

	/* If the SIC1 status is 0, then the interrupt falls to SIC2 */
	cmp	\irqstat, #0
	beq	1001f

1000:
	clz	\irqnr, \irqstat
	rsb	\irqnr, \irqnr, #31
	add	\irqnr, \irqnr, \tmp /* Interrupt # between 0 and 95 */

1001:
	teq	\irqstat, #0
	.endm

	.macro	disable_fiq
	.endm

	.macro  get_irqnr_preamble, base, tmp
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

//	.macro	irq_prio_table
//	.endm



