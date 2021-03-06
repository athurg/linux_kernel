/*
 * asm-arm/arch-lpc32xx/system.h
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

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/platform.h>
#include "../../sys-lpc32xx.h"

static void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode)
{
	switch (mode) {
		case 's':
		case 'h':
			printk(KERN_CRIT "RESET: Rebooting system\n");
			/* Disable interrupts */
			local_irq_disable();
			lpc32xx_watchdog_reset();
			break;

		default:
			/* Do nothing */
			break;
	}

	/* Wait for watchdog to reset system */
	while (1) ;
}

#endif
