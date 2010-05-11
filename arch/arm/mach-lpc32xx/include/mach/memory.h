/*
 * asm-arm/arch-lpc32xx/memory.h
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

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Physical DRAM offset.
 */
#define PHYS_OFFSET     (0x80000000)

#define __virt_to_bus(x)	 __virt_to_phys(x)
#define __bus_to_virt(x)	 __phys_to_virt(x)

#if defined (CONFIG_ARCH_DISCONTIGMEM_ENABLE)
/*
 * The LPC32XX supports 2 DRAM banks on physical addresss 0x80000000 (DYCS0)
 * and 0xA0000000 (DYCS1). A hole exists between the 2 banks making the
 * memory non-contiguous.
 *
 * This change allows support for both of those banks and is only needed on
 * systems with 2 DRAM devices on different chip selects.
 * 
 * 	node 0:  0x80000000 - 0x83ffffff
 * 	node 1:  0xa0000000 - 0xa3ffffff
 */
#define NODE_MEM_SIZE_BITS	28

#endif

#endif

