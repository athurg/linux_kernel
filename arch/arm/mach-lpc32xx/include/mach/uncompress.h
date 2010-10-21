/*
 * asm-arm/arch-lpc32xx/uncompress.h
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

#include <mach/platform.h>

#if defined(CONFIG_ARM9DIMM3250_UNCOMPRESS_ON_UART1)
#define HSUART_FIFO     (*(volatile unsigned char *)(HS_UART1_BASE + 0x00))
#define HSUART_LEVEL    (*(volatile unsigned short *)(HS_UART1_BASE + 0x04))

static inline void putc(int ch)
{
    /* Wait for transmit FIFO to empty */
    while ((HSUART_LEVEL & 0xFF00) != 0);
    HSUART_FIFO = ch;
}
static inline void flush(void)
{
    /* Don't see a reset? */
    /* Then just wait for transmition to complete */
    while ((HSUART_LEVEL & 0xFF00) != 0);
}

#else
/* Access UART with physical addresses before MMU is setup */
#define UART_DATA	(*(volatile unsigned long*) (UART5_BASE + 0x00))
#define UART_FIFO_CTL	(*(volatile unsigned long*) (UART5_BASE + 0x08))
#define UART_FIFO_CTL_TX_RESET (1 << 2)
#define UART_STATUS	(*(volatile unsigned long*) (UART5_BASE + 0x14))
#define UART_STATUS_TX_MT (1 << 6)

static inline void putc(int ch)
{
	/* Wait for transmit FIFO to empty */
	while ((UART_STATUS & UART_STATUS_TX_MT) == 0);

	UART_DATA = ch;
}

static inline void flush(void)
{
	UART_FIFO_CTL |= UART_FIFO_CTL_TX_RESET;
}
#endif

	/* NULL functions; we don't presently need them */
#define arch_decomp_setup()
#define arch_decomp_wdog()
