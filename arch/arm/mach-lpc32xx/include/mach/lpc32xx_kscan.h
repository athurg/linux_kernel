/*
 * asm-arm/arch-lpc32xx/lpc32xx_kscan.h
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

#ifndef LPC32XX_KSCAN_H
#define LPC32XX_KSCAN_H

#define _BIT(n) (1 << (n))

/**********************************************************************
* Key scanner register offsets
**********************************************************************/

#define KS_DEB(x)		(x + 0x00)
#define KS_STATE_COND(x)	(x + 0x04)
#define KS_IRQ(x)		(x + 0x08)
#define KS_SCAN_CTL(x)		(x + 0x0C)
#define KS_FAST_TST(x)		(x + 0x10)
#define KS_MATRIX_DIM(x)	(x + 0x14)
#define KS_DATA(x)		(x + 0x40)

/**********************************************************************
* ks_deb register definitions
**********************************************************************/
/* Keypad debouncing register, number of equal matrix values read,
   n = 0 to 255 passes */
#define KSCAN_DEB_NUM_DEB_PASS(n)  ((n) & 0xFF)

/**********************************************************************
* ks_state_cond register definitions
**********************************************************************/
/* Keypad in the idle state */
#define KSCAN_SCOND_IN_IDLE        0x0
/* Keypad in the scan-once state */
#define KSCAN_SCOND_IN_SCANONCE    0x1
/* Keypad in the IRQ generation state */
#define KSCAN_SCOND_IN_IRQGEN      0x2
/* Keypad in the IRQ scan-matrix state */
#define KSCAN_SCOND_IN_SCAN_MATRIX 0x3

/**********************************************************************
* ks_irq register definitions
**********************************************************************/
/* Interrupt pending flag, write to clear */
#define KSCAN_IRQ_PENDING_CLR      0x1

/**********************************************************************
* ks_scan_ctl register definitions
**********************************************************************/
/* Time in clocks between each scan state in matrix mode, n = 1 to
   255, use 0 for scan always */
#define KSCAN_SCTRL_SCAN_DELAY(n)  ((n) & 0xFF)

/**********************************************************************
* ks_fast_tst register definitions
**********************************************************************/
/* Force scan-once state */
#define KSCAN_FTST_FORCESCANONCE   0x1
/* Select keypad scanner clock, 0 = PERIPH_CLK, 1 = 32K clock */
#define KSCAN_FTST_USE32K_CLK      0x2

/**********************************************************************
* ks_matrix_dim register definitions
**********************************************************************/
/* Matrix dimension selection clock, n = 1 to 8 for a 1x1 to 8x8
   matrix */
#define KSCAN_MSEL_SELECT(n)       ((n) & 0xF)

#endif /* LPC32XX_KSCAN_H */
