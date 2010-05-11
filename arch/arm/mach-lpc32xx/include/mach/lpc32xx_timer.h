/*
 * asm-arm/arch-lpc32xx/lpc32xx_timer.h
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

#ifndef LPC32XX_TIMER_H
#define LPC32XX_TIMER_H

/**********************************************************************
* Timer/counter register offsets
**********************************************************************/

#define TIMER_IR(x)	(x + 0x00)
#define TIMER_TCR(x)	(x + 0x04)
#define TIMER_TC(x)	(x + 0x08)
#define TIMER_PR(x)	(x + 0x0C)
#define TIMER_PC(x)	(x + 0x10)
#define TIMER_MCR(x)	(x + 0x14)
#define TIMER_MR0(x)	(x + 0x18)
#define TIMER_MR1(x)	(x + 0x1C)
#define TIMER_MR2(x)	(x + 0x20)
#define TIMER_MR3(x)	(x + 0x24)
#define TIMER_CCR(x)	(x + 0x28)
#define TIMER_CR0(x)	(x + 0x2C)
#define TIMER_CR1(x)	(x + 0x30)
#define TIMER_CR2(x)	(x + 0x34)
#define TIMER_CR3(x)	(x + 0x38)
#define TIMER_EMR(x)	(x + 0x3C)
#define TIMER_CTCR(x)	(x + 0x70)

/**********************************************************************
* ir register definitions
* Write a '1' to clear interrupt, reading a '1' indicates active int
**********************************************************************/
/* Macro for getting a timer match interrupt bit */
#define TIMER_CNTR_MTCH_BIT(n)     (1 << ((n) & 0x3))

/* Macro for getting a capture event interrupt bit */
#define TIMER_CNTR_CAPT_BIT(n)     (1 << (4 + ((n) & 0x3)))

/**********************************************************************
* tcr register definitions
**********************************************************************/
/* Timer/counter enable bit */
#define TIMER_CNTR_TCR_EN          0x1

/* Timer/counter reset bit */
#define TIMER_CNTR_TCR_RESET       0x2

/**********************************************************************
* mcr register definitions
**********************************************************************/
/* Bit location for interrupt on MRx match, n = 0 to 3 */
#define TIMER_CNTR_MCR_MTCH(n)     (0x1 << ((n) * 3))

/* Bit location for reset on MRx match, n = 0 to 3 */
#define TIMER_CNTR_MCR_RESET(n)    (0x1 << (((n) * 3) + 1))

/* Bit location for stop on MRx match, n = 0 to 3 */
#define TIMER_CNTR_MCR_STOP(n)     (0x1 << (((n) * 3) + 2))

/**********************************************************************
* ccr register definitions
**********************************************************************/
/* Bit location for CAP.n on CRx rising edge, n = 0 to 3 */
#define TIMER_CNTR_CCR_CAPNRE(n)   (0x1 << ((n) * 3))

/* Bit location for CAP.n on CRx falling edge, n = 0 to 3 */
#define TIMER_CNTR_CCR_CAPNFE(n)   (0x1 << (((n) * 3) + 1))

/* Bit location for CAP.n on CRx interrupt enable, n = 0 to 3 */
#define TIMER_CNTR_CCR_CAPNI(n)    (0x1 << (((n) * 3) + 2))

/**********************************************************************
* emr register definitions
**********************************************************************/
/* Bit location for output state change of MAT.n when external match
   happens, n = 0 to 3 */
#define TIMER_CNTR_EMR_DRIVE(n)    (1 << (n))

/* Macro for setting MAT.n soutput state */
#define TIMER_CNTR_EMR_DRIVE_SET(n, s) (((s) & 0x1) << (n))

/* Output state change of MAT.n when external match happens */
#define TIMER_CNTR_EMR_NOTHING     0x0
#define TIMER_CNTR_EMR_LOW         0x1
#define TIMER_CNTR_EMR_HIGH        0x2
#define TIMER_CNTR_EMR_TOGGLE      0x3

/* Macro for setting for the MAT.n change state bits */
#define TIMER_CNTR_EMR_EMC_SET(n, s) (((s) & 0x3) << (4 + ((n) * 2)))

/* Mask for the MAT.n change state bits */
#define TIMER_CNTR_EMR_EMC_MASK(n) (0x3 << (4 + ((n) * 2)))

/**********************************************************************
* ctcr register definitions
**********************************************************************/
/* Mask to get the Counter/timer mode bits */
#define TIMER_CNTR_CTCR_MODE_MASK  0x3

/* Mask to get the count input select bits */
#define TIMER_CNTR_CTCR_INPUT_MASK 0xC

/* Counter/timer modes */
#define TIMER_CNTR_CTCR_TIMER_MODE 0x0
#define TIMER_CNTR_CTCR_TCINC_MODE 0x1
#define TIMER_CNTR_CTCR_TCDEC_MODE 0x2
#define TIMER_CNTR_CTCR_TCBOTH_MODE 0x3

/* Count input selections */
#define TIMER_CNTR_CTCR_INPUT_CAP0 0x0
#define TIMER_CNTR_CTCR_INPUT_CAP1 0x1
#define TIMER_CNTR_CTCR_INPUT_CAP2 0x2
#define TIMER_CNTR_CTCR_INPUT_CAP3 0x3

/* Macro for setting the counter/timer mode */
#define TIMER_CNTR_SET_MODE(n)     ((n) & 0x3)

/* Macro for setting the count input select */
#define TIMER_CNTR_SET_INPUT(n)    (((n) & 0x3) << 2)

#endif /* LPC32XX_TIMER_H */
