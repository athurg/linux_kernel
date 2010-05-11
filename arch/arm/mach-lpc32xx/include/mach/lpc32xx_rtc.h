/*
 * asm-arm/arch-lpc32xx/lpc32xx_rtc.h
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

#ifndef LPC32XX_RTC_H
#define LPC32XX_RTC_H

#define _BIT(n) (1 << (n))

/**********************************************************************
* Clock and Power control register offsets
**********************************************************************/

#define RTC_UCOUNT(x)			(x + 0x00)
#define RTC_DCOUNT(x)			(x + 0x04)
#define RTC_MATCH0(x)			(x + 0x08)
#define RTC_MATCH1(x)			(x + 0x0C)
#define RTC_CTRL(x)			(x + 0x10)
#define RTC_INTSTAT(x)			(x + 0x14)
#define RTC_KEY(x)			(x + 0x18)
#define RTC_SRAM(x, y)			(x + 0x80 + (y * 4))

/**********************************************************************
* ctrl register definitions
**********************************************************************/
/* Bit for enabling RTC match 0 interrupt */
#define RTC_MATCH0_EN              _BIT(0)
/* Bit for enabling RTC match 1 interrupt */
#define RTC_MATCH1_EN              _BIT(1)
/* Bit for enabling ONSW signal on match 0 */
#define RTC_ONSW_MATCH0_EN         _BIT(2)
/* Bit for enabling ONSW signal on match 1 */
#define RTC_ONSW_MATCH1_EN         _BIT(3)
/* Bit for performing an RC software reset, must be cleared after set */
#define RTC_SW_RESET               _BIT(4)
/* Bit for disabling 1Hz up/down counters */
#define RTC_CNTR_DIS               _BIT(6)
/* Bit for forcing ONSW high (1) or default state */
#define RTC_ONSW_FORCE_HIGH        _BIT(7)

/**********************************************************************
* intstat register definitions
**********************************************************************/
/* Bit for match 0 interrupt status or clearing latched state */
#define RTC_MATCH0_INT_STS         _BIT(0)
/* Bit for match 1 interrupt status or clearing latched state */
#define RTC_MATCH1_INT_STS         _BIT(1)
/* Bit for ONSW interrupt status or clearing ONSW high state */
#define RTC_ONSW_INT_STS           _BIT(2)

/**********************************************************************
* key register definitions
**********************************************************************/
/* RTC key register enable value (must be loaded for ONSW to work) */
#define RTC_KEY_ONSW_LOADVAL       0xB5C13F27

#endif /* LPC32XX_RTC_H */
