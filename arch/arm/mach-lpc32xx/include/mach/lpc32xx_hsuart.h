/*
 * asm-arm/arch-lpc32xx/lpc32xx_clkpwr.h
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

#ifndef LPC32XX_HSUART_H
#define LPC32XX_HSUART_H

#define _BIT(n) (1 << (n))
#define _SBF(f,v) (((v)) << (f))

/**********************************************************************
* High speed UART register offsets
**********************************************************************/

#define HSUART_FIFO(x)			(x + 0x00)
#define HSUART_LEVEL(x)			(x + 0x04)
#define HSUART_IIR(x)			(x + 0x08)
#define HSUART_CTRL(x)			(x + 0x0C)
#define HSUART_RATE(x)			(x + 0x10)

/**********************************************************************
* high speed UART receiver FIFO register definitions
**********************************************************************/
#define HSU_BREAK_DATA      _BIT(10)    /* break condition indicator*/
#define HSU_ERROR_DATA      _BIT(9)     /* framing error indicator */
#define HSU_RX_EMPTY        _BIT(8)     /* Rx FIFO empty status */

/**********************************************************************
* high speed UART level register definitions
**********************************************************************/
#define HSU_TX_LEV(n)       ((n>>8) & 0xFF)
#define HSU_RX_LEV(n)       ((n) & 0xFF)

/**********************************************************************
* high speed UART interrupt identification register definitions
**********************************************************************/
#define HSU_TX_INT_SET      _BIT(6)     /* set the Tx int flag */
#define HSU_RX_OE_INT       _BIT(5)     /* overrun int flag */
#define HSU_BRK_INT         _BIT(4)     /* break int flag */
#define HSU_FE_INT          _BIT(3)     /* framing error flag */
#define HSU_RX_TIMEOUT_INT  _BIT(2)     /* Rx timeout int flag */
#define HSU_RX_TRIG_INT     _BIT(1)     /* Rx FIFO trig level ind */
#define HSU_TX_INT          _BIT(0)     /* Tx interrupt flag */                  

/**********************************************************************
* high speed UART control register definitions
**********************************************************************/
#define HSU_HRTS_INV        _BIT(21)            /* HRTS invert ctrl */
#define HSU_HRTS_TRIG(n)    (_SBF(19,(n&0x3)))  /* HRTS trig level */
#define HSU_HRTS_TRIG_8B    0                   /* HRTS trig lev 8B */
#define HSU_HRTS_TRIG_16B   _BIT(19)            /* HRTS trig lev 16B */
#define HSU_HRTS_TRIG_32B   _BIT(20)            /* HRTS trig lev 32B */
#define HSU_HRTS_TRIG_48B   _SBF(19,0x3)        /* HRTS trig lev 48B */
#define HSU_HRTS_EN         _BIT(18)            /* HRTS enable ctrl */
#define HSU_TMO_CONFIG(n)   ((n) & _SBF(16,0x3))/* tout intconfig */
#define HSU_TMO_DISABLED    0                   /* Rx tmo disabled */
#define HSU_TMO_INACT_4B    _BIT(16)            /* tmo after 4 bits */
#define HSU_TMO_INACT_8B    _BIT(17)            /* tmo after 8 bits */
#define HSU_TMO_INACT_16B   _SBF(16,0x3)        /* tmo after 16 bits */
#define HSU_HCTS_INV        _BIT(15)            /* HCTS inverted */
#define HSU_HCTS_EN         _BIT(14)            /* Tx flow control */
#define HSU_OFFSET(n)       _SBF(9,n)           /* 1st samplingpoint */
#define HSU_BREAK           _BIT(8)             /* break control */
#define HSU_ERR_INT_EN      _BIT(7)             /* HSUART err int en */
#define HSU_RX_INT_EN       _BIT(6)             /* HSUART Rx int en */
#define HSU_TX_INT_EN       _BIT(5)             /* HSUART Tx int en */
#define HSU_RX_TRIG(n)      ((n) & _SBF(2,0x7)) /* Rx FIFO trig lev */
#define HSU_RX_TL1B         _SBF(2,0x0)         /* Rx FIFO trig 1B  */
#define HSU_RX_TL4B         _SBF(2,0x1)         /* Rx FIFO trig 4B */
#define HSU_RX_TL8B         _SBF(2,0x2)         /* Rx FIFO trig 8B */
#define HSU_RX_TL16B        _SBF(2,0x3)         /* Rx FIFO trig 16B */
#define HSU_RX_TL32B        _SBF(2,0x4)         /* Rx FIFO trig 32B */
#define HSU_RX_TL48B        _SBF(2,0x5)         /* Rx FIFO trig 48B */
#define HSU_TX_TRIG(n)      ((n) & _SBF(0,0x3)) /* Tx FIFO trig lev */
#define HSU_TX_TLEMPTY      _SBF(0,0x0)         /* Tx FIFO trig empty */
#define HSU_TX_TL0B         _SBF(0,0x0)         /* Tx FIFO trig empty */
#define HSU_TX_TL4B         _SBF(0,0x1)         /* Tx FIFO trig 4B */
#define HSU_TX_TL8B         _SBF(0,0x2)         /* Tx FIFO trig 8B */
#define HSU_TX_TL16B        _SBF(0,0x3)         /* Tx FIFO trig 16B */

#endif /* LPC32XX_HSUART_H */ 
