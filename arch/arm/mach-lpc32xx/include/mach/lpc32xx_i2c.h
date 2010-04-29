/*
 * asm-arm/arch-lpc32xx/lpc32xx_i2c.h
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

#ifndef LPC32XX_I2C_H
#define LPC32XX_I2C_H

#define _BIT(n) (1 << (n))

/**********************************************************************
* I2C controller register offsets
**********************************************************************/

#define I2C_TXRX(x)			(x + 0x000)
#define I2C_STAT(x)			(x + 0x004)
#define I2C_CTRL(x)			(x + 0x008)
#define I2C_CLK_HI(x)			(x + 0x00C)
#define I2C_CLK_LO(x)			(x + 0x010)
#define I2C_ADR(x)			(x + 0x014)
#define I2C_RXFL(x)			(x + 0x018)
#define I2C_TXFL(x)			(x + 0x01C)
#define I2C_RXB(x)			(x + 0x020)
#define I2C_TXB(x)			(x + 0x024)
#define I2C_STX(x)			(x + 0x028)
#define I2C_STXFL(x)			(x + 0x02C)

/**********************************************************************
* i2c_txrx register definitions
**********************************************************************/
#define I2C_START    _BIT(8)		/* generate a START before this B*/
#define I2C_STOP     _BIT(9)		/* generate a STOP after this B */

/**********************************************************************
* i2c_stat register definitions
**********************************************************************/
#define I2C_TDI     _BIT(0)         /* Transaction Done Interrupt */
#define I2C_AFI     _BIT(1)         /* Arbitration Failure Interrupt */
#define I2C_NAI     _BIT(2)         /* No Acknowledge Interrupt */
#define I2C_DRMI    _BIT(3)         /* Master Data Request Interrupt */
#define I2C_DRSI    _BIT(4)         /* Slave Data Request Interrupt */
#define I2C_ACTIVE  _BIT(5)         /* Busy bus indicator */
#define I2C_SCL     _BIT(6)         /* The current SCL signal value */
#define I2C_SDA     _BIT(7)         /* The current SDA signal value */
#define I2C_RFF     _BIT(8)         /* Receive FIFO Full */
#define I2C_RFE     _BIT(9)         /* Receive FIFO Empty */
#define I2C_TFF     _BIT(10)        /* Transmit FIFO Full */
#define I2C_TFE     _BIT(11)        /* Transmit FIFO Empty */
#define I2C_TFFS    _BIT(12)        /* Slave Transmit FIFO Full */
#define I2C_TFES    _BIT(13)        /* Slave Transmit FIFO Empty */

/**********************************************************************
* i2c_ctrl register definitions
**********************************************************************/
#define I2C_TDIE    _BIT(0)         /* Transaction Done Int Enable */
#define I2C_AFIE    _BIT(1)         /* Arbitration Failure Int Ena */
#define I2C_NAIE    _BIT(2)         /* No Acknowledge Int Enable */
#define I2C_DRMIE   _BIT(3)         /* Master Data Request Int Ena */
#define I2C_DRSIE   _BIT(4)         /* Slave Data Request Int Ena */
#define I2C_RFFIE   _BIT(5)         /* Receive FIFO Full Int Ena */
#define I2C_RFDAIE  _BIT(6)         /* Rcv Data Available Int Ena */
#define I2C_TFFIE   _BIT(7)         /* Trnsmt FIFO Not Full Int Ena */
#define I2C_RESET   _BIT(8)         /* Soft Reset */
#define I2C_SEVEN   _BIT(9)         /* Seven-bit slave address */
#define I2C_TFFSIE  _BIT(10)        /* Slave Trnsmt FIFO Not Full IE */

/**********************************************************************
* i2c channel select
**********************************************************************/

/* Macro pointing to I2C controller registers */
#define I2C1  ((I2C_REGS_T  *)(I2C1_BASE))
#define I2C2  ((I2C_REGS_T  *)(I2C2_BASE))

#define I2C_RX_BUFFER_SIZE  4
#define I2C_TX_BUFFER_SIZE  4
#define I2C_STX_BUFFER_SIZE 4

#ifdef __cplusplus
}
#endif

#endif /* LPC32XX_I2C_H */
