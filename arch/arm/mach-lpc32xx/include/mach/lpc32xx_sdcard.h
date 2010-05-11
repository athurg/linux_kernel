/*
 * asm-arm/arch-lpc32xx/lpc32xx_sdcard.h
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

#ifndef LPC32XX_SDCARD_H
#define LPC32XX_SDCARD_H

#define _BIT(n) (1 << (n))

/**********************************************************************
* SD Card controller register offsets
**********************************************************************/

#define SD_POWER(x)			(x + 0x00)
#define SD_CLOCK(x)			(x + 0x04)
#define SD_ARG(x)			(x + 0x08)
#define SD_CMD(x)			(x + 0x10)
#define SD_RESPCMD(x)			(x + 0x14)
#define SD_RESP(x)			(x + 0x18)
#define SD_DTIMER(x)			(x + 0x28)
#define SD_DLEN(x)			(x + 0x2C)
#define SD_DCTRL(x)			(x + 0x30)
#define SD_DCNT(x)			(x + 0x34)
#define SD_STATUS(x)			(x + 0x38)
#define SD_CLEAR(x)			(x + 0x3C)
#define SD_MASK0(x)			(x + 0x40)
#define SD_MASK1(x)			(x + 0x44)
#define SD_FIFOCNT(x)			(x + 0x4C)
#define SD_FIFO(x)			(x + 0x80)

/**********************************************************************
* sd_power register definitions
**********************************************************************/
/* SD bit for enabling open drain mode (1) or pushpull mode (0) */
#define SD_OPENDRAIN_EN            _BIT(6)
/* SD power control mode: power off */
#define SD_POWER_OFF_MODE          0x0
/* SD power control mode: power up */
#define SD_POWER_UP_MODE           0x2
/* SD power control mode: power on */
#define SD_POWER_ON_MODE           0x3
/* SD power control mode mask */
#define SD_POWER_MODE_MASK         0x3

/**********************************************************************
* sd_clock register definitions
**********************************************************************/
/* SD bit for enabling side bus mode */
#define SD_WIDEBUSMODE_EN          _BIT(11)
/* SD bit for enabling SDCLK clock bypass */
#define SD_SDCLK_BYPASS            _BIT(10)
/* SD bit for enabling clock throttling during idle states */
#define SD_SDCLK_PWRSAVE           _BIT(9)
/* SD bit for enabling the SD clock */
#define SD_SDCLK_EN                _BIT(8)
/* SD clock divider bit mask */
#define SD_CLKDIV_MASK             0xFF

/**********************************************************************
* sd_cmd register definitions
**********************************************************************/
/* SD bit for enabling command path state machine */
#define SD_CPST_EN                 _BIT(10)
/* SD bit for wait for CMDPEND prior to sending command */
#define SD_CMDPEND_WAIT            _BIT(9)
/* SD bit for enabling card interrupt request (without timeout) */
#define SD_INTERRUPT_EN            _BIT(8)
/* SD bit for enabling 136-bit response support */
#define SD_LONGRESP_EN             _BIT(7)
/* SD bit for enabling response support */
#define SD_RESPONSE                _BIT(6)
/* SD command mask */
#define SD_CMD_MASK                0x3F

/**********************************************************************
* sd_dctrl register definitions
**********************************************************************/
/* SD data transfer blocksize of 1 byte */
#define SD_BLKSIZE_1BYTE           0x00
/* SD data transfer blocksize of 2 bytes */
#define SD_BLKSIZE_2BYTES          0x10
/* SD data transfer blocksize of 4 bytes */
#define SD_BLKSIZE_4BYTES          0x20
/* SD data transfer blocksize of 8 bytes */
#define SD_BLKSIZE_8BYTES          0x30
/* SD data transfer blocksize of 16 bytes */
#define SD_BLKSIZE_16BYTES         0x40
/* SD data transfer blocksize of 32 bytes */
#define SD_BLKSIZE_32BYTES         0x50
/* SD data transfer blocksize of 64 bytes */
#define SD_BLKSIZE_64BYTES         0x60
/* SD data transfer blocksize of 128 bytes */
#define SD_BLKSIZE_128BYTES        0x70
/* SD data transfer blocksize of 256 bytes */
#define SD_BLKSIZE_256BYTES        0x80
/* SD data transfer blocksize of 512 bytes */
#define SD_BLKSIZE_512BYTES        0x90
/* SD data transfer blocksize of 1024 bytes */
#define SD_BLKSIZE_1024BYTES       0xA0
/* SD data transfer blocksize of 2048 bytes */
#define SD_BLKSIZE_2048BYTES       0xB0
/* SD bit for enabling DMA */
#define SD_DMA_EN                  _BIT(3)
/* SD bit for enabling a stream transfer */
#define SD_STREAM_EN               _BIT(2)
/* SD direction bit (1 = receive, 0 = transmit) */
#define SD_DIR_FROMCARD            _BIT(1)
/* SD data transfer enable bit */
#define SD_DATATRANSFER_EN         _BIT(0)

/**********************************************************************
* sd_status register definitions
* sd_clear register definitions (bits 0..10 only)
* sd_mask0, sd_mask1 register definitions
**********************************************************************/
/* SD bit for data receive FIFO NOT empty status */
#define SD_FIFO_RXDATA_AVAIL       _BIT(21)
/* SD bit for data transmit FIFO NOT empty status */
#define SD_FIFO_TXDATA_AVAIL       _BIT(20)
/* SD bit for data receive FIFO empty status */
#define SD_FIFO_RXDATA_EMPTY       _BIT(19)
/* SD bit for data transmit FIFO empty status */
#define SD_FIFO_TXDATA_EMPTY       _BIT(18)
/* SD bit for data receive FIFO full status */
#define SD_FIFO_RXDATA_FULL        _BIT(17)
/* SD bit for data transmit FIFO full status */
#define SD_FIFO_TXDATA_FULL        _BIT(16)
/* SD bit for data receive FIFO half-full status */
#define SD_FIFO_RXDATA_HFULL       _BIT(15)
/* SD bit for data transmit FIFO half-empty status */
#define SD_FIFO_TXDATA_HEMPTY      _BIT(14)
/* SD bit for data receive in progress status */
#define SD_RX_INPROGRESS           _BIT(13)
/* SD bit for data transmit in progress status */
#define SD_TX_INPROGRESS           _BIT(12)
/* SD bit for command transfer in progress status */
#define SD_CMD_INPROGRESS          _BIT(11)
/* SD bit for data block send/received complete (CRC good) status */
#define SD_DATABLK_END             _BIT(10)
/* SD bit for start bit detection error status */
#define SD_STARTBIT_ERR            _BIT(9)
/* SD bit for data end (data counter is 0) status */
#define SD_DATA_END                _BIT(8)
/* SD bit for command sent status */
#define SD_CMD_SENT                _BIT(7)
/* SD bit for command response received (CRC good) status */
#define SD_CMD_RESP_RECEIVED       _BIT(6)
/* SD bit for data receive FIFO overflow status */
#define SD_FIFO_RXDATA_OFLOW       _BIT(5)
/* SD bit for data transmit FIFO underflow status */
#define SD_FIFO_TXDATA_UFLOW       _BIT(4)
/* SD bit for data timeout status */
#define SD_DATA_TIMEOUT            _BIT(3)
/* SD bit for command timeout status */
#define SD_CMD_TIMEOUT             _BIT(2)
/* SD bit for data CRC failure status */
#define SD_DATA_CRC_FAIL           _BIT(1)
/* SD bit for command CRC failure status */
#define SD_CMD_CRC_FAIL            _BIT(0)

#endif /* LPC32XX_SDCARD_H */
