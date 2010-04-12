/*
 * asm-arm/arch-lpc32xx/lpc32xx_ssp.h
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

#ifndef LPC32XX_SSP_H
#define LPC32XX_SSP_H

#define _BIT(n) (1 << (n))
#define _SBF(f,v) (((v)) << (f))

/**********************************************************************
* Key scanner register offsets
**********************************************************************/

#define SSP_CR0(x)		(x + 0x00)
#define SSP_CR1(x)		(x + 0x04)
#define SSP_DATA(x)		(x + 0x08)
#define SSP_SR(x)		(x + 0x0C)
#define SSP_CPSR(x)		(x + 0x10)
#define SSP_IMSC(x)		(x + 0x14)
#define SSP_RIS(x)		(x + 0x18)
#define SSP_MIS(x)		(x + 0x1C)
#define SSP_ICR(x)		(x + 0x20)
#define SSP_DMACR(x)		(x + 0x24)

/***********************************************************************
 * cr0 register definitions
 **********************************************************************/
/* SSP data size load macro, must be 4 bits to 16 bits */
#define SSP_CR0_DSS(n)   _SBF(0, (((n) - 1) & 0xF)) // Data Size Select
/* SSP control 0 Motorola SPI mode */
#define SSP_CR0_FRF_SPI  0x00000000
/* SSP control 0 TI synchronous serial mode */
#define SSP_CR0_FRF_TI   0x00000010
/* SSP control 0 National Microwire mode */
#define SSP_CR0_FRF_NS   0x00000020
/* SSP control 0 protocol mask */
#define SSP_CR0_PRT_MSK  0x00000030
/* SPI clock polarity bit (used in SPI mode only), (1) = maintains the
   bus clock high between frames, (0) = low */
#define SSP_CR0_CPOL(n)  _SBF(6, ((n) & 0x01))
/* SPI clock out phase bit (used in SPI mode only), (1) = captures data
   on the second clock transition of the frame, (0) = first */
#define SSP_CR0_CPHA(n)  _SBF(7, ((n) & 0x01))
/* SSP serial clock rate value load macro, divider rate is
   PERIPH_CLK / (cpsr * (SCR + 1)) */
#define SSP_CR0_SCR(n)   _SBF(8, ((n) & 0xFF))

/***********************************************************************
 * cr1 register definitions
 **********************************************************************/
/* SSP control 1 loopback mode enable bit */
#define SSP_CR1_LBM         _BIT(0)
/* SSP control 1 enable bit */
#define SSP_CR1_SSE(n)      _SBF(1, ((n) & 0x01))
#define SSP_CR1_SSP_ENABLE  _BIT(1)
#define SSP_CR1_SSP_DISABLE 0
/* SSP control 1 master/slave bit, (1) = master, (0) = slave */
#define SSP_CR1_MS       _BIT(2)
#define SSP_CR1_MASTER   0
#define SSP_CR1_SLAVE    _BIT(2)
/* SSP control 1 slave out disable bit, disables transmit line in slave
   mode */
#define SSP_CR1_SOD      _BIT(3)

/***********************************************************************
 * data register definitions
 **********************************************************************/
/* SSP data load macro */
#define SSP_DATAMASK(n)   ((n) & 0xFFFF)

/***********************************************************************
 * SSP status register (sr) definitions
 **********************************************************************/
/* SSP status TX FIFO Empty bit */
#define SSP_SR_TFE      _BIT(0)
/* SSP status TX FIFO not full bit */
#define SSP_SR_TNF      _BIT(1)
/* SSP status RX FIFO not empty bit */
#define SSP_SR_RNE      _BIT(2)
/* SSP status RX FIFO full bit */
#define SSP_SR_RFF      _BIT(3)
/* SSP status SSP Busy bit */
#define SSP_SR_BSY      _BIT(4)

/***********************************************************************
 * SSP clock prescaler register (cpsr) definitions
 **********************************************************************/
/* SSP clock prescaler load macro */
#define SSP_CPSR_CPDVSR(n) _SBF(0, (n) & 0xFE)

/***********************************************************************
 * SSP interrupt registers (imsc, ris, mis, icr) definitions
 **********************************************************************/
/* SSP interrupt bit for RX FIFO overflow */
#define SSP_IMSC_RORIM   _BIT(0)
#define SSP_RIS_RORRIS   _BIT(0)
#define SSP_MIS_RORMIS   _BIT(0)
#define SSP_ICR_RORIC    _BIT(0)
/* SSP interrupt bit for RX FIFO not empty and has a data timeout */
#define SSP_IMSC_RTIM    _BIT(1)
#define SSP_RIS_RTRIS    _BIT(1)
#define SSP_MIS_RTMIS    _BIT(1)
#define SSP_ICR_RTIC     _BIT(1)
/* SSP interrupt bit for RX FIFO half full */
#define SSP_IMSC_RXIM    _BIT(2)
#define SSP_RIS_RXRIS    _BIT(2)
#define SSP_MIS_RXMIS    _BIT(2)
/* SSP interrupt bit for TX FIFO half empty */
#define SSP_IMSC_TXIM    _BIT(3)
#define SSP_RIS_TXRIS    _BIT(3)
#define SSP_MIS_TXMIS    _BIT(3)

/***********************************************************************
 * SSP DMA enable register (dmacr) definitions
 **********************************************************************/
/* SSP bit for enabling RX DMA */
#define SSP_DMA_RXDMAEN  _BIT(0)
/* SSP bit for enabling TX DMA */
#define SSP_DMA_TXDMAEN  _BIT(1)

#define SSP_FIFO_DEPTH_WORDS 8

#endif /* LPC3250_SSP_H */
