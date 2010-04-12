/*
 * asm-arm/arch-lpc32xx/lpc32xx_slcnand.h
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

#ifndef LPC32XX_SLCNAND_H
#define LPC32XX_SLCNAND_H

#define _BIT(n) (1 << (n))

/**********************************************************************
* SLC NAND controller register offsets
**********************************************************************/

#define SLC_DATA(x)			(x + 0x000)
#define SLC_ADDR(x)			(x + 0x004)
#define SLC_CMD(x)			(x + 0x008)
#define SLC_STOP(x)			(x + 0x00C)
#define SLC_CTRL(x)			(x + 0x010)
#define SLC_CFG(x)			(x + 0x014)
#define SLC_STAT(x)			(x + 0x018)
#define SLC_INT_STAT(x)			(x + 0x01C)
#define SLC_IEN(x)			(x + 0x020)
#define SLC_ISR(x)			(x + 0x024)
#define SLC_ICR(x)			(x + 0x028)
#define SLC_TAC(x)			(x + 0x02C)
#define SLC_TC(x)			(x + 0x030)
#define SLC_ECC(x)			(x + 0x034)
#define SLC_DMA_DATA(x)			(x + 0x038)

/**********************************************************************
* slc_ctrl register definitions
**********************************************************************/
#define SLCCTRL_SW_RESET    _BIT(2) /* Reset the NAND controller bit */
#define SLCCTRL_ECC_CLEAR   _BIT(1) /* Reset ECC bit */
#define SLCCTRL_DMA_START   _BIT(0) /* Start DMA channel bit */

/**********************************************************************
* slc_cfg register definitions
**********************************************************************/
#define SLCCFG_CE_LOW       _BIT(5) /* Force CE low bit */
#define SLCCFG_DMA_ECC      _BIT(4) /* Enable DMA ECC bit */
#define SLCCFG_ECC_EN       _BIT(3) /* ECC enable bit */
#define SLCCFG_DMA_BURST    _BIT(2) /* DMA burst bit */
#define SLCCFG_DMA_DIR      _BIT(1) /* DMA write(0)/read(1) bit */
#define SLCCFG_WIDTH        _BIT(0) /* External device width, 0=8bit */

/**********************************************************************
* slc_stat register definitions
**********************************************************************/
#define SLCSTAT_DMA_FIFO    _BIT(2) /* DMA FIFO has data bit */
#define SLCSTAT_SLC_FIFO    _BIT(1) /* SLC FIFO has data bit */
#define SLCSTAT_NAND_READY  _BIT(0) /* NAND device is ready bit */

/**********************************************************************
* slc_int_stat, slc_ien, slc_isr, and slc_icr register definitions
**********************************************************************/
#define SLCSTAT_INT_TC      _BIT(1) /* Transfer count bit */
#define SLCSTAT_INT_RDY_EN  _BIT(0) /* Ready interrupt bit */

/**********************************************************************
* slc_tac register definitions
**********************************************************************/
/* Clock setting for RDY write sample wait time in 2*n clocks */
#define SLCTAC_WDR(n)       (((n) & 0xF) << 28)
/* Write pulse width in clocks cycles, 1 to 16 clocks */
#define SLCTAC_WWIDTH(n)    (((n) & 0xF) << 24)
/* Write hold time of control and data signals, 1 to 16 clocks */
#define SLCTAC_WHOLD(n)     (((n) & 0xF) << 20)
/* Write setup time of control and data signals, 1 to 16 clocks */
#define SLCTAC_WSETUP(n)    (((n) & 0xF) << 16)
/* Clock setting for RDY read sample wait time in 2*n clocks */
#define SLCTAC_RDR(n)       (((n) & 0xF) << 12)
/* Read pulse width in clocks cycles, 1 to 16 clocks */
#define SLCTAC_RWIDTH(n)    (((n) & 0xF) << 8)
/* Read hold time of control and data signals, 1 to 16 clocks */
#define SLCTAC_RHOLD(n)     (((n) & 0xF) << 4)
/* Read setup time of control and data signals, 1 to 16 clocks */
#define SLCTAC_RSETUP(n)    (((n) & 0xF) << 0)

/**********************************************************************
* slc_ecc register definitions
**********************************************************************/
/* ECC line party fetch macro */
#define SLCECC_TO_LINEPAR(n) (((n) >> 6) & 0x7FFF)
#define SLCECC_TO_COLPAR(n)  ((n) & 0x3F)

#endif /* LPC32XX_SLCNAND_H */
