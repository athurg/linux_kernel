/*
 * asm-arm/arch-lpc32xx/lpc32xx_usbd.h
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

#ifndef LPC32XX_USBD_H
#define LPC32XX_USBD_H

/**********************************************************************
* USE device controller register offsets
**********************************************************************/

#define USBD_DEVINTST(x)	(x + 0x200)
#define USBD_DEVINTEN(x)	(x + 0x204)
#define USBD_DEVINTCLR(x)	(x + 0x208)
#define USBD_DEVINTSET(x)	(x + 0x20C)
#define USBD_CMDCODE(x)		(x + 0x210)
#define USBD_CMDDATA(x)		(x + 0x214)
#define USBD_RXDATA(x)		(x + 0x218)
#define USBD_TXDATA(x)		(x + 0x21C)
#define USBD_RXPLEN(x)		(x + 0x220)
#define USBD_TXPLEN(x)		(x + 0x224)
#define USBD_CTRL(x)		(x + 0x228)
#define USBD_DEVINTPRI(x)	(x + 0x22C)
#define USBD_EPINTST(x)		(x + 0x230)
#define USBD_EPINTEN(x)		(x + 0x234)
#define USBD_EPINTCLR(x)	(x + 0x238)
#define USBD_EPINTSET(x)	(x + 0x23C)
#define USBD_EPINTPRI(x)	(x + 0x240)
#define USBD_REEP(x)		(x + 0x244)
#define USBD_EPIND(x)		(x + 0x248)
#define USBD_EPMAXPSIZE(x)	(x + 0x24C)
/* DMA support registers only below */
/* Set, clear, or get enabled state of the DMA request status. If
   enabled, an IN or OUT token will start a DMA transfer for the EP */
#define USBD_DMARST(x)		(x + 0x250)
#define USBD_DMARCLR(x)		(x + 0x254)
#define USBD_DMARSET(x)		(x + 0x258)
/* DMA UDCA head pointer */
#define USBD_UDCAH(x)		(x + 0x280)
/* EP DMA status, enable, and disable. This is used to specifically
   enabled or disable DMA for a specific EP */
#define USBD_EPDMAST(x)		(x + 0x284)
#define USBD_EPDMAEN(x)		(x + 0x288)
#define USBD_EPDMADIS(x)	(x + 0x28C)
/* DMA master interrupts enable and pending interrupts */
#define USBD_DMAINTST(x)	(x + 0x290)
#define USBD_DMAINTEN(x)	(x + 0x294)
/* DMA end of transfer interrupt enable, disable, status */
#define USBD_EOTINTST(x)	(x + 0x2A0)
#define USBD_EOTINTCLR(x)	(x + 0x2A4)
#define USBD_EOTINTSET(x)	(x + 0x2A8)
/* New DD request interrupt enable, disable, status */
#define USBD_NDDRTINTST(x)	(x + 0x2AC)
#define USBD_NDDRTINTCLR(x)	(x + 0x2B0)
#define USBD_NDDRTINTSET(x)	(x + 0x2B4)
/* DMA error interrupt enable, disable, status */
#define USBD_SYSERRTINTST(x)	(x + 0x2B8)
#define USBD_SYSERRTINTCLR(x)	(x + 0x2BC)
#define USBD_SYSERRTINTSET(x)	(x + 0x2C0)

/**********************************************************************
* USBD_DEVINTST/USBD_DEVINTEN/USBD_DEVINTCLR/USBD_DEVINTSET/
* USBD_DEVINTPRI register definitions
**********************************************************************/
#define USBD_ERR_INT               (1 << 9)
#define USBD_EP_RLZED              (1 << 8)
#define USBD_TXENDPKT              (1 << 7)
#define USBD_RXENDPKT              (1 << 6)
#define USBD_CDFULL                (1 << 5)
#define USBD_CCEMPTY               (1 << 4)
#define USBD_DEV_STAT              (1 << 3)
#define USBD_EP_SLOW               (1 << 2)
#define USBD_EP_FAST               (1 << 1)
#define USBD_FRAME                 (1 << 0)

/**********************************************************************
* USBD_EPINTST/USBD_EPINTEN/USBD_EPINTCLR/USBD_EPINTSET/
* USBD_EPINTPRI register definitions
**********************************************************************/
/* End point selection macro (RX) */
#define USBD_RX_EP_SEL(e)          (1 << ((e) << 1))

/* End point selection macro (TX) */
#define USBD_TX_EP_SEL(e)          (1 << (((e) << 1) + 1))

/**********************************************************************
* USBD_REEP/USBD_DMARST/USBD_DMARCLR/USBD_DMARSET/USBD_EPDMAST/
* USBD_EPDMAEN/USBD_EPDMADIS/
* USBD_NDDRTINTST/USBD_NDDRTINTCLR/USBD_NDDRTINTSET/
* USBD_EOTINTST/USBD_EOTINTCLR/USBD_EOTINTSET/
* USBD_SYSERRTINTST/USBD_SYSERRTINTCLR/USBD_SYSERRTINTSET
* register definitions
**********************************************************************/
/* Endpoint selection macro */
#define USBD_EP_SEL(e)              (1 << (e))

/**********************************************************************USBD_DMAINTST/USBD_DMAINTEN
**********************************************************************/
#define USBD_SYS_ERR_INT            (1 << 2)
#define USBD_NEW_DD_INT             (1 << 1)
#define USBD_EOT_INT                (1 << 0)

/**********************************************************************
* USBD_RXPLEN register definitions
**********************************************************************/
#define USBD_PKT_RDY               (1 << 11)
#define USBD_DV                    (1 << 10)
#define USBD_PK_LEN_MASK           0x3FF

/**********************************************************************
* USBD_CTRL register definitions
**********************************************************************/
#define USBD_LOG_ENDPOINT(e)      ((e) << 2)
#define USBD_WR_EN                (1 << 1)
#define USBD_RD_EN                (1 << 0)

/**********************************************************************
* USBD_CMDCODE register definitions
**********************************************************************/
#define USBD_CMD_CODE(c)          ((c) << 16)
#define USBD_CMD_PHASE(p)         ((p) << 8)

/**********************************************************************
* USBD_DMARST/USBD_DMARCLR/USBD_DMARSET register definitions
**********************************************************************/
#define USBD_DMAEP(e)             (1 << (e))

/* DD (DMA Descriptor) structure, requires word alignment */
struct lpc32xx_usbd_dd
{
	u32 *dd_next;
	u32 dd_setup;
	u32 dd_buffer_addr;
	u32 dd_status;
	u32 dd_iso_ps_mem_addr;
};

/* dd_setup bit defines */
#define DD_SETUP_ATLE_DMA_MODE 0x01
#define DD_SETUP_NEXT_DD_VALID 0x04
#define DD_SETUP_ISO_EP        0x10
#define DD_SETUP_PACKETLEN(n)  (((n) & 0x7FF) << 5)
#define DD_SETUP_DMALENBYTES(n)(((n) & 0xFFFF) << 16)

/* dd_status bit defines */
#define DD_STATUS_DD_RETIRED   0x01
#define DD_STATUS_STS_MASK     0x1E
#define DD_STATUS_STS_NS       0x00 /* Not serviced */
#define DD_STATUS_STS_BS       0x02 /* Being serviced */
#define DD_STATUS_STS_NC       0x04 /* Normal completion */
#define DD_STATUS_STS_DUR      0x06 /* Data underrun (short packet) */
#define DD_STATUS_STS_DOR      0x08 /* Data overrun */
#define DD_STATUS_STS_SE       0x12 /* System error */
#define DD_STATUS_PKT_VAL      0x20 /* Packet valid */
#define DD_STATUS_LSB_EX       0x40 /* LS byte extracted (ATLE) */
#define DD_STATUS_MSB_EX       0x80 /* MS byte extracted (ATLE) */
#define DD_STATUS_MLEN(n)      (((n) >> 8) & 0x3F)
#define DD_STATUS_CURDMACNT(n) (((n) >> 16) & 0xFFFF)

/*
 *
 * Protocol engine bits below
 *
 */
/* Device Interrupt Bit Definitions */
#define FRAME_INT           0x00000001
#define EP_FAST_INT         0x00000002
#define EP_SLOW_INT         0x00000004
#define DEV_STAT_INT        0x00000008
#define CCEMTY_INT          0x00000010
#define CDFULL_INT          0x00000020
#define RxENDPKT_INT        0x00000040
#define TxENDPKT_INT        0x00000080
#define EP_RLZED_INT        0x00000100
#define ERR_INT             0x00000200

/* Rx & Tx Packet Length Definitions */
#define PKT_LNGTH_MASK      0x000003FF
#define PKT_DV              0x00000400
#define PKT_RDY             0x00000800

/* USB Control Definitions */
#define CTRL_RD_EN          0x00000001
#define CTRL_WR_EN          0x00000002

/* Command Codes */
#define CMD_SET_ADDR        0x00D00500
#define CMD_CFG_DEV         0x00D80500
#define CMD_SET_MODE        0x00F30500
#define CMD_RD_FRAME        0x00F50500
#define DAT_RD_FRAME        0x00F50200
#define CMD_RD_TEST         0x00FD0500
#define DAT_RD_TEST         0x00FD0200
#define CMD_SET_DEV_STAT    0x00FE0500
#define CMD_GET_DEV_STAT    0x00FE0500
#define DAT_GET_DEV_STAT    0x00FE0200
#define CMD_GET_ERR_CODE    0x00FF0500
#define DAT_GET_ERR_CODE    0x00FF0200
#define CMD_RD_ERR_STAT     0x00FB0500
#define DAT_RD_ERR_STAT     0x00FB0200
#define DAT_WR_BYTE(x)     (0x00000100 | ((x) << 16))
#define CMD_SEL_EP(x)      (0x00000500 | ((x) << 16))
#define DAT_SEL_EP(x)      (0x00000200 | ((x) << 16))
#define CMD_SEL_EP_CLRI(x) (0x00400500 | ((x) << 16))
#define DAT_SEL_EP_CLRI(x) (0x00400200 | ((x) << 16))
#define CMD_SET_EP_STAT(x) (0x00400500 | ((x) << 16))
#define CMD_CLR_BUF         0x00F20500
#define DAT_CLR_BUF         0x00F20200
#define CMD_VALID_BUF       0x00FA0500

/* Device Address Register Definitions */
#define DEV_ADDR_MASK       0x7F
#define DEV_EN              0x80

/* Device Configure Register Definitions */
#define CONF_DVICE          0x01

/* Device Mode Register Definitions */
#define AP_CLK              0x01
#define INAK_CI             0x02
#define INAK_CO             0x04
#define INAK_II             0x08
#define INAK_IO             0x10
#define INAK_BI             0x20
#define INAK_BO             0x40

/* Device Status Register Definitions */
#define DEV_CON             0x01
#define DEV_CON_CH          0x02
#define DEV_SUS             0x04
#define DEV_SUS_CH          0x08
#define DEV_RST             0x10

/* Error Code Register Definitions */
#define ERR_EC_MASK         0x0F
#define ERR_EA              0x10

/* Error Status Register Definitions */
#define ERR_PID             0x01
#define ERR_UEPKT           0x02
#define ERR_DCRC            0x04
#define ERR_TIMOUT          0x08
#define ERR_EOP             0x10
#define ERR_B_OVRN          0x20
#define ERR_BTSTF           0x40
#define ERR_TGL             0x80

/* Endpoint Select Register Definitions */
#define EP_SEL_F            0x01
#define EP_SEL_ST           0x02
#define EP_SEL_STP          0x04
#define EP_SEL_PO           0x08
#define EP_SEL_EPN          0x10
#define EP_SEL_B_1_FULL     0x20
#define EP_SEL_B_2_FULL     0x40

/* Endpoint Status Register Definitions */
#define EP_STAT_ST          0x01
#define EP_STAT_DA          0x20
#define EP_STAT_RF_MO       0x40
#define EP_STAT_CND_ST      0x80

/* Clear Buffer Register Definitions */
#define CLR_BUF_PO          0x01

/* DMA Interrupt Bit Definitions */
#define EOT_INT             0x01
#define NDD_REQ_INT         0x02
#define SYS_ERR_INT         0x04


#endif /* LPC32XX_USBD_H */
