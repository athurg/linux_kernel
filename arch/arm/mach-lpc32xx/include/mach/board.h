/*
 * asm-arm/arch-lpc32xx/board.h
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


#ifndef BOARD_H
#define BOARD_H

#include "platform.h"
#include <linux/mtd/partitions.h>

/*
 * NAND platform configuration structure
 */
typedef int (*en_wp)(int);
struct lpc32XX_nand_cfg
{
	u32		wdr_clks;
	u32		wwidth;
	u32		whold;
	u32		wsetup;
	u32		rdr_clks;
	u32		rwidth;
	u32		rhold;
	u32		rsetup;
	int		use16bus; /* 0 = 8-bit, !0 = not support */
	en_wp		enable_write_prot;
	struct mtd_partition* (*partition_info)(int, int*);
};

/*
 * Key scanner platform configuration structure
 */
struct lpc32XX_kscan_cfg
{
	u32	matrix_sz;	/* Size of matrix in XxY, ie. 3 = 3x3 */
	int	*keymap;	/* Pointer to key map for the scan matrix */
	u32	deb_clks;	/* Debounce clocks (based on 32KHz clock) */
	u32	scan_delay;	/* Scan delay (based on 32KHz clock) */
};

/*
 * Network configuration structure
 */
typedef int (*get_mac_f)(u8 *mac);
struct lpc32xx_net_cfg
{
	get_mac_f get_mac_addr;	/* Function to get MAC address */
	int	phy_irq;	/* PHY IRQ number, or -1 for polling */
	u32	phy_mask;	/* PHY mask value */
};

/*
 * SPI (via SSP) configuration structure
 */
typedef int (*spi_cs_sel)(int, int);
typedef void (*spi_cs_setup)(int);
struct lpc32xx_spi_cfg
{
	int	num_cs;		/* Number of chip selects */
	spi_cs_setup spi_cs_setup; /* Initializes chip selects */
	spi_cs_sel spi_cs_set; /* Sets state of SPI chip select */
};

/*
 * USB device configuration structure
 */
typedef void (*usc_chg_event)(int);
struct lpc32xx_usbd_cfg
{
	int	vbus_drv_pol;	/* 0=active low drive for VBUS via ISP1301 */
	usc_chg_event conn_chgb; /* Connection change event callback (optional) */
	usc_chg_event susp_chgb; /* Suspend/resume event callback (optional) */
	usc_chg_event rmwk_chgb; /* Enable/disable remote wakeup */
};

#endif	/* BOARD_H */

