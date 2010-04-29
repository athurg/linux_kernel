/*
 * asm-arm/arch-lpc32xx/lpc32xx_clcd.h
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

#ifndef LPC32xx_CLCDC_H
#define LPC32xx_CLCDC_H

#define _BIT(n) (1 << (n))
#define _SBF(f,v) (((v)) << (f))

/**********************************************************************
* Color LCD controller register offsets
**********************************************************************/

#define CLCD_TIMH(x)			(x + 0x000)
#define CLCD_TIMV(x)			(x + 0x004)
#define CLCD_POL(x)			(x + 0x008)
#define CLCD_LE(x)			(x + 0x00C)
#define CLCD_UPBASE(x)			(x + 0x010)
#define CLCD_LPBASE(x)			(x + 0x014)
#define CLCD_CTRL(x)			(x + 0x018)
#define CLCD_INTMASK(x)			(x + 0x01C)
#define CLCD_INTRAW(x)			(x + 0x020)
#define CLCD_INTSTAT(x)			(x + 0x024)
#define CLCD_INTCLR(x)			(x + 0x02C)
#define CLCD_UPCURR(x)			(x + 0x030)
#define CLCD_PAL(x)			(x + 0x100)
#define CLCD_CRSR_IMG(x)		(x + 0x800)
#define CLCD_CRSR_CTRL(x)		(x + 0xC00)
#define CLCD_CRSR_CFG(x)		(x + 0xC04)
#define CLCD_CRSR_PAL0(x)		(x + 0xC08)
#define CLCD_CRSR_PAL1(x)		(x + 0xC0C)
#define CLCD_CRSR_XY(x)			(x + 0xC10)
#define CLCD_CRSR_CLIP(x)		(x + 0xC14)
#define CLCD_CRSR_INTMASK(x)		(x + 0xC20)
#define CLCD_CRSR_INTCLR(x)		(x + 0xC24)
#define CLCD_CRSR_INTRAW(x)		(x + 0xC28)
#define CLCD_CRSR_INTSTAT(x)		(x + 0xC2C)

/***********************************************************************
 * Color LCD controller timing 0 register definitions
 **********************************************************************/

/* LCD controller timing 0 pixel per line load macro */
#define CLCDC_LCDTIMING0_PPL(n) _SBF(2, (((n) / 16) - 1) & 0x0000003F)
/* LCD controller timing 0 HSYNC pulse width load macro */
#define CLCDC_LCDTIMING0_HSW(n) _SBF(8, ((n) - 1) & 0x000000FF)
/* LCD controller timing 0 horizontal front porch load macro */
#define CLCDC_LCDTIMING0_HFP(n)	_SBF(16, ((n) - 1) & 0x000000FF)
/* LCD controller timing 0 horizontal back porch load macro */
#define CLCDC_LCDTIMING0_HBP(n)	_SBF(24, ((n) - 1) & 0x000000FF)

/***********************************************************************
 * Color LCD controller timing 1 register definitions
 **********************************************************************/

/* LCD controller timing 1 lines per panel load macro */
#define CLCDC_LCDTIMING1_LPP(n)	_SBF(0, ((n) - 1) & 0x000003FF)
/* LCD controller timing 1 VSYNC pulse width load macro */
#define CLCDC_LCDTIMING1_VSW(n)	_SBF(10, ((n) - 1) & 0x0000003F)
/* LCD controller timing 1 vertical front porch load macro */
#define CLCDC_LCDTIMING1_VFP(n) _SBF(16, (n & 0x000000FF))
/* LCD controller timing 1 vertical back porch load macro */
#define CLCDC_LCDTIMING1_VBP(n) _SBF(24, (n & 0x000000FF))

/***********************************************************************
 * Color LCD controller timing 2 register definitions
 **********************************************************************/

/* LCD controller timing 2 panel clock divisor load macro */
#define CLCDC_LCDTIMING2_PCD(n)	 CLCDC_LCDTIMING2_PCD_hi(n) | CLCDC_LCDTIMING2_PCD_lo(n)
#define CLCDC_LCDTIMING2_PCD_hi(n)	 (_SBF(22,((n) - 2 )) & 0xf8000000)
#define CLCDC_LCDTIMING2_PCD_lo(n)	 (_SBF(0, ((n) - 2 )) & 0x0000001f)
/* LCD controller timing 2 pixel clock selector bit */
#define CLCDC_LCDTIMING2_CLKSEL 0x00000020
/* LCD controller timing 2 AC bias frequency load macro */
#define CLCDC_LCDTIMING2_ACB(n)	_SBF(6, ((n) - 1) & 0x0000001F)
/* LCD controller timing 2 VSYNC invert bit */
#define CLCDC_LCDTIMING2_IVS    0x00000800
/* LCD controller timing 2 HSYNC invert bit */
#define CLCDC_LCDTIMING2_IHS    0x00001000
/* LCD controller timing 2 clock invert bit */
#define CLCDC_LCDTIMING2_IPC    0x00002000
/* LCD controller timing 2 output enable invert bit */
#define CLCDC_LCDTIMING2_IOE    0x00004000
/* LCD controller timing 2 clocks per line load macro */
#define CLCDC_LCDTIMING2_CPL(n)	_SBF(16, (n) & 0x000003FF)
/* LCD controller timing 2 bypass pixel divider bit */
#define CLCDC_LCDTIMING2_BCD 	0x04000000

/***********************************************************************
 * Color LCD controller interrupt enable/status register definitions
 **********************************************************************/


/* LCDIntrEnable, LCDInterrupt, LCDStatus FIFO underflow */
#define CLCDC_LCDSTATUS_FUF   0x00000002
/* LCDIntrEnable, LCDInterrupt, LCDStatus load next base bit */
#define CLCDC_LCDSTATUS_LNBU 	0x00000004
/* LCDIntrEnable, LCDInterrupt, LCDStatus vertical compare bit */
#define CLCDC_LCDSTATUS_VCOMP 	0x00000008
/* LCDIntrEnable, LCDInterrupt, LCDStatus aHB bus error bit */
#define CLCDC_LCDSTATUS_MBERROR	0x00000010

/***********************************************************************
 * Color LCD controller control register definitions
 **********************************************************************/

/* LCD control enable bit */
#define CLCDC_LCDCTRL_ENABLE	0x00000001
/* LCD control 1 bit per pixel bit field */
#define CLCDC_LCDCTRL_BPP1      0x00000000
/* LCD control 2 bits per pixel bit field */
#define CLCDC_LCDCTRL_BPP2      0x00000002
/* LCD control 4 bits per pixel bit field */
#define CLCDC_LCDCTRL_BPP4      0x00000004
/* LCD control 8 bits per pixel bit field */
#define CLCDC_LCDCTRL_BPP8      0x00000006
/* LCD control 16 bits per pixel bit field */
#define CLCDC_LCDCTRL_BPP16     0x00000008
/* LCD control 24 bits per pixel bit field */
#define CLCDC_LCDCTRL_BPP24     0x0000000A
/* LCD control 16 bits per pixel bit field (565 mode) */
#define CLCDC_LCDCTRL_BPP16_565 0x0000000C
/* LCD control 12 bits per pixel bit field */
#define CLCDC_LCDCTRL_BPP12     0x0000000E
/* LCD control 16 bits per pixel bit field */
#define CLCDC_LCDCTRL_BPP_MASK  0x0000000E
/* LCD control mono select bit */
#define CLCDC_LCDCTRL_BW_MONO   0x00000010
/* LCD controler TFT select bit */
#define CLCDC_LCDCTRL_TFT       0x00000020
/* LCD control monochrome LCD has 4-bit/8-bit select bit */
#define CLCDC_LCDCTRL_MONO8     0x00000040
/* LCD control dual panel select bit */
#define CLCDC_LCDCTRL_DUAL      0x00000080
/* LCD control normal RGB bit */
#define CLCDC_LCDCTRL_RGB       0x00000100
/* LCD control swap RGB (555 BGR mode) bit */
#define CLCDC_LCDCTRL_BGR       0x00000000
/* LCD control power enable bit */
#define CLCDC_LCDCTRL_PWR       0x00000800
/* LCD control VCOMP interrupt is start of VSYNC */
#define CLCDC_LCDCTRL_VCOMP_VS  0x00000000
/* LCD control VCOMP interrupt is start of back porch */
#define CLCDC_LCDCTRL_VCOMP_BP  0x00001000
/* LCD control VCOMP interrupt is start of video */
#define CLCDC_LCDCTRL_VCOMP_AV  0x00002000
/* LCD control VCOMP interrupt is start of front porch */
#define CLCDC_LCDCTRL_VCOMP_FP  0x00003000
/* LCD control interrupt condition bits mask */
#define CLCDC_LCDCTRL_VCOMP_IC  0x00003000
/* LCD control watermark is 4 or 8 words free mask */
#define CLCDC_LCDCTRL_WATERMARK 0x00010000

#endif /* LPC32xx_CLCDC_H */
