/*
 * asm-arm/arch-lpc32xx/clock.h
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

#ifndef __ARCH_ARM_LPC32XX_CLOCK_H__
#define __ARCH_ARM_LPC32XX_CLOCK_H__

struct clk {
	struct list_head node;
	struct module *owner;
	const char *name;		/* Clock name */
	struct clk *parent;		/* Parent clock */
	u32 rate;			/* Rate in Hz for the clock (0 = disabled) */
	u32 flags;			/* Setup flags */
	s8 usecount;			/* Number of users of this clock */
	/* Required functions per clock */
	int (*set_rate) (struct clk *, u32);
	u32 (*round_rate) (struct clk *, u32);
	int (*enable) (struct clk *clk, int);
	/* Optional functions per clock */
	int (*set_parent) (struct clk * clk, struct clk * parent);
	u32 (*get_rate) (struct clk *clk);
	/* Register and mask for enablings/disabling the clock, only
	   used when the CLK_FLAG_ST_ENAB flag is set */
	u32 enable_reg;			/* Register to enable and disable associated clock */
	u32 enable_mask;		/* Or mask for enable, AND ~mask for disable */
};

/* Clock flags */
#define CLK_FLAG_FIXED		0x01 /* Fixed frequency clock */

/* Main clocks used as parents for other clocks */
extern struct clk osc_32KHz;
extern struct clk osc_main;
extern struct clk clk_armpll;
extern struct clk clk_usbpll;
extern struct clk clk_hclk;
extern struct clk clk_pclk;
int clk_register(struct clk *clk);
void clk_unregister(struct clk *clk);

#endif

