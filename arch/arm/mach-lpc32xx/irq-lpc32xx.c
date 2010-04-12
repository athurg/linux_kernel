/*
 *  linux/arch/arm/mach-lpc32xx/irq-lpc32xx.c
 *
 *  Copyright (C) 2008 NXP Semiconductors
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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/system.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <mach/irqs.h>
#include <mach/platform.h>

static void get_controller(unsigned int irq, unsigned int *base, unsigned int *irqbit)
{
	if (irq < 32) {
		*base = io_p2v(MIC_BASE);
		*irqbit = 1 << irq;
	}
	else if (irq < 64) {
		*base = io_p2v(SIC1_BASE);
		*irqbit = 1 << (irq - 32);
	}
	else {
		*base = io_p2v(SIC2_BASE);
		*irqbit = 1 << (irq - 64);
	}
}

static void lpc32xx_mask_irq(unsigned int irq)
{
	unsigned int reg, ctrl, mask;

	get_controller(irq, &ctrl, &mask);

	reg = __raw_readl(ctrl + INTC_MASK);
	reg &= ~mask;
	__raw_writel(reg, (ctrl + INTC_MASK));
}

static void lpc32xx_unmask_irq(unsigned int irq)
{
	unsigned int reg, ctrl, mask;

	get_controller(irq, &ctrl, &mask);

	reg = __raw_readl(ctrl + INTC_MASK);
	reg |= mask;
	__raw_writel(reg, (ctrl + INTC_MASK));
}

static void lpc32xx_mask_ack_irq(unsigned int irq)
{
	unsigned int ctrl, mask;

	get_controller(irq, &ctrl, &mask);

	__raw_writel(mask, (ctrl + INTC_RAW_STAT));
}

static int lpc32xx_set_irq_type(unsigned int irq, unsigned int type)
{
	unsigned int reg, ctrl, mask;

	get_controller(irq, &ctrl, &mask);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		/* Rising edge sensitive */
		reg = __raw_readl(ctrl + INTC_POLAR);
		reg |= mask;
		__raw_writel(reg, (ctrl + INTC_POLAR));
		reg = __raw_readl(ctrl + INTC_ACT_TYPE);
		reg |= mask;
		__raw_writel(reg, (ctrl + INTC_ACT_TYPE));
		set_irq_handler(irq, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		/* Falling edge sensitive */
		reg = __raw_readl(ctrl + INTC_POLAR);
		reg &= ~mask;
		__raw_writel(reg, (ctrl + INTC_POLAR));
		reg = __raw_readl(ctrl + INTC_ACT_TYPE);
		reg |= mask;
		__raw_writel(reg, (ctrl + INTC_ACT_TYPE));
		set_irq_handler(irq, handle_edge_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		/* Low level sensitive */
		reg = __raw_readl(ctrl + INTC_POLAR);
		reg &= ~mask;
		__raw_writel(reg, (ctrl + INTC_POLAR));
		reg = __raw_readl(ctrl + INTC_ACT_TYPE);
		reg &= ~mask;
		__raw_writel(reg, (ctrl + INTC_ACT_TYPE));
		set_irq_handler(irq, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		/* High level sensitive */
		reg = __raw_readl(ctrl + INTC_POLAR);
		reg |= mask;
		__raw_writel(reg, (ctrl + INTC_POLAR));
		reg = __raw_readl(ctrl + INTC_ACT_TYPE);
		reg &= ~mask;
		__raw_writel(reg, (ctrl + INTC_ACT_TYPE));
		set_irq_handler(irq, handle_level_irq);
		break;

	/* IRQT_BOTHEDGE is not supported */
	default:
		printk(KERN_ERR "LPC32XX IRQ: Unsupported irq type %d\n", type);
		return -1;
	}
	return 0;
}

void __init lpc32xx_set_default_mappings(unsigned int base, unsigned int apr, unsigned int atr, unsigned int offset) {
	unsigned int i, lvl;

	/* Set activation levels for each interrupt */
	i = 0;
	while (i < 32)
	{
		lvl = ((apr >> i) & 0x1) | (((atr >> i) & 0x1) << 1);
		switch (lvl) {
			case 0x0: /* Low polarity and level operation */
				lpc32xx_set_irq_type((offset + i), IRQ_TYPE_LEVEL_LOW);
				break;

			case 0x1: /* High polarity and level operation */
				lpc32xx_set_irq_type((offset + i), IRQ_TYPE_LEVEL_HIGH);
				break;

			case 0x2: /* Low polarity and edge operation */
				lpc32xx_set_irq_type((offset + i), IRQ_TYPE_EDGE_FALLING);
				break;

			case 0x3: /* High polarity and edge operation */
				lpc32xx_set_irq_type((offset + i), IRQ_TYPE_EDGE_RISING);
				break;
		}

		i++;
	}
}

static struct irq_chip lpc32xx_irq_chip = {
	.ack = lpc32xx_mask_ack_irq,
	.mask = lpc32xx_mask_irq,
	.unmask = lpc32xx_unmask_irq,
	.set_type = lpc32xx_set_irq_type,
};

void __init lpc32xx_init_irq(void)
{
	unsigned int i, vloc;

	/* Setup MIC */
	vloc = io_p2v(MIC_BASE);
	__raw_writel(0, (vloc + INTC_MASK));
	__raw_writel(MIC_APR_DEFAULT, (vloc + INTC_POLAR));
	__raw_writel(MIC_ATR_DEFAULT, (vloc + INTC_ACT_TYPE));

	/* Setup SIC1 */
	vloc = io_p2v(SIC1_BASE);
	__raw_writel(0, (vloc + INTC_MASK));
	__raw_writel(SIC1_APR_DEFAULT, (vloc + INTC_POLAR));
	__raw_writel(SIC1_ATR_DEFAULT, (vloc + INTC_ACT_TYPE));

	/* Setup SIC2 */
	vloc = io_p2v(SIC2_BASE);
	__raw_writel(0, (vloc + INTC_MASK));
	__raw_writel(SIC2_APR_DEFAULT, (vloc + INTC_POLAR));
	__raw_writel(SIC2_ATR_DEFAULT, (vloc + INTC_ACT_TYPE));

	/* Configure supported IRQ's */
	for (i = 0; i < NR_IRQS; i++) {
		set_irq_flags(i, IRQF_VALID);
		set_irq_chip(i, &lpc32xx_irq_chip);
	}

	/* Set default mappings */
	lpc32xx_set_default_mappings(io_p2v(MIC_BASE), MIC_APR_DEFAULT, MIC_ATR_DEFAULT, 0);
	lpc32xx_set_default_mappings(io_p2v(SIC1_BASE), SIC1_APR_DEFAULT, SIC1_ATR_DEFAULT, INTC_SIC1_OFFS);
	lpc32xx_set_default_mappings(io_p2v(SIC2_BASE), SIC2_APR_DEFAULT, SIC2_ATR_DEFAULT, INTC_SIC2_OFFS);

	/* mask all interrupts except SUBIRQA and SUBFIQ */
	__raw_writel((1 << IRQ_SUB1IRQ) | (1 << IRQ_SUB2IRQ) |
			(1 << IRQ_SUB1FIQ) | (1 << IRQ_SUB2FIQ),
		(io_p2v(MIC_BASE) + INTC_MASK));
	__raw_writel(0, (io_p2v(SIC1_BASE) + INTC_MASK));
	__raw_writel(0, (io_p2v(SIC2_BASE) + INTC_MASK));
}

