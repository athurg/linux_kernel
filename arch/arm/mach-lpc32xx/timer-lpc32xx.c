/*
 *  linux/arch/arm/mach-lpc32xx/timer-lpc32xx.h
 *
 *  Copyright (C) 2008 NXP Semiconductors
 *  Copyright (C) 2009 Fontys University of Applied Sciences, Eindhoven
 *                     Ed Schouten <e.schouten@fontys.nl>
 *                     Laurens Timmermans <l.timmermans@fontys.nl>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/time.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/mach/time.h>
	
#include <mach/platform.h>
#include <mach/lpc32xx_timer.h>
#include <mach/lpc32xx_clkpwr.h>
#include <mach/lpc32xx_gpio.h>
#include <mach/clock.h>
#include "sys-lpc32xx.h"

#ifndef LEDTICK
#define LEDTICK
#endif

extern int clk_is_sysclk_mainosc(void);
extern u32 local_clk_get_pllrate_from_reg(u32 inputclk, u32 regval);
extern u32 clk_get_pclk_div(void);

static struct clock_event_device lpc32xx_clkevt;

static cycle_t lpc32xx_clksrc_read(void)
{
	return (cycle_t)__raw_readl(TIMER_TC(TIMER1_IOBASE));
}

static struct clocksource lpc32xx_clksrc = {
	.name	= "lpc32xx_clksrc",
	.shift	= 24,
	.rating	= 300,
	.read	= lpc32xx_clksrc_read,
	.mask	= CLOCKSOURCE_MASK(32),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int lpc32xx_clkevt_next_event(unsigned long delta,
    struct clock_event_device *dev)
{
	unsigned long flags;

	if (delta < 1)
		return -ETIME;

	local_irq_save(flags);

	__raw_writel(TIMER_CNTR_TCR_RESET, TIMER_TCR(TIMER0_IOBASE));
	__raw_writel(delta, TIMER_PR(TIMER0_IOBASE));
	__raw_writel(TIMER_CNTR_TCR_EN, TIMER_TCR(TIMER0_IOBASE));

	local_irq_restore(flags);

	return 0;
}

static void lpc32xx_clkevt_mode(enum clock_event_mode mode,
    struct clock_event_device *dev)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		WARN_ON(1);
		break;

	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/*
		 * Disable the timer. When using oneshot, we must also
		 * disable the timer to wait for the first call to
		 * set_next_event().
		 */
		__raw_writel(0, TIMER_TCR(TIMER0_IOBASE));
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static struct clock_event_device lpc32xx_clkevt = {
	.name		= "lpc32xx_clkevt",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.rating		= 300,
	.cpumask	= CPU_MASK_CPU0,
	.set_next_event	= lpc32xx_clkevt_next_event,
	.set_mode	= lpc32xx_clkevt_mode,
};

static irqreturn_t lpc32xx_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &lpc32xx_clkevt;

	/* Clear match */
	__raw_writel(TIMER_CNTR_MTCH_BIT(0), TIMER_IR(TIMER0_IOBASE));

	evt->event_handler(evt);

	/* Optional board specific LED function */
	LEDTICK

	return IRQ_HANDLED;
}

static struct irqaction lpc32xx_timer_irq = {
	.name		= "LPC32XX Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= lpc32xx_timer_interrupt,
};

static void __init lpc32xx_timer_init (void)
{
	u32 clkrate, pllreg;

	/* Enable timer clock */
	__raw_writel((CLKPWR_TMRPWMCLK_TIMER0_EN | CLKPWR_TMRPWMCLK_TIMER1_EN),
		CLKPWR_TIMERS_PWMS_CLK_CTRL_1(CLKPWR_IOBASE));

	/* The clock driver isn't initialized at this point. So determine if the SYSCLK
	   is driven from the PLL397 or main oscillator and then use it to compute
	   the PLL frequency and the PCLK divider to get the base timer 0 rate */
	if (clk_is_sysclk_mainosc() != 0)
		clkrate = MAIN_OSC_FREQ;
	else
		clkrate = 397 * CLOCK_OSC_FREQ;

	/* Get ARM HCLKPLL register and convert it into a frequency*/
	pllreg = __raw_readl(CLKPWR_HCLKPLL_CTRL(CLKPWR_IOBASE)) & 0x1FFFF;
	clkrate = local_clk_get_pllrate_from_reg(clkrate, pllreg);

	/* Get PCLK divider and divde ARM PLL clock with it to get timer rate */
	clkrate = clkrate / clk_get_pclk_div();

	/* Initial timer setup */
	__raw_writel(0, TIMER_TCR(TIMER0_IOBASE));
	__raw_writel(TIMER_CNTR_MTCH_BIT(0), TIMER_IR(TIMER0_IOBASE));
	__raw_writel(1, TIMER_MR0(TIMER0_IOBASE));
	__raw_writel(TIMER_CNTR_MCR_MTCH(0) | TIMER_CNTR_MCR_STOP(0) |
	    TIMER_CNTR_MCR_RESET(0), TIMER_MCR(TIMER0_IOBASE));

	/* Setup tick interrupt */
	setup_irq (IRQ_TIMER0, &lpc32xx_timer_irq);

	/* Setup the clockevent structure. */
	lpc32xx_clkevt.mult = div_sc(clkrate, NSEC_PER_SEC, lpc32xx_clkevt.shift);
	lpc32xx_clkevt.max_delta_ns = clockevent_delta2ns(-1, &lpc32xx_clkevt);
	lpc32xx_clkevt.min_delta_ns = clockevent_delta2ns(1, &lpc32xx_clkevt);
	lpc32xx_clkevt.cpumask = cpumask_of_cpu(0);
	clockevents_register_device(&lpc32xx_clkevt);

	/* Use timer1 as clock source. */
	__raw_writel(TIMER_CNTR_TCR_RESET, TIMER_TCR(TIMER1_IOBASE));
	__raw_writel(0, TIMER_PR(TIMER1_IOBASE));
	__raw_writel(0, TIMER_MCR(TIMER1_IOBASE));
	__raw_writel(TIMER_CNTR_TCR_EN, TIMER_TCR(TIMER1_IOBASE));
	lpc32xx_clksrc.mult = clocksource_hz2mult(clkrate, lpc32xx_clksrc.shift);
	clocksource_register(&lpc32xx_clksrc);
}

struct sys_timer lpc32xx_timer = {
	.init		= &lpc32xx_timer_init,
};

