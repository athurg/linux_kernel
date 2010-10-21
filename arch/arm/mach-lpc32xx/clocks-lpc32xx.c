/*
 *  linux/arch/arm/mach-lpc32xx/clock.c
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
/*
 * Functions of this driver:
 * This driver provides system clocking and control mechanisms for most
 * peripheral clocks and partial control over systems clocks.
 * The clock organization of the system is as follows:
 *     RTC clock
 *        |
 *      PLL397                          Main oscillator
 *        |                                    |   |
 *        +------------------------------------+   |
 *                          |                      |
 *                        SYSCLK                USB PLL
 *                          |
 *                      Arm PLL (HCLKPLL)
 *                          |
 *        +-----------------+------------------+
 *        |                 |                  |
 *   HCLK (divider)   PCLK (divider)       SD card clock
 *                          |
 *                  Peripherals (many)
 *
 * The PLL397 and main oscillator can be individually enabled and
 * disabled using clk_enable(), clk_disable(), or clk_rate_set().
 * The clk_rate_get() function will return the actual frequency of
 * those oscillators or 0 Hz if they are disabled.
 *
 * The USB PLL clock can be enabled and disabled using the clk_enable(),
 * clk_disable(), clk_set_rate(), and clk_round_rate() functions. The
 * clk_rate_get() function will return the actual rate of the USB PLL
 * in Hz (or 0 if disabled).
 *
 * The ARM PLL clock can be enabled and disabled using the clk_enable(),
 * clk_disable(), clk_set_rate(), and clk_round_rate() functions. The
 * clk_rate_get() function will return the actual rate of the USB PLL
 * in Hz (or 0 if disabled).
 */

#define CLKDEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/platform.h>
#include <linux/jiffies.h>
#include <asm/io.h>

#include <mach/lpc32xx_clkpwr.h>
#include <mach/lpc32xx_clcdc.h>
#include <mach/clock.h>

static LIST_HEAD(clocks);

static struct clk osc_pll397;

static void clk_upate_children(struct clk *clk);
static void local_update_armpll_rate(void);
static void local_update_usbpll_rate(void);

/*
 * Structure used for setting up and querying the HCLK PLL
 */
typedef struct
{
  /* (0) = analog off, (!0) = on */
  int analog_on;
  /* (0) = CCO clock sent to post divider, (!0) = PLL input clock sent
  to post div */
  int cco_bypass_b15;
  /* (0) = PLL out from post divider, (!0) = PLL out bypasses post
  divider */
  int direct_output_b14;
  /* (0) = use CCO clock, (!0) = use FCLKOUT */
  int fdbk_div_ctrl_b13;
  /* Must be 1, 2, 4, or 8 */
  int pll_p;
  /* Must be 1, 2, 3, or 4 */
  int pll_n;
  /* Feedback multiplier 1-256 */
  u32 pll_m;
} CLKPWR_HCLK_PLL_SETUP_T;

/*
 * Post divider values for PLLs based on selected register value
 */
static u32 pll_postdivs[4] = {1, 2, 4, 8};

#define CLKPWR_IOBASE io_p2v(CLK_PM_BASE)
#define USB_OTG_IOBASE io_p2v(USB_BASE)

static int local_clk_dummy_set_rate(struct clk *clk, u32 rate)
{
	return 0;
}

static u32 local_clk_get_st_rate(struct clk *clk)
{
	return clk->rate;
}

static int local_clk_dummy_enable(struct clk *clk, int enable)
{
	return 0;
}

/* Primary system clock sources */
struct clk osc_32KHz =
{
	.owner		= NULL,
	.name		= "osc_32KHz",
	.parent		= NULL,
	.rate		= CLOCK_OSC_FREQ,
	.flags		= CLK_FLAG_FIXED,
	.usecount	= 0,
	.set_rate	= &local_clk_dummy_set_rate,
	.set_parent	= NULL,
	.round_rate	= NULL,
	.get_rate	= &local_clk_get_st_rate,
	.enable		= &local_clk_dummy_enable,
	.enable_reg	= 0,
	.enable_mask	= 0,
};

static void clk_upate_children(struct clk *clk)
{
	local_update_armpll_rate();
	local_update_usbpll_rate();
}

static int local_pll397_set_rate(struct clk *clk, u32 rate)
{
	u32 reg;

	reg = __raw_readl(CLKPWR_PLL397_CTRL(CLKPWR_IOBASE));

	/* Disable if rate is 0 */
	if (rate == 0)
	{
		/* 1 = disable for PLL397 */
		reg |= CLKPWR_SYSCTRL_PLL397_DIS;
		__raw_writel(reg, CLKPWR_PLL397_CTRL(CLKPWR_IOBASE));
		clk->rate = 0;
	}
	else
	{
		/* Enable PLL397 */
		reg &= ~CLKPWR_SYSCTRL_PLL397_DIS;
		__raw_writel(reg, CLKPWR_PLL397_CTRL(CLKPWR_IOBASE));
		clk->rate = CLOCK_OSC_FREQ * 397;

		/* Wait for PLL397 lock */
		while ((__raw_readl(CLKPWR_PLL397_CTRL(CLKPWR_IOBASE)) &
			CLKPWR_SYSCTRL_PLL397_STS) == 0);
	}

	/* Update clocks derived from this clock */
	clk_upate_children(clk);

	return 0;
}

static int local_pll397_enable(struct clk *clk, int enable)
{
	return (int) local_pll397_set_rate(clk, (u32) enable);
}


static int local_oscmain_set_rate(struct clk *clk, u32 rate)
{
	u32 reg;

	reg = __raw_readl(CLKPWR_MAIN_OSC_CTRL(CLKPWR_IOBASE));

	/* Disable if rate is 0 */
	if (rate == 0)
	{
		/* 1 = disable for main oscillator */
		reg |= CLKPWR_MOSC_DISABLE;
		__raw_writel(reg, CLKPWR_MAIN_OSC_CTRL(CLKPWR_IOBASE));
		clk->rate = 0;
	}
	else
	{
		/* Enable main oscillator */
		reg &= ~CLKPWR_MOSC_DISABLE;
		__raw_writel(reg, CLKPWR_MAIN_OSC_CTRL(CLKPWR_IOBASE));
		clk->rate = MAIN_OSC_FREQ;

		/* Wait for main oscillator to start */
		while ((__raw_readl(CLKPWR_MAIN_OSC_CTRL(CLKPWR_IOBASE)) &
			CLKPWR_MOSC_DISABLE) != 0);
	}

	/* Update clocks derived from this clock */
	clk_upate_children(clk);

	return 0;
}

static int local_oscmain_enable(struct clk *clk, int enable)
{
	return (int) local_oscmain_set_rate(clk, (u32) enable);
}

static int local_sysclk_set_rate(struct clk *clk, u32 rate)
{
	u32 reg, ret = 0;

	reg = __raw_readl(CLKPWR_SYSCLK_CTRL(CLKPWR_IOBASE));

	if (rate == 0)
	{
		/* 0 = Switch to main oscillator */
		reg &= ~CLKPWR_SYSCTRL_USEPLL397;
		__raw_writel(reg, CLKPWR_SYSCLK_CTRL(CLKPWR_IOBASE));
		clk->parent = &osc_main;
	}
	else
	{
		/* 1 = Switch to PLL397 opscillator */
		reg |= CLKPWR_SYSCTRL_USEPLL397;
		__raw_writel(reg, CLKPWR_SYSCLK_CTRL(CLKPWR_IOBASE));
		clk->parent = &osc_pll397;
	}

	clk->rate = clk->parent->rate;

	/* Update clocks derived from this clock */
	clk_upate_children(clk);

	return ret;
}

static int local_sysclk_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, ret = 0;

	reg = __raw_readl(CLKPWR_SYSCLK_CTRL(CLKPWR_IOBASE));

	if (parent == &osc_main)
	{
		local_sysclk_set_rate(parent, 0);
	}
	else if (parent == &osc_pll397)
	{
		local_sysclk_set_rate(parent, 1);
	}
	else
	{
		ret = -EINVAL;
	}

	return ret;
}

static struct clk osc_pll397 =
{
	.owner		= NULL,
	.name		= "osc_pll397",
	.parent		= &osc_32KHz,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_pll397_set_rate,
	.set_parent	= NULL,
	.round_rate	= NULL,
	.get_rate	= NULL,
	.enable		= &local_pll397_enable,
	.enable_reg	= 0,
	.enable_mask	= 0,
};
struct clk osc_main =
{
	.owner		= NULL,
	.name		= "osc_main",
	.parent		= NULL,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_oscmain_set_rate,
	.set_parent	= NULL,
	.round_rate	= NULL,
	.get_rate	= NULL,
	.enable		= &local_oscmain_enable,
	.enable_reg	= 0,
	.enable_mask	= 0,
};
static struct clk clk_sys =
{
	.owner		= NULL,
	.name		= "sys_ck",
	.parent		= &osc_main,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_sysclk_set_rate, /* 0 = main osc, 1 = PLL397 */
	.set_parent	= local_sysclk_set_parent, /* Can set osc_main or osc_pll397 as parent */
	.round_rate	= NULL,
	.get_rate	= NULL,
	.enable		= &local_clk_dummy_enable,
	.enable_reg	= 0,
	.enable_mask	= 0,
};

/*
 * Computes PLL rate from PLL register and input clock
 */
static u32 local_clk_check_pll_setup(u32 ifreq, CLKPWR_HCLK_PLL_SETUP_T *pllsetup)
{
	u32 i64freq, p, m, n, fcco, fref, cfreq;
	int mode;

	/* PLL requirements */
	/* ifreq must be >= 1MHz and <= 20MHz */
	/* FCCO must be >= 156MHz and <= 320MHz */
	/* FREF must be >= 1MHz and <= 27MHz. */
	/* Assume the passed input data is not valid */

	/* Work with 64-bit values to prevent overflow */
	i64freq = ifreq;
	m = pllsetup->pll_m;
	n = pllsetup->pll_n;
	p = pllsetup->pll_p;

	/* Get components of the PLL register */
	mode = (pllsetup->cco_bypass_b15 << 2) |
		(pllsetup->direct_output_b14 << 1) |
	pllsetup->fdbk_div_ctrl_b13;

	switch (mode)
	{
		case 0x0: /* Non-integer mode */
			cfreq = (m * i64freq) / (2 * p * n);
			fcco = (m * i64freq) / n;
			fref = i64freq / n;
			break;

		case 0x1: /* integer mode */
			cfreq = (m * i64freq) / n;
			fcco = (m * i64freq) / (n * 2 * p);
			fref = i64freq / n;
			break;

		case 0x2:
		case 0x3: /* Direct mode */
			cfreq = (m * i64freq) / n;
			fcco = cfreq;
			fref = i64freq / n;
			break;

		case 0x4:
		case 0x5: /* Bypass mode */
			cfreq = i64freq / (2 * p);
			fcco = 156000000;
			fref = 1000000;
			break;

		case 0x6:
		case 0x7: /* Direct bypass mode */
		default:
			cfreq = i64freq;
			fcco = 156000000;
			fref = 1000000;
			break;
	}

	if ((fcco < 156000000) || (fcco > 320000000))
	{
		/* not a valid range */
		cfreq = 0;
	}

	if ((fref < 1000000) || (fref > 27000000))
	{
		/* not a valid range */
		cfreq = 0;
	}

	return (u32) cfreq;
}

/*
 * Convert a PLL register value to a PLL output frequency
 */
u32 local_clk_get_pllrate_from_reg(u32 inputclk, u32 regval)
{
	CLKPWR_HCLK_PLL_SETUP_T pllcfg;

	/* Get components of the PLL register */
	pllcfg.cco_bypass_b15 = 0;
	pllcfg.direct_output_b14 = 0;
	pllcfg.fdbk_div_ctrl_b13 = 0;
	if ((regval & CLKPWR_HCLKPLL_CCO_BYPASS) != 0)
	{
		pllcfg.cco_bypass_b15 = 1;
	}
	if ((regval & CLKPWR_HCLKPLL_POSTDIV_BYPASS) != 0)
	{
		pllcfg.direct_output_b14 = 1;
	}
	if ((regval & CLKPWR_HCLKPLL_FDBK_SEL_FCLK) != 0)
	{
		pllcfg.fdbk_div_ctrl_b13 = 1;
	}
	pllcfg.pll_m = 1 + ((regval >> 1) & 0xFF);
	pllcfg.pll_n = 1 + ((regval >> 9) & 0x3);
	pllcfg.pll_p = pll_postdivs[((regval >> 11) & 0x3)];

	return local_clk_check_pll_setup(inputclk, &pllcfg);
}

/*
 * Update the ARM core PLL frequency rate variable from the actual PLL setting
 */
static void local_update_armpll_rate(void)
{
	u32 clkin, pllreg;

	/* Get PLL input clock rate */
	clkin = clk_armpll.parent->rate;

	/* Get ARM HCLKPLL register */
	pllreg = __raw_readl(CLKPWR_HCLKPLL_CTRL(CLKPWR_IOBASE)) & 0x1FFFF;

	clk_armpll.rate = local_clk_get_pllrate_from_reg(clkin, pllreg);
}

static int clkpwr_abs(int v1, int v2)
{
  if (v1 > v2)
  {
    return v1 - v2;
  }

  return v2 - v1;
}

/*
 * Find a PLL configuration for the selected input frequency
 */
static u32 local_clk_find_pll_cfg(u32 pllin_freq,
                           u32 target_freq,
                           CLKPWR_HCLK_PLL_SETUP_T *pllsetup)
{
  u32 ifreq, freqtol, m, n, p, fclkout = 0;
  u32 flag = 0, freqret = 0;

  /* Determine frequency tolerance limits */
  freqtol = target_freq / 250;

  /* Get PLL clock */
  ifreq = pllin_freq;

  /* Is direct bypass mode possible? */
  if (clkpwr_abs(pllin_freq, target_freq) <= freqtol)
  {
    flag = 1;
    pllsetup->analog_on = 0;
    pllsetup->cco_bypass_b15 = 1; /* Bypass CCO */
    pllsetup->direct_output_b14 = 1; /* Bypass post divider */
    pllsetup->fdbk_div_ctrl_b13 = 1;
    pllsetup->pll_p = pll_postdivs[0]; /* Doesn't matter */
    pllsetup->pll_n = 1; /* Doesn't matter */
    pllsetup->pll_m = 1; /* Doesn't matter */
    fclkout = local_clk_check_pll_setup(ifreq, pllsetup);
  }
  else if (target_freq <= ifreq) /* Is bypass mode possible? */
  {
    pllsetup->analog_on = 0;
    pllsetup->cco_bypass_b15 = 1; /* Bypass CCO */
    pllsetup->direct_output_b14 = 0;
    pllsetup->fdbk_div_ctrl_b13 = 1;
    pllsetup->pll_n = 1; /* Doesn't matter */
    pllsetup->pll_m = 1; /* Doesn't matter */
    for (p = 0; ((p <= 3) && (flag == 0)); p++)
    {
      pllsetup->pll_p = pll_postdivs[p];
      fclkout = local_clk_check_pll_setup(ifreq, pllsetup);
      if (clkpwr_abs(target_freq, fclkout) <= freqtol)
      {
        /* Found a matching frequency */
        flag = 1;
      }
    }
  }

  /* Is direct mode possible? */
  if (flag == 0)
  {
    pllsetup->analog_on = 1;
    pllsetup->cco_bypass_b15 = 0; /* Bypass CCO */
    pllsetup->direct_output_b14 = 1;
    pllsetup->fdbk_div_ctrl_b13 = 0;
    pllsetup->pll_p = pll_postdivs[0];
    for (m = 1; ((m <= 256) && (flag == 0)); m++)
    {
      for (n = 1; ((n <= 4) && (flag == 0)); n++)
      {
        /* Compute output frequency for this value */
        pllsetup->pll_n = n;
        pllsetup->pll_m = m;
        fclkout = local_clk_check_pll_setup(ifreq, pllsetup);
        if (clkpwr_abs(target_freq, fclkout) <= freqtol)
        {
          flag = 1;
        }
      }
    }
  }

  /* Is integer mode possible? */
  if (flag == 0)
  {
    /* Bypass and direct modes won't work with this frequency, so
       integer mode may need to be used */
    pllsetup->analog_on = 1;
    pllsetup->cco_bypass_b15 = 0;
    pllsetup->direct_output_b14 = 0;
    pllsetup->fdbk_div_ctrl_b13 = 1;
    for (m = 1; ((m <= 256) && (flag == 0)); m++)
    {
      for (n = 1; ((n <= 4) && (flag == 0)); n++)
      {
        for (p = 0; ((p < 4) && (flag == 0)); p++)
        {
          /* Compute output frequency for this value */
          pllsetup->pll_p = pll_postdivs[p];
          pllsetup->pll_n = n;
          pllsetup->pll_m = m;
          fclkout = local_clk_check_pll_setup(ifreq, pllsetup);
          if (clkpwr_abs(target_freq, fclkout) <= freqtol)
          {
            flag = 1;
          }
        }
      }
    }
  }

  if (flag == 0)
  {
    /* Try non-integer mode */
    pllsetup->analog_on = 1;
    pllsetup->cco_bypass_b15 = 0;
    pllsetup->direct_output_b14 = 0;
    pllsetup->fdbk_div_ctrl_b13 = 0;
    for (m = 1; ((m <= 256) && (flag == 0)); m++)
    {
      for (n = 1; ((n <= 4) && (flag == 0)); n++)
      {
        for (p = 0; ((p < 4) && (flag == 0)); p++)
        {
          /* Compute output frequency for this value */
          pllsetup->pll_p = pll_postdivs[p];
          pllsetup->pll_n = n;
          pllsetup->pll_m = m;
          fclkout = local_clk_check_pll_setup(ifreq, pllsetup);
          if (clkpwr_abs(target_freq, fclkout) <= freqtol)
          {
            flag = 1;
          }
        }
      }
    }
  }

  if (flag == 1)
  {
    freqret = fclkout;
  }

  return freqret;
}

/*
 * Setup the HCLK PLL with a PLL structure
 */
static u32 local_clk_hclkpll_setup(CLKPWR_HCLK_PLL_SETUP_T *pHCLKPllSetup)
{
  u32 tv, tmp = 0;

  if (pHCLKPllSetup->analog_on != 0)
  {
    tmp |= CLKPWR_HCLKPLL_POWER_UP;
  }
  if (pHCLKPllSetup->cco_bypass_b15 != 0)
  {
    tmp |= CLKPWR_HCLKPLL_CCO_BYPASS;
  }
  if (pHCLKPllSetup->direct_output_b14 != 0)
  {
    tmp |= CLKPWR_HCLKPLL_POSTDIV_BYPASS;
  }
  if (pHCLKPllSetup->fdbk_div_ctrl_b13 != 0)
  {
    tmp |= CLKPWR_HCLKPLL_FDBK_SEL_FCLK;
  }

  switch (pHCLKPllSetup->pll_p)
  {
    case 1:
      tv = 0;
      break;

    case 2:
      tv = 1;
      break;

    case 4:
      tv = 2;
      break;

    case 8:
      tv = 3;
      break;

    default:
      return 0;
  }
  tmp |= CLKPWR_HCLKPLL_POSTDIV_2POW(tv);
  tmp |= CLKPWR_HCLKPLL_PREDIV_PLUS1(pHCLKPllSetup->pll_n - 1);
  tmp |= CLKPWR_HCLKPLL_PLLM(pHCLKPllSetup->pll_m - 1);

	__raw_writel(tmp, CLKPWR_HCLKPLL_CTRL(CLKPWR_IOBASE));

  return local_clk_check_pll_setup(clk_armpll.parent->rate, pHCLKPllSetup);
}

static int local_armpll_set_rate(struct clk *clk, u32 rate)
{
	u32 clkin;
	CLKPWR_HCLK_PLL_SETUP_T pllsetup;

	if ((rate < 33000000) || (rate > 266000000))
	{
		return -EINVAL;
	}

	/* Get input frequency to PLL */
	clkin = clk->parent->rate;

	/* Try to find a good rate setup */
	if (local_clk_find_pll_cfg(clkin, rate, &pllsetup) == 0)
	{
		/* Cannot setup the frequency */
		return -EINVAL;
	}

	/* Setup PLL */
	local_clk_hclkpll_setup(&pllsetup);
	clk->rate = local_clk_check_pll_setup(clkin, &pllsetup);

	return 0;
}

static int local_armpll_enable(struct clk *clk, int enable)
{
	u32 reg;

	/* Read PLL reg */
	reg = __raw_readl(CLKPWR_HCLKPLL_CTRL(CLKPWR_IOBASE));

	if (enable == 0)
	{
		/* Disable PLL - not recommended unless you really know
		   what your are doing (must be in direct-run mode!) */
		reg &= ~CLKPWR_HCLKPLL_POWER_UP;
		__raw_writel(reg, CLKPWR_HCLKPLL_CTRL(CLKPWR_IOBASE));
		clk->rate = 0;
	}
	else
	{
		reg |= CLKPWR_HCLKPLL_POWER_UP;
		__raw_writel(reg, CLKPWR_HCLKPLL_CTRL(CLKPWR_IOBASE));

		/* Wait for PLL lock */
		while ((__raw_readl(CLKPWR_HCLKPLL_CTRL(CLKPWR_IOBASE)) &
			CLKPWR_HCLKPLL_PLL_STS) == 0);

		local_update_armpll_rate();
	}

	return 0;
}

/*
 * Update the USB PLL frequency rate variable from the actual PLL setting
 */
static void local_update_usbpll_rate(void)
{
	u32 clkin, pllreg;

	/* Get PLL input clock rate */
	clkin = clk_armpll.parent->rate;

	/* Get USBPLL register */
	pllreg = __raw_readl(CLKPWR_USB_CTRL(CLKPWR_IOBASE)) & 0x1FFFF;

	if ((pllreg & CLKPWR_USBCTRL_PLL_PWRUP) == 0)
	{
		clk_usbpll.rate = 0;
	}
	else
	{
		clk_usbpll.rate = local_clk_get_pllrate_from_reg(clkin, pllreg);
	}
}

/*
 * Setup the USB PLL with a PLL structure
 */
static u32 local_clk_usbpll_setup(CLKPWR_HCLK_PLL_SETUP_T *pHCLKPllSetup)
{
  u32 tv, reg, tmp = 0;

  if (pHCLKPllSetup->analog_on != 0)
  {
    tmp |= CLKPWR_HCLKPLL_POWER_UP;
  }
  if (pHCLKPllSetup->cco_bypass_b15 != 0)
  {
    tmp |= CLKPWR_HCLKPLL_CCO_BYPASS;
  }
  if (pHCLKPllSetup->direct_output_b14 != 0)
  {
    tmp |= CLKPWR_HCLKPLL_POSTDIV_BYPASS;
  }
  if (pHCLKPllSetup->fdbk_div_ctrl_b13 != 0)
  {
    tmp |= CLKPWR_HCLKPLL_FDBK_SEL_FCLK;
  }

  switch (pHCLKPllSetup->pll_p)
  {
    case 1:
      tv = 0;
      break;

    case 2:
      tv = 1;
      break;

    case 4:
      tv = 2;
      break;

    case 8:
      tv = 3;
      break;

    default:
      return 0;
  }
  tmp |= CLKPWR_HCLKPLL_POSTDIV_2POW(tv);
  tmp |= CLKPWR_HCLKPLL_PREDIV_PLUS1(pHCLKPllSetup->pll_n - 1);
  tmp |= CLKPWR_HCLKPLL_PLLM(pHCLKPllSetup->pll_m - 1);

	reg = __raw_readl(CLKPWR_USB_CTRL(CLKPWR_IOBASE)) & ~0x1FFFF;
	reg |= tmp;
	__raw_writel(reg, CLKPWR_USB_CTRL(CLKPWR_IOBASE));

  return local_clk_check_pll_setup(clk_usbpll.parent->rate, pHCLKPllSetup);
}

static int local_usbpll_enable(struct clk *clk, int enable)
{
	u32 reg;
	int ret = -ENODEV, qj = (jiffies / 4);

	reg = __raw_readl(CLKPWR_USB_CTRL(CLKPWR_IOBASE));

	/* Only disable is supported */
	if (enable == 0)
	{
		/* Stop PLL clocking */
		reg &= ~(CLKPWR_USBCTRL_CLK_EN1 | CLKPWR_USBCTRL_CLK_EN2);
		__raw_writel(reg, CLKPWR_USB_CTRL(CLKPWR_IOBASE));
	}
	else if (reg & CLKPWR_USBCTRL_PLL_PWRUP)
	{
		/* Start PLL clock input */
		reg |= CLKPWR_USBCTRL_CLK_EN1;
		__raw_writel(reg, CLKPWR_USB_CTRL(CLKPWR_IOBASE));

		/* Wait for PLL lock */
		while (qj < jiffies)
		{
			reg = __raw_readl(CLKPWR_USB_CTRL(CLKPWR_IOBASE));
			if (reg & CLKPWR_USBCTRL_PLL_STS)
			{
				ret = 0;
			}
		}

		if (ret == 0)
		{
			/* Allow PLL output clock */
			reg |= CLKPWR_USBCTRL_CLK_EN2;
			__raw_writel(reg, CLKPWR_USB_CTRL(CLKPWR_IOBASE));
		}
	}

	return ret;
}

static int local_usbpll_set_rate(struct clk *clk, u32 rate)
{
	u32 clkin, reg, usbdiv;
	CLKPWR_HCLK_PLL_SETUP_T pllsetup;

	/* Unlike other clocks, this clock has a KHz input rate, so bump
	   it up to work with the PLL function */
	rate = rate * 1000;

	/* Stop clocks */
	local_usbpll_enable(clk, 0);

	if (rate == 0)
	{
		return 0;
	}

	/* Get input frequency to PLL */
	clkin = clk->parent->rate;
	usbdiv = __raw_readl(CLKPWR_USBCLK_PDIV(CLKPWR_IOBASE)) + 1;
	clkin = clkin / usbdiv;

	/* Try to find a good rate setup */
	if (local_clk_find_pll_cfg(clkin, rate, &pllsetup) == 0)
	{
		/* Cannot setup the frequency */
		return -EINVAL;
	}

	/* Start PLL clock input */
	reg = __raw_readl(CLKPWR_USB_CTRL(CLKPWR_IOBASE));
	reg |= CLKPWR_USBCTRL_CLK_EN1;
	__raw_writel(reg, CLKPWR_USB_CTRL(CLKPWR_IOBASE));

	/* Setup PLL */
	pllsetup.analog_on = 1;
	local_clk_usbpll_setup(&pllsetup);

	clk->rate = local_clk_check_pll_setup(clkin, &pllsetup);

	/* Enable PLL output */
	reg = __raw_readl(CLKPWR_USB_CTRL(CLKPWR_IOBASE));
	reg |= CLKPWR_USBCTRL_CLK_EN2;
	__raw_writel(reg, CLKPWR_USB_CTRL(CLKPWR_IOBASE));

	return 0;
}

/* Derived system clock sources */
struct clk clk_armpll =
{
	.owner		= NULL,
	.name		= "arm_pll_ck",
	.parent		= &clk_sys,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_armpll_set_rate,
	.set_parent	= NULL,
	.round_rate	= NULL,
	.get_rate	= &local_clk_get_st_rate,
	.enable		= &local_armpll_enable,
	.enable_reg	= 0,
	.enable_mask	= 0,
};
struct clk clk_usbpll =
{
	.owner		= NULL,
	.name		= "ck_pll5", /* USB PLL clock */
	.parent		= &osc_main,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_usbpll_set_rate,
	.set_parent	= NULL,
	.round_rate	= NULL,
	.get_rate	= &local_clk_get_st_rate,
	.enable		= &local_usbpll_enable,
	.enable_reg	= 0,
	.enable_mask	= 0,
};

static u32 clk_get_hclk_div(void)
{
	static const u32 hclkdivs[4] = {1, 2, 4, 4};
	return hclkdivs[CLKPWR_HCLKDIV_DIV_2POW(
		__raw_readl(CLKPWR_HCLK_DIV(CLKPWR_IOBASE)))];
}

/*
 * Set new HCLK bus divider value
 */
static int clk_set_hclk_div(struct clk *clk, u32 div)
{
	u32 hclkdiv = __raw_readl(CLKPWR_HCLK_DIV(CLKPWR_IOBASE));

	hclkdiv &= ~CLKPWR_HCLKDIV_DIV_2POW(0x3);
	switch (div)
	{
		case 1:
		case 2:
			hclkdiv |= CLKPWR_HCLKDIV_DIV_2POW(div - 1);
			break;

		case 4:
		default:
			hclkdiv |= CLKPWR_HCLKDIV_DIV_2POW(2);
			break;
	}
	__raw_writel(hclkdiv, CLKPWR_HCLK_DIV(CLKPWR_IOBASE));

	return 0;
}

static u32 local_hclk_get_rate(struct clk *clk)
{
	return clk->parent->rate / clk_get_hclk_div();
}

u32 clk_get_pclk_div(void)
{
	return (1 + ((__raw_readl(CLKPWR_HCLK_DIV(CLKPWR_IOBASE)) >> 2) & 0x1F));
}

/*
 * Set new PCLK bus divider value
 */
static int clk_set_pclk_div(struct clk *clk, u32 div)
{
	u32 hclkdiv = __raw_readl(CLKPWR_HCLK_DIV(CLKPWR_IOBASE));

	hclkdiv &= ~CLKPWR_HCLKDIV_PCLK_DIV(0x1F);
	if (div > 32)
	{
		/* Limit PCLK divider */
		div = 32;
	}

	hclkdiv |= CLKPWR_HCLKDIV_PCLK_DIV(div - 1);
	__raw_writel(hclkdiv, CLKPWR_HCLK_DIV(CLKPWR_IOBASE));

	return 0;
}

static u32 local_pclk_get_rate(struct clk *clk)
{
	return clk->parent->rate / clk_get_pclk_div();
}

/* Bus clocks */
struct clk clk_hclk =
{
	.owner		= NULL,
	.name		= "hclk_ck",
	.parent		= &clk_armpll,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &clk_set_hclk_div,
	.set_parent	= NULL,
	.round_rate	= NULL,
	.get_rate	= &local_hclk_get_rate,
	.enable		= &local_clk_dummy_enable,
	.enable_reg	= 0,
	.enable_mask	= 0,
};
struct clk clk_pclk =
{
	.owner		= NULL,
	.name		= "pclk_ck",
	.parent		= &clk_armpll,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &clk_set_pclk_div,
	.set_parent	= NULL,
	.round_rate	= NULL,
	.get_rate	= &local_pclk_get_rate,
	.enable		= &local_clk_dummy_enable,
	.enable_reg	= 0,
};

static int local_onoff_set_rate(struct clk *clk, u32 rate)
{
	u32 tmp;

	tmp = __raw_readl(clk->enable_reg);

	/* Disable clock if rate is 0 */
	if (rate == 0)
	{
		tmp &= ~clk->enable_mask;
	}
	else
	{
		tmp |= clk->enable_mask;
	}

	__raw_writel(tmp, clk->enable_reg);

	return 0;
}

static u32 local_get_periph_rate(struct clk *clk)
{
	u32 tmp;

	tmp = __raw_readl(clk->enable_reg) & clk->enable_mask;

	if (!tmp)
	{
		/* Clock is disabled */
		return 0;
	}

	if (clk->parent != NULL)
	{
		/* If the parent clock has a get_rate function, call it */
		if (clk->parent->get_rate != NULL)
		{
			return clk->parent->get_rate(clk->parent);
		}

		return clk->parent->rate;
	}

	return 1;
}

static int local_onoff_enable(struct clk *clk, int enable)
{
	return local_onoff_set_rate(clk, enable);
}

static u32 local_onoff_round_rate(struct clk *clk, u32 rate)
{
	return (rate ? 1 : 0);
}

/* Peripheral clock sources */
static struct clk clk_timer0 =
{
	.owner		= NULL,
	.name		= "timer0_ck",
	.parent		= &clk_pclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_TIMERS_PWMS_CLK_CTRL_1(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_TMRPWMCLK_TIMER0_EN,
};
static struct clk clk_timer1 =
{
	.owner		= NULL,
	.name		= "timer1_ck",
	.parent		= &clk_pclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_TIMERS_PWMS_CLK_CTRL_1(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_TMRPWMCLK_TIMER1_EN,
};
static struct clk clk_timer2 =
{
	.owner		= NULL,
	.name		= "timer2_ck",
	.parent		= &clk_pclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_TIMERS_PWMS_CLK_CTRL_1(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_TMRPWMCLK_TIMER2_EN,
};
static struct clk clk_timer3 =
{
	.owner		= NULL,
	.name		= "timer3_ck",
	.parent		= &clk_pclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_TIMERS_PWMS_CLK_CTRL_1(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_TMRPWMCLK_TIMER3_EN,
};
#if defined (CONFIG_LPC32XX_WATCHDOG)
static struct clk clk_wdt =
{
	.owner		= NULL,
	.name		= "wdt_ck",
	.parent		= &clk_pclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_TIMER_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_PWMCLK_WDOG_EN,
};
#endif
static struct clk clk_vfp9 =
{
	.owner		= NULL,
	.name		= "vfp9_ck",
	.parent		= &clk_pclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_DEBUG_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_VFP_CLOCK_ENABLE_BIT,
};
static struct clk clk_dma =
{
	.owner		= NULL,
	.name		= "clk_dmac",
	.parent		= &clk_hclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_DMA_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_DMACLKCTRL_CLK_EN,
};

#if defined (CONFIG_MACH_LPC32XX_UART3_ENABLE)
static struct clk clk_uart3 =
{
	.owner		= NULL,
	.name		= "uart3_ck",
	.parent		= &clk_pclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_UART_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_UARTCLKCTRL_UART3_EN,
};
#endif
#if defined (CONFIG_MACH_LPC32XX_UART4_ENABLE)
static struct clk clk_uart4 =
{
	.owner		= NULL,
	.name		= "uart4_ck",
	.parent		= &clk_pclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_UART_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_UARTCLKCTRL_UART4_EN,
};
#endif
#if defined (CONFIG_MACH_LPC32XX_UART5_ENABLE)
static struct clk clk_uart5 =
{
	.owner		= NULL,
	.name		= "uart5_ck",
	.parent		= &clk_pclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_UART_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_UARTCLKCTRL_UART5_EN,
};
#endif
#if defined (CONFIG_MACH_LPC32XX_UART6_ENABLE)
static struct clk clk_uart6 =
{
	.owner		= NULL,
	.name		= "uart6_ck",
	.parent		= &clk_pclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_UART_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_UARTCLKCTRL_UART6_EN,
};
#endif
#if defined (CONFIG_MACH_LPC32XX_I2C0_ENABLE)
struct clk clk_i2c0 =
{
	.owner		= NULL,
	.name		= "i2c0_ck",
	.parent		= &clk_hclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_I2C_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_I2CCLK_I2C1CLK_EN,
};
#endif
#if defined (CONFIG_MACH_LPC32XX_I2C1_ENABLE)
struct clk clk_i2c1 =
{
	.owner		= NULL,
	.name		= "i2c1_ck",
	.parent		= &clk_hclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_I2C_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_I2CCLK_I2C2CLK_EN,
};
#endif
#if defined (CONFIG_MACH_LPC32XX_USBOTG_I2C_ENABLE)
struct clk clk_i2c2 =
{
	.owner		= NULL,
	.name		= "i2c2_ck",
	.parent		= &clk_pclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= USB_OTG_IOBASE + 0xFF4,
	.enable_mask	= 0x4,
};
#endif
#if defined(CONFIG_SPI_LPC32XX)
struct clk clk_ssp0 =
{
	.owner		= NULL,
	.name		= "spi0_ck",
	.parent		= &clk_hclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_SSP_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_SSPCTRL_SSPCLK0_EN,
};
struct clk clk_ssp1 =
{
	.owner		= NULL,
	.name		= "spi1_ck",
	.parent		= &clk_hclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_SSP_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_SSPCTRL_SSPCLK1_EN,
};
#endif
#if defined(CONFIG_KEYBOARD_LPC32XX)
struct clk clk_kscan =
{
	.owner		= NULL,
	.name		= "key_ck",
	.parent		= &osc_32KHz,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_KEY_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_KEYCLKCTRL_CLK_EN,
};
#endif
#if defined(CONFIG_MTD_NAND_SLC_LPC32XX)
struct clk clk_nand =
{
	.owner		= NULL,
	.name		= "nand_ck",
	.parent		= &clk_hclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_NAND_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_NANDCLK_SLCCLK_EN,
};
#endif
#if defined(CONFIG_SND_LPC3XXX_SOC)
struct clk clk_i2s0 =
{
	.owner		= NULL,
	.name		= "i2s0_ck",
	.parent		= &clk_hclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_I2S_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_I2SCTRL_I2SCLK0_EN,
};
struct clk clk_i2s1 =
{
	.owner		= NULL,
	.name		= "i2s1_ck",
	.parent		= &clk_hclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_I2S_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_I2SCTRL_I2SCLK1_EN,
};
#endif
#if defined (CONFIG_TOUCHSCREEN_LPC32XX)
static int tsc_onoff_enable(struct clk *clk, int enable)
{
	u32 tmp;

	/* Make sure 32KHz clock is the selected clock */
	tmp = __raw_readl(CLKPWR_ADC_CLK_CTRL_1(CLKPWR_IOBASE));
	tmp &= ~CLKPWR_ADCCTRL1_PCLK_SEL;
	__raw_writel(tmp, CLKPWR_ADC_CLK_CTRL_1(CLKPWR_IOBASE));

	if (enable == 0)
	{
		__raw_writel(0, clk->enable_reg);
	}
	else
	{
		__raw_writel(clk->enable_mask, clk->enable_reg);
	}

	return 0;
}
static int tsc_set_rate(struct clk *clk, u32 rate)
{
	return tsc_onoff_enable(clk, rate);
}
struct clk clk_tsc =
{
	.owner		= NULL,
	.name		= "tsc_ck",
	.parent		= &osc_32KHz,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &tsc_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &tsc_onoff_enable,
	.enable_reg	= CLKPWR_ADC_CLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_ADC32CLKCTRL_CLK_EN,
};
#endif
#if defined (CONFIG_MMC_ARMMMCI)
static int mmc_onoff_enable(struct clk *clk, int enable)
{
	u32 tmp;

	tmp = __raw_readl(CLKPWR_MS_CTRL(CLKPWR_IOBASE)) &
		~(CLKPWR_MSCARD_SDCARD_EN|CLKPWR_MSCARD_SDCARD_DIV(15));

	/* If rate is 0, disable clock */
	if (enable != 0)
	{
		tmp |= CLKPWR_MSCARD_SDCARD_EN | CLKPWR_MSCARD_SDCARD_DIV(1);
	}

	__raw_writel(tmp, CLKPWR_MS_CTRL(CLKPWR_IOBASE));

	return 0;
}
static u32 mmc_get_rate(struct clk *clk)
{
	u32 div, tmp, rate;

	div = __raw_readl(CLKPWR_MS_CTRL(CLKPWR_IOBASE));
	tmp = div & CLKPWR_MSCARD_SDCARD_EN;

	if (!tmp)
	{
		/* Clock is disabled */
		return 0;
	}

	/* Get the parent clock rate */
	rate = clk->parent->get_rate(clk->parent);

	/* Get the LCD controller clock divider value */
	div = div & 0xF;

	if (!div)
	{
		/* Clock is disabled */
		return 0;
	}

	tmp = rate / div;

	return tmp;
}
static int mmc_set_rate(struct clk *clk, u32 rate)
{
	/* If rate is 0, disable clock */
	if (rate == 0)
	{
		mmc_onoff_enable(clk, 0);
	}
	else
	{
		mmc_onoff_enable(clk, 1);
	}

	return 0;
}
struct clk clk_mmc =
{
	.owner		= NULL,
	.name		= "MCLK",
	.parent		= &clk_armpll,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &mmc_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &mmc_get_rate,
	.enable		= &mmc_onoff_enable,
	.enable_reg	= CLKPWR_MS_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_MSCARD_SDCARD_EN,
};
#endif
#if defined (CONFIG_LPC32XX_MII)
struct clk clk_net =
{
	.owner		= NULL,
	.name		= "net_ck",
	.parent		= &clk_hclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_MACCLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= (CLKPWR_MACCTRL_DMACLK_EN | CLKPWR_MACCTRL_MMIOCLK_EN | CLKPWR_MACCTRL_HRCCLK_EN),
};
#endif
#if defined (CONFIG_FB_ARMCLCD)
static u32 clcd_get_rate(struct clk *clk)
{
	u32 tmp, div, rate;

	tmp = __raw_readl(CLKPWR_LCDCLK_CTRL(CLKPWR_IOBASE)) &
		CLKPWR_LCDCTRL_CLK_EN;

	if (!tmp)
	{
		/* Clock is disabled */
		return 0;
	}

	/* Get the parent clock rate */
	rate = clk->parent->get_rate(clk->parent);

	/* Get the LCD controller clock divider value */
	tmp = __raw_readl(CLCD_POL(io_p2v(LCD_BASE)));

	/* Only supports internal clocking */
	if (tmp & CLCDC_LCDTIMING2_BCD)
	{
		/* No divider, LCD clock rate is parent rate */
		return rate;
	}

	div = (tmp & 0x1F) | ((tmp & 0xF8) >> 22);
	tmp = rate / (2 + div);

	return tmp;
}
static int clcd_set_rate(struct clk *clk, u32 rate)
{
	u32 tmp, prate, div;

	/* Get the LCD controller clock divider value */
	tmp = __raw_readl(CLCD_POL(io_p2v(LCD_BASE)));

	/* Get parent clock rate */
	prate = clk->parent->get_rate(clk->parent);

	/* If rate is 0, disable clock */
	if (rate == 0)
	{
		local_onoff_enable(clk, 0);
	}
	/* If rate is the parent rate, then just set divider bypass */
	else if (rate == prate)
	{
		tmp |= CLCDC_LCDTIMING2_BCD;
		local_onoff_enable(clk, 1);
	}
	else
	{
		/* Find closest divider */
		div = prate / rate;
		if (div == 1)
		{
			/* Divide by 2 */
			div = 0;
		}
		else
		{
			/* adjust for built-in /2 divider */
			div -= 2;
		}

		tmp &= ~(0xF800001F);
		tmp &= ~CLCDC_LCDTIMING2_BCD;
		tmp |= (div & 0x1F);
		tmp |= (((div >> 5) & 0x1F) << 27);
		__raw_writel(tmp, CLCD_POL(io_p2v(LCD_BASE)));
		local_onoff_enable(clk, 1);
	}

	return 0;
}
struct clk clk_lcd =
{
	.owner		= NULL,
	.name		= "CLCDCLK",
	.parent		= &clk_hclk,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &clcd_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &clcd_get_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_LCDCLK_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_LCDCTRL_CLK_EN,
};
#endif
#if defined (CONFIG_USB_GADGET_LPC32XX)
struct clk clk_usbd =
{
	.owner		= NULL,
	.name		= "ck_usbd",
	.parent		= &clk_usbpll,
	.rate		= 0,
	.flags		= 0,
	.usecount	= 0,
	.set_rate	= &local_onoff_set_rate,
	.set_parent	= NULL,
	.round_rate	= &local_onoff_round_rate,
	.get_rate	= &local_get_periph_rate,
	.enable		= &local_onoff_enable,
	.enable_reg	= CLKPWR_USB_CTRL(CLKPWR_IOBASE),
	.enable_mask	= CLKPWR_USBCTRL_HCLK_EN,
};
#endif

/*
 * System and peripheral clocks
 */
static struct clk *chip_clks[] = {
	&osc_32KHz,
	&osc_pll397,
	&osc_main,
	&clk_sys,
	&clk_armpll,
	&clk_hclk,
	&clk_pclk,
	&clk_usbpll,
	&clk_timer0,
	&clk_timer1,
	&clk_timer2,
	&clk_timer3,
	&clk_vfp9,
	&clk_dma,
#if defined (CONFIG_LPC32XX_WATCHDOG)
	&clk_wdt,
#endif
#if defined (CONFIG_MACH_LPC32XX_UART3_ENABLE)
	&clk_uart3,
#endif
#if defined (CONFIG_MACH_LPC32XX_UART4_ENABLE)
	&clk_uart4,
#endif
#if defined (CONFIG_MACH_LPC32XX_UART5_ENABLE)
	&clk_uart5,
#endif
#if defined (CONFIG_MACH_LPC32XX_UART6_ENABLE)
	&clk_uart6,
#endif
#if defined (CONFIG_MACH_LPC32XX_I2C0_ENABLE)
	&clk_i2c0,
#endif
#if defined (CONFIG_MACH_LPC32XX_I2C1_ENABLE)
	&clk_i2c1,
#endif
#if defined (CONFIG_MACH_LPC32XX_USBOTG_I2C_ENABLE)
	&clk_i2c2,
#endif
#if defined(CONFIG_SPI_LPC32XX)
	&clk_ssp0,
	&clk_ssp1,
#endif
#if defined(CONFIG_KEYBOARD_LPC32XX)
	&clk_kscan,
#endif
#if defined(CONFIG_MTD_NAND_SLC_LPC32XX)
	&clk_nand,
#endif
#if defined(CONFIG_SND_LPC3XXX_SOC)
	&clk_i2s0,
	&clk_i2s1,
#endif
#if defined (CONFIG_TOUCHSCREEN_LPC32XX)
	&clk_tsc,
#endif
#if defined (CONFIG_MMC_ARMMMCI)
	&clk_mmc,
#endif
#if defined (CONFIG_LPC32XX_MII)
	&clk_net,
#endif
#if defined (CONFIG_FB_ARMCLCD)
	&clk_lcd,
#endif
#if defined (CONFIG_USB_GADGET_LPC32XX)
	&clk_usbd,
#endif
};

static inline void clk_lock(void)
{
	local_irq_disable();
}

static inline void clk_unlock(void)
{
	local_irq_enable();
}

struct clk *local_clk_get(struct device *dev, const char *id)
{
	struct clk *p, *clk = ERR_PTR(-ENOENT);
	list_for_each_entry(p, &clocks, node)
	{
		if ((strcmp(id, p->name) == 0) && (try_module_get(p->owner)))
		{
			clk = p;
			goto found;
		}
	}

found:
	return clk;
}

static int local_clk_enable(struct clk *clk)
{
	int ret = 0;


	/* Enable parent clocks and update use counter for this clock */
	if (clk->parent != NULL)
	{
		ret = local_clk_enable(clk->parent);
	}

	/* Enable clock if necessary */
	if (clk->usecount == 0)
	{
		clk->enable(clk, 1);
	}

	/* Increment use count for this clock */
	clk->usecount++;

	return ret;
}

static int local_clk_disable(struct clk *clk)
{
	int ret = 0;

	/* Decrement use count for this clock */
	clk->usecount--;

	/* Disable clock if no other devices are using it */
	if (clk->usecount == 0)
	{
		clk->enable(clk, 0);
	}

	/* Disable parent clocks and update use there use counters */
	if ((clk->parent != NULL) && (ret == 0))
	{
		ret = local_clk_disable(clk->parent);
	}

	return ret;
}

unsigned long local_clk_get_rate(struct clk *clk)
{
	if (clk->rate != 0)
	{
		return clk->rate;
	}

	if (clk->get_rate != NULL)
	{
		return (clk->get_rate)(clk);
	}

	if (clk->parent != NULL)
	{
		return local_clk_get_rate(clk->parent);
	}

	return 0;
}

/*
 * clk_get - lookup and obtain a reference to a clock producer.
 */
struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *clk;

	clk_lock();
	clk = local_clk_get(dev, id);
	clk_unlock();

	return clk;
}

/*
 * clk_enable - inform the system when the clock source should be running.
 */
int clk_enable(struct clk *clk)
{
	int ret;

	if ((IS_ERR(clk)) || (clk == NULL))
	{
		return -ENODEV;
	}

	clk_lock();
	ret = local_clk_enable(clk);
	clk_unlock();

	return ret;
}

/*
 * clk_disable - inform the system when the clock source is no longer required.
 */
void clk_disable(struct clk *clk)
{
	if ((IS_ERR(clk)) || (clk == NULL))
	{
		return;
	}

	clk_lock();
	local_clk_disable(clk);
	clk_unlock();
}

/*
 * clk_get_rate - obtain the current clock rate (in Hz) for a clock source.
 *		  This is only valid once the clock source has been enabled.
 */
unsigned long clk_get_rate(struct clk *clk)
{
	unsigned long rate = 0;

	if (!IS_ERR(clk))
	{
		rate = local_clk_get_rate(clk);
	}

	return rate;
}

/*
 * clk_put - "free" the clock source
 */
void clk_put(struct clk *clk)
{
	module_put(clk->owner);
}

/*
 * clk_round_rate - adjust a rate to the exact rate a clock can provide
 */
long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (!IS_ERR(clk))
	{
		clk_lock();
		rate = clk->round_rate(clk, rate);
		clk_unlock();
	}
	else
	{
		rate = 0;
	}

	return rate;
}
 
/*
 * clk_set_rate - set the clock rate for a clock source
 */
int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -ENODEV;

	if (!IS_ERR(clk))
	{
		clk_lock();
		ret = (clk->set_rate)(clk, rate);
		clk_unlock();
	}

	return ret;
}
 
/*
 * clk_set_parent - set the parent clock source for this clock
 */
int clk_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -ENODEV;

	if (!IS_ERR(clk))
	{
		if (!clk->set_parent)
		{
			clk_lock();
			ret = clk->set_parent(clk, parent);
			clk_unlock();
		}
	}

	return ret;
}

/*
 * clk_get_parent - get the parent clock source for this clock
 */
struct clk *clk_get_parent(struct clk *clk)
{
	return clk->parent;
}

int clk_register(struct clk *clk)
{
	clk_lock();
	list_add(&clk->node, &clocks);
	clk_unlock();
	return 0;
}

void clk_unregister(struct clk *clk)
{
	clk_lock();
	list_del(&clk->node);
	clk_unlock();
}

EXPORT_SYMBOL(clk_get);
EXPORT_SYMBOL(clk_disable);
EXPORT_SYMBOL(clk_enable);
EXPORT_SYMBOL(clk_get_rate);
EXPORT_SYMBOL(clk_put);
EXPORT_SYMBOL(clk_round_rate);
EXPORT_SYMBOL(clk_set_rate);
EXPORT_SYMBOL(clk_set_parent);
EXPORT_SYMBOL(clk_get_parent);
EXPORT_SYMBOL(clk_register);
EXPORT_SYMBOL(clk_unregister);

static void clk_inc_counts(struct clk *clk)
{
	clk->usecount++;

	if (clk->parent != NULL)
	{
		clk_inc_counts(clk->parent);
	}
}

int clk_is_sysclk_mainosc(void)
{
	u32 reg;

	/* Update PLL397 or main osc use count and parent based on
	   current SYSCLK selection */
	reg = __raw_readl(CLKPWR_SYSCLK_CTRL(CLKPWR_IOBASE));
	if ((reg & CLKPWR_SYSCTRL_SYSCLKMUX) == 0)
	{
		return 1;
	}

	return 0;
}

int __init clk_init(void)
{
	u32 reg;
	int i;

	/* Register all clocks */
	for (i = 0; i < ARRAY_SIZE(chip_clks); i++)
	{
		clk_register(chip_clks[i]);
	}

	/* Set initial rates for PLL397 and main oscillator */
	reg = __raw_readl(CLKPWR_PLL397_CTRL(CLKPWR_IOBASE));
	if ((reg & CLKPWR_SYSCTRL_PLL397_DIS) != 0)
	{
		osc_pll397.rate = 0;
	}
	else
	{
		osc_pll397.rate = 397 * osc_32KHz.rate;
		clk_inc_counts(&osc_32KHz);
	}
	reg = __raw_readl(CLKPWR_MAIN_OSC_CTRL(CLKPWR_IOBASE));
	if ((reg & CLKPWR_MOSC_DISABLE) != 0)
	{
		osc_main.rate = 0;
	}
	else
	{
		osc_main.rate = MAIN_OSC_FREQ;
	}

	/* Update PLL397 or main osc use count and parent based on
	   current SYSCLK selection */
	if (clk_is_sysclk_mainosc() != 0)
	{
		/* Main oscillator used as SYSCLK source */
		clk_sys.parent = &osc_main;
	}
	else
	{
		/* PLL397 used as SYSCLK soource */
		clk_sys.parent = &osc_pll397;
	}
	clk_sys.rate = clk_sys.parent->rate;

	/* Compute the ARM PLL and USB PLL frequencies */
	local_update_armpll_rate();
	local_update_usbpll_rate();
	if (clk_usbpll.rate != 0)
	{
		/* Based on main oscillator rate */
		clk_inc_counts(&osc_main);
	}

	/* HCLK and PCLKs are always on and use the ARM PLL clock*/
	clk_inc_counts(clk_hclk.parent);
	clk_inc_counts(clk_pclk.parent);

	/* Timer 0 is used for the system clock tick. It was pre-initialized
	   prior to this elsewhere */
	clk_inc_counts(clk_timer0.parent);

#ifdef CLKDEBUG
	for (i = 0; i < ARRAY_SIZE(chip_clks); i++)
	{
		struct clk *clkp = chip_clks[i];

		pr_debug("%s: clock %s, rate %ld(Hz) Users:%d\n",
			__func__, clkp->name, local_clk_get_rate(clkp),
			clkp->usecount);
	}
#endif

	return 0;
}

