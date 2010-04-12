/*
 * drivers/input/touchscreen/lpc32xx_tsc.c
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

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/clk.h>

#include <mach/hardware.h>
#include <asm/io.h>

#include <mach/clock.h>
#include <mach/lpc32xx_tsc.h>

#define MOD_NAME "lpc32xx-ts"

struct lpc32xx_tsc_t
{
	struct input_dev *dev;
	void __iomem *tsc_base;
	int irq;
	struct clk *clk;
};

static void lpc32xx_fifo_clear(struct lpc32xx_tsc_t *lpc32xx_tsc_dat)
{
	volatile u32 tmp;

	while (!(__raw_readl(TSC_STAT(lpc32xx_tsc_dat->tsc_base)) & TSC_STAT_FIFO_EMPTY))
	{
		tmp = __raw_readl(TSC_FIFO(lpc32xx_tsc_dat->tsc_base));
	}
}

static irqreturn_t lpc32xx_ts_interrupt(int irq, void *dev_id)
{
	u32 tmp, rv [16], xs [16], ys [16];
	int idx;
	struct lpc32xx_tsc_t *lpc32xx_tsc_dat = (struct lpc32xx_tsc_t *) dev_id;
	struct input_dev *input = lpc32xx_tsc_dat->dev;

	/* Get the touch status */
	tmp = __raw_readl(TSC_STAT(lpc32xx_tsc_dat->tsc_base));

	/* FIFO overflow - throw away samples */
	if (tmp & TSC_STAT_FIFO_OVRRN)
	{
		lpc32xx_fifo_clear(lpc32xx_tsc_dat);
		return IRQ_HANDLED;
	}

	/* Get samples from FIFO */
	idx = 0;
	while ((idx < 4) &&
		(!(__raw_readl(TSC_STAT(lpc32xx_tsc_dat->tsc_base)) &
		TSC_STAT_FIFO_EMPTY)))
	{
		tmp = __raw_readl(TSC_FIFO(lpc32xx_tsc_dat->tsc_base));
		xs [idx] = TSC_ADCDAT_VALUE_MASK - TSC_FIFO_NORMALIZE_X_VAL(tmp);
		ys [idx] = TSC_ADCDAT_VALUE_MASK - TSC_FIFO_NORMALIZE_Y_VAL(tmp);
		rv [idx] = tmp;
		idx++;
	}

	/* Data is only valid if pen is still down */
	if ((!(rv [3] & TSC_FIFO_TS_P_LEVEL)) && (idx == 4))
	{
		input_report_abs(input, ABS_X, ((xs [1] + xs [2]) / 2));
		input_report_abs(input, ABS_Y, ((ys [1] + ys [2]) / 2));
#if defined(CONFIG_TOUCHSCREEN_LPC32XX_ADDPRESSURE)
		input_report_abs(input, ABS_PRESSURE, 1);
#endif
		input_report_key(input, BTN_TOUCH, 1);
	}
	else
	{
#if defined(CONFIG_TOUCHSCREEN_LPC32XX_ADDPRESSURE)
		input_report_abs(input, ABS_PRESSURE, 0);
#endif
		input_report_key(input, BTN_TOUCH, 0);
	}

	input_sync(input);

	return IRQ_HANDLED;
}

static void stop_tsc(struct lpc32xx_tsc_t *lpc32xx_tsc_dat)
{
	u32 tmp;

	/* Disable auto mode */
	tmp = __raw_readl(TSC_CON(lpc32xx_tsc_dat->tsc_base));
	tmp &= ~TSC_ADCCON_AUTO_EN;
	__raw_writel(tmp, TSC_CON(lpc32xx_tsc_dat->tsc_base));
}

static void setup_tsc(struct lpc32xx_tsc_t *lpc32xx_tsc_dat)
{
	u32 tmp;

	/* Power down ADC */
	tmp = __raw_readl(TSC_CON(lpc32xx_tsc_dat->tsc_base));
	tmp &= ~TSC_ADCCON_POWER_UP;
	__raw_writel(tmp, TSC_CON(lpc32xx_tsc_dat->tsc_base));

	/* Set the TSC FIFO depth to 4 samples @ 10-bits sample */
	tmp = (TSC_ADCCON_IRQ_TO_FIFO_4 | TSC_ADCCON_X_SAMPLE_SIZE(10) |
		TSC_ADCCON_Y_SAMPLE_SIZE(10));
	__raw_writel(tmp, TSC_CON(lpc32xx_tsc_dat->tsc_base));

	/* Set SEL to default value */
	__raw_writel(0x0284, TSC_SEL(lpc32xx_tsc_dat->tsc_base));

	/* Min/max and aux settings */
	__raw_writel(0x0000, TSC_MIN_X(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x03FF, TSC_MAX_X(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x0000, TSC_MIN_Y(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x03FF, TSC_MAX_Y(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x0000, TSC_AUX_UTR(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x0000, TSC_AUX_MIN(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x0000, TSC_AUX_MAX(lpc32xx_tsc_dat->tsc_base));

	/* Rise, update, delay, touch, drain X times */
	__raw_writel(TSC_RTR_RISE_TIME(0x2), TSC_RTR(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(TSC_UTR_UPDATE_TIME(446), TSC_UTR(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(TSC_DTR_DELAY_TIME(0x2), TSC_DTR(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(TSC_TTR_TOUCH_TIME(0x10), TSC_TTR(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(TSC_DXP_DRAINX_TIME(0x4), TSC_DXP(lpc32xx_tsc_dat->tsc_base));

	/* Set sample rate to about 60Hz, this rate is based on the
	   RTC clock, which should be a stable 32768Hz */
	__raw_writel(TSC_UTR_UPDATE_TIME(88), TSC_UTR(lpc32xx_tsc_dat->tsc_base));

	/* Empty the touchscreen FIFO */
	lpc32xx_fifo_clear(lpc32xx_tsc_dat);

	/* Enable auto mode */
	tmp = __raw_readl(TSC_CON(lpc32xx_tsc_dat->tsc_base));
	tmp |= TSC_ADCCON_AUTO_EN;
	__raw_writel(tmp, TSC_CON(lpc32xx_tsc_dat->tsc_base));
}

static int __devinit lpc32xx_ts_probe(struct platform_device *pdev)
{
	struct lpc32xx_tsc_t *lpc32xx_tsc_dat = NULL;
	struct resource *res;
	int retval = -ENODEV;

	/* Get resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
	{
		retval = -EBUSY;
		goto errout;
	}

	/* Allocate private driver data */
	lpc32xx_tsc_dat = kzalloc(sizeof(struct lpc32xx_tsc_t), GFP_KERNEL);
	if (unlikely(!lpc32xx_tsc_dat))
	{
		retval = -ENOMEM;
		goto errout;
	}

	/* Map IO resources */
	lpc32xx_tsc_dat->tsc_base = ioremap(res->start, res->end - res->start + 1);
	if (!lpc32xx_tsc_dat->tsc_base)
	{
		retval = -EBUSY;
		goto errout;
	}

	lpc32xx_tsc_dat->dev = input_allocate_device();
	if (!lpc32xx_tsc_dat->dev)
	{
		retval = -ENOMEM;
		goto errout;
	}

	/* Enable clock */
	lpc32xx_tsc_dat->clk = clk_get(&pdev->dev, "tsc_ck");
	if (IS_ERR(lpc32xx_tsc_dat->clk))
	{
		goto errout;
	}
	clk_enable(lpc32xx_tsc_dat->clk);

	/* Initially setup TSC */
	setup_tsc(lpc32xx_tsc_dat);

	/* Get interrupt */
	lpc32xx_tsc_dat->irq = platform_get_irq(pdev, 0);
	if ((lpc32xx_tsc_dat->irq < 0) || (lpc32xx_tsc_dat->irq >= NR_IRQS))
	{
		retval = -EINVAL;
		goto errout;
	}

	retval = request_irq(lpc32xx_tsc_dat->irq, lpc32xx_ts_interrupt,
		0, MOD_NAME, lpc32xx_tsc_dat);
	if (retval < 0)
	{
		goto err_free_irq;
	}

	platform_set_drvdata(pdev, lpc32xx_tsc_dat);

	/* Setup TS driver */
	lpc32xx_tsc_dat->dev->name = "LPC32xx Touchscreen";
	lpc32xx_tsc_dat->dev->phys = "lpc32xx/input0";
	lpc32xx_tsc_dat->dev->id.bustype = BUS_HOST;
	lpc32xx_tsc_dat->dev->id.vendor = 0x0001;
	lpc32xx_tsc_dat->dev->id.product = 0x0002;
	lpc32xx_tsc_dat->dev->id.version = 0x0100;
	lpc32xx_tsc_dat->dev->dev.parent = &pdev->dev;

	lpc32xx_tsc_dat->dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	lpc32xx_tsc_dat->dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(lpc32xx_tsc_dat->dev, ABS_X, 0, 1023, 0, 0);
	input_set_abs_params(lpc32xx_tsc_dat->dev, ABS_Y, 0, 1023, 0, 0);

#if defined(CONFIG_TOUCHSCREEN_LPC32XX_ADDPRESSURE)
	input_set_abs_params(lpc32xx_tsc_dat->dev, ABS_PRESSURE, 0, 1, 0, 0);
#endif

	retval = input_register_device(lpc32xx_tsc_dat->dev);
	if (retval)
		goto err_free_irq;

	return 0;

err_free_irq:
	stop_tsc(lpc32xx_tsc_dat);
	platform_set_drvdata(pdev, NULL);
	free_irq(lpc32xx_tsc_dat->irq, lpc32xx_tsc_dat->dev);

errout:
	if (lpc32xx_tsc_dat)
	{
		if (lpc32xx_tsc_dat->clk)
		{
			clk_disable(lpc32xx_tsc_dat->clk);
			clk_put(lpc32xx_tsc_dat->clk);
		}

		if (lpc32xx_tsc_dat->dev)
		{
			input_free_device(lpc32xx_tsc_dat->dev);
		}

		if (lpc32xx_tsc_dat->tsc_base)
		{
			iounmap(lpc32xx_tsc_dat->tsc_base);
		}

		kfree(lpc32xx_tsc_dat);
	}

	return retval;
}

static int __devexit lpc32xx_ts_remove(struct platform_device *pdev)
{
	struct lpc32xx_tsc_t *lpc32xx_tsc_dat = platform_get_drvdata(pdev);

	stop_tsc(lpc32xx_tsc_dat);
	free_irq(lpc32xx_tsc_dat->irq, lpc32xx_tsc_dat->dev);
	platform_set_drvdata(pdev, NULL);
	input_unregister_device(lpc32xx_tsc_dat->dev);

	if (lpc32xx_tsc_dat->clk)
	{
		clk_disable(lpc32xx_tsc_dat->clk);
		clk_put(lpc32xx_tsc_dat->clk);
	}

	if (lpc32xx_tsc_dat->tsc_base)
	{
		iounmap(lpc32xx_tsc_dat->tsc_base);
	}
	kfree(lpc32xx_tsc_dat);

	return 0;
}

static struct platform_driver lpc32xx_ts_driver = {
	.probe		= lpc32xx_ts_probe,
	.remove		= __devexit_p(lpc32xx_ts_remove),
	.driver		= {
		.name	= MOD_NAME,
	},
};

static int __init lpc32xx_ts_init(void)
{
	return platform_driver_register(&lpc32xx_ts_driver);
}

static void __exit lpc32xx_ts_exit(void)
{
	platform_driver_unregister(&lpc32xx_ts_driver);
}

module_init(lpc32xx_ts_init);
module_exit(lpc32xx_ts_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com");
MODULE_DESCRIPTION("LPC32XX TSC Driver");
MODULE_LICENSE("GPL");

