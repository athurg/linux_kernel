/*
 *  linux/drivers/input/keyboard/lpc32xx-keys.c
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

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/ioport.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <mach/board.h>
#include <mach/lpc32xx_kscan.h>

struct lpc32xx_kscan_drv
{
	struct input_dev *input;
	struct lpc32XX_kscan_cfg *kscancfg;
	struct clk *clk;
	void __iomem *kscan_base;
	int irq;
	u8 lastkeystates[8];
};

static irqreturn_t lpc32xx_kscan_irq(int irq, void *dev_id)
{
	int i, j, scancode, keycode;
	u8 key, st;
	struct lpc32xx_kscan_drv *kscandat = (struct lpc32xx_kscan_drv *) dev_id;

	for (i = 0; i < kscandat->kscancfg->matrix_sz; i++)
	{
		key = (u8) __raw_readl((i * 4) + KS_DATA(kscandat->kscan_base));
		if (key != kscandat->lastkeystates[i])
		{
			for (j = 0; j < kscandat->kscancfg->matrix_sz; j++)
			{
				st = key & (1 << j);
				if (st != (kscandat->lastkeystates[i] & (1 << j)))
				{
					/* Key state changed */
					scancode = (int) (j * kscandat->kscancfg->matrix_sz) + i;
					keycode = kscandat->kscancfg->keymap[scancode];
					input_report_key(kscandat->input, keycode, (st != 0));
				}
			}

			kscandat->lastkeystates[i] = key;
		}
	}

	/* Clear IRQ */
	__raw_writel(1, KS_IRQ(kscandat->kscan_base));

	input_sync(kscandat->input);

	return IRQ_HANDLED;
}

static int __devinit lpc32xx_kscan_probe(struct platform_device *pdev)
{
	struct lpc32xx_kscan_drv *kscandat; 
	struct resource *res;
	int retval, i;

	/* Allocate private driver data */
	kscandat = kzalloc(sizeof(struct lpc32xx_kscan_drv), GFP_KERNEL);
	if (unlikely(!kscandat))
	{
		return -ENOMEM;
	}

	/* Get resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
	{
		retval = -EBUSY;
		goto errout;
	}

	/* Save IO resources */
	kscandat->kscan_base = ioremap(res->start, res->end - res->start + 1);
	if (!kscandat->kscan_base)
	{
		retval = -EBUSY;
		goto errout;
	}

	/* Get IRQ resource */
	kscandat->irq = platform_get_irq(pdev, 0);
	if ((kscandat->irq < 0) || (kscandat->irq >= NR_IRQS))
	{
		retval = -EINVAL;
		goto errout;
	}
	retval = request_irq(kscandat->irq, lpc32xx_kscan_irq,
		IRQF_DISABLED, "kscanirq", kscandat);
	if (retval < 0)
	{
		goto errout;
	}

	kscandat->input = input_allocate_device();
	if (!kscandat->input)
	{
		retval = -ENOMEM;
		goto err_free_irq;
	}

	kscandat->kscancfg = (struct lpc32XX_kscan_cfg *) pdev->dev.platform_data;
	platform_set_drvdata(pdev, kscandat);

	/* Setup key input */
	kscandat->input->evbit[0]	= BIT_MASK(EV_KEY);
	kscandat->input->name		= pdev->name;
	kscandat->input->phys		= "matrix-keys/input0";
	kscandat->input->dev.parent	=  &pdev->dev;
	kscandat->input->id.vendor	= 0x0001;
	kscandat->input->id.product	= 0x0001;
	kscandat->input->id.version	= 0x0100;
	for (i = 0; i < kscandat->kscancfg->matrix_sz; i++)
	{
		__set_bit(kscandat->kscancfg->keymap[i], kscandat->input->keybit);
	}

	input_set_capability(kscandat->input, EV_MSC, MSC_SCAN);

	retval = input_register_device(kscandat->input);
	if (retval)
	{
		goto err_free_irq;
	}

	/* Get the key scanner clock */
	kscandat->clk = clk_get(&pdev->dev, "key_ck");
	if (IS_ERR(kscandat->clk))
	{
		goto err_free_irq;
	}
	clk_enable(kscandat->clk);

	/* Configure the key scanner */
	__raw_writel(kscandat->kscancfg->deb_clks, KS_DEB(kscandat->kscan_base));
	__raw_writel(kscandat->kscancfg->scan_delay, KS_SCAN_CTL(kscandat->kscan_base));
	__raw_writel(KSCAN_FTST_USE32K_CLK, KS_FAST_TST(kscandat->kscan_base));
	__raw_writel(kscandat->kscancfg->matrix_sz, KS_MATRIX_DIM(kscandat->kscan_base));
	__raw_writel(1, KS_IRQ(kscandat->kscan_base));

	return 0;

err_free_irq:
	free_irq(kscandat->irq, pdev);
errout:
	if (!kscandat)
	{
		if (!kscandat->input)
		{
			input_free_device(kscandat->input);
		}
		if (!kscandat->kscan_base)
		{
			iounmap(kscandat->kscan_base);
		}

		kfree(kscandat);
	}

	return retval;
}

static int __devexit lpc32xx_kscan_remove(struct platform_device *pdev)
{
	struct lpc32xx_kscan_drv *kscandat = platform_get_drvdata(pdev);

	free_irq(kscandat->irq, pdev);
	input_unregister_device(kscandat->input);
	clk_put(kscandat->clk);
	iounmap(kscandat->kscan_base);
	kfree(kscandat);

	return 0;
}

#ifdef CONFIG_PM
static int lpc32xx_kscan_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct lpc32xx_kscan_drv *kscandat = platform_get_drvdata(pdev);

	/* Clear IRQ and disable clock */
	__raw_writel(1, KS_IRQ(kscandat->kscan_base));
	clk_disable(kscandat->clk);

	return 0;
}

static int lpc32xx_kscan_resume(struct platform_device *pdev)
{
	struct lpc32xx_kscan_drv *kscandat = platform_get_drvdata(pdev);

	/* Enable clock and clear IRQ */
	clk_enable(kscandat->clk);
	__raw_writel(1, KS_IRQ(kscandat->kscan_base));

	return 0;
}
#else
#define lpc32xx_kscan_suspend	NULL
#define lpc32xx_kscan_resume	NULL
#endif

struct platform_driver lpc32xx_kscan_driver = {
	.probe		= lpc32xx_kscan_probe,
	.remove		= __devexit_p(lpc32xx_kscan_remove),
	.suspend	= lpc32xx_kscan_suspend,
	.resume		= lpc32xx_kscan_resume,
	.driver		= {
		.name	= "lpc32xx_keys",
	}
};

static int __init lpc32xx_kscan_init(void)
{
	return platform_driver_register(&lpc32xx_kscan_driver);
}

static void __exit lpc32xx_kscan_exit(void)
{
	platform_driver_unregister(&lpc32xx_kscan_driver);
}

module_init(lpc32xx_kscan_init);
module_exit(lpc32xx_kscan_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("Key scanner driver for LPC32XX devices");

