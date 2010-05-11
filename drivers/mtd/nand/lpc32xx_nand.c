/*
 *  drivers/mtd/nand/lpc32xx_nand.c
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/lpc32xx_slcnand.h>

struct lpc32xx_nand_host {
	struct nand_chip	nand_chip;
	struct clk		*clk;
	struct mtd_info		mtd;
	void __iomem		*io_base;
	struct lpc32XX_nand_cfg	*ncfg;
};

static void lpc32xx_nand_setup(struct lpc32xx_nand_host *host)
{
	u32 clkrate, tmp;

	/* Reset SLC controller */
	__raw_writel(SLCCTRL_SW_RESET, SLC_CTRL(host->io_base));
	udelay(1000);

	/* Basic setup */
	__raw_writel(0, SLC_CFG(host->io_base));
	__raw_writel(0, SLC_IEN(host->io_base));
	__raw_writel((SLCSTAT_INT_TC | SLCSTAT_INT_RDY_EN), SLC_ICR(host->io_base));

	/* Get base clock for SLC block */
	clkrate = clk_get_rate(host->clk);
	if (clkrate == 0)
	{
		clkrate = 104000000;
	}

	/* Compute clock setup values */
	tmp = SLCTAC_WDR(host->ncfg->wdr_clks) |
		SLCTAC_WWIDTH(1 + (clkrate / host->ncfg->wwidth)) |
		SLCTAC_WHOLD(1 + (clkrate / host->ncfg->whold)) |
		SLCTAC_WSETUP(1 + (clkrate / host->ncfg->wsetup)) |
		SLCTAC_RDR(host->ncfg->rdr_clks) |
		SLCTAC_RWIDTH(1 + (clkrate / host->ncfg->rwidth)) |
		SLCTAC_RHOLD(1 + (clkrate / host->ncfg->rhold)) |
		SLCTAC_RSETUP(1 + (clkrate / host->ncfg->rsetup));
	__raw_writel(tmp, SLC_TAC(host->io_base));
}

/*
 * Hardware specific access to control lines
 */
static void lpc32xx_nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	u32 tmp;
	struct nand_chip *nand_chip = mtd->priv;
	struct lpc32xx_nand_host *host = nand_chip->priv;

	/* Does CE state need to be changed? */
	tmp = __raw_readl(SLC_CFG(host->io_base));
	if (ctrl & NAND_NCE)
	{
		tmp |= SLCCFG_CE_LOW;
	}
	else
	{
		tmp &= ~SLCCFG_CE_LOW;
	}
	__raw_writel(tmp, SLC_CFG(host->io_base));

	if (cmd != NAND_CMD_NONE)
	{
		if (ctrl & NAND_CLE)
		{
			__raw_writel(cmd, SLC_CMD(host->io_base));
		}
		else
		{
			__raw_writel(cmd, SLC_ADDR(host->io_base));
		}
	}
}

/*
 * Read the Device Ready pin.
 */
static int lpc32xx_nand_device_ready(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct lpc32xx_nand_host *host = nand_chip->priv;
	int rdy = 0;

	if ((__raw_readl(SLC_STAT(host->io_base)) & SLCSTAT_NAND_READY) != 0)
	{
		rdy = 1;
	}

	return rdy;
}

/*
 * Enable NAND write protect
 */
static void lpc32xx_wp_enable(struct lpc32xx_nand_host *host)
{
	if (host->ncfg->enable_write_prot != NULL)
	{
		/* Disable write protection */
		host->ncfg->enable_write_prot(1);
	}
}

/*
 * Disable NAND write protect
 */
static void lpc32xx_wp_disable(struct lpc32xx_nand_host *host)
{
	if (host->ncfg->enable_write_prot != NULL)
	{
		/* Enable write protection */
		host->ncfg->enable_write_prot(0);
	}
}

static uint8_t lpc32xx_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct lpc32xx_nand_host *host = nand_chip->priv;

	return (uint8_t) __raw_readl(SLC_DATA(host->io_base));
}

static void lpc32xx_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct lpc32xx_nand_host *host = nand_chip->priv;
	int i;

	for (i = 0; i < len; i++)
	{
		buf[i] = (uint8_t) __raw_readl(SLC_DATA(host->io_base));
	}
}

static int lpc32xx_verify_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct lpc32xx_nand_host *host = nand_chip->priv;
	int i;

	for (i = 0; i < len; i++)
	{
		if (buf[i] != (uint8_t) __raw_readl(SLC_DATA(host->io_base)))
		{
			return -EFAULT;
		}
	}

	return 0;
}

static void lpc32xx_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct lpc32xx_nand_host *host = nand_chip->priv;
	int i;

	for (i = 0; i < len; i++)
	{
		__raw_writel((u32) buf[i], SLC_DATA(host->io_base));
	}
}

#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif

/*
 * Probe for NAND controller
 */
static int __init lpc32xx_nand_probe(struct platform_device *pdev)
{
	struct lpc32xx_nand_host *host;
	struct mtd_info *mtd;
	struct nand_chip *nand_chip;
	int res;

#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *partitions = NULL;
	int num_partitions = 0;
#endif

	/* Allocate memory for the device structure (and zero it) */
	host = kzalloc(sizeof(struct lpc32xx_nand_host), GFP_KERNEL);
	if (!host) {
		printk(KERN_ERR "lpc32xx_nand: failed to allocate device structure.\n");
		return -ENOMEM;
	}

	host->io_base = ioremap(pdev->resource[0].start,
				pdev->resource[0].end - pdev->resource[0].start + 1);
	if (host->io_base == NULL) {
		printk(KERN_ERR "lpc32xx_nand: ioremap failed\n");
		res = -EIO;
		goto err_exit1;
	}

	mtd = &host->mtd;
	nand_chip = &host->nand_chip;
	host->ncfg = pdev->dev.platform_data;

	nand_chip->priv = host;		/* link the private data structures */
	mtd->priv = nand_chip;
	mtd->owner = THIS_MODULE;

	/* Get NAND clock */
	host->clk = clk_get(&pdev->dev, "nand_ck");
	if (IS_ERR(host->clk)) {
		printk(KERN_ERR "lpc32xx_nand: Clock failure\n");
		res = -ENOENT;
		goto err_exit2;
	}
	clk_enable(host->clk);

	/* Set address of NAND IO lines */
	nand_chip->IO_ADDR_R = SLC_DATA(host->io_base);
	nand_chip->IO_ADDR_W = SLC_DATA(host->io_base);
	nand_chip->cmd_ctrl = lpc32xx_nand_cmd_ctrl;
	nand_chip->dev_ready = lpc32xx_nand_device_ready;
	nand_chip->ecc.mode = NAND_ECC_SOFT;	/* enable ECC */
	nand_chip->chip_delay = 20;		/* 20us command delay time */
	nand_chip->read_byte = lpc32xx_read_byte;
	nand_chip->read_buf = lpc32xx_read_buf;
	nand_chip->verify_buf = lpc32xx_verify_buf;
	nand_chip->write_buf = lpc32xx_write_buf;

	/* Init NAND controller */
	lpc32xx_nand_setup(host);
	lpc32xx_wp_disable(host);

	platform_set_drvdata(pdev, host);

	/* Scan to find existance of the device */
	if (nand_scan(mtd, 1)) {
		res = -ENXIO;
		goto err_exit3;
	}

#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
	mtd->name = "lpc32xx_nand";
	num_partitions = parse_mtd_partitions(mtd, part_probes,
					      &partitions, 0);
#endif
	if ((num_partitions <= 0) && (host->ncfg->partition_info))
	{
		partitions = host->ncfg->partition_info(mtd->size,
							 &num_partitions);
	}

	if ((!partitions) || (num_partitions == 0)) {
		printk(KERN_ERR "lpc32xx_nand: No parititions defined, or unsupported device.\n");
		res = ENXIO;
		nand_release(mtd);
		goto err_exit3;
	}

	res = add_mtd_partitions(mtd, partitions, num_partitions);
#else
	res = add_mtd_device(mtd);
#endif

	if (!res)
	{
		return res;
	}

	nand_release(mtd);
err_exit3:
	clk_put(host->clk);
	platform_set_drvdata(pdev, NULL);
err_exit2:
	lpc32xx_wp_enable(host);
	iounmap(host->io_base);
err_exit1:
	kfree(host);

	return res;
}

/*
 * Remove NAND device.
 */
static int __devexit lpc32xx_nand_remove(struct platform_device *pdev)
{
	u32 tmp;
	struct lpc32xx_nand_host *host = platform_get_drvdata(pdev);
	struct mtd_info *mtd = &host->mtd;

	nand_release(mtd);

	/* Force CE high */
	tmp = __raw_readl(SLC_CTRL(host->io_base));
	tmp &= ~SLCCFG_CE_LOW;
	__raw_writel(tmp, SLC_CTRL(host->io_base));

	lpc32xx_wp_enable(host);
	clk_disable(host->clk);
	clk_put(host->clk);

	iounmap(host->io_base);
	kfree(host);

	return 0;
}

#if defined(CONFIG_PM)
static int lpc32xx_nand_resume(struct platform_device *pdev)
{
	struct lpc32xx_nand_host *host = platform_get_drvdata(pdev);

	/* Re-enable NAND clock */
	clk_enable(host->clk);

	/* Fresh init of NAND controller */
	lpc32xx_nand_setup(host);

	/* Disable write protect */
	lpc32xx_wp_disable(host);

	return 0;
}

static int lpc32xx_nand_suspend(struct platform_device *pdev, pm_message_t pm)
{
	u32 tmp;
	struct lpc32xx_nand_host *host = platform_get_drvdata(pdev);

	/* Force CE high */
	tmp = __raw_readl(SLC_CTRL(host->io_base));
	tmp &= ~SLCCFG_CE_LOW;
	__raw_writel(tmp, SLC_CTRL(host->io_base));

	/* Enable write protect for safety */
	lpc32xx_wp_enable(host);

	/* Disable clock */
	clk_disable(host->clk);

	return 0;
}

#else
#define lpc32xx_nand_resume NULL
#define lpc32xx_nand_suspend NULL
#endif

static struct platform_driver lpc32xx_nand_driver = {
	.probe		= lpc32xx_nand_probe,
	.remove		= __devexit_p(lpc32xx_nand_remove),
	.resume		= lpc32xx_nand_resume,
	.suspend	= lpc32xx_nand_suspend,
	.driver		= {
		.name	= "lpc32xx-nand",
		.owner	= THIS_MODULE,
	},
};

static int __init lpc32xx_nand_init(void)
{
	return platform_driver_register(&lpc32xx_nand_driver);
}

static void __exit lpc32xx_nand_exit(void)
{
	platform_driver_unregister(&lpc32xx_nand_driver);
}


module_init(lpc32xx_nand_init);
module_exit(lpc32xx_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Wells(kevin.wells@nxp.com)");
MODULE_DESCRIPTION("NAND driver for the NXP LPC32XX SLC controller");

