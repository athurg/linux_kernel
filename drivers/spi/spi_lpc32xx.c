/*
 * drivers/spi/spi_lpc32xx.c
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

#define SSP_DEBUG

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/lpc32xx_ssp.h>
#include <mach/board.h>

struct lpc32xxspi
{
	spinlock_t lock;
	struct workqueue_struct	*workqueue;
	struct work_struct work;
	struct list_head queue;
	wait_queue_head_t waitq;
	struct spi_master *master;
	struct clk *clk;
	void __iomem *membase;
	int irq;
	int id;
	struct lpc32xx_spi_cfg *psspcfg;
	u32 current_speed_hz;
	u8 current_bits_wd;
};

/*
 * Default configuration for boards without specific setup
 */
static struct lpc32xx_spi_cfg lpc32xx_stdspi_cfg =
{
	.num_cs = 1,
};

static void lpc32xx_spi_prep(struct lpc32xxspi *spidat)
{
	u32 tmp;

	/* Clear and mask SSP interrupts */
	__raw_writel((SSP_ICR_RORIC | SSP_ICR_RTIC), SSP_ICR(spidat->membase));
	__raw_writel(0, SSP_IMSC(spidat->membase));

	/* Setup default SPI mode */
	__raw_writel((SSP_CR0_DSS(16) | SSP_CR0_FRF_SPI | SSP_CR0_CPOL(0) |
		SSP_CR0_CPHA(0) | SSP_CR0_SCR(0)), SSP_CR0(spidat->membase));
	__raw_writel(SSP_CR1_SSP_ENABLE, SSP_CR1(spidat->membase));
	__raw_writel(SSP_CPSR_CPDVSR(2), SSP_CPSR(spidat->membase));

	/* Flush FIFO */
	while (__raw_readl(SSP_SR(spidat->membase)) & SSP_SR_RNE)
	{
		tmp = __raw_readl(SSP_DATA(spidat->membase));
	}

	/* Controller stays disabled until a transfer occurs */
}

static void lpc32xx_cs_set_state(struct spi_device *spi, unsigned int cs,
				unsigned int active, unsigned int delay_ns)
{
	struct lpc32xxspi *spidat = spi_master_get_devdata(spi->master);
	int val = (spi->mode & SPI_CS_HIGH) ? active : !active;

	if (spidat->psspcfg->spi_cs_set != NULL)
	{
		spidat->psspcfg->spi_cs_set(cs, val);
	}

	ndelay(delay_ns);
}

static void lpc32xx_update_spi_clock(struct lpc32xxspi *spidat, u32 speed_hz)
{
	u32 tmp, scr, cpsr, cmp_clk, bus_hz;

	/* Get base clock for this device */
	bus_hz = clk_get_rate(spidat->clk);

	/* speed zero convention is used by some upper layers */
	if ((speed_hz) && (speed_hz > (bus_hz / (0x100 * 0xFF))))
	{
		cmp_clk = 0xFFFFFFFF;
		cpsr = 2;
		scr = 0;

		/* Find best clock dividers to get desired rate */
		while (cmp_clk > speed_hz)
		{
			cmp_clk = bus_hz / ((scr + 1) * cpsr);
			if (cmp_clk > speed_hz)
			{
				scr++;
				if (scr > 0xFF)
				{
					scr = 0;
					cpsr += 2;
				}
			}
		}
	}
	else
	{
		/* Use maximum dividers */
		scr = 0xFF;
		cpsr = 0xFF;
	}

	spidat->current_speed_hz = bus_hz / (cpsr * (scr + 1));

	tmp = __raw_readl(SSP_CR0(spidat->membase));
	tmp &= ~SSP_CR0_SCR(0xFF);
	tmp |= SSP_CR0_SCR(scr);
	__raw_writel(tmp, SSP_CR0(spidat->membase));
	__raw_writel(cpsr, SSP_CPSR(spidat->membase));
}

static void lpc32xx_update_spi_dwidth(struct lpc32xxspi *spidat, u32 bits_per_word)
{
	u32 tmp;

	tmp = __raw_readl(SSP_CR0(spidat->membase));
	tmp &= ~SSP_CR0_DSS(16);
	tmp |= SSP_CR0_DSS(bits_per_word);
	__raw_writel(tmp, SSP_CR0(spidat->membase));

	spidat->current_bits_wd = bits_per_word;
}

/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH)

static int lpc32xx_spi_setup(struct spi_device *spi)
{
	struct lpc32xxspi *spidat = spi_master_get_devdata(spi->master);
	unsigned long flags;
	unsigned int bits = spi->bits_per_word;
	u32 tmp;

	if (spi->chip_select > spi->master->num_chipselect)
	{
		dev_dbg(&spi->dev,
			"setup: invalid chipselect %u (%u defined)\n",
			spi->chip_select, spi->master->num_chipselect);
		return -EINVAL;
	}

	if (bits == 0)
	{
		bits = 8;
	}
	if ((bits < 4) || (bits > 16))
	{
		dev_dbg(&spi->dev,
			"setup: invalid bits_per_word %u (8 to 16)\n", bits);
		return -EINVAL;
	}

	if (spi->mode & ~MODEBITS)
	{
		dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
			spi->mode & ~MODEBITS);
		return -EINVAL;
	}

	spin_lock_irqsave(&spidat->lock, flags);
	clk_enable(spidat->clk);

	/* Setup CR0 register */
	tmp = SSP_CR0_FRF_SPI;
	if (spi->mode & SPI_CPOL)
	{
		tmp |= SSP_CR0_CPOL(1);
	}
	if (spi->mode & SPI_CPHA)
	{
		tmp |= SSP_CR0_CPHA(1);
	}

	__raw_writel(tmp, SSP_CR0(spidat->membase));
	lpc32xx_update_spi_dwidth(spidat, bits);
	lpc32xx_update_spi_clock(spidat, spi->max_speed_hz);
	lpc32xx_cs_set_state(spi, spi->chip_select, 0, 0);
	clk_disable(spidat->clk);
	spin_unlock_irqrestore(&spidat->lock, flags);

	dev_dbg(&spi->dev, "SSP (%d) prog rate = %d / "
		"actual rate = %d, bits = %d\n", spi->chip_select,
		spi->max_speed_hz, spidat->current_speed_hz, spidat->current_bits_wd);	

	return 0;
}

static irqreturn_t lpc32xx_spi_irq(int irq, void *dev_id)
{
	struct lpc32xxspi *spidat = dev_id;

	/* Disable interrupts for now, do not clear the interrupt states */
	__raw_writel(0, SSP_IMSC(spidat->membase));

	wake_up(&spidat->waitq);

	return IRQ_HANDLED;
}

static void lpc32xx_work_one(struct lpc32xxspi *spidat, struct spi_message *m)
{
	struct spi_device *spi = m->spi;
	struct spi_transfer *t;
	u32 tmp;
	int cs_delay, wsize, cs_change = 1;
	int status = 0;
	unsigned long flags;

	/* Enable SSP clock and interrupts */
	spin_lock_irqsave(&spidat->lock, flags);
	clk_enable(spidat->clk);
	enable_irq(spidat->irq);

	/* CS setup/hold/recovery time in nsec */
	cs_delay = 100 + (NSEC_PER_SEC / 2) / spi->max_speed_hz;

	list_for_each_entry (t, &m->transfers, transfer_list) {
		const void *txbuf = t->tx_buf;
		void *rxbuf = t->rx_buf;
		u32 data;
		int tsize, rlen, tlen = t->len;
		u32 speed_hz = t->speed_hz ? : spi->max_speed_hz;
		u8 bits_per_word = t->bits_per_word ? : spi->bits_per_word;

		bits_per_word = bits_per_word ? : 8;
		wsize = bits_per_word >> 3;

		if (spidat->current_speed_hz != speed_hz)
		{
			lpc32xx_update_spi_clock(spidat, speed_hz);
		}
		if (spidat->current_bits_wd != bits_per_word)
		{
			lpc32xx_update_spi_dwidth(spidat, bits_per_word);
		}

		/* Number of actual words to transfer (versus bytes) */
		tlen = tlen / wsize;

		/* Make sure FIFO is flushed, clear pending interrupts, and
		   then enable SSP interface */
		while (__raw_readl(SSP_SR(spidat->membase)) & SSP_SR_RNE)
		{
			tmp = __raw_readl(SSP_DATA(spidat->membase));
		}
		__raw_writel((SSP_ICR_RORIC | SSP_ICR_RTIC), SSP_ICR(spidat->membase));
		tmp = __raw_readl(SSP_CR1(spidat->membase));
		tmp |= SSP_CR1_SSP_ENABLE;
		__raw_writel(tmp, SSP_CR1(spidat->membase));

		/* Assert selected chip select */
		if (cs_change)
		{
			lpc32xx_cs_set_state(spi, spi->chip_select, 1, cs_delay);
		}
		cs_change = t->cs_change;

		/* Verify idle state */
		while (__raw_readl(SSP_SR(spidat->membase)) & SSP_SR_BSY)
		{
			cpu_relax();
		}

		if (rxbuf != NULL)
			rlen = tlen;			
		else
			rlen = 0;				

		while (tlen > 0)
		{
			if(txbuf != NULL)
			{
				/* Valid TX data */
				if (__raw_readl(SSP_SR(spidat->membase)) & SSP_SR_TFE)
				{
					tsize = tlen;
										
					if (tsize > SSP_FIFO_DEPTH_WORDS)
					{
						tsize = SSP_FIFO_DEPTH_WORDS;
					}					
					tlen -= tsize;
					

					// Fill TX FIFO 
					while (tsize > 0)
					{						
						data = (wsize == 1)
							? *(const u8 *) txbuf
							: *(const u16 *) txbuf;
						__raw_writel(data, SSP_DATA(spidat->membase));
						txbuf += wsize;						
						tsize --;
					}
				}									
			}
			else 
			{
				/* Dummy TX data for RX */
				if (__raw_readl(SSP_SR(spidat->membase)) & SSP_SR_TFE)
				{
					/* Send dummy data for each read */
					__raw_writel(0x00, SSP_DATA(spidat->membase));
				}
				tlen--;	
			}	

			/* Wait for data */
			__raw_writel((SSP_IMSC_RTIM | SSP_IMSC_RXIM), SSP_IMSC(spidat->membase));
			spin_unlock_irqrestore(&spidat->lock, flags);
			wait_event(spidat->waitq,
				(__raw_readl(SSP_RIS(spidat->membase)) &
				(SSP_MIS_RTMIS | SSP_MIS_RXMIS)));
			spin_lock_irqsave(&spidat->lock, flags);

			/* Has an overflow occurred? */
			if (unlikely(__raw_readl(SSP_MIS(spidat->membase)) &
				SSP_MIS_RORMIS))
			{
				lpc32xx_cs_set_state(spi, spi->chip_select, 0, 0);
				status = -EIO;
				goto exit;
			}

			/* Clear other interrupts */
			__raw_writel((SSP_ICR_RORIC | SSP_ICR_RTIC),
				SSP_ICR(spidat->membase));

			/* Is there any data to read? */
			while (__raw_readl(SSP_SR(spidat->membase)) & SSP_SR_RNE)
			{
				data = __raw_readl(SSP_DATA(spidat->membase));
				if ((rxbuf) && (rlen > 0))
				{
					if (wsize == 1)
					{
						*(u8 *)rxbuf = (u8) data;
					}
					else
					{
						*(u16 *)rxbuf = (u16) data;
					}
					rxbuf += wsize;
					rlen --;
				}										
			}
		}

		m->actual_length += t->len;
		if (t->delay_usecs)
		{
			udelay(t->delay_usecs);
		}

		if (!cs_change)
			continue;
		if (t->transfer_list.next == &m->transfers)
			break;
		/* sometimes a short mid-message deselect of the chip
		 * may be needed to terminate a mode or command
		 */
		lpc32xx_cs_set_state(spi, spi->chip_select, 0, cs_delay);
	}

exit:
	spin_unlock_irqrestore(&spidat->lock, flags);

	/* Disable SSP and SSP interrupts, stop SSP clock to save power */
	tmp = __raw_readl(SSP_CR1(spidat->membase));
	tmp &= ~SSP_CR1_SSP_ENABLE;
	__raw_writel(tmp, SSP_CR1(spidat->membase));
	disable_irq(spidat->irq);
	clk_disable(spidat->clk);

	if (!(status == 0 && cs_change))
	{
		lpc32xx_cs_set_state(spi, spi->chip_select, 0, cs_delay);
	}

	m->status = status;
	m->complete(m->context);
}

static void lpc32xx_work(struct work_struct *work)
{
	struct lpc32xxspi *spidat = container_of(work, struct lpc32xxspi, work);
	unsigned long flags;

	spin_lock_irqsave(&spidat->lock, flags);

	while (!list_empty(&spidat->queue))
	{
		struct spi_message *m;

		m = container_of(spidat->queue.next, struct spi_message, queue);
		list_del_init(&m->queue);

		spin_unlock_irqrestore(&spidat->lock, flags);
		lpc32xx_work_one(spidat, m);
		spin_lock_irqsave(&spidat->lock, flags);
	}

	spin_unlock_irqrestore(&spidat->lock, flags);
}



static int lpc32xx_spi_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct spi_master *master = spi->master;
	struct lpc32xxspi *spidat = spi_master_get_devdata(master);
	struct device *controller = spi->master->dev.parent;
	struct spi_transfer *t;
	unsigned long flags;

	m->actual_length = 0;

	/* check each transfer's parameters */
	list_for_each_entry (t, &m->transfers, transfer_list)
	{
		u8 bits_per_word = t->bits_per_word ? : spi->bits_per_word;

		bits_per_word = bits_per_word ? : 8;
		if ((!t->tx_buf) && (!t->rx_buf) && (t->len))
		{
			return -EINVAL;
		}
		if ((bits_per_word < 4) || (bits_per_word > 16))
		{
			return -EINVAL;
		}

		dev_dbg(controller,
			"  xfer %p: len %u tx %p/%08x rx %p/%08x\n",
			t, t->len, t->tx_buf, t->tx_dma,
			t->rx_buf, t->rx_dma);
	}

	spin_lock_irqsave(&spidat->lock, flags);
	list_add_tail(&m->queue, &spidat->queue);
	queue_work(spidat->workqueue, &spidat->work);
	spin_unlock_irqrestore(&spidat->lock, flags);

	return 0;
}

static int __init lpc32xx_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct lpc32xxspi *spidat;
	struct resource *res;
	char clkname[16];
	int ret, irq, i;
	
	/* Get required resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if ((!res) || (irq < 0) | (irq >= NR_IRQS))
	{
		dev_err(&pdev->dev, "Error getting resources\n");
		return -EBUSY;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(struct lpc32xxspi));
	if (!master)
	{
		dev_err(&pdev->dev, "Error allocating SPI master\n");
		return -ENODEV;
	}
	spidat = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, master);

	/* Save ID for this device */
	spidat->id = pdev->id;
	spidat->irq = irq;
	spin_lock_init(&spidat->lock);

	INIT_WORK(&spidat->work, lpc32xx_work);
	INIT_LIST_HEAD(&spidat->queue);
	init_waitqueue_head(&spidat->waitq);
	spidat->workqueue = create_singlethread_workqueue(master->dev.parent->bus_id);
	if (!spidat->workqueue)
	{
		dev_err(&pdev->dev, "Error creating work queue\n");
		ret = -ENOMEM;
		goto errout;
	}

	/* Generate clock name and get clock */
	snprintf(clkname, 10, "spi%d_ck", spidat->id);
	spidat->clk = clk_get(&pdev->dev, clkname);
	if (IS_ERR(spidat->clk)) {
		dev_err(&pdev->dev, "Error getting clock\n");
		ret = -ENODEV;
		goto errout_qdel;
	}
	clk_enable(spidat->clk);

	/* Save IO resources */
	spidat->membase = ioremap(res->start, res->end - res->start + 1);
	if (!spidat->membase)
	{
		dev_err(&pdev->dev, "Error mapping IO\n");
		ret = -EBUSY;
		goto errout2;
	}

	ret = request_irq(spidat->irq, lpc32xx_spi_irq,
		IRQF_DISABLED, "spiirq", spidat);
	if (ret)
	{
		dev_err(&pdev->dev, "Error requesting interrupt\n");
		ret = -EBUSY;
		goto errout3;
	}
	disable_irq(spidat->irq);

	master->bus_num = spidat->id;
	master->setup = lpc32xx_spi_setup;
	master->transfer = lpc32xx_spi_transfer;

	/* Is a board specific configuration available? */
	spidat->psspcfg = (struct lpc32xx_spi_cfg *) pdev->dev.platform_data;
	if (spidat->psspcfg == NULL)
	{
		spidat->psspcfg = &lpc32xx_stdspi_cfg;
	}
	if (spidat->psspcfg->num_cs < 1)
	{
		spidat->psspcfg = &lpc32xx_stdspi_cfg;
	}

	master->num_chipselect = spidat->psspcfg->num_cs;

	/* Initialize each chip select and set chip select low */
	for (i = 0; i < spidat->psspcfg->num_cs; i++)
	{
		if (spidat->psspcfg->spi_cs_setup != NULL)
		{
			spidat->psspcfg->spi_cs_setup(i);
		}
		if (spidat->psspcfg->spi_cs_set != NULL)
		{
			spidat->psspcfg->spi_cs_set(i, 0);
		}
	}

	/* Initial setup of SPI */
	lpc32xx_spi_prep(spidat);

	/* Keep the SSP clock off until a transfer is performed to save power */
	clk_disable(spidat->clk);

	ret = spi_register_master(master);
	if (ret)
	{
		dev_err(&pdev->dev, "Error registering SPI driver\n");
		goto errout4;
	}

	return 0;

errout4:
	free_irq(spidat->irq, pdev);
errout3:
	iounmap(spidat->membase);
errout2:
	clk_disable(spidat->clk);
	clk_put(spidat->clk);
errout_qdel:
	destroy_workqueue(spidat->workqueue);
errout:
	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return ret;
}

static int __devexit lpc32xx_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct lpc32xxspi *spidat = spi_master_get_devdata(master);

	spi_unregister_master(master);
	platform_set_drvdata(pdev, NULL);

	free_irq(spidat->irq, pdev);

	iounmap(spidat->membase);
	clk_disable(spidat->clk);
	clk_put(spidat->clk);

	destroy_workqueue(spidat->workqueue);
	spi_master_put(master);

	return 0;
}

static struct platform_driver lpc32xx_spi_driver = {
	.probe		= lpc32xx_spi_probe,
	.remove		= __devexit_p(lpc32xx_spi_remove),
	.driver		= {
		.name	= "spi_lpc32xx",
		.owner	= THIS_MODULE,
	},
};

static int __init lpc32xx_spi_init(void)
{
	return platform_driver_register(&lpc32xx_spi_driver);
}

static void __exit lpc32xx_spi_exit(void)
{
	platform_driver_unregister(&lpc32xx_spi_driver);
}

module_init(lpc32xx_spi_init);
module_exit(lpc32xx_spi_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com");
MODULE_DESCRIPTION("LPC32XX SSP/SPI Driver");
MODULE_LICENSE("GPL");

