/*
 * drivers/rtc/rtc-lpc32xx.c
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

#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include <asm/div64.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/lpc32xx_rtc.h>

static char rtc_name[] = "rtc-lpc32xx";

struct lpc32xx_rtc_priv
{
	void __iomem *rtc_base;
	int irq;
	int alarm_enabled;
	struct rtc_device *rtc;
	spinlock_t lock;
};

static unsigned long epoch = 1970;	/* Jan 1 1970 00:00:00 */

static inline unsigned long read_seconds(u32 iobase)
{
	return __raw_readl(RTC_UCOUNT(iobase));
}

static inline void write_seconds(u32 iobase, unsigned long sec)
{
	u32 tmp;

	/* Stop counter first */
	tmp = __raw_readl(RTC_CTRL(iobase));
	tmp |= RTC_CNTR_DIS;
	__raw_writel(tmp, RTC_CTRL(iobase));

	__raw_writel(sec, RTC_UCOUNT(iobase));
	__raw_writel((0xFFFFFFFF - sec), RTC_DCOUNT(iobase));

	/* Restart counter */
	tmp &= ~RTC_CNTR_DIS;
	__raw_writel(tmp, RTC_CTRL(iobase));

	/* Set key so boot ROM won't clear RTC on reset */
	__raw_writel(RTC_KEY_ONSW_LOADVAL, RTC_KEY(iobase));
}

static void lpc32xx_rtc_release(struct device *dev)
{
	u32 reg;
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = platform_get_drvdata(pdev);

	spin_lock_irq(&lpc32xx_rtc_dat->lock);

	/* Disable RTC match interrupt */
	reg = __raw_readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
	reg &= ~RTC_MATCH0_EN;
	__raw_writel(reg, RTC_CTRL(lpc32xx_rtc_dat->rtc_base));

	spin_unlock_irq(&lpc32xx_rtc_dat->lock);

	/* Disable RTC interrupt in interrupt controller */
	disable_irq(lpc32xx_rtc_dat->irq);
}

static int lpc32xx_rtc_read_time(struct device *dev, struct rtc_time *time)
{
	unsigned long epoch_sec, elapsed_sec;
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = platform_get_drvdata(pdev);

	epoch_sec = mktime(epoch, 1, 1, 0, 0, 0);
	elapsed_sec = read_seconds((u32) lpc32xx_rtc_dat->rtc_base);

	rtc_time_to_tm(epoch_sec + elapsed_sec, time);

	return 0;
}

static int lpc32xx_rtc_set_time(struct device *dev, struct rtc_time *time)
{
	unsigned long epoch_sec, current_sec;
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = platform_get_drvdata(pdev);

	epoch_sec = mktime(epoch, 1, 1, 0, 0, 0);
	current_sec = mktime(time->tm_year + 1900, time->tm_mon + 1, time->tm_mday,
	                     time->tm_hour, time->tm_min, time->tm_sec);

	spin_lock_irq(&lpc32xx_rtc_dat->lock);
	write_seconds((u32) lpc32xx_rtc_dat->rtc_base, (current_sec - epoch_sec));
	spin_unlock_irq(&lpc32xx_rtc_dat->lock);

	return 0;
}

static int lpc32xx_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	unsigned long alarmsecs;
	struct rtc_time *time = &wkalrm->time;
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = platform_get_drvdata(pdev);

	/* Read alarm match register */
	alarmsecs = __raw_readl(RTC_MATCH0(lpc32xx_rtc_dat->rtc_base));
	wkalrm->enabled = lpc32xx_rtc_dat->alarm_enabled;

	rtc_time_to_tm(alarmsecs, time);

	return 0;
}

static int lpc32xx_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	unsigned long alarmsecs;
	u32 tmp;
	struct rtc_time *time = &wkalrm->time;
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = platform_get_drvdata(pdev);

	alarmsecs = mktime(time->tm_year + 1900, time->tm_mon + 1, time->tm_mday,
	                   time->tm_hour, time->tm_min, time->tm_sec);

	spin_lock_irq(&lpc32xx_rtc_dat->lock);

	if (lpc32xx_rtc_dat->alarm_enabled)
	{
		disable_irq(lpc32xx_rtc_dat->irq);

		tmp = __raw_readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
		tmp &= ~RTC_MATCH0_EN;
		__raw_writel(tmp, RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
	}

	__raw_writel(alarmsecs, RTC_MATCH0(lpc32xx_rtc_dat->rtc_base));
	lpc32xx_rtc_dat->alarm_enabled = wkalrm->enabled;

	if (wkalrm->enabled)
	{
		enable_irq(lpc32xx_rtc_dat->irq);

		tmp = __raw_readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
		tmp |= RTC_MATCH0_EN;
		__raw_writel(tmp, RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
	}

	spin_unlock_irq(&lpc32xx_rtc_dat->lock);

	return 0;
}

static int lpc32xx_rtc_proc(struct device *dev, struct seq_file *seq)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = platform_get_drvdata(pdev);
	u32 tmp;

	tmp = __raw_readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
	seq_printf(seq, "Alarm_IRQ\t: %s\n",
		   (tmp & RTC_MATCH0_EN) ? "yes" : "no");

	return 0;
}

static int lpc32xx_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = platform_get_drvdata(pdev);

	switch (cmd) {
	case RTC_AIE_ON:
		spin_lock_irq(&lpc32xx_rtc_dat->lock);

		if (!lpc32xx_rtc_dat->alarm_enabled)
		{
			enable_irq(lpc32xx_rtc_dat->irq);
			lpc32xx_rtc_dat->alarm_enabled = 1;
		}

		spin_unlock_irq(&lpc32xx_rtc_dat->lock);
		break;

	case RTC_AIE_OFF:
		spin_lock_irq(&lpc32xx_rtc_dat->lock);

		if (lpc32xx_rtc_dat->alarm_enabled)
		{
			disable_irq(lpc32xx_rtc_dat->irq);
			lpc32xx_rtc_dat->alarm_enabled = 0;
		}

		spin_unlock_irq(&lpc32xx_rtc_dat->lock);
		break;

	case RTC_PIE_ON:
		enable_irq(lpc32xx_rtc_dat->irq);
		break;

	case RTC_PIE_OFF:
		disable_irq(lpc32xx_rtc_dat->irq);
		break;

	case RTC_IRQP_READ:
		return put_user(1, (unsigned long __user *)arg);
		break;

	case RTC_IRQP_SET:
		/* Only 1Hz is supported */
		if (arg != 1)
			return -EINVAL;
		break;

	case RTC_EPOCH_READ:
		return put_user(epoch, (unsigned long __user *)arg);

	case RTC_EPOCH_SET:
		/* Doesn't support before 1900 */
		if (arg < 1900)
			return -EINVAL;
		epoch = arg;
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

static irqreturn_t lpc32xx_rtc_alarm_interrupt(int irq, void *dev)
{
	u32 tmp;
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = (struct lpc32xx_rtc_priv *) dev;

	spin_lock(&lpc32xx_rtc_dat->lock);

	/* If the alarm isn't disabled, the match will keep occuring, so disable
           it now */
	tmp = __raw_readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
	tmp &= ~RTC_MATCH0_EN;
	__raw_writel(tmp, RTC_CTRL(lpc32xx_rtc_dat->rtc_base));

	/* Clear match event */
	__raw_writel(RTC_MATCH0_INT_STS, RTC_INTSTAT(lpc32xx_rtc_dat->rtc_base));

	/* Alarm event */
	rtc_update_irq(lpc32xx_rtc_dat->rtc, 1, RTC_AF);

	spin_unlock(&lpc32xx_rtc_dat->lock);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops lpc32xx_rtc_ops = {
	.release	= lpc32xx_rtc_release,
	.ioctl		= lpc32xx_rtc_ioctl,
	.read_time	= lpc32xx_rtc_read_time,
	.set_time	= lpc32xx_rtc_set_time,
	.read_alarm	= lpc32xx_rtc_read_alarm,
	.set_alarm	= lpc32xx_rtc_set_alarm,
	.proc		= lpc32xx_rtc_proc,
};

static int __devinit lpc32xx_rtc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = NULL;
	int retval;
	u32 tmp;

	/* Get resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
	{
		retval = -EBUSY;
		goto errout;
	}

	/* Allocate private driver data */
	lpc32xx_rtc_dat = kzalloc(sizeof(struct lpc32xx_rtc_priv), GFP_KERNEL);
	if (unlikely(!lpc32xx_rtc_dat))
	{
		retval = -ENOMEM;
		goto errout;
	}

	/* Save resources */
	lpc32xx_rtc_dat->rtc_base = ioremap(res->start, res->end - res->start + 1);
	if (!lpc32xx_rtc_dat->rtc_base)
	{
		retval = -EBUSY;
		goto errout;
	}

	lpc32xx_rtc_dat->rtc = rtc_device_register(rtc_name, &pdev->dev,
		&lpc32xx_rtc_ops, THIS_MODULE);
	if (IS_ERR(lpc32xx_rtc_dat->rtc))
	{
		retval = PTR_ERR(lpc32xx_rtc_dat->rtc);
		goto errout;
	}

	spin_lock_init(&lpc32xx_rtc_dat->lock);

	/* Setup RTC */
	spin_lock_irq(&lpc32xx_rtc_dat->lock);
	tmp = __raw_readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
	tmp &= ~(RTC_SW_RESET | RTC_CNTR_DIS | RTC_MATCH0_EN | RTC_MATCH1_EN |
		RTC_ONSW_MATCH0_EN | RTC_ONSW_MATCH1_EN);
	__raw_writel(tmp, RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
	__raw_writel((RTC_MATCH0_INT_STS | RTC_MATCH1_INT_STS),
		RTC_INTSTAT(lpc32xx_rtc_dat->rtc_base));
	__raw_writel(RTC_KEY_ONSW_LOADVAL, RTC_KEY(lpc32xx_rtc_dat->rtc_base));
	spin_unlock_irq(&lpc32xx_rtc_dat->lock);

	/* Get interrupt */
	lpc32xx_rtc_dat->irq = platform_get_irq(pdev, 0);
	if ((lpc32xx_rtc_dat->irq < 0) || (lpc32xx_rtc_dat->irq >= NR_IRQS))
	{
		retval = -EINVAL;
		goto errout;
	}

	retval = request_irq(lpc32xx_rtc_dat->irq, lpc32xx_rtc_alarm_interrupt,
		IRQF_DISABLED, "rtcalarm", lpc32xx_rtc_dat);
	if (retval < 0)
	{
		goto err_free_irq;
	}

	platform_set_drvdata(pdev, lpc32xx_rtc_dat);
	disable_irq(lpc32xx_rtc_dat->irq);

#if defined(CONFIG_RTC_DEBUG)
	printk(KERN_INFO "rtc: %s initialized\n", rtc_name);
	printk(KERN_INFO "rtc: Using IO range 0x%08x to 0x%08x\n",
		res->start, res->end);
	printk(KERN_INFO "rtc: Alarm mapped on IRQ %d\n", lpc32xx_rtc_dat->irq);
#endif

	return 0;

err_free_irq:
	free_irq(lpc32xx_rtc_dat->irq, pdev);

errout:
#if defined(CONFIG_RTC_DEBUG)
	printk(KERN_ERR "%s: Error initializing RTC\n", rtc_name);
#endif

	if (lpc32xx_rtc_dat->rtc)
	{
		rtc_device_unregister(lpc32xx_rtc_dat->rtc);
	}
	if (lpc32xx_rtc_dat)
	{
		if (lpc32xx_rtc_dat->rtc_base)
		{
			iounmap(lpc32xx_rtc_dat->rtc_base);
		}

		kfree(lpc32xx_rtc_dat);
	}

	return retval;
}

static int __devexit lpc32xx_rtc_remove(struct platform_device *pdev)
{
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat;

	lpc32xx_rtc_dat = platform_get_drvdata(pdev);
	if (lpc32xx_rtc_dat->rtc)
	{
		rtc_device_unregister(lpc32xx_rtc_dat->rtc);
	}

	platform_set_drvdata(pdev, NULL);

	if (lpc32xx_rtc_dat)
	{
		free_irq(lpc32xx_rtc_dat->irq, pdev);
		if (lpc32xx_rtc_dat->rtc_base)
		{
			iounmap(lpc32xx_rtc_dat->rtc_base);
		}

		kfree(lpc32xx_rtc_dat);
	}

	return 0;
}

static struct platform_driver lpc32xx_rtc_driver = {
	.probe		= lpc32xx_rtc_probe,
	.remove		= __devexit_p(lpc32xx_rtc_remove),
	.driver		= {
		.name	= rtc_name,
		.owner	= THIS_MODULE,
	},
};

static int __init lpc32xx_rtc_init(void)
{
	return platform_driver_register(&lpc32xx_rtc_driver);
}

static void __exit lpc32xx_rtc_exit(void)
{
	platform_driver_unregister(&lpc32xx_rtc_driver);
}

module_init(lpc32xx_rtc_init);
module_exit(lpc32xx_rtc_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com");
MODULE_DESCRIPTION("LPC32XX RTC Driver");
MODULE_LICENSE("GPL");

