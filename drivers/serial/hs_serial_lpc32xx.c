/*
 *  drivers/serial/hs_serial_lpc32xx.c
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
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/nmi.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/lpc32xx_hsuart.h>

#define MODNAME "lpc32xx_hsuart"
#define REVISION "$Revision: 0.90 $"

struct lpc32xx_hsuart_port
{
	struct uart_port port;
};

#define PASS_LIMIT	128
#define MAX_PORTS 3
#define LPC32XX_TTY_NAME "ttyTX"
#define LPC32XX_TTY_MINOR_START	196
#define LPC32XX_TTY_MAJOR	204
static struct lpc32xx_hsuart_port lpc32xx_hs_ports[MAX_PORTS];

#ifndef CONFIG_LPC32XX_HSUART_CONSOLE
#define CONFIG_LPC32XX_HSUART_CONSOLE 1
#endif

#ifdef CONFIG_LPC32XX_HSUART_CONSOLE
void wait_for_xmit_empty(struct uart_port *port)
{
    unsigned int timeout = 10000;

    // Wait up to 10 ms for the transmit buffer to empty.
    do {
        // Stop when the level reaches zero
        if (HSU_TX_LEV(__raw_readl(HSUART_LEVEL(port->membase))) == 0)
            break;
        // Check for timeout every 1 uSec
        if (--timeout == 0)
            break;
        udelay(1);
    } while (1);
}

void wait_for_xmit_ready(struct uart_port *port)
{
    unsigned int timeout = 10000;
    while (1) {
        // Stop when the level goes below the threshold
        if (HSU_TX_LEV(__raw_readl(HSUART_LEVEL(port->membase))) < 32)
            break;
        // Check for timeout every 1 uSec
        if (--timeout == 0)
            break;
        udelay(1);
    }
}

static void lpc32xx_hsuart_console_putchar(struct uart_port *port, int ch)
{
    // Wait for room in the xmit buffer
    wait_for_xmit_ready(port);
    // Send out a character
    __raw_writel((u32) ch, HSUART_FIFO(port->membase));
}

static void
lpc32xx_hsuart_console_write(struct console *co, const char *s, unsigned int count)
{
    struct lpc32xx_hsuart_port *up = &lpc32xx_hs_ports[co->index];
    unsigned long flags;
    int locked = 1;

    touch_nmi_watchdog();
    local_irq_save(flags);
    if (up->port.sysrq) {
        locked = 0;
    } else if (oops_in_progress) {
        locked = spin_trylock(&up->port.lock);
    } else {
        spin_lock(&up->port.lock);
    }

    uart_console_write(&up->port, s, count, lpc32xx_hsuart_console_putchar);
    wait_for_xmit_empty(&up->port);

    if (locked)
        spin_unlock(&up->port.lock);
    local_irq_restore(flags);
}

static int __init lpc32xx_hsuart_console_setup(struct console *co, char *options)
{
    struct uart_port *port;
    int baud = 115200;
    int bits = 8;
    int parity = 'n';
    int flow = 'n';

    // Setup the console to the given index, or always use ttyTX0
    if (co->index >= MAX_PORTS)
        co->index = 0;

    // Check that the port has a base address (thus, valid)
    port = &lpc32xx_hs_ports[co->index].port;
    if (!port->membase)
        return -ENODEV;

    // Parse the options (if any)
    if (options)
        uart_parse_options(options, &baud, &parity, &bits, &flow);

    // Set the options
    return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver lpc32xx_hsuart_reg;
static struct console lpc32xx_hsuart_console = {
    .name         = LPC32XX_TTY_NAME,
    .write        = lpc32xx_hsuart_console_write,
    .device       = uart_console_device,
    .setup        = lpc32xx_hsuart_console_setup,
    .flags        = CON_PRINTBUFFER,
    .index        = -1,
    .data         = &lpc32xx_hsuart_reg,
};

static __init lpc32xx_hsuart_console_init(void)
{
	// NOTE: Do we need to initialize ports any here?  Probably doesn't
	// matter since bootloader initialized the hardware anyway
	// -- lshields 5/14/2009
    register_console(&lpc32xx_hsuart_console);
    return 0;
}
console_initcall(lpc32xx_hsuart_console_init);

#define LPC32XX_HSUART_CONSOLE	&lpc32xx_hsuart_console
#else
#define LPC32XX_HSUART_CONSOLE NULL
#endif

static struct uart_driver lpc32xx_hs_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= MODNAME,
	.dev_name	= LPC32XX_TTY_NAME,
	.major		= LPC32XX_TTY_MAJOR,
	.minor		= LPC32XX_TTY_MINOR_START,
	.nr		= MAX_PORTS,
	.cons		= LPC32XX_HSUART_CONSOLE,
};
static int uarts_registered = 0;

static u32 __serial_abs(u32 in1, u32 in2) {
	if (in1 > in2) {
		return in1 - in2;
	}

	return in2 - in1;
}

static unsigned int __serial_get_clock_div(unsigned long uartclk,
	unsigned long rate)
{
	u32 div, goodrate, hsu_rate, l_hsu_rate, comprate;
	u32 rate_diff;

	/* Find the closest divider to get the desired clock rate */
	div = uartclk / rate;
	goodrate = hsu_rate = (div / 14) - 1;
	if (hsu_rate != 0)
	{
		hsu_rate--;
	}

	/* Tweak divider */
	l_hsu_rate = hsu_rate + 3;
	rate_diff = 0xFFFFFFFF;

	while (hsu_rate < l_hsu_rate)
	{
		comprate = uartclk / ((hsu_rate + 1) * 14);
		if (__serial_abs(comprate, rate) < rate_diff)
		{
			goodrate = hsu_rate;
			rate_diff = __serial_abs(comprate, rate);
		}

		hsu_rate++;
	}
	if (hsu_rate > 0xFF)
	{
		hsu_rate = 0xFF;
	}

	return goodrate;
}

static void __serial_uart_flush(struct uart_port *port)
{
	u32 tmp;
	int cnt = 0;

	while ((__raw_readl(HSUART_LEVEL(port->membase)) > 0) &&
		(cnt++ < PASS_LIMIT))
	{
		tmp = __raw_readl(HSUART_FIFO(port->membase));
	}
}

static void __serial_lpc32xx_rx(struct uart_port *port)
{
	struct tty_struct *tty = port->info->port.tty;
	unsigned int tmp, flag;

	/* Read data from FIFO and push into terminal */
	tmp = __raw_readl(HSUART_FIFO(port->membase));
	while (!(tmp & HSU_RX_EMPTY))
	{
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (tmp & HSU_ERROR_DATA)
		{
			/* Framing error */
			__raw_writel(HSU_FE_INT, HSUART_IIR(port->membase));
			port->icount.frame++;
			flag = TTY_FRAME;
			tty_insert_flip_char (port->info->port.tty, 0, TTY_FRAME);
			tty_schedule_flip (port->info->port.tty);
		}

		tty_insert_flip_char (port->info->port.tty, (tmp & 0xFF), flag);

		tmp = __raw_readl(HSUART_FIFO(port->membase));
	}

	tty_flip_buffer_push(tty);
}

static void __serial_lpc32xx_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->info->xmit;
	unsigned int tmp;

	if (port->x_char)
	{
		__raw_writel((u32) port->x_char, HSUART_FIFO(port->membase));
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
	{
		goto exit_tx;
	}

	/* Transfer data */
	while (HSU_TX_LEV(__raw_readl(HSUART_LEVEL(port->membase))) < 64)
	{
		__raw_writel((u32) xmit->buf[xmit->tail],
			HSUART_FIFO(port->membase));
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
	{
		uart_write_wakeup(port);
	}

exit_tx:
	if (uart_circ_empty(xmit))
	{
		/* Stop TX interrrupt */
		tmp = __raw_readl(HSUART_CTRL(port->membase));
		tmp &= ~HSU_TX_INT_EN;
		__raw_writel(tmp, HSUART_CTRL(port->membase));
	}
}

static irqreturn_t serial_lpc32xx_interrupt(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	u32 status;

	spin_lock(&port->lock);

	/* Read UART status and clear latched interrupts */
	status = __raw_readl(HSUART_IIR(port->membase));

	if (status & HSU_BRK_INT)
	{
		/* Break received */
		__raw_writel(HSU_BRK_INT, HSUART_IIR(port->membase));
		port->icount.brk++;
		uart_handle_break(port);
	}

	if (status & HSU_FE_INT)
	{
		/* framing error received */
		__raw_writel(HSU_FE_INT, HSUART_IIR(port->membase));
	}

	if (status & HSU_RX_OE_INT)
	{
		/* Receive FIFO overrun */
		__raw_writel(HSU_RX_OE_INT, HSUART_IIR(port->membase));
		port->icount.overrun++;
		tty_insert_flip_char (port->info->port.tty, 0, TTY_OVERRUN);
		tty_schedule_flip (port->info->port.tty);
	}

	/* Data received? */
	if (status & (HSU_RX_TIMEOUT_INT | HSU_RX_TRIG_INT))
	{
		__serial_lpc32xx_rx(port);
	}

	/* Transmit data request? */
	if ((status & HSU_TX_INT) && (!uart_tx_stopped(port)))
	{
		__raw_writel(HSU_TX_INT, HSUART_IIR(port->membase));
		__serial_lpc32xx_tx(port);
	}

	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}

static unsigned int serial_lpc32xx_tx_empty(struct uart_port *port)
{
	unsigned int ret = 0;

	if (HSU_TX_LEV(__raw_readl(HSUART_LEVEL(port->membase))) == 0)
	{
		/* TX FIFO is empty */
		ret = TIOCSER_TEMT;
	}

	return ret;
}

static void serial_lpc32xx_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* No signals are supported on HS UARTs */
}

static unsigned int serial_lpc32xx_get_mctrl(struct uart_port *port)
{
	/* No signals are supported on HS UARTs */
	return (TIOCM_CAR | TIOCM_DSR | TIOCM_CTS);
}

static void serial_lpc32xx_stop_tx(struct uart_port *port)
{
	unsigned long flags;
	u32 tmp;

	spin_lock_irqsave(&port->lock, flags);

	/* Stop TX interrrupt */
	tmp = __raw_readl(HSUART_CTRL(port->membase));
	tmp &= ~HSU_TX_INT_EN;
	__raw_writel(tmp, HSUART_CTRL(port->membase));

	spin_unlock_irqrestore(&port->lock, flags);
}

static void serial_lpc32xx_start_tx(struct uart_port *port)
{
	unsigned long flags;
	u32 tmp;

	spin_lock_irqsave(&port->lock, flags);

	/* Start TX interrrupt */
	__serial_lpc32xx_tx(port);
	tmp = __raw_readl(HSUART_CTRL(port->membase));
	tmp |= HSU_TX_INT_EN;
	__raw_writel(tmp, HSUART_CTRL(port->membase));

	spin_unlock_irqrestore(&port->lock, flags);
}

static void serial_lpc32xx_stop_rx(struct uart_port *port)
{
	unsigned long flags;
	u32 tmp;

	spin_lock_irqsave(&port->lock, flags);

	tmp = __raw_readl(HSUART_CTRL(port->membase));
	tmp &= ~(HSU_RX_INT_EN | HSU_ERR_INT_EN);
	__raw_writel(tmp, HSUART_CTRL(port->membase));

	__raw_writel((HSU_BRK_INT | HSU_RX_OE_INT | HSU_FE_INT),
		HSUART_IIR(port->membase));

	spin_unlock_irqrestore(&port->lock, flags);
}

static void serial_lpc32xx_enable_ms(struct uart_port *port)
{
	/* Modem status is not supported */
}

static void serial_lpc32xx_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;
	u32 tmp;

	spin_lock_irqsave(&port->lock, flags);
	tmp = __raw_readl(HSUART_CTRL(port->membase));
	if (break_state != 0)
	{
		/* Send break */
		tmp |= HSU_BREAK;
	}
	else
	{
		tmp &= ~HSU_BREAK;
	}
	__raw_writel(tmp, HSUART_CTRL(port->membase));
	spin_unlock_irqrestore(&port->lock, flags);
}

static int serial_lpc32xx_startup(struct uart_port *port)
{
	int retval;
	u32 tmp;

	/* Empty FIFO */
	__serial_uart_flush(port);

	/* Clear latched interrupt states */
	__raw_writel((HSU_TX_INT | HSU_FE_INT | HSU_BRK_INT | HSU_RX_OE_INT),
		HSUART_IIR(port->membase));

	/* Setup initial rate to slowest speed */
	__raw_writel(0xFF, HSUART_RATE(port->membase));

	/* Set receiver timeout, HSU offset of 20, no break, no interrupts,
	   and default FIFO trigger levels */
	tmp = HSU_TX_TL8B | HSU_RX_TL32B | HSU_OFFSET(20) | HSU_TMO_INACT_4B;
	__raw_writel(tmp, HSUART_CTRL(port->membase));

	retval = request_irq(port->irq, serial_lpc32xx_interrupt,
			     0, MODNAME, port);
	if (retval)
	{
		return retval;
	}

	/* Enable receive interrupts */
	__raw_writel((tmp | HSU_RX_INT_EN | HSU_ERR_INT_EN),
		HSUART_CTRL(port->membase));

	return 0;
}

static void serial_lpc32xx_shutdown(struct uart_port *port)
{
	u32 tmp;

	/* Disable interrupts and break */
	tmp = HSU_TX_TL8B | HSU_RX_TL32B | HSU_OFFSET(20) | HSU_TMO_INACT_4B;
	__raw_writel(tmp, HSUART_CTRL(port->membase));

	free_irq(port->irq, port);
}

static void serial_lpc32xx_set_termios(struct uart_port *port,
	struct ktermios *termios, struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud, quot;
	u32 tmp;

	/* The high speed UART only supports 8-bit operation with 1 stop bit and
	   no parity, so there isn't really a lot to do here. Clock speed is
	   supported, so set it to the desired rate */

	/* Always 8-bit, no parity, 1 stop bit */
	termios->c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD);
	termios->c_cflag |= CS8;

	/* No support for modem control lines */
	termios->c_cflag &= ~(HUPCL | CMSPAR | CLOCAL | CRTSCTS);

	/* Compute block divider */
	baud = uart_get_baud_rate(port, termios, old, 0, (port->uartclk / 14));
	quot = __serial_get_clock_div(port->uartclk, baud);

	spin_lock_irqsave(&port->lock, flags);

	/* Ignore characters? */
	tmp = __raw_readl(HSUART_CTRL(port->membase));
	if ((termios->c_cflag & CREAD) == 0)
	{
		/* Disable UART interrupts */
		tmp &= ~(HSU_RX_INT_EN | HSU_ERR_INT_EN);
	}
	else
	{
		/* Enable UART interrupts */
		tmp |= HSU_RX_INT_EN | HSU_ERR_INT_EN;
	}
	__raw_writel(tmp, HSUART_CTRL(port->membase));

	/* Setup baud rate */
	__raw_writel(quot, HSUART_RATE(port->membase));

	/* Update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *serial_lpc32xx_type(struct uart_port *port)
{
	return MODNAME;
}

static void serial_lpc32xx_release_port(struct uart_port *port)
{
	if ((port->iotype == UPIO_MEM32) && (port->mapbase))
	{
		if (port->flags & UPF_IOREMAP)
		{
			iounmap(port->membase);
			port->membase = NULL;
		}

		release_mem_region(port->mapbase, SZ_4K);
	}
}

static int serial_lpc32xx_request_port(struct uart_port *port)
{
	int ret = -ENODEV;

	if ((port->iotype == UPIO_MEM32) && (port->mapbase))
	{
		ret = 0;

		if (!request_mem_region(port->mapbase, SZ_4K, MODNAME))
		{
			ret = -EBUSY;
		}
		else if (port->flags & UPF_IOREMAP)
		{
			port->membase = ioremap(port->mapbase, SZ_4K);
			if (!port->membase)
			{
				release_mem_region(port->mapbase, SZ_4K);
				ret = -ENOMEM;
			}
		}
	}

	return ret;
}

static void serial_lpc32xx_config_port(struct uart_port *port, int uflags)
{
	int ret;
	u32 tmp;

	/*
	 * Find the region that we can probe for.  This in turn
	 * tells us whether we can probe for the type of port.
	 */
	ret = serial_lpc32xx_request_port(port);
	if (ret < 0)
		return;
	port->type = PORT_UART00;
	port->fifosize = 64;

	/* Empty FIFO */
	__serial_uart_flush(port);

	/* Clear latched interrupt states */
	__raw_writel((HSU_TX_INT | HSU_FE_INT | HSU_BRK_INT | HSU_RX_OE_INT),
		HSUART_IIR(port->membase));

	/* Setup initial rate to slowest speed */
	__raw_writel(0xFF, HSUART_RATE(port->membase));

	/* Set receiver timeout, HSU offset of 20, no break, no interrupts,
	   and default FIFO trigger levels */
	tmp = HSU_TX_TL8B | HSU_RX_TL32B | HSU_OFFSET(20) | HSU_TMO_INACT_4B;
	__raw_writel(tmp, HSUART_CTRL(port->membase));
}

static int serial_lpc32xx_verify_port(struct uart_port *port,
	struct serial_struct *ser)
{
	int ret = 0;

	if (ser->type != PORT_UART00)
		ret = -EINVAL;

	return ret;
}

static struct uart_ops serial_lpc32xx_pops =
{
	.tx_empty	= serial_lpc32xx_tx_empty,
	.set_mctrl	= serial_lpc32xx_set_mctrl,
	.get_mctrl	= serial_lpc32xx_get_mctrl,
	.stop_tx	= serial_lpc32xx_stop_tx,
	.start_tx	= serial_lpc32xx_start_tx,
	.stop_rx	= serial_lpc32xx_stop_rx,
	.enable_ms	= serial_lpc32xx_enable_ms,
	.break_ctl	= serial_lpc32xx_break_ctl,
	.startup	= serial_lpc32xx_startup,
	.shutdown	= serial_lpc32xx_shutdown,
	.set_termios	= serial_lpc32xx_set_termios,
	.type		= serial_lpc32xx_type,
	.release_port	= serial_lpc32xx_release_port,
	.request_port	= serial_lpc32xx_request_port,
	.config_port	= serial_lpc32xx_config_port,
	.verify_port	= serial_lpc32xx_verify_port,
};

/*
 * Register a set of serial devices attached to a platform device
 */
static int __devinit serial_hs_lpc32xx_probe(struct platform_device *pdev)
{
	struct uart_port *p = pdev->dev.platform_data;
	struct lpc32xx_hsuart_port *pdr;
	int i, ret = 0;

	uarts_registered = 0;
	for (i = 0; p && (p->flags != 0); i++)
	{
		pdr = &lpc32xx_hs_ports[i];
		memset(pdr, 0, sizeof(struct lpc32xx_hsuart_port));

		pdr->port.iotype	= p->iotype;
		pdr->port.membase	= p->membase;
		pdr->port.mapbase	= p->mapbase;
		pdr->port.irq		= p->irq;
		pdr->port.uartclk	= p->uartclk;
		pdr->port.regshift	= p->regshift;
		pdr->port.flags		= p->flags | UPF_FIXED_PORT;
		pdr->port.dev		= &pdev->dev;
		pdr->port.ops		= &serial_lpc32xx_pops;
		pdr->port.line		= p->line;
		spin_lock_init(&pdr->port.lock);

		uart_add_one_port(&lpc32xx_hs_reg, &pdr->port);
		p++;
		uarts_registered++;
	}

	return ret;
}

/*
 * Remove serial ports registered against a platform device.
 */
static int __devexit serial_hs_lpc32xx_remove(struct platform_device *pdev)
{
	struct lpc32xx_hsuart_port *p;
	int i;

	for (i = 0; i < uarts_registered; i++)
	{
		p = &lpc32xx_hs_ports[i];

		if (p->port.dev == &pdev->dev)
		{
			uart_remove_one_port(&lpc32xx_hs_reg, &p->port);
		}
	}

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver serial_hs_lpc32xx_driver =
{
	.probe		= serial_hs_lpc32xx_probe,
	.remove		= __devexit_p(serial_hs_lpc32xx_remove),
	/* Suspend and resume are not needed, as the UART autoclocks */
	.driver		=
	{
		.name	= MODNAME,
		.owner	= THIS_MODULE,
	},
};

static int __init lpc32xx_hsuart_init(void)
{
	int ret;

	ret = uart_register_driver(&lpc32xx_hs_reg);
	if (ret == 0)
	{
		ret = platform_driver_register(&serial_hs_lpc32xx_driver);
		if (ret)
		{
			uart_unregister_driver(&lpc32xx_hs_reg);
		}
	}

	return ret;
}

static void __exit lpc32xx_hsuart_exit(void)
{
	platform_driver_unregister(&serial_hs_lpc32xx_driver);
	uart_unregister_driver(&lpc32xx_hs_reg);
}

module_init (lpc32xx_hsuart_init);
module_exit (lpc32xx_hsuart_exit);

MODULE_AUTHOR ("Kevin Wells (kevin.wells@nxp.com)");
MODULE_DESCRIPTION ("NXP LPC32XX High speed UART driver");
MODULE_LICENSE ("GPL");

