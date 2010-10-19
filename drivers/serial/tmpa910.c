/*
 *  linux/drivers/serial/tmpa910.c
 *
 *  Based on drivers/serial/8250.c by Russell King.
 *
 *  Copyright:	(C) 2008 Yin, Fengwei (fengwei.yin@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Note 1: This driver is made separate from the already too overloaded
 * 8250.c because it needs some kirks of its own and that'll make it
 * easier to add DMA support.
 *
 * Note 2: I'm too sick of device allocation policies for serial ports.
 * If someone else wants to request an "official" allocation of major/minor
 * for this driver please be my guest.  And don't forget that new hardware
 * to come from Intel might have more than 3 or 4 of those UARTs.  Let's
 * hope for a better port registration and dynamic device allocation scheme
 * with the serial core maintainer satisfaction to appear soon.
 */


#if defined(CONFIG_SERIAL_TMPA910_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>

#include <asm/io.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <mach/tmpa910_regs.h>
#include <mach/tmpa910_serial.h>

static int GetUartClock(void);

struct uart_tmpa910_port {
	struct uart_port        port;
	
	unsigned int		ecr;
	unsigned int		mcr;
	unsigned int		rsr;
	unsigned int		fr;
	unsigned int		fbrd;
	unsigned int		brd;
	unsigned int		lcr;
	unsigned int		cr;
	unsigned int		imsc;
	unsigned int		fls;
	unsigned int		icr;
	
	char			*name;
};

static struct uart_driver serial_tmpa910_reg;

static inline unsigned int serial_in(struct uart_tmpa910_port *up, int offset)
{
	return readl(up->port.membase + offset);
}

static inline void serial_out(struct uart_tmpa910_port *up, int offset, int value)
{
	writel(value, up->port.membase + offset);
}

static void serial_tmpa910_enable_ms(struct uart_port *port)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;

	up->imsc &= ~(UARTIMSC_DSRMIM | UARTIMSC_DCDMIM |
		UARTIMSC_CTSMIM | UARTIMSC_RIMIM);

	serial_out(up, UART_IMSC, up->imsc);
}

static void serial_tmpa910_stop_tx(struct uart_port *port)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;

	if (up->imsc & UARTIMSC_TXIM) {
		up->imsc &= ~UARTIMSC_TXIM;
		serial_out(up, UART_IMSC, up->imsc);
	}
}

static void serial_tmpa910_stop_rx(struct uart_port *port)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;

	/* We can disable TXE in CR when stop rx because we know we
	 * don't have more charator to receive (or we don't care).
	 */
	up->cr &= ~UART_CR_RXE;
	serial_out(up, UART_CR, up->cr);

	up->imsc &= ~(UARTIMSC_RTIM | UARTIMSC_RXIM);
	serial_out(up, UART_IMSC, up->imsc);
}

void receive_chars(struct uart_tmpa910_port *up)
{
	struct tty_struct *tty = up->port.state->port.tty;
	unsigned int ch, flag, status, fr;
	int max_count = 256;
	fr = serial_in(up, UART_FR);

	do {
		ch = serial_in(up, UART_DR);
		status = ch & 0xF00;
		ch &= 0xFF;
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(status & (UART_DR_BE | UART_DR_PE |
				       UART_DR_FE | UART_DR_OE))) {
			/*
			 * For statistics only
			 */
			if (status & UART_DR_BE) {
				status &= ~(UART_DR_FE | UART_DR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (status & UART_DR_PE)
				up->port.icount.parity++;
			else if (status & UART_DR_FE)
				up->port.icount.frame++;
			if (status & UART_DR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			status &= up->port.read_status_mask;

			if (status & UART_DR_BE) {
				flag = TTY_BREAK;
			} else if (status & UART_DR_PE)
				flag = TTY_PARITY;
			else if (status & UART_DR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;

		uart_insert_char(&up->port, status, UART_DR_OE, ch, flag);
		

	ignore_char:
		fr = serial_in(up, UART_FR);
	} while ((!(fr & UART_FR_RXFE)) && (max_count-- > 0));

	tty_flip_buffer_push(tty);
}

void transmit_chars(struct uart_tmpa910_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;

	if (up->port.x_char) {
		serial_out(up, UART_DR, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		serial_tmpa910_stop_tx(&up->port);
		return;
	}

	/* Try to write data to FIFO till TX FIFO is full or not more data */
	while (!(serial_in(up, UART_FR) & UART_FR_TXFF)) {
		serial_out(up, UART_DR, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);


	if (uart_circ_empty(xmit))
		serial_tmpa910_stop_tx(&up->port);
}

static void serial_tmpa910_start_tx(struct uart_port *port)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;

	/* We can't stop TXE in CR here because maybe there are some
	 * charater still in TX FIFO. We need make sure that they will
	 * be sent out. So just disable TX IRQ.
	 */
	if (!(up->imsc & UARTIMSC_TXIM)) {
		up->imsc |= UARTIMSC_TXIM;
		serial_out(up, UART_IMSC, up->imsc);
	}

	transmit_chars(up);
}

void check_modem_status(struct uart_tmpa910_port *up, u32 status)
{
	if ((status & UART_MIS_ANY_DELTA) == 0)
		return;

	if (status & UART_MIS_RI)
		up->port.icount.rng++;
	if (status & UART_MIS_DSR)
		up->port.icount.dsr++;
	if (status & UART_MIS_DCD)
		uart_handle_dcd_change(&up->port, status & UART_MIS_DCD);
	if (status & UART_MIS_CTS)
		uart_handle_cts_change(&up->port, status & UART_MIS_CTS);

	wake_up_interruptible(&up->port.state->port.delta_msr_wait);
}

/*
 * This handles the interrupt from one port.
 */
static inline irqreturn_t serial_tmpa910_irq(int irq, void *dev_id)
{
	struct uart_tmpa910_port *up = dev_id;
	unsigned int ris, mis;

	ris = serial_in(up, UART_RIS) & 0x3ff;
	mis = serial_in(up, UART_MIS);

	serial_out(up, UART_ICR, ris);	/* Clear the interrupt */

	if (!mis)	
		return IRQ_NONE;

	if (mis & (UART_RIS_RX | UART_RIS_RT))
		receive_chars(up);
	
	check_modem_status(up, mis);

	if (mis & UART_RIS_TX)	
		transmit_chars(up);

	return IRQ_HANDLED;
}

static unsigned int serial_tmpa910_tx_empty(struct uart_port *port)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&up->port.lock, flags);
	ret = serial_in(up, UART_FR) & UART_FR_TXFE ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return ret;
}

static unsigned int serial_tmpa910_get_mctrl(struct uart_port *port)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;
	unsigned int cr;
	unsigned int ret;

	cr = serial_in(up, UART_CR);

	ret = 0;
	if (cr & UART_CR_RTS)
		ret |= TIOCM_RTS;
	if (cr & UART_CR_DTR)
		ret |= TIOCM_DTR;

	return ret;
}

static void serial_tmpa910_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;
	unsigned int cr = 0;

	if (mctrl & TIOCM_RTS)
		cr |= UART_CR_RTS;
	if (mctrl & TIOCM_DTR)
		cr |= UART_CR_DTR;
		
/*
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;
*/

	up->cr |= cr;

	serial_out(up, UART_CR, up->cr);
}

static void serial_tmpa910_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static int serial_tmpa910_startup(struct uart_port *port)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;
	unsigned long flags;
	int retval;

	up->mcr = 0;

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(up->port.irq, serial_tmpa910_irq, 0, up->name, up);
	if (retval)
		return retval;

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	up->lcr = serial_in(up, UART_LCR);
	up->lcr &= ~UART_LCR_FEN;
	serial_out(up, UART_LCR, up->lcr);

	/*
	 * Clear the interrupt registers.
	 */
	up->icr = 0x3ff;
	serial_out(up, UART_ICR, up->icr);

	/* Clear receive status */
	serial_out(up, UART_ECR, 0);
	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&up->port.lock, flags);
	/* Enable UART, RX, TX */
	up->cr = (UART_CR_TXE | UART_CR_RXE | UART_CR_UARTEN);
	serial_out(up, UART_CR, up->cr);
	serial_tmpa910_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	up->imsc = (UARTIMSC_RTIM | UARTIMSC_TXIM | UARTIMSC_RXIM);
	serial_out(up, UART_IMSC, up->imsc);

	/*
	 * And clear the interrupt registers again for luck.
	 */
	up->icr = 0x3ff;
	serial_out(up, UART_ICR, up->icr);

	return 0;
}

static void serial_tmpa910_shutdown(struct uart_port *port)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;
	unsigned long flags;

	free_irq(up->port.irq, up);

	/*
	 * Disable interrupts from this port
	 */
	spin_lock_irqsave(&up->port.lock, flags);
	up->imsc = 0;
	serial_out(up, UART_IMSC, up->imsc);

	/* Disable TX, RX and UART */
	up->cr = 0;
	serial_out(up, UART_CR, up->cr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static void
serial_tmpa910_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;
	unsigned int lcr = 0, brd = 0, fbrd = 0;
	unsigned long flags;
	unsigned int baud;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = UART_LCR_WLEN5;
		break;
	case CS6:
		lcr = UART_LCR_WLEN6;
		break;
	case CS7:
		lcr = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		lcr = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		lcr |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		lcr |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		lcr |= UART_LCR_EPAR;

	/* Get the baud. Max baudrate is 460800 */
	baud = uart_get_baud_rate(port, termios, old, 0, 460800);
	
	/* 
	 * brd = f-uart / (16 * baud)
	 * fbrd = (reminder(f-uart/(16 * baud)) *64) / (16 * baud)
	 */
	brd = port->uartclk >> 4; /* (f-uart / 16) */
	fbrd = do_div(brd, baud);
	fbrd <<= 10;
	do_div(fbrd, baud);
	fbrd <<= 6;
	fbrd >>= 10;

	up->cr = serial_in(up, UART_CR);

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Ensure the port will be enabled.
	 * This is required especially for serial console.
	 */
	up->cr |= UART_CR_UARTEN;

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_DR_FE | UART_DR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_DR_BE;

	/*
	 * Characters to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_DR_PE | UART_DR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_DR_BE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_DR_OE;
	}

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->imsc &= ~(UARTIMSC_DSRMIM | UARTIMSC_DCDMIM |
		UARTIMSC_CTSMIM | UARTIMSC_RIMIM);

	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->imsc |= (UARTIMSC_DSRMIM | UARTIMSC_DCDMIM |
			UARTIMSC_CTSMIM | UARTIMSC_RIMIM);

	serial_out(up, UART_IMSC, up->imsc);

	serial_out(up, UART_IBRD, brd);
	serial_out(up, UART_FBRD, fbrd);	
	up->brd = brd;
	up->fbrd = fbrd;
	
	serial_tmpa910_set_mctrl(&up->port, up->port.mctrl);

	/* 
	 * RX FIFO threshold: 9
	 * TX FIFO threshold: 8
	 */
	up->fls = (2 << 3) | (0x02);	
	serial_out(up, UART_FLS, up->fls);
	up->lcr = lcr | UART_LCR_FEN;
	serial_out(up, UART_LCR, up->lcr);
	serial_out(up, UART_CR, up->cr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static void serial_tmpa910_release_port(struct uart_port *port)
{
	return;
}

static int serial_tmpa910_request_port(struct uart_port *port)
{
	return 0;
}

static void serial_tmpa910_config_port(struct uart_port *port, int flags)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;
	up->port.type = PORT_TMPA910;
}

static int
serial_tmpa910_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	return -EINVAL;
}

static const char *serial_tmpa910_type(struct uart_port *port)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;
	return up->name;
}


struct uart_ops serial_tmpa910_pops = {
	.tx_empty	= serial_tmpa910_tx_empty,
	.set_mctrl	= serial_tmpa910_set_mctrl,
	.get_mctrl	= serial_tmpa910_get_mctrl,
	.stop_tx	= serial_tmpa910_stop_tx,
	.start_tx	= serial_tmpa910_start_tx,
	.stop_rx	= serial_tmpa910_stop_rx,
	.enable_ms	= serial_tmpa910_enable_ms,
	.break_ctl	= serial_tmpa910_break_ctl,
	.startup	= serial_tmpa910_startup,
	.shutdown	= serial_tmpa910_shutdown,
	.set_termios	= serial_tmpa910_set_termios,
	.type		= serial_tmpa910_type,
	.release_port	= serial_tmpa910_release_port,
	.request_port	= serial_tmpa910_request_port,
	.config_port	= serial_tmpa910_config_port,
	.verify_port	= serial_tmpa910_verify_port,
};


static struct uart_tmpa910_port serial_tmpa910_ports[] = {
	{	/* UART 0 */
		.name	= "UART 0",
		.port	= {
			.type		= PORT_TMPA910,
			.iotype		= UPIO_MEM,
			.membase	= (void *)&UART0DR,
			.mapbase	= __PREG(UART0DR),
			.irq		= INTR_VECT_UART_CH0,
			.uartclk	= 96000000,
			.fifosize	= 16,
			.ops		= &serial_tmpa910_pops,
			.line		= 0,
		},
	}, 
	{	/* UART 1 */
		.name	= "UART 1",
		.port	= {
			.type		= PORT_TMPA910,
			.iotype		= UPIO_MEM,
			.membase	= (void *)&UART1DR,
			.mapbase	= __PREG(UART1DR),
			.irq		= INTR_VECT_UART_CH1,
			.uartclk	= 96000000, 
			.fifosize	= 16,
			.ops		= &serial_tmpa910_pops,
			.line		= 1,
		},
	}, 
#ifdef CONFIG_CPU_TMPA900
	{	/* UART 2 */
		.name	= "UART 2",
		.port	= {
			.type		= PORT_TMPA910,
			.iotype		= UPIO_MEM,
			.membase	= (void *)&UART2DR,
			.mapbase	= __PREG(UART2DR),
			.irq		= INTR_VECT_UART_CH2,
			.uartclk	= 96000000, 
			.fifosize	= 16,
			.ops		= &serial_tmpa910_pops,
			.line		= 2,
		},
	}
#endif
};



#ifdef CONFIG_SERIAL_TMPA910_CONSOLE
/*
 *	Wait for transmitter & holding register to empty
 */
void wait_for_xmitr(struct uart_tmpa910_port *up)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	do {
		status = serial_in(up, UART_FR);

		if (--tmout == 0)
			break;
		udelay(1);
	} while (!(status & UART_FR_TXFE));
}

void serial_tmpa910_console_putchar(struct uart_port *port, int ch)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;

	wait_for_xmitr(up);
	serial_out(up, UART_DR, ch);
}

/*
 * Print a string to the serial port trying not to disturb
 * any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
void serial_tmpa910_console_write(struct console *co,
		const char *s, unsigned int count)
{
	struct uart_tmpa910_port *up = &serial_tmpa910_ports[co->index];
	unsigned int int_mask, uart_cr;

	/*
	 * Save uart int mask and disable the interrupts
	 */
	int_mask = serial_in(up, UART_IMSC);
	serial_out(up, UART_IMSC, 0x7FF);

	uart_cr = serial_in(up, UART_CR);
	serial_out(up, UART_CR, uart_cr | 0x101);	/* Enable UART, UART TX */

	uart_console_write(&up->port, s, count, serial_tmpa910_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the INT MASK
	 */
	wait_for_xmitr(up);
	serial_out(up, UART_IMSC, int_mask);
	serial_out(up, UART_CR, uart_cr);
}

static int __init serial_tmpa910_console_setup(struct console *co, char *options)
{
	struct uart_tmpa910_port *up;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int getclk;

    getclk = GetUartClock();

	if (co->index == -1 || co->index >= serial_tmpa910_reg.nr)
		co->index = 1;

    if(getclk > 0) {
        serial_tmpa910_ports[0].port.uartclk = getclk * 1000000;
        serial_tmpa910_ports[1].port.uartclk = getclk * 1000000;
    }

    up = &serial_tmpa910_ports[co->index];
    
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&up->port, co, baud, parity, bits, flow);
}

static struct console serial_tmpa910_console = {
	.name		= "ttyS",
	.write		= serial_tmpa910_console_write,
	.device		= uart_console_device,
	.setup		= serial_tmpa910_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial_tmpa910_reg,
};

static int __init
serial_tmpa910_console_init(void)
{
#ifdef CONFIG_SERIAL_TMPA910_CONSOLE_PREFERED
  	add_preferred_console("ttyS", 0, NULL);
	printk("Welcome. Myself as prefered console :-)\n");
#else
	printk("Welcome\n");
#endif
	
	register_console(&serial_tmpa910_console);
	return 0;
}

console_initcall(serial_tmpa910_console_init);

#define TMPA910_CONSOLE	&serial_tmpa910_console
#else
#define	TMPA910_CONSOLE		NULL
#endif

static struct uart_driver serial_tmpa910_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= "TMPA9xx serial",
	.dev_name	= "ttyS",
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= ARRAY_SIZE(serial_tmpa910_ports),
	.cons		= TMPA910_CONSOLE,
};

static int serial_tmpa910_probe(struct platform_device *dev)
{
    serial_tmpa910_ports[dev->id].port.dev = &dev->dev;
    uart_add_one_port(&serial_tmpa910_reg, &serial_tmpa910_ports[dev->id].port);
	platform_set_drvdata(dev, &serial_tmpa910_ports[dev->id]);
	return 0;
}

static int serial_tmpa910_remove(struct platform_device *dev)
{
	struct uart_tmpa910_port *sport = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	if (sport)
		uart_remove_one_port(&serial_tmpa910_reg, &sport->port);

	return 0;
}

static struct platform_driver serial_tmpa910_driver = {
        .probe          = serial_tmpa910_probe,
        .remove         = serial_tmpa910_remove,

	.driver		= {
	        .name	= "tmpa910-uart",
	},
};

int __init serial_tmpa910_init(void)
{
	int ret;

	ret = uart_register_driver(&serial_tmpa910_reg);
	if (ret != 0)
		return ret;

	ret = platform_driver_register(&serial_tmpa910_driver);
	if (ret != 0)
		uart_unregister_driver(&serial_tmpa910_reg);

	return ret;
}

void __exit serial_tmpa910_exit(void)
{
	platform_driver_unregister(&serial_tmpa910_driver);
	uart_unregister_driver(&serial_tmpa910_reg);
}

static int GetUartClock(void)
{
	unsigned char clock;
	clock = -1;
	if (SYSCR3&PLLON)
	{
		if ((SYSCR3&PLLVALUE)==PLL_6)
		{
			clock = UARTCLK_72MHz;
			if ((SYSCR1&SYS_GEAR) == GEAR_2)
			{
				clock = UARTCLK_36MHz;
			}
			else if ((SYSCR1&SYS_GEAR) == GEAR_4)
			{
				clock = UARTCLK_18MHz;
			}
			else if ((SYSCR1&SYS_GEAR) == GEAR_8)
			{
				clock = UARTCLK_9MHz;
			}		
		}
		else if((SYSCR3&PLLVALUE)==PLL_8)
		{
			clock = UARTCLK_96MHz;
			if ((SYSCR1&SYS_GEAR) == GEAR_2)
			{
				clock = UARTCLK_48MHz;
			}
			else if ((SYSCR1&SYS_GEAR) == GEAR_4)
			{
				clock = UARTCLK_24MHz;
			}
			else if ((SYSCR1&SYS_GEAR) == GEAR_8)
			{
				clock = UARTCLK_12MHz;
			}
		}
		else
		{
			clock = -1;
			//error
		}
	}
	else
	{
		clock = -1;
		//error
	}
	return clock;
}

module_init(serial_tmpa910_init);
module_exit(serial_tmpa910_exit);

MODULE_LICENSE("GPL");

