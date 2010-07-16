/*
 *  linux/drivers/serial/tmpa910.c
 *
 *  Based on drivers/serial/8250.c by Russell King.
 *
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
 *  Copyright:	(C) 2008 Yin, Fengwei (fengwei.yin@gmail.com)
=======
 *  Copyright 2008 bplan GmbH
 *  Copyright 2010 Toshiba Electronics Europe GmbH
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
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
=======
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
 */

#if defined(CONFIG_SERIAL_TMPA910_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
	#define SUPPORT_SYSRQ
#endif

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
#if defined(CONFIG_SERIAL_TMPA910_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

=======
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
=======
#include <linux/ioport.h>
>>>>>>> Stashed changes:drivers/serial/tmpa910.c

#include <asm/io.h>
#include <mach/hardware.h>
#include <asm/irq.h>
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
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
=======

#define DRIVER_NAME  	"tmpa910_uart"

#define TMPA910_UART_REGSIZE 0x50
#define TMPA910_SERIAL_MAX 2
#define TMPA910_NAME_PATTERN  "TMPA910 UART Channel %d"

struct tmpa910_uart_regs
{
  uint32_t dr;      // 0x000
  uint32_t sr;      // 0x004
  uint32_t ecr;     // 0x004
  uint32_t rsd1[3]; // 0x008 -> 0x014
  uint32_t fr;      // 0x018
  uint32_t rsd2;    // 0x01c
  uint32_t ilpr;    // 0x020
  uint32_t ibrd;    // 0x024
  uint32_t fbrd;    // 0x028
  uint32_t lcr_h;   // 0x02c
  uint32_t cr;      // 0x030
  uint32_t ifls;    // 0x034
  uint32_t imsc;    // 0x038
  uint32_t ris;     // 0x03c
  uint32_t mis;     // 0x040
  uint32_t icr;     // 0x044
  uint32_t dmacr;   // 0x048
};

/* The data register contains 8bit for the char and 4 status bits */

#define DR_OE  (1<<11)  /* OE RO Undefined Overrun error
                           0y0: There is an empty space in the FIFO.
                           0y1: Overrun error flag */
#define DR_BE  (1<<10)  /* BE RO Undefined Break error
                           0y0: No error detected
                           0y1: Error detected */
#define DR_PE  (1<<9)   /* PE RO Undefined Parity error
                           0y0: No error detected
                           0y1: Error detected */
#define DR_FE  (1<<8)   /* FE RO Undefined Framing error
                           0y0: No error detected
                           0y1: Error detected */

/* The data register contains 8bit for the char and 4 status bits */
#define SR_OE  (1<<3)  
#define SR_BE  (1<<2) 
#define SR_PE  (1<<1) 
#define SR_FE  (1<<0)

#define LCRH_SPS      (1<<7)   SPS  R/W 0y0  Stick parity select:
#define LCRH_WLEN_5B  (0x0)
#define LCRH_WLEN_6B  (0x1<<5)
#define LCRH_WLEN_7B  (0x2<<5)
#define LCRH_WLEN_8B  (0x3<<5)
#define LCRH_FEN      (1<<4)  /* R/W 0y0  FIFO control
                                 0y1: FIFO mode
                                 0y0: Character mode */
#define LCRH_STP2     (1<<3)  /* STP2 R/W 0y0  Stop bit select
                                 0y0: 1 stop bit
                                 0y1: 2 stop bits */
#define LCRH_EPS      (1<<2)  /* EPS  R/W 0y0  Even parity select (Refer to Table 3.13.1 for
                                 the truth table.)
                                 0y1: Even
                                 0y0: Odd */
#define LCRH_PEN      (1<<1)  /* PEN  R/W 0y0  Parity control (Refer to Table 3.13.1 for the
                                 truth table.)
                                 0y0: Disable
                                 0y1: Enable */
#define LCRH_BRK      (1<<0)  /* BRK  R/W 0y0  Send break
                                 0y0: No effect
                                 0y1: Send break */

#define CR_CTSEn (1<<15)      /* CTSEn  R/W 0y0       CTS hardware flow control enable */
#define CR_RTSEn (1<<14)      /* RTSEn  R/W 0y0       RTS hardware flow control enable */

#define CR_RTS (1<<11)        /* R/W 0y0       Complement of the UART Request To Send
                                 (nUARTRTS) modem status output
                                 0y0: Modem status output is "1".
                                 0y1: Modem status output is "0". */
#define CR_DTR (1<<10)        /*  R/W 0y0       Complement of the UART Data Set Ready
                                 (nUARTDTR) modem status output
                                 0y0: Modem status output is "1".
                                 0y1: Modem status output is "0". */
#define CR_RXE (1<<9)         /* RXE    R/W 0y1       UART receive enable */
#define CR_TXE (1<<8)         /* TXE    R/W 0y1       UART transmit enable */

#define CR_SIRLP (1<<2)       /* SIRLP  R/W 0y0       IrDA encoding mode select for transmitting "0"
                                 0y0: "0" bits are transmitted as an active high
                                 pulse of 3/16th of the bit period.
                                 0y1:"0" bits are transmitted with a pulse width
                                 that is 3 times the period of the IrLPBaud16
                                 input signal. */
			      
#define CR_SIREN (1<<1)       /* SIREN  R/W 0y0       SIR enable */
#define CR_UARTEN (1<<0)      /* UARTEN R/W 0y0       UART enable */

/* Flag register */

#define FR_RI         (1<<8) // Ring indicator flag
#define FR_TXFE       (1<<7) // Transmit FIFO empty flag
#define FR_RXFF       (1<<6) // Receive FIFO full flag
#define FR_TXFF       (1<<5) // Transmit FIFO full flag
#define FR_RXFE       (1<<4) // Receive FIFO empty flag
#define FR_BUSY       (1<<3) // Busy flag
#define FR_DCD        (1<<2) // Data carrier detect flag
#define FR_DSR        (1<<1) // Data set ready flag
#define FR_CTS        (1<<0) // Clear To Send flag

/* for interrupt mask and status
   0y0: Clear the mask
   0y1: Set the mask */
		    
#define INT_OE      (1<<10)  // OEIM   R/W 0y0 Overrun error 
#define INT_BE      (1<<9)   // BEIM   R/W 0y0 Break error 
#define INT_BEIM    (1<<8)   // PEIM   R/W 0y0 Parity error 
#define INT_FEIM    (1<<7)   // FEIM   R/W 0y0 Framing error 
#define INT_RTIM    (1<<6)   // RTIM   R/W 0y0 Receive timeout 
#define INT_TX      (1<<5)   // TXIM   R/W 0y0 Transmit interrupt
#define INT_RX      (1<<4)   // RXIM   R/W 0y0 Receive interrupt
#define INT_DSRM    (1<<3)   // DSRMIM R/W 0y0 U0DSRn modem interrupt
#define INT_DCDM    (1<<2)   // DCDMIM R/W 0y0 U0DCDn modem interrupt
#define INT_CTSM    (1<<1)   // CTSMIM R/W 0y0 U0CTSn modem interrupt
#define INT_RIM     (1<<0)   // RIMIM  R/W 0y0 U0RIn modem interrupt

struct uart_tmpa910_handle {
  struct uart_port        port;
  int channel;
  int irq_allocated;

  volatile struct tmpa910_uart_regs *regs;
  char name[sizeof(TMPA910_NAME_PATTERN)];
};

static struct uart_tmpa910_handle serial_ports[TMPA910_SERIAL_MAX];

static int _fill_uarthandle(
	struct uart_tmpa910_handle *uart_tmpa910_handle,
	int index);

static inline void wait_for_xmitr(struct uart_tmpa910_handle *uart_tmpa910_handle);
static inline void wait_for_txempty(struct uart_tmpa910_handle *uart_tmpa910_handle);

static int _get_uartclk(struct uart_tmpa910_handle *uart_tmpa910_handle)
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
{
	return readl(up->port.membase + offset);
}

static inline void serial_out(struct uart_tmpa910_port *up, int offset, int value)
{
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
	writel(value, up->port.membase + offset);
}

static void serial_tmpa910_enable_ms(struct uart_port *port)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;

	up->imsc &= ~(UARTIMSC_DSRMIM | UARTIMSC_DCDMIM |
		UARTIMSC_CTSMIM | UARTIMSC_RIMIM);

	serial_out(up, UART_IMSC, up->imsc);
=======
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	
	printk("port at 0x%p / 0x%lx\n", regs, (unsigned long) uart_tmpa910_handle->port.mapbase);
	
	if (regs == NULL)
		return;
		
	printk("dr    at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, dr),    regs->dr);
	printk("sr    at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, sr),    regs->sr);
	printk("ecr   at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, ecr),   regs->ecr);
	printk("fr    at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, fr),    regs->fr);
	printk("ilpr  at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, ilpr),  regs->ilpr);
	printk("ibrd  at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, ibrd),  regs->ibrd);
	printk("fbrd  at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, fbrd),  regs->fbrd);
	printk("lcr_h at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, lcr_h), regs->lcr_h);
	printk("cr    at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, cr),    regs->cr);
	printk("ifls  at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, ifls),  regs->ifls);
	printk("imsc  at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, imsc),  regs->imsc);
	printk("ris   at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, ris),   regs->ris);
	printk("mis   at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, mis),   regs->mis);
	printk("icr   at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, icr),   regs->icr);
	printk("dmacr at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, dmacr), regs->dmacr);
}
#endif

static int _map_tmpa910(struct uart_tmpa910_handle *uart_tmpa910_handle)
{
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	
	if (regs) {
		printk(KERN_ERR "port already requested or mapped (regs=0x%p)\n", regs);
		return 0;
	}
	
	regs = (struct tmpa910_uart_regs *)uart_tmpa910_handle->port.mapbase;
	if (regs == NULL) {
		printk(KERN_ERR "Fail to map UART controller (mapbase=0x%x)\n", uart_tmpa910_handle->port.mapbase);
		return -EINVAL;
	}
	
	uart_tmpa910_handle->port.membase = (void *) regs;
	uart_tmpa910_handle->regs = regs;

	
#ifdef __DEBUG__
	_dump_regs(uart_tmpa910_handle);
#endif
	
	return 0;
}

static void serial_tmpa910_enable_ms(struct uart_port *port)
{
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
}

static void serial_tmpa910_stop_tx(struct uart_port *port)
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;

	if (up->imsc & UARTIMSC_TXIM) {
		up->imsc &= ~UARTIMSC_TXIM;
		serial_out(up, UART_IMSC, up->imsc);
	}
}

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
static void serial_tmpa910_stop_rx(struct uart_port *port)
=======
static void transmit_chars(struct uart_tmpa910_handle *uart_tmpa910_handle)
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
{
	struct uart_tmpa910_port *up = (struct uart_tmpa910_port *)port;

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
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

=======
	if (uart_circ_empty(xmit))
		serial_tmpa910_stop_tx(&uart_tmpa910_handle->port);
}

static inline void
receive_chars(struct uart_tmpa910_handle *uart_tmpa910_handle)
{
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	struct uart_port *port = &uart_tmpa910_handle->port;
	struct tty_struct *tty = port->state->port.tty;
	unsigned int ch, flag;
	uint32_t fr_reg;
	uint32_t dr_reg;
	int max_count = 256;
	int ret;
	
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
	do {
		ch = serial_in(up, UART_DR);
		status = ch & 0xF00;
		ch &= 0xFF;
		flag = TTY_NORMAL;
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
		up->port.icount.rx++;

		if (unlikely(status & (UART_DR_BE | UART_DR_PE |
				       UART_DR_FE | UART_DR_OE))) {
			/*
			 * For statistics only
			 */
			if (status & UART_DR_BE) {
				status &= ~(UART_DR_FE | UART_DR_PE);
				up->port.icount.brk++;
=======
		
		port->icount.rx++;

		if (unlikely(dr_reg & (DR_BE | DR_PE |
				       DR_FE | DR_OE))) {
			/*
			 * For statistics only
			 */
			if (dr_reg & DR_BE) {
				port->icount.brk++;
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (status & UART_DR_PE)
				up->port.icount.parity++;
			else if (status & UART_DR_FE)
				up->port.icount.frame++;
			if (status & UART_DR_OE)
				up->port.icount.overrun++;
=======
				ret = uart_handle_break(port);
				if(ret)
					goto ignore_char;
			} else if (dr_reg & DR_PE)
				port->icount.parity++;
			else if (dr_reg &DR_FE)
				port->icount.frame++;
			if (dr_reg & DR_OE)
				port->icount.overrun++;
>>>>>>> Stashed changes:drivers/serial/tmpa910.c

			/*
			 * Mask off conditions which should be ignored.
			 */
			status &= up->port.read_status_mask;

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
			if (status & UART_DR_BE) {
=======
#if defined(CONFIG_SERIAL_TMPA910_CONSOLE) & 0
			if (up->port.line == up->port.cons->index) {
				/* Recover the break flag from console xmit */
				*status |= up->lsr_break_flag;
				uart_tmpa910_handle->lsr_break_flag = 0;
			}
#endif
			if (dr_reg & DR_BE) {
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
				flag = TTY_BREAK;
			} else if (status & UART_DR_PE)
				flag = TTY_PARITY;
			else if (status & UART_DR_FE)
				flag = TTY_FRAME;
		}

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
		if (uart_handle_sysrq_char(&up->port, ch))
=======
		ret = uart_handle_sysrq_char(&uart_tmpa910_handle->port, ch);
		if (ret)
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
			goto ignore_char;

		uart_insert_char(&up->port, status, UART_DR_OE, ch, flag);
		

	ignore_char:
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
		fr = serial_in(up, UART_FR);
	} while ((!(fr & UART_FR_RXFE)) && (max_count-- > 0));
=======
		fr_reg = regs->fr; 
	} while (((fr_reg & FR_RXFE)==0) && (max_count-- > 0));
>>>>>>> Stashed changes:drivers/serial/tmpa910.c

	tty_flip_buffer_push(tty);
}

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
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
=======
static unsigned int serial_tmpa910_tx_empty(struct uart_port *port);
static void serial_tmpa910_start_tx(struct uart_port *port)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	struct circ_buf *xmit = &uart_tmpa910_handle->port.state->xmit;
	
	regs->imsc |= INT_TX;
	
	if (uart_circ_chars_pending(xmit))
	{
		wait_for_xmitr(uart_tmpa910_handle);
		transmit_chars(uart_tmpa910_handle);
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
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
=======
	/* rng is ring ? not documented.. */
	if (fr_reg & FR_RI)
		uart_tmpa910_handle->port.icount.rng++;
	if (fr_reg & FR_DSR)
		uart_tmpa910_handle->port.icount.dsr++;
	if (fr_reg & FR_DCD)
		uart_tmpa910_handle->port.icount.dcd++;
	if (fr_reg & FR_CTS)
		uart_tmpa910_handle->port.icount.cts++;

	wake_up_interruptible(&uart_tmpa910_handle->port.state->port.delta_msr_wait);
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
}

/*
 * This handles the interrupt from one port.
 */
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
static inline irqreturn_t serial_tmpa910_irq(int irq, void *dev_id)
=======
static unsigned int serial_tmpa910_tx_empty(struct uart_port *port)
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
	up->cr |= cr;

	serial_out(up, UART_CR, up->cr);
=======
	regs->cr |= cr_reg;
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(up->port.irq, serial_tmpa910_irq, 0, up->name, up);
	if (retval)
		return retval;
=======
	/* Make sure the port is off and no IRQ could be generated */
	regs->cr   = 0;
	regs->imsc   = 0;
	regs->icr    = 0x7ff;
	
	/* Clear errors if any */
	regs->ecr    = 0x0;
	
	/*
	 * Allocate the IRQ
	 */
	ret = request_irq(uart_tmpa910_handle->port.irq, serial_tmpa910_irq, 0, uart_tmpa910_handle->name, uart_tmpa910_handle);
	if (ret) {
		printk(KERN_ERR "TMPA910 UART: Fail allocate the interrupt (vector=%d)\n", uart_tmpa910_handle->port.irq);
		return ret;
	}

	uart_tmpa910_handle->irq_allocated = 1;
>>>>>>> Stashed changes:drivers/serial/tmpa910.c

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
	up->lcr = serial_in(up, UART_LCR);
	up->lcr &= ~UART_LCR_FEN;
	serial_out(up, UART_LCR, up->lcr);
=======
>>>>>>> Stashed changes:drivers/serial/tmpa910.c

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
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
	spin_lock_irqsave(&up->port.lock, flags);
	up->imsc = 0;
	serial_out(up, UART_IMSC, up->imsc);

	/* Disable TX, RX and UART */
	up->cr = 0;
	serial_out(up, UART_CR, up->cr);
	spin_unlock_irqrestore(&up->port.lock, flags);
=======
}

static inline int _get_div_fract(int uartclk, int baud, int div_integer)
{
	int a;
	
	/* Baud rate divisor = (UARTCLK)/(16 × baud)= integer part + fract part;
	  
	   When the required baud rate is 230400 and fUARTCLK = 4MHz:
	   Baud rate divisor = (4 × 106)/(16 × 230400)= 1.085
	   Therefore, fractional part is ((0.085 × 64)+ 0.5) = 5.94. */

	a = ((uartclk/16) *10) / (baud/100);
	a = a - div_integer*1000;
	a = a * 64 + 500;
	a = a / 1000;

	return a;
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
	/*
	 * Ensure the port will be enabled.
	 * This is required especially for serial console.
	 */
	up->cr |= UART_CR_UARTEN;

=======
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
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
=======
	//uart_tmpa910_handle->ier &= ~UART_IER_MSI;
	//if (UART_ENABLE_MS(&uart_tmpa910_handle->port, termios->c_cflag))
	//	uart_tmpa910_handle->ier |= UART_IER_MSI;

	serial_tmpa910_set_mctrl(&uart_tmpa910_handle->port, uart_tmpa910_handle->port.mctrl);

	regs->lcr_h = lcrh_reg;
	regs->cr   |= CR_UARTEN;
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
	
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
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
	return;
=======
	port->membase = NULL;
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
}

static int serial_tmpa910_request_port(struct uart_port *port)
{
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
	return 0;
=======
	return _map_tmpa910((struct uart_tmpa910_handle *) port);
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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

<<<<<<< Updated upstream:drivers/serial/tmpa910.c

=======
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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

<<<<<<< Updated upstream:drivers/serial/tmpa910.c

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
	}, {	/* UART 1 */
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
	}
};


=======
static int _fill_uarthandle(struct uart_tmpa910_handle *uart_tmpa910_handle, int index)
{
	unsigned long mapbase;
	int irq;

	irq = 10+index;
	mapbase = 0xf2000000 + 0x1000*index;

	snprintf(uart_tmpa910_handle->name, sizeof(uart_tmpa910_handle->name), TMPA910_NAME_PATTERN, index);

	uart_tmpa910_handle->channel		   = index;
	uart_tmpa910_handle->port.type  	 = PORT_TMPA910;
	uart_tmpa910_handle->port.iotype	 = UPIO_MEM;
	uart_tmpa910_handle->port.membase	 = NULL;
	uart_tmpa910_handle->port.mapbase	 = mapbase;
	uart_tmpa910_handle->port.irq		   = irq;
	uart_tmpa910_handle->port.uartclk	 = _get_uartclk(uart_tmpa910_handle);
	uart_tmpa910_handle->port.fifosize = 16;
	uart_tmpa910_handle->port.ops		   = &serial_tmpa910_pops;
	uart_tmpa910_handle->port.line		 = 0;

	return 0;
}
>>>>>>> Stashed changes:drivers/serial/tmpa910.c

#ifdef CONFIG_SERIAL_TMPA910_CONSOLE
/*
 *	Wait for transmitter & holding register to empty
 */
void wait_for_xmitr(struct uart_tmpa910_port *up)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
	do {
		status = serial_in(up, UART_FR);

=======
	while((regs->fr & FR_TXFF))
	{
		if (--tmout == 0)
			break;
			
		udelay(1);
	}

	/* Wait up to 1s for flow control if necessary */
	if (uart_tmpa910_handle->port.flags & UPF_CONS_FLOW) {
		tmout = 1000000;
		while (--tmout &&
		       ((regs->fr & FR_CTS) == 0))
			udelay(1);
	}
	
}

/*
 * Wait until the TX FIFO is totally empty
 */
static inline void wait_for_txempty(struct uart_tmpa910_handle *uart_tmpa910_handle)
{
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	unsigned int tmout = 10000;
	
	/* Wait up to 10ms for the character(s) to be sent. */
	while((regs->fr & FR_TXFE) == 0)
	{
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
		if (--tmout == 0)
			break;
		udelay(1);
	} while (!(status & UART_FR_TXFE));
}

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
void serial_tmpa910_console_putchar(struct uart_port *port, int ch)
=======
#ifdef CONFIG_SERIAL_TMPA910_CONSOLE

static struct uart_driver serial_tmpa910_reg;

static void serial_tmpa910_console_putchar(struct uart_port *port, int ch)
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
	wait_for_xmitr(up);
	serial_out(up, UART_IMSC, int_mask);
	serial_out(up, UART_CR, uart_cr);
}

static int __init serial_tmpa910_console_setup(struct console *co, char *options)
=======
	regs->imsc = imsc_reg;
}

static int __init
serial_tmpa910_console_setup(struct console *co, char *options)
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
{
	struct uart_tmpa910_port *up;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
    int getclk;

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
    getclk = GetUartClock();

	if (co->index == -1 || co->index >= serial_tmpa910_reg.nr)
		co->index = 1;

    if(getclk > 0) {
        serial_tmpa910_ports[0].port.uartclk = getclk*1000000;
        serial_tmpa910_ports[1].port.uartclk = getclk*1000000;
    }

    up = &serial_tmpa910_ports[co->index];
    
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&up->port, co, baud, parity, bits, flow);
=======
	//co->index = 1;

	spin_lock_init(&port->lock);	
	
	uart_tmpa910_handle = &serial_ports[co->index];
	port = &uart_tmpa910_handle->port;

	ret = _fill_uarthandle(uart_tmpa910_handle, co->index);
	if (ret<0) {
		return ret;
	}

	ret = _map_tmpa910(uart_tmpa910_handle);
	if(ret < 0) {
		printk(KERN_ERR "Fail to map IO mem. ret=%d\n", ret);
		return ret;
	}
	
	
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
		
	return uart_set_options(&uart_tmpa910_handle->port, co, baud, parity, bits, flow);
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
	printk("Welcome. Myself as prefered console :-)\n");
#else
	printk("Welcome\n");
=======
	printk(DRIVER_NAME ": Welcome. Myself as prefered console :-)\n");
#else
	printk(DRIVER_NAME ": Welcome\n");
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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
	.driver_name	= "TMPA910 serial",
	.dev_name	= "ttyS",
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= ARRAY_SIZE(serial_tmpa910_ports),
	.cons		= TMPA910_CONSOLE,
};

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
static int serial_tmpa910_probe(struct platform_device *dev)
{
    serial_tmpa910_ports[dev->id].port.dev = &dev->dev;
    uart_add_one_port(&serial_tmpa910_reg, &serial_tmpa910_ports[dev->id].port);
	platform_set_drvdata(dev, &serial_tmpa910_ports[dev->id]);
=======
#if 1
static int serial_tmpa910_suspend(struct platform_device *dev, pm_message_t state)
{
        struct uart_tmpa910_handle *sport = platform_get_drvdata(dev);

        if (sport)
                uart_suspend_port(&serial_tmpa910_reg, &sport->port);

        return 0;
}

static int serial_tmpa910_resume(struct platform_device *dev)
{
        struct uart_tmpa910_handle *sport = platform_get_drvdata(dev);

        if (sport)
                uart_resume_port(&serial_tmpa910_reg, &sport->port);

        return 0;
}

static int tmpa910_uart_portsetup(struct uart_tmpa910_handle *uart_tmpa910_handle);

static int serial_tmpa910_probe(struct platform_device *pdev)
{
	static struct uart_tmpa910_handle serial_tmpa910_ports[TMPA910_SERIAL_MAX];

	struct uart_tmpa910_handle *uart_tmpa910_handle;
	struct uart_port *port;
	int ret;
	unsigned long mapbase;
	int irq;
	struct resource *addr;

	uart_tmpa910_handle = &serial_ports[pdev->id];

	if (pdev->num_resources < 2) {
		printk(KERN_ERR "not enough ressources! %d\n", pdev->num_resources);
		return -ENODEV;
	}

	addr = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq  = platform_get_irq(pdev, 0);

	if (addr == NULL) {
		printk(KERN_ERR "no IO mem ressources!\n");
		return -ENODEV;
	}

	if (irq == NO_IRQ) {
		printk(KERN_ERR "no IRQ ressources! irq=%d\n", irq);
		return -ENODEV;
	}

	mapbase = addr->start;

	snprintf(uart_tmpa910_handle->name, sizeof(uart_tmpa910_handle->name), TMPA910_NAME_PATTERN, pdev->id);

	uart_tmpa910_handle->channel		   = pdev->id;
	uart_tmpa910_handle->port.type  	 = PORT_TMPA910;
	uart_tmpa910_handle->port.iotype	 = UPIO_MEM;
	uart_tmpa910_handle->port.membase	 = NULL;
	uart_tmpa910_handle->port.mapbase	 = mapbase;
	uart_tmpa910_handle->port.irq		   = irq;
	uart_tmpa910_handle->port.uartclk	 = _get_uartclk(uart_tmpa910_handle);
	uart_tmpa910_handle->port.fifosize = 16;
	uart_tmpa910_handle->port.ops		   = &serial_tmpa910_pops;
	uart_tmpa910_handle->port.line		 = pdev->id;

	port = &uart_tmpa910_handle->port;
	spin_lock_init(&port->lock);
	
	/* do the needed setup (ressournces, allocation, hardware config...) */
	ret = tmpa910_uart_portsetup(uart_tmpa910_handle);
	if (ret < 0) {
		return ret;
	}

	ret = uart_add_one_port(&serial_tmpa910_reg, port);
	if (ret != 0)
	{
		uart_unregister_driver(&serial_tmpa910_reg);
		return ret;
	}

	serial_tmpa910_ports[pdev->id].port.dev = &pdev->dev;
	platform_set_drvdata(pdev, port);

>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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
<<<<<<< Updated upstream:drivers/serial/tmpa910.c
=======
#endif

static int tmpa910_uart_portsetup(struct uart_tmpa910_handle *uart_tmpa910_handle)
{
	return _map_tmpa910(uart_tmpa910_handle);
}
>>>>>>> Stashed changes:drivers/serial/tmpa910.c

int __init serial_tmpa910_init(void)
{
	int ret;

	ret = uart_register_driver(&serial_tmpa910_reg);
	if (ret != 0)
		return ret;

<<<<<<< Updated upstream:drivers/serial/tmpa910.c
	ret = platform_driver_register(&serial_tmpa910_driver);
	if (ret != 0)
		uart_unregister_driver(&serial_tmpa910_reg);

	return ret;
=======
	return platform_driver_register(&serial_tmpa910_driver);
>>>>>>> Stashed changes:drivers/serial/tmpa910.c
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
