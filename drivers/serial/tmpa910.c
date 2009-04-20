/*
 *  linux/drivers/serial/tmpa910_fw.c
 *
 *  Based on drivers/serial/pxa.c by Nicolas Pitre
 *
 *  Author:	Nicolas 
 *  Created:	Feb 20, 2003
 *  Copyright:	(C) 2008 bplan GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *

 */


/*********/
/*********/
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/serial_reg.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/ioport.h>


#include <asm/io.h>
#include <mach/hardware.h>
#include <asm/irq.h>
/*********/
/*********/

//#define __DEBUG__
#include <linux/debug.h>

/*********/
/*********/



#define DRIVER_NAME  	"tmpa910_uart"

#define TMPA910_UART_REGSIZE 0x50
#define TMPA910_SERIAL_MAX 2
#define TMPA910_NAME_PATTERN  "TMPA910 UART Channel %d"
/*********/
/*********/
struct tmpa910_uart_regs
{
  uint32_t dr;      // 0x000
  uint32_t sr;     // 0x004
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
/*********/
/*********/
// The data register contains 8bit for the char and 4 status bits

#define DR_OE  (1<<11)  // OE RO Undefined Overrun error
                        // 0y0: There is an empty space in the FIFO.
                        // 0y1: Overrun error flag
#define DR_BE  (1<<10)  // BE RO Undefined Break error
                        // 0y0: No error detected
                        // 0y1: Error detected
#define DR_PE  (1<<9)   // PE RO Undefined Parity error
                        // 0y0: No error detected
                        // 0y1: Error detected
#define DR_FE  (1<<8)   // FE RO Undefined Framing error
                        // 0y0: No error detected
                        // 0y1: Error detected
/*********/
/*********/
// The data register contains 8bit for the char and 4 status bits

#define SR_OE  (1<<3)  
#define SR_BE  (1<<2) 
#define SR_PE  (1<<1) 
#define SR_FE  (1<<0)

/*********/
/*********/

#define LCRH_SPS     (1<<7)   SPS  R/W 0y0  Stick parity select:
#define LCRH_WLEN_5B (0x0   )
#define LCRH_WLEN_6B (0x1<<5)
#define LCRH_WLEN_7B (0x2<<5)
#define LCRH_WLEN_8B (0x3<<5)
#define LCRH_FEN      (1<<4)  // R/W 0y0  FIFO control
                              // 0y1: FIFO mode
                              // 0y0: Character mode
#define LCRH_STP2     (1<<3)  //   STP2 R/W 0y0  Stop bit select
                              // 0y0: 1 stop bit
                              // 0y1: 2 stop bits
#define LCRH_EPS      (1<<2)  // EPS  R/W 0y0  Even parity select (Refer to Table 3.13.1 for
                              // the truth table.)
                              // 0y1: Even
                              // 0y0: Odd
#define LCRH_PEN      (1<<1)  // PEN  R/W 0y0  Parity control (Refer to Table 3.13.1 for the
                              // truth table.)
                              // 0y0: Disable
                              // 0y1: Enable
#define LCRH_BRK      (1<<0)  // BRK  R/W 0y0  Send break
                              // 0y0: No effect
                              // 0y1: Send break

/*********/
/*********/
#define CR_CTSEn (1<<15)      // CTSEn  R/W 0y0       CTS hardware flow control enable
#define CR_RTSEn (1<<14)      // RTSEn  R/W 0y0       RTS hardware flow control enable

#define CR_RTS (1<<11)        // R/W 0y0       Complement of the UART Request To Send
                              // (nUARTRTS) modem status output
                              // 0y0: Modem status output is "1".
                              // 0y1: Modem status output is "0".
#define CR_DTR (1<<10)        //  R/W 0y0       Complement of the UART Data Set Ready
                              // (nUARTDTR) modem status output
                              // 0y0: Modem status output is "1".
                              // 0y1: Modem status output is "0".
#define CR_RXE (1<<9)         // RXE    R/W 0y1       UART receive enable
#define CR_TXE (1<<8)         // TXE    R/W 0y1       UART transmit enable

#define CR_SIRLP (1<<2)       // SIRLP  R/W 0y0       IrDA encoding mode select for transmitting "0"
                              // 0y0: "0" bits are transmitted as an active high
                              // pulse of 3/16th of the bit period.
                              // 0y1:"0" bits are transmitted with a pulse width
                              // that is 3 times the period of the IrLPBaud16
                              // input signal.
			      
#define CR_SIREN (1<<1)       // SIREN  R/W 0y0       SIR enable
#define CR_UARTEN (1<<0)      // UARTEN R/W 0y0       UART enable


/*********/
/*********/
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


/*********/
/*********/

// for interrupt mask and status
// 0y0: Clear the mask
// 0y1: Set the mask
		    
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

/*********/
/*********/
static struct uart_tmpa910_handle serial_ports[TMPA910_SERIAL_MAX];

static int _fill_uarthandle(
	struct uart_tmpa910_handle *uart_tmpa910_handle,
	int index);


static inline void wait_for_xmitr(struct uart_tmpa910_handle *uart_tmpa910_handle);
static inline void wait_for_txempty(struct uart_tmpa910_handle *uart_tmpa910_handle);
/*********/
/*********/

static int _get_uartclk(struct uart_tmpa910_handle *uart_tmpa910_handle)
{
	return 96*1000*1000;
}

#ifdef __DEBUG__
static void _dump_regs(struct uart_tmpa910_handle *uart_tmpa910_handle)
{
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	
	printk("port at 0x%p / 0x%lx\n", regs, (unsigned long) uart_tmpa910_handle->port.mapbase);
	
	if (regs == NULL)
		return;
		
	printk("dr    at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, dr ),    regs->dr);
	printk("sr    at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, sr ),   regs->sr);
	printk("ecr   at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, ecr ),   regs->ecr);
	printk("fr    at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, fr),     regs->fr);
	printk("ilpr  at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, ilpr ),  regs->ilpr);
	printk("ibrd  at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, ibrd ),  regs->ibrd);
	printk("fbrd  at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, fbrd ),  regs->fbrd);
	printk("lcr_h at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, lcr_h),  regs->lcr_h);
	printk("cr    at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, cr ),    regs->cr);
	printk("ifls  at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, ifls ),  regs->ifls);
	printk("imsc  at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, imsc ),  regs->imsc);
	printk("ris   at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, ris ),   regs->ris);
	printk("mis   at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, mis ),   regs->mis);
	printk("icr   at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, icr ),   regs->icr);
	printk("dmacr at 0x%2x: 0x%8x\n", offsetof(struct tmpa910_uart_regs, dmacr ), regs->dmacr);

}

#endif
/*********/
/*********/


static int _map_tmpa910(struct uart_tmpa910_handle *uart_tmpa910_handle)
{

	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	
	NPRINTK("-> regs = %p\n", regs);

	if (regs)
	{
		NPRINTK("port already requested or mapped (regs=0x%p)\n", regs);
		return 0;
	}
	
	NPRINTK("regs = %p\n", regs);

	regs = uart_tmpa910_handle->port.mapbase;
	if (regs == NULL)
	{
		NPRINTK("Fail to map UART controller (mapbase=0x%x)\n", uart_tmpa910_handle->port.mapbase);
		return -EINVAL;
	}
	

	NPRINTK("%s mapped at 0x%p\n", uart_tmpa910_handle->name, regs);
	
	uart_tmpa910_handle->port.membase = (void *) regs;
	uart_tmpa910_handle->regs = regs;

	
#ifdef __DEBUG__
	_dump_regs(uart_tmpa910_handle);
#endif
	
	NPRINTK("return 0\n");
	
	return 0;
}


/*********/
/*********/



static void serial_tmpa910_enable_ms(struct uart_port *port)
{
	//struct uart_tmpa910_handle *up = (struct uart_tmpa910_handle *)port;

	NPRINTK("->\n");
}

static void serial_tmpa910_stop_tx(struct uart_port *port)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	
	NPRINTK("->\n");
	regs->imsc &= ~INT_TX;
}

static void serial_tmpa910_stop_rx(struct uart_port *port)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	
	NPRINTK("->\n");
	regs->imsc &= ~INT_RX;
}



static void transmit_chars(struct uart_tmpa910_handle *uart_tmpa910_handle)
{
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	struct circ_buf *xmit = &uart_tmpa910_handle->port.info->xmit;
	int count;

	NPRINTK("->\n");

	if (uart_tmpa910_handle->port.x_char) {
		wait_for_xmitr(uart_tmpa910_handle);
		regs->dr = uart_tmpa910_handle->port.x_char & 0xff;
		uart_tmpa910_handle->port.icount.tx++;
		uart_tmpa910_handle->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&uart_tmpa910_handle->port)) {
		serial_tmpa910_stop_tx(&uart_tmpa910_handle->port);
		return;
	}

	count = uart_tmpa910_handle->port.fifosize;
	do {
		wait_for_xmitr(uart_tmpa910_handle);
		regs->dr = xmit->buf[xmit->tail] & 0xff;
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		uart_tmpa910_handle->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&uart_tmpa910_handle->port);


	if (uart_circ_empty(xmit))
		serial_tmpa910_stop_tx(&uart_tmpa910_handle->port);
}


static inline void
receive_chars(struct uart_tmpa910_handle *uart_tmpa910_handle)
{
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	struct tty_struct *tty = uart_tmpa910_handle->port.info->port.tty;
	unsigned int ch, flag;
	uint32_t fr_reg;
	uint32_t dr_reg;
	int max_count = 256;

	do {
		fr_reg = regs->fr; 
		dr_reg = regs->dr; 
		
		ch     = regs->dr & 0xff;
		flag = TTY_NORMAL;
		
		uart_tmpa910_handle->port.icount.rx++;

		if (unlikely(fr_reg & (UART_LSR_BI | UART_LSR_PE |
				       UART_LSR_FE | UART_LSR_OE))) {
			/*
			 * For statistics only
			 */
			if (dr_reg & DR_BE) {
				uart_tmpa910_handle->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&uart_tmpa910_handle->port))
					goto ignore_char;
			} else if (dr_reg & DR_PE)
				uart_tmpa910_handle->port.icount.parity++;
			else if (dr_reg &DR_FE)
				uart_tmpa910_handle->port.icount.frame++;
			if (dr_reg & DR_OE)
				uart_tmpa910_handle->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			// *status &= up->port.read_status_mask;

#if defined(CONFIG_SERIAL_TMPA910_CONSOLE) && 0
			if (up->port.line == up->port.cons->index) {
				/* Recover the break flag from console xmit */
				*status |= up->lsr_break_flag;
				uart_tmpa910_handle->lsr_break_flag = 0;
			}
#endif
			if (dr_reg & DR_BE) {
				flag = TTY_BREAK;
			} else if (dr_reg & DR_PE)
				flag = TTY_PARITY;
			else if (dr_reg & DR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&uart_tmpa910_handle->port, ch))
			goto ignore_char;

		uart_insert_char(&uart_tmpa910_handle->port, dr_reg, DR_OE, ch, flag);

	ignore_char:
		fr_reg = regs->fr; 
	} while ( ((fr_reg & FR_RXFE)==0) && (max_count-- > 0));

	tty_flip_buffer_push(tty);
}


static unsigned int serial_tmpa910_tx_empty(struct uart_port *port);
static void serial_tmpa910_start_tx(struct uart_port *port)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	struct circ_buf *xmit = &uart_tmpa910_handle->port.info->xmit;
	
	NPRINTK("->\n uart_circ_chars_pending(xmit)=%d", uart_circ_chars_pending(xmit) );
	regs->imsc |= INT_TX;
	
	if (uart_circ_chars_pending(xmit) )
	{
		wait_for_xmitr(uart_tmpa910_handle);
		transmit_chars(uart_tmpa910_handle);
	}
}

static inline void check_modem_status(struct uart_tmpa910_handle *uart_tmpa910_handle)
{

	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	uint32_t fr_reg;
	
	NPRINTK("->\n");
	
	fr_reg = regs->fr;

	// rng is ring ? not documented..
	if (fr_reg & FR_RI)
		uart_tmpa910_handle->port.icount.rng++;
	if (fr_reg & FR_DSR)
		uart_tmpa910_handle->port.icount.dsr++;
	if (fr_reg & FR_DCD)
		uart_tmpa910_handle->port.icount.dcd++;
	if (fr_reg & FR_CTS)
		uart_tmpa910_handle->port.icount.cts++;

	wake_up_interruptible(&uart_tmpa910_handle->port.info->delta_msr_wait);

}

/*
 * This handles the interrupt from one port.
 */


static unsigned int serial_tmpa910_tx_empty(struct uart_port *port)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	
	unsigned long flags;
	unsigned int ret;
	

	NPRINTK("->\n");
		
	spin_lock_irqsave(&uart_tmpa910_handle->port.lock, flags);

	ret = regs->fr & FR_TXFE;

	spin_unlock_irqrestore(&uart_tmpa910_handle->port.lock, flags);
	NPRINTK("<- ret=%d. regs->fr=0x%x\n", ret, regs->fr);
	
	return ret;
}

static inline irqreturn_t
serial_tmpa910_irq(int irq, void *dev_id)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)dev_id;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	
	uint32_t mis_reg;
	uint32_t icr_reg;
	
	NPRINTK("->\n");
	
	icr_reg = 0;
	mis_reg = regs->mis;
	
	if (mis_reg==0)
		return IRQ_NONE;

	if (mis_reg & INT_RX)
	{
		receive_chars(uart_tmpa910_handle);
		icr_reg |= INT_RX;
	}
		
	check_modem_status(uart_tmpa910_handle);
	
	if (mis_reg & INT_TX)
	{
		transmit_chars(uart_tmpa910_handle);
		icr_reg |= INT_TX;
	}
	
	if (icr_reg)
		regs->icr = icr_reg;

	return IRQ_HANDLED;
}

static unsigned int serial_tmpa910_get_mctrl(struct uart_port *port)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;

	unsigned int ret;
	uint32_t fr_reg;
	
	NPRINTK("->\n");

	ret    = 0;
	fr_reg = regs->fr;

	if (fr_reg & FR_DCD)
		ret |= TIOCM_CAR;
		
	if (fr_reg & FR_RI)
		ret |= TIOCM_RNG;
		
	if (fr_reg & FR_DSR)
		ret |= TIOCM_DSR;
		
	if (fr_reg & FR_CTS)
		ret |= TIOCM_CTS;
		
	NPRINTK("<- ret=0x%x, fr_reg=0x%x\n", ret, fr_reg);
	return ret;
}

static void serial_tmpa910_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	uint32_t cr_reg;
	
	NPRINTK("->mctrl=0x%x, regs=0x%p\n", mctrl, regs);

	cr_reg = 0;
	
	if (mctrl & TIOCM_RTS)
		cr_reg |= CR_RTS;
	if (mctrl & TIOCM_DTR)
		cr_reg |= CR_DTR;

	regs->cr |= cr_reg;

}

static void serial_tmpa910_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	unsigned long flags;
	uint32_t lcrh_reg;

	NPRINTK("->break_state=%d\n", break_state);

	spin_lock_irqsave(&uart_tmpa910_handle->port.lock, flags);

	lcrh_reg = regs->lcr_h;

	if (break_state == -1)
		lcrh_reg |= LCRH_BRK;
	else
		lcrh_reg &= ~LCRH_BRK;
		
		
	regs->lcr_h = lcrh_reg;
		
	spin_unlock_irqrestore(&uart_tmpa910_handle->port.lock, flags);
}


static int serial_tmpa910_startup(struct uart_port *port)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	unsigned long flags;
	int ret;

	NPRINTK("-> regs=0x%p\n", regs);

	// Make sure the port is off and no IRQ could be generated
	regs->cr   = 0;
	regs->imsc   = 0;
	regs->icr    = 0x7ff;
	
	// Clear errors if any
	regs->ecr    = 0x0;
	
	/*
	 * Allocate the IRQ
	 */
	ret = request_irq(uart_tmpa910_handle->port.irq, serial_tmpa910_irq, 0, uart_tmpa910_handle->name, uart_tmpa910_handle);
	if (ret)
	{
		NPRINTK("Fail allocate the interrupt (vector=%d)\n", uart_tmpa910_handle->port.irq );
		return ret;
	}

	NPRINTK("irq %d allocated\n", uart_tmpa910_handle->port.irq);
		
	uart_tmpa910_handle->irq_allocated = 1;

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */


	/*
	 * Clear the interrupt registers.
	 */
	regs->icr    = 0x7ff;

	/*
	 * Now, initialize the UART
	 */

	NPRINTK("initialize the UART\n");
	
	spin_lock_irqsave(&uart_tmpa910_handle->port.lock, flags);
	serial_tmpa910_set_mctrl(&uart_tmpa910_handle->port, uart_tmpa910_handle->port.mctrl);
	spin_unlock_irqrestore(&uart_tmpa910_handle->port.lock, flags);

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	regs->imsc   = INT_TX | INT_RX;
	regs->lcr_h |= LCRH_FEN;
	regs->cr    |= CR_RXE | CR_TXE | CR_UARTEN;

#ifdef __DEBUG__
	_dump_regs(uart_tmpa910_handle);
#endif

	NPRINTK("done\n");
	return 0;
}

static void serial_tmpa910_shutdown(struct uart_port *port)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	unsigned long flags;

	NPRINTK("->\n");
	
	/*
	 * Disable interrupts from this port
	 */
	regs->cr   = 0;
	regs->imsc = 0;
	regs->icr  = 0x7ff;
	regs->ecr  = 0x0;
	
	if (uart_tmpa910_handle->irq_allocated)
		free_irq(uart_tmpa910_handle->port.irq, uart_tmpa910_handle);

	uart_tmpa910_handle->irq_allocated = 0;
	

	spin_lock_irqsave(&uart_tmpa910_handle->port.lock, flags);
	serial_tmpa910_set_mctrl(&uart_tmpa910_handle->port, uart_tmpa910_handle->port.mctrl);
	spin_unlock_irqrestore(&uart_tmpa910_handle->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */

}

static inline int _get_div_fract(int uartclk, int baud, int div_integer)
{
	int a;
	
	// Baud rate divisor = (UARTCLK)/(16 × baud)= integer part + fract part;
	//
	// When the required baud rate is 230400 and fUARTCLK = 4MHz:
	// Baud rate divisor = (4 × 106)/(16 × 230400)= 1.085
	// Therefore, fractional part is ((0.085 × 64)+ 0.5) = 5.94.

	// a = divisor * 1000
	a = ( (uartclk/16) *10) / (baud/100);

	// a = integer part*1000 - divisor * 1000
	// a -> fract part * 1000

	a = a - div_integer*1000;
	a = a * 64 + 500;
	a = a / 1000;

	return a;
}

static void
serial_tmpa910_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	uint32_t lcrh_reg;
	unsigned long flags;
	unsigned int baud;
	int div_integer;
	int div_fract;

	NPRINTK("-> regs = 0x%x\n");
	
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcrh_reg = LCRH_WLEN_5B;
		break;
	case CS6:
		lcrh_reg = LCRH_WLEN_5B;
		break;
	case CS7:
		lcrh_reg = LCRH_WLEN_7B;
		break;
	default:
	case CS8:
		lcrh_reg = LCRH_WLEN_8B;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		lcrh_reg |= LCRH_STP2;
	if (termios->c_cflag & PARENB)
		lcrh_reg |= LCRH_PEN;
	if (!(termios->c_cflag & PARODD))
		lcrh_reg |= LCRH_EPS;

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 1200, _get_uartclk(uart_tmpa910_handle) / 16);
	div_integer = uart_get_divisor(port, baud);
	div_fract   = _get_div_fract(_get_uartclk(uart_tmpa910_handle), baud, div_integer);
	
	NPRINTK("baud=%d, div_integer=%d, div_fract=%d\n", baud, div_integer, div_fract);

	regs->ibrd = div_integer;
	regs->fbrd = div_fract;
	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&uart_tmpa910_handle->port.lock, flags);


	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	uart_tmpa910_handle->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		uart_tmpa910_handle->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		uart_tmpa910_handle->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characters to ignore
	 */
	uart_tmpa910_handle->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		uart_tmpa910_handle->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		uart_tmpa910_handle->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			uart_tmpa910_handle->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		uart_tmpa910_handle->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	//uart_tmpa910_handle->ier &= ~UART_IER_MSI;
	//if (UART_ENABLE_MS(&uart_tmpa910_handle->port, termios->c_cflag))
	//	uart_tmpa910_handle->ier |= UART_IER_MSI;


	serial_tmpa910_set_mctrl(&uart_tmpa910_handle->port, uart_tmpa910_handle->port.mctrl);


	regs->lcr_h = lcrh_reg;
	regs->cr   |= CR_UARTEN;
	
#ifdef __DEBUG__
	_dump_regs(uart_tmpa910_handle);
#endif
	spin_unlock_irqrestore(&uart_tmpa910_handle->port.lock, flags);
	
}

static void
serial_tmpa910_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	//struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	NPRINTK("->\n");
}

static void serial_tmpa910_release_port(struct uart_port *port)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	NPRINTK("->\n");
	



	port->membase = NULL;
}



static int serial_tmpa910_request_port(struct uart_port *port)
{
	return _map_tmpa910( (struct uart_tmpa910_handle *) port);
}

static void serial_tmpa910_config_port(struct uart_port *port, int flags)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	
	NPRINTK("->\n");
	uart_tmpa910_handle->port.type = PORT_TMPA910;
}

static int
serial_tmpa910_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	NPRINTK("->\n");
	return -EINVAL;
}

static const char *
serial_tmpa910_type(struct uart_port *port)
{
	struct uart_tmpa910_handle *up = (struct uart_tmpa910_handle *)port;
	
	NPRINTK("->\n");
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
	.pm		= serial_tmpa910_pm,
	.type		= serial_tmpa910_type,
	.release_port	= serial_tmpa910_release_port,
	.request_port	= serial_tmpa910_request_port,
	.config_port	= serial_tmpa910_config_port,
	.verify_port	= serial_tmpa910_verify_port,
};




static int _fill_uarthandle(
	struct uart_tmpa910_handle *uart_tmpa910_handle,
	int index )
{
	unsigned long mapbase;
	char name[256];
	struct device_node *dp = NULL;
	const char *onestr;
	const u32 *long_ptr;

	unsigned int len;
	int irq;

	irq = 10+index;
	mapbase = 0xf2000000 + 0x1000*index;

	NPRINTK("index=%d. irq=%d, mappase=%lx\n", index, irq, mapbase);

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

/*
 *	Wait for transmitter & holding register to empty
 */
static inline void wait_for_xmitr(struct uart_tmpa910_handle *uart_tmpa910_handle)
{
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	unsigned int tmout = 10000;


	//NPRINTK("-> regs=0x%x. regs->fr=%x, port flags=0x%x\n", regs, regs->fr, uart_tmpa910_handle->port.flags);

	/* Wait up to 10ms for the character(s) to be sent. */
	while( (regs->fr & FR_TXFF) )
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

	//NPRINTK("-> regs=0x%x. regs->fr=%x, port flags=0x%x\n", regs, regs->fr, uart_tmpa910_handle->port.flags);
	
	/* Wait up to 10ms for the character(s) to be sent. */
	while( (regs->fr & FR_TXFE) == 0 )
	{
		if (--tmout == 0)
			break;
			
		udelay(1);
	}
}




#ifdef CONFIG_SERIAL_TMPA910_CONSOLE


static struct uart_driver serial_tmpa910_reg;


static void serial_tmpa910_console_putchar(struct uart_port *port, int ch)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = (struct uart_tmpa910_handle *)port;
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	//NPRINTK("->\n");
	
	wait_for_xmitr(uart_tmpa910_handle);
	regs->dr = ch & 0xff;
}

/*
 * Print a string to the serial port trying not to disturb
 * any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void
serial_tmpa910_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle = &serial_ports[co->index];
	volatile struct tmpa910_uart_regs *regs = uart_tmpa910_handle->regs;
	uint32_t imsc_reg;
	
	NPRINTK("-> co->index = %d\n", co->index);


	/*
	 *	First save the IER then disable the interrupts
	 */
	imsc_reg = regs->imsc;
	regs->imsc = 0;

	uart_console_write(&uart_tmpa910_handle->port, s, count, serial_tmpa910_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	regs->imsc = imsc_reg;

}


static int __init
serial_tmpa910_console_setup(struct console *co, char *options)
{
	struct uart_tmpa910_handle *uart_tmpa910_handle;
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;
	
	NPRINTK("-> co->index=%d\n");
	

	if (co->index == -1 || co->index >= serial_tmpa910_reg.nr)
		co->index = 0;
		

	//co->index = 1;

	spin_lock_init(&port->lock);	
	
  uart_tmpa910_handle = &serial_ports[co->index];
	port = &uart_tmpa910_handle->port;


	ret = _fill_uarthandle(uart_tmpa910_handle, co->index);
	if (ret<0)
	{
		NPRINTK("_fill_uarthandle failed. ret=%d\n", ret);
		return ret;
	}

	ret = _map_tmpa910(uart_tmpa910_handle);
	if( ret < 0)
	{
		NPRINTK("Fail to map IO mem. ret=%d\n", ret);
		return ret;
	}
	
	
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
		
	return uart_set_options(&uart_tmpa910_handle->port, co, baud, parity, bits, flow);

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
	NPRINTK("->\n");

	
#ifdef CONFIG_SERIAL_TMPA910_CONSOLE_PREFERED
  	add_preferred_console("ttyS", 0, NULL);
	printk(DRIVER_NAME ": Welcome. Myself as prefered console :-)\n" );
#else
	printk(DRIVER_NAME ": Welcome\n" );
#endif
	
	register_console(&serial_tmpa910_console);
	return 0;
}

console_initcall(serial_tmpa910_console_init);

#define TMPA910_CONSOLE	&serial_tmpa910_console
#else
#define TMPA910_CONSOLE	NULL
#endif


static struct uart_driver serial_tmpa910_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= "TMPA910 serial",
	.dev_name	= "ttyS",
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= TMPA910_SERIAL_MAX,
	.cons		= TMPA910_CONSOLE,
};

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

	NPRINTK("-> pdev = %p\n", pdev);

	// pb
	//if (pdev->id==0)
	//	return -1;

	uart_tmpa910_handle = &serial_ports[pdev->id];


	if (pdev->num_resources < 2)
	{
		NPRINTK("no enough ressources! %d\n", pdev->num_resources);
		return -ENODEV;
	}

	addr = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq  = platform_get_irq(pdev, 0);

	if (addr == NULL)
	{
		NPRINTK("no IO mem ressources!\n");
		return -ENODEV;
	}

	if (irq == NO_IRQ)
	{
		NPRINTK("no IRQ ressources! irq=%d\n", irq);
		return -ENODEV;
	}

	mapbase = addr->start;

	NPRINTK("index=%d. irq=%d, mappase=%lx\n",  pdev->id, irq, mapbase);

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
	
	NPRINTK("tmpa910_uart_portsetup i=%d, port=%p\n", pdev->id, port);
	// do the needed setup (ressournces, allocation, hardware config...)
	ret = tmpa910_uart_portsetup(uart_tmpa910_handle);
	if(ret<0)
	{
		NPRINTK("_fill_uarthandle with error %d\n", ret);
		return ret;
	}


	NPRINTK("uart_add_one_port pdev->id=%d, port=%p\n", pdev->id, port);
	ret = uart_add_one_port(&serial_tmpa910_reg, port);
	if (ret != 0)
	{
		uart_unregister_driver(&serial_tmpa910_reg);
		return ret;
	}


	NPRINTK("platform_set_drvdata and co pdev->id=%d, port=%p\n", pdev->id, port);
	serial_tmpa910_ports[pdev->id].port.dev = &pdev->dev;
	platform_set_drvdata(pdev, port);

	return 0;
}

static int serial_tmpa910_remove(struct platform_device *pdev)
{
	struct uart_tmpa910_handle *sport = platform_get_drvdata(pdev);

	NPRINTK("-> pdev = %p\n", pdev);

	platform_set_drvdata(pdev, NULL);

	if (sport)
		uart_remove_one_port(&serial_tmpa910_reg, &sport->port);

	return 0;
}

static struct platform_driver serial_tmpa910_driver = {
        .probe          = serial_tmpa910_probe,
        .remove         = serial_tmpa910_remove,

	.suspend	= serial_tmpa910_suspend,
	.resume		= serial_tmpa910_resume,
	.driver		= {
	        .name	= "tmpa910-uart",
	},
};
#endif

static int tmpa910_uart_portsetup(struct uart_tmpa910_handle *uart_tmpa910_handle)
{
	return _map_tmpa910(uart_tmpa910_handle);

}

int __init serial_tmpa910_init(void)
{
	int ret;
	
	NPRINTK("->\n");

	ret = uart_register_driver(&serial_tmpa910_reg);
	if (ret != 0)
		return ret;

	return platform_driver_register(&serial_tmpa910_driver);

}

void __exit serial_tmpa910_exit(void)
{
	NPRINTK("->\n");
	
	platform_driver_unregister(&serial_tmpa910_driver);
}

module_init(serial_tmpa910_init);
module_exit(serial_tmpa910_exit);

MODULE_LICENSE("GPL");

