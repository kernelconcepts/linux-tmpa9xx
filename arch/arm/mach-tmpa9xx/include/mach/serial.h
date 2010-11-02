#ifndef	__ASM_ARCH_TMPA9XX_SERIAL_H__
#define	__ASM_ARCH_TMPA9XX_SERIAL_H__

#define PLLON (0x80)
#define PLLVALUE (0x1f)
#define PLL_6 (0x05)
#define PLL_8 (0x07)
#define SYS_GEAR (0x03)
#define TOPGEAR (0x0)
#define GEAR_2 (0x1)
#define GEAR_4 (0x2)
#define GEAR_8 (0x3)
#define UARTCLK_96MHz (96)
#define UARTCLK_72MHz (72)
#define UARTCLK_48MHz (48)
#define UARTCLK_36MHz (36)
#define UARTCLK_24MHz (24)
#define UARTCLK_18MHz (18)
#define UARTCLK_12MHz (12)
#define UARTCLK_9MHz  ( 9)

enum {
	UART_DR = 0x00,
	UART_ECR = 0x04,
	UART_FR = 0x18,
	UART_IBRD = 0x24,
	UART_ILPR = 0x18,
	UART_FBRD = 0x28,
	UART_LCR = 0x2c,
	UART_CR = 0x30,
	UART_FLS = 0x34,
	UART_IMSC = 0x38,
	UART_RIS = 0x3c,
	UART_MIS = 0x40,
	UART_ICR = 0x44,
	UART_DMACR = 0x48,
};
	

/* UARTDR */
#define	UART_DR_OE		(1 << 11)
#define	UART_DR_BE		(1 << 10)
#define	UART_DR_PE		(1 << 9)
#define	UART_DR_FE		(1 << 8)

/* UARTRSR */
#define	UART_RSR_OE		(1 << 3)
#define	UART_RSR_BE		(1 << 2)
#define	UART_RSR_PE		(1 << 1)
#define	UART_RSR_FE		(1 << 0)

/* UARTFR */
#define	UART_FR_RI		(1 << 8)
#define	UART_FR_TXFE		(1 << 7)
#define	UART_FR_RXFF		(1 << 6)
#define	UART_FR_TXFF		(1 << 5)
#define	UART_FR_RXFE		(1 << 4)
#define	UART_FR_BUSY		(1 << 3)
#define	UART_FR_DCD		(1 << 2)
#define	UART_FR_DSR		(1 << 1)
#define	UART_FR_CTS		(1 << 0)

/* UARTILPR */

/* UARTLCR */
#define	UART_LCR_SPS		(1 << 7)
#define	UART_LCR_WLEN5		(0x0)
#define	UART_LCR_WLEN6		(1 << 5)
#define	UART_LCR_WLEN7		(2 << 5)
#define	UART_LCR_WLEN8		(3 << 5)
#define	UART_LCR_FEN		(1 << 4)
#define	UART_LCR_STOP		(1 << 3)
#define	UART_LCR_EPAR		(1 << 2)
#define	UART_LCR_PARITY		(1 << 1)
#define	UART_LCR_SBC		(1 << 0)

/* UARTCR */
#define	UART_CR_CTS_EN		(1 << 15)
#define	UART_CR_RTS_EN		(1 << 14)
#define	UART_CR_OUT2		(1 << 13)
#define	UART_CR_OUT1		(1 << 12)
#define	UART_CR_RTS		(1 << 11)
#define	UART_CR_DTR		(1 << 10)
#define	UART_CR_RXE		(1 << 9)
#define	UART_CR_TXE		(1 << 8)
#define	UART_CR_LBE		(1 << 7)
#define	UART_CR_SIRLP		(1 << 2)
#define	UART_CR_SIREN		(1 << 1)
#define	UART_CR_UARTEN		(1 << 0)

/* UART_INT_MASK */
#define	UARTIMSC_OEIM		(1 << 10)
#define	UARTIMSC_BEIM		(1 << 9)
#define	UARTIMSC_PEIM		(1 << 8)
#define	UARTIMSC_FEIM		(1 << 7)
#define	UARTIMSC_RTIM		(1 << 6)
#define	UARTIMSC_TXIM		(1 << 5)
#define	UARTIMSC_RXIM		(1 << 4)
#define	UARTIMSC_DSRMIM		(1 << 3)
#define	UARTIMSC_DCDMIM		(1 << 2)
#define	UARTIMSC_CTSMIM		(1 << 1)
#define	UARTIMSC_RIMIM		(1 << 0)

/* UART raw interrupt status */
#define	UART_RIS_OE		(1 << 10)
#define	UART_RIS_BE		(1 << 9)
#define	UART_RIS_PE		(1 << 8)
#define	UART_RIS_FE		(1 << 7)
#define	UART_RIS_RT		(1 << 6)
#define	UART_RIS_TX		(1 << 5)
#define	UART_RIS_RX		(1 << 4)
#define	UART_RIS_DSR		(1 << 3)
#define	UART_RIS_DCD		(1 << 2)
#define	UART_RIS_CTS		(1 << 1)
#define	UART_RIS_RI		(1 << 0)

/* UART masked interrupt status */
#define	UART_MIS_OE		(1 << 10)
#define	UART_MIS_BE		(1 << 9)
#define	UART_MIS_PE		(1 << 8)
#define	UART_MIS_FE		(1 << 7)
#define	UART_MIS_RT		(1 << 6)
#define	UART_MIS_TX		(1 << 5)
#define	UART_MIS_RX		(1 << 4)
#define	UART_MIS_DSR		(1 << 3)
#define	UART_MIS_DCD		(1 << 2)
#define	UART_MIS_CTS		(1 << 1)
#define	UART_MIS_RI		(1 << 0)

/* UART DMA CR */
#define	UART_DMACR_DMA_ERR	(1 << 2)
#define	UART_DMACR_TXDMAE	(1 << 1)
#define	UART_DMACR_RSDMAE	(1 << 0)

#define UART_MIS_ANY_DELTA	(0x0f)

#endif
