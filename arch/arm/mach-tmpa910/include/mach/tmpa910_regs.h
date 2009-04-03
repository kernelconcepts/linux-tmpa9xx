/*
 * Copyright (C) 2008 bplan GmbH. All rights reserved.
 *
 * TMPA910 register
 * header
*/

#ifndef __TMPA910_REGS__
#define __TMPA910_REGS__

/********/
/*
 *   Base address = 0xF0805000
 *    Register    Address
 *                                                       Description
 *   Name        (base+)
 * GPIOFDATA   0x03FC      PortF Data Regsiter
 * GPIOFDIR    0x0400      PortF Data Direction Register
 * GPIOFFR1    0x0424      PortF Function Register1
*/

#define PORTF_BASE  			0xF0805000
#define PORTF_GPIOFDIR		(PORTF_BASE + 0x0400)
#define PORTF_GPIOFFR     (PORTF_BASE + 0x0424)
#define PORTF_GPIOFODE    (PORTF_BASE + 0x0c00)

/********/
#define TMPA910_TIMER0 0xf0040000

/********/
#define PORTD_BASE  			0xF0803000
#define PORTD_GPIOFR1 		(PORTD_BASE + 0x0424)
#define PORTD_GPIOFR2 		(PORTD_BASE + 0x0428)
#define PORTD_GPIOIE  		(PORTD_BASE + 0x0810)
#define PORTD_GPIOIC  		(PORTD_BASE + 0x081C)
#define PORTD_GPIOMIS  		(PORTD_BASE + 0x0818)
/********/
#define PORTE_BASE  			0xF0804000
#define PORTE_GPIOEFR 		(PORTE_BASE + 0x0424)

/********/
#define PORTB_BASE  			0xF0801000
#define PORTB_GPIODATA	 (PORTB_BASE + 0x03fc)

/********/
#define PORTG_BASE  			0xF0806000
#define PORTG_GPIOFR	 (PORTG_BASE + 0x0424)

/********/
#define LCDC_BASE 0xf4200000
/********/
#define I2C0_BASE   0xF0070000
#define I2C1_BASE   0xF0071000

/********/
#define CMOSCAM_BASE  0xF2020000

/********/
#define DMAC_BASE  0xF4100000

/********/
#define SYSCTRL_BASE  (0xF0050000)

/********/
#define NANDF_BASE  0xf2010000

/********/
#define LCDDA_BASE  0xF2050000

/********/
#define INTR_BASE  0xF4000000

/********/
#define TS_BASE   0xf00601f0

/********/
#define ADC_BASE  0xf0080000
/********/
#define TOUCHSCREEN_BASE   0xf00601f0
#define ADC_BASE           0xf0080000

/********/
#define SRAM_BASE      0xF8002000
#define SRAM_SIZE      0x0000c000

#endif /* __TMPA910_REGS__ */
