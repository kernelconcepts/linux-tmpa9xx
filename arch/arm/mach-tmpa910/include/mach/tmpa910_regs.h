/*
 * Copyright (C) 2008 bplan GmbH. All rights reserved.
 * Copyright (C) 2009 Florian Boor <florian.boor@kernelconcepts.de>
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
 * 
 * TMPA910 register header
 */

#ifndef __TMPA910_REGS__
#define __TMPA910_REGS__

/* GPIO Ports */

#define PORT_BASE          0xF0800000
#define PORTA    0x0000
#define PORTB    0x1000
#define PORTC    0x2000
#define PORTD    0x3000
#define PORTE    0x4000
#define PORTF    0x5000
#define PORTG    0x6000
#define PORTH    0x7000
#define PORTJ    0x8000
#define PORTK    0x9000
#define PORTL    0xA000
#define PORTM    0xB000
#define PORTN    0xC000
#define PORTP    0xD000
#define PORTR    0xE000
#define PORTT    0xF000

#define PORT_OFS_DATA      0x03FC  /* 0x000 - 0x3FC, data register masked from 0x00 to 0xFF << 2 */
#define PORT_OFS_DIR       0x0400  /* direction register */
#define PORT_OFS_FR1       0x0424  /* function register 1 */
#define PORT_OFS_FF2       0x0428  /* function register 2 */
#define PORT_OFS_IS        0x0804  /* interrupt sensitivity */
#define PORT_OFS_IBE       0x0808  /* interrupt both edge register */
#define PORT_OFS_IEV       0x080C  /* interrupt event register */
#define PORT_OFS_IE        0x0810  /* interrupt enable register*/
#define PORT_OFS_RIS       0x0814  /* raw interrupt status register */
#define PORT_OFS_MIS       0x0818  /* masked interrupt status */
#define PORT_OFS_IC        0x081C  /* interrupt clear register */
#define PORT_OFS_ODE       0x0C00  /* open drain output */

#define TMPA910_GPIO_REG(x,y) (PORT_BASE | (x) | (y)) /* base addr + port offset + register offset */

#define TMPA910_GPIO_REG_DATA(x) (PORT_BASE | PORT_OFS_DATA | (x))
#define TMPA910_GPIO_REG_IS(x)   (PORT_BASE | PORT_OFS_IS | (x))
#define TMPA910_GPIO_REG_IBE(x)  (PORT_BASE | PORT_OFS_IBE | (x))
#define TMPA910_GPIO_REG_IEV(x)  (PORT_BASE | PORT_OFS_IEV | (x))
#define TMPA910_GPIO_REG_IE(x)   (PORT_BASE | PORT_OFS_IE | (x))
#define TMPA910_GPIO_REG_MIS(x)  (PORT_BASE | PORT_OFS_MIS | (x))
#define TMPA910_GPIO_REG_IC(x)   (PORT_BASE | PORT_OFS_IC | (x))

//#define TMPA910_CFG_PORT_GPIO(x) (*((volatile int*)(PORT_BASE | (x) | PORT_OFS_FR1)) = 0)
#define TMPA910_CFG_PORT_GPIO(x) (__REG(PORT_BASE | (x) | PORT_OFS_FR1) = 0)

/********/
#define PORTB_BASE       (PORT_BASE + 0x1000)

#define PORTB_GPIODATA	 (PORTB_BASE + PORT_OFS_DATA)


#define PORTF_BASE  			0xF0805000
#define PORTF_GPIOFDIR		(PORTF_BASE + 0x0400)
#define PORTF_GPIOFFR     (PORTF_BASE + 0x0424)
#define PORTF_GPIOFODE    (PORTF_BASE + 0x0c00)


/********/
#define PORTD_BASE  		0xF0803000
#define PORTD_GPIOFR1 		(PORTD_BASE + 0x0424)
#define PORTD_GPIOFR2 		(PORTD_BASE + 0x0428)
#define PORTD_GPIOIE  		(PORTD_BASE + 0x0810)
#define PORTD_GPIOIC  		(PORTD_BASE + 0x081C)
#define PORTD_GPIOMIS  		(PORTD_BASE + 0x0818)
/********/
#define PORTE_BASE  		0xF0804000
#define PORTE_GPIOEFR 		(PORTE_BASE + 0x0424)

/********/
#define PORTG_BASE  			0xF0806000
#define PORTG_GPIOFR	 (PORTG_BASE + 0x0424)



/* Timer */
#define TMPA910_TIMER0 0xf0040000

/* LCD Controller */
#define LCDC_BASE 0xf4200000


/* I2C Ports */
#define I2C0_BASE   0xF0070000
#define I2C1_BASE   0xF0071000

/* Camera sensor controller */
#define CMOSCAM_BASE  0xF2020000

/* DMA */
#define DMAC_BASE  0xF4100000

/* System Control */
#define SYSCTRL_BASE  (0xF0050000)

/* NAND Flash Controller */
#define NANDF_BASE  0xF2010000

#define NDFMCR0  __REG(NANDF_BASE | 0x0000) /* NAND-Flash Control Register 0 */
#define NDFMCR1  __REG(NANDF_BASE | 0x0004) /* NAND-Flash Control Register 1 */
#define NDFMCR2  __REG(NANDF_BASE | 0x0008) /* NAND-Flash Control Register 2 */
#define NDFINTC  __REG(NANDF_BASE | 0x000C) /* NAND-Flash Interrupt Control Register */
#define NDFDTR   __REG(NANDF_BASE | 0x0010) /* NAND-Flash Data Register */
#define NDECCRD0 __REG(NANDF_BASE | 0x0020) /* NAND-Flash ECC Read Register 0 */
#define NDECCRD1 __REG(NANDF_BASE | 0x0024) /* NAND-Flash ECC Read Register 1 */
#define NDECCRD2 __REG(NANDF_BASE | 0x0028) /* NAND-Flash ECC Read Register 2 */
#define NDRSCA0  __REG(NANDF_BASE | 0x0030) /* NAND-Flash Reed-Solomon Calculation Result Address Register 0 */
#define NDRSCD0  __REG(NANDF_BASE | 0x0034) /* NAND-Flash Reed-Solomon Calculation Result Data Register 0 */
#define NDRSCA1  __REG(NANDF_BASE | 0x0038) /* NAND-Flash Reed-Solomon Calculation Result Address Register 1 */
#define NDRSCD1  __REG(NANDF_BASE | 0x003C) /* NAND-Flash Reed-Solomon Calculation Result Data Register 1 */
#define NDRSCA2  __REG(NANDF_BASE | 0x0040) /* NAND-Flash Reed-Solomon Calculation Result Address Register 2 */
#define NDRSCD2  __REG(NANDF_BASE | 0x0044) /* NAND-Flash Reed-Solomon Calculation Result Data Register 2 */
#define NDRSCA3  __REG(NANDF_BASE | 0x0048) /* NAND-Flash Reed-Solomon Calculation Result Address Register 3 */
#define NDRSCD3  __REG(NANDF_BASE | 0x004C) /* NAND-Flash Reed-Solomon Calculation Result Data Register 3 */



/* LCDDA (LCD Data Process Accelerator) */
#define LCDDA_BASE  0xF2050000

/* Interrupt Controller */
#define INTR_BASE  0xF4000000

/* Touchscreen Controller */
#define TS_BASE   0xf00601f0
#define TOUCHSCREEN_BASE   0xF00601F0

/* ADC */
#define ADC_BASE  0xf0080000

/* SDRAM */
#define SRAM_BASE      0xF8002000
#define SRAM_SIZE      0x0000C000

#endif /* __TMPA910_REGS__ */
