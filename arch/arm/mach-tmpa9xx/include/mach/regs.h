/*
 *  Copyright (C) 2008 bplan GmbH. All rights reserved.
 *  Copyright (C) 2009 Florian Boor <florian.boor@kernelconcepts.de>
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
 */

#ifndef __TMPA9XX_REGS_H__
#define __TMPA9XX_REGS_H__

#include <mach/hardware.h>

/* GPIO Ports */

#define PORT_BASE                 (0xF0800000)
#define PORTA                     0x0000
#define PORTB                     0x1000
#define PORTC                     0x2000
#define PORTD                     0x3000
#ifndef CONFIG_CPU_TMPA900
#define PORTE                     0x4000
#endif
#define PORTF                     0x5000
#define PORTG                     0x6000
#ifndef CONFIG_CPU_TMPA900
#define PORTH                     0x7000
#endif
#define PORTJ                     0x8000
#define PORTK                     0x9000
#define PORTL                     0xA000
#define PORTM                     0xB000
#define PORTN                     0xC000
#define PORTP                     0xD000
#define PORTR                     0xE000
#define PORTT                     0xF000

/* TMPA900 */
#ifdef CONFIG_CPU_TMPA900
#define PORTU                     0x4000
#define PORTV                     0x7000
#endif

#define PORT_OFS_DATA             0x03FC  /* 0x000 - 0x3FC, data register masked from 0x00 to 0xFF << 2 */
#define PORT_OFS_DIR              0x0400  /* direction register */
#define PORT_OFS_FR1              0x0424  /* function register 1 */
#define PORT_OFS_FR2              0x0428  /* function register 2 */
#define PORT_OFS_IS               0x0804  /* interrupt sensitivity */
#define PORT_OFS_IBE              0x0808  /* interrupt both edge register */
#define PORT_OFS_IEV              0x080C  /* interrupt event register */
#define PORT_OFS_IE               0x0810  /* interrupt enable register*/
#define PORT_OFS_RIS              0x0814  /* raw interrupt status register */
#define PORT_OFS_MIS              0x0818  /* masked interrupt status */
#define PORT_OFS_IC               0x081C  /* interrupt clear register */
#define PORT_OFS_ODE              0x0C00  /* open drain output */

#define TMPA9XX_GPIO_REG(x,y)     (PORT_BASE | (x) | (y)) /* base addr + port offset + register offset */

#define TMPA9XX_GPIO_REG_DATA(x)  (PORT_BASE | PORT_OFS_DATA | (x))
#define TMPA9XX_GPIO_REG_DIR(x)   (PORT_BASE | PORT_OFS_DIR | (x))
#define TMPA9XX_GPIO_REG_IS(x)    (PORT_BASE | PORT_OFS_IS | (x))
#define TMPA9XX_GPIO_REG_IBE(x)   (PORT_BASE | PORT_OFS_IBE | (x))
#define TMPA9XX_GPIO_REG_IEV(x)   (PORT_BASE | PORT_OFS_IEV | (x))
#define TMPA9XX_GPIO_REG_IE(x)    (PORT_BASE | PORT_OFS_IE | (x))
#define TMPA9XX_GPIO_REG_MIS(x)   (PORT_BASE | PORT_OFS_MIS | (x))
#define TMPA9XX_GPIO_REG_IC(x)    (PORT_BASE | PORT_OFS_IC | (x))
#define TMPA9XX_GPIO_REG_ODE(x)   (PORT_BASE | PORT_OFS_ODE | (x))

#define TMPA9XX_CFG_PORT_GPIO(x)  {(__REG(PORT_BASE | (x) | PORT_OFS_FR1) = 0); (__REG(PORT_BASE | (x) | PORT_OFS_FR2) = 0);}
#define TMPA9XX_PORT_FR1(x)           __REG(PORT_BASE | (x) | PORT_OFS_FR1)
#define TMPA9XX_PORT_T_FR1            TMPA9XX_PORT_FR1(PORTT)
#define TMPA9XX_PORT_B_ODE  

/******* redundent stuff */
#define PORTB_BASE                (PORT_BASE + 0x1000)
#define PORTB_GPIODATA            (PORTB_BASE + PORT_OFS_DATA)

#define PORTF_BASE                (0xF0805000)
#define PORTF_GPIOFDIR            (PORTF_BASE + 0x0400)
#define PORTF_GPIOFFR             (PORTF_BASE + 0x0424)
#define PORTF_GPIOFODE            (PORTF_BASE + 0x0c00)

/********/
#define PORTD_BASE                (0xF0803000)
#define PORTD_GPIOFR1             (PORTD_BASE + 0x0424)
#define PORTD_GPIOFR2             (PORTD_BASE + 0x0428)
#define PORTD_GPIOIS              (PORTD_BASE + 0x0804)
#define PORTD_GPIOIBE             (PORTD_BASE + 0x0808)
#define PORTD_GPIOIEV             (PORTD_BASE + 0x080C)
#define PORTD_GPIOIE              (PORTD_BASE + 0x0810)
#define PORTD_GPIOIC              (PORTD_BASE + 0x081C)
#define PORTD_GPIOMIS             (PORTD_BASE + 0x0818)
/********/
#define PORTE_BASE                (0xF0804000)
#define PORTE_GPIOEFR             (PORTE_BASE + 0x0424)

/********/
#define PORTG_BASE                (0xF0806000)
#define PORTG_GPIOFR              (PORTG_BASE + 0x0424)


/* GPIO registers */
#define IO_1_GPIO_BASE            (0xF0800000)
#define IO_1_BASE                 (IO_1_GPIO_BASE)
#define GPIO_A_BASE               (IO_1_GPIO_BASE + PORTA)
#define GPIO_B_BASE               (IO_1_GPIO_BASE + PORTB)
#define GPIO_C_BASE               (IO_1_GPIO_BASE + PORTC)
#define GPIO_D_BASE               (IO_1_GPIO_BASE + PORTD)

#ifndef CONFIG_CPU_TMPA900
#define GPIO_E_BASE               (IO_1_GPIO_BASE + PORTE)
#endif

#define GPIO_F_BASE               (IO_1_GPIO_BASE + PORTF)
#define GPIO_G_BASE               (IO_1_GPIO_BASE + PORTG)
#ifndef CONFIG_CPU_TMPA900

#define GPIO_H_BASE               (IO_1_GPIO_BASE + PORTH)
#endif

#define GPIO_J_BASE               (IO_1_GPIO_BASE + PORTJ)
#define GPIO_K_BASE               (IO_1_GPIO_BASE + PORTK)
#define GPIO_L_BASE               (IO_1_GPIO_BASE + PORTL)
#define GPIO_M_BASE               (IO_1_GPIO_BASE + PORTM)
#define GPIO_N_BASE               (IO_1_GPIO_BASE + PORTN)
#define GPIO_P_BASE               (IO_1_GPIO_BASE + PORTP)
#define GPIO_R_BASE               (IO_1_GPIO_BASE + PORTR)
#define GPIO_T_BASE               (IO_1_GPIO_BASE + PORTT)

#ifdef CONFIG_CPU_TMPA900
#define GPIO_U_BASE               (IO_1_GPIO_BASE + PORTU)
#define GPIO_V_BASE               (IO_1_GPIO_BASE + PORTV)
#endif

#define GPIOADATA                 __REG(GPIO_A_BASE + 0x3FC)
#define GPIOADIR                  __REG(GPIO_A_BASE + 0x400)
#define GPIOAFR1                  __REG(GPIO_A_BASE + 0x424)
#define GPIOAFR2                  __REG(GPIO_A_BASE + 0x428)
#define GPIOAIS                   __REG(GPIO_A_BASE + 0x804)
#define GPIOAIBE                  __REG(GPIO_A_BASE + 0x808)
#define GPIOAIEV                  __REG(GPIO_A_BASE + 0x80C)
#define GPIOAIE                   __REG(GPIO_A_BASE + 0x810)
#define GPIOARIS                  __REG(GPIO_A_BASE + 0x814)
#define GPIOAMIS                  __REG(GPIO_A_BASE + 0x818)
#define GPIOAIC                   __REG(GPIO_A_BASE + 0x81C)

#define GPIOBDATA                 __REG(GPIO_B_BASE + 0x3FC)
#define GPIOBDIR                  __REG(GPIO_B_BASE + 0x400)
#define GPIOBFR1                  __REG(GPIO_B_BASE + 0x424)
#define GPIOBFR2                  __REG(GPIO_B_BASE + 0x428)
#define GPIOBODE                  __REG(GPIO_B_BASE + 0xC00)

#define GPIOCDATA                 __REG(GPIO_C_BASE + 0x3FC)
#define GPIOCDIR                  __REG(GPIO_C_BASE + 0x400)
#define GPIOCFR1                  __REG(GPIO_C_BASE + 0x424)
#define GPIOCFR2                  __REG(GPIO_C_BASE + 0x428)
#define GPIOCIS                   __REG(GPIO_C_BASE + 0x804)
#define GPIOCIBE                  __REG(GPIO_C_BASE + 0x808)
#define GPIOCIEV                  __REG(GPIO_C_BASE + 0x80C)
#define GPIOCIE                   __REG(GPIO_C_BASE + 0x810)
#define GPIOCRIS                  __REG(GPIO_C_BASE + 0x814)
#define GPIOCMIS                  __REG(GPIO_C_BASE + 0x818)
#define GPIOCIC                   __REG(GPIO_C_BASE + 0x81C)
#define GPIOCODE                  __REG(GPIO_C_BASE + 0xC00)

#define GPIOC_FR2_PWM2OUT (1 << 4)

#define GPIODDATA                 __REG(GPIO_D_BASE + 0x3FC)
#define GPIODDIR                  __REG(GPIO_D_BASE + 0x400)
#define GPIODFR1                  __REG(GPIO_D_BASE + 0x424)
#define GPIODFR2                  __REG(GPIO_D_BASE + 0x428)
#define GPIODIS                   __REG(GPIO_D_BASE + 0x804)
#define GPIODIBE                  __REG(GPIO_D_BASE + 0x808)
#define GPIODIEV                  __REG(GPIO_D_BASE + 0x80C)
#define GPIODIE                   __REG(GPIO_D_BASE + 0x810)
#define GPIODRIS                  __REG(GPIO_D_BASE + 0x814)
#define GPIODMIS                  __REG(GPIO_D_BASE + 0x818)
#define GPIODIC                   __REG(GPIO_D_BASE + 0x81C)

#ifndef CONFIG_CPU_TMPA900
#define GPIOEDATA                 __REG(GPIO_E_BASE + 0x3FC)
#define GPIOEDIR                  __REG(GPIO_E_BASE + 0x400)
#define GPIOEFR1                  __REG(GPIO_E_BASE + 0x424)
#define GPIOEFR2                  __REG(GPIO_E_BASE + 0x428)
#endif

#define GPIOFDATA                 __REG(GPIO_F_BASE + 0x3FC)
#define GPIOFDIR                  __REG(GPIO_F_BASE + 0x400)
#define GPIOFFR1                  __REG(GPIO_F_BASE + 0x424)
#define GPIOFFR2                  __REG(GPIO_F_BASE + 0x428)
#define GPIOFIS                   __REG(GPIO_F_BASE + 0x804) 
#define GPIOFIBE                  __REG(GPIO_F_BASE + 0x808) 
#define GPIOFIEV                  __REG(GPIO_F_BASE + 0x80C) 
#define GPIOFIE                   __REG(GPIO_F_BASE + 0x810) 
#define GPIOFRIS                  __REG(GPIO_F_BASE + 0x814) 
#define GPIOFMIS                  __REG(GPIO_F_BASE + 0x818) 
#define GPIOFIC                   __REG(GPIO_F_BASE + 0x81C) 
#define GPIOFODE                  __REG(GPIO_F_BASE + 0xC00) 

#define GPIOGDATA                 __REG(GPIO_G_BASE + 0x3FC) 
#define GPIOGDIR                  __REG(GPIO_G_BASE + 0x400) 
#define GPIOGFR1                  __REG(GPIO_G_BASE + 0x424) 
#define GPIOGFR2                  __REG(GPIO_G_BASE + 0x428) 

#ifndef CONFIG_CPU_TMPA900
#define GPIOHDATA                 __REG(GPIO_H_BASE + 0x3FC) 
#define GPIOHDIR                  __REG(GPIO_H_BASE + 0x400)    
#define GPIOHFR1                  __REG(GPIO_H_BASE + 0x424)    
#define GPIOHFR2                  __REG(GPIO_H_BASE + 0x428)    
#endif

#define GPIOJDATA                 __REG(GPIO_J_BASE + 0x3FC)  
#define GPIOJDIR                  __REG(GPIO_J_BASE + 0x400)  
#define GPIOJFR1                  __REG(GPIO_J_BASE + 0x424)  
#define GPIOJFR2                  __REG(GPIO_J_BASE + 0x428)

#define GPIOKDATA                 __REG(GPIO_K_BASE + 0x3FC)
#define GPIOKDIR                  __REG(GPIO_K_BASE + 0x400)
#define GPIOKFR1                  __REG(GPIO_K_BASE + 0x424)
#define GPIOKFR2                  __REG(GPIO_K_BASE + 0x428)

#define GPIOLDATA                 __REG(GPIO_L_BASE + 0x3FC)
#define GPIOLDIR                  __REG(GPIO_L_BASE + 0x400)
#define GPIOLFR1                  __REG(GPIO_L_BASE + 0x424)
#define GPIOLFR2                  __REG(GPIO_L_BASE + 0x428)

#define GPIOMDATA                 __REG(GPIO_M_BASE + 0x3FC)
#define GPIOMDIR                  __REG(GPIO_M_BASE + 0x400)
#define GPIOMFR1                  __REG(GPIO_M_BASE + 0x424)
#define GPIOMFR2                  __REG(GPIO_M_BASE + 0x428)

#define GPIONDATA                 __REG(GPIO_N_BASE + 0x3FC)
#define GPIONDIR                  __REG(GPIO_N_BASE + 0x400)
#define GPIONFR1                  __REG(GPIO_N_BASE + 0x424)
#define GPIONFR2                  __REG(GPIO_N_BASE + 0x428)
#define GPIONIS                   __REG(GPIO_N_BASE + 0x804)
#define GPIONIBE                  __REG(GPIO_N_BASE + 0x808)
#define GPIONIEV                  __REG(GPIO_N_BASE + 0x80C)
#define GPIONIE                   __REG(GPIO_N_BASE + 0x810)
#define GPIONRIS                  __REG(GPIO_N_BASE + 0x814)
#define GPIONMIS                  __REG(GPIO_N_BASE + 0x818)
#define GPIONIC                   __REG(GPIO_N_BASE + 0x81C)

#define GPIOPDATA                 __REG(GPIO_P_BASE + 0x3FC)
#define GPIOPDIR                  __REG(GPIO_P_BASE + 0x400)
#define GPIOPFR1                  __REG(GPIO_P_BASE + 0x424)
#define GPIOPFR2                  __REG(GPIO_P_BASE + 0x428)
#define GPIOPIS                   __REG(GPIO_P_BASE + 0x804)
#define GPIOPIBE                  __REG(GPIO_P_BASE + 0x808)
#define GPIOPIEV                  __REG(GPIO_P_BASE + 0x80C)
#define GPIOPIE                   __REG(GPIO_P_BASE + 0x810)
#define GPIOPRIS                  __REG(GPIO_P_BASE + 0x814)
#define GPIOPMIS                  __REG(GPIO_P_BASE + 0x818)
#define GPIOPIC                   __REG(GPIO_P_BASE + 0x81C)

#define GPIORDATA                 __REG(GPIO_R_BASE + 0x3FC)
#define GPIORDIR                  __REG(GPIO_R_BASE + 0x400)
#define GPIORFR1                  __REG(GPIO_R_BASE + 0x424)
#define GPIORFR2                  __REG(GPIO_R_BASE + 0x428)
#define GPIORIS                   __REG(GPIO_R_BASE + 0x804)
#define GPIORIBE                  __REG(GPIO_R_BASE + 0x808)
#define GPIORIEV                  __REG(GPIO_R_BASE + 0x80C)
#define GPIORIE                   __REG(GPIO_R_BASE + 0x810)
#define GPIORRIS                  __REG(GPIO_R_BASE + 0x814)
#define GPIORMIS                  __REG(GPIO_R_BASE + 0x818)
#define GPIORIC                   __REG(GPIO_R_BASE + 0x81C)

#define GPIOTDATA                 __REG(GPIO_T_BASE + 0x3FC)
#define GPIOTDIR                  __REG(GPIO_T_BASE + 0x400)
#define GPIOTFR1                  __REG(GPIO_T_BASE + 0x424)
#define GPIOTFR2                  __REG(GPIO_T_BASE + 0x428)

/* Port U and V TMPA900 only */
#ifdef CONFIG_CPU_TMPA900
#define GPIOUDATA                 __REG(GPIO_U_BASE + 0x3FC)
#define GPIOUDIR                  __REG(GPIO_U_BASE + 0x400)
#define GPIOUFR1                  __REG(GPIO_U_BASE + 0x424)
#define GPIOUFR2                  __REG(GPIO_U_BASE + 0x428)
#define GPIOVDATA                 __REG(GPIO_U_BASE + 0x3FC)
#define GPIOVDIR                  __REG(GPIO_U_BASE + 0x400)
#define GPIOVFR1                  __REG(GPIO_V_BASE + 0x424)
#define GPIOVFR2                  __REG(GPIO_V_BASE + 0x428)
#endif

/* Timer */
#define TMPA9XX_TIMER0            (0xf0040000)
#define TMPA9XX_TIMER2            (0xf0041000)
#define TMPA9XX_TIMER4            (0xf0042000)

/* LCD Controller */
#define LCDC_BASE 0xf4200000

/* LCD Controller Option Function (unsupported at the moment) */
#define LCDCOP_BASE 0xf00b0000

/* RTC */
#define RTC_BASE	(0xf0030000)

/* Melody / Alarm */
#define MA_BASE                   (0xf0030100)

/* I2C Ports */
#define I2C0_BASE                 (0xF0070000)
#define I2C1_BASE                 (0xF0071000)

/* Camera sensor controller */
#define CMOSCAM_BASE              (0xF2020000)

/* DMA */
#define DMAC_BASE                 (0xF4100000
#define DMACIntTCStatus           __REG(DMAC_BASE | 0x004)
#define DMACIntTCClear            __REG(DMAC_BASE | 0x008)
#define DMACConfiguration         __REG(DMAC_BASE | 0x030)
#define DMACC5SrcAddr             __REG(DMAC_BASE | 0x1a0)
#define DMACC5DestAddr            __REG(DMAC_BASE | 0x1a4)
#define DMACC5Control             __REG(DMAC_BASE | 0x1ac)
#define DMACC5Configuration       __REG(DMAC_BASE | 0x1b0)

/*  Memory controller MPMC0 */
#define SMC_MPMC0_BASE            (0xf4301000)
#define SMC_MEMC_STATUS_3         __REG(SMC_MPMC0_BASE + 0x000)
#define SMC_MEMIF_CFG_3           __REG(SMC_MPMC0_BASE + 0x004)
#define SMC_DIRECT_CMD_3          __REG(SMC_MPMC0_BASE + 0x010)
#define SMC_SET_CYCLES_3          __REG(SMC_MPMC0_BASE + 0x014)
#define SMC_SET_OPMODE_3          __REG(SMC_MPMC0_BASE + 0x018)
#define SMC_SRAM_CYCLES_0_3       __REG(SMC_MPMC0_BASE + 0x100)
#define SMC_SRAM_CYCLES_1_3       __REG(SMC_MPMC0_BASE + 0x120)
#define SMC_SRAM_CYCLES_2_3       __REG(SMC_MPMC0_BASE + 0x140)
#define SMC_SRAM_CYCLES_3_3       __REG(SMC_MPMC0_BASE + 0x160)
#define SMC_OPMODE0_0_3           __REG(SMC_MPMC0_BASE + 0x104)
#define SMC_OPMODE0_1_3           __REG(SMC_MPMC0_BASE + 0x124)
#define SMC_OPMODE0_2_3           __REG(SMC_MPMC0_BASE + 0x144)
#define SMC_OPMODE0_3_3           __REG(SMC_MPMC0_BASE + 0x164)

#define SMC_TIMEOUT               __REG(0xf00a0000 + 0x050)

/*  Memory controller MPMC1 */
#define SMC_MPMC1_BASE            (0xf4311000)
#define SMC_MEMC_STATUS_5         __REG(SMC_MPMC1_BASE + 0x000)
#define SMC_MEMIF_CFG_5           __REG(SMC_MPMC1_BASE + 0x004)
#define SMC_DIRECT_CMD_5          __REG(SMC_MPMC1_BASE + 0x010)
#define SMC_SET_CYCLES_5          __REG(SMC_MPMC1_BASE + 0x014)
#define SMC_SET_OPMODE_5          __REG(SMC_MPMC1_BASE + 0x018)
#define SMC_SRAM_CYCLES_0_5       __REG(SMC_MPMC1_BASE + 0x100)
#define SMC_SRAM_CYCLES_1_5       __REG(SMC_MPMC1_BASE + 0x120)
#define SMC_SRAM_CYCLES_2_5       __REG(SMC_MPMC1_BASE + 0x140)
#define SMC_SRAM_CYCLES_3_5       __REG(SMC_MPMC1_BASE + 0x160)
#define SMC_OPMODE0_0_5           __REG(SMC_MPMC1_BASE + 0x104)
#define SMC_OPMODE0_1_5           __REG(SMC_MPMC1_BASE + 0x124)
#define SMC_OPMODE0_2_5           __REG(SMC_MPMC1_BASE + 0x144)
#define SMC_OPMODE0_3_5           __REG(SMC_MPMC1_BASE + 0x164)

/* I2S Interface   */
#define I2S_BASE                  (0xF2040000)

/* Power Management Controller */
#define PMC_BASE_ADDRESS         (0xF0020000)
#define PMCDRV                   __REG(PMC_BASE_ADDRESS + 0x0260)
#define PMCCTL                   __REG(PMC_BASE_ADDRESS + 0x0300)
#define PMCWV1                   __REG(PMC_BASE_ADDRESS + 0x0400)
#define PMCRES                   __REG(PMC_BASE_ADDRESS + 0x041C)

#define PMCCTL_PMCPWE  (1 << 6)
#define PMCWV1_PMCCTLV (1 << 5)

/* System Control / PLL */
#define PLL_BASE_ADDRESS        0xF0050000

/* Watchdog Timer registers */
#define WDT_BASE_ADDRESS        (0xF0010000)

/* NAND Flash Controller */
#define NANDF_BASE  0xF2010000

#define NDFMCR0                 __REG(NANDF_BASE | 0x0000) /* NAND-Flash Control Register 0 */
#define NDFMCR1                 __REG(NANDF_BASE | 0x0004) /* NAND-Flash Control Register 1 */
#define NDFMCR2                 __REG(NANDF_BASE | 0x0008) /* NAND-Flash Control Register 2 */
#define NDFINTC                 __REG(NANDF_BASE | 0x000C) /* NAND-Flash Interrupt Control Register */
#define NDFDTR                  __REG(NANDF_BASE | 0x0010) /* NAND-Flash Data Register */
#define NDECCRD0                __REG(NANDF_BASE | 0x0020) /* NAND-Flash ECC Read Register 0 */
#define NDECCRD1                __REG(NANDF_BASE | 0x0024) /* NAND-Flash ECC Read Register 1 */
#define NDECCRD2                __REG(NANDF_BASE | 0x0028) /* NAND-Flash ECC Read Register 2 */
#define NDRSCA0                 __REG(NANDF_BASE | 0x0030) /* NAND-Flash Reed-Solomon Calculation Result Address Register 0 */
#define NDRSCD0                 __REG(NANDF_BASE | 0x0034) /* NAND-Flash Reed-Solomon Calculation Result Data Register 0 */
#define NDRSCA1                 __REG(NANDF_BASE | 0x0038) /* NAND-Flash Reed-Solomon Calculation Result Address Register 1 */
#define NDRSCD1                 __REG(NANDF_BASE | 0x003C) /* NAND-Flash Reed-Solomon Calculation Result Data Register 1 */
#define NDRSCA2                 __REG(NANDF_BASE | 0x0040) /* NAND-Flash Reed-Solomon Calculation Result Address Register 2 */
#define NDRSCD2                 __REG(NANDF_BASE | 0x0044) /* NAND-Flash Reed-Solomon Calculation Result Data Register 2 */
#define NDRSCA3                 __REG(NANDF_BASE | 0x0048) /* NAND-Flash Reed-Solomon Calculation Result Address Register 3 */
#define NDRSCD3                 __REG(NANDF_BASE | 0x004C) /* NAND-Flash Reed-Solomon Calculation Result Data Register 3 */

#define NDFDTR_PHY              (NANDF_BASE + 0x10)

#define NDFMCR0_ECCRST          (1 << 0)
#define NDFMCR0_BUSY            (1 << 1)
#define NDFMCR0_ECCE            (1 << 2)
 
#define NDFMCR0_CE1             (1 << 3)
#define NDFMCR0_CE0             (1 << 4)
#define NDFMCR0_CLE             (1 << 5)
#define NDFMCR0_ALE             (1 << 6)
#define NDFMCR0_WE              (1 << 7)
#define NDFMCR0_RSEDN           (1 << 10)

#define NDFMCR1_ECCS            (1 << 1)
#define NDFMCR1_SELAL           (1 << 9)
#define NDFMCR1_ALS             (1 << 8)

#define NAND_DMAC_STATUS        (1 << 5)
#define NAND_DMAC_CLEAR         (1 << 5)

/* added for MLC */
#define NDFMCR0_ECC_RSM_ON      0x394
#define NDFMCR0_ECC_RSECGW_ON   0x194
#define NDFMCR0_ECC_RSECGW_OFF  0x94
#define NDFINTC_LATCH_CLEAR     0x99
#define NDFINTC_RSERIS          0x20
#define NDFINTC_RSEIC           0x80
#define INTC_EN                 1
#define INTC_DIS                0

/* Interrupt Controller */
#define INTR_BASE               (0xF4000000)

/* LCDDA (LCD Data Process Accelerator) */
#define LCDDA_BASE              (0xf2050000)

/* Touchscreen Controller */
#define TS_BASE                 (0xF0060000)

/* ADC */
#define ADC_BASE                (0xf0080000)

/* SDRAM */
#define SRAM_BASE               (0xF8002000)
#define SRAM_SIZE               (0x0000C000)

/* DMA registers */
#define DMA_BASE                (0xF4100000)
#define DMA_INT_STATUS          __REG(DMA_BASE)
#define DMA_TC_STATUS           __REG(DMA_BASE + 0x0004)
#define DMA_TC_CLEAR            __REG(DMA_BASE + 0x0008)
#define DMA_ERR_STATUS          __REG(DMA_BASE + 0x000c)
#define DMA_ERR_CLEAR           __REG(DMA_BASE + 0x0010)
#define DMA_RAW_TC_STATUS       __REG(DMA_BASE + 0x0014)
#define DMA_RAW_ERR_STATUS      __REG(DMA_BASE + 0x0018)
#define DMA_ENABLED_CHN         __REG(DMA_BASE + 0x001c)
#define DMA_CONFIGURE           __REG(DMA_BASE + 0x0030)

#define DMA_SRC_ADDR(x)         __REG(DMA_BASE + 0x100 + ((x) << 5))
#define DMA_DEST_ADDR(x)        __REG(DMA_BASE + 0x100 + ((x) << 5) + 0x04)
#define DMA_LLI(x)              __REG(DMA_BASE + 0x100 + ((x) << 5) + 0x08)
#define DMA_CONTROL(x)          __REG(DMA_BASE + 0x100 + ((x) << 5) + 0x0c)
#define DMA_CONFIG(x)           __REG(DMA_BASE + 0x100 + ((x) << 5) + 0x10)

#define DMA_CONFIG_EN                (1 << 0)

/* I2S : 0xf2040000   */
#define I2S_BASE                (0xF2040000)

/* UART registers: 0xF200x000 */
#define UART0_BASE_ADDRESS     (0xF2000000)
#define UART1_BASE_ADDRESS     (0xF2001000)
#define UART2_BASE_ADDRESS     (0xF2004000)

/* USB Host */
#define USB_HOST_BASE_ADDRESS  (0xF4500000)
/* fixme: remove me, too */
#define HCBCR0                 __REG(USB_HOST_BASE_ADDRESS + 0x0080)

/* USB Device */
#define USB_DEVICE_BASE_ADDRESS  (0xF4400000)

/* OFD */
#define OFD_BASE_ADDRESS       (0xF0090000)

/* SSP */
#define SSP0_BASE_ADDRESS       (0xF2002000)
#define SSP1_BASE_ADDRESS       (0xF2003000)

#endif
