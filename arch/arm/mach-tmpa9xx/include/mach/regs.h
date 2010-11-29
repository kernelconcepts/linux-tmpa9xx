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
 * TMPA9xx register header
 */

#ifndef __TMPA9XX_REGS__
#define __TMPA9XX_REGS__

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

#define GPIOFDATA                 __REG(GPIO_F_BASE + 0x000)
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
#define GPIOUFR1                  __REG(GPIO_U_BASE + 0x424)
#define GPIOUFR2                  __REG(GPIO_U_BASE + 0x428)
#define GPIOVFR1                  __REG(GPIO_V_BASE + 0x424)
#define GPIOVFR2                  __REG(GPIO_V_BASE + 0x428)
#endif

/* Timer */
#define TMPA9XX_TIMER0            (0xf0040000)
#define TMPA9XX_TIMER2            (0xf0041000)
#define TMPA9XX_TIMER4            (0xf0042000)

/* LCD Controller */
#define LCDC_BASE 0xf4200000

/* LCD Controller Option Function */
#define LCDCOP_BASE 0xf00b0000
#define LCDCOP_STN64CR             __REG(LCDCOP_BASE + 0x000)
#define LCDCOP_STN64CR_G64_8bit    (1 << 1)


/* RTC */
#define RTC_BASE                   (0xF0030000)
#define RTCDATA                    (RTC_BASE + 0x0000)             /* RTC Data Register */
#define RTCCOMP                    (RTC_BASE + 0x0004)             /* RTC Compare Register */
#define RTCPRST                    (RTC_BASE + 0x0008)             /* RTC Preset Register */
#define MLDALMINV                  __REG(RTC_BASE + 0x0100)        /* Melody Alarm Invert Register */
#define MLDALMSEL                  __REG(RTC_BASE + 0x0104)        /* Melody Alarm signal Select Register */ 
#define ALMCNTCR                   __REG(RTC_BASE + 0x0108)        /* Alarm Counter Control Register */
#define ALMPATERN                  __REG(RTC_BASE + 0x010C)        /* Alarm Pattern Register */
#define MLDCNTCR                   __REG(RTC_BASE + 0x0110)        /* Melody Counter Control Register */
#define MLDFRQ                     __REG(RTC_BASE + 0x0114)        /* Melody Frequency Register */
#define RTCALMINTCTR              (RTC_BASE + 0x0200)              /* RTC ALM Interrupt Control Register */
#define RTCALMMIS                 (RTC_BASE + 0x0204)              /* RTC ALM Interrupt Status Register */

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
#define I2STCON                   __REG(I2S_BASE + 0x000)
#define I2STSLVON                 __REG(I2S_BASE + 0x004)
#define I2STFCLR                  __REG(I2S_BASE + 0x008)
#define I2STMS                    __REG(I2S_BASE + 0x00C)
#define I2STMCON                  __REG(I2S_BASE + 0x010)
#define I2STMSTP                  __REG(I2S_BASE + 0x014)
#define I2STDMA1                  __REG(I2S_BASE + 0x018)
#define I2SRCON                   __REG(I2S_BASE + 0x020)
#define I2SRSLVON                 __REG(I2S_BASE + 0x024)
#define I2SFRFCLR                 __REG(I2S_BASE + 0x028)
#define I2SRMS                    __REG(I2S_BASE + 0x02C)
#define I2SRMCON                  __REG(I2S_BASE + 0x030)
#define I2SRMSTP                  __REG(I2S_BASE + 0x034)
#define I2SRDMA1                  __REG(I2S_BASE + 0x038)
#define I2SCOMMON                 __REG(I2S_BASE + 0x044) 
#define I2STST                    __REG(I2S_BASE + 0x048)
#define I2SRST                    __REG(I2S_BASE + 0x04C)
#define I2SINT                    __REG(I2S_BASE + 0x050)
#define I2SINTMSK                 __REG(I2S_BASE + 0x054)
#define I2STDAT                   __REG(I2S_BASE + 0x1000)
#define I2SRDAT                   __REG(I2S_BASE + 0x2000)
#define I2STDAT_ADR              (I2S_BASE + 0x1000)
#define I2SRDAT_ADR              (I2S_BASE + 0x2000)


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
#define SYSCR0                  __REG(PLL_BASE_ADDRESS + 0x000)
#define SYSCR1                  __REG(PLL_BASE_ADDRESS + 0x004)
#define SYSCR2                  __REG(PLL_BASE_ADDRESS + 0x008)
#define SYSCR3                  __REG(PLL_BASE_ADDRESS + 0x00C)
#define SYSCR4                  __REG(PLL_BASE_ADDRESS + 0x010)
#define SYSCR5                  __REG(PLL_BASE_ADDRESS + 0x014)
#define SYSCR6                  __REG(PLL_BASE_ADDRESS + 0x018)
#define SYSCR7                  __REG(PLL_BASE_ADDRESS + 0x01C)
#define SYSCR8                  __REG(PLL_BASE_ADDRESS + 0x020)
#define CLKCR5                  __REG(PLL_BASE_ADDRESS + 0x054)


/* timer registers */
#define TIME0_BASE_ADDRESS      (0xF0040000)
#define TIME0_LD                __REG(TIME0_BASE_ADDRESS + 0x0000)
#define TIME0_DATA              __REG(TIME0_BASE_ADDRESS + 0x0004)
#define TIME0_CONTROL           __REG(TIME0_BASE_ADDRESS + 0x0008)
#define TIME0_INT_CLR           __REG(TIME0_BASE_ADDRESS + 0x000C)
#define TIME0_IRQ_REQ           __REG(TIME0_BASE_ADDRESS + 0x0010)
#define TIME0_IRQ_MASK          __REG(TIME0_BASE_ADDRESS + 0x0014)
#define TIME0_MODE              __REG(TIME0_BASE_ADDRESS + 0x001C)
#define TIME0_CAPINTCLR         __REG(TIME0_BASE_ADDRESS + 0x0040)
#define TIME0_CAPEN             __REG(TIME0_BASE_ADDRESS + 0x0060)
#define TIME0_CMPINTCLR         __REG(TIME0_BASE_ADDRESS + 0x00A0)
#define TIME0_CMPEN             __REG(TIME0_BASE_ADDRESS + 0x00E0)

#define TIME1_CONTROL           __REG(TIME0_BASE_ADDRESS + 0x0108)
#define TIME1_BASE_ADDRESS      (0xF0040100)
#define TIMER1_LOAD             __REG(TIME1_BASE_ADDRESS + 0x0000)
#define TIMER1_VALUE            __REG(TIME1_BASE_ADDRESS + 0x0004)
#define TIMER1_CONTROL          __REG(TIME1_BASE_ADDRESS + 0x0008)
#define TIMER1_INTCLR           __REG(TIME1_BASE_ADDRESS + 0x000C)
#define TIMER1_RIS              __REG(TIME1_BASE_ADDRESS + 0x0010)
#define TIMER1_MIS              __REG(TIME1_BASE_ADDRESS + 0x0014)

#define TIME2_BASE_ADDRESS      (0xF0041000)
#define TIMER2_LOAD             __REG(TIME2_BASE_ADDRESS + 0x0000)
#define TIMER2_VALUE            __REG(TIME2_BASE_ADDRESS + 0x0004)
#define TIMER2_CONTROL          __REG(TIME2_BASE_ADDRESS + 0x0008)
#define TIMER2_INTCLR           __REG(TIME2_BASE_ADDRESS + 0x000C)
#define TIMER2_RIS              __REG(TIME2_BASE_ADDRESS + 0x0010)
#define TIMER2_MIS              __REG(TIME2_BASE_ADDRESS + 0x0014)
#define TIMER2_BGLOAD           __REG(TIME2_BASE_ADDRESS + 0x0018)
#define TIMER2_MODE             __REG(TIME2_BASE_ADDRESS + 0x001C)
#define TIMER2_COMPARE_1        __REG(TIME2_BASE_ADDRESS + 0x00A0)
#define TIMER2_CMPINTCLR_1      __REG(TIME2_BASE_ADDRESS + 0x00C0)
#define TIMER2_CMPEN            __REG(TIME2_BASE_ADDRESS + 0x00E0)
#define TIMER2_CMP_RIS          __REG(TIME2_BASE_ADDRESS + 0x00E4)
#define TIMER2_CMP_MIS          __REG(TIME2_BASE_ADDRESS + 0x00E8)
#define TIMER2_BGCMP            __REG(TIME2_BASE_ADDRESS + 0x00EC)

#define TIME3_BASE_ADDRESS      (0xF0041100)
#define TIMER3_LOAD             __REG(TIME3_BASE_ADDRESS + 0x0000)
#define TIMER3_VALUE            __REG(TIME3_BASE_ADDRESS + 0x0004)
#define TIMER3_CONTROL          __REG(TIME3_BASE_ADDRESS + 0x0008)
#define TIMER3_INTCLR           __REG(TIME3_BASE_ADDRESS + 0x000C)
#define TIMER3_RIS              __REG(TIME3_BASE_ADDRESS + 0x0010)
#define TIMER3_MIS              __REG(TIME3_BASE_ADDRESS + 0x0014)
#define TIMER3_BGLOAD           __REG(TIME3_BASE_ADDRESS + 0x0018)

#define TIME4_BASE_ADDRESS      (0xF0042000)
#define TIMER4_LOAD             __REG(TIME4_BASE_ADDRESS + 0x0000)
#define TIMER4_VALUE            __REG(TIME4_BASE_ADDRESS + 0x0004)
#define TIMER4_CONTROL          __REG(TIME4_BASE_ADDRESS + 0x0008)
#define TIMER4_INTCLR           __REG(TIME4_BASE_ADDRESS + 0x000C)
#define TIMER4_RIS              __REG(TIME4_BASE_ADDRESS + 0x0010)
#define TIMER4_MIS              __REG(TIME4_BASE_ADDRESS + 0x0014)
#define TIMER4_BGLOAD           __REG(TIME4_BASE_ADDRESS + 0x0018)

#define TIME5_BASE_ADDRESS      (0xF0042100)
#define TIMER5_LOAD             __REG(TIME5_BASE_ADDRESS + 0x0000)
#define TIMER5_VALUE            __REG(TIME5_BASE_ADDRESS + 0x0004)
#define TIMER5_CONTROL          __REG(TIME5_BASE_ADDRESS + 0x0008)
#define TIMER5_INTCLR           __REG(TIME5_BASE_ADDRESS + 0x000C)
#define TIMER5_RIS              __REG(TIME5_BASE_ADDRESS + 0x0010)
#define TIMER5_MIS              __REG(TIME5_BASE_ADDRESS + 0x0014)
#define TIMER5_BGLOAD           __REG(TIME5_BASE_ADDRESS + 0x0018)


/* Watchdog Timer registers */
#define WDT_BASE_ADDRESS        (0xF0010000)
#define WDT_WDOGLOAD            __REG(WDT_BASE_ADDRESS + 0x0000) /* Watchdog load register */
#define WDT_WDOGVALUE           __REG(WDT_BASE_ADDRESS + 0x0004) /* The current value for the watchdog counter */
#define WDT_WDOGCONTROL         __REG(WDT_BASE_ADDRESS + 0x0008) /* Watchdog control register */
#define WDT_WDOGINTCLR          __REG(WDT_BASE_ADDRESS + 0x000C) /* Clears the watchdog interrupt */
#define WDT_WDOGRIS             __REG(WDT_BASE_ADDRESS + 0x0010) /* Watchdog raw interrupt status */
#define WDT_WDOGMIS             __REG(WDT_BASE_ADDRESS + 0x0014) /* Watchdog masked interrupt status */
#define WDT_WDOGLOCK            __REG(WDT_BASE_ADDRESS + 0x0C00) /* Watchdog Lock register */

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
#define VICINTENABLE            (0xF4000010)
#define VICADDRESS              __REG(INTR_BASE + 0x0F00)


/* LCDDA (LCD Data Process Accelerator) */
#define LCDDA_BASE              (0xF2050000)
#define LCDDA_LDACR0            __REG(LCDDA_BASE+0x00)
#define LCDDA_LDADRSRC1         __REG(LCDDA_BASE+0x04)
#define LCDDA_LDADRSRC0         __REG(LCDDA_BASE+0x08)
#define LCDDA_LDAFCPSRC1        __REG(LCDDA_BASE+0x0c)
#define LCDDA_LDAEFCPSRC1       __REG(LCDDA_BASE+0x10)
#define LCDDA_LDADVSRC0         __REG(LCDDA_BASE+0x14)
#define LCDDA_LDADVSRC1         __REG(LCDDA_BASE+0x18)
#define LCDDA_LDADXDST          __REG(LCDDA_BASE+0x1c)
#define LCDDA_LDADYDST          __REG(LCDDA_BASE+0x20)
#define LCDDA_LDASSIZE          __REG(LCDDA_BASE+0x24)
#define LCDDA_LDADSIZE          __REG(LCDDA_BASE+0x28)
#define LCDDA_LDAS0AD           __REG(LCDDA_BASE+0x2c)
#define LCDDA_LDADAD            __REG(LCDDA_BASE+0x30)
#define LCDDA_LDACR1            __REG(LCDDA_BASE+0x34)
#define LCDDA_LDACR2            __REG(LCDDA_BASE+0x38)


/* Touchscreen Controller */
#define TS_BASE                 (0xF00601F0)
#define TOUCHSCREEN_BASE        (0xF00601F0)

/* ADC */
#define ADC_BASE                (0xf0080000)
#define ADC_ADREG0L             __REG(ADC_BASE + 0x0000) /* A/D conversion result lower-order register 0 */
#define ADC_ADREG0H             __REG(ADC_BASE + 0x0004) /* A/D conversion result higher-order register 0 */
#define ADC_ADREG1L             __REG(ADC_BASE + 0x0008) /* A/D conversion result lower-order register 1 */
#define ADC_ADREG1H             __REG(ADC_BASE + 0x000c) /* A/D conversion result higher-order register 1 */
#define ADC_ADREG2L             __REG(ADC_BASE + 0x0010) /* A/D conversion result lower-order register 2 */
#define ADC_ADREG2H             __REG(ADC_BASE + 0x0014) /* A/D conversion result higher-order register 2 */
#define ADC_ADREG3L             __REG(ADC_BASE + 0x0018) /* A/D conversion result lower-order register 3 */
#define ADC_ADREG3H             __REG(ADC_BASE + 0x001c) /* A/D conversion result higher-order register 3 */
#define ADC_ADREG4L             __REG(ADC_BASE + 0x0020) /* A/D conversion result lower-order register 4 */
#define ADC_ADREG4H             __REG(ADC_BASE + 0x0024) /* A/D conversion result higher-order register 4 */
#define ADC_ADREG5L             __REG(ADC_BASE + 0x0028) /* A/D conversion result lower-order register 5 */
#define ADC_ADREG5H             __REG(ADC_BASE + 0x002c) /* A/D conversion result higher-order register 5 */
#define ADC_ADREG6L             __REG(ADC_BASE + 0x0030) /* A/D conversion result lower-order register 6 */
#define ADC_ADREG6H             __REG(ADC_BASE + 0x0034) /* A/D conversion result higher-order register 6 */
#define ADC_ADREG7L             __REG(ADC_BASE + 0x0038) /* A/D conversion result lower-order register 7 */
#define ADC_ADREG7H             __REG(ADC_BASE + 0x003c) /* A/D conversion result higher-order register 7 */
#define ADC_ADREGSPL            __REG(ADC_BASE + 0x0040) /* Top-priority A/D conversion result lower-order register */
#define ADC_ADREGSPH            __REG(ADC_BASE + 0x0044) /* Top-priority A/D conversion result higher-order register */
#define ADC_ADCOMREGL           __REG(ADC_BASE + 0x0048) /* A/D conversion result comparison lower-order register */
#define ADC_ADCOMREGH           __REG(ADC_BASE + 0x004c) /* A/D conversion result comparison lower-order register */
#define ADC_ADMOD0              __REG(ADC_BASE + 0x0050) /* A/D mode control register 0 */
#define ADC_ADMOD1              __REG(ADC_BASE + 0x0054) /* A/D mode control register 1 */
#define ADC_ADMOD2              __REG(ADC_BASE + 0x0058) /* A/D mode control register 2 */
#define ADC_ADMOD3              __REG(ADC_BASE + 0x005c) /* A/D mode control register 3 */
#define ADC_ADMOD4              __REG(ADC_BASE + 0x0060) /* A/D mode control register 4 */
#define ADC_ADCLK               __REG(ADC_BASE + 0x0070) /* A/D conversion clock setting register */
#define ADC_ADIE                __REG(ADC_BASE + 0x0074) /* A/D interrupt enable register */
#define ADC_ADIS                __REG(ADC_BASE + 0x0078) /* A/D interrupt status register */
#define ADC_ADIC                __REG(ADC_BASE + 0x007c) /* A/D interrupt clear register */

#define ADC_ADREGxL(X)          __REG(ADC_BASE + X*0x08) 
#define ADC_ADREGxH(X)          __REG(ADC_BASE + X*0x08 + 0x04) 

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


/* I2C_1 : 0xf0071000        */
/* to be removed - handled in I2C bus driver */
/*#define        I2C1_BASE                            (0xF0071000)         */
#define I2C1CR1                 __REG(I2C1_BASE + 0x00)
#define I2C1DBR                 __REG(I2C1_BASE + 0x04)
#define I2C1AR                  __REG(I2C1_BASE + 0x08)
#define I2C1CR2                 __REG(I2C1_BASE + 0x0c)
#define I2C1SR                  __REG(I2C1_BASE + 0x0c)
#define I2C1PRS                 __REG(I2C1_BASE + 0x10)
#define I2C1IE                  __REG(I2C1_BASE + 0x14)
#define I2C1IR                  __REG(I2C1_BASE + 0x18)

/* I2S : 0xf2040000   */
#define I2S_BASE                (0xF2040000)
#define I2STCON                 __REG(I2S_BASE + 0x000)
#define I2STSLVON               __REG(I2S_BASE + 0x004)
#define I2STFCLR                __REG(I2S_BASE + 0x008)
#define I2STMS                  __REG(I2S_BASE + 0x00C)
#define I2STMCON                __REG(I2S_BASE + 0x010)
#define I2STMSTP                __REG(I2S_BASE + 0x014)
#define I2STDMA1                __REG(I2S_BASE + 0x018)
#define I2SRCON                 __REG(I2S_BASE + 0x020)
#define I2SRSLVON               __REG(I2S_BASE + 0x024)
#define I2SFRFCLR               __REG(I2S_BASE + 0x028)
#define I2SRMS                  __REG(I2S_BASE + 0x02C)
#define I2SRMCON                __REG(I2S_BASE + 0x030)
#define I2SRMSTP                __REG(I2S_BASE + 0x034)
#define I2SRDMA1                __REG(I2S_BASE + 0x038)
#define I2SCOMMON               __REG(I2S_BASE + 0x044) 
#define I2STST                  __REG(I2S_BASE + 0x048)
#define I2SRST                  __REG(I2S_BASE + 0x04C)
#define I2SINT                  __REG(I2S_BASE + 0x050)
#define I2SINTMSK               __REG(I2S_BASE + 0x054)
#define I2STDAT                 __REG(I2S_BASE + 0x1000)
#define I2SRDAT                 __REG(I2S_BASE + 0x2000)
#define I2STDAT_ADR            (I2S_BASE + 0x1000)
#define I2SRDAT_ADR            (I2S_BASE + 0x2000)

/* UART registers: 0xF200x000 */
#define UART0_BASE_ADDRESS     (0xF2000000)
#define UART0DR                __REG(UART0_BASE_ADDRESS + 0x000)
#define UART0RSR               __REG(UART0_BASE_ADDRESS + 0x004)
#define UART0ECR               __REG(UART0_BASE_ADDRESS + 0x004)
#define UART0FR                __REG(UART0_BASE_ADDRESS + 0x018)
#define UART0ILPR              __REG(UART0_BASE_ADDRESS + 0x020)
#define UART0IBRD              __REG(UART0_BASE_ADDRESS + 0x024)
#define UART0FBRD              __REG(UART0_BASE_ADDRESS + 0x028)
#define UART0LCR_H             __REG(UART0_BASE_ADDRESS + 0x02c)
#define UART0CR                __REG(UART0_BASE_ADDRESS + 0x030)
#define UART0IFLS              __REG(UART0_BASE_ADDRESS + 0x034)
#define UART0IMSC              __REG(UART0_BASE_ADDRESS + 0x038)
#define UART0RIS               __REG(UART0_BASE_ADDRESS + 0x03c)
#define UART0MIS               __REG(UART0_BASE_ADDRESS + 0x040)
#define UART0ICR               __REG(UART0_BASE_ADDRESS + 0x044)
#define UART0DMACR             __REG(UART0_BASE_ADDRESS + 0x048)
#define UART0PeriphID0         __REG(UART0_BASE_ADDRESS + 0xfe0)
#define UART0PeriphID1         __REG(UART0_BASE_ADDRESS + 0xfe4)
#define UART0PeriphID2         __REG(UART0_BASE_ADDRESS + 0xfe8)
#define UART0PeriphID3         __REG(UART0_BASE_ADDRESS + 0xfec)
#define UART0PCellID0          __REG(UART0_BASE_ADDRESS + 0xff0)
#define UART0PCellID1          __REG(UART0_BASE_ADDRESS + 0xff4)
#define UART0PCellID2          __REG(UART0_BASE_ADDRESS + 0xff8)
#define UART0PCellID3          __REG(UART0_BASE_ADDRESS + 0xffc)

#define UART1_BASE_ADDRESS     (0xF2001000)
#define UART1DR                __REG(UART1_BASE_ADDRESS + 0x000)
#define UART1RSR               __REG(UART1_BASE_ADDRESS + 0x004)
#define UART1ECR               __REG(UART1_BASE_ADDRESS + 0x004)
#define UART1FR                __REG(UART1_BASE_ADDRESS + 0x018)
#define UART1ILPR              __REG(UART1_BASE_ADDRESS + 0x020)
#define UART1IBRD              __REG(UART1_BASE_ADDRESS + 0x024)
#define UART1FBRD              __REG(UART1_BASE_ADDRESS + 0x028)
#define UART1LCR_H             __REG(UART1_BASE_ADDRESS + 0x02c)
#define UART1CR                __REG(UART1_BASE_ADDRESS + 0x030)
#define UART1IFLS              __REG(UART1_BASE_ADDRESS + 0x034)
#define UART1IMSC              __REG(UART1_BASE_ADDRESS + 0x038)
#define UART1RIS               __REG(UART1_BASE_ADDRESS + 0x03c)
#define UART1MIS               __REG(UART1_BASE_ADDRESS + 0x040)
#define UART1ICR               __REG(UART1_BASE_ADDRESS + 0x044)
#define UART1DMACR             __REG(UART1_BASE_ADDRESS + 0x048)
#define UART1PeriphID0         __REG(UART1_BASE_ADDRESS + 0xfe0)
#define UART1PeriphID1         __REG(UART1_BASE_ADDRESS + 0xfe4)
#define UART1PeriphID2         __REG(UART1_BASE_ADDRESS + 0xfe8)
#define UART1PeriphID3         __REG(UART1_BASE_ADDRESS + 0xfec)
#define UART1PCellID0          __REG(UART1_BASE_ADDRESS + 0xff0)
#define UART1PCellID1          __REG(UART1_BASE_ADDRESS + 0xff4)
#define UART1PCellID2          __REG(UART1_BASE_ADDRESS + 0xff8)
#define UART1PCellID3          __REG(UART1_BASE_ADDRESS + 0xffc)


#define UART2_BASE_ADDRESS     (0xF2004000)
#define UART2DR                __REG(UART2_BASE_ADDRESS + 0x000)


/* USB Host */
#define USB_HOST_BASE_ADDRESS  (0xF4500000)
#define HCREVISION             __REG(USB_HOST_BASE_ADDRESS + 0x0000)
#define HHCCONTROL             __REG(USB_HOST_BASE_ADDRESS + 0x0004)
#define HCCOMMANDSTATUS        __REG(USB_HOST_BASE_ADDRESS + 0x0008)
#define HCINTERRUPTSTATUS      __REG(USB_HOST_BASE_ADDRESS + 0x000C)
#define HCINTERRUPTENABLE      __REG(USB_HOST_BASE_ADDRESS + 0x0010)
#define HCINTERRUPTDISABLE     __REG(USB_HOST_BASE_ADDRESS + 0x0014)
#define HCHCCA                 __REG(USB_HOST_BASE_ADDRESS + 0x0018)
#define HCPERIODCURRENTED      __REG(USB_HOST_BASE_ADDRESS + 0x001C)
#define HCCONTROLHEADED        __REG(USB_HOST_BASE_ADDRESS + 0x0020)
#define HCCONTROLCURRENTED     __REG(USB_HOST_BASE_ADDRESS + 0x0024)
#define HCBULKHEADED           __REG(USB_HOST_BASE_ADDRESS + 0x0028)
#define HCBULKCURRENTED        __REG(USB_HOST_BASE_ADDRESS + 0x002C)
#define HCDONEHEAD             __REG(USB_HOST_BASE_ADDRESS + 0x0030)
#define HCFMINTERVALL          __REG(USB_HOST_BASE_ADDRESS + 0x0034)
#define HCFMREMAINING          __REG(USB_HOST_BASE_ADDRESS + 0x0038)
#define HCFMNUMBER             __REG(USB_HOST_BASE_ADDRESS + 0x003C)
#define HCPERIODSTART          __REG(USB_HOST_BASE_ADDRESS + 0x0040)
#define HCLSTHRESHOLD          __REG(USB_HOST_BASE_ADDRESS + 0x0044)
#define HCRHDESCRIPTORA        __REG(USB_HOST_BASE_ADDRESS + 0x0048)
#define HCRHDESCRIPTORB        __REG(USB_HOST_BASE_ADDRESS + 0x004C)
#define HCRHSTATUS             __REG(USB_HOST_BASE_ADDRESS + 0x0050)
#define HCRHPORTSTATUS         __REG(USB_HOST_BASE_ADDRESS + 0x0054)
#define HCBCR0                 __REG(USB_HOST_BASE_ADDRESS + 0x0080)

/* USB Device */
#define USB_DEVICE_BASE_ADDRESS  (0xF4400000)

/* OFD */
#define OFD_BASE_ADDRESS       (0xF0090000)
#define OFD_CLKSCR1            __REG(OFD_BASE_ADDRESS + 0x0000)
#define OFD_CLKSCR2            __REG(OFD_BASE_ADDRESS + 0x0004)
#define OFD_CLKSCR3            __REG(OFD_BASE_ADDRESS + 0x0008)
#define OFD_CLKSMN             __REG(OFD_BASE_ADDRESS + 0x0010)
#define OFD_CLKSMX             __REG(OFD_BASE_ADDRESS + 0x0020)

/* SSP */
#define SSP0_BASE_ADDRESS       (0xF2002000)
#define SSP0_CR0                __REG(OFD_BASE_ADDRESS + 0x0000) /* SSP0 Control register 0 */
#define SSP0_CR1                __REG(OFD_BASE_ADDRESS + 0x0004) /* SSP0 Control register 1 */
#define SSP0_DR                 __REG(OFD_BASE_ADDRESS + 0x0008) /* SSP0 Data register */
#define SSP0_SR                 __REG(OFD_BASE_ADDRESS + 0x000C) /* SSP0 Status register */
#define SSP0_CPSR               __REG(OFD_BASE_ADDRESS + 0x0010) /* SSP0 Clock prescale register */
#define SSP0_IMSC               __REG(OFD_BASE_ADDRESS + 0x0014) /* SSP0 Interrupt mask set and clear register */
#define SSP0_RIS                __REG(OFD_BASE_ADDRESS + 0x0018) /* SSP0 Raw interrupt status register */
#define SSP0_MIS                __REG(OFD_BASE_ADDRESS + 0x001C) /* SSP0 Masked interrupt status register */
#define SSP0_ICR                __REG(OFD_BASE_ADDRESS + 0x0020) /* SSP0 Interrupt clear register */
#define SSP0_DMACR              __REG(OFD_BASE_ADDRESS + 0x0024) /* DMA Control register */

#define SSP1_BASE_ADDRESS       (0xF2003000)
#define SSP1_CR0                __REG(OFD_BASE_ADDRESS + 0x0000) /* SSP1 Control register 0 */
#define SSP1_CR1                __REG(OFD_BASE_ADDRESS + 0x0004) /* SSP1 Control register 1 */
#define SSP1_DR                 __REG(OFD_BASE_ADDRESS + 0x0008) /* SSP1 Data register */
#define SSP1_SR                 __REG(OFD_BASE_ADDRESS + 0x000C) /* SSP1 Status register */
#define SSP1_CPSR               __REG(OFD_BASE_ADDRESS + 0x0010) /* SSP1 Clock prescale register */
#define SSP1_IMSC               __REG(OFD_BASE_ADDRESS + 0x0014) /* SSP1 Interrupt mask set and clear register */
#define SSP1_RIS                __REG(OFD_BASE_ADDRESS + 0x0018) /* SSP1 Raw interrupt status register */
#define SSP1_MIS                __REG(OFD_BASE_ADDRESS + 0x001C) /* SSP1 Masked interrupt status register */
#define SSP1_ICR                __REG(OFD_BASE_ADDRESS + 0x0020) /* SSP1 Interrupt clear register */
#define SSP1_DMACR              __REG(OFD_BASE_ADDRESS + 0x0024) /* DMA Control register */


#endif /* __TMPA9XX_REGS__ */
