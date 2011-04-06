/*
 *  Copyright (C) 2008 bplan GmbH
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

#ifndef __TMPA9XX_IRQS_H__
#define __TMPA9XX_IRQS_H__

#define TMPA9XX_NUM_IRQS	32

#define NR_IRQS		(MAXIRQNUM + 1 + TMPA9XX_NUM_GPIO_IRQS)
#define NR_FIQS		(MAXFIQNUM + 1)

/*
 *  TMPA9xx IRQ definitions
 *  TODO: Move to a board definitions specific file.
 */
#define TOPAS_INT_DM9000       INT_GPIO_INTH
#define TONGA_INT_SMSC911X     INT_GPIO_INTH

/*
 * Chip internal -> not of the board
 */
#define INTR_VECT_WDT         0
#define INTR_VECT_RTC         1
#define INTR_VECT_TIMER01     2
#define INTR_VECT_TIMER23     3
#define INTR_VECT_TIMER45     4
#define INTR_VECT_GPIOD       5
#define INTR_VECT_I2C_CH0     6 
#define INTR_VECT_I2C_CH1     7 
#define INTR_VECT_ADC         8 

#ifdef CONFIG_CPU_TMPA900
#define INTR_VECT_UART_CH2    9
#endif
#define INTR_VECT_UART_CH0    10
#define INTR_VECT_UART_CH1    11
#define INTR_VECT_SSP_CH0     12
#define INTR_VECT_SSP_CH1     13
#define INTR_VECT_NDFC        14
#define INTR_VECT_CMSIF       15
#define INTR_VECT_DMA_ERROR   16
#define INTR_VECT_DMA_END     17
#define INTR_VECT_LCDC        18 

#define INTR_VECT_LCDDA       20 
#define INTR_VECT_USB         21
#define INTR_VECT_SDHC        22 
#define INTR_VECT_I2S         23
#define INTR_VECT_USB_HOST    27

#define INTR_VECT_GPIOR       26
#define INTR_VECT_GPIOP       27
#define INTR_VECT_GPION       28
#define INTR_VECT_GPIOF       29
#define INTR_VECT_GPIOC       30
#define INTR_VECT_GPIOA       31

#define MAXIRQNUM             31 
#define MAXFIQNUM             31

#define DMA_ERR_INT           INTR_VECT_DMA_ERROR
#define DMA_END_INT           INTR_VECT_DMA_END
#define I2S_INT               INTR_VECT_I2S
#define USB_INT               INTR_VECT_USB

/* 
 * GPIO Interrupts
 */

/* Port A */
#define INT_GPIO_KI0	       32
#define INT_GPIO_KI1	       33
#define INT_GPIO_KI2	       34
#define INT_GPIO_KI3	       35

#ifdef CONFIG_CPU_TMPA910
#define INT_GPIO_KI4	       36
#define INT_GPIO_KI5	       37
#define INT_GPIO_KI6	       38
#define INT_GPIO_KI7	       39

/* Port C */
#define INT_GPIO_INT8	       40
#define INT_GPIO_INT9	       41

/* Port D */
#define INT_GPIO_INTA          42
#define INT_GPIO_INTB          43

/* Port F */
#define INT_GPIO_INTC	       44

/* Port N */
#define INT_GPIO_INTD	       45
#define INT_GPIO_INTE	       46
#define INT_GPIO_INTF	       47
#define INT_GPIO_INTG	       48

/* Port P */
#define INT_GPIO_INT0	       49
#define INT_GPIO_INT1	       50
#define INT_GPIO_INT2	       51
#define INT_GPIO_INT3	       52
#define INT_GPIO_INT4	       53
#define INT_GPIO_INT5	       54
#define INT_GPIO_INT6	       55
#define INT_GPIO_INT7	       56

/* Port R */
#define INT_GPIO_INTH	       57

#define TMPA9XX_NUM_GPIO_IRQS  26

#endif /* CONFIG_CPU_TMPA910 */

#ifdef CONFIG_CPU_TMPA900

/* Port C */
#define INT_GPIO_INT9	       36

/* Port D */
#define INT_GPIO_INTA          37
#define INT_GPIO_INTB          38

/* Port F */
#define INT_GPIO_INTC	       39

/* Port N */
#define INT_GPIO_INTD	       40
#define INT_GPIO_INTE	       41
#define INT_GPIO_INTF	       42
#define INT_GPIO_INTG	       43

/* Port R */
#define INT_GPIO_INTH	       44

#define TMPA9XX_NUM_GPIO_IRQS  13

#endif /* CONFIG_CPU_TMPA900 */

#endif
