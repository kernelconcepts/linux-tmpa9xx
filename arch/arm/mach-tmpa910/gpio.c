/*
 * linux/arch/arm/mach-tmpa910/gpio.c
 *
 * Generic TMPA910 / TMPA910CR / TMPA910CRAXBG GPIO handling
 *
 * Copyright (c) 2009 Florian Boor <florian.boor@kernelconcepts.de>
 *
 * Based on mach-ep93xx/gpio.c
 * Copyright (c) 2008 Ryan Mallon <ryan@bluewatersys.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * GPIO functions implementation
 *
 * The GPIO ports are are organized in 16 ports of 8bit each, even if the 
 * are all 32bit. Not all ports allow all functions / directions.
 */

/* TODO: Interrupt support, check valid modes */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/io.h>

#include <mach/tmpa910_regs.h>
#include <mach/gpio.h>
#include <asm/gpio.h>

struct tmpa910_gpio_chip {
	struct gpio_chip chip;

	unsigned int data_reg;     /* Associated data register for GPIO bank */	  
	unsigned int data_dir_reg; /* Register for pin direction setting     */
    
	unsigned int input_mask;   /* Bits that are allowed to be input */
	unsigned int output_mask;  /* Bits that are allowed to be output */
	unsigned int irq_mask;     /* Bits that can trigger an interrupt */
};

#define to_tmpa910_gpio_chip(c) container_of(c, struct tmpa910_gpio_chip, chip)

static int tmpa910_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct tmpa910_gpio_chip *tmpa910_chip = to_tmpa910_gpio_chip(chip);
	unsigned long flags;
	u8 v;

	if (!(tmpa910_chip->input_mask & (1 << offset)))
		return -EINVAL;
    
	local_irq_save(flags);
	v = __raw_readb(tmpa910_chip->data_dir_reg);
	v &= ~(1 << offset);
	__raw_writeb(v, tmpa910_chip->data_dir_reg);
	local_irq_restore(flags);

	return 0;
}

static int tmpa910_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset, int val)
{
	struct tmpa910_gpio_chip *tmpa910_chip = to_tmpa910_gpio_chip(chip);
	unsigned long flags;
//	int line;
	u8 v;

	if (!(tmpa910_chip->output_mask & (1 << offset)))
		return -EINVAL;

	local_irq_save(flags);

	/* Set the value */
	v = __raw_readb(tmpa910_chip->data_reg);
	if (val)
		v |= (1 << offset);
	else
		v &= ~(1 << offset);
	__raw_writeb(v, tmpa910_chip->data_reg);

#if 0
	/* Drive as an output */
	line = chip->base + offset;
	if (line <= TMPA910_GPIO_LINE_MAX_IRQ) {
		/* Ports A/B/F */
		tmpa910_gpio_int_mask(line);
		tmpa910_gpio_update_int_params(line >> 3);
	}
#endif
	v = __raw_readb(tmpa910_chip->data_dir_reg);
	v |= (1 << offset);
	__raw_writeb(v, tmpa910_chip->data_dir_reg);

	local_irq_restore(flags);

	return 0;
}

static int tmpa910_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct tmpa910_gpio_chip *tmpa910_chip = to_tmpa910_gpio_chip(chip);

	return !!(__raw_readb(tmpa910_chip->data_reg) & (1 << offset));
}

static void tmpa910_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	struct tmpa910_gpio_chip *tmpa910_chip = to_tmpa910_gpio_chip(chip);
	unsigned long flags;
	u8 v;

	/* TODO: We could use the clever address based writing function here. */
	local_irq_save(flags);
	v = __raw_readb(tmpa910_chip->data_reg);
	if (val)
		v |= (1 << offset);
	else
		v &= ~(1 << offset);
	__raw_writeb(v, tmpa910_chip->data_reg);
	local_irq_restore(flags);
}

static void tmpa910_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct tmpa910_gpio_chip *tmpa910_chip = to_tmpa910_gpio_chip(chip);
	u8 data_reg, data_dir_reg;
	int i;

	data_reg = __raw_readb(tmpa910_chip->data_reg);
	data_dir_reg = __raw_readb(tmpa910_chip->data_dir_reg);

	for (i = 0; i < chip->ngpio; i++)
		seq_printf(s, "GPIO %s%d: %s %s\n", chip->label, i,
			   (data_reg & (1 << i)) ? "set" : "clear",
			   (data_dir_reg & (1 << i)) ? "out" : "in");
}

#define TMPA910_GPIO_BANK(name, port_base, base_gpio, im, om, irqm)	\
	{								\
		.chip = {						\
			.label		  = name,			\
			.direction_input  = tmpa910_gpio_direction_input, \
			.direction_output = tmpa910_gpio_direction_output, \
			.get		  = tmpa910_gpio_get,		\
			.set		  = tmpa910_gpio_set,		\
			.dbg_show	  = tmpa910_gpio_dbg_show,	\
			.base		  = base_gpio,			\
			.ngpio		  = 8,				\
		},							\
		.data_reg	= TMPA910_GPIO_REG(port_base, PORT_OFS_DATA),	\
		.data_dir_reg	= TMPA910_GPIO_REG(port_base, PORT_OFS_DIR),\
		.input_mask	= im,	\
		.output_mask	= om,\
		.irq_mask	= irqm,	\
	}

static struct tmpa910_gpio_chip tmpa910_gpio_banks[] = {
	TMPA910_GPIO_BANK("A", PORTA, 0,  0xFF, 0x00, 0xFF),
	TMPA910_GPIO_BANK("B", PORTB, 8,  0x00, 0xFF, 0x00),
	TMPA910_GPIO_BANK("C", PORTC, 16, 0xE0, 0xFF, 0xA0),
	TMPA910_GPIO_BANK("D", PORTD, 24, 0xFF, 0x00, 0xC0),
	TMPA910_GPIO_BANK("E", PORTE, 32, 0xFF, 0x00, 0x00),
	TMPA910_GPIO_BANK("F", PORTF, 40, 0xCF, 0xC0, 0x80),
	TMPA910_GPIO_BANK("G", PORTG, 48, 0xFF, 0xFF, 0x00),
	TMPA910_GPIO_BANK("H", PORTH, 56, 0xFF, 0xFF, 0x00),
	TMPA910_GPIO_BANK("J", PORTJ, 72, 0x00, 0xFF, 0x00),
	TMPA910_GPIO_BANK("K", PORTK, 80, 0x00, 0xFF, 0x00),
	TMPA910_GPIO_BANK("L", PORTL, 88, 0x1F, 0x1F, 0x00),
	TMPA910_GPIO_BANK("M", PORTM, 96, 0x0F, 0x0F, 0x00),
	TMPA910_GPIO_BANK("N", PORTN, 104, 0xFF, 0xFF, 0xF0),
	TMPA910_GPIO_BANK("P", PORTP, 112, 0xFF, 0xFF, 0xFF),
	TMPA910_GPIO_BANK("R", PORTR, 120, 0x04, 0x07, 0x04),
	TMPA910_GPIO_BANK("T", PORTT, 128, 0xFF, 0xFF, 0x00),
};

static int __init tmpa910_gpio_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tmpa910_gpio_banks); i++)
		gpiochip_add(&tmpa910_gpio_banks[i].chip);
    
	return 0;
}

arch_initcall(tmpa910_gpio_init);
