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

	unsigned int data_reg;
	unsigned int data_dir_reg;
    
	unsigned int input_mask;
	unsigned int output_mask;
};

#define to_tmpa910_gpio_chip(c) container_of(c, struct tmpa910_gpio_chip, chip)

static int tmpa910_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct tmpa910_gpio_chip *tmpa910_chip = to_tmpa910_gpio_chip(chip);
	unsigned long flags;
	u8 v;

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

#define TMPA910_GPIO_BANK(name, port_base, num, base_gpio)	\
	{								\
		.chip = {						\
			.label		  = name,			\
			.direction_input  = tmpa910_gpio_direction_input, \
			.direction_output = tmpa910_gpio_direction_output, \
			.get		  = tmpa910_gpio_get,		\
			.set		  = tmpa910_gpio_set,		\
			.dbg_show	  = tmpa910_gpio_dbg_show,	\
			.base		  = base_gpio,			\
			.ngpio		  = num,				\
		},							\
		.data_reg	= TMPA910_GPIO_REG(port_base, PORT_OFS_DATA),	\
		.data_dir_reg	= TMPA910_GPIO_REG(port_base, PORT_OFS_DIR),\
	}

static struct tmpa910_gpio_chip tmpa910_gpio_banks[] = {
	TMPA910_GPIO_BANK("A", PORTA, 8, 0),
	TMPA910_GPIO_BANK("B", PORTB, 8, 8),
	TMPA910_GPIO_BANK("C", PORTC, 8, 16),
	TMPA910_GPIO_BANK("D", PORTD, 8, 24),
	TMPA910_GPIO_BANK("E", PORTE, 8, 32),
	TMPA910_GPIO_BANK("F", PORTF, 8, 40),
	TMPA910_GPIO_BANK("G", PORTG, 8, 48),
	TMPA910_GPIO_BANK("H", PORTH, 8, 56),
	TMPA910_GPIO_BANK("J", PORTJ, 8, 72),
	TMPA910_GPIO_BANK("K", PORTK, 8, 80),
	TMPA910_GPIO_BANK("L", PORTL, 8, 88),
	TMPA910_GPIO_BANK("M", PORTM, 8, 96),
	TMPA910_GPIO_BANK("N", PORTN, 8, 104),
	TMPA910_GPIO_BANK("P", PORTP, 8, 112),
	TMPA910_GPIO_BANK("R", PORTR, 8, 120),
	TMPA910_GPIO_BANK("T", PORTT, 8, 128),
};

void __init tmpa910_gpio_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tmpa910_gpio_banks); i++)
		gpiochip_add(&tmpa910_gpio_banks[i].chip);
}

arch_initcall(tmpa910_gpio_init);
