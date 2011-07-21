/*
 * linux/arch/arm/mach-tmpa9xx/gpio.c
 *
 * Generic TMPA9xx GPIO handling
 *
 * Copyright (c) 2009, 2010 Florian Boor <florian.boor@kernelconcepts.de>
 * Copyright (c) 2011 Michael Hunold <michael@mihu.de>
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
 * The same applies to interrupts.
 *
 * TODO: Allow sharing beween GPIO and non-GPIO IRQs
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <mach/regs.h>
#include <mach/gpio.h>
#include <asm/gpio.h>
#include <mach/irqs.h>


struct tmpa9xx_gpio_chip {
	struct gpio_chip chip;

	int intr_vect;

	unsigned int data_reg;     /* Associated data register for GPIO bank */
	unsigned int data_dir_reg; /* Register for pin direction setting     */
	unsigned int status_reg;   /* IRQ status register     */

	unsigned int input_mask;   /* Bits that are allowed to be input */
	unsigned int output_mask;  /* Bits that are allowed to be output */
	unsigned int irq_mask;     /* Bits that can trigger an interrupt */
};

#define to_tmpa9xx_gpio_chip(c) container_of(c, struct tmpa9xx_gpio_chip, chip)


static int tmpa9xx_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct tmpa9xx_gpio_chip *tmpa9xx_chip = to_tmpa9xx_gpio_chip(chip);
	unsigned long flags;
	u8 v;

	if (!(tmpa9xx_chip->input_mask & (1 << offset)))
		return -EINVAL;

	local_irq_save(flags);
	v = __raw_readb(tmpa9xx_chip->data_dir_reg);
	v &= ~(1 << offset);
	__raw_writeb(v, tmpa9xx_chip->data_dir_reg);
	local_irq_restore(flags);

	return 0;
}

static int tmpa9xx_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset, int val)
{
	struct tmpa9xx_gpio_chip *tmpa9xx_chip = to_tmpa9xx_gpio_chip(chip);
	unsigned long flags;
	u8 v;

	if (!(tmpa9xx_chip->output_mask & (1 << offset)))
		return -EINVAL;

	local_irq_save(flags);

	/* Set the value */
	v = __raw_readb(tmpa9xx_chip->data_reg);
	if (val)
		v |= (1 << offset);
	else
		v &= ~(1 << offset);
	__raw_writeb(v, tmpa9xx_chip->data_reg);

	/* check if it can generate an interrupt, disable int in this case. */
	if (tmpa9xx_chip->irq_mask & (1 << offset)) {
		v = __raw_readb(tmpa9xx_chip->data_reg + 0x414);
		v &= ~(1 << offset);
		__raw_writeb(v, tmpa9xx_chip->data_reg + 0x414);
	}

	/* Set the direction */
	v = __raw_readb(tmpa9xx_chip->data_dir_reg);
	v |= (1 << offset);
	__raw_writeb(v, tmpa9xx_chip->data_dir_reg);

	local_irq_restore(flags);

	return 0;
}

static int tmpa9xx_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct tmpa9xx_gpio_chip *tmpa9xx_chip = to_tmpa9xx_gpio_chip(chip);

	return !!(__raw_readb(tmpa9xx_chip->data_reg) & (1 << offset));
}

static void tmpa9xx_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	struct tmpa9xx_gpio_chip *tmpa9xx_chip = to_tmpa9xx_gpio_chip(chip);
	unsigned long flags;
	u8 v;

    	if (!chip) {
		printk(KERN_ERR "GPIO chip undefined - unable to continue.\n");
		BUG();
	}

	/* TODO: We could use the clever address based writing function here. */
	local_irq_save(flags);
	v = __raw_readb(tmpa9xx_chip->data_reg);
	if (val)
		v |= (1 << offset);
	else
		v &= ~(1 << offset);
	__raw_writeb(v, tmpa9xx_chip->data_reg);
	local_irq_restore(flags);
}

static void tmpa9xx_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct tmpa9xx_gpio_chip *tmpa9xx_chip = to_tmpa9xx_gpio_chip(chip);
	u8 data_reg, data_dir_reg;
	int i;

	data_reg = __raw_readb(tmpa9xx_chip->data_reg);
	data_dir_reg = __raw_readb(tmpa9xx_chip->data_dir_reg);

	for (i = 0; i < chip->ngpio; i++)
		seq_printf(s, "GPIO %s%d: %s %s\n", chip->label, i,
			   (data_reg & (1 << i)) ? "set" : "clear",
			   (data_dir_reg & (1 << i)) ? "out" : "in");
}


/*
 * GPIO Interrupts
 */

struct tmpa9xx_gpio_irq {
	unsigned int gpio;
	unsigned int port;
	unsigned int bit;
	int intr_vect;
};


/* Nothing is obvious here. We need to know quite a lot. */
#ifdef CONFIG_CPU_TMPA900
static struct tmpa9xx_gpio_irq irq_gpio_desc[TMPA9XX_NUM_GPIO_IRQS] = {
	{   0, PORTA, 0, INTR_VECT_GPIOA },
	{   1, PORTA, 1, INTR_VECT_GPIOA },
	{   2, PORTA, 2, INTR_VECT_GPIOA },
	{   3, PORTA, 3, INTR_VECT_GPIOA },
	{  23, PORTC, 7, INTR_VECT_GPIOC },
	{  30, PORTD, 6, INTR_VECT_GPIOD },
	{  31, PORTD, 7, INTR_VECT_GPIOD },
	{  47, PORTF, 7, INTR_VECT_GPIOF },
	{ 108, PORTN, 4, INTR_VECT_GPION },
	{ 109, PORTN, 5, INTR_VECT_GPION },
	{ 110, PORTN, 6, INTR_VECT_GPION },
	{ 111, PORTN, 7, INTR_VECT_GPION },
	{ 114, PORTR, 2, INTR_VECT_GPIOR },
};
#endif

#ifdef CONFIG_CPU_TMPA910
static struct tmpa9xx_gpio_irq irq_gpio_desc[TMPA9XX_NUM_GPIO_IRQS] = {
	{   0, PORTA, 0, INTR_VECT_GPIOA },
	{   1, PORTA, 1, INTR_VECT_GPIOA },
	{   2, PORTA, 2, INTR_VECT_GPIOA },
	{   3, PORTA, 3, INTR_VECT_GPIOA },
	{   4, PORTA, 4, INTR_VECT_GPIOA },
	{   5, PORTA, 5, INTR_VECT_GPIOA },
	{   6, PORTA, 6, INTR_VECT_GPIOA },
	{   7, PORTA, 7, INTR_VECT_GPIOA },
	{  21, PORTC, 5, INTR_VECT_GPIOC },
	{  23, PORTC, 7, INTR_VECT_GPIOC },
	{  30, PORTD, 6, INTR_VECT_GPIOD },
	{  31, PORTD, 7, INTR_VECT_GPIOD },
	{  47, PORTF, 7, INTR_VECT_GPIOF },
	{ 108, PORTN, 4, INTR_VECT_GPION },
	{ 109, PORTN, 5, INTR_VECT_GPION },
	{ 110, PORTN, 6, INTR_VECT_GPION },
	{ 111, PORTN, 7, INTR_VECT_GPION },
	{ 112, PORTP, 0, INTR_VECT_GPIOP },
	{ 113, PORTP, 1, INTR_VECT_GPIOP },
	{ 114, PORTP, 2, INTR_VECT_GPIOP },
	{ 115, PORTP, 3, INTR_VECT_GPIOP },
	{ 116, PORTP, 4, INTR_VECT_GPIOP },
	{ 117, PORTP, 5, INTR_VECT_GPIOP },
	{ 118, PORTP, 6, INTR_VECT_GPIOP },
	{ 119, PORTP, 7, INTR_VECT_GPIOP },
	{ 122, PORTR, 2, INTR_VECT_GPIOP },
};
#endif

#define GPIO_NUM_FOR_GPIO_IRQ(_x) (irq_gpio_desc[_x].gpio)

int tmpa9xx_gpio_to_irq(unsigned gpio) {
	int i;

	for (i = 0; i < TMPA9XX_NUM_GPIO_IRQS; i++)
		if (irq_gpio_desc[i].gpio == gpio)
			return (i + TMPA9XX_NUM_IRQS);

	return -1;
}

EXPORT_SYMBOL(tmpa9xx_gpio_to_irq);

int tmpa9xx_irq_to_gpio(unsigned irq) {
	return GPIO_NUM_FOR_GPIO_IRQ(irq - TMPA9XX_NUM_IRQS);
}

EXPORT_SYMBOL(tmpa9xx_irq_to_gpio);

/*
 * Interrupt handler
 * The matching GPIO is easy to find: Port base GPIO number + bit offset.
 */
static irqreturn_t interrupt_handler(int irq, void *ptr)
{
	struct tmpa9xx_gpio_chip *irq_chip = ptr;
	unsigned char status;
	int i;

	status = __raw_readb(irq_chip->status_reg);
	for (i = 0; i < 8; i++) {
		if (status & (1 << i)) {
			int gpio_irq = gpio_to_irq(irq_chip->chip.base + i);
			generic_handle_irq(gpio_irq);
		}
	}
	return IRQ_HANDLED;
}

static void tmpa9xx_gpio_irq_ack(unsigned int irq)
{
	unsigned int gpio_irq = irq - TMPA9XX_NUM_IRQS;
	struct tmpa9xx_gpio_irq girq;

	BUG_ON((irq < TMPA9XX_NUM_IRQS) || (gpio_irq >= TMPA9XX_NUM_GPIO_IRQS));

	girq = irq_gpio_desc[gpio_irq];

	__raw_writeb(1 << girq.bit, TMPA9XX_GPIO_REG_IC(girq.port));
}

static void tmpa9xx_gpio_irq_mask(unsigned int irq)
{
	unsigned int gpio_irq = irq - TMPA9XX_NUM_IRQS;
	struct tmpa9xx_gpio_irq girq;
	unsigned char reg;

	BUG_ON((irq < TMPA9XX_NUM_IRQS) || (gpio_irq >= TMPA9XX_NUM_GPIO_IRQS));

	girq = irq_gpio_desc[gpio_irq];
        reg = __raw_readb(TMPA9XX_GPIO_REG_IE(girq.port));
        reg &= ~(1 << girq.bit);
	__raw_writeb(reg, TMPA9XX_GPIO_REG_IE(girq.port));
}

static void tmpa9xx_gpio_irq_unmask(unsigned int irq)
{
	unsigned int gpio_irq = irq - TMPA9XX_NUM_IRQS;
	struct tmpa9xx_gpio_irq girq;
	unsigned char reg;

	BUG_ON((irq < TMPA9XX_NUM_IRQS) || (gpio_irq >= TMPA9XX_NUM_GPIO_IRQS));

	girq = irq_gpio_desc[gpio_irq];

	/* Make sure pin is input */
        reg = __raw_readb(TMPA9XX_GPIO_REG_DIR(girq.port));
        reg &= ~(1 << girq.bit);
	__raw_writeb(reg, TMPA9XX_GPIO_REG_DIR(girq.port));
	/* Enable interrupt function */
        reg = __raw_readb(TMPA9XX_GPIO_REG_IE(girq.port));
        reg |= (1 << girq.bit);
	__raw_writeb(reg, TMPA9XX_GPIO_REG_IE(girq.port));
}

static int tmpa9xx_gpio_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct tmpa9xx_gpio_irq *i = irq_data_get_irq_chip_data(d);

	if (on)
		enable_irq_wake(i->intr_vect);
	else
		disable_irq_wake(i->intr_vect);

	return 0;
}

static int tmpa9xx_gpio_irq_type(unsigned int irq, unsigned int type)
{
	struct irq_desc *desc = irq_desc + irq;
	const int gpio = irq_to_gpio(irq);
	struct tmpa9xx_gpio_irq girq;
	unsigned int gpio_irq = irq - TMPA9XX_NUM_IRQS;
	unsigned char reg_level_sel;
	unsigned char reg_dir_sel;
	unsigned char reg_edge_both;
	unsigned char reg_raise_high;
	unsigned char reg_enable;
	unsigned char port_mask;
	unsigned long flags;

	BUG_ON((irq < TMPA9XX_NUM_IRQS) || (gpio_irq >= TMPA9XX_NUM_GPIO_IRQS));

	girq = irq_gpio_desc[gpio_irq];
        reg_level_sel = __raw_readb(TMPA9XX_GPIO_REG_IS(girq.port));
        reg_edge_both = __raw_readb(TMPA9XX_GPIO_REG_IBE(girq.port));
        reg_raise_high = __raw_readb(TMPA9XX_GPIO_REG_IEV(girq.port));

	port_mask = (1 << girq.bit);

	/* we following the prodcedure mentioned in section 3.9.3 of the data sheet */

	local_irq_save(flags);
	reg_dir_sel = __raw_readb(TMPA9XX_GPIO_REG_DIR(girq.port));
	reg_dir_sel &= ~(1 << girq.bit);
	__raw_writeb(reg_dir_sel, TMPA9XX_GPIO_REG_DIR(girq.port));
	local_irq_restore(flags);

	tmpa9xx_gpio_irq_mask(irq); /* disable interrupt */

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		reg_level_sel &= ~port_mask;
		reg_edge_both &= ~port_mask;
		reg_raise_high |= port_mask;
		desc->handle_irq = handle_edge_irq;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		reg_level_sel &= ~port_mask;
		reg_edge_both &= ~port_mask;
		reg_raise_high &= ~port_mask;
		desc->handle_irq = handle_edge_irq;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		reg_level_sel |= port_mask;
		reg_edge_both &= ~port_mask;
		reg_raise_high |= port_mask;
		desc->handle_irq = handle_level_irq;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		reg_level_sel |= port_mask;
		reg_edge_both &= ~port_mask;
		reg_raise_high &= ~port_mask;
		desc->handle_irq = handle_level_irq;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		reg_level_sel &= ~port_mask;
		reg_edge_both |= port_mask;
		desc->handle_irq = handle_edge_irq;
		break;
	default:
		pr_err("tmpa9xx: failed to set irq type %d for gpio %d\n",
		       type, gpio);
		return -EINVAL;
	}

	/* apply settings */
	__raw_writeb(reg_level_sel, TMPA9XX_GPIO_REG_IS(girq.port));
	__raw_writeb(reg_edge_both, TMPA9XX_GPIO_REG_IBE(girq.port));
	__raw_writeb(reg_raise_high, TMPA9XX_GPIO_REG_IEV(girq.port));

	tmpa9xx_gpio_irq_ack(irq); /* clear interrupt state */

	desc->status &= ~IRQ_TYPE_SENSE_MASK;
	desc->status |= type & IRQ_TYPE_SENSE_MASK;

        reg_enable = __raw_readb(TMPA9XX_GPIO_REG_IE(girq.port));
        reg_enable |= port_mask;
	__raw_writeb(reg_enable, TMPA9XX_GPIO_REG_IE(girq.port));

	return 0;
}

static struct irq_chip tmpa9xx_gpio_irq_chip = {
	.name		= "GPIO",
	.ack		= tmpa9xx_gpio_irq_ack,
	.mask		= tmpa9xx_gpio_irq_mask,
	.unmask		= tmpa9xx_gpio_irq_unmask,
	.set_type	= tmpa9xx_gpio_irq_type,
	.irq_set_wake	= tmpa9xx_gpio_irq_set_wake,
};


#define TMPA9XX_GPIO_BANK(name, port_base, base_gpio, im, om, irqm, iv)	\
	{								\
		.chip = {						\
			.label		  = name,			\
			.direction_input  = tmpa9xx_gpio_direction_input, \
			.direction_output = tmpa9xx_gpio_direction_output, \
			.get		  = tmpa9xx_gpio_get,		\
			.set		  = tmpa9xx_gpio_set,		\
			.dbg_show	  = tmpa9xx_gpio_dbg_show,	\
			.base		  = base_gpio,			\
			.ngpio		  = 8,				\
		},							\
		.data_reg	= TMPA9XX_GPIO_REG(port_base, PORT_OFS_DATA),	\
		.data_dir_reg	= TMPA9XX_GPIO_REG(port_base, PORT_OFS_DIR),\
		.status_reg	= TMPA9XX_GPIO_REG(port_base, PORT_OFS_MIS),	\
		.input_mask	= im,	\
		.output_mask	= om,\
		.irq_mask	= irqm,	\
		.intr_vect	= iv, \
	}

#define INTR_VECT_NONE (-1)

#ifdef CONFIG_CPU_TMPA910
static struct tmpa9xx_gpio_chip tmpa9xx_gpio_banks[] = {
	TMPA9XX_GPIO_BANK("A", PORTA,   0, 0xFF, 0x00, 0xFF, INTR_VECT_GPIOA), /* 8 interrupts */
	TMPA9XX_GPIO_BANK("B", PORTB,   8, 0x00, 0xFF, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("C", PORTC,  16, 0xE0, 0xFF, 0xA0, INTR_VECT_GPIOC), /* 2 interrupts */
	TMPA9XX_GPIO_BANK("D", PORTD,  24, 0xFF, 0x00, 0xC0, INTR_VECT_GPIOD), /* 2 interrupts */
	TMPA9XX_GPIO_BANK("E", PORTE,  32, 0xFF, 0x00, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("F", PORTF,  40, 0xCF, 0xC0, 0x80, INTR_VECT_GPIOF), /* 1 interrupt  */
	TMPA9XX_GPIO_BANK("G", PORTG,  48, 0xFF, 0xFF, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("H", PORTH,  56, 0xFF, 0xFF, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("J", PORTJ,  72, 0x00, 0xFF, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("K", PORTK,  80, 0x00, 0xFF, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("L", PORTL,  88, 0x1F, 0x1F, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("M", PORTM,  96, 0x0F, 0x0F, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("N", PORTN, 104, 0xFF, 0xFF, 0xF0, INTR_VECT_GPION), /* 4 interrupts */
	TMPA9XX_GPIO_BANK("P", PORTP, 112, 0xFF, 0xFF, 0xFF, INTR_VECT_GPIOP), /* 8 interrupts */
	TMPA9XX_GPIO_BANK("R", PORTR, 120, 0x04, 0x07, 0x04, INTR_VECT_GPIOR), /* 1 interrupt  */
	TMPA9XX_GPIO_BANK("T", PORTT, 128, 0xFF, 0xFF, 0x00, INTR_VECT_NONE),
};
enum gpio_bankidx {
    BANK_NR_PORTA = 0,
    BANK_NR_PORTB,
    BANK_NR_PORTC,
    BANK_NR_PORTD,
    BANK_NR_PORTE,
    BANK_NR_PORTF,
    BANK_NR_PORTG,
    BANK_NR_PORTH,
    BANK_NR_PORTJ,
    BANK_NR_PORTK,
    BANK_NR_PORTL,
    BANK_NR_PORTM,
    BANK_NR_PORTN,
    BANK_NR_PORTP,
    BANK_NR_PORTR,
    BANK_NR_PORTT,
};
#endif
#ifdef CONFIG_CPU_TMPA900
static struct tmpa9xx_gpio_chip tmpa9xx_gpio_banks[] = {
	TMPA9XX_GPIO_BANK("A", PORTA,   0, 0x0F, 0x00, 0x0F, INTR_VECT_GPIOA), /* 4 interrupts */
	TMPA9XX_GPIO_BANK("B", PORTB,   8, 0x00, 0x0F, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("C", PORTC,  16, 0xC0, 0xDC, 0x80, INTR_VECT_GPIOC), /* 1 interrupts */
	TMPA9XX_GPIO_BANK("D", PORTD,  24, 0xFF, 0x00, 0xC0, INTR_VECT_GPIOD), /* 2 interrupts */
	TMPA9XX_GPIO_BANK("F", PORTF,  40, 0xCF, 0xC0, 0x80, INTR_VECT_GPIOF), /* 1 interrupt  */
	TMPA9XX_GPIO_BANK("G", PORTG,  48, 0xFF, 0xFF, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("J", PORTJ,  72, 0xFF, 0xFF, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("K", PORTK,  80, 0xFF, 0xFF, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("L", PORTL,  88, 0x1F, 0x1F, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("M", PORTM,  96, 0x0F, 0x0F, 0x00, INTR_VECT_NONE),
	TMPA9XX_GPIO_BANK("N", PORTN, 104, 0xFF, 0xFF, 0xF0, INTR_VECT_GPION), /* 4 interrupts */
	TMPA9XX_GPIO_BANK("R", PORTR, 112, 0x04, 0x07, 0x04, INTR_VECT_GPIOR), /* 1 interrupt  */
	TMPA9XX_GPIO_BANK("T", PORTT, 120, 0xFF, 0xFF, 0x00, INTR_VECT_NONE),
};
enum gpio_bankidx {
    BANK_NR_PORTA = 0,
    BANK_NR_PORTB,
    BANK_NR_PORTC,
    BANK_NR_PORTD,
    BANK_NR_PORTF,
    BANK_NR_PORTG,
    BANK_NR_PORTJ,
    BANK_NR_PORTK,
    BANK_NR_PORTL,
    BANK_NR_PORTM,
    BANK_NR_PORTN,
    BANK_NR_PORTR,
    BANK_NR_PORTT,
};
#endif

int __init tmpa9xx_gpio_init(void)
{
	int i;
	int gpio_irq;
	int ret;

	/* Register GPIO banks */
	for (i = 0; i < ARRAY_SIZE(tmpa9xx_gpio_banks); i++)
		BUG_ON(gpiochip_add(&tmpa9xx_gpio_banks[i].chip) < 0);

	/* Now the interrupts */
	for (i = 0; i < TMPA9XX_NUM_GPIO_IRQS; i++) {
		gpio_irq = gpio_to_irq(0) + i;
	   	tmpa9xx_gpio_irq_mask(gpio_irq);
		set_irq_chip_data(gpio_irq, (void *)&irq_gpio_desc[i]);
		set_irq_chip(gpio_irq, &tmpa9xx_gpio_irq_chip);
		set_irq_handler(gpio_irq, handle_level_irq);
		set_irq_flags(gpio_irq, IRQF_VALID);
	}

	ret = request_irq(INTR_VECT_GPIOA, interrupt_handler, IRQF_DISABLED, "gpio_a", &tmpa9xx_gpio_banks[BANK_NR_PORTA]);
	if (ret) {
		printk(KERN_ERR "request_irq() @ INTR_VECT_GPIOA failed\n");
	}
	ret = request_irq(INTR_VECT_GPIOC, interrupt_handler, IRQF_DISABLED, "gpio_c", &tmpa9xx_gpio_banks[BANK_NR_PORTC]);
	if (ret) {
		printk(KERN_ERR "request_irq() @ INTR_VECT_GPIOC failed\n");
	}
#if !defined(CONFIG_TOUCHSCREEN_TMPA9XX) && !defined(CONFIG_TOUCHSCREEN_TMPA9XX_MODULE)
	ret = request_irq(INTR_VECT_GPIOD, interrupt_handler, IRQF_DISABLED, "gpio_d", &tmpa9xx_gpio_banks[BANK_NR_PORTD]);
	if (ret) {
		printk(KERN_ERR "request_irq() @ INTR_VECT_GPIOD failed\n");
	}
#endif
	ret = request_irq(INTR_VECT_GPIOF, interrupt_handler, IRQF_DISABLED, "gpio_f", &tmpa9xx_gpio_banks[BANK_NR_PORTF]);
	if (ret) {
		printk(KERN_ERR "request_irq() @ INTR_VECT_GPIOF failed\n");
	}
	ret = request_irq(INTR_VECT_GPION, interrupt_handler, IRQF_DISABLED, "gpio_n", &tmpa9xx_gpio_banks[BANK_NR_PORTN]);
	if (ret) {
		printk(KERN_ERR "request_irq() @ INTR_VECT_GPION failed\n");
	}
	ret = request_irq(INTR_VECT_GPIOR, interrupt_handler, IRQF_DISABLED, "gpio_r", &tmpa9xx_gpio_banks[BANK_NR_PORTR]);
	if (ret) {
		printk(KERN_ERR "request_irq() @ INTR_VECT_GPIOR failed\n");
	}
#ifdef CONFIG_CPU_TMPA910
#if !defined(CONFIG_USB_OHCI_HCD_TMPA9XX) && !defined(CONFIG_USB_OHCI_HCD_TMPA9XX_MODULE)
	ret = request_irq(INTR_VECT_GPIOP, interrupt_handler, IRQF_DISABLED, "gpio_p", &tmpa9xx_gpio_banks[BANK_NR_PORTP]);
	if (ret) {
		printk(KERN_ERR "request_irq() @ INTR_VECT_GPIOP failed\n");
	}
#endif
#endif
	return 0;
}
