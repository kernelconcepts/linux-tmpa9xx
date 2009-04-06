#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H
/*
 * arch/arm/mach-tmpa910/include/mach/gpio.h
 */
/* Use as GPIO_BASE_ADDR(GPIOA)- GPIO_BASE_ADDR(GPIOF)*/
// Bank: portbase+portoffset+register
// irqmask,imask, omask

#define ARCH_NR_GPIOS 136

#include <asm-generic/gpio.h>

#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_cansleep	__gpio_cansleep

/*
 * Map GPIO A0..A7  (0..7)  to irq 64..71,
 *          B0..B7  (7..15) to irq 72..79, and
 *          F0..F7 (16..24) to irq 80..87.
 */
static inline int gpio_to_irq(unsigned gpio)
{
	return -EINVAL;
}

static inline int irq_to_gpio(unsigned irq)
{
	return irq - gpio_to_irq(0);
}

#endif
