#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H
/*
 * arch/arm/mach-tmpa910/include/mach/gpio.h
 */

#define ARCH_NR_GPIOS 136

#include <asm-generic/gpio.h>
#include <mach/irqs.h>

#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_cansleep	__gpio_cansleep

/*
 * Map GPIO A0..A7  (0..7)  to irq 64..71,
 *          B0..B7  (7..15) to irq 72..79, and
 *          F0..F7 (16..24) to irq 80..87.
 */

#define gpio_to_irq __tmpa910_gpio_to_irq
#define irq_to_gpio __tmpa910_irq_to_gpio
extern int __tmpa910_irq_to_gpio(unsigned irq);
extern int __tmpa910_gpio_to_irq(unsigned gpio);
/*static inline int gpio_to_irq(unsigned gpio)
{
	return TMPA910_NUM_IRQS + gpio;
}

static inline int irq_to_gpio(unsigned irq)
{
	return irq - TMPA910_NUM_IRQS;
}
*/
#endif
