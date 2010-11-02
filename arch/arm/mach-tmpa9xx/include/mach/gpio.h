#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H
/*
 * arch/arm/mach-tmpa9xx/include/mach/gpio.h
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

#define gpio_to_irq tmpa9xx_gpio_to_irq
#define irq_to_gpio tmpa9xx_irq_to_gpio

int tmpa9xx_irq_to_gpio(unsigned irq);
int tmpa9xx_gpio_to_irq(unsigned gpio);
int tmpa9xx_gpio_init(void);

#endif
