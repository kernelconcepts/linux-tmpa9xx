/*
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

#ifndef __TMPA9XX_GPIO_H__
#define __TMPA9XX_GPIO_H__

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
