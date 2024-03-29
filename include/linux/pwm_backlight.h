/*
 * Generic PWM backlight driver data - see drivers/video/backlight/pwm_bl.c
 */
#ifndef __LINUX_PWM_BACKLIGHT_H
#define __LINUX_PWM_BACKLIGHT_H

struct platform_pwm_backlight_data {
	const char *name;
	unsigned int max_brightness;
	unsigned int dft_brightness;
	unsigned int lth_brightness;
	unsigned int pwm_period_ns;
	bool inverted_polarity;
	int (*init)(struct device *dev);
	int (*notify)(struct device *dev, int brightness);
	void (*exit)(struct device *dev);
};

#endif
