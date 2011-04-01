/*
 * PWM support for TMPA9xx
 *
 * Copyright (C) 2010 Nils Faerber <nils.faerber@kernelconcepts.de>
 * Copyright (C) 2011 Michael Hunold <michael@mihu.de>
 *
 * This program is free software; you may redistribute and/or modify
 * it under the terms of the GNU General Public License Version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm/pwm.h>
#include <asm/div64.h>

#include <mach/timer.h>

static int prescaler_clock_khz_0;
module_param(prescaler_clock_khz_0, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(prescaler_clock_khz_0, "pwm prescaler clock in khz");

static int prescaler_clock_khz_1;
module_param(prescaler_clock_khz_1, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(prescaler_clock_khz_1, "pwm prescaler clock in khz");

#define DRIVER_NAME KBUILD_MODNAME

#define tmr_writel(b, o, v)	writel(v, b->regs + o)
#define tmr_readl(b, o)		readl(b->regs + o)

struct tmpa9xx_pwm_priv {
	void __iomem *regs;
	struct device *dev;
	struct resource *r;
	struct clk *clk;
	struct pwm_device *pwm;
	struct pwm_device_ops ops;

	unsigned long active : 1;
	unsigned long polarity : 1;
};

static void start(struct pwm_device *p)
{
	struct tmpa9xx_pwm_priv *pp = pwm_get_drvdata(p);
	uint32_t val;

	dev_dbg(pp->dev, "%s(): active %d\n", __func__, pp->active);

	val = tmr_readl(pp, TIMER_MODE);
	val |= (0x01 << 6);	/* select PWM mode */
	tmr_writel(pp, TIMER_MODE, val);

	val = tmr_readl(pp, TIMER_CONTROL);
	val |=   ((0x01 << 1)	/* 16 bit counter */
		| (0x01 << 6)	/* Timer0 periodic timer */
		| (0x01 << 7));	/* Timer0 enable */
	tmr_writel(pp, TIMER_CONTROL, val);

	val = tmr_readl(pp, TIMER_CMPEN);
	val |= (0x01 << 0); /* enable compare */
	tmr_writel(pp, TIMER_CMPEN, val);

	pp->active = 1;
	set_bit(PWM_FLAG_RUNNING, &p->flags);
}

static int stop_sync(struct pwm_device *p)
{
	struct tmpa9xx_pwm_priv *pp = pwm_get_drvdata(p);
	uint32_t val;

	dev_dbg(pp->dev, "%s(): active %d\n", __func__, pp->active);

	if (!pp->active)
		return 0;

	val = tmr_readl(pp, TIMER_CONTROL);
	val &=   ~((0x01 << 1)	/* deselect 16 bit counter */
		| (0x01 << 6)	/* disable Timer0 periodic timer */
		| (0x01 << 7));	/* disable Timer0 enable */
	tmr_writel(pp, TIMER_CONTROL, val);

	val = tmr_readl(pp, TIMER_MODE);
	val &= ~(0x01 << 6);	/* deselect PWM mode */
	tmr_writel(pp, TIMER_MODE, val);

	val = tmr_readl(pp, TIMER_CMPEN);
	val &= ~(0x01 << 0); /* disable compare */
	tmr_writel(pp, TIMER_CMPEN, val);

	clear_bit(PWM_FLAG_RUNNING, &p->flags);

	return 1;
}

struct pwm_calc
{
	/* in/out */
	unsigned long period_ticks;
	unsigned long duty_ticks;
	/* out */
	int period;
	int prescaler;
};

static void calc(struct tmpa9xx_pwm_priv *pp, struct pwm_calc *c)
{
	int pwm_period[] = {255, 511, 1023, 65535};
	int prescaler[] = {1, 16, 256};
	int i;
	int j;

	int approx_i;
	int approx_j;
	int approx_diff = INT_MAX;

	uint64_t c_period_ticks;
	uint64_t c_duty_ticks;
	uint64_t div;
	unsigned long diff;

	dev_dbg(pp->dev, "%s(): period_ticks %ld, duty_ticks %ld\n", __func__, c->period_ticks, c->duty_ticks);

	for(i = 0; i < 4; i++) {
		for(j = 0; j < 3; j++) {
			c_period_ticks = prescaler[j] * pwm_period[i];
			diff = abs(c->period_ticks - c_period_ticks);
			if(diff < approx_diff) {
				approx_i = i;
				approx_j = j;
				approx_diff = diff;
			}
			dev_dbg(pp->dev, "%s(): %5d:%3d => %12llu, diff %12lu\n", __func__, pwm_period[i], prescaler[j], c_period_ticks, diff);
		}
	}

	c_period_ticks = prescaler[approx_j] * pwm_period[approx_i];

	div = (c->duty_ticks * c_period_ticks);
	do_div(div, c->period_ticks);
	c_duty_ticks =  div;

	if(c_period_ticks != c->period_ticks) {
		long percent;
		diff = c->period_ticks - c_period_ticks;
		percent = (100 * abs(diff)) / c->period_ticks;
		dev_dbg(pp->dev, "%s(): warning: wanted period_ticks %lu, got %llu, diff %ld, %ld%% off\n", __func__, c->period_ticks, c_period_ticks, diff, percent);
	}
	if(c_duty_ticks != c->duty_ticks) {
		long percent;
		diff = c->duty_ticks - c_duty_ticks;
		percent = (100 * abs(diff)) / c->duty_ticks;
		dev_dbg(pp->dev, "%s(): warning: wanted duty_ticks %lu, got %llu, diff %ld, %ld%% off\n", __func__, c->duty_ticks, c_duty_ticks, diff, percent);
	}

	c->period_ticks = c_period_ticks;
	c->duty_ticks = c_duty_ticks;
	c->period = approx_i;
	c->prescaler = approx_j;

	dev_dbg(pp->dev, "%s(): period_ticks %ld, duty_ticks %ld\n", __func__, c->period_ticks, c->duty_ticks);
}

static void reconfigure(struct pwm_device *p)
{
	struct tmpa9xx_pwm_priv *pp = pwm_get_drvdata(p);
	struct pwm_calc c;
	uint32_t val;

	dev_dbg(pp->dev, "%s(): period_ticks %ld, duty_ticks %ld\n", __func__, p->period_ticks, p->duty_ticks);

	if(!p->period_ticks || !p->duty_ticks)
		return;

	c.period_ticks = p->period_ticks;
	c.duty_ticks = p->duty_ticks;

	calc(pp, &c);

	p->period_ticks = c.period_ticks;
	p->duty_ticks = c.duty_ticks;

	tmr_writel(pp, TIMER_COMPARE_1, c.duty_ticks);

	val = tmr_readl(pp, TIMER_MODE);
	val &= ~(0x3 << 4);
	val |= (c.period << 4);
	tmr_writel(pp, TIMER_MODE, val);

	val = tmr_readl(pp, TIMER_CONTROL);
	val &= ~(0x3 << 4);
	val |= (c.prescaler << 4);
	tmr_writel(pp, TIMER_CONTROL, val);

	dev_dbg(pp->dev, "%s(): compare %lu, period %d, prescaler %d\n", __func__, c.duty_ticks, c.period, c.prescaler);
}

static int config_nosleep(struct pwm_device *p, struct pwm_config *c)
{
	struct tmpa9xx_pwm_priv *pp = pwm_get_drvdata(p);
	int was_on = 0;

	dev_dbg(pp->dev, "%s(): config_mask 0x%02lx\n", __func__, c->config_mask);

	was_on = stop_sync(p);
	if (was_on < 0)
		return was_on;

	if (test_bit(PWM_CONFIG_PERIOD_TICKS, &c->config_mask)) {
		dev_dbg(pp->dev, "%s(): period_ticks %ld\n", __func__, c->period_ticks);
		p->period_ticks = c->period_ticks;
		reconfigure(p);
	}

	if (test_bit(PWM_CONFIG_DUTY_TICKS, &c->config_mask)) {
		dev_dbg(pp->dev, "%s(): duty_ticks %ld\n", __func__, c->duty_ticks);
		p->duty_ticks = c->duty_ticks;
		reconfigure(p);
	}

	if (test_bit(PWM_CONFIG_POLARITY, &c->config_mask)) {
		dev_dbg(pp->dev, "%s(): polarity %d\n", __func__, c->polarity);
		pp->polarity = !!c->polarity;
	}

	if (test_bit(PWM_CONFIG_START, &c->config_mask)
	    || (was_on && !test_bit(PWM_CONFIG_STOP, &c->config_mask)))
		start(p);

	return 0;
}

static int config(struct pwm_device *p, struct pwm_config *c)
{
	return config_nosleep(p, c);
}

static int request(struct pwm_device *p)
{
	struct tmpa9xx_pwm_priv *pp = pwm_get_drvdata(p);

	p->tick_hz = clk_get_rate(pp->clk);

	dev_dbg(pp->dev, "%s(): rate %lu\n", __func__, p->tick_hz);

	return 0;
}

static const struct pwm_device_ops device_ops = {
	.owner		= THIS_MODULE,
	.config		= config,
	.config_nosleep	= config_nosleep,
	.request	= request,
};

static int __devinit tmpa9xx_pwm_probe(struct platform_device *pdev)
{
	const struct platform_device_id *id = platform_get_device_id(pdev);
	int prescaler_clock_khz;
	struct tmpa9xx_pwm_priv *pp;
	int ret;

	pp = kzalloc(sizeof(*pp), GFP_KERNEL);
	if (IS_ERR_OR_NULL(pp)) {
		dev_err(&pdev->dev, "kzalloc() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	pp->dev = &pdev->dev;

	pp->r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(pp->r)) {
		dev_err(&pdev->dev, "platform_get_resource() failed\n");
		ret = -EINVAL;
		goto err1;
	}

	pp->r = request_mem_region(pp->r->start, resource_size(pp->r), pdev->name);
	if (IS_ERR_OR_NULL(pp->r)) {
		dev_err(&pdev->dev, "request_mem_region() failed\n");
		ret = -EBUSY;
		goto err2;
	}

	pp->regs = ioremap(pp->r->start, resource_size(pp->r));
	if (IS_ERR_OR_NULL(pp->regs)) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto err3;
	}

	pp->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(pp->clk)) {
		dev_err(&pdev->dev, "clk_get() failed\n");
		ret = PTR_ERR(pp->clk);
		if (!ret)
			ret = -EINVAL;
		goto err4;
	}

	prescaler_clock_khz = (int)pdev->dev.platform_data;
	if (!prescaler_clock_khz)
		prescaler_clock_khz = pdev->id ? prescaler_clock_khz_1 : prescaler_clock_khz_0;
	if (!prescaler_clock_khz)
		prescaler_clock_khz = 32 * 1000;

	dev_dbg(&pdev->dev, "prescaler_clock_khz %d\n", prescaler_clock_khz);

	ret = clk_set_rate(pp->clk, prescaler_clock_khz);
	if (ret) {
		dev_err(&pdev->dev, "clk_set_rate() failed\n");
		ret = -ENODEV;
		goto err5;
	}

	ret = clk_enable(pp->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk_enable() failed\n");
		ret = -ENODEV;
		goto err5;
	}

	platform_set_drvdata(pdev, pp);

	pp->pwm = pwm_register(&device_ops, &pdev->dev, "%s:%d", DRIVER_NAME, pdev->id);
	if (IS_ERR_OR_NULL(pp->pwm)) {
		dev_err(&pdev->dev, "pwm_register() failed\n");
		ret = PTR_ERR(pp->pwm);
		if (!ret)
			ret = -EINVAL;
		goto err5;
	}

	pwm_set_drvdata(pp->pwm, pp);

	dev_info(&pdev->dev, "channel %d, clk speed %lu kHz\n", pdev->id, clk_get_rate(pp->clk));

	return 0;

err5:
	platform_set_drvdata(pdev, NULL);
	clk_put(pp->clk);
err4:
	iounmap(pp->regs);
err3:
	release_mem_region(pp->r->start, resource_size(pp->r));
err2:
err1:
	kfree(pp);
err0:
	return ret;
}

static int __devexit tmpa9xx_pwm_remove(struct platform_device *pdev)
{
	struct tmpa9xx_pwm_priv *pp = platform_get_drvdata(pdev);

	if (pwm_is_requested(pp->pwm)) {
		if (pwm_is_running(pp->pwm))
			pwm_stop(pp->pwm);
		pwm_release(pp->pwm);
	}
	pwm_unregister(pp->pwm);
	platform_set_drvdata(pdev, NULL);
	clk_disable(pp->clk);
	clk_put(pp->clk);
	iounmap(pp->regs);
	release_mem_region(pp->r->start, resource_size(pp->r));
	kfree(pp);

	return 0;
}

static const struct platform_device_id tmpa9xx_pwm_id_table[] = {
	/* PWM has_secondary_pwm? */
	{"tmpa9xx-pwm", 0},
	{},
};

MODULE_DEVICE_TABLE(platform, tmpa9xx_pwm_id_table);

static struct platform_driver pwm_driver = {
	.driver = {
		.name = "tmpa9xx-pwm",
		.owner = THIS_MODULE,
	},
	.probe = tmpa9xx_pwm_probe,
	.remove = __devexit_p(tmpa9xx_pwm_remove),
	.id_table = tmpa9xx_pwm_id_table,
};

static int __init tmpa9xx_pwm_init(void)
{
	return platform_driver_register(&pwm_driver);
}

static void __exit tmpa9xx_pwm_exit(void)
{
	platform_driver_unregister(&pwm_driver);
}

module_init(tmpa9xx_pwm_init);
module_exit(tmpa9xx_pwm_exit);

MODULE_AUTHOR("Nils Faerber <nils.faerber@kernelconcepts.de>");
MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("PWM driver for TMPA9xx");
MODULE_LICENSE("GPL");
