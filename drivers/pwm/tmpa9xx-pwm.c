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
 * Foundation, Inp->, 59 Temple Place, Suite 330, Boston, MA 02111-1307
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

#define tmr_writel(b, o, v)	writel(v, b->regs + o)
#define tmr_readl(b, o)		readl(b->regs + o)

struct tmpa9xx_pwm_priv {
	void __iomem *regs;
	struct device *dev;
	struct resource *r;
	struct clk *clk;
	struct pwm_device *pwm;
	struct pwm_device_ops ops;
	int once;
	int period;
	int prescaler;
};

static void start(struct pwm_device *pp)
{
	struct tmpa9xx_pwm_priv *p = pwm_get_drvdata(pp);
	int active = test_bit(PWM_FLAG_RUNNING, &pp->flags);
	uint32_t val;

	BUG_ON(active);
	set_bit(PWM_FLAG_RUNNING, &pp->flags);

	val = tmr_readl(p, TIMER_MODE);
	val &= ~(0x3 << 4);
	val |= (p->period << 4);
	tmr_writel(p, TIMER_MODE, val);

	val = tmr_readl(p, TIMER_CONTROL);
	val &= ~(0x3 << 4);
	val |= (p->prescaler << 4);
	tmr_writel(p, TIMER_CONTROL, val);

	if (pp->polarity) {
		val = pp->period_ticks - pp->duty_ticks;
		tmr_writel(p, TIMER_COMPARE_1, val);
		dev_dbg(p->dev, "%s(): honouring inverted polarity, duty_ticks %d\n", __func__, val);
	} else {
		tmr_writel(p, TIMER_COMPARE_1, pp->duty_ticks);
		dev_dbg(p->dev, "%s(): duty_ticks %ld\n", __func__, pp->duty_ticks);
	}

	if (p->once)
		return;
	
	p->once = 1;
	
	val = tmr_readl(p, TIMER_MODE);
	val |= (0x01 << 6);	/* select PWM mode */
	tmr_writel(p, TIMER_MODE, val);

	val = tmr_readl(p, TIMER_CMPEN);
	val |= (0x01 << 0); /* enable compare */
	tmr_writel(p, TIMER_CMPEN, val);

	val = tmr_readl(p, TIMER_CONTROL);
	val |=   ((0x01 << 1)	/* 16 bit counter */
		| (0x01 << 6)	/* Timer0 periodic timer */
		| (0x01 << 7));	/* Timer0 enable */
	tmr_writel(p, TIMER_CONTROL, val);
}

static int stop_sync(struct pwm_device *pp)
{
	struct tmpa9xx_pwm_priv *p = pwm_get_drvdata(pp);
	int active = test_bit(PWM_FLAG_RUNNING, &pp->flags);
	uint32_t val;
	
	if (!active) {
		dev_dbg(p->dev, "%s(): already stopped\n", __func__);
		return 0;
	}
	dev_dbg(p->dev, "%s():\n", __func__);

	val = tmr_readl(p, TIMER_CONTROL);
	val &= ~(0x01 << 7);	/* Timer0 disable */
	tmr_writel(p, TIMER_CONTROL, val);

	val = tmr_readl(p, TIMER_MODE);
	val &= ~(0x01 << 6);	/* deselect PWM mode */
	tmr_writel(p, TIMER_MODE, val);
		
	if (pp->polarity) {
		tmr_writel(p, TIMER_COMPARE_1, pp->period_ticks);
		dev_dbg(p->dev, "%s(): honouring inverted polarity\n", __func__);
	} else {
		tmr_writel(p, TIMER_COMPARE_1, 0);
	}

	val = tmr_readl(p, TIMER_MODE);
	val |= (0x01 << 6);	/* select PWM mode */
	tmr_writel(p, TIMER_MODE, val);

	val = tmr_readl(p, TIMER_CONTROL);
	val |= (0x01 << 7);	/* Timer0 enable */
	tmr_writel(p, TIMER_CONTROL, val);
	
	clear_bit(PWM_FLAG_RUNNING, &pp->flags);

	return 1;
}

static void reconfigure(struct pwm_device *pp)
{
	struct tmpa9xx_pwm_priv *p = pwm_get_drvdata(pp);
	int pwm_period[] = {255, 511, 1023, 65535};
	int prescaler[] = {1, 16, 256};
	int i;
	int j;

	dev_dbg(p->dev, "%s(): period_ticks %ld, duty_ticks %ld\n", __func__, pp->period_ticks, pp->duty_ticks);

	if(!pp->period_ticks)
		return;

	for(i = 0; i < 3; i++) {
		for(j = 0; j < 4; j++) {
			if (pp->period_ticks <= pwm_period[j] * prescaler[i])
				goto out;
		}
	}
out:
	if (i == 3 && j == 4) {
		i = 2;
		j = 3;
	}

	p->period = j;
	p->prescaler = i;
	pp->period_ticks = pwm_period[j] * prescaler[i];

	dev_dbg(p->dev, "%s(): period %d, prescaler %d -> %ld\n", __func__, p->period, p->prescaler, pp->period_ticks);
}

static int config_nosleep(struct pwm_device *pp, struct pwm_config *c)
{
	struct tmpa9xx_pwm_priv *p = pwm_get_drvdata(pp);
	int was_on = 0;

	dev_dbg(p->dev, "%s(): config_mask 0x%02lx\n", __func__, c->config_mask);

	was_on = stop_sync(pp);
	if (was_on < 0)
		return was_on;

	if (test_bit(PWM_CONFIG_PERIOD_TICKS, &c->config_mask)) {
		dev_dbg(p->dev, "%s(): period_ticks %ld\n", __func__, c->period_ticks);
		pp->period_ticks = c->period_ticks;
		reconfigure(pp);
	}

	if (test_bit(PWM_CONFIG_DUTY_TICKS, &c->config_mask)) {
		dev_dbg(p->dev, "%s(): duty_ticks %ld\n", __func__, c->duty_ticks);
		pp->duty_ticks = c->duty_ticks;
		reconfigure(pp);
	}

	if (test_bit(PWM_CONFIG_POLARITY, &c->config_mask)) {
		dev_dbg(p->dev, "%s(): polarity %d\n", __func__, c->polarity);
		pp->polarity = !!c->polarity;
	}

	if (test_bit(PWM_CONFIG_START, &c->config_mask)
	    || (was_on && !test_bit(PWM_CONFIG_STOP, &c->config_mask)))
		start(pp);

	return 0;
}

static int config(struct pwm_device *pp, struct pwm_config *c)
{
	return config_nosleep(pp, c);
}

static int request(struct pwm_device *pp)
{
	struct tmpa9xx_pwm_priv *p = pwm_get_drvdata(pp);

	pp->tick_hz = clk_get_rate(p->clk);

	dev_dbg(p->dev, "%s(): rate %lu\n", __func__, pp->tick_hz);

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
	int prescaler_clock_khz;
	struct tmpa9xx_pwm_priv *p;
	int ret;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (IS_ERR_OR_NULL(p)) {
		dev_err(&pdev->dev, "kzalloc() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	p->dev = &pdev->dev;

	p->r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(p->r)) {
		dev_err(&pdev->dev, "platform_get_resource() failed\n");
		ret = -EINVAL;
		goto err1;
	}

	p->r = request_mem_region(p->r->start, resource_size(p->r), pdev->name);
	if (IS_ERR_OR_NULL(p->r)) {
		dev_err(&pdev->dev, "request_mem_region() failed\n");
		ret = -EBUSY;
		goto err2;
	}

	p->regs = ioremap(p->r->start, resource_size(p->r));
	if (IS_ERR_OR_NULL(p->regs)) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto err3;
	}

	p->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(p->clk)) {
		dev_err(&pdev->dev, "clk_get() failed\n");
		ret = PTR_ERR(p->clk);
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

	ret = clk_set_rate(p->clk, prescaler_clock_khz);
	if (ret) {
		dev_err(&pdev->dev, "clk_set_rate() failed\n");
		ret = -ENODEV;
		goto err5;
	}

	ret = clk_enable(p->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk_enable() failed\n");
		ret = -ENODEV;
		goto err5;
	}

	platform_set_drvdata(pdev, p);

	p->pwm = pwm_register(&device_ops, &pdev->dev, "tmpa9xx-pwm:%d", pdev->id);
	if (IS_ERR_OR_NULL(p->pwm)) {
		dev_err(&pdev->dev, "pwm_register() failed\n");
		ret = PTR_ERR(p->pwm);
		if (!ret)
			ret = -EINVAL;
		goto err5;
	}

	pwm_set_drvdata(p->pwm, p);

	dev_info(&pdev->dev, "channel %d, clk speed %lu kHz\n", pdev->id, clk_get_rate(p->clk));

	return 0;

err5:
	platform_set_drvdata(pdev, NULL);
	clk_put(p->clk);
err4:
	iounmap(p->regs);
err3:
	release_mem_region(p->r->start, resource_size(p->r));
err2:
err1:
	kfree(p);
err0:
	return ret;
}

static int __devexit tmpa9xx_pwm_remove(struct platform_device *pdev)
{
	struct tmpa9xx_pwm_priv *p = platform_get_drvdata(pdev);

	if (pwm_is_requested(p->pwm)) {
		if (pwm_is_running(p->pwm))
			pwm_stop(p->pwm);
		pwm_release(p->pwm);
	}
	pwm_unregister(p->pwm);
	platform_set_drvdata(pdev, NULL);
	clk_disable(p->clk);
	clk_put(p->clk);
	iounmap(p->regs);
	release_mem_region(p->r->start, resource_size(p->r));
	kfree(p);

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
