/*
 * drivers/pwm/tmpa9xx-pwm.c
 *
 * (c) 2010 Nils Faerber <nils.faerber@kernelconcepts.de>
 * based on pxa2xx pwm.c by eric miao <eric.miao@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/*
 * The TMPA9xx PWM is implemented in the timer controller.
 * The TMPA9xx has six timers in three blocks. Each block uses
 * one common input clock, so there are only three independant
 * timers available.
 * Only Block-1 and Block-2 habe PWM option in their first parts, Block-3
 * only has pure timers.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm/pwm.h>

#include <asm/div64.h>

#include <mach/regs.h>
#include <mach/timer.h>

#define HAS_SECONDARY_PWM	0x10
#define PWM_ID_BASE(d)		((d) & 0xf)

static const struct platform_device_id tmpa9xx_pwm_id_table[] = {
	/*   PWM    has_secondary_pwm? */
	{ "tmpa9xx-pwm", 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, tmpa9xx_pwm_id_table);

/* PWM registers and bits definitions */
struct tmpa9xx_pwm {
	struct pwm_device       pwm;
	spinlock_t              lock;

	struct clk      	*clk;
	int             	clk_enabled;
	unsigned int		prescale;
	unsigned long		requested_period_ticks;
	unsigned long		requested_duty_ticks;
	void  		*mmio_base;
};

static inline struct tmpa9xx_pwm *to_tmpa9xx_pwm(const struct pwm_channel *p)
{
       return container_of(p->pwm, struct tmpa9xx_pwm, pwm);
}

static inline int __tmpa9xx_pwm_enable(struct pwm_channel *p)
{
	struct tmpa9xx_pwm *tmpa9xx = to_tmpa9xx_pwm(p);
	volatile struct tmpa9xx_hw_timer *hw_timer =(volatile struct tmpa9xx_hw_timer *)tmpa9xx->mmio_base;

	hw_timer->TimerMode    |= (0x01<<6);		/* select PWM mode */
	hw_timer->TimerControl |= (  (0x01<<1) 		/* 16 bit counter */
                                   |(0x01<<6)		/* Timer0 periodic timer */
                                   |(0x01<<7) );	/* Timer0 enable */

	hw_timer->TimerCmpEn   |= (0x01<<0);		/* enable compare */
	if (!tmpa9xx->clk_enabled) {
		clk_enable(tmpa9xx->clk);
		tmpa9xx->clk_enabled = 1;
	}

	return 0;
}

static inline int __tmpa9xx_pwm_disable(struct pwm_channel *p)
{
	struct tmpa9xx_pwm *tmpa9xx = to_tmpa9xx_pwm(p);
	volatile struct tmpa9xx_hw_timer *hw_timer =(volatile struct tmpa9xx_hw_timer *)tmpa9xx->mmio_base;

	hw_timer->TimerControl &= ~( (0x01<<1)		/* deselect 16 bit counter */
                                    |(0x01<<6)		/* Disable Timer0 periodic timer */
                                    |(0x01<<7) );	/* Timer0 disable */

	hw_timer->TimerMode    &= ~(0x01<<6);		/* deselect PWM mode */
	hw_timer->TimerCmpEn   &= ~(0x01<<0);		/* disable compare */

	if (tmpa9xx->clk_enabled) {
		clk_disable(tmpa9xx->clk);
		tmpa9xx->clk_enabled = 0;
	}

       return 0;
}

static int __tmpa9xx_config_duty_ticks(struct pwm_channel *p,struct pwm_channel_config *c)
{
	struct tmpa9xx_pwm *tmpa9xx = to_tmpa9xx_pwm(p);
	volatile struct tmpa9xx_hw_timer *hw_timer =(volatile struct tmpa9xx_hw_timer *)tmpa9xx->mmio_base;

        tmpa9xx->requested_duty_ticks=c->duty_ticks;
        
	p->duty_ticks = (p->period_ticks * tmpa9xx->requested_duty_ticks) / tmpa9xx->requested_period_ticks;

	if (tmpa9xx->clk_enabled==1)
		hw_timer->TimerMode    &= ~(0x01<<6);		/* deselect PWM mode */
                
	hw_timer->TimerCompare1 = p->duty_ticks ;
        
	if (tmpa9xx->clk_enabled==1)
		hw_timer->TimerMode    |= (0x01<<6);		/* select PWM mode */
	
	return 0;
}

static int __tmpa9xx_config_period_ticks(struct pwm_channel *p,struct pwm_channel_config *c)
{
	struct tmpa9xx_pwm *tmpa9xx = to_tmpa9xx_pwm(p);
	volatile struct tmpa9xx_hw_timer *hw_timer =(volatile struct tmpa9xx_hw_timer *)tmpa9xx->mmio_base;
        unsigned int periodlength[]={0xff,0x1ff,0x2ff,0xffff};
        unsigned char periodtype;
        unsigned char prescalemode = 0;
        int locked=0;

        tmpa9xx->prescale=1;
        
	tmpa9xx->requested_period_ticks=c->period_ticks;

	p->period_ticks = c->period_ticks;

	while ((prescalemode<=2) && (locked==0))
	{
        	int i;
                
                p->period_ticks /= tmpa9xx->prescale;
        	for (i=0;i<4;i++)
                	if (p->period_ticks < periodlength[i])
                        {
                        	locked=1;
                                p->period_ticks=periodlength[i];
                                periodtype=i;
                                break;
                        }
                tmpa9xx->prescale *= 16;
                prescalemode++;
        }
        
        tmpa9xx->prescale /= 16;
        prescalemode--;
        
        if (prescalemode>2)
        	return -EINVAL;
        

	__tmpa9xx_pwm_disable(p);

	hw_timer->TimerMode    &= ~(0x3<<4);
	hw_timer->TimerMode    |=  (periodtype<<4);
	hw_timer->TimerControl &= ~(0x3<<4);
 	hw_timer->TimerControl |=  (prescalemode<<4);
       
        return 0;
}

static int tmpa9xx_pwm_request(struct pwm_channel *p)
{
       struct tmpa9xx_pwm *tmpa9xx = to_tmpa9xx_pwm(p);

       p->tick_hz = clk_get_rate(tmpa9xx->clk);
       return 0;
}


static int tmpa9xx_pwm_config_nosleep(struct pwm_channel *p,
                                 struct pwm_channel_config *c)
{
       int ret = 0;
       unsigned long flags;

       if (!(c->config_mask & (PWM_CONFIG_STOP
                             | PWM_CONFIG_START
                             | PWM_CONFIG_DUTY_TICKS
                             | PWM_CONFIG_PERIOD_TICKS)))
               return -EINVAL;

       spin_lock_irqsave(&p->lock, flags);

       if (c->config_mask & PWM_CONFIG_STOP)
               __tmpa9xx_pwm_disable(p);

       if (c->config_mask & PWM_CONFIG_PERIOD_TICKS)
               __tmpa9xx_config_period_ticks(p, c);

       if (c->config_mask & PWM_CONFIG_DUTY_TICKS)
               __tmpa9xx_config_duty_ticks(p, c);

       if (c->config_mask & PWM_CONFIG_START)
               __tmpa9xx_pwm_enable(p);

       spin_unlock_irqrestore(&p->lock, flags);
       return ret;
}

static int tmpa9xx_pwm_config(struct pwm_channel *p,
                         struct pwm_channel_config *c)
{
       return tmpa9xx_pwm_config_nosleep(p, c);
}

static int __init tmpa9xx_pwm_probe(struct platform_device *pdev)
{
       const struct platform_device_id *id = platform_get_device_id(pdev);
       struct tmpa9xx_pwm *tmpa9xx;
       struct resource *r;
       int ret = 0;

       r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
       if (IS_ERR_OR_NULL(r)) {
               dev_err(&pdev->dev, "error, missing mmio_base resource\n");
               return -EINVAL;
       }

       r = request_mem_region(r->start, resource_size(r), pdev->name);
       if (IS_ERR_OR_NULL(r)) {
               dev_err(&pdev->dev, "error, failed to request mmio_base resource\n");
               return -EBUSY;
       }

       tmpa9xx = kzalloc(sizeof *tmpa9xx, GFP_KERNEL);
       if (IS_ERR_OR_NULL(tmpa9xx)) {
               dev_err(&pdev->dev, "failed to allocate memory\n");
               ret = -ENOMEM;
               goto err_kzalloc;
       }

       tmpa9xx->mmio_base = ioremap(r->start, resource_size(r));
       if (IS_ERR_OR_NULL(tmpa9xx->mmio_base)) {
               dev_err(&pdev->dev, "error, failed to ioremap() registers\n");
               ret = -ENODEV;
               goto err_ioremap;
       }
       else

       tmpa9xx->clk = clk_get(&pdev->dev, NULL);
       if (IS_ERR_OR_NULL(tmpa9xx->clk)) {
               ret = PTR_ERR(tmpa9xx->clk);
               if (!ret)
                       ret = -EINVAL;
               goto err_clk_get;
       }
       tmpa9xx->clk_enabled = 0;

       spin_lock_init(&tmpa9xx->lock);

       tmpa9xx->pwm.dev = &pdev->dev;
       tmpa9xx->pwm.bus_id = dev_name(&pdev->dev);
       tmpa9xx->pwm.owner = THIS_MODULE;
       tmpa9xx->pwm.request = tmpa9xx_pwm_request;
       tmpa9xx->pwm.config_nosleep = tmpa9xx_pwm_config_nosleep;
       tmpa9xx->pwm.config = tmpa9xx_pwm_config;

       if (id->driver_data & HAS_SECONDARY_PWM)
               tmpa9xx->pwm.nchan = 2;
       else
               tmpa9xx->pwm.nchan = 1;

       ret =  pwm_register(&tmpa9xx->pwm);

       if (ret)
               goto err_pwm_register;

       platform_set_drvdata(pdev, tmpa9xx);
       return 0;

err_pwm_register:
       clk_put(tmpa9xx->clk);
err_clk_get:
       iounmap(tmpa9xx->mmio_base);
err_ioremap:
       kfree(tmpa9xx);
err_kzalloc:
       release_mem_region(r->start, resource_size(r));
       return ret;
}

#if 0
static int __exit tmpa9xx_pwm_remove(struct platform_device *pdev)
{
       struct tmpa9xx_pwm *tmpa9xx = platform_get_drvdata(pdev);

       if (IS_ERR_OR_NULL(tmpa9xx))
               return -ENODEV;

       iounmap(tmpa9xx->mmio_base);
       pwm_unregister(&tmpa9xx->pwm);
       clk_put(tmpa9xx->clk);
       kfree(tmpa9xx);
       platform_set_drvdata(pdev, NULL);

       return 0;
}
#endif

static struct platform_driver pwm_driver = {
       .driver         = {
               .name   = "tmpa9xx-pwm",
               .owner  = THIS_MODULE,
       },
       .probe          = tmpa9xx_pwm_probe,
       .remove         = NULL,
       .id_table       = tmpa9xx_pwm_id_table,
};

static int __init tmpa9xx_pwm_init(void)
{
      return platform_driver_register(&pwm_driver);
}
/* TODO: do we have to do this at arch_initcall? */
module_init(tmpa9xx_pwm_init);

static void __exit tmpa9xx_pwm_exit(void)
{
       platform_driver_unregister(&pwm_driver);
}
module_exit(tmpa9xx_pwm_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Nils Faerber <nils.faerber@kernelconcepts.de>");

