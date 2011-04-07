/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2005 David Brownell
 * (C) Copyright 2002 Hewlett-Packard Company
 * (C) Copyright 2008 Magnus Damm
 * (C) Copyright 2009 bPlan GmbH
 * 
 * Toshiba TMPA9xx Bus Glue - based on ohci-tmpa9xx.c
 *
 * This file is licenced under the GPL.
 */

#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include <mach/sram.h>

static struct clk *clk;

static int ohci_tmpa9xx_init(struct usb_hcd *hcd)
{
	struct device *dev = hcd->self.controller;
	int ret;

	ret = ohci_init(hcd_to_ohci(hcd));
	if (ret) {
		dev_err(dev, "can't init %s", hcd->self.bus_name);
	}

	return ret;
}

static int ohci_tmpa9xx_start(struct usb_hcd *hcd)
{
	struct device *dev = hcd->self.controller;
	int ret;

	ret = ohci_run(hcd_to_ohci(hcd));
	if (ret < 0) {
		dev_err(dev, "can't start %s", hcd->self.bus_name);
		ohci_stop(hcd);
	}

	return ret;
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_tmpa9xx_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"tmpa9xx OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY | HCD_LOCAL_MEM,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ohci_tmpa9xx_init,
	.start =		ohci_tmpa9xx_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,

#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};
/*-------------------------------------------------------------------------*/

static int ohci_hcd_tmpa9xx_drv_probe(struct platform_device *pdev)
{
	const struct hc_driver *driver = &ohci_tmpa9xx_hc_driver;
	struct device *dev = &pdev->dev;
	struct resource	*res, *mem;
	int retval, irq;
	struct usb_hcd *hcd = NULL;

	irq = retval = platform_get_irq(pdev, 0);
	if (retval < 0) {
		dev_err(dev, "platform_get_irq() failed\n");
		retval = -ENOENT;
		goto err0;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "platform_get_resource() failed\n");
		retval = -ENOENT;
		goto err1;
	}

	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(dev, "clk_get() failed\n");
		retval = -ENOENT;
		goto err2;
	}

	retval = clk_enable(clk);
	if (retval) {
		dev_err(dev, "clk_enable() failed\n");
		retval = -ENOENT;
		goto err3;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, "tmpa9xx");
	if (!hcd) {
		dev_err(dev, "usb_create_hcd() failed\n");
		retval = -ENOMEM;
		goto err4;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,	pdev->name)) {
		dev_err(dev, "request_mem_region() failed\n");
		retval = -EBUSY;
		goto err5;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (hcd->regs == NULL) {
		dev_err(dev, "ioremap() failed\n");
		retval = -ENXIO;
		goto err6;
	}

	ohci_hcd_init(hcd_to_ohci(hcd));

	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED);
	if (retval) {
		dev_err(dev, "usb_add_hcd() failed\n");
		goto err7;
	}

	return 0;

err7:
	iounmap(hcd->regs);
err6:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err5:
	usb_put_hcd(hcd);
err4:
	clk_disable(clk);
err3:
	clk_put(clk);
err2:
err1:
err0:
	return retval;
}

static int ohci_hcd_tmpa9xx_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);

	iounmap(hcd->regs);

	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);

	usb_put_hcd(hcd);

	clk_disable(clk);

	clk_put(clk);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_PM
static int ohci_tmpa9xx_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(platform_get_drvdata(pdev));

	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;


	ohci_to_hcd(ohci)->state = HC_STATE_SUSPENDED;
	return 0;
}

static int ohci_tmpa9xx_resume(struct platform_device *pdev)
{
	struct usb_hcd	*hcd = platform_get_drvdata(pdev);
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);

	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;


	ohci_finish_controller_resume(hcd);
	return 0;
}
#else
#define ohci_tmpa9xx_suspend NULL
#define ohci_tmpa9xx_resume NULL
#endif

/*-------------------------------------------------------------------------*/

/*
 * Driver definition to register with the tmpa9xx bus
 */
static struct platform_driver ohci_hcd_tmpa9xx_driver = {
	.probe		= ohci_hcd_tmpa9xx_drv_probe,
	.remove		= ohci_hcd_tmpa9xx_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.suspend	= ohci_tmpa9xx_suspend,
	.resume		= ohci_tmpa9xx_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tmpa9xx-usb",
	},
};

MODULE_ALIAS("platform:tmpa9xx-usb");
