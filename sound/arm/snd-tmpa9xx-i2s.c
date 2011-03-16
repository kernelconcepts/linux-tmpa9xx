#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include <mach/dma.h>
#include <mach/regs.h>

#include "tmpa9xx_i2s.h"

struct scatter_dma_t {
	unsigned long srcaddr;
	unsigned long dstaddr;
	unsigned long lli;
	unsigned long control;
};

struct i2s_stream
{
	int dma_ch;

	/* DMA descriptor ring head of current audio stream*/
	struct scatter_dma_t *dma_desc;
	unsigned int desc_bytes;
	unsigned int dma_phydesc;
	unsigned int dma_buf;
	unsigned int run; /* is running */
	struct scatter_dma_t *curr_desc;
};

struct tmpa9xx_i2s_priv
{
	struct device *dev;

	struct i2s_stream rx;
	void (*rx_callback)(void *data);
	void *tx_data;

	struct i2s_stream tx;
	void (*tx_callback)(void *data);
	void *rx_data;
};

struct tmpa9xx_i2s_priv *g_tmpa9xx_i2s_priv;

static int i2s_tx_start(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;

	tmpa9xx_dma_enable(i2s->tx.dma_ch);

	/* I2S DMA set complete */
	I2STDMA1 = 0x0001;
	/* I2S transfer start */
	I2STSLVON = 0x0001;

	return 0;
}

static int i2s_rx_start(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;

	tmpa9xx_dma_enable(i2s->rx.dma_ch);

	/* I2S DMA set complete */
	I2SRDMA1 = 0x0001;
	/* I2S transfer start */
	I2SRSLVON = 0x0001;

	return 0;
}

static int i2s_tx_stop(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;

	I2STDMA1 = 0x0000;
	I2STSLVON = 0x0000;

	tmpa9xx_dma_disable(i2s->tx.dma_ch);

	return 0;
}

static int i2s_rx_stop(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;

	I2SRDMA1 = 0x0000;
	I2SRSLVON = 0x0000;

	tmpa9xx_dma_disable(i2s->rx.dma_ch);

	return 0;
}

static int i2s_tx_dma_start(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;

	int dma_ch = i2s->tx.dma_ch;
	struct scatter_dma_t *dma_desc;

	i2s->tx.curr_desc = i2s->tx.dma_desc;
	dma_desc = i2s->tx.curr_desc;

        DMA_SRC_ADDR(dma_ch) = dma_desc->srcaddr;
	DMA_DEST_ADDR(dma_ch) = dma_desc->dstaddr;
	DMA_LLI(dma_ch) = dma_desc->lli;
	DMA_CONTROL(dma_ch) = dma_desc->control;
	DMA_CONFIG(dma_ch) = 0x00008a81;

	return 0;
}

static int i2s_rx_dma_start(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;

	int dma_ch = i2s->rx.dma_ch;
	struct scatter_dma_t *dma_desc;

	i2s->rx.curr_desc = i2s->rx.dma_desc;
	dma_desc = i2s->rx.curr_desc;

	DMA_SRC_ADDR(dma_ch) = dma_desc->srcaddr;
	DMA_DEST_ADDR(dma_ch) = dma_desc->dstaddr;
	DMA_LLI(dma_ch) = dma_desc->lli;
	DMA_CONTROL(dma_ch) = dma_desc->control;
	DMA_CONFIG(dma_ch) = 0x00009017;

	return 0;
}

int tmpa9xx_i2s_tx_start(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;

	if (i2s->tx.run)
		return -EBUSY;

	i2s_tx_dma_start();
	i2s_tx_start();
	i2s->tx.run = 1;

	return 0;
}

int tmpa9xx_i2s_rx_start(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;

	if (i2s->rx.run)
		return -EBUSY;

	i2s_rx_dma_start();
	i2s_rx_start();
	i2s->rx.run = 1;

	return 0;
}

int tmpa9xx_i2s_tx_stop(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;

	if (!i2s->tx.run)
		return 0;

	/* Both rx and tx dma stopped */
	i2s_tx_stop();
	i2s->tx.curr_desc = NULL;

	i2s->tx.run = 0;

	return 0;
}

int tmpa9xx_i2s_rx_stop(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;

	if (!i2s->rx.run)
		return 0;

	i2s_rx_stop();
	i2s->rx.curr_desc = NULL;

	i2s->rx.run = 0;

	return 0;
}

static void i2s_tx_shutdown(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;
	if (!i2s->tx.dma_desc)
		return;

	dma_free_coherent(NULL, i2s->tx.desc_bytes, i2s->tx.dma_desc, i2s->tx.dma_phydesc);
	i2s->tx.desc_bytes = 0;
	i2s->tx.dma_desc = 0;
	i2s->tx.dma_phydesc = 0;
}

int tmpa9xx_i2s_tx_setup(struct tmpa9xx_i2s_config *c)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;
	struct scatter_dma_t *desc;
	unsigned int count;
	dma_addr_t addr;
	int i;

	i2s_tx_shutdown();

	i2s->tx_callback = c->callback;
	i2s->tx_data = c->data;

	count = c->fragsize / c->size;

	/* for fragments larger than 16k words we use 2d dma,
	 * denote fragecount as two numbers' mutliply and both of them
	 * are less than 64k.*/
	if (count >= 0x1000) {
		return -EINVAL;
	}

	i2s->tx.dma_desc = dma_alloc_coherent(NULL, c->fragcount * sizeof(struct scatter_dma_t), &addr, GFP_USER);
	i2s->tx.desc_bytes = c->fragcount * sizeof(struct scatter_dma_t);
	i2s->tx.dma_phydesc = addr;
	i2s->tx.dma_buf = c->phy_buf;

	if (!i2s->tx.dma_desc) {
		return -ENOMEM;
	}

	desc = i2s->tx.dma_desc;
	for (i=0; i<c->fragcount; i++) {
		desc[i].lli = (unsigned long)(addr + (i + 1) * sizeof(struct scatter_dma_t));
		desc[i].srcaddr = (unsigned long)(c->phy_buf + i*c->fragsize);
		desc[i].dstaddr = (unsigned long)I2STDAT_ADR;
		desc[i].control = 0x84492000 + (unsigned long)(c->fragsize >> 2);
	}

	/* make circular */
	desc[c->fragcount-1].lli = (unsigned long)addr;

	return 0;
}

static void i2s_rx_shutdown(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;
	if (!i2s->rx.dma_desc)
		return;

	dma_free_coherent(NULL, i2s->rx.desc_bytes, i2s->rx.dma_desc, i2s->rx.dma_phydesc);
	i2s->rx.desc_bytes = 0;
	i2s->rx.dma_desc = 0;
	i2s->rx.dma_phydesc = 0;
}

int tmpa9xx_i2s_rx_setup(struct tmpa9xx_i2s_config *c)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;
	struct scatter_dma_t *desc;
	unsigned int count;
	dma_addr_t addr;
	int i;

	i2s_rx_shutdown();

	i2s->rx_callback = c->callback;
	i2s->rx_data = c->data;

	count = c->fragsize / c->size;

	/* for fragments larger than 16k words we use 2d dma,
	 * denote fragecount as two numbers' mutliply and both of them
	 * are less than 64k.*/
	if (count >= 0x1000) {
		printk(KERN_INFO "Error: rx dma size too large %d\n", count);
		return -EINVAL;
	}

	i2s->rx.dma_desc = dma_alloc_coherent(NULL, c->fragcount * sizeof(struct scatter_dma_t), &addr, GFP_USER);
	i2s->rx.desc_bytes = c->fragcount * sizeof(struct scatter_dma_t);
	i2s->rx.dma_phydesc = addr;
	i2s->rx.dma_buf = c->phy_buf;

	if (!i2s->rx.dma_desc) {
		return -ENOMEM;
	}

	desc = i2s->tx.dma_desc;
	for (i=0; i<c->fragcount; ++i)
	{
		desc[i].lli  = (unsigned long)(addr + (i + 1) * sizeof(struct scatter_dma_t));
		desc[i].srcaddr = (unsigned long)I2SRDAT_ADR;
		desc[i].dstaddr = (unsigned long)(c->phy_buf + i*c->fragsize);
		desc[i].control = 0x88492000 + (unsigned long)(c->fragsize >> 2);
	}

	/* make circular */
	desc[c->fragcount-1].lli = (unsigned long)addr;

	return 0;
}

int tmpa9xx_i2s_curr_offset_tx(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;
	return DMA_SRC_ADDR(i2s->tx.dma_ch) - i2s->tx.dma_buf;
}

int tmpa9xx_i2s_curr_offset_rx(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;
	return DMA_SRC_ADDR(i2s->rx.dma_ch) - i2s->rx.dma_buf;
}

static void tx_handler(int dma_ch, void *dev_id)
{
	struct tmpa9xx_i2s_priv *i2s = dev_id;

	if (i2s->tx_callback)
		i2s->tx_callback(i2s->tx_data);
}

static void rx_handler(int dma_ch, void *dev_id)
{
	struct tmpa9xx_i2s_priv *i2s = dev_id;

	if (i2s->rx_callback)
		i2s->rx_callback(i2s->rx_data);
}

static void err_handler(int dma_ch, void *dev_id)
{
	pr_info("%s():\n", __func__);
}

EXPORT_SYMBOL(tmpa9xx_i2s_tx_setup);
EXPORT_SYMBOL(tmpa9xx_i2s_tx_start);
EXPORT_SYMBOL(tmpa9xx_i2s_tx_stop);
EXPORT_SYMBOL(tmpa9xx_i2s_curr_offset_tx);

EXPORT_SYMBOL(tmpa9xx_i2s_rx_setup);
EXPORT_SYMBOL(tmpa9xx_i2s_rx_start);
EXPORT_SYMBOL(tmpa9xx_i2s_rx_stop);
EXPORT_SYMBOL(tmpa9xx_i2s_curr_offset_rx);

static int __devinit probe(struct platform_device *pdev)
{
	struct tmpa9xx_i2s_priv *p;
	int ret;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		dev_err(&pdev->dev, "kzalloc() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

 	p->tx.dma_ch = tmpa9xx_dma_request("I2S TX", 1, tx_handler, err_handler, p);
	if (p->tx.dma_ch < 0) {
		dev_err(&pdev->dev, "tmpa9xx_dma_request() @ tx failed\n");
		ret = -ENODEV;
		goto err1;
	}

	p->rx.dma_ch = tmpa9xx_dma_request("I2S RX", 2, rx_handler, err_handler, p);
	if (p->rx.dma_ch < 0) {
		dev_err(&pdev->dev, "tmpa9xx_dma_request() @ rx failed\n");
		ret = -ENODEV;
		goto err2;
	}

	g_tmpa9xx_i2s_priv = p;

	p->dev = &pdev->dev;

	platform_set_drvdata(pdev, p);

	I2SCOMMON = 0x19;		/* IISSCLK = Fosch(X1),       Set SCK/WS/CLKO of Tx and Rx as Common */
	I2STMCON = 0x04;		/* I2SMCLK = Fosch/4 = 11.2896M Hz */
	I2SRMCON = 0x04;
	I2STCON = 0x00;			/* IIS Standard Format */
	I2STFCLR = 0x01;		/* Clear FIFO */
	I2SRMS = 0x00;			/* Slave */
	I2STMS = 0x00;			/* Slave */

	dev_dbg(p->dev, "\n");

	return 0;

err2:
	tmpa9xx_dma_free(p->rx.dma_ch);
err1:
	kfree(p);
err0:
	return ret;
}

static int __devexit remove(struct platform_device *pdev)
{
	struct tmpa9xx_i2s_priv *p = platform_get_drvdata(pdev);

	i2s_rx_shutdown();
	i2s_tx_shutdown();

	tmpa9xx_dma_free(p->tx.dma_ch);
	tmpa9xx_dma_free(p->rx.dma_ch);

	platform_set_drvdata(pdev, NULL);

	g_tmpa9xx_i2s_priv = NULL;

	kfree(p);

	return 0;
}

static struct platform_driver tmpa9xx_i2s_driver = {
	.probe = probe,
	.remove = __devexit_p(remove),
	.driver = {
		.name  = "tmpa9xx-i2s",
		.owner = THIS_MODULE,
	},
};

static int __init tmpa9xx_i2s_init_module(void)
{
	return platform_driver_register(&tmpa9xx_i2s_driver);
}

static void __exit tmpa9xx_i2s_exit_module(void)
{
	platform_driver_unregister(&tmpa9xx_i2s_driver);
}

module_init(tmpa9xx_i2s_init_module);
module_exit(tmpa9xx_i2s_exit_module);

MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("I2S driver for TMPA9xx");
MODULE_LICENSE("GPL");
