#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include <mach/dma.h>
#include <mach/regs.h>

#include "tmpa9xx_i2s.h"

#define CON(st)		((st->is_rx * 0x20) + 0x000)
#define SLVON(st)	((st->is_rx * 0x20) + 0x004)
#define FCLR(st)	((st->is_rx * 0x20) + 0x008)
#define MS(st)		((st->is_rx * 0x20) + 0x00C)
#define MCON(st)	((st->is_rx * 0x20) + 0x010)
#define MSTP(st)	((st->is_rx * 0x20) + 0x014)
#define DMA1(st)	((st->is_rx * 0x20) + 0x018)

#define COMMON	(0x044)
#define TST	(0x048)
#define RST	(0x04C)
#define INT	(0x050)
#define INTMSK	(0x054)

#define i2s_writel(b, o, v)	writel(v, b->regs + o)
#define i2s_readl(b, o)		readl(b->regs + o)

#define I2STDAT_ADR            (I2S_BASE + 0x1000)
#define I2SRDAT_ADR            (I2S_BASE + 0x2000)

struct scatter_dma_t {
	unsigned long srcaddr;
	unsigned long dstaddr;
	unsigned long lli;
	unsigned long control;
};

struct i2s_stream
{
	int dma_ch;
	bool is_rx;

	/* DMA descriptor ring head of current audio stream*/
	struct scatter_dma_t *dma_desc;
	unsigned int desc_bytes;
	unsigned int dma_phydesc;
	unsigned int dma_buf;
	unsigned int run; /* is running */
	struct scatter_dma_t *curr_desc;

	void (*callback)(void *data);
	void *data;
};

struct tmpa9xx_i2s_priv
{
	void __iomem *regs;
	struct device *dev;

	struct i2s_stream rx;
	struct i2s_stream tx;
};

struct tmpa9xx_i2s_priv *g_tmpa9xx_i2s_priv;

static void stream_start(struct i2s_stream *s)
{
	struct tmpa9xx_i2s_priv *i = g_tmpa9xx_i2s_priv;
	struct scatter_dma_t *dma_desc;

	if (s->run)
		return;

	s->curr_desc = s->dma_desc;
	dma_desc = s->curr_desc;

        DMA_SRC_ADDR(s->dma_ch)  = dma_desc->srcaddr;
	DMA_DEST_ADDR(s->dma_ch) = dma_desc->dstaddr;
	DMA_LLI(s->dma_ch)       = dma_desc->lli;
	DMA_CONTROL(s->dma_ch)   = dma_desc->control;
	if (s->is_rx)
		DMA_CONFIG(s->dma_ch) = 0x00009017;
	else
		DMA_CONFIG(s->dma_ch) = 0x00008a81;

	tmpa9xx_dma_enable(s->dma_ch);

	/* I2S DMA set complete */
	i2s_writel(i, DMA1(s), 0x01);
	/* I2S transfer start */
	i2s_writel(i, SLVON(s), 0x01);

	s->run = 1;
}

static void stream_stop(struct i2s_stream *s)
{
	struct tmpa9xx_i2s_priv *i = g_tmpa9xx_i2s_priv;

	if (!s->run)
		return;

	i2s_writel(i,  DMA1(s), 0x00);
	i2s_writel(i, SLVON(s), 0x00);

	tmpa9xx_dma_disable(s->dma_ch);
	s->run = 0;
}

static void stream_shutdown(struct i2s_stream *s)
{
	if (!s->dma_desc)
		return;

	dma_free_coherent(NULL, s->desc_bytes, s->dma_desc, s->dma_phydesc);
	s->desc_bytes = 0;
	s->dma_desc = 0;
	s->dma_phydesc = 0;
}

static int stream_setup(struct i2s_stream *s, struct tmpa9xx_i2s_config *c)
{
	struct scatter_dma_t *desc;
	unsigned int count;
	dma_addr_t addr;
	int i;

	stream_shutdown(s);

	s->callback = c->callback;
	s->data = c->data;

	count = c->fragsize / c->size;

	/* for fragments larger than 16k words we use 2d dma,
	 * denote fragecount as two numbers' mutliply and both of them
	 * are less than 64k.*/
	if (count >= 0x1000) {
		return -EINVAL;
	}

	s->dma_desc = dma_alloc_coherent(NULL, c->fragcount * sizeof(struct scatter_dma_t), &addr, GFP_USER);
	s->desc_bytes = c->fragcount * sizeof(struct scatter_dma_t);
	s->dma_phydesc = addr;
	s->dma_buf = c->phy_buf;

	if (!s->dma_desc) {
		return -ENOMEM;
	}

	desc = s->dma_desc;
	for (i=0; i<c->fragcount; i++) {
		desc[i].lli = (unsigned long)(addr + (i + 1) * sizeof(struct scatter_dma_t));
		if(s->is_rx) {
			desc[i].srcaddr = (unsigned long)I2SRDAT_ADR;
			desc[i].dstaddr = (unsigned long)(c->phy_buf + i*c->fragsize);
			desc[i].control = 0x88492000 + (unsigned long)(c->fragsize >> 2);
		} else {
			desc[i].srcaddr = (unsigned long)(c->phy_buf + i*c->fragsize);
			desc[i].dstaddr = (unsigned long)I2STDAT_ADR;
			desc[i].control = 0x84492000 + (unsigned long)(c->fragsize >> 2);
		}
	}

	/* make circular */
	desc[c->fragcount-1].lli = (unsigned long)addr;

	return 0;
}

int tmpa9xx_i2s_tx_setup(struct tmpa9xx_i2s_config *c)
{
	struct tmpa9xx_i2s_priv *i = g_tmpa9xx_i2s_priv;
	return stream_setup(&i->tx, c);
}

int tmpa9xx_i2s_rx_setup(struct tmpa9xx_i2s_config *c)
{
	struct tmpa9xx_i2s_priv *i = g_tmpa9xx_i2s_priv;
	return stream_setup(&i->rx, c);
}

int tmpa9xx_i2s_tx_start(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;
	stream_start(&i2s->tx);
	return 0;
}

int tmpa9xx_i2s_rx_start(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;
	stream_start(&i2s->rx);
	return 0;
}

int tmpa9xx_i2s_tx_stop(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;
	stream_stop(&i2s->tx);
	return 0;
}

int tmpa9xx_i2s_rx_stop(void)
{
	struct tmpa9xx_i2s_priv *i2s = g_tmpa9xx_i2s_priv;
	stream_stop(&i2s->rx);
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

	if (i2s->tx.callback)
		i2s->tx.callback(i2s->tx.data);
}

static void rx_handler(int dma_ch, void *dev_id)
{
	struct tmpa9xx_i2s_priv *i2s = dev_id;

	if (i2s->rx.callback)
		i2s->rx.callback(i2s->rx.data);
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

	p->regs = ioremap(I2S_BASE	, 0x2fff); // res->start, resource_size(res));
	if (!p->regs) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto err3;
	}

	g_tmpa9xx_i2s_priv = p;

	p->dev = &pdev->dev;

	platform_set_drvdata(pdev, p);

	p->rx.is_rx = true;

	i2s_writel(p, COMMON, 0x19); /* IISSCLK = Fosch(X1), Set SCK/WS/CLKO of Tx and Rx as Common */

	i2s_writel(p, MCON((&p->tx)), 0x04); /* I2SMCLK = Fosch/4 = 11.2896M Hz */
	i2s_writel(p,  CON((&p->tx)), 0x00); /* I2S standard format */
	i2s_writel(p, FCLR((&p->tx)), 0x01); /* clear fifo */
	i2s_writel(p,   MS((&p->tx)), 0x00); /* slave */

	i2s_writel(p, MCON((&p->rx)), 0x04);
	i2s_writel(p,   MS((&p->rx)), 0x00); /* slave */

	dev_dbg(p->dev, "\n");

	return 0;

err3:
	tmpa9xx_dma_free(p->tx.dma_ch);
err2:
	tmpa9xx_dma_free(p->rx.dma_ch);
err1:
	kfree(p);
err0:
	return ret;
}

static int __devexit remove(struct platform_device *pdev)
{
	struct tmpa9xx_i2s_priv *i = platform_get_drvdata(pdev);

	stream_shutdown(&i->rx);
	stream_shutdown(&i->tx);

	tmpa9xx_dma_free(i->tx.dma_ch);
	tmpa9xx_dma_free(i->rx.dma_ch);

	iounmap(i->regs);

	platform_set_drvdata(pdev, NULL);

	g_tmpa9xx_i2s_priv = NULL;

	kfree(i);

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
