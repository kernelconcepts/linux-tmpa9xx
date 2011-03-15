#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include <mach/dma.h>
#include <mach/regs.h>

#include "tmpa9xx_i2s.h"

struct tmpa9xx_i2s_priv
{
	struct device *dev;
};

struct tmpa9xx_i2s_priv *g_tmpa9xx_i2s_priv;

static void setup_tx_desc(struct scatter_dma_t *desc, unsigned int phydesc,
            unsigned int phy_buf, int fragcount, size_t fragsize)
{
	int i;

	for (i=0; i<fragcount; i++) {
		desc[i].lli = (unsigned long)(phydesc + (i + 1) * sizeof(struct scatter_dma_t));
		desc[i].srcaddr = (unsigned long)(phy_buf + i*fragsize);    //Phy Addr
		desc[i].dstaddr = (unsigned long)I2STDAT_ADR;
		desc[i].control = 0x84492000 + (unsigned long)(fragsize >> 2);
	}

	/* make circular */
	desc[fragcount-1].lli = (unsigned long)phydesc;
}

static void setup_rx_desc(struct scatter_dma_t *desc, unsigned int phydesc,
			unsigned int phy_buf, int fragcount, size_t fragsize)
{
	int i;

	for (i=0; i<fragcount; ++i)
	{
		desc[i].lli  = (unsigned long)(phydesc + (i + 1) * sizeof(struct scatter_dma_t));
		desc[i].srcaddr = (unsigned long)I2SRDAT_ADR;
		desc[i].dstaddr = (unsigned long)(phy_buf + i*fragsize);    //Phy Addr
		desc[i].control = 0x88492000 + (unsigned long)(fragsize >> 2);
	}

	/* make circular */
	desc[fragcount-1].lli = (unsigned long)phydesc;
}

static int i2s_tx_start(struct tmpa9xx_i2s *i2s)
{
	tmpa9xx_dma_enable(i2s->dma_tx_ch);

	/* I2S DMA set complete */
	I2STDMA1 = 0x0001;
	/* I2S transfer start */
	I2STSLVON = 0x0001;

	return 0;
}

static int i2s_rx_start(struct tmpa9xx_i2s *i2s)
{
	tmpa9xx_dma_enable(i2s->dma_rx_ch);

	/*----I2S DMA set complete */
	I2SRDMA1 = 0x0001;

	/*----I2S transfer start */
	I2SRSLVON = 0x0001;

	return 0;
}

static int i2s_tx_stop(struct tmpa9xx_i2s *i2s)
{
	I2STDMA1 = 0x0000;
	I2STSLVON = 0x0000;

	tmpa9xx_dma_disable(i2s->dma_tx_ch);

	return 0;
}

static int i2s_rx_stop(struct tmpa9xx_i2s *i2s)
{
	I2SRDMA1 = 0x0000;
	I2SRSLVON = 0x0000;

	tmpa9xx_dma_disable(i2s->dma_rx_ch);

	return 0;
}

static inline int i2s_tx_dma_start(struct tmpa9xx_i2s *i2s)
{
	int dma_ch = i2s->dma_tx_ch;
	struct scatter_dma_t *dma_desc;

	i2s->curr_tx_desc = i2s->dma_tx_desc;
	dma_desc = i2s->curr_tx_desc;

        DMA_SRC_ADDR(dma_ch) = dma_desc->srcaddr;
	DMA_DEST_ADDR(dma_ch) = dma_desc->dstaddr;
	DMA_LLI(dma_ch) = dma_desc->lli;
	DMA_CONTROL(dma_ch) = dma_desc->control;
	DMA_CONFIG(dma_ch) = 0x00008a81;

	return 0;
}

static inline int i2s_rx_dma_start(struct tmpa9xx_i2s *i2s)
{
	int dma_ch = i2s->dma_rx_ch;
	struct scatter_dma_t *dma_desc;

	i2s->curr_rx_desc = i2s->dma_rx_desc;
	dma_desc = i2s->curr_rx_desc;

	DMA_SRC_ADDR(dma_ch) = dma_desc->srcaddr;
	DMA_DEST_ADDR(dma_ch) = dma_desc->dstaddr;
	DMA_LLI(dma_ch) = dma_desc->lli;
	DMA_CONTROL(dma_ch) = dma_desc->control;
	DMA_CONFIG(dma_ch) = 0x00009017;

	return 0;
}

int tmpa9xx_i2s_tx_start(struct tmpa9xx_i2s *i2s)
{
	if (i2s->tx_run)
		return -EBUSY;

	i2s_tx_dma_start(i2s);
	i2s_tx_start(i2s);
	i2s->tx_run = 1;

	return 0;
}

int tmpa9xx_i2s_rx_start(struct tmpa9xx_i2s *i2s)
{
	if (i2s->rx_run)
		return -EBUSY;

	i2s_rx_dma_start(i2s);
	i2s_rx_start(i2s);
	i2s->rx_run = 1;

	return 0;
}

int tmpa9xx_i2s_tx_stop(struct tmpa9xx_i2s *i2s)
{
	if (!i2s->tx_run)
		return 0;

	/* Both rx and tx dma stopped */
	i2s_tx_stop(i2s);
	i2s->curr_tx_desc = NULL;

	i2s->tx_run = 0;

	return 0;
}

int tmpa9xx_i2s_rx_stop(struct tmpa9xx_i2s *i2s)
{
	if (!i2s->rx_run)
		return 0;

	i2s_rx_stop(i2s);
	i2s->curr_rx_desc = NULL;

	i2s->rx_run = 0;

	return 0;
}

int tmpa9xx_i2s_config_tx_dma(struct tmpa9xx_i2s *i2s,
		unsigned char *cpu_buf, unsigned int phy_buf,
		int fragcount, size_t fragsize, size_t size)
{
	unsigned int count;
	dma_addr_t addr;

	count = fragsize / size;

	/* for fragments larger than 16k words we use 2d dma,
	 * denote fragecount as two numbers' mutliply and both of them
	 * are less than 64k.*/
	if (count >= 0x1000) {
		return -EINVAL;
	}

	if (i2s->dma_tx_desc) {
		dma_free_coherent(NULL, i2s->tx_desc_bytes, i2s->dma_tx_desc, i2s->dma_tx_phydesc);
	}

	i2s->dma_tx_desc = dma_alloc_coherent(NULL, fragcount * sizeof(struct scatter_dma_t), &addr, GFP_USER);
	i2s->tx_desc_bytes = fragcount * sizeof(struct scatter_dma_t);
	i2s->dma_tx_phydesc = addr;
	i2s->dma_tx_buf = phy_buf;

	if (!i2s->dma_tx_desc) {
		return -ENOMEM;
	}

	setup_tx_desc(i2s->dma_tx_desc, addr, phy_buf, fragcount, fragsize);

	return 0;
}

int tmpa9xx_i2s_config_rx_dma(struct tmpa9xx_i2s *i2s,
		unsigned char *cpu_buf, unsigned int phy_buf,
		int fragcount, size_t fragsize, size_t size)
{
	unsigned int count;
	dma_addr_t addr;

	count = fragsize / size;

	/* for fragments larger than 16k words we use 2d dma,
	 * denote fragecount as two numbers' mutliply and both of them
	 * are less than 64k.*/
	if (count >= 0x1000) {
		printk(KERN_INFO "Error: rx dma size too large %d\n", count);
		return -EINVAL;
	}

	if (i2s->dma_rx_desc) {
		dma_free_coherent(NULL, i2s->rx_desc_bytes, i2s->dma_rx_desc, i2s->dma_rx_phydesc);
	}

	i2s->dma_rx_desc = dma_alloc_coherent(NULL, fragcount * sizeof(struct scatter_dma_t), &addr, GFP_USER);
	i2s->rx_desc_bytes = fragcount * sizeof(struct scatter_dma_t);
	i2s->dma_rx_phydesc = addr;
	i2s->dma_rx_buf = phy_buf;

	if (!i2s->dma_rx_desc) {
		return -ENOMEM;
	}

	setup_rx_desc(i2s->dma_rx_desc, addr, phy_buf, fragcount, fragsize);

	return 0;
}

unsigned int tmpa9xx_i2s_curr_offset_tx(struct tmpa9xx_i2s *i2s)
{
	int dma_ch = i2s->dma_tx_ch;
	unsigned int addr, size;

	addr = DMA_SRC_ADDR(dma_ch);
	size = addr - i2s->dma_tx_buf;

	return size;
}

unsigned int tmpa9xx_i2s_curr_offset_rx(struct tmpa9xx_i2s *i2s)
{
	int dma_ch = i2s->dma_rx_ch;
	unsigned int addr, size;

	addr = DMA_DEST_ADDR(dma_ch);
	size = addr - i2s->dma_rx_buf;

	return size;
}

static int i2s_check_status(struct tmpa9xx_i2s *i2s,
		unsigned int *i2s_stat,
		unsigned int *rx_stat,
		unsigned int *tx_stat)
{
	int status = 0;

	return status;
}

static void tx_handler(int dma_ch, void *dev_id)
{
	unsigned int tx_stat;
	struct tmpa9xx_i2s *i2s = dev_id;

	i2s_check_status(i2s, NULL, NULL, &tx_stat);

	if (i2s->tx_callback) {
		i2s->tx_callback(i2s->data);
	}
}

static void rx_handler(int dma_ch, void *dev_id)
{
	unsigned int rx_stat;
	struct tmpa9xx_i2s *i2s = dev_id;

	i2s_check_status(i2s, NULL, NULL, &rx_stat);

	if (i2s->rx_callback) {
		i2s->rx_callback(i2s->data);
	}
}

static void err_handler(int dma_ch, void *dev_id)
{
	unsigned int status;
	struct tmpa9xx_i2s *i2s = dev_id;

	if (i2s_check_status(i2s, &status, NULL, NULL)) {
		printk(KERN_ERR "error checking status ??");
		return;
	}

	if (i2s->err_callback)
		i2s->err_callback(i2s->data);
}

struct tmpa9xx_i2s *tmpa9xx_i2s_init(
		void (*rx_callback)(void*),
		void (*tx_callback)(void*),
		void (*err_callback)(void*),
		void *data)
{
	struct tmpa9xx_i2s *i2s;

	i2s = kmalloc(sizeof(struct tmpa9xx_i2s), GFP_KERNEL);

	if (i2s == NULL) {
		printk(KERN_ERR "kmalloc() failed\n");
		return NULL;
	}
	memset(i2s, 0, sizeof(struct tmpa9xx_i2s));

 	i2s->dma_tx_ch = tmpa9xx_dma_request("I2S TX", 1, tx_handler,
					err_handler, i2s);
	if (i2s->dma_tx_ch < 0) {
		printk(KERN_ERR "request tx audio dma failed\n");
		goto __init_err2;
	}
	i2s->dma_rx_ch = tmpa9xx_dma_request("I2S RX", 2, rx_handler,
					err_handler, i2s);
	if (i2s->dma_rx_ch < 0) {
		printk(KERN_ERR "request rx audio dma failed\n");
		goto __init_err1;
	}
	i2s->rx_callback = rx_callback;
	i2s->tx_callback = tx_callback;
	i2s->err_callback = err_callback;
	i2s->data = data;

	return i2s;

__init_err1:
	tmpa9xx_dma_free(i2s->dma_rx_ch);
__init_err2:
	kfree(i2s);

	return NULL;
}

void tmpa9xx_i2s_free(struct tmpa9xx_i2s *i2s)
{
	if (i2s == NULL)
		return;

	i2s_tx_stop(i2s);
	i2s_rx_stop(i2s);

	if (i2s->dma_tx_desc) {
		dma_free_coherent(NULL, i2s->tx_desc_bytes, i2s->dma_tx_desc, i2s->dma_tx_phydesc);
	}
	if (i2s->dma_rx_desc) {
		dma_free_coherent(NULL, i2s->rx_desc_bytes, i2s->dma_rx_desc, i2s->dma_rx_phydesc);
	}

	tmpa9xx_dma_free(i2s->dma_tx_ch);
	tmpa9xx_dma_free(i2s->dma_rx_ch);

	kfree(i2s);
}


EXPORT_SYMBOL(tmpa9xx_i2s_rx_start);
EXPORT_SYMBOL(tmpa9xx_i2s_init);
EXPORT_SYMBOL(tmpa9xx_i2s_curr_offset_tx);
EXPORT_SYMBOL(tmpa9xx_i2s_config_tx_dma);
EXPORT_SYMBOL(tmpa9xx_i2s_rx_stop);
EXPORT_SYMBOL(tmpa9xx_i2s_config_rx_dma);
EXPORT_SYMBOL(tmpa9xx_i2s_curr_offset_rx);
EXPORT_SYMBOL(tmpa9xx_i2s_free);
EXPORT_SYMBOL(tmpa9xx_i2s_tx_start);
EXPORT_SYMBOL(tmpa9xx_i2s_tx_stop);

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

	g_tmpa9xx_i2s_priv = p;

	p->dev = &pdev->dev;

	platform_set_drvdata(pdev, p);

	dev_dbg(p->dev, "\n");

	return 0;

err0:
	return ret;
}

static int __devexit remove(struct platform_device *pdev)
{
	struct tmpa9xx_i2s_priv *p = platform_get_drvdata(pdev);

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
