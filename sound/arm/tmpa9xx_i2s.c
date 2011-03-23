/*
 * Author?
 * (c) ?
 *
 * Function: Handles TMPA I2S DMA
 *
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
 
#include <asm/bug.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <mach/dma.h>
#include <mach/regs.h>

/* I2S Interface   */
#define I2S_BASE                  (0xF2040000)
#define I2STCON                   __REG(I2S_BASE + 0x000)
#define I2STSLVON                 __REG(I2S_BASE + 0x004)
#define I2STFCLR                  __REG(I2S_BASE + 0x008)
#define I2STMS                    __REG(I2S_BASE + 0x00C)
#define I2STMCON                  __REG(I2S_BASE + 0x010)
#define I2STMSTP                  __REG(I2S_BASE + 0x014)
#define I2STDMA1                  __REG(I2S_BASE + 0x018)
#define I2SRCON                   __REG(I2S_BASE + 0x020)
#define I2SRSLVON                 __REG(I2S_BASE + 0x024)
#define I2SFRFCLR                 __REG(I2S_BASE + 0x028)
#define I2SRMS                    __REG(I2S_BASE + 0x02C)
#define I2SRMCON                  __REG(I2S_BASE + 0x030)
#define I2SRMSTP                  __REG(I2S_BASE + 0x034)
#define I2SRDMA1                  __REG(I2S_BASE + 0x038)
#define I2SCOMMON                 __REG(I2S_BASE + 0x044) 
#define I2STST                    __REG(I2S_BASE + 0x048)
#define I2SRST                    __REG(I2S_BASE + 0x04C)
#define I2SINT                    __REG(I2S_BASE + 0x050)
#define I2SINTMSK                 __REG(I2S_BASE + 0x054)
#define I2STDAT                   __REG(I2S_BASE + 0x1000)
#define I2SRDAT                   __REG(I2S_BASE + 0x2000)
#define I2STDAT_ADR              (I2S_BASE + 0x1000)
#define I2SRDAT_ADR              (I2S_BASE + 0x2000)

#include "tmpa9xx_i2s.h"

#undef TMPA9XX_I2S_DEBUG

#ifdef TMPA9XX_I2S_DEBUG
#define i2s_printd(level, format, arg...) \
		printk(level "i2s: " format, ## arg)
#define I2S_ASSERT(expr) \
	do { \
		if (unlikely(!(expr))) { \
			printk(KERN_ERR "%s: %d, bug\n", __FUNCTION__, __LINE__); \
		} \
	} while(0)
#else
#define i2s_printd(level, format, arg...)
#define I2S_ASSERT(expr)
#endif

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

	//printk("The fragcount is %d.\n",fragcount);	

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

	//printk("====> Start TX I2S\n");
	return 0;
}

static int i2s_rx_start(struct tmpa9xx_i2s *i2s)
{
	tmpa9xx_dma_enable(i2s->dma_rx_ch);

	/*----I2S DMA set complete */
	I2SRDMA1 = 0x0001;

	/*----I2S transfer start */
	I2SRSLVON = 0x0001;	

	//printk("====> Start RX I2S\n");
	return 0;
}

static int i2s_tx_stop(struct tmpa9xx_i2s *i2s)
{
	I2STDMA1 = 0x0000;
	I2STSLVON = 0x0000;

	tmpa9xx_dma_disable(i2s->dma_tx_ch);

	//printk("====> Stop TX I2S\n");

	return 0;
}

static int i2s_rx_stop(struct tmpa9xx_i2s *i2s)
{
	I2SRDMA1 = 0x0000;
	I2SRSLVON = 0x0000;	

	tmpa9xx_dma_disable(i2s->dma_rx_ch);

	//printk("====> Stop RX I2S\n");

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
//	printk("srcaddr = 0x%02x\n", dma_desc->srcaddr);
	DMA_DEST_ADDR(dma_ch) = dma_desc->dstaddr;
//        printk("dest addr = 0x%02x\n", dma_desc->dstaddr);	
	DMA_LLI(dma_ch) = dma_desc->lli;
//        printk("lli addr = 0x%02x\n", dma_desc->lli);
	DMA_CONTROL(dma_ch) = dma_desc->control;
//        printk("control = 0x%02x\n", dma_desc->control);
	DMA_CONFIG(dma_ch) = 0x00009017;

	return 0;
}

int tmpa9xx_i2s_tx_start(struct tmpa9xx_i2s *i2s)
{
	i2s_printd(KERN_ERR, "%s: tx_run:%d\n", __FUNCTION__, i2s->tx_run);

	if (i2s->tx_run)
		return -EBUSY;

	i2s_tx_dma_start(i2s);
	i2s_tx_start(i2s);
	i2s->tx_run = 1;

	return 0;
}

int tmpa9xx_i2s_rx_start(struct tmpa9xx_i2s *i2s)
{
	i2s_printd(KERN_ERR, "%s: rx_run:%d\n", __FUNCTION__, i2s->rx_run);

	if (i2s->rx_run)
		return -EBUSY;

	i2s_rx_dma_start(i2s);
	i2s_rx_start(i2s);
	i2s->rx_run = 1;

	return 0;
}

int tmpa9xx_i2s_tx_stop(struct tmpa9xx_i2s *i2s)
{
	i2s_printd(KERN_ERR, "%s: rx_run:%d\n", __FUNCTION__, i2s->rx_run);
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
	i2s_printd(KERN_ERR, "%s: rx_run:%d\n", __FUNCTION__, i2s->rx_run);
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

	i2s_printd(KERN_ERR, "%s( %p, %X, %d, %x, %x )\n", __FUNCTION__, cpu_buf, phy_buf,
			fragcount, fragsize, size);
 
	count = fragsize / size;

	/* for fragments larger than 16k words we use 2d dma, 
	 * denote fragecount as two numbers' mutliply and both of them 
	 * are less than 64k.*/
	if (count >= 0x1000) {
		i2s_printd(KERN_ERR, "Error: tx dma size too large %d\n", count);
		return -EINVAL;
	}

	if (i2s->dma_tx_desc) {
		i2s_printd(KERN_ERR, "+dma_free_coherent dma_tx_desc=0x%08lx\n", (unsigned long)i2s->dma_tx_desc);
		dma_free_coherent(NULL, i2s->tx_desc_bytes, i2s->dma_tx_desc, i2s->dma_tx_phydesc);
		i2s_printd(KERN_ERR, "-dma_free_coherent\n");
	}

	i2s_printd(KERN_ERR, "+dma_alloc_coherent\n");
	i2s->dma_tx_desc = dma_alloc_coherent(NULL, fragcount * sizeof(struct scatter_dma_t), &addr, GFP_USER);
	i2s_printd(KERN_ERR, "-dma_alloc_coherent dma_tx_desc=0x%08lx\n", (unsigned long)i2s->dma_tx_desc);
	i2s->tx_desc_bytes = fragcount * sizeof(struct scatter_dma_t);
	i2s->dma_tx_phydesc = addr;
	i2s->dma_tx_buf = phy_buf;

	if (!i2s->dma_tx_desc) {
		return -ENOMEM;
	}

	i2s_printd(KERN_ERR, "+setup_tx_desc\n");
	setup_tx_desc(i2s->dma_tx_desc, addr, phy_buf, fragcount, fragsize);
	i2s_printd(KERN_ERR, "-setup_tx_desc\n");
	
	return 0;
}

int tmpa9xx_i2s_config_rx_dma(struct tmpa9xx_i2s *i2s, 
		unsigned char *cpu_buf, unsigned int phy_buf,
		int fragcount, size_t fragsize, size_t size)
{
	unsigned int count;
	dma_addr_t addr;

	i2s_printd(KERN_ERR, "%s( %p, %X, %d, %x, %x )\n", __FUNCTION__, cpu_buf, phy_buf,
			fragcount, fragsize, size);

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
    
	//printk("size[%d]", size);

	return size;
}

unsigned int tmpa9xx_i2s_curr_offset_rx(struct tmpa9xx_i2s *i2s)
{
	int dma_ch = i2s->dma_rx_ch;
	unsigned int addr, size;
	
	addr = DMA_DEST_ADDR(dma_ch);
	size = addr - i2s->dma_rx_buf;
    
	//printk("size[%d]", size);

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
	

	i2s_printd(KERN_ERR, "%s\n", __FUNCTION__);

	i2s_check_status(i2s, NULL, NULL, &tx_stat);

	if (i2s->tx_callback) {
		i2s->tx_callback(i2s->data);
	}
}

static void rx_handler(int dma_ch, void *dev_id)
{
	unsigned int rx_stat;
	struct tmpa9xx_i2s *i2s = dev_id;
	

	i2s_printd(KERN_ERR, "%s\n", __FUNCTION__);
	i2s_check_status(i2s, NULL, NULL, &rx_stat);

	if (i2s->rx_callback) {
		i2s->rx_callback(i2s->data);
	}

        //printk("handler srcaddr = 0x%02x\n", DMA_SRC_ADDR(dma_ch));
        //printk("handler dest addr = 0x%02x\n", DMA_DEST_ADDR(dma_ch));
        //printk("handler lli addr = 0x%02x\n", DMA_LLI(dma_ch));
        //printk("handler control = 0x%02x\n", DMA_CONTROL(dma_ch));
}

static void err_handler(int dma_ch, void *dev_id)
{
	unsigned int status;
	struct tmpa9xx_i2s *i2s = dev_id;

	i2s_printd(KERN_ERR, "%s\n", __FUNCTION__);
	if (i2s_check_status(i2s, &status, NULL, NULL)) {
		printk(KERN_ERR "error checking status ??");
		return;
	}

	if (i2s->err_callback)
		i2s->err_callback(i2s->data);
}

struct tmpa9xx_i2s *tmpa9xx_i2s_init(
		int dma_rx, void (*rx_callback)(void*),
		int dma_tx, void (*tx_callback)(void*),
		int err_irq, void (*err_callback)(void*),
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
		printk(KERN_ERR "request tx audio dma 0x%x failed\n", dma_tx);
		goto __init_err2;
	}
	//printk("dma_tx_ch = %d\n", i2s->dma_tx_ch);
	
	i2s->dma_rx_ch = tmpa9xx_dma_request("I2S RX", 2, rx_handler, 
					err_handler, i2s);
	if (i2s->dma_rx_ch < 0) {
		printk(KERN_ERR "request rx audio dma 0x%x failed\n", dma_rx);
		goto __init_err1;
	}
	//printk("dma_rx_ch = %d\n", i2s->dma_rx_ch);
	
	i2s->err_irq = err_irq;
	i2s->rx_callback = rx_callback;
	i2s->tx_callback = tx_callback;
	i2s->err_callback = err_callback;
	i2s->data = data;

	//i2s_printd(KERN_ERR, "dma tx: %p\n", i2s->dma_tx);

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

	i2s_printd(KERN_ERR, "+%s\n", __FUNCTION__);

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
	i2s_printd(KERN_ERR, "-%s\n", __FUNCTION__);
}
