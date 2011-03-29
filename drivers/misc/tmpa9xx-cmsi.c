#define DEBUG
/*
 * CMSI driver for Toshiba TMPA9xx processors.
 *
 * Copyright (C) 2011 Michael Hunold <michael@mihu.de>
 *
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/poll.h>
#include <linux/dma-mapping.h>
#include <asm/bug.h>

#include <mach/dma.h>
#include <mach/regs.h>

#define DRIVER_DESC "TMPA9xx CMSI driver"

#define CR	(0x000)	/* Control Register */
#define CV	(0x004)	/* Color Space Conversion Register */
#define CVP0	(0x008)	/* Color Conversion Parameter Register0 */
#define CVP1	(0x00C)	/* Color Conversion Parameter Register1 */
#define YD	(0x010)	/* Soft Conversion Y-data Resister */
#define UD	(0x014)	/* Soft Conversion U-data Resister */
#define VD	(0x018)	/* Soft Conversion V-data Resister */
#define FPT	(0x020)	/* FIFO Port Read Register */
#define CSTR	(0x024)	/* Scaling & Trimming Control Register */
#define TS	(0x030)	/* Trimming Space Start Point Setting Register */
#define TE	(0x034)	/* Trimming Space End Point Setting Register */
#define SCDMA	(0x040)	/* Soft Conversion DMA YUV-Data */

#define CR_CSRST 	(1 <<  0)
#define CR_CSIZE_QVGA	(1 <<  1)
#define CR_CFPCLR 	(1 <<  6)
#define CR_CHBKPH	(1 << 10)
#define CR_CSINTM	(1 << 15)
#define CR_CFINTF	(1 << 16)
#define CR_CSINTF	(1 << 17)

#define CV_DMAEN		(1 <<  2)
#define CV_YUV422_48_RGB888	((1 << 5) | (1 << 3))

#define cmsi_writel(b, o, v)	writel(v, b->regs + o)
#define cmsi_readl(b, o)	readl(b->regs + o)

#define OVCAM_GAIN  0x00
#define OVCAM_BLUE  0x01
#define OVCAM_RED   0x02
#define OVCAM_SAT   0x03
#define OVCAM_CTR   0x05
#define OVCAM_BRT   0x06
#define OVCAM_SHP   0x07
#define OVCAM_ABLU  0x0C
#define OVCAM_ARED  0x0D
#define OVCAM_COMR  0x0E

#define OVCAM_COMS  0x0F
#define OVCAM_AEC   0x10
#define OVCAM_CLKRC 0x11
  #define OVCAM_CLKRC_HN_CHN_VP  (0x0<<6)
  #define OVCAM_CLKRC_HN_CHN_VN  (0x1<<6)
  #define OVCAM_CLKRC_HP_CHN_VP  (0x2<<6)
  #define OVCAM_CLKRC_HP_CHP_VP  (0x3<<6)
  #define OVCAM_CLKRC_PRESCALER_MASK (0x3f)

#define OVCAM_COMA  0x12
  #define OVCAM_COMA_TESTBAR   (1<<1)
  #define OVCAM_COMA_AWB       (1<<2)
  #define OVCAM_COMA_VIDRGB    (1<<3)
  #define OVCAM_COMA_YUYV      (1<<4)
  #define OVCAM_COMA_AGCEN     (1<<5)
  #define OVCAM_COMA_MIRR      (1<<6)
  #define OVCAM_COMA_SRST      (1<<7)

#define OVCAM_COMB  0x13
  #define OVCAM_COMB_AUTOADJ        (1<<0)
  #define OVCAM_COMB_SINGLEFR       (1<<1)
  #define OVCAM_COMB_3STATE_BUS     (1<<2)
  #define OVCAM_COMB_CHSYNC         (1<<3)
  #define OVCAM_COMB_ITU656         (1<<4)
  #define OVCAM_COMB_8BIT           (1<<5)

#define OVCAM_COMC  0x14
#define OVCAM_COMD  0x15

#define OVCAM_FSD     0x16
#define OVCAM_HREFST  0x17
  #define OVCAM_HREFST_MIN 0x38
  #define OVCAM_HREFST_MAX 0xeb

#define OVCAM_HREFEND 0x18
  #define OVCAM_HREFEND_MIN 0x39
  #define OVCAM_HREFEND_MAX 0xec

#define OVCAM_VSTRT   0x19
  #define OVCAM_VSTRT_MIN 0x03
  #define OVCAM_VSTRT_MAX 0x93

#define OVCAM_VEND    0x1A
  #define OVCAM_VEND_MIN 0x04
  #define OVCAM_VEND_MAX 0x94

#define OVCAM_PSHFT   0x1B
#define OVCAM_MIDH    0x1C
#define OVCAM_MIDL    0x1D
#define OVCAM_COME    0x20

#define OVCAM_COMO    0x3E

#define i2c_packet_send(reg, val) \
	{ \
	int ret = i2c_smbus_write_byte_data(m->i2c_client, reg, val); \
	if (ret) \
		dev_err(m->dev, "i2c_smbus_write_byte_data() failed @ 0x%02x, ret %d\n", reg, ret); \
	} \
	mdelay(10);

#define dev_dbg_list(...)
#define dev_dbg_mm dev_dbg

struct scatter_dma_t {
	unsigned long srcaddr;
	unsigned long dstaddr;
	unsigned long lli;
	unsigned long control;
};

struct cmsi_buf
{
	void *ptr;
	size_t len;
};

#define CMSI_QBUF	_IOW('C', 0x02, struct cmsi_buf)
#define CMSI_DQBUF	_IOW('C', 0x03, struct cmsi_buf)

struct buf
{
	struct list_head item;

	struct cmsi_buf buf;

	struct scatter_dma_t *dma_desc;
	unsigned int desc_bytes;
	unsigned int dma_phydesc;
	unsigned int dma_buf;
};

struct tmpa9xx_cmsi_priv
{
	void __iomem *regs;
	struct device *dev;
	struct mutex lock;
	struct completion complete;
	bool should_stop;
	int irq;
	int dma;
	struct i2c_client *i2c_client;
	int type;
	bool is_running;

	struct list_head todo;
	struct buf *cur;
	struct list_head done;
	wait_queue_head_t queue;
};

struct tmpa9xx_cmsi_priv *g_ma;

static void cmsi_stop_helper(struct tmpa9xx_cmsi_priv *m)
{
	uint32_t val;

	val = cmsi_readl(m, CR);
	val |= CR_CSRST;
	cmsi_writel(m, CR, val);

	m->is_running = false;
}

static int setup_next_buffer(struct tmpa9xx_cmsi_priv *m)
{
	BUG_ON(m->cur);

	if (list_empty(&m->todo)) {
		dev_dbg_list(m->dev, "%s(): todo list is empty\n", __func__);
		cmsi_stop_helper(m);
		return -1;
	}

	m->cur = list_first_entry(&m->todo, struct buf, item);
	list_del(&m->cur->item);

	dev_dbg_list(m->dev, "%s(): current is now %p\n", __func__, m->cur);
	return 0;
}

static int cmsi_start(struct tmpa9xx_cmsi_priv *m)
{
	uint32_t val;
	int ret;

	if (m->is_running) {
		dev_dbg_list(m->dev, "%s(): already running\n", __func__);
		return 0;
	}

	ret = setup_next_buffer(m);
	if (ret) {
		dev_dbg_list(m->dev, "%s(): setup_next_buffer() failed\n", __func__);
		return -EFAULT;
	}

	val = cmsi_readl(m, CR);
	val &= ~CR_CSRST;
	val |= CR_CFPCLR;
	cmsi_writel(m, CR, val);

	m->is_running = true;

	dev_dbg_list(m->dev, "%s(): started processing\n", __func__);

	return 0;
}

static void finish_buf(struct tmpa9xx_cmsi_priv *m, struct buf *b)
{
	if (!b)
		return;

	dev_dbg_list(m->dev, "%s(): buf %p\n", __func__, b);

	dma_free_coherent(NULL, b->desc_bytes, b->dma_desc, b->dma_phydesc);

	list_del(&b->item);

	kfree(b);
}

static int cmsi_stop(struct tmpa9xx_cmsi_priv *m)
{
	struct buf *b, *next;

	if (m->is_running) {
		init_completion(&m->complete);

		m->should_stop = true;

		wait_for_completion(&m->complete);
	}

	BUG_ON(m->cur);

	dev_dbg_list(m->dev, "%s(): flushing todo list\n", __func__);
	list_for_each_entry_safe(b, next, &m->todo, item)
		finish_buf(m, b);

	dev_dbg_list(m->dev, "%s(): flushing done list\n", __func__);
	list_for_each_entry_safe(b, next, &m->done, item)
		finish_buf(m, b);

	dev_dbg_list(m->dev, "%s(): stopped processing\n", __func__);

	return 0;
}

static int tmpa9xx_cmsi_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int tmpa9xx_cmsi_close(struct inode *inode, struct file *file)
{
	struct tmpa9xx_cmsi_priv *m = g_ma;

	return cmsi_stop(m);
}

unsigned int tmpa9xx_cmsi_poll(struct file *file, struct poll_table_struct *wait)
{
	struct tmpa9xx_cmsi_priv *m = g_ma;
        unsigned int mask = 0;

	poll_wait(file, &m->queue, wait);

	if (!list_empty(&m->done))
		mask |= POLLIN;

	dev_dbg_list(m->dev, "%s(): mask 0x%08x\n", __func__, mask);

	return mask;
}

#if 0
struct dmabuf
{
};

static int pin_down_user_pages(struct tmpa9xx_cmsi_priv *m, struct buf *b)
{
	unsigned long data = (unsigned long)b->buf.ptr;
	unsigned long size = b->buf.len;
        unsigned long first, last;
	struct dmabuf *dma = &b->dma;
	int i;
	int ret;

	first = (data	       & PAGE_MASK) >> PAGE_SHIFT;
	last  = ((data+size-1) & PAGE_MASK) >> PAGE_SHIFT;
	dma->offset = data & ~PAGE_MASK;
	dma->size = size;
	dma->nr_pages = last-first+1;
	dma->pages = kmalloc(dma->nr_pages * sizeof(struct page *), GFP_KERNEL);
	if (NULL == dma->pages)
		return -ENOMEM;

	dev_dbg(m->dev, "%s(): adr %p, size %d, offset %d, nr_pages %d\n", __func__, b->buf.ptr, dma->size, dma->offset, dma->nr_pages);
#if 0
	ret = get_user_pages(current, current->mm,
		data & PAGE_MASK, dma->nr_pages,
		1, /* write */ 1, /* force */
		dma->pages, NULL);

	for (i = 0; i < dma->nr_pages; i++) {
		struct page *p = dma->pages[i];
		dev_dbg(m->dev, "%s(): %d => count %d\n", __func__, i, p->_count.counter);
#if 0
		set_page_dirty(dma->pages[i]);
		page_cache_release(dma->pages[i]);
#endif
	}
#endif
	kfree(dma->pages);

	return 0;
}
#endif

static int cmsi_qbuf(struct tmpa9xx_cmsi_priv *m, struct cmsi_buf *a)
{
	struct scatter_dma_t *desc;
	dma_addr_t addr;
	struct buf *b;
	int i;
	int ret;

	b = kzalloc(sizeof(b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	b->buf = *a;

	b->dma_desc = dma_alloc_coherent(NULL, 240 * sizeof(struct scatter_dma_t), &addr, GFP_USER);
	BUG_ON(!b->dma_desc);

	b->dma_phydesc = addr;
	b->desc_bytes = 240 * sizeof(struct scatter_dma_t);

#define FB 0x43a00000
#define BPP 4
#define W 320

	desc = b->dma_desc;
	for (i = 0; i < 239; i++) {
		desc[i].srcaddr = (unsigned long)(CMOSCAM_BASE+0x20);
		desc[i].dstaddr = (unsigned long)(FB + W * BPP * i);
		desc[i].lli = (unsigned long)(b->dma_phydesc + (i + 1) * sizeof(struct scatter_dma_t));
		desc[i].control = 0x0849B140;
	}
	desc[i].srcaddr = (unsigned long)(CMOSCAM_BASE+0x20);
	desc[i].dstaddr = (unsigned long)(FB + W * BPP * i);
	desc[i].lli = 0;
	desc[i].control = (1 << 31) | 0x0849B140;

        DMA_SRC_ADDR(m->dma)  = desc[0].srcaddr;
	DMA_DEST_ADDR(m->dma) = desc[0].dstaddr;
	DMA_LLI(m->dma)       = desc[0].lli;
	DMA_CONTROL(m->dma)   = desc[0].control;
	DMA_CONFIG(m->dma)    = 0x0000D24B;

	list_add_tail(&b->item, &m->todo);

	dev_dbg_list(m->dev, "%s(): new buf %p\n", __func__, b);

	ret = cmsi_start(m);
	if (ret)
		return ret;

	return 0;
}

static int cmsi_dqbuf(struct tmpa9xx_cmsi_priv *m, struct cmsi_buf *a)
{
	struct buf *b;

	if (list_empty(&m->done)) {
		dev_dbg_list(m->dev, "%s(): done list is empty\n", __func__);
		return -1;
	}

	b = list_first_entry(&m->done, struct buf, item);

	memcpy(a, &b->buf, sizeof(*a));
	dev_dbg_list(m->dev, "%s(): old buf %p\n", __func__, b);

	finish_buf(m, b);

	return 0;
}

static long tmpa9xx_cmsi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct tmpa9xx_cmsi_priv *m = g_ma;
	void __user *argp = (void __user *)arg;
	int ret;

	if (cmd == CMSI_QBUF) {
		struct cmsi_buf a;

		ret = copy_from_user(&a, argp, sizeof(a));
		if (ret) {
			dev_dbg(m->dev, "%s(): copy_from_user() failed, ret %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = cmsi_qbuf(m, &a);
		if (ret) {
			dev_dbg(m->dev, "%s(): cmsi_qbuf() failed, ret %d\n", __func__, ret);
			return -EFAULT;
		}

		return 0;
	}

	if (cmd == CMSI_DQBUF) {
		struct cmsi_buf a;

		ret = cmsi_dqbuf(m, &a);
		if (ret) {
			dev_dbg(m->dev, "%s(): cmsi_dqbuf() failed, ret %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = copy_to_user(argp, &a, sizeof(a));
		if (ret) {
			dev_dbg(m->dev, "%s(): copy_to_user() failed, ret %d\n", __func__, ret);
			return -EFAULT;
		}

		return 0;
	}

	dev_dbg(m->dev, "%s(): EOPNOTSUPP\n", __func__);
	return -EOPNOTSUPP;
}

static int probe_cmsi(struct tmpa9xx_cmsi_priv *m)
{
	int i;

	dev_info(m->dev, "%s():\n", __func__);

	i2c_packet_send(OVCAM_COMA,	OVCAM_COMA_SRST);
	i2c_packet_send(OVCAM_CLKRC,	0x01);
	i2c_packet_send(OVCAM_COMA,	0x24);
	i2c_packet_send(OVCAM_COMB,	0x21);
	i2c_packet_send(OVCAM_COMD,	0x00);
	i2c_packet_send(OVCAM_HREFST,	0x38);
	i2c_packet_send(OVCAM_HREFEND,	0xda);
	i2c_packet_send(OVCAM_VSTRT,	0x03);
	i2c_packet_send(OVCAM_VEND,	0x7a);
	i2c_packet_send(OVCAM_COMA,	0x24);

	cmsi_writel(m, CR, CR_CSINTM | CR_CHBKPH | CR_CFPCLR | CR_CSIZE_QVGA | CR_CSRST);
	cmsi_writel(m, CV, CV_YUV422_48_RGB888 | CV_DMAEN);

	cmsi_writel(m, CVP0, 0x1d2b3a2b);
	cmsi_writel(m, CVP1, 0x703d2b0e);

	cmsi_writel(m, YD, 0x0);
	cmsi_writel(m, UD, 0x0);
	cmsi_writel(m, VD, 0x0);

	return 0;
}

static int remove_cmsi(struct tmpa9xx_cmsi_priv *m)
{
	cmsi_stop(m);

	return 0;
}

static irqreturn_t interrupt_handler(int irq, void *ptr)
{
	struct tmpa9xx_cmsi_priv *m = ptr;
	struct scatter_dma_t *desc;
	uint32_t val;

	BUG_ON(!m->cur);
	desc = m->cur->dma_desc;

	val = cmsi_readl(m, CR);
	val &= ~(CR_CFINTF | CR_CSINTF);
	val |= CR_CFPCLR;
	cmsi_writel(m, CR, val);

        DMA_SRC_ADDR(m->dma)  = desc[0].srcaddr;
	DMA_DEST_ADDR(m->dma) = desc[0].dstaddr;
	DMA_LLI(m->dma)       = desc[0].lli;
	DMA_CONTROL(m->dma)   = desc[0].control;
	DMA_CONFIG(m->dma)    = 0x0000D24B;

	tmpa9xx_dma_enable(m->dma);

	return IRQ_HANDLED;
}

static void dma_handler(int channel, void *ptr)
{
	struct tmpa9xx_cmsi_priv *m = ptr;
	int ret;

	tmpa9xx_dma_disable(m->dma);

	BUG_ON(!m->cur);

	list_add_tail(&m->cur->item, &m->done);
	m->cur = NULL;
	wake_up(&m->queue);

	if (m->should_stop) {
		cmsi_stop_helper(m);
		dev_dbg_list(m->dev, "%s(): stop requested\n", __func__);
		m->should_stop = false;
		complete(&m->complete);
		return;
	}

	ret = setup_next_buffer(m);
	if (ret) {
		cmsi_stop_helper(m);
		dev_dbg_list(m->dev, "%s(): no more buffers. stopping\n", __func__);
		return;
	}
}

static void err_handler(int dma_ch, void *ptr)
{
	struct tmpa9xx_cmsi_priv *m = ptr;
	dev_dbg_list(m->dev, "%s():\n", __func__);
}

static const struct file_operations tmpa9xx_cmsi_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = tmpa9xx_cmsi_ioctl,
	.open = tmpa9xx_cmsi_open,
	.release = tmpa9xx_cmsi_close,
	.poll = tmpa9xx_cmsi_poll,
};

static struct miscdevice tmpa9xx_cmsi_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cmsi",
	.fops = &tmpa9xx_cmsi_fops,
};

static int cmsi_i2c_probe(struct i2c_client *i2c_client, const struct i2c_device_id *iid)
{
	struct platform_device *pdev = i2c_client->dev.platform_data;
	struct tmpa9xx_cmsi_priv *m;
	struct resource *res;
	int ret;

	if (tmpa9xx_cmsi_miscdev.parent)
		return -EBUSY;

	m = kzalloc(sizeof(struct tmpa9xx_cmsi_priv), GFP_KERNEL);
	if (!m) {
		dev_err(&i2c_client->dev, "kzalloc() failed\n");
		return -ENOMEM;
	}

	m->dev = &i2c_client->dev;
	m->i2c_client = i2c_client;
	mutex_init(&m->lock);
	m->is_running = false;
	INIT_LIST_HEAD(&m->todo);
	INIT_LIST_HEAD(&m->done);
	init_waitqueue_head(&m->queue);

	m->irq = platform_get_irq(pdev, 0);
	if (m->irq <= 0) {
		dev_err(m->dev, "platform_get_irq() failed\n");
		kfree(m);
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(m->dev, "platform_get_resource() failed\n");
		kfree(m);
		return -ENXIO;
	}

	ret = request_irq(m->irq, interrupt_handler, IRQF_DISABLED, "tmpa9xx-cmsi", m);
	if (ret) {
		dev_err(m->dev, "request_irq() failed\n");
		kfree(m);
		return -ENXIO;
	}

	m->regs = ioremap(res->start, resource_size(res));
	if (!m->regs) {
		dev_err(m->dev, "ioremap() failed\n");
		free_irq(m->irq, m);
		kfree(m);
		return -ENODEV;
	}

 	m->dma = tmpa9xx_dma_request(dma_handler, err_handler, m);
	if (m->dma < 0) {
		dev_err(&pdev->dev, "tmpa9xx_dma_request() failed\n");
		iounmap(m->regs);
		free_irq(m->irq, m);
		kfree(m);
		return -ENXIO;
	}

	ret = probe_cmsi(m);
	if (ret) {
		dev_err(m->dev, "probe_cmsi() failed\n");
		tmpa9xx_dma_free(m->dma);
		iounmap(m->regs);
		free_irq(m->irq, m);
		kfree(m);
		return -ENXIO;
	}

	g_ma = m;

	ret = misc_register(&tmpa9xx_cmsi_miscdev);
	if (ret) {
		dev_err(m->dev, "misc_register() failed\n");
		tmpa9xx_dma_free(m->dma);
		remove_cmsi(m);
		iounmap(m->regs);
		free_irq(m->irq, m);
		kfree(m);
		return ret;
	}

	i2c_set_clientdata(i2c_client, m);

	dev_info(m->dev, DRIVER_DESC " ready\n");

	tmpa9xx_cmsi_miscdev.parent = m->dev;

	return 0;
}

static int __devexit cmsi_i2c_remove(struct i2c_client *i2c_client)
{
	struct tmpa9xx_cmsi_priv *m = i2c_get_clientdata(i2c_client);

	misc_deregister(&tmpa9xx_cmsi_miscdev);

	remove_cmsi(m);

	tmpa9xx_dma_free(m->dma);

	iounmap(m->regs);

	free_irq(m->irq, m);

	kfree(m);

	i2c_set_clientdata(i2c_client, NULL);

	return 0;
}
static struct i2c_device_id cmsi_id[] = {
	{"ov6620", 0},
	{}, /* mandatory */
};
MODULE_DEVICE_TABLE(i2c, cmsi_id);

static struct i2c_driver cmsi_i2c_driver = {
	.driver = {
		.name = "ov6620",
		.owner = THIS_MODULE,
	},
	.id_table = cmsi_id,
	.probe = cmsi_i2c_probe,
	.remove = cmsi_i2c_remove,
};

static int __init cmsi_init(void)
{
	return i2c_add_driver(&cmsi_i2c_driver);
}
module_init(cmsi_init);

static void __exit cmsi_exit(void)
{
	i2c_del_driver(&cmsi_i2c_driver);
}
module_exit(cmsi_exit);

MODULE_AUTHOR("Michael Hunold <michael@mihu.de>");
MODULE_DESCRIPTION("OV6620/CMSI driver for TMPA900");
MODULE_LICENSE("GPL");
