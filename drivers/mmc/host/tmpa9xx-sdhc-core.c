/*
 * drivers/mmc/host/tmpa9xx-sdhc-core.c
 *
 * Copyright (C) 
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
 * 
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/platform_device.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>

#include <mach/dma.h>
#include <mach/regs.h>

#define SDHC_SD_WRITE DMAC_PERIPHERAL_SDHC_SD_WRITE
#define SDHC_SD_READ  DMAC_PERIPHERAL_SDHC_SD_READ

#include "tmpa9xx-sdhc.h"

//#define SDHC_DEBUG

#ifdef SDHC_DEBUG
#define sdprintk(...)	do {printk("%s(): ", __func__); printk(__VA_ARGS__);} while(0)
#else
#define sdprintk(...)
#endif

#define DRIVER_NAME	"tmpa9xx-sdhc"

struct sdhc_host {

	struct mmc_host *mmc;
	struct mmc_request *mrq;

	int irq;
	int dma_channel;

	uint32_t fifo;

	dma_addr_t sg_dma;
	struct tmpa9xx_dma_desc *sg_cpu;

	struct work_struct sdhc_work;
	struct work_struct dma_work;
	struct work_struct init_work;

	int isd0wp_wp_active;
};

struct sdhc_host *h;

static int prepare_data_dma(struct sdhc_host *host)
{
	struct mmc_data *data = host->mrq->data;
	struct scatterlist *sg;
	struct tmpa9xx_dma_desc *lli;
	int xfer_iswrite;
	u32 ctrl_reg;
	u32 conf_reg;
	int dma_dir;
	int i;

	BUG_ON(!host->mrq);
	BUG_ON(!host->mrq->data);

	xfer_iswrite = host->mrq->data->flags & MMC_DATA_WRITE;

	sdprintk("\n");

	if (xfer_iswrite) {
		dma_dir = DMA_TO_DEVICE;
		conf_reg = ((SDHC_SD_WRITE << DMAC_CH_CONF_DEST_OFS)
			    | DMAC_CH_CONF_FLOWCNTRL_MEM2PER
			    | DMAC_CH_CONF_ITC | DMAC_CH_CONF_IE);
		ctrl_reg = 0
		    | DMAC_CH_CTRL_SWIDTH_HWORD
		    | DMAC_CH_CTRL_DWIDTH_HWORD
		    | DMAC_CH_CTRL_SBSIZE_256B
		    | DMAC_CH_CTRL_DBSIZE_256B
		    | DMAC_CH_CTRL_PROT3
		    | DMAC_CH_CTRL_PROT2 | DMAC_CH_CTRL_S | DMAC_CH_CTRL_SI;

	} else {
		dma_dir = DMA_FROM_DEVICE;
		conf_reg = ((SDHC_SD_READ << DMAC_CH_CONF_SRC_OFS)
			    | DMAC_CH_CONF_FLOWCNTRL_PER2MEM
			    | DMAC_CH_CONF_ITC | DMAC_CH_CONF_IE);
		ctrl_reg = 0
		    | DMAC_CH_CTRL_SWIDTH_HWORD
		    | DMAC_CH_CTRL_DWIDTH_HWORD
		    | DMAC_CH_CTRL_SBSIZE_256B
		    | DMAC_CH_CTRL_DBSIZE_256B
		    | DMAC_CH_CTRL_PROT3
		    | DMAC_CH_CTRL_PROT2 | DMAC_CH_CTRL_S | DMAC_CH_CTRL_DI;
	}

	dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len, dma_dir);

	for_each_sg(data->sg, sg, data->sg_len, i) {
		lli = &host->sg_cpu[i];

		sdprintk("%d: %d\n", i, sg_dma_len(sg));

		lli->control = sg_dma_len(sg) / 2 | ctrl_reg;
		lli->src_addr = xfer_iswrite ? (uint32_t) sg_dma_address(sg) : host->fifo;
		lli->dest_addr = xfer_iswrite ? host->fifo : (uint32_t) sg_dma_address(sg);
		lli->dma_lli = host->sg_dma + (i + 1) * sizeof(struct tmpa9xx_dma_desc);
	}

	host->sg_cpu[i - 1].dma_lli = 0;
	host->sg_cpu[i - 1].control |= DMAC_CH_CTRL_I;

	lli = &host->sg_cpu[0];
	DMA_DEST_ADDR(host->dma_channel) = lli->dest_addr;
	DMA_SRC_ADDR(host->dma_channel) = lli->src_addr;
	DMA_LLI(host->dma_channel) = lli->dma_lli;
	DMA_CONTROL(host->dma_channel) = lli->control;
	DMA_CONFIG(host->dma_channel) = conf_reg;

	return 0;
}

static void start_dma_transfer(struct sdhc_host *host)
{
	DMA_CONFIG(host->dma_channel) |= (1 << 0);
}

static void stop_dma_transfer(struct sdhc_host *host)
{
	DMA_CONFIG(host->dma_channel) &= ~(1 << 0);
}

static void cmd_done_irq(struct sdhc_host *host)
{
	struct mmc_command *cmd = host->mrq->cmd;
	int res_type;

	sdprintk("\n");

	BUG_ON(!cmd);

	res_type = mmc_resp_type(cmd);

	if (res_type & MMC_RSP_PRESENT) {

		if (res_type & MMC_RSP_136) {
			uint32_t *my_res_32b = (uint32_t *) cmd->resp;
			struct sdhc_response_get_8b_struct s;

			sdhc_response_get_8b(&s);
			memcpy(cmd->resp, &s.b[0], sizeof(s.b));

			/* Ouch ! workaround for UNSTUFF_BIT into the MMC stack */
			my_res_32b[0] = cpu_to_be32(my_res_32b[0]);
			my_res_32b[1] = cpu_to_be32(my_res_32b[1]);
			my_res_32b[2] = cpu_to_be32(my_res_32b[2]);
			my_res_32b[3] = cpu_to_be32(my_res_32b[3]);
		} else {
			struct sdhc_response_get_16b_struct s;
			uint16_t *my_res_16b = (uint16_t *) cmd->resp;
			sdhc_response_get_16b(&s);
			*my_res_16b++ = s.b[0];
			*my_res_16b++ = s.b[1];
		}
	}

	if (host->mrq && host->mrq->data && !cmd->error) {
		sdhc_wait_data_ready();
		start_dma_transfer(host);
	} else {
		schedule_work(&host->dma_work);
	}
}

static void irq_dma_work(struct work_struct *work)
{
	struct sdhc_host *host = container_of(work, struct sdhc_host, dma_work);
	struct mmc_request *mrq;

	sdprintk("\n");

	BUG_ON(!host->mrq);

	if (host->mrq->data)
		host->mrq->data->bytes_xfered = host->mrq->data->blocks * host->mrq->data->blksz;

	if (host->mrq->stop)
		sdhc_mrq_stop();

	mrq = host->mrq;
	host->mrq = NULL;

	mmc_request_done(host->mmc, mrq);
}

static void card_remove(struct sdhc_host *host)
{
	sdprintk("\n");

	/* safety check */
	if (!sdhc_is_card_present())
		return;

	if(!host->mrq)
		return;

	if (host->mrq->cmd)
		host->mrq->cmd->error = -ENOMEDIUM;

	if (host->mrq->data)
		host->mrq->data->error = -ENOMEDIUM;

	schedule_work(&host->dma_work);
}

static void dma_interrupt(int channel_idx, void *dev)
{
	struct sdhc_host *host = dev;
	struct mmc_data *data = host->mrq->data;

	stop_dma_transfer(host);

	dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len, (data->flags & MMC_DATA_WRITE) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	schedule_work(&host->dma_work);
}

static void irq_sdhc_work(struct work_struct *work)
{
	struct sdhc_host *host = container_of(work, struct sdhc_host, sdhc_work);
	int ret;

	ret = sdhc_irq_handler();

	switch(ret) {
	case SDHC_CARD_INSERT:
		mmc_detect_change(host->mmc, msecs_to_jiffies(250));
		break;
	case SDHC_CARD_REMOVE:
		card_remove(host);
		mmc_detect_change(host->mmc, msecs_to_jiffies(250));
		break;
	case SDHC_REQ_TIMEOUT:
		if (host->mrq->data)
			host->mrq->data->error = ETIMEDOUT;

		if (host->mrq->cmd)
			host->mrq->cmd->error = ETIMEDOUT;

		schedule_work(&host->dma_work);
		break;
	case SDHC_REQ_ERROR:
		if (host->mrq->data)
			host->mrq->data->error = EILSEQ;

		if (host->mrq->cmd)
			host->mrq->cmd->error = EILSEQ;

		schedule_work(&host->dma_work);
		break;
	case SDHC_REQ_COMPLETED:
		cmd_done_irq(host);
		break;
	case 0:
		break;
	default:
		printk(KERN_ERR "unhandled interrupt\n");
		break;
	}
	
	enable_irq(host->irq);
}

irqreturn_t interrupt_handler(int irq, void *devid)
{
	struct sdhc_host *host = devid;
	host->irq = irq;
	disable_irq_nosync(host->irq);
	schedule_work(&host->sdhc_work);
	return IRQ_HANDLED;
}

static void handle_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sdhc_host *host = mmc_priv(mmc);
	struct sdhc_start_cmd_execution_struct s;
	int ret;

	int is_data = 0;
	int is_read = 0;
	int is_multiblock = 0;

	BUG_ON(host->mrq);

	BUG_ON(!mrq);
	BUG_ON(!mrq->cmd);

	sdprintk("opcode %d\n", mrq->cmd->opcode);

	host->mrq = mrq;

	if (!sdhc_is_card_present()) {
		host->mrq->cmd->error = -ENOMEDIUM;
		schedule_work(&host->dma_work);
		return;
	}

	if (mrq->data) {
		struct sdhc_prepare_data_transfer_struct s;

		if (mrq->data->flags & MMC_DATA_STREAM) {
			printk(KERN_ERR "stream not supported\n");
			ret = -1;
			goto fail;
		}

		sdprintk("opcode %2d, arg 0x%08x, is_write %d, blksz %3d, blocks %2d\n",
			 host->mrq->cmd->opcode, host->mrq->cmd->arg, 
		         !!(mrq->data->flags & MMC_DATA_WRITE), mrq->data->blksz,
			 mrq->data->blocks);

		is_data = 1;

		if (!(mrq->data->flags & MMC_DATA_WRITE))
			is_read = 1;

		if (mrq->data->blocks > 1)
			is_multiblock = 1;

		s.seccnt = mrq->data->blocks;
		s.size = mrq->data->blksz;
		sdhc_prepare_data_transfer(&s);

		ret = prepare_data_dma(host);

		if (ret < 0)
			goto fail;
	}
	else
	{
		sdprintk("opcode %2d, arg 0x%08x\n",
			 host->mrq->cmd->opcode, host->mrq->cmd->arg);
	}

	s.opcode = host->mrq->cmd->opcode;
	s.arg = host->mrq->cmd->arg;
	s.res_type = mmc_resp_type(host->mrq->cmd);
	s.is_data = is_data;
	s.is_read = is_read;
	s.is_multiblock = is_multiblock;

	sdhc_start_cmd_execution(&s);
	return;

fail:
	mrq->cmd->error = ret;
	mrq->cmd->data->error = ret;
	host->mrq = NULL;

	mmc_request_done(mmc, mrq);
}

static void set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sdhc_setup_ios_struct s;
	int is_width_4 = 0;

	/* ios->power_mode is not taken into account */

	if (ios->bus_width == MMC_BUS_WIDTH_4)
		is_width_4 = 1;

	s.clock_mini = mmc->f_min;
	s.sd_clock = ios->clock;
	s.is_width_4 = is_width_4;

	sdhc_setup_ios(&s);
}

static void enable_sdio_irq(struct mmc_host *host, int enable)
{
	printk(KERN_DEBUG "enable_sdio_irq not supported enable=%d\n", enable);
}

static int get_ro(struct mmc_host *mmc)
{
	struct sdhc_host *host = mmc_priv(mmc);
	int ret;
	int ret2;

	ret2 = sdhc_is_readonly();
	ret = host->isd0wp_wp_active == ret2;

	/* we have to return 1 if read only, read only = write protected */
	return ret;
}

static const struct mmc_host_ops sdhc_ops = {
	.request = handle_request,
	.set_ios = set_ios,
	.enable_sdio_irq = enable_sdio_irq,
	.get_ro = get_ro,
};

static void init_work(struct work_struct *work)
{
	struct sdhc_host *host = container_of(work, struct sdhc_host, init_work);
	struct sdhc_connect_struct s;
	struct mmc_host *mmc;
	int ret;

	mmc = host->mmc;

	ret = sdhc_connect(&s);
	BUG_ON(ret);

	host->fifo = s.fifo;

	host->isd0wp_wp_active = s.data.isd0wp_wp_active;
	mmc->ocr_avail = 0;
	if (s.data.vdd_32_33)
		mmc->ocr_avail |= MMC_VDD_32_33;
	if (s.data.vdd_33_34)
		mmc->ocr_avail |= MMC_VDD_33_34;
	if (s.data.has_4b_data)
		mmc->caps |= MMC_CAP_4_BIT_DATA;

	sdhc_reset();

	mmc_add_host(mmc);
}

void sdhc_functions_connect(void)
{
	struct sdhc_host *host = h;
	schedule_work(&host->init_work);
}

static int sdhc_probe(struct platform_device *pdev)
{
	struct sdhc_host *host;
	struct mmc_host *mmc;
	struct device *dev;
	struct resource	*res;
	int ret;

	dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		printk(KERN_ERR "platform_get_resource() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	mmc = mmc_alloc_host(sizeof(struct sdhc_host), dev);
	if (!mmc) {
		printk(KERN_ERR "mmc_alloc_host() failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	/* fixme: get these from somewhere else */
#define CPU_CLOCK  		192000000
#define SDHC_CLOCK 		(CPU_CLOCK/2)
#define MAX_CLOCK_DIVIDER 	512
#define MIN_CLOCK_DIVIDER 	2

	mmc->ops = &sdhc_ops;
	mmc->f_min = SDHC_CLOCK / MAX_CLOCK_DIVIDER;
	mmc->f_max = SDHC_CLOCK / MIN_CLOCK_DIVIDER;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = 64;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = 4096;
	mmc->max_hw_segs = 128;
	mmc->max_phys_segs = 128;
	mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;

	host = mmc_priv(mmc);
	host->mmc = mmc;

	h = host;

	INIT_WORK(&host->sdhc_work, irq_sdhc_work);
	INIT_WORK(&host->dma_work,  irq_dma_work);
	INIT_WORK(&host->init_work, init_work);

	ret = sdhc_init();
	if (ret) {
		printk(KERN_ERR "sdhc_init() failed\n");
		ret = -ENOMEM;
		goto err1;
	}

	host->dma_channel = tmpa9xx_dma_request("TMPA9xx SDHC", 0, dma_interrupt, NULL, host);
	if (host->dma_channel < 0) {
		printk(KERN_ERR "tmpa9xx_dma_request() failed\n");
		ret = -ENOMEM;
		goto err2;
	}

	host->sg_cpu = dma_alloc_coherent(dev, PAGE_SIZE, &host->sg_dma, GFP_KERNEL);
	if (!host->sg_cpu) {
		printk(KERN_ERR "dma_alloc_coherent() failed\n");
		ret = -ENOMEM;
		goto err3;
	}

	ret = request_irq(res->start, interrupt_handler, 0, DRIVER_NAME, host);
	if (ret) {
		printk(KERN_ERR "request_irq() failed\n");
		ret = -ENOMEM;
		goto err4;
	}

	platform_set_drvdata(pdev, mmc);

	return 0;

err4:
	dma_free_coherent(dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);
err3:
	tmpa9xx_dma_free(host->dma_channel);
err2:
	sdhc_deinit();
err1:
	mmc_free_host(mmc);
err0:
	return ret;
}

static int sdhc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct resource	*res;
	struct sdhc_host *host;

	sdprintk("\n");

	BUG_ON(!mmc);

	host = mmc_priv(mmc);
	BUG_ON(!host);

	mmc_remove_host(mmc);

	flush_work(&host->sdhc_work);

	sdhc_deinit();

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	BUG_ON(!res);
	free_irq(res->start, host);

	tmpa9xx_dma_free(host->dma_channel);
	dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);

	mmc_free_host(mmc);

	return 0;
}

static struct platform_driver sdhc_driver = {
	.probe = sdhc_probe,
	.remove = sdhc_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init sdhc_m_init(void)
{
	return platform_driver_register(&sdhc_driver);
}

static void __exit sdhc_m_exit(void)
{
	platform_driver_unregister(&sdhc_driver);
}

module_init(sdhc_m_init);
module_exit(sdhc_m_exit);

MODULE_DESCRIPTION("TMPA9xx SDHC Driver");
MODULE_LICENSE("GPL");
