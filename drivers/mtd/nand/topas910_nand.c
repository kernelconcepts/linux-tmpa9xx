/*
 * drivers/mtd/nand/topas910_nand.c
 *
 *  Copyright (c) 2009 Florian Boor <florian.boor@kernelconcepts.de>
 *  Based on tmpa910/nand.c from U-Boot by Michael Hasselberg
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>

#include <mach/tmpa910_regs.h>

#define DMA_TIMEOUT  0xFFFF
#define ECCBUF_SIZE 256

struct tmpa910_nand_host {
	struct nand_chip	nand_chip;
	struct mtd_info		mtd;
	void __iomem		*io_base;
	struct device		*dev;
    
	unsigned int chip_select;
	unsigned int mlc;
	unsigned int dma;
	unsigned int softecc;
	unsigned int column;
	unsigned int orig_column;
	unsigned int ecc_pos;
	unsigned char *eccbuf;
	unsigned char *buf;
};


void tmpa910_nand_dma_read(unsigned char *buf, unsigned short size)
{

	DMACConfiguration = 0x00000001;	// DMAC Enable ON
	DMACC5SrcAddr=(unsigned int)&NDFDTR;		// Source address
	DMACC5DestAddr=(unsigned int)buf;		// Destination address
	DMACC5Control = 0x88489000+(size/4);
	DMACC5Configuration=0x00009009; //FlowCntrl:peripheral to Memory,ITC=1;DMA=EN

}

void tmpa910_nand_dma_write(unsigned char *buf, unsigned short size)
{

	DMACConfiguration = 0x00000001;	// DMAC Enable ON
	DMACC5SrcAddr=(unsigned int)buf;			// Source address
	DMACC5DestAddr=(unsigned int)&NDFDTR;	// Destination address
	DMACC5Control = 0x84489000+(size/4);
	DMACC5Configuration=0x00008901;//FlowCntrl:Memory to peripheral,ITC=1;DMA=EN

}

static int tmpa910_nand_dev_ready(struct mtd_info *mtd)
{
       return !(NDFMCR0 & NDFMCR0_BUSY);
}

static void tmpa910_nand_set_cmd(unsigned int cmd)
{
	/* Begin command latch cycle */
	NDFMCR0 |= NDFMCR0_CLE |NDFMCR0_WE;
	/* Write out the command to the device. */
	NDFDTR = cmd;
	/* End command latch cycle */
	NDFMCR0 &= ~(NDFMCR0_CLE |NDFMCR0_WE);
}


static void tmpa910_nand_set_addr(unsigned int column, unsigned int page_addr,
				unsigned int buswidth_16, unsigned int third_addr)
{
	if (column != -1 || page_addr != -1) {
		NDFMCR0 |= NDFMCR0_ALE |NDFMCR0_WE;

		/* Serially input address */
		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
			if (buswidth_16)
				column >>= 1;
			NDFDTR = column & 0xff;
			NDFDTR = column >> 8;
		}
		if (page_addr != -1) {
			NDFDTR = (unsigned char) (page_addr & 0xff);
			NDFDTR = (unsigned char) ((page_addr >> 8) & 0xff);
			/* One more address cycle for devices > 128MiB */
			if (third_addr)
				NDFDTR = (unsigned char) ((page_addr >> 16) & 0xff);
		}
		/* Latch in address */
		NDFMCR0 &= ~(NDFMCR0_ALE |NDFMCR0_WE);
	}
}

static int tmpa910_nand_wait_dma_complete(unsigned int timeout)
{
	unsigned int val, count = 0;

	do {
		val = DMACIntTCStatus;
		count++;
	}while (((val & NAND_DMAC_STATUS) != NAND_DMAC_STATUS) && count < timeout);	// NDRB=READY wait

	if (count == timeout && ((val & NAND_DMAC_STATUS) != NAND_DMAC_STATUS))
		return -1;
	val = DMACIntTCClear;
	val |= NAND_DMAC_CLEAR;
	DMACIntTCClear = val;

	return 0;
}

static void  tmpa910_nand_get_hwecc(unsigned char *ecc_code, unsigned int mlc)
{
	unsigned int ecc_val = 0;
	unsigned char *buf = ecc_code;
	
	/*printf("----Enable HWECCL %x, %x, %x\n ", NDFMCR0, val, NDFMCR1);*/
	if (!mlc) {
		ecc_val = NDECCRD1;
		*buf++ = (unsigned char)(ecc_val&0xff);
		*buf++ = (unsigned char)(ecc_val>>8);	
		*buf++ = (unsigned char)(ecc_val>>16);	
		ecc_val =  NDECCRD0;
		*buf++ = (unsigned char)(ecc_val&0xff);
		*buf++ = (unsigned char)(ecc_val>>8);		
		*buf++ = (unsigned char)(ecc_val>>16);	
	}
	else
	{
	//MLC, Reed solomn ECC data
		ecc_val = NDECCRD0;
		*buf++ = (unsigned char)(ecc_val>>8);	
		*buf++ = (unsigned char)(ecc_val&0xff);
		*buf++ = (unsigned char)(ecc_val>>24);	
		*buf++ = (unsigned char)(ecc_val>>16);	

		ecc_val = NDECCRD1;
		*buf++ = (unsigned char)(ecc_val>>8);
		*buf++ = (unsigned char)(ecc_val&0xff);
		*buf++ = (unsigned char)(ecc_val>>24);
		*buf++ = (unsigned char)(ecc_val>>16);	

		ecc_val = NDECCRD2;
		*buf++ = (unsigned char)(ecc_val>>8);
		*buf++ = (unsigned char)(ecc_val&0xff);
	}
}

static void tmpa910_nand_set_ecctype(unsigned int mlc)
{
	if (mlc) {
		NDFMCR1 |= NDFMCR1_ECCS;
	}
	else {
		NDFMCR1 &= ~NDFMCR1_ECCS;
	}
}

static void tmpa910_nand_start_autoload(unsigned int read)
{
	unsigned int val;
	
	val = NDFMCR1;
	if (read) {
		/*Set it to be read */
		val &= ~NDFMCR1_SELAL;
	}
	else
		val |= NDFMCR1_SELAL;
	/* start autoload */
	val |= NDFMCR1_ALS;
	NDFMCR1 = val;
}

static void tmpa910_nand_start_ecc(unsigned int read)
{
	if (read)
		NDFMCR0 |= NDFMCR0_ECCE |  NDFMCR0_ECCRST;
	else
		NDFMCR0 |= NDFMCR0_ECCE |  NDFMCR0_ECCRST | NDFMCR0_RSEDN;
}

static void tmpa910_nand_set_rw_mode(unsigned int read)
{
	if (read)
		NDFMCR0 &= ~NDFMCR0_WE;
	else
		NDFMCR0 |= NDFMCR0_WE;
}

static void tmpa910_nand_calculate_softecc(const unsigned char * dat, unsigned char *ecc_code)
{
}

static void tmpa910_nand_command (struct mtd_info *mtd, unsigned command, int column, int page_addr)
{
	register struct nand_chip *this = mtd->priv;
	struct tmpa910_nand_host * priv= (struct tmpa910_nand_host *)this->priv;
	unsigned char * buf, *eccbuf;
	unsigned int buswidth_16 = 0, third_addr = 0;

	unsigned int pagelen = mtd->writesize;
// 		+ ((mtd->opts->writeoob != 0) ? mtd->oobsize : 0);
    
	if (pagelen > 512) {
		/* Emulate NAND_CMD_READOOB */
		if (command == NAND_CMD_READOOB) {
			column += pagelen;
			command = NAND_CMD_READ0;
		}
		if (this->chipsize > (128 << 20))
			third_addr = 1;
	}
	else {
		if (this->chipsize > (32 << 20))
			third_addr = 1;
	}
	if (this->options & NAND_BUSWIDTH_16)
		buswidth_16 = 1;
	priv->column = priv->orig_column = column;
	if (this->ecc.size) {
		if (priv->mlc)
			priv->ecc_pos = (column / this->ecc.size) * 10;
		else
			priv->ecc_pos = (column / this->ecc.size) * 6;
	}
	
	if (command == NAND_CMD_READ0) {
		column = column & ~(this->ecc.size -1);

		if (priv->dma && !priv->softecc) {
			if (column < pagelen) {
				while (column < pagelen) {
					tmpa910_nand_set_ecctype(priv->mlc);
					buf = priv->buf + column;
					if (priv->mlc)
						eccbuf = priv->eccbuf + (column / this->ecc.size) * 10;
					else
						eccbuf = priv->eccbuf + (column / this->ecc.size) * 6;
					tmpa910_nand_set_cmd(command);
					tmpa910_nand_dma_read(buf, this->ecc.size);
					tmpa910_nand_start_autoload(1);
					tmpa910_nand_set_addr(column, page_addr, buswidth_16, third_addr);
					/* Large page NAND */
					if (pagelen > 512)
						tmpa910_nand_set_cmd(NAND_CMD_READSTART);
					tmpa910_nand_start_ecc(1);
					tmpa910_nand_set_rw_mode(1);
					if ( tmpa910_nand_wait_dma_complete(DMA_TIMEOUT)) {
						printk(KERN_ERR "read page :0x%x, column:0x%x time out\n", page_addr, column);
						return;
					}
				
					while(!tmpa910_nand_dev_ready(mtd));

					tmpa910_nand_get_hwecc(eccbuf, priv->mlc);
					if (priv->mlc)
						eccbuf += 6;
					else
						eccbuf += 10;
					column += this->ecc.size;
				}
				//buf += this->eccsize;	
				/*Should not be > this->oobblock */
				if (column >= pagelen) {
					buf = priv->buf + column;
					while (column < (pagelen + mtd->oobsize)) {
						*buf = NDFDTR;
						buf++;
						column++;
					}
				}
			}
			else {
				tmpa910_nand_set_cmd(command);
				tmpa910_nand_set_addr(column, page_addr, buswidth_16, third_addr);
				if (pagelen > 512)
					tmpa910_nand_set_cmd(NAND_CMD_READSTART);
				tmpa910_nand_set_rw_mode(1);
				while(!tmpa910_nand_dev_ready(mtd));	
			}
		}
		else {	/* NO DMA || soft ECC*/
			tmpa910_nand_set_cmd(command);
			if (priv->dma)
				tmpa910_nand_dma_read(priv->buf + column, pagelen + mtd->oobsize - column);
			tmpa910_nand_set_addr(column, page_addr, buswidth_16, third_addr);
			/* Large page NAND */
			if (pagelen > 512)
				tmpa910_nand_set_cmd(NAND_CMD_READSTART);

			if (priv->dma && tmpa910_nand_wait_dma_complete(DMA_TIMEOUT)) {
				printk(KERN_ERR "read page :0x%x, column:0x%x time out\n", page_addr, column);
				return;
			}
			/*HWECC  start will be set at enable_ecc function */
			
			while(!tmpa910_nand_dev_ready(mtd));
		}
	}
	else if (command == NAND_CMD_SEQIN) {	/* For Write */
		column = column & ~(this->ecc.size -1);
		//if (priv->dma && column < this->oobblock) {
		/*if (!priv->softecc) 
			tmpa910_nand_set_ecctype(priv->mlc);			*/

		tmpa910_nand_set_cmd(command);
		tmpa910_nand_set_addr(column, page_addr, buswidth_16, third_addr);
	}
	else if (command == NAND_CMD_PAGEPROG || command == NAND_CMD_CACHEDPROG) {
		if (priv->dma && priv->softecc) {
			/*nand_base.c can make sure that priv->column is aligned with ecc_step */
			tmpa910_nand_dma_write(priv->buf + priv->orig_column, pagelen + mtd->oobsize -  priv->orig_column);
			if ( tmpa910_nand_wait_dma_complete(DMA_TIMEOUT)) {
				printk(KERN_ERR "Write page :, column:0x%x time out\n", priv->column);
				return;
			}
			
			while(!tmpa910_nand_dev_ready(mtd));
		}
		tmpa910_nand_set_cmd(command);
		tmpa910_nand_set_addr(column, page_addr, buswidth_16, third_addr);
	}
	else {
		tmpa910_nand_set_cmd(command);
		tmpa910_nand_set_addr(column, page_addr, buswidth_16, third_addr);
		if (command == NAND_CMD_RESET)
			while(!tmpa910_nand_dev_ready(mtd));
	}
}


/* Set WP on deselect, write enable on select */
static void tmpa910_nand_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *this = mtd->priv;
	struct tmpa910_nand_host * priv= (struct tmpa910_nand_host *)this->priv;
	unsigned int val;

	priv->chip_select = chip;
	if (chip == 0) {
		val =  NDFMCR0;
		val |= NDFMCR0_CE0;	/*Enable chip0 */
		val &= ~NDFMCR0_CE1;  /* Disable chip1 */
		 NDFMCR0 = val;
	}
	else if (chip == 1) {
		val = NDFMCR0;
		val |= NDFMCR0_CE1;	/*Enable chip0 */
		val &= ~NDFMCR0_CE0;  /* Disable chip1 */
		NDFMCR0 = val;
	}
	else {
		val = NDFMCR0;
		val &= ~(NDFMCR0_CE1 | NDFMCR0_CE0); 
		NDFMCR0 = val;
	}
		
}

#ifdef CFG_NAND_LARGEPAGE
static struct nand_ecclayout tmpa910_nand_oobinfo = {
	//.useecc = MTD_NANDECC_AUTOPLACE,
	.useecc = MTD_NANDECC_AUTOPL_USR, /* MTD_NANDECC_PLACEONLY, */
	.eccbytes = 24,
	.eccpos = {8, 9, 10, 13, 14, 15, 24, 25,
			   26, 29, 30, 31, 40, 41, 42, 45,
			   46, 47, 56, 57, 58, 61, 62, 63},
	.oobfree = { {2, 7}, {16, 21}, {32, 37}, {48,53}}
};
#else
static struct nand_ecclayout tmpa910_nand_oobinfo = {
	.eccbytes = 6,
	.eccpos = {8, 9, 10, 13, 14, 15},
	.oobfree = { {2, 7} }
};
#endif

static void tmpa910_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	unsigned i = 0;
	struct nand_chip *this = mtd->priv;
	struct tmpa910_nand_host * priv= (struct tmpa910_nand_host *)this->priv;
	unsigned int pagelen = mtd->writesize;

	if (len > (pagelen + mtd->oobsize - priv->column))
		len = pagelen + mtd->oobsize- priv->column;
	if (priv->dma && priv->orig_column <  pagelen) {
		memcpy(buf, priv->buf + priv->column, len);
	}
	else {
		while (i < len)
			buf[i++] = NDFDTR;
	}
	priv->column += len;
}

static void tmpa910_nand_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;
	struct tmpa910_nand_host * priv= (struct tmpa910_nand_host *)this->priv;
	unsigned int pagelen = mtd->writesize;

	if (len > (pagelen + mtd->oobsize - priv->column))
		len = pagelen + mtd->oobsize - priv->column;
	
	if (priv->dma && priv->orig_column < pagelen) {
		memcpy(priv->buf + priv->column, buf, len);
		if (!priv->softecc) {
			unsigned char *eccbuf;

			if (priv->mlc)
				eccbuf = priv->eccbuf + (priv->column / this->ecc.size) * 10;
			else
				eccbuf = priv->eccbuf + (priv->column / this->ecc.size) * 6;

			if (priv->column < pagelen) {
				/* we can be sure that len = ecc_step and priv->column is aligned by ecc_step(512 byte) */
				tmpa910_nand_dma_write(priv->buf + priv->column, len);
				tmpa910_nand_start_autoload(0);
				if ( tmpa910_nand_wait_dma_complete(DMA_TIMEOUT)) {
					printk(KERN_ERR "Write page :, column:0x%x time out\n", priv->column);
					return;
				}

				while(!tmpa910_nand_dev_ready(mtd));

				//tmpa910_nand_set_rw_mode(1);
				tmpa910_nand_get_hwecc(eccbuf, priv->mlc);
			}
			else {
				for (i = 0; i < len; i++)
					NDFDTR = buf[i];
			}
		}
	}
	else {
		for (i = 0; i < len; i++)
			NDFDTR = buf[i];
	}
	priv->column += len;
}

static void tmpa910_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct nand_chip *this = mtd->priv;
       struct tmpa910_nand_host * priv= (struct tmpa910_nand_host *)this->priv;

	if (mode == NAND_ECC_READ) {
		/* No DMA || soft ECC*/
		if (!priv->dma || priv->softecc) {
			/* Disable WE */
			tmpa910_nand_set_rw_mode(1);
		
			if (!priv->softecc) {
				tmpa910_nand_set_ecctype(priv->mlc);
				tmpa910_nand_start_ecc(mode == NAND_ECC_READ);
			}
		}
	}
	else if (mode == NAND_ECC_WRITE) {
		tmpa910_nand_set_rw_mode(0);
		if (!priv->softecc) {
			tmpa910_nand_set_ecctype(priv->mlc);
			tmpa910_nand_start_ecc(mode == NAND_ECC_READ);
		}
	}
	//printf("Enable HWECCL %x, %x\n ", NDFMCR0, NDFMCR1);
}

static int tmpa910_nand_calculate_ecc(struct mtd_info *mtd, const unsigned char *dat, unsigned char *ecc_code)
{
	struct nand_chip *this = mtd->priv;	
	struct tmpa910_nand_host * priv= (struct tmpa910_nand_host *)this->priv;
	int steps;

	/* No DMA && No read OOB */
	if (!priv->dma || priv->softecc) {
		if (priv->softecc) {
			steps = this->ecc.size / ECCBUF_SIZE;
			while (steps--) {
				tmpa910_nand_calculate_softecc(dat, ecc_code);
				dat += ECCBUF_SIZE;
				ecc_code += 3;
			}
		}
		else {
			tmpa910_nand_get_hwecc(ecc_code, priv->mlc);
		}
	}
	else {
		memcpy(ecc_code, priv->eccbuf + priv->ecc_pos, 6);
		priv->ecc_pos += 6;
	}

return 0; 
}

#define BIT7        0x80
#define BIT6        0x40
#define BIT5        0x20
#define BIT4        0x10
#define BIT3        0x08
#define BIT2        0x04
#define BIT1        0x02
#define BIT0        0x01
#define BIT1BIT0    0x03
#define BIT23       0x00800000L
#define MASK_CPS    0x3f
#define CORRECTABLE 0x00555554L

static int tmpa910_nand_part_correctdata(unsigned char *data, unsigned char *eccdata, unsigned char ecc1, unsigned char ecc2, unsigned char ecc3)
{
	/*FIXME should be U31, U16 and U8 */
	unsigned int l; /* Working to check d */
	unsigned int d; /* Result of comparison */
	unsigned short i; /* For counting */
	unsigned char d1,d2,d3; /* Result of comparison */
	unsigned char a; /* Working for add */
	unsigned char add; /* Byte address of cor. DATA */
	unsigned char b; /* Working for bit */
	unsigned char bit; /* Bit address of cor. DATA */

	d1=ecc1^eccdata[1]; d2=ecc2^eccdata[0]; /* Compare LP's */
	d3=ecc3^eccdata[2]; /* Comapre CP's */
	d=((unsigned int)d1<<16) /* Result of comparison */
	    +((unsigned int)d2<<8)
	        +(unsigned int)d3;
	if (d==0) 
		return(0); /* If No error, return */
    
	if (((d^(d>>1))&CORRECTABLE)==CORRECTABLE) { /* If correctable */
		l=BIT23;
		add=0; /* Clear parameter */
		a=BIT7;
		for(i=0; i<8; ++i) { /* Checking 8 bit */
			if ((d&l)!=0) 
				add|=a; /* Make byte address from LP's */
			l>>=2; a>>=1; /* Right Shift */
		}
		bit=0; /* Clear parameter */
		b=BIT2;
		for (i = 0; i < 3; ++i) { /* Checking 3 bit */
		    if ((d&l)!=0) 
				bit|=b; /* Make bit address from CP's */
		    l>>=2; b>>=1; /* Right shift */
		}
		b=BIT0;
		data[add]^=(b<<bit); /* Put corrected data */
		return(1);
	}
	i=0; /* Clear count */
	d&=0x00ffffffL; /* Masking */
	while (d)	{ /* If d=0 finish counting */
		if (d&BIT0) 
			++i; /* Count number of 1 bit */
		d>>=1; /* Right shift */
	}
	if (i==1)	{ /* If ECC error */
		eccdata[1]=ecc1; 
		eccdata[0]=ecc2; /* Put right ECC code */
		eccdata[2]=ecc3;
		return(2);
	}
	return(3); /* Uncorrectable error */
}

static int tmpa910_nand_correct_data(struct mtd_info *mtd, u_char *data, u_char *read_ecc, u_char *calc_ecc)
{
	struct nand_chip *this = mtd->priv;
       //struct tmpa910_nand_host * priv= (struct tmpa910_nand_host *)this->priv;
	unsigned int size;
	unsigned char ecc1, ecc2, ecc3; /* FIXME should define as U8 */
	unsigned int ret, ret_tmp;

	size = this->ecc.size;

	ret_tmp = 0;
	while (size > 0)
	{
		ecc2 = *calc_ecc++;
		ecc1 = *calc_ecc++;
		ecc3 = *calc_ecc++;
		ret = tmpa910_nand_part_correctdata(data, read_ecc, ecc1, ecc2, ecc3);
		if(ret > ret_tmp)
			ret_tmp = ret;
			
		size -= ECCBUF_SIZE;
		data += ECCBUF_SIZE;
		read_ecc += 3;	
	}
	return (ret_tmp != 0 && ret_tmp != 1)? -1 : 0;
}



static void tmpa910_nand_set_timing(struct tmpa910_nand_host *priv)
{
	NDFMCR2 = 0x2222;
}


#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL };
#endif

/*
 * Probe for the NAND device.
 */
static int __init topas910_nand_probe(struct platform_device *pdev)
{
	struct tmpa910_nand_host *host;
	struct mtd_info *mtd;
	struct nand_chip *nand_chip;
	int res;

#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *partitions = NULL;
	int num_partitions = 0;
#endif
    
	/* Allocate memory for the device structure (and zero it) */
	host = kzalloc(sizeof(struct tmpa910_nand_host), GFP_KERNEL);
	if (!host) {
		printk(KERN_ERR
		       "topas910_nand: failed to allocate device structure.\n");
		return -ENOMEM;
	}

/*	host->io_base = ioremap(NANDF_BASE, SZ_1K);
	if (host->io_base == NULL) {
		printk(KERN_ERR "topas910_nand: ioremap failed\n");
		kfree(host);
		return -EIO;
	}
*/
	mtd = &host->mtd;
	nand_chip = &host->nand_chip;
	host->dev = &pdev->dev;

	nand_chip->priv = host;		/* link the private data structures */
	mtd->priv = nand_chip;
	mtd->name = "topas910_nand";
	mtd->owner = THIS_MODULE;
	mtd->dev.parent = &pdev->dev;

	nand_chip->IO_ADDR_R   = (void  __iomem *)(&NDFDTR);
	nand_chip->IO_ADDR_W   = (void  __iomem *)(&NDFDTR);

	/* FIXME: give virtual address for the controller register base*/
	/* priv->baseaddr = ...*/
	nand_chip->chip_delay  = 0;
	nand_chip->select_chip = tmpa910_nand_select_chip;

	/* The controller will generate 6 bytes ecc for 512 bytes data*/
//later	nand->eccmode = NAND_ECC_HW6_512;
	nand_chip->ecc.mode     = NAND_ECC_SOFT;

	nand_chip->ecc.layout    = &tmpa910_nand_oobinfo;
//later	nand_chip->calculate_ecc = tmpa910_nand_calculate_ecc;
//later	nand_chip->correct_data  = tmpa910_nand_correct_data;
//later	nand_chip->enable_hwecc  = tmpa910_nand_enable_hwecc;
	/* Set address of hardware control function */
	nand_chip->cmdfunc = tmpa910_nand_command;
	nand_chip->dev_ready = tmpa910_nand_dev_ready;
	nand_chip->read_buf = tmpa910_nand_read_buf;
	nand_chip->write_buf = tmpa910_nand_write_buf;
	nand_chip->ecc.size = 0x0;
    
	host->dma = 1;
	host->softecc = 1;
	host->buf = kzalloc(4096, GFP_KERNEL | GFP_DMA);
	host->buf = (unsigned char *)((unsigned int)(host->buf + 31) & ~31);
	host->eccbuf = host->buf + 2112;

	NDFMCR0 =0x0;
	NDFMCR1 = 0;
	tmpa910_nand_set_timing(nand_chip->priv);

	dev_set_drvdata(&pdev->dev, host);

	/* first scan to find the device and get the page size */
	if (nand_scan_ident(mtd, 1)) {
		res = -ENXIO;
		goto out;
	}

	/* second phase scan */
	if (nand_scan_tail(mtd)) {
		res = -ENXIO;
		goto out;
	}

#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
	num_partitions = parse_mtd_partitions(mtd, part_probes,
					      &partitions, 0);
	if (num_partitions < 0) {
		res = num_partitions;
		goto release;
	}
#endif

#ifdef CONFIG_MTD_OF_PARTS
	if (num_partitions == 0) {
		num_partitions = of_mtd_parse_partitions(&ofdev->dev,
							 ofdev->node,
							 &partitions);
		if (num_partitions < 0) {
			res = num_partitions;
			goto release;
		}
	}
#endif
	if (partitions && (num_partitions > 0))
		res = add_mtd_partitions(mtd, partitions, num_partitions);
	else
#endif
		res = add_mtd_device(mtd);

	if (!res)
		return res;

#ifdef CONFIG_MTD_PARTITIONS
release:
#endif
	nand_release(mtd);

out:
	dev_set_drvdata(&pdev->dev, NULL);
	iounmap(host->io_base);
	kfree(host);
	return res;
}

/*
 * Remove a NAND device.
 */
static int __devexit topas910_nand_remove(struct platform_device *pdev)
{
	struct tmpa910_nand_host *host = dev_get_drvdata(&pdev->dev);
	struct mtd_info *mtd = &host->mtd;

	nand_release(mtd);

	dev_set_drvdata(&pdev->dev, NULL);
	iounmap(host->io_base);
	kfree(host);

	return 0;
}

static struct platform_driver topas910_nand_driver = {
	.driver = {
		.name = "topas910_nand",
		.owner		= THIS_MODULE,
	},
	.probe		= topas910_nand_probe,
	.remove		= __devexit_p(topas910_nand_remove),
};

static int __init topas910_nand_init(void)
{
	return platform_driver_register(&topas910_nand_driver);
}

static void __exit topas910_nand_exit(void)
{
	platform_driver_unregister(&topas910_nand_driver);
}

module_init(topas910_nand_init);
module_exit(topas910_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florian Boor");
MODULE_DESCRIPTION("NAND driver for Topas910 board");
