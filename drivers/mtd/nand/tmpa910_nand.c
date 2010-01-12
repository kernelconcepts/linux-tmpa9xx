/*
 *  drivers/mtd/nand/tmpa910_nand.c 
 *
 * Copyright (C) 2008 bplan GmbH. All rights reserved. (?)
 * Copyright (C) 2009 Florian Boor <florian.boor@kernelconcepts.de>
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
 * TMPA 910 NAND controller driver, unclear origin (BPlan or Toshiba likely)
 */
 
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#include <asm/io.h>
#include <mach/dma.h>
#include <mach/tmpa910_regs.h>

#define ECCBUF_SIZE 256
#define WAIT_TIMEOUT  0xFFFF
#define DMA_TIMEOUT  0xFFFF

struct tmpa910_nand_timing {
	unsigned int splw;
	unsigned int sphw;
	unsigned int splr;
	unsigned int sphr;
};

struct tmpa910_nand_private {
	struct nand_chip chip;
	unsigned int chip_select;
	unsigned int mlc;
	unsigned int dma;
	unsigned int softecc;
	unsigned int column;
	unsigned int orig_column;
	unsigned int ecc_pos;
	unsigned char eccbuf[64];
	unsigned char *buf;
	unsigned int phy_buf;
	unsigned int dma_ch;
	struct completion dma_completion;
	struct tmpa910_nand_timing timing;
};

struct tmpa910_nand_private tmpa910_nand = {
	.chip_select = 0,
	.mlc = 0,
	.timing =
	{
		.splw = 0x2,
		.sphw = 0x2,
		.splr = 0x2,
		.sphr = 0x2,
	},
};


#ifdef CONFIG_MTD_PARTITIONS
/*
 * Define static partitions for flash device
 */
static int num_partitions = 3;

static struct mtd_partition mtd_parts_builtin[] = {
	{
		.name		= "bootloader",
		.offset		= 0x00000000,
		.size		= 0x00080000,
	}, {
		.name		= "kernel",
		.offset		= 0x00080000,
		.size		= 0x00200000,
	}, {
		.name		= "filesystem",
		.offset		= 0x00280000,
		.size		= 0x08000000 - 0x00280000,
	}, 
};

static const char *part_probes[] = { "cmdlinepart", NULL };

#endif

void tmpa910_nand_dma_read(struct tmpa910_nand_private *priv, unsigned int phy_addr, unsigned short size)
{
	//tmpa910_dma_enable(priv->dma_ch);
	DMA_SRC_ADDR(priv->dma_ch) = NDFDTR_PHY;
	DMA_DEST_ADDR(priv->dma_ch) = phy_addr;
	DMA_CONTROL(priv->dma_ch) = 0x88489000 + (size/4);
	DMA_CONFIG(priv->dma_ch) = 0x9009; 
}

void tmpa910_nand_dma_write(struct tmpa910_nand_private *priv, unsigned int phy_addr, unsigned short size)
{

	//tmpa910_dma_enable(priv->dma_ch);
	DMA_SRC_ADDR(priv->dma_ch) = phy_addr;		// Source address
	DMA_DEST_ADDR(priv->dma_ch) = NDFDTR_PHY;	// Destination address
	DMA_CONTROL(priv->dma_ch) = 0x84489000 + (size/4);
	DMA_CONFIG(priv->dma_ch)= 0x00008901;//FlowCntrl:Memory to peripheral,ITC=1;DMA=EN
}

static int tmpa910_nand_dev_ready(struct mtd_info *mtd)
{
       return !(NDFMCR0 & NDFMCR0_BUSY);
}

static int tmpa910_nand_als_ready(struct mtd_info *mtd)
{
       return !(NDFMCR1 & 0x100);
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

static int tmpa910_nand_wait_dma_complete(struct tmpa910_nand_private *priv, 
						unsigned int timeout)
{
	int ret;

	ret = wait_for_completion_interruptible_timeout(&priv->dma_completion, timeout);
	tmpa910_dma_disable(priv->dma_ch);

	return !ret;
}

static void  tmpa910_nand_get_hwecc(unsigned char *ecc_code, unsigned int mlc)
{
	unsigned int ecc_val = 0;
	unsigned char *buf = ecc_code;
	
	//printf("----Enable HWECCL %x, %x, %x\n ", NDFMCR0, val, NDFMCR1);
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
	int eccsize = this->ecc.size;
	struct tmpa910_nand_private * priv= (struct tmpa910_nand_private *)this->priv;
	unsigned char * buf, *eccbuf;
	unsigned int buswidth_16 = 0, third_addr = 0;

	/* printk("Send command;%x, at column;%x, page;%x\n", command, column, page_addr); */
	if (mtd->writesize > 512) {
		/* Emulate NAND_CMD_READOOB */
		if (command == NAND_CMD_READOOB) {
			column += mtd->writesize;
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
	if (eccsize) {
		if (priv->mlc)
			priv->ecc_pos = (column / eccsize) * 10;
		else
			priv->ecc_pos = (column / eccsize) * 6;
	}
	
	if (command == NAND_CMD_READ0) {
		column = column & ~(eccsize -1);

		if (priv->dma && !priv->softecc) {
			if (column < mtd->writesize) {
				while (column < mtd->writesize) {
					tmpa910_nand_set_ecctype(priv->mlc);
					buf = priv->buf + column;
					if (priv->mlc)
						eccbuf = priv->eccbuf + (column / eccsize) * 10;
					else
						eccbuf = priv->eccbuf + (column / eccsize) * 6;
					tmpa910_nand_set_cmd(command);
					tmpa910_nand_dma_read(priv, priv->phy_buf + column, eccsize);
					tmpa910_nand_start_autoload(1);
					tmpa910_nand_set_addr(column, page_addr, buswidth_16, third_addr);
					/* Large page NAND */
					if (mtd->writesize > 512)
						tmpa910_nand_set_cmd(NAND_CMD_READSTART);
					tmpa910_nand_start_ecc(1);
					tmpa910_nand_set_rw_mode(1);
					if (tmpa910_nand_wait_dma_complete(priv, DMA_TIMEOUT)) {
						printk("ERROR: read page :0x%x, column:0x%x time out\n", page_addr, column);
						return;
					}
				
					while(!tmpa910_nand_als_ready(mtd));

					tmpa910_nand_get_hwecc(eccbuf, priv->mlc);
					if (priv->mlc)
						eccbuf += 6;
					else
						eccbuf += 10;
					column += eccsize;
				}
				//buf += this->eccsize;	
				/*Should not be > this->oobblock */
				if (column >= mtd->writesize) {
					buf = priv->buf + column;
					while (column < (mtd->writesize + mtd->oobsize)) {
						*buf = NDFDTR;
						buf++;
						column++;
					}
				}
			}
			else {
				tmpa910_nand_set_cmd(command);
				tmpa910_nand_set_addr(column, page_addr, buswidth_16, third_addr);
				if (mtd->writesize > 512)
					tmpa910_nand_set_cmd(NAND_CMD_READSTART);
				tmpa910_nand_set_rw_mode(1);
				while(!tmpa910_nand_dev_ready(mtd));	
				/*buf = priv->buf + column;
				while (column < (mtd->oobblock + mtd->oobsize)) {
					*buf = NDFDTR;
					printf("0x%x,,\t ", *buf);
					buf++;
					column++;
				}*/
			}
		}
		else {	/* NO DMA || soft ECC*/
			tmpa910_nand_set_cmd(command);
			if (priv->dma)
				tmpa910_nand_dma_read(priv, priv->phy_buf + column, mtd->writesize + mtd->oobsize - column);
			tmpa910_nand_set_addr(column, page_addr, buswidth_16, third_addr);
			/* Large page NAND */
			if (mtd->writesize > 512)
				tmpa910_nand_set_cmd(NAND_CMD_READSTART);

			if (priv->dma && tmpa910_nand_wait_dma_complete(priv, DMA_TIMEOUT)) {
				printk("ERROR: read page :0x%x, column:0x%x time out\n", page_addr, column);
				return;
			}
			/*HWECC  start will be set at enable_ecc function */
			
			while(!tmpa910_nand_dev_ready(mtd));
		}
	}
	else if (command == NAND_CMD_SEQIN) {	/* For Write */
		column = column & ~(eccsize -1);
		//if (priv->dma && column < this->oobblock) {
		/*if (!priv->softecc) 
			tmpa910_nand_set_ecctype(priv->mlc);			*/

		tmpa910_nand_set_cmd(command);
		tmpa910_nand_set_addr(column, page_addr, buswidth_16, third_addr);
	}
	else if (command == NAND_CMD_PAGEPROG || command == NAND_CMD_CACHEDPROG) {
		if (priv->dma && priv->softecc) {
			/*nand_base.c can make sure that priv->column is aligned with ecc_step */
			tmpa910_nand_dma_write(priv, priv->phy_buf + priv->orig_column, mtd->writesize + mtd->oobsize -  priv->orig_column);
			if (tmpa910_nand_wait_dma_complete(priv, DMA_TIMEOUT)) {
				printk("ERROR: Write page :, column:0x%x time out\n", priv->column);
				return;
			}
			
			while(!tmpa910_nand_als_ready(mtd));
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
	struct tmpa910_nand_private * priv= (struct tmpa910_nand_private *)this->priv;
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

static struct nand_ecclayout nand_lp_hw_eccoob = {
	//.useecc = MTD_NANDECC_AUTOPLACE,
	.eccbytes = 24,
	.eccpos = {8, 9, 10, 13, 14, 15, 24, 25,
			   26, 29, 30, 31, 40, 41, 42, 45,
			   46, 47, 56, 57, 58, 61, 62, 63},
	.oobfree = { {2, 6}, {16, 8}, {32, 8}, {48,8}, {11,2}, {27,2}, {43,2}, {59,2}}
};

static struct nand_ecclayout nand_sp_hw_eccoob = {
	.eccbytes = 6,
	.eccpos = {8, 9, 10, 13, 14, 15},
	.oobfree = { {2, 6} }
};

static void tmpa910_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	unsigned i = 0;
	struct nand_chip *this = mtd->priv;
	struct tmpa910_nand_private * priv= (struct tmpa910_nand_private *)this->priv;

	if (len > (mtd->writesize + mtd->oobsize - priv->column))
		len = mtd->writesize + mtd->oobsize- priv->column;
	if (priv->dma && priv->orig_column <  mtd->writesize) {
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
	struct nand_chip *this = mtd->priv;
	int i, eccsize = this->ecc.size;
	struct tmpa910_nand_private * priv= (struct tmpa910_nand_private *)this->priv;

	if (len > (mtd->writesize + mtd->oobsize - priv->column))
		len = mtd->writesize + mtd->oobsize - priv->column;
	
	if (priv->dma && priv->orig_column < mtd->writesize) {
		memcpy(priv->buf + priv->column, buf, len);
		if (!priv->softecc) {
			unsigned char *eccbuf;

			if (priv->mlc)
				eccbuf = priv->eccbuf + (priv->column / eccsize) * 10;
			else
				eccbuf = priv->eccbuf + (priv->column / eccsize) * 6;

			if (priv->column < mtd->writesize) {
				/* we can be sure that len = ecc_step and priv->column is aligned by ecc_step(512 byte) */
				tmpa910_nand_dma_write(priv, priv->phy_buf + priv->column, len);
				tmpa910_nand_start_autoload(0);
				if (tmpa910_nand_wait_dma_complete(priv, DMA_TIMEOUT)) {
					printk("ERROR: Write page :, column:0x%x time out\n", priv->column);
					return;
				}

				while(!tmpa910_nand_als_ready(mtd));

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
       struct tmpa910_nand_private * priv= (struct tmpa910_nand_private *)this->priv;

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

static int  tmpa910_nand_calculate_ecc(struct mtd_info *mtd, const unsigned char *dat, unsigned char *ecc_code)
{
	struct nand_chip *this = mtd->priv;	
	int eccsize = this->ecc.size;
	struct tmpa910_nand_private * priv= (struct tmpa910_nand_private *)this->priv;
	int steps;

	/* No DMA && No read OOB */
	if (!priv->dma || priv->softecc) {
		if (priv->softecc) {
			steps = eccsize / ECCBUF_SIZE;
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

static int  tmpa910_nand_part_correctdata(unsigned char *data, unsigned char *eccdata, unsigned char ecc1, unsigned char ecc2, unsigned char ecc3)
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

	/* sometimes people do not think about using the ECC, so check
	 * to see if we have an 0xff,0xff,0xff read ECC and then ignore
	 * the error, on the assumption that this is an un-eccd page.
	 */
	if (eccdata[0] == 0xff && eccdata[1] == 0xff && eccdata[2] == 0xff)
		return 0;

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
	int eccsize = this->ecc.size;
	unsigned int size;
	unsigned char ecc1, ecc2, ecc3; /* FIXME should define as U8 */
	unsigned int ret, ret_tmp;

	size = eccsize;

	ret_tmp = 0;
	while (size > 0)
	{
		ecc2 = *calc_ecc++;
		ecc1 = *calc_ecc++;
		ecc3 = *calc_ecc++;
		ret = tmpa910_nand_part_correctdata(data, read_ecc, ecc1, ecc2, ecc3);
		if(ret > ret_tmp) {
			ret_tmp = ret;
		}
			
		size -= ECCBUF_SIZE;
		data += ECCBUF_SIZE;
		read_ecc += 3;	
	}
	return (ret_tmp != 0 && ret_tmp != 1)? -1 : 0;
}

static void tmpa910_nand_set_timing(struct tmpa910_nand_private *priv)
{
#if 1
	NDFMCR2 = 0x2222;
#else
	unsigned timing;
	timing = 	(priv->timing.splw << 12) |
			(priv->timing.sphw << 8) |
			(priv->timing.splr << 4) |
			priv->timing.sphr;
	printk("NAND: setiing NDFMCR2: 0x%04x\n", timing);
	NDFMCR2 = timing;
#endif
}

static void tmpa910_nand_dma_handler(int dma_ch, void *data)
{
	struct tmpa910_nand_private * priv;

	priv = (struct tmpa910_nand_private *)data;
	complete(&priv->dma_completion);	

	return;
}

static void tmpa910_nand_dma_error_handler(int dma_ch, void *data)
{
	struct tmpa910_nand_private * priv;

	priv = (struct tmpa910_nand_private *)data;
	complete(&priv->dma_completion);	
	printk("DMA Error happens at DMA channel %d\n", dma_ch);
	return;
}

static int tmpa910_nand_init_priv(struct tmpa910_nand_private *priv)
{
#if 1	//wym
	priv->dma = 1;
	priv->softecc = 0;
#else
	priv->dma = 0;
	priv->softecc = 1;
#endif
//	__REG(0xF080E008) = 0x02;
	//NDFMCR0 =0x95;
	NDFMCR0 =0x0;
	NDFMCR1 = 0;
	tmpa910_nand_set_timing(priv);
	
	return 0;
}

static int tmpa910_nand_probe(struct platform_device *pdev)
{	
	struct tmpa910_nand_private * priv;
	struct mtd_info *mtd;
	struct nand_chip *nand;
	int ret;
#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *partitions = NULL;
	int num_cmdline_parts;
#endif

	/* We only get one Nand Controller, so we do not modify CFG_NAND_BASE_LIST 
	     to get the multiple IO address for the controllers */
	/* Remeber to set CFG_MAX_NAND_DEVICE in the board config file to be the 
	    controller number. Now we only have one controller, so set it to be 1 */

	mtd = (struct mtd_info *)kzalloc(sizeof(struct mtd_info) + sizeof(struct tmpa910_nand_private), GFP_KERNEL);
	if (mtd == NULL) {
		printk(KERN_ERR "TMPA910_NAND: not enough mempry");
		ret = -ENOMEM;
		goto error;
	}
	priv = (struct tmpa910_nand_private *)(mtd + 1);
	priv->buf = (unsigned char *)dma_alloc_coherent(NULL,2112, &priv->phy_buf, GFP_KERNEL);
	if (priv->buf == NULL) {
		ret = -ENOMEM;
		goto free_mtd;
	}

	nand = &priv->chip;
	nand->IO_ADDR_R   = (void  __iomem *)(&NDFDTR);
	nand->IO_ADDR_W   = (void  __iomem *)(&NDFDTR);
	nand->chip_delay  = 0;
	nand->select_chip = tmpa910_nand_select_chip;

	/* The controller will generate 6 bytes ecc for 512 bytes data*/
	nand->ecc.calculate = tmpa910_nand_calculate_ecc;
	nand->ecc.correct   = tmpa910_nand_correct_data;
	nand->ecc.hwctl  = tmpa910_nand_enable_hwecc;
	nand->ecc.mode	    = NAND_ECC_HW;
	nand->ecc.size	    = 512;
	nand->ecc.bytes	    = 6;


	/* Set address of hardware control function */
	nand->cmdfunc = tmpa910_nand_command;
	nand->dev_ready = tmpa910_nand_dev_ready;
	nand->read_buf = tmpa910_nand_read_buf;
	nand->write_buf = tmpa910_nand_write_buf;

	mtd->priv = nand;
	platform_set_drvdata(pdev, mtd);

	ret = tmpa910_nand_init_priv(priv);
	if (ret) {
		goto free_dma_buf;
	}
	nand->priv = priv;

	init_completion(&priv->dma_completion);	
	priv->dma_ch = tmpa910_dma_request("TMPA910 NAND", 5, tmpa910_nand_dma_handler, 
					tmpa910_nand_dma_error_handler, priv);
	if (priv->dma_ch < 0) {
		ret = -ENODEV;
		goto free_dma_buf;	
	}
	
	mtd->owner    = THIS_MODULE;
	/* Many callers got this wrong, so check for it for a while... */
	ret = nand_scan_ident(mtd, 1);
	if (ret) {
		goto free_dma_ch;
	}

	if (mtd->writesize == 2048)
		nand->ecc.layout    = &nand_lp_hw_eccoob;
	else
		nand->ecc.layout    = &nand_sp_hw_eccoob;
	ret = nand_scan_tail(mtd);
	if (ret) {
		goto free_dma_ch;
	}

/* Partitions:
 * If there is support for partitions then use commandline partitions if
 * available, the defauts otherwise.
 * If there is no support for partitions then add the whole device.
 */
    
#ifdef CONFIG_MTD_PARTITIONS

#ifdef CONFIG_MTD_CMDLINE_PARTS
	mtd->name = "tmpa910-nand",
	num_cmdline_parts = parse_mtd_partitions(mtd, part_probes,
					      &partitions, 0);
	if (num_cmdline_parts)
		add_mtd_partitions(mtd, partitions, num_cmdline_parts);
    	else
		add_mtd_partitions(mtd, mtd_parts_builtin, num_partitions);
#else
	add_mtd_partitions(mtd, mtd_parts_builtin, num_partitions);
    
#endif
    
#else
	add_mtd_device(mtd);
#endif
	return(0);

free_dma_ch:
	tmpa910_dma_free(priv->dma_ch);
free_dma_buf:
	dma_free_coherent(&pdev->dev, 2112, priv->buf, priv->phy_buf);
free_mtd:
	kfree(mtd);
error:
	return ret;
}


static int tmpa910_nand_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd;
	struct tmpa910_nand_private * priv;

	mtd = platform_get_drvdata(pdev);
	priv = (struct tmpa910_nand_private *)(mtd->priv);
	dma_free_coherent(&pdev->dev, 2112, priv->buf, priv->phy_buf);

	kfree(mtd);
	return 0;
}

static int tmpa910_nand_suspend(struct platform_device *dev, pm_message_t pm)
{
	return 0;
}

static int tmpa910_nand_resume(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver tmpa910_nand_driver = {
	.probe = tmpa910_nand_probe,
	.remove = __devexit_p(tmpa910_nand_remove),
	.suspend = tmpa910_nand_suspend,
	.resume = tmpa910_nand_resume,
	.driver	= {
		.name = "tmpa910-nand",
		.owner	= THIS_MODULE,
	},
};

static int tmpa910_nand_init(void)
{
	return platform_driver_register(&tmpa910_nand_driver);
}

static void tmpa910_nand_exit(void)
{
	platform_driver_unregister(&tmpa910_nand_driver);
}
module_init(tmpa910_nand_init);
module_exit(tmpa910_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NAND flash driver for TMPA910");
