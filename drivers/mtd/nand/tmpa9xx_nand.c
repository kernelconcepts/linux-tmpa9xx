/*
 *  drivers/mtd/nand/tmpa9xx_nand.c 
 *
 * Copyright (C) 2008 ?. All rights reserved. (?)
 * Copyright (C) 2009, 2010 Florian Boor <florian.boor@kernelconcepts.de>
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
 * TMPA 9xx NAND controller driver
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
#include <mach/regs.h>

#define ECCBUF_SIZE 		 256
#define DMA_TIMEOUT   		0xFFFF
#define ECC_BYTES		 512
#define NAND_DMA_TRANSFER_SIZE 	 512
#define INTERNAL_BUFFER_SIZE 	 512
#define NORMALPAGE_SIZE		 512
#define LAGEPAGE_SIZE 		 (4*NORMALPAGE_SIZE)
#define NORMALPAGE_SPARE	  16
#define LARGEPAGE_SPARE		 (4*NORMALPAGE_SPARE)
#define ECC_BYTES_HAMMING	   6
#define ECC_BYTES_RS		  10
#define NAND_DMA_CHANNEL	   5

struct tmpa9xx_nand_timing {
	unsigned int splw;
	unsigned int sphw;
	unsigned int splr;
	unsigned int sphr;
};

struct tmpa9xx_nand_private {
	unsigned int dma;			/* use DMA */
        struct tmpa9xx_nand_timing timing;	/* Timing for NAND access */
        
	unsigned int mlc;			/* Flash is mlc (autodetection) */
        unsigned int page_size;			/* Page size of flash (autodetection) */
        unsigned int spare_size;		/* Spare size of flash (autodetection) */
	unsigned int column;			/* internal column position (internal) */
	unsigned int page_addr;			/* internal page position (internal) */
        unsigned int dma_ch;			/* used DMA channel (internal) */
        unsigned int chip_select;		/* Chips selected (internal) */
	unsigned char *buf;			/* Buffer pointer (internal) */
	unsigned int phy_buf;			/* internal Buffer (internal) */
        struct completion dma_completion;       /* copletion of DMA (internal) */ 
        struct nand_chip chip;			/* nand chip structure (internal) */
        spinlock_t  lock;			/* Lock for Pageread needed due to HW implementation */
};

/* Nand Control */
struct tmpa9xx_nand_private tmpa9xx_nand = {
	.dma	= 1,				/* Use driver with DMA */
	.timing =
	{
		.splw = 0x2,			/* NDWEn Low pulse width setting  */		
		.sphw = 0x2,			/* NDWEn High pulse width setting */
		.splr = 0x2,			/* NDREn Low pulse width setting  */
		.sphr = 0x2,			/* NDREn High pulse width setting */
	},
};

/* NAND OOB layout for different chip types */
static struct nand_ecclayout nand_oob_hamming_512 = {
	.eccbytes = 6,
	.eccpos =  {8, 9, 10, 13, 14, 15},
	.oobfree = { {2, 6} }
};

static struct nand_ecclayout nand_oob_hamming_2048 = {
	.eccbytes = 24,
	.eccpos   =  { 8,  9, 10, 13, 14, 15, 24, 25,
                      26, 29, 30, 31, 40, 41, 42, 45,
                      46, 47, 56, 57, 58, 61, 62, 63},
	.oobfree  = {{ 2, 6},
                     {16, 8},
                     {32, 8},
                     {48, 8},
                     {11, 2},
                     {27, 2},
                     {43, 2},
                     {59, 2}}
};

static struct nand_ecclayout nand_oob_rs_2048 = {
	.eccbytes = 40,
	.eccpos   =  { 6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
                      22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
                      38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
                      54, 55, 56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree  = {{ 2, 4},
                     {16, 6},
                     {32, 6},
                     {48, 6}}
};

#ifdef CONFIG_MTD_PARTITIONS
/*
 * Define static partitions for flash device
 */
static int num_partitions = 3;

static struct mtd_partition mtd_parts_builtin[] = {
	{
		.name		= "u-boot",
		.offset		= 0x00000000,
		.size		= 0x00060000,
	}, {
		.name		= "u-boot_env",
		.offset		= 0x00060000,
		.size		= 0x00020000,
	}, {
		.name		= "splash",
		.offset		= 0x00080000,
		.size		= 0x00300000,
	}, {
		.name		= "kernel",
		.offset		= 0x00380000,
		.size		= 0x00300000,
	}, {
		.name		= "rootfs",
		.offset		= 0x00680000,
		.size		= MTDPART_SIZ_FULL,
	}, 
};

static const char *part_probes[] = { "cmdlinepart", NULL };

#endif

void tmpa9xx_nand_dma_read(struct tmpa9xx_nand_private *priv, unsigned int buf, unsigned short size)
{
	DMA_SRC_ADDR(priv->dma_ch) = NDFDTR_PHY;	// Source address
	DMA_DEST_ADDR(priv->dma_ch)= buf;		// Destination address
	DMA_CONTROL(priv->dma_ch)  =((0x1<<12) |	// SBSize[2:0] - 0y001: 4 beats
        			     (0x1<<15) |	// DBSize[2:0] - 0y001: 4 beats
                                     (0x1<<19) |	// Swidth[2:0] - 0y010: Word (32 bits)
                                     (0x1<<22) |	// Dwidth[2:0] - 0y010: Word (32 bits)
                                     (0x1<<27) |	// DI          - 0y1  : Auto Increment Destination
                                     (0x1<<31) |	// I           - 0y1  : Enable Terminal count interrupt
                                     (size/4)); 	// TransferSize[11:0

	DMA_CONFIG(priv->dma_ch)   =((0x1<<0)  |	// E             - Channel Enable
        			     (0x1<<3)  |	// SrcPeripheral - Peripheral
                                     (0x1<<12) |	// FlowCntrl	 - Peripheral to Memory
                                     (0x1<<15));	// ITC           - Terminal count interrupt enable
}

void tmpa9xx_nand_dma_write(struct tmpa9xx_nand_private *priv, const unsigned int buf, unsigned short size)
{
	DMA_SRC_ADDR(priv->dma_ch) = buf;		// Source address
	DMA_DEST_ADDR(priv->dma_ch)= NDFDTR_PHY;	// Destination address
	DMA_CONTROL(priv->dma_ch)  =((0x1<<12) |	// SBSize[2:0] - 0y001: 4 beats
        			     (0x1<<15) |	// DBSize[2:0] - 0y001: 4 beats
                                     (0x1<<19) |	// Swidth[2:0] - 0y010: Word (32 bits)
                                     (0x1<<22) |	// Dwidth[2:0] - 0y010: Word (32 bits)
                                     (0x1<<26) |	// SI          - 0y1  : Auto Increment Source
                                     (0x1<<31) |	// I           - 0y1  : Enable Terminal count interrupt
                                     (size/4)); 	// TransferSize[11:0]
        
	DMA_CONFIG(priv->dma_ch)   =((0x1<<0)  |	// E             - Channel Enable
                                     (0x1<<8)  |	// DestPeripheral- Peripheral 
                                     (0x1<<11) |	// FlowCntrl     - Memory to Peripheral
                                     (0x1<<15));	// ITC           - Terminal count interrupt enable
}

static int tmpa9xx_nand_wait_dma_complete(struct tmpa9xx_nand_private *priv,unsigned int timeout)
{
	int ret;

	ret = wait_for_completion_interruptible_timeout(&priv->dma_completion, timeout);
	tmpa9xx_dma_disable(priv->dma_ch);

	return !ret;
}

static void tmpa9xx_nand_set_ecctype(unsigned int mlc)
{
	if (mlc)
		NDFMCR1 |= NDFMCR1_ECCS;	/* RS ECC */
	else
		NDFMCR1 &= ~NDFMCR1_ECCS;	/* Hamming ECC */
}

static void tmpa9xx_nand_start_autoload(unsigned int read)
{
	unsigned int val;

	val = NDFMCR1;
	if (read)
		val &= ~NDFMCR1_SELAL;	/*Set it to be read */
	else
		val |= NDFMCR1_SELAL;	/*Set it to be written */

	/* start autoload */
	val |= NDFMCR1_ALS;
	NDFMCR1 = val;
}

static void tmpa9xx_nand_start_ecc(unsigned int read,unsigned int mlc)
{
	if (mlc && read==0) {
		NDFMCR0 |= NDFMCR0_RSEDN;
	}
	if (read)
		NDFMCR0 |= NDFMCR0_ECCE |  NDFMCR0_ECCRST;
	else
		NDFMCR0 |= NDFMCR0_ECCE |  NDFMCR0_ECCRST | NDFMCR0_WE;
}

static void tmpa9xx_nand_set_rw_mode(unsigned int read)
{
	if (read)
		NDFMCR0 &= ~NDFMCR0_WE;	/* Set read mode */
	else
		NDFMCR0 |= NDFMCR0_WE;	/* Set write mode */
}

#if 0
static int tmpa9xx_nand_waitreedsolomon(void)
{
	unsigned int reg;
	unsigned short nCnt;
	
	for(nCnt=0;nCnt<0xffff;nCnt++)
	{
		reg = NDFINTC;
		
		if((reg & NDFINTC_RSERIS) == NDFINTC_RSERIS)
		{
			NDFINTC = NDFINTC_RSEIC;	// latch clear
			return 0;
		}
	}	
	return -1;	
}

static int tmpa9xx_nand_chkreedsolomon(unsigned char * databuf)
{
	unsigned short  sts_work;
	unsigned short  err_count;
	unsigned short  chkaddr;
	unsigned char   chkbit;
	unsigned char   i;

	NDFINTC = NDFINTC_LATCH_CLEAR;	// latch clear
	NDFMCR0 = NDFMCR0_ECC_RSM_ON;	/* Error bit calculation circuit start */
	
	if(tmpa9xx_nand_waitreedsolomon() == -1){
		return -1;
	}

	sts_work = (( NDFMCR1 & 0xf000 ) >> 12 );

	if(sts_work == 0){			/* 0:NO ERROR */
		return 0;
	}
	if(sts_work == 1){			/* 1:OVER 5 ERRORS */
		return -2;
	}
	if( (sts_work == 2) || (sts_work == 3) ){			
		err_count = (( NDFMCR1 & 0x0c00 ) >> 10 );

		for( i=0 ; i<err_count+1; i++ ){
			switch(i){
				case 0:
					chkaddr = NDRSCA0;
					chkbit = NDRSCD0;
					break;
				case 1:
					chkaddr = NDRSCA1;
					chkbit = NDRSCD1;
					break;
				case 2:
					chkaddr = NDRSCA2;
					chkbit = NDRSCD2;
					break;
				case 3:
					chkaddr = NDRSCA3;
					chkbit = NDRSCD3;
					break;
				default:
					return -3;
			}

			if( 0x0007 < chkaddr ){
				chkaddr = 0x207 - chkaddr;
				databuf[chkaddr] = databuf[chkaddr] ^ chkbit;
			}
		}
	}
	else{
		return -4;			/* OTHER: ERROR */
	}
	return 0;
}

static void tmpa9xx_nand_rson(void)
{
    NDFMCR0 = NDFMCR0_ECC_RSECGW_ON;
}

static void tmpa9xx_nand_rsoff(void)
{
    NDFMCR0 = NDFMCR0_ECC_RSECGW_OFF;
}
#endif

static int tmpa9xx_nand_dev_ready(struct mtd_info *mtd)
{
       return !(NDFMCR0 & NDFMCR0_BUSY);/* Wait for NAND device ready */
}

static int tmpa9xx_nand_als_ready(struct mtd_info *mtd)
{
       return !(NDFMCR1 & NDFMCR1_ALS);	/* Wait for autoload finished */
}

/* Set WP on deselect, write enable on select */
static void tmpa9xx_nand_select_chip(struct mtd_info *mtd, int chip)
{
	unsigned int val;

	switch (chip)
	{
	case 0:
		val =  NDFMCR0;
		val |= NDFMCR0_CE0;	/*Enable  chip0 */
		val &= ~NDFMCR0_CE1;    /*Disable chip1 */
		NDFMCR0 = val;
		break;
		
	case 1:
		val = NDFMCR0;
		val |= NDFMCR0_CE1;	/*Enable  chip0 */
		val &= ~NDFMCR0_CE0;    /*Disable chip1 */
		NDFMCR0 = val;
		break;
		
	case -1:
		val = NDFMCR0;		/* Disable both */
		val &= ~(NDFMCR0_CE1 | NDFMCR0_CE0); 
		NDFMCR0 = val;
		break;
	
	default:
		printk(KERN_ERR "tmpa9xx_nand_select_chip - trying to enable chip %d - that is not possible\n",chip);
		break;
	}					
}

static u_char tmpa9xx_nand_read_byte(struct mtd_info *mtd)
{
	tmpa9xx_nand_set_rw_mode(1);	/* Set direction to read */
	return NDFDTR;			/* Poll out data */
}

static void tmpa9xx_nand_set_cmd(unsigned int cmd)
{
	/* Begin command latch cycle */
	NDFMCR0 |= NDFMCR0_CLE |NDFMCR0_WE;
	/* Write out the command to the device. */
	NDFDTR = cmd;
	/* End command latch cycle */
	NDFMCR0 &= ~(NDFMCR0_CLE |NDFMCR0_WE);
}

static void tmpa9xx_nand_set_addr(unsigned int column, unsigned int page_addr)
{
	NDFMCR0 |= NDFMCR0_ALE |NDFMCR0_WE;

	/* Serially input address */
	if (column != -1) {
		/* Adjust columns for 16 bit buswidth */
		NDFDTR = column & 0xff;
		NDFDTR = column >> 8;
	}
	if (page_addr != -1) {
		NDFDTR = (unsigned char) (page_addr & 0xff);
		NDFDTR = (unsigned char) ((page_addr >> 8) & 0xff);
		NDFDTR = (unsigned char) ((page_addr >> 16) & 0xff);
	}
	/* Latch in address */
	NDFMCR0 &= ~(NDFMCR0_ALE |NDFMCR0_WE);
}

static void tmpa9xx_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	struct tmpa9xx_nand_private * priv= (struct tmpa9xx_nand_private *)this->priv;
	unsigned long irq_flags;

	/* The DMA Transfer can only be started for an exact size of 512 bytes to read */
        /* Therefore do the 512 pagesize reads with DMA (if enabled) and the 64/16 bytes for OOB with polling */
	/* In contrast to the u-boot case we can not directly use the provided buffer for the DMA transfer */
        /* Therefore we have to use our own buffer created with dma_alloc_coherent() */
        /* The startup of the DMA transfer is a little bit silly: */
        /* First send the command, then set up the DMA and autoload functions, then the address */
        /* The autoload waits then until the rising edge of the busy signal from the NAND and starts the transfer to the FIFO */

	/* During read, due to the design of the chip, it can happen that the NAND_CMD_READSTART is too long during */
	/* latching this in to NAND chip due to an interrupt appearing */
        /* Work around: Disable all interrupts at the time latching in the NAND_CMD_READSTART */
        
	if ((len==NAND_DMA_TRANSFER_SIZE) && (priv->dma==1))
        {
		tmpa9xx_nand_set_cmd(NAND_CMD_READ0);			/* Set readpage */
		tmpa9xx_nand_dma_read(priv,priv->phy_buf,len);		/* Set up the DMA transfer */
		tmpa9xx_nand_start_autoload(1);				/* Set up autoload for reading */
		tmpa9xx_nand_set_addr(priv->column,priv->page_addr);	/* Set adress to start reading */
               	spin_lock_irqsave(&priv->lock, irq_flags);		/* Disable interrupts */
		tmpa9xx_nand_set_cmd(NAND_CMD_READSTART);		/* Set readstart for large page devices */
		tmpa9xx_nand_set_rw_mode(1);				/* Set controller to read mode */
		spin_unlock_irqrestore(&priv->lock, irq_flags);		/* Enable Interrupts again */
		if ( tmpa9xx_nand_wait_dma_complete(priv,DMA_TIMEOUT)) {
			printk(KERN_ERR "read page :0x%x, column:0x%x time out\n", priv->page_addr, priv->column);
			return;
		}
		while(!tmpa9xx_nand_als_ready(mtd));
                        
                memcpy(buf,priv->buf,len);				/* Have to use our own dma_coherend created buffer */
                priv->column+=len;					/* Remember internal position */
	}
        else
        {
	       	int i;
 		tmpa9xx_nand_set_cmd(NAND_CMD_READ0);
		tmpa9xx_nand_set_addr(priv->column,priv->page_addr);
		tmpa9xx_nand_set_cmd(NAND_CMD_READSTART);
		tmpa9xx_nand_set_rw_mode(1);
		while(!tmpa9xx_nand_dev_ready(mtd));
		for (i=0;i < len;i++)
			*(buf+i)=tmpa9xx_nand_read_byte(mtd);
                priv->column+=len;
	}       
}

static void tmpa9xx_nand_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	struct tmpa9xx_nand_private * priv= (struct tmpa9xx_nand_private *)this->priv;
        static int former_column;

	/* The DMA Transfer can only be started for an exact size of 512 bytes to read */
        /* Therefore do the 512 pagesize reads with DMA (if enabled) and the 64/16 bytes for OOB with polling */
	/* In contrast to the u-boot case we can not directly use the provided buffer for the DMA transfer */
        /* Therefore we have to use our own buffer created with dma_alloc_coherent() */
        /* The startup of the DMA transfer is a little bit silly: */
        /* First send the command, then set up the DMA and autoload functions, then the address */
        /* The autoload waits then until the rising edge of the busy signal from the NAND and starts the transfer to the FIFO */
        
	if (priv->column==0 || (former_column!=2048 && len==64))
        {
		tmpa9xx_nand_set_cmd(NAND_CMD_SEQIN);			/* Set sqeuential data in */
		tmpa9xx_nand_set_addr(priv->column,priv->page_addr);	/* Set adress to start writing */
		tmpa9xx_nand_set_rw_mode(0);				/* Set controller to write mode */
        }       

	if ((len==NAND_DMA_TRANSFER_SIZE) && (priv->dma==1))
        {
                memcpy(priv->buf,buf,len);					/* Have to use our own dma_coherend created buffer */
		tmpa9xx_nand_dma_write(priv,priv->phy_buf, len);		/* Set up the DMA transfer */
		tmpa9xx_nand_start_autoload(0);					/* Set up autoload for writing */
		if ( tmpa9xx_nand_wait_dma_complete(priv,DMA_TIMEOUT)) {
			printk(KERN_ERR "write page :0x%x, column:0x%x time out\n", priv->page_addr, priv->column);
			return;
		}
		while(!tmpa9xx_nand_als_ready(mtd));				/* Wait till autoload has finished */
                priv->column+=len;						/* Remember internal position */
	}
        else
        {
        	int i;
		for (i=0;i < len;i++)
			NDFDTR=buf[i];						/* Poll in the data */
		while(!tmpa9xx_nand_dev_ready(mtd));
                priv->column+=len;						/* Remember internal position */
	} 
              
        former_column = priv->column;       
}

static void tmpa9xx_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct nand_chip *this = mtd->priv;	
	struct tmpa9xx_nand_private * priv= (struct tmpa9xx_nand_private *)this->priv;
        
	tmpa9xx_nand_start_ecc(mode == NAND_ECC_READ,priv->mlc);
}

static int  tmpa9xx_nand_calculate_ecc(struct mtd_info *mtd, const unsigned char *dat, unsigned char *ecc_code)
{
	struct nand_chip *this = mtd->priv;	
	struct tmpa9xx_nand_private * priv= (struct tmpa9xx_nand_private *)this->priv;
	unsigned int ecc_val = 0;
	unsigned char *buf = ecc_code;

	if (!priv->mlc) {
	//SLC, Hamming ECC data
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

static int  tmpa9xx_nand_part_correctdata_rs(unsigned char *data, unsigned char *eccdata, unsigned char ecc1, unsigned char ecc2, unsigned char ecc3)
{
	/*FIXME should be U31, U16 and U8 */
	unsigned int l; 	/* Working to check d */
	unsigned int d; 	/* Result of comparison */
	unsigned short i; 	/* For counting */
	unsigned char d1,d2,d3; /* Result of comparison */
	unsigned char a; 	/* Working for add */
	unsigned char add; 	/* Byte address of cor. DATA */
	unsigned char b; 	/* Working for bit */
	unsigned char bit; 	/* Bit address of cor. DATA */

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

static int tmpa9xx_nand_part_correctdata_hamming(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	struct nand_chip *this = mtd->priv;
	u_int32_t ecc_nand = read_ecc[0] | (read_ecc[1] << 8) | (read_ecc[2] << 16);
	u_int32_t ecc_calc = calc_ecc[0] | (calc_ecc[1] << 8) | (calc_ecc[2] << 16);
	u_int32_t diff     = ecc_calc ^ ecc_nand;
	
	if (diff) {
		if ((((diff >> 12) ^ diff) & 0xfff) == 0xfff) {
			/* Correctable error */
			if ((diff >> (12 + 3)) < this->ecc.size) {
				uint8_t find_bit = 1 << ((diff >> 12) & 7);
				uint32_t find_byte = diff >> (12 + 3);

				dat[find_byte] ^= find_bit;
				printk(KERN_DEBUG "Correcting single bit ECC error at offset: %d, bit: %d\n", find_byte, find_bit);
				return 1;
			} else {
				printk(KERN_DEBUG "ECC UNCORRECTED_ERROR 1\n");
				return -1;
			}
		} else if (!(diff & (diff - 1))) {
			/* Single bit ECC error in the ECC itself,
			   nothing to fix */
			printk(KERN_DEBUG "Single bit ECC error in ECC.\n");
			return 1;
		} else {
			/* Uncorrectable error */
			printk(KERN_DEBUG "ECC UNCORRECTED_ERROR 2\n");
			return -1;
		}
	}
	return(0);
}

static int tmpa9xx_nand_correct_data(struct mtd_info *mtd, u_char *data, u_char *read_ecc, u_char *calc_ecc)
{
	struct nand_chip *this = mtd->priv;
        struct tmpa9xx_nand_private * priv= (struct tmpa9xx_nand_private *)this->priv;
	
	
	if (priv->mlc)
	{
		unsigned int size;
		unsigned char ecc1, ecc2, ecc3;
		unsigned int ret, ret_tmp;

		size = 10;

		ret_tmp = 0;
		while (size > 0)
		{
			ecc2 = *calc_ecc++;
			ecc1 = *calc_ecc++;
			ecc3 = *calc_ecc++;
			ret = tmpa9xx_nand_part_correctdata_rs(data, read_ecc, ecc1, ecc2, ecc3);
			if(ret > ret_tmp)
				ret_tmp = ret;
			
			size -= ECCBUF_SIZE;
			data += ECCBUF_SIZE;
			read_ecc += 3;	
		}
		return (ret_tmp != 0 && ret_tmp != 1)? -1 : 0;
	}	
	else
		return tmpa9xx_nand_part_correctdata_hamming(mtd,data,read_ecc,calc_ecc);
	
	return 0; /* Fix compiler warning */
}

static void tmpa9xx_nand_set_timing(struct tmpa9xx_nand_private *priv)
{
	NDFMCR2 = ((priv->timing.splw<<12)|(priv->timing.sphw<<8)|(priv->timing.splr<<4)|(priv->timing.sphr<<0));
}

static void tmpa9xx_nand_dma_handler(int dma_ch, void *data)
{
	struct tmpa9xx_nand_private * priv;

	priv = (struct tmpa9xx_nand_private *)data;
	complete(&priv->dma_completion);	

	return;
}

static void tmpa9xx_nand_dma_error_handler(int dma_ch, void *data)
{
	struct tmpa9xx_nand_private * priv;

	priv = (struct tmpa9xx_nand_private *)data;
	complete(&priv->dma_completion);	
	printk(KERN_ERR "DMA Error happens at DMA channel %d\n", dma_ch);
	return;
}

static void tmpa9xx_nand_command (struct mtd_info *mtd, unsigned command, int column, int page_addr)
{
	register struct nand_chip *this = mtd->priv;
	struct tmpa9xx_nand_private * priv= (struct tmpa9xx_nand_private *)this->priv;

	switch (command)
	{
	case NAND_CMD_READ0:
                priv->column=column;
                priv->page_addr=page_addr;
		break;
	
	case NAND_CMD_PAGEPROG:
		tmpa9xx_nand_set_cmd(NAND_CMD_PAGEPROG);
		break;

	case NAND_CMD_READOOB:
		priv->column=priv->page_size;
                priv->page_addr=page_addr;
		break;

	case NAND_CMD_ERASE1:
		tmpa9xx_nand_set_cmd(NAND_CMD_ERASE1);
		tmpa9xx_nand_set_addr(-1,page_addr);
		break;

	case NAND_CMD_STATUS:
		tmpa9xx_nand_set_cmd(NAND_CMD_STATUS);
		break;

	case NAND_CMD_SEQIN:
                priv->column=column;
                priv->page_addr=page_addr;
		break;

	case NAND_CMD_READID:
		tmpa9xx_nand_set_cmd(NAND_CMD_READID);
		NDFMCR0 |= NDFMCR0_ALE |NDFMCR0_WE;
		NDFDTR = 0x00;
		/* Latch in address */
		NDFMCR0 &= ~(NDFMCR0_ALE |NDFMCR0_WE);
		break;

	case NAND_CMD_ERASE2:
		tmpa9xx_nand_set_cmd(NAND_CMD_ERASE2);
		break;

	case NAND_CMD_RESET:
		tmpa9xx_nand_set_cmd(NAND_CMD_RESET);
		break;

	case NAND_CMD_READSTART:
		tmpa9xx_nand_set_cmd(NAND_CMD_READSTART);
		break;
                
	case NAND_CMD_RNDOUTSTART:
		printk(KERN_DEBUG "NAND_CMD_RNDOUTSTART unsupported\n");
		break;

	case NAND_CMD_CACHEDPROG:
		printk(KERN_DEBUG "NAND_CMD_CACHEDPROG unsupported\n");
		break;

	case NAND_CMD_READ1:
		printk(KERN_DEBUG "NAND_CMD_READ1 unsupported\n");
		break;

	case NAND_CMD_RNDOUT:
		printk(KERN_DEBUG "NAND_CMD_RNDOUT unsupported\n");
		break;

	case NAND_CMD_RNDIN:
		printk(KERN_DEBUG "NAND_CMD_RNDIN unsupported\n");
		break;

	case NAND_CMD_STATUS_MULTI:
		printk(KERN_DEBUG "NAND_CMD_STATUS_MULTI unsupported\n");
		break;

	default:
		printk(KERN_DEBUG "Unknown command code:%x\n",command);
		break;	
	}

	while(!tmpa9xx_nand_dev_ready(mtd));
}

static void tmpa9xx_nand_get_internal_structure(struct tmpa9xx_nand_private *priv)
{
	int i;
	unsigned char id_code[4];

	tmpa9xx_nand_set_cmd(NAND_CMD_RESET);
	while(!tmpa9xx_nand_dev_ready(NULL));
	tmpa9xx_nand_set_cmd(NAND_CMD_READID);
	while(!tmpa9xx_nand_dev_ready(NULL));
	NDFMCR0 |= NDFMCR0_ALE |NDFMCR0_WE;
	NDFDTR = 0x00;
	/* Latch in address */
	NDFMCR0 &= ~(NDFMCR0_ALE |NDFMCR0_WE);
	while(!tmpa9xx_nand_dev_ready(NULL));

	NDFMCR0 &= ~NDFMCR0_WE;

	for (i=0;i<4;i++)
		id_code[i]=NDFDTR;
	
        if (id_code[1]==0x0c)
        	priv->mlc=1;
                
        if ( (id_code[3]&0x03) == 0x1)
        {
        	priv->page_size = LAGEPAGE_SIZE;
                priv->spare_size= LARGEPAGE_SPARE;
        }
        else
        {
        	priv->page_size = NORMALPAGE_SIZE;
                priv->spare_size= NORMALPAGE_SPARE;
        }
}	

static int __devinit tmpa9xx_nand_probe(struct platform_device *pdev)
{	
	struct tmpa9xx_nand_private * priv;
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

	mtd = (struct mtd_info *)kzalloc(sizeof(struct mtd_info) + sizeof(struct tmpa9xx_nand_private), GFP_KERNEL);
	if (mtd == NULL) {
		printk(KERN_ERR "tmpa9xx_NAND: not enough memory");
		ret = -ENOMEM;
		goto error;
	}

	priv = (struct tmpa9xx_nand_private *)(mtd + 1);

        nand = &priv->chip;
	priv = &tmpa9xx_nand;
       	nand->priv = priv;
	
	nand->IO_ADDR_R     = (void  __iomem *)(&NDFDTR);
	nand->IO_ADDR_W     = (void  __iomem *)(&NDFDTR);
	
	nand->chip_delay    = 0;

	/* Control Functions */
	nand->select_chip   = tmpa9xx_nand_select_chip;
	nand->cmdfunc       = tmpa9xx_nand_command;
	nand->dev_ready     = tmpa9xx_nand_dev_ready;
		
	/* Data access functions */
	nand->read_byte     = tmpa9xx_nand_read_byte;
	nand->read_buf      = tmpa9xx_nand_read_buf;
	nand->write_buf     = tmpa9xx_nand_write_buf;
	
	/* ECC Mode */
	nand->ecc.mode      = NAND_ECC_HW;

 	tmpa9xx_nand_select_chip(NULL,0);
 	tmpa9xx_nand_set_timing(priv);
	tmpa9xx_nand_get_internal_structure(priv);
        
	if (priv->dma==1)
	{
		nand->ecc.mode      = NAND_ECC_HW;
		nand->ecc.calculate = tmpa9xx_nand_calculate_ecc;
		nand->ecc.hwctl     = tmpa9xx_nand_enable_hwecc;
		nand->ecc.correct   = tmpa9xx_nand_correct_data;
		nand->ecc.size      = ECC_BYTES;
	
		if (priv->mlc) /* MLC identifier */
		{
			tmpa9xx_nand_set_ecctype(1);			/* Set for RS ECC */
			nand->ecc.layout	= &nand_oob_rs_2048;	/* MLC is alvawy largepage device */
			nand->ecc.bytes 	= ECC_BYTES_RS;		/* RS produces 10 bytes of ECC code */
		}
		else
		{
			tmpa9xx_nand_set_ecctype(0);			/* Set for Hamming ECC */
			nand->ecc.bytes = ECC_BYTES_HAMMING;		/* Hamming produces 6 bytes of ECC */
			if (priv->page_size==LAGEPAGE_SIZE)		/* Select OOB Layout depending of pagesize */
			{
				nand->ecc.layout    = &nand_oob_hamming_2048;
			}
			else
			{
				nand->ecc.layout    = &nand_oob_hamming_512;
			}
		}
	}
	else
	{
		nand->ecc.mode      = NAND_ECC_SOFT;
	}
        
        /* Reinitialize with reset values */
	NDFMCR0 =0x0;
	NDFMCR1 =0x0;

	mtd->priv = nand;
	platform_set_drvdata(pdev, mtd);
        
       	spin_lock_init(&priv->lock);

	init_completion(&priv->dma_completion);	
	priv->dma_ch = tmpa9xx_dma_request("TMPA9XX NAND", NAND_DMA_CHANNEL, tmpa9xx_nand_dma_handler,tmpa9xx_nand_dma_error_handler, priv);
	if (priv->dma_ch < 0) {
		ret = -ENODEV;
		goto free_mtd;	
	}

	priv->buf = (unsigned char *)dma_alloc_coherent(NULL,INTERNAL_BUFFER_SIZE, &priv->phy_buf, GFP_KERNEL);
	if (priv->buf == NULL) {
		ret = -ENOMEM;
		goto free_dma_ch;
	}
	
	mtd->owner    = THIS_MODULE;
	/* Many callers got this wrong, so check for it for a while... */
	ret = nand_scan_ident(mtd, 1, NULL);
	if (ret) {
		goto free_dma_buf;
	}
	
	ret = nand_scan_tail(mtd);
	if (ret) {
		goto free_dma_buf;
	}
        
/* Partitions:
 * If there is support for partitions then use commandline partitions if
 * available, the defauts otherwise.
 * If there is no support for partitions then add the whole device.
 */

#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
	mtd->name = "tmpa9xx-nand",
	num_cmdline_parts = parse_mtd_partitions(mtd, part_probes,
					      &partitions, 0);
	if (num_cmdline_parts)
		add_mtd_partitions(mtd, partitions, num_cmdline_parts);
    	else
		add_mtd_partitions(mtd, mtd_parts_builtin, num_partitions);
#else
	add_mtd_partitions(mtd, mtd_parts_builtin, num_partitions);
#endif //CONFIG_MTD_CMDLINE_PARTS
#else
	add_mtd_device(mtd);
#endif // CONFIG_MTD_PARTITIONS

	return(0);

free_dma_buf:
	dma_free_coherent(&pdev->dev, INTERNAL_BUFFER_SIZE, priv->buf, priv->phy_buf);
free_dma_ch:
	tmpa9xx_dma_free(priv->dma_ch);
free_mtd:
	kfree(mtd);
error:
	return ret;
}


static int __devexit tmpa9xx_nand_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd;
	struct tmpa9xx_nand_private * priv;

	mtd = platform_get_drvdata(pdev);
	priv = (struct tmpa9xx_nand_private *)(mtd->priv);
	dma_free_coherent(&pdev->dev, INTERNAL_BUFFER_SIZE, priv->buf, priv->phy_buf);

	kfree(mtd);
	return 0;
}

static int tmpa9xx_nand_suspend(struct platform_device *dev, pm_message_t pm)
{
	return 0;
}

static int tmpa9xx_nand_resume(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver tmpa9xx_nand_driver = {
	.probe = tmpa9xx_nand_probe,
	.remove = __devexit_p(tmpa9xx_nand_remove),
	.suspend = tmpa9xx_nand_suspend,
	.resume = tmpa9xx_nand_resume,
	.driver	= {
		.name = "tmpa9xx-nand",
	},
};

static int __init tmpa9xx_nand_init(void)
{
	return platform_driver_register(&tmpa9xx_nand_driver);
}

static void __exit tmpa9xx_nand_exit(void)
{
	platform_driver_unregister(&tmpa9xx_nand_driver);
}
module_init(tmpa9xx_nand_init);
module_exit(tmpa9xx_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NAND flash driver for TMPA9xx");
MODULE_ALIAS("platform:tmpa9xx-nand");

