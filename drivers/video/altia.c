/*
 *  drivers/video/altia.c
 *
 * Copyright (C) 2010,2011 Altia Inc.
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
 * Accellerator driver for Altia Middleware
 */


/* Module */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>

/* Memory */
#include <linux/mm.h>
#include <linux/mman.h>
#include <asm/io.h>
#include <asm/page.h>

/* Interrupts */
#include <linux/sched.h>
#include <linux/interrupt.h>

/* File Structure */
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>

/* poll */
#include <linux/poll.h>

/* ioctl */
#include <linux/fcntl.h>
#include "altia_ioctl.h"

/* Locks */
#include <linux/spinlock.h>

/* Misc */
#include <asm/string.h>
#include <linux/errno.h>

/* Time */
#include <linux/time.h>

/* DMA */
#include <mach/dma.h>

//MAFR
#include <asm/cacheflush.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Altia Inc.");

/* Amount of memory used by Altia for images and frame buffers.
** Values in bytes!
*/
static int block_size = 4; /* MB */
module_param(block_size, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(block_size, "Video memory block size in MBytes (range 1-4)");

static int block_count = 2;
module_param(block_count, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(block_count, "Number of blocks to use (range 1 to 64)");

#define ALTIA_MODULE_VERSION    0x010100

#define ENDIAN_LE32(x)      (x)
#define ENDIAN_LE16(x)      (x)

#define ALTIA_DEBUG         0
#define ALTIA_DEBUG_MEM     0
#define ALTIA_DEBUG_REG     0
#define ALTIA_DEBUG_ISR     0
#define ALTIA_TIMESTAMP     0


/***************************************************************************
** local definitions
***************************************************************************/

/* Maximum user count for this device */
#define ALTIA_MAX_USERS         1

/* Video memory */
#define ALTIA_MIN_MEMORY        1   /* MB */
#define ALTIA_MAX_MEMORY        4   /* MB */
#define ALTIA_MAX_BLOCK         6
#define ALTIA_BLOCK_COUNT       ALTIA_MAX_BLOCK

/* Internal memory (for scaling) */
#define ALTIA_INT_MEMORY        0xf8004000
#define ALTIA_INT_MEMORY_STEP   0x800
#define ALTIA_INT_MEMORY_COUNT  8
#define ALTIA_INT_MEMORY_SIZE   (ALTIA_INT_MEMORY_STEP*ALTIA_INT_MEMORY_COUNT)

/* IRQ Configuration */
#define ALTIA_LCDDA_IRQ         20
#define ALTIA_ISR_DEV           &altia_sync
#define ALTIA_ISR_ERROR         ENDIAN_LE32(1<<21)
#define ALTIA_ISR_LINE          ENDIAN_LE32(1<<20)
#define ALTIA_ISR_MASK          (ALTIA_ISR_ERROR | ALTIA_ISR_LINE)
#define ALTIA_IST_OFF           0x20

/* Register Access */
#define ALTIA_LCDC_OFFSET       0xF4200000UL
#define ALTIA_LCDC_SIZE         0x400

#define ALTIA_LCDDA_OFFSET      0xF2050000UL
#define ALTIA_LCDDA_SIZE        0x100

#define ALTIA_DMAC_OFFSET       0xF4100000UL
#define ALTIA_DMAC_SIZE         0x1000

#define ALTIA_LCDO_OFFSET       0xF00B0000UL
#define ALTIA_LCDO_SIZE         0x4

/* Device Memory */
#define ALTIA_MMAP_PHYS         0x00000000UL

/* Blit Configuration */
#define ALTIA_BLIT_START        ENDIAN_LE32(1<<30)
#define ALTIA_BLIT_ISR          ENDIAN_LE32((1<<16)|(1<<17))
#define ALTIA_BLIT_ISR_MASK     ENDIAN_LE32(~((1<<20)|(1<<21)))
#define ALTIA_BLIT_DMA_MASK     ENDIAN_LE32(~0x00008800)
#define ALTIA_BLIT_FIRST_LINE   ENDIAN_LE32(0x0800)
#define ALTIA_BLIT_LAST_LINE    ENDIAN_LE32(0x8000)
#define ALTIA_BLIT_X_PAD        ENDIAN_LE32(0x0200)

/* DMA Configuration */
#define ALTIA_DMA_CHAN          7
#define ALTIA_DMA_OFFSET        (0x100 + ALTIA_DMA_CHAN * 0x20)
#define ALTIA_DMA_SRC(x)        __raw_writel((x), altia_reg[ALTIA_DMAC_REG] + ALTIA_DMA_OFFSET + 0x00);
#define ALTIA_DMA_DST(x)        __raw_writel((x), altia_reg[ALTIA_DMAC_REG] + ALTIA_DMA_OFFSET + 0x04);
#define ALTIA_DMA_LLI(x)        __raw_writel((x), altia_reg[ALTIA_DMAC_REG] + ALTIA_DMA_OFFSET + 0x08);
#define ALTIA_DMA_CTL(x)        __raw_writel((x), altia_reg[ALTIA_DMAC_REG] + ALTIA_DMA_OFFSET + 0x0C);
#define ALTIA_DMA_CFG(x)        __raw_writel((x), altia_reg[ALTIA_DMAC_REG] + ALTIA_DMA_OFFSET + 0x10);
#define ALTIA_DMA_CLEAR_INT()   __raw_writel((1 << ALTIA_DMA_CHAN), altia_reg[ALTIA_DMAC_REG] + 0x8);
#define ALTIA_DMA_ENABLE()      __raw_writel(1, altia_reg[ALTIA_DMAC_REG] + 0x30);


/***************************************************************************
** type definitions
***************************************************************************/

typedef struct
{
    void * addr;
} ALTIA_MEM_BLOCK_T;

typedef struct
{
    unsigned long offset;
    unsigned long max;
} ALTIA_REG_BLOCK_T;

typedef enum
{
    ALTIA_LCDC_REG,
    ALTIA_LCDDA_REG,
    ALTIA_DMAC_REG,
    ALTIA_LCDO_REG,
    ALTIA_IRAM_REG,
    ALTIA_REG_COUNT
} ALTIA_REG_ID_T;

typedef struct
{
    int active;
    int line;
    int line_count;
    int dot_count;
    unsigned long saddr;
    int sstep;
    int sshift;
} ALTIA_DMA_T;


/***************************************************************************
** private functions
***************************************************************************/

static int altia_open(struct inode *inode, struct file *filp);
static int altia_release(struct inode *inode, struct file *filp);
static ssize_t altia_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);
static ssize_t altia_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos);
static long altia_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int altia_mmap(struct file *filp, struct vm_area_struct *vma);
static int altia_fasync(int fd, struct file *filp, int mode);
static int altia_remove(struct platform_device *pdev);
static int __init altia_probe(struct platform_device *pdev);
static irqreturn_t altia_isr(int irq, void * data);
static void altia_reg_write(int block, unsigned long reg, unsigned long value, int size);
static unsigned long altia_reg_read(int block, unsigned long reg, int size);
static void altia_blit(void);
static int altia_dma(void);
static void tmpa9xx_altia_dma_handler(int dma_ch, void *data);
static void tmpa9xx_altia_dma_error_handler(int dma_ch, void *data);


#if ALTIA_DEBUG
static void altia_dump_lcdda(char * msg);
#endif

#if ALTIA_TIMESTAMP
static unsigned long altia_getMS(void);
#endif


/***************************************************************************
** private data
***************************************************************************/

/* Welcome */
static const char * altia_msg =
{
    "Altia Hardware Interface Driver V1.0\n"
};

/* Module definition */
struct file_operations altia_fops =
{
    .owner          = THIS_MODULE,
    .read           = altia_read,
    .write          = altia_write,
    .open           = altia_open,
    .release        = altia_release,
    .unlocked_ioctl = altia_ioctl,
    .mmap           = altia_mmap,
    .fasync         = altia_fasync,
};

/* General */
static spinlock_t   altia_lock;
static int          altia_initialized;
static int          altia_user_count;

/* Interrupts */
static struct fasync_struct * altia_sync;
static volatile unsigned long altia_ist;
static volatile unsigned long altia_ga_cnt;
static volatile unsigned long altia_err_cnt;
static unsigned long altia_ga_use;
static unsigned long altia_err_use;

/* Memory management */
static spinlock_t   altia_mem_lock;
static ALTIA_MEM_BLOCK_T altia_mem_block[ALTIA_BLOCK_COUNT];
static int altia_mem_count;

/* Registers */
static ALTIA_REG_BLOCK_T altia_reg_block[ALTIA_REG_COUNT] =
{
    { ALTIA_LCDC_OFFSET,  ( ALTIA_LCDC_OFFSET  + ALTIA_LCDC_SIZE       - 1 ) },
    { ALTIA_LCDDA_OFFSET, ( ALTIA_LCDDA_OFFSET + ALTIA_LCDDA_SIZE      - 1 ) },
    { ALTIA_DMAC_OFFSET,  ( ALTIA_DMAC_OFFSET  + ALTIA_DMAC_SIZE       - 1 ) },
    { ALTIA_LCDO_OFFSET,  ( ALTIA_LCDO_OFFSET  + ALTIA_LCDO_SIZE       - 1 ) },
    { ALTIA_INT_MEMORY,   ( ALTIA_INT_MEMORY   + ALTIA_INT_MEMORY_SIZE - 1 ) },
};

static volatile void __iomem * altia_reg[ALTIA_REG_COUNT];

/* Blit operations */
static volatile int altia_blit_status;
static ALTIA_IO_UPDATE_T altia_blit_op;
static volatile ALTIA_DMA_T altia_dma_op;

/* Dual Port RAM */
static const unsigned long altia_internal_ram[ALTIA_INT_MEMORY_COUNT] =
{
    ALTIA_INT_MEMORY,                               /* Area 0 */
    ALTIA_INT_MEMORY + (2 * ALTIA_INT_MEMORY_STEP), /* Area 1 */
    ALTIA_INT_MEMORY + (4 * ALTIA_INT_MEMORY_STEP), /* Area 2 */
    ALTIA_INT_MEMORY + (6 * ALTIA_INT_MEMORY_STEP), /* Area 3 */
    ALTIA_INT_MEMORY + (1 * ALTIA_INT_MEMORY_STEP), /* Area 4 */
    ALTIA_INT_MEMORY + (3 * ALTIA_INT_MEMORY_STEP), /* Area 5 */
    ALTIA_INT_MEMORY + (5 * ALTIA_INT_MEMORY_STEP), /* Area 6 */
    ALTIA_INT_MEMORY + (7 * ALTIA_INT_MEMORY_STEP), /* Area 7 */
};

static struct cdev *altia_cdev;
static dev_t        altia_dev;
static struct class *altia_class;

static struct platform_driver altia_driver = {
  .remove = altia_remove,
  .probe = altia_probe,
  .driver = {
	     .name = "tmpa9xx-lcdda",
	     .owner = THIS_MODULE,
	     },
};
/***************************************************************************
** public functions
***************************************************************************/

int __init altia_init (void)
{
  int ret;
#if ALTIA_DEBUG
    printk(KERN_INFO "altia:  init\r\n");
#endif

  ret = platform_driver_register (&altia_driver);
  return ret;
}

void __exit altia_exit (void)
{
#if ALTIA_DEBUG
    printk(KERN_INFO "altia:  exit\r\n");
#endif
  platform_driver_unregister (&altia_driver);
}

static int __devinit altia_probe(struct platform_device *pdev)
{
    int status, count, size, ret;

#if ALTIA_DEBUG
    printk(KERN_INFO "altia:  probe\r\n");
#endif

    /* Reset init flag */
    altia_initialized = 0;

    /* user count */
    altia_user_count = 0;

    /* lock */
    spin_lock_init(&altia_lock);

    /* Initialize ISR status */
    altia_ist = 0;
    altia_ga_cnt = 0;
    altia_ga_use = 0;
    altia_err_cnt = 0;
    altia_err_use = 0;

    /* Clear operation status */
    altia_blit_status = 0;

    /* Clear async helpers */
    altia_sync = NULL;

    /* Validate Memory Parameters */
    memset(altia_mem_block, 0, sizeof(altia_mem_block));
    if ((block_count < 1) || (block_count > ALTIA_MAX_BLOCK))
    {
        printk(KERN_INFO "altia: invalid memory block count (%d).  Range is 1 to %d.\r\n",
               block_count, ALTIA_MAX_BLOCK);
        return -EBADMSG;
    }
    if ((block_size < ALTIA_MIN_MEMORY) || (block_size > ALTIA_MAX_MEMORY))
    {
        printk(KERN_INFO "altia: invalid memory block size (%d).  Range is %d to %d.\r\n",
               block_size, ALTIA_MIN_MEMORY, ALTIA_MAX_MEMORY);
        return -EBADMSG;
    }

    /* Map the registers */
    memset(altia_reg, 0, sizeof(altia_reg));
    for (count=0; count<ALTIA_REG_COUNT; count++)
    {
        altia_reg[count] =  ioremap_nocache(altia_reg_block[count].offset,
                                            altia_reg_block[count].max - altia_reg_block[count].offset + 1);
        if (!altia_reg[count])
        {
            printk(KERN_INFO "altia: cannot map device registers\r\n");
            return -EIO;
        }
    }

    /* Allocate the memory block */
    size = block_size * 1048576;
    for (count=0; count<block_count; count++)
    {
        altia_mem_block[count].addr = kzalloc(size, GFP_DMA);
        if (NULL == altia_mem_block[count].addr)
        {
            printk(KERN_INFO "altia: failed to allocate video memory\r\n");
            while (count > 0)
            {
                count--;
                kfree(altia_mem_block[count].addr);
            }
            return -ENOMEM;
        }
    }
    printk(KERN_INFO "altia: Using %d blocks of %d bytes for video RAM\r\n",
           block_count, size);

    /* Setup LCDDA interrupt routine */
    status = request_irq(ALTIA_LCDDA_IRQ,       /* IRQ number */
                 altia_isr,                     /* ISR function */
                 IRQF_DISABLED | IRQF_SHARED,   /* Make interrupt fast + shareable */
                 "altia",                       /* Name of this device */
                 ALTIA_ISR_DEV);                /* dev_id -- unique ID for us */
    if (status)
    {
        printk(KERN_INFO "altia: cannot create LCDDA IRQ handler\r\n");
        return status;
    }
	
    status = tmpa9xx_dma_request(tmpa9xx_altia_dma_handler, tmpa9xx_altia_dma_error_handler, NULL);
    if (status<0)
    {
        printk(KERN_INFO "altia: cannot create DMA IRQ handler\r\n");
        return status;
    }

    /* Register Altia device */
    status = alloc_chrdev_region (&altia_dev, 0, 1, "altia");
    if (status < 0)
    {
        printk(KERN_INFO "altia: cannot register character device %d\r\n", MAJOR (altia_dev));
        return status;
    }
    
    /* add lcdda dev into cdev */
    altia_cdev = cdev_alloc ();
    cdev_init (altia_cdev, &altia_fops);
    ret = cdev_add (altia_cdev, altia_dev, 1);
    if (ret)
    {
        printk(KERN_ERR "Unable to add cdev %s.", "altia");
        unregister_chrdev_region (altia_dev, 1);
        return ret;
    }
   
    altia_class=class_create(THIS_MODULE, "altia");
    device_create(altia_class, NULL, altia_dev ,NULL , "altia");

    /* we're now initialized */
    altia_initialized = 1;

    return 0;
}

static int __devexit altia_remove(struct platform_device *pdev)
{
#if ALTIA_DEBUG
    printk(KERN_INFO "altia:  remove\r\n");
#endif
    if (altia_initialized)
    {
        int count;

        /* Kill the ISR */
        free_irq(ALTIA_LCDDA_IRQ, ALTIA_ISR_DEV);

	/* Free DMA Channel */
	tmpa9xx_dma_free(ALTIA_DMA_CHAN);
        /* Release register maps */
        for (count = 0; count < ALTIA_REG_COUNT; count++)
        {
            if (altia_reg[count])
                iounmap(altia_reg[count]);
        }
        memset(altia_reg, 0, sizeof(altia_reg));

        /* Free video memory */
        for (count = 0; count < ALTIA_BLOCK_COUNT; count++)
        {
            if (altia_mem_block[count].addr)
            {
                kfree(altia_mem_block[count].addr);
                altia_mem_block[count].addr = NULL;
            }
        }

        altia_initialized = 0;

        /* Remove Altia device */
	
        unregister_chrdev_region (altia_dev, 1);
        cdev_del (altia_cdev);
        device_destroy(altia_class,altia_dev);
        class_destroy(altia_class);
    }
    
    return 0;
}

static int altia_open(struct inode *inode, struct file *filp)
{
#if ALTIA_DEBUG
    printk(KERN_INFO "altia:  application connect\r\n");
#endif

    if (!altia_initialized)
    {
        printk(KERN_INFO "altia:  module not initialized\r\n");
        return -EPERM;
    }

    /* Initialize memory lock */
    spin_lock_init(&altia_mem_lock);

    /* Set execution lock */
    spin_lock(&altia_lock);

    /* Only one user allowed */
    if (altia_user_count >= ALTIA_MAX_USERS)
    {
        spin_unlock(&altia_lock);
        printk(KERN_INFO "altia:  too many users\r\n");
        return -EUSERS;
    }
    else
        altia_user_count++;

    /* Clear operation status */
    altia_blit_status = 0;

    /* Reset video memory */
    altia_mem_count = 0;

    spin_unlock(&altia_lock);

    return 0;
}

static int altia_release(struct inode *inode, struct file *filp)
{
#if ALTIA_DEBUG
    printk(KERN_INFO "altia:  application release\r\n");
#endif

    spin_lock(&altia_lock);

    if (altia_user_count > 0)
        altia_user_count--;

    /* remove async helpers */
    if (altia_sync)
    {
        altia_fasync(-1, filp, 0);
        altia_sync = NULL;
    }

    spin_unlock(&altia_lock);

    return 0;
}

static ssize_t altia_read (struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    int size = strlen(altia_msg);

#if ALTIA_DEBUG
    printk(KERN_INFO "altia:  read data\r\n");
#endif

    if (count < size)
        size = count;

    if (copy_to_user(buf, altia_msg, size))
        return -EBADE;

    *f_pos += size;
    return size;
}

static ssize_t altia_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
#if ALTIA_DEBUG
    printk(KERN_INFO "altia:  write data\r\n");
#endif
    return -ENOSYS;
}

static long altia_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int size;

    if (!altia_initialized)
        return -EPERM;

    switch(cmd)
    {
        case ALTIA_IOIST:
        {
            ALTIA_IO_IST_T result;
            unsigned long status;
            unsigned long gcnt, ecnt;

            if (!access_ok(VERIFY_WRITE, (void *)arg, sizeof(ALTIA_IO_IST_T)))
            {
                printk(KERN_INFO "altia: ioctl argument not accessible\r\n");
                return -EPROTO;
            }

            /* Get data from ISR routine */
            spin_lock(&altia_lock);
            status = altia_ist;
            gcnt = altia_ga_cnt;
            ecnt = altia_err_cnt;
            spin_unlock(&altia_lock);

            /* Process number of GA finished ISRs since last call */
            gcnt -= altia_ga_use;
            altia_ga_use += gcnt;
            result.ga = gcnt;

            /* Process number of error ISRs since last call */
            ecnt -= altia_err_use;
            altia_err_use += ecnt;
            result.err = ecnt;

            result.status = status;

#if ALTIA_DEBUG_ISR
            printk(KERN_INFO "altia: ALTIA_IOIST ist=%ld ga=%i\r\n", result.status, result.ga);
#endif

            size = copy_to_user((void *)arg, &result, sizeof(ALTIA_IO_IST_T));
            if (size)
            {
                printk(KERN_INFO "altia: cannot copy to ioctl argument (%d bytes missed)\r\n", size);
                return -EBADE;
            }
        }
        break;

        case ALTIA_IOREGR:
        case ALTIA_IOREGW:
        {
            ALTIA_IO_REG_T regblk;
            int block;

            if (!access_ok(VERIFY_WRITE, (void *)arg, sizeof(ALTIA_IO_REG_T)))
            {
                printk(KERN_INFO "altia: ioctl argument not accessible\r\n");
                return -EPROTO;
            }

            size = copy_from_user(&regblk, (void *)arg, sizeof(ALTIA_IO_REG_T));
            if (size)
            {
                printk(KERN_INFO "altia: cannot copy from ioctl argument (%d bytes missed)\r\n", size);
                return -EBADE;
            }

            for (block=0; block<ALTIA_REG_COUNT; block++)
            {
                if ((regblk.reg >= altia_reg_block[block].offset) &&
                    (regblk.reg <= altia_reg_block[block].max))
                {
                    if (cmd == ALTIA_IOREGR)
                    {
                        regblk.value = altia_reg_read(block, regblk.reg, regblk.size);
#if ALTIA_DEBUG_REG
                        printk(KERN_INFO "altia: ALTIA_IOREGR 0x%08lx -> 0x%08lx\r\n", regblk.reg, regblk.value);
#endif
                        size = copy_to_user((void *)arg, &regblk, sizeof(ALTIA_IO_REG_T));
                        if (size)
                        {
                            printk(KERN_INFO "altia: cannot copy to ioctl argument (%d bytes missed)\r\n", size);
                            return -EBADE;
                        }
                    }
                    else
                    {
                        altia_reg_write(block, regblk.reg, regblk.value, regblk.size);
#if ALTIA_DEBUG_REG
                        printk(KERN_INFO "altia: ALTIA_IOREGW 0x%08lx <- 0x%08lx\r\n", regblk.reg, regblk.value);
#endif
                    }

                    /* Successful read/write register operation */
                    return 0;
                }
            }

            /* If we made it this far then the register address was not valid */
            printk(KERN_INFO "altia: invalid register access (0x%08lx)\r\n", regblk.reg);
            return -EBADMSG;
        }
        break;

        case ALTIA_IOUPDATE:
        {
            int pending;

            /* If an operation is pending, then abort this new update */
            spin_lock(&altia_lock);
            if (altia_blit_status > 0)
                pending = 1;
            else
                pending = 0;
            spin_unlock(&altia_lock);
            if (pending)
            {
                printk(KERN_INFO "altia: invalid update (blitter busy)\r\n");
                return -EBUSY;
            }

            if (!access_ok(VERIFY_WRITE, (void *)arg, sizeof(ALTIA_IO_UPDATE_T)))
            {
                printk(KERN_INFO "altia: ioctl argument not accessible\r\n");
                return -EPROTO;
            }

            size = copy_from_user((void *)&altia_blit_op, (void *)arg, sizeof(ALTIA_IO_UPDATE_T));
            if (size)
            {
                printk(KERN_INFO "altia: cannot copy from ioctl argument (%d bytes missed)\r\n", size);
                return -EBADE;
            }

            /* Configure the operation */
            __raw_writel(altia_blit_op.dr1, altia_reg[ALTIA_LCDDA_REG] + 0x0004);
            __raw_writel(altia_blit_op.dr0, altia_reg[ALTIA_LCDDA_REG] + 0x0008);
            __raw_writel(altia_blit_op.fcp, altia_reg[ALTIA_LCDDA_REG] + 0x000C);
            __raw_writel(altia_blit_op.efcp, altia_reg[ALTIA_LCDDA_REG] + 0x0010);
            __raw_writel(altia_blit_op.dv1, altia_reg[ALTIA_LCDDA_REG] + 0x0014);
            __raw_writel(altia_blit_op.cr2, altia_reg[ALTIA_LCDDA_REG] + 0x0018);
            __raw_writel(altia_blit_op.dxdst, altia_reg[ALTIA_LCDDA_REG] + 0x001C);
            __raw_writel(altia_blit_op.dydst, altia_reg[ALTIA_LCDDA_REG] + 0x0020);
            __raw_writel(altia_blit_op.ssize, altia_reg[ALTIA_LCDDA_REG] + 0x0024);
            __raw_writel(altia_blit_op.dsize, altia_reg[ALTIA_LCDDA_REG] + 0x0028);
            __raw_writel(altia_blit_op.s0adr, altia_reg[ALTIA_LCDDA_REG] + 0x002C);
            __raw_writel(altia_blit_op.dadr, altia_reg[ALTIA_LCDDA_REG] + 0x0030);
            __raw_writel(altia_blit_op.dv0, altia_reg[ALTIA_LCDDA_REG] + 0x0038);

            /* Setup the DMA if this is a scaled blit, otherwise just blit */
            if (0x01000000 == (altia_blit_op.cr1 & 0x1F000000))
            {
                //memset((void *)altia_reg[ALTIA_IRAM_REG], 0, ALTIA_INT_MEMORY_SIZE);

                /* DMA is active */
                altia_dma_op.active = 1;

                /* Set blit status to line count */
                altia_dma_op.line_count = 1 + ((altia_blit_op.ssize & 0x3ff000) >> 12);
                altia_dma_op.line = 0;

                /* Set blit status to active (line count) */
                altia_blit_status = altia_dma_op.line_count;

                /* Source */
                altia_dma_op.saddr = ((altia_blit_op.cr0 & 0xff) << 24) | (altia_blit_op.cr1 & 0xffffff);
                altia_dma_op.dot_count = (altia_blit_op.ssize & 0x3ff) + 1;

                /* Source size */
                if (altia_blit_op.cr0 & (1 << 10))
                {
                    /* 16-bit source color fmt (2-byte source) */
                    altia_dma_op.sshift = 2;
                }
                else
                {
                    /* 32-bit source color fmt (4-byte source) */
                    altia_dma_op.sshift = 4;
                }

                /* Byte step from start of source line to start of next line (stride)
                ** Remove the 2-pixel pad added to source width for scaling
                ** (hardware defect)
                */
                altia_dma_op.sstep = (altia_blit_op.ssize & 0x3ff) - 1; //2;
                altia_dma_op.sstep *= altia_blit_op.dv1 & 0x7;
                altia_dma_op.sstep += (altia_blit_op.dv1 & 0x7ffc0) >> 6;

                /* Enable the interrupts */
                altia_blit_op.cr0 &= ALTIA_BLIT_ISR_MASK;
                altia_blit_op.cr0 &= ALTIA_BLIT_DMA_MASK;
                altia_blit_op.cr0 |= ALTIA_BLIT_ISR | ALTIA_BLIT_X_PAD;

                __raw_writel(altia_blit_op.cr0, altia_reg[ALTIA_LCDDA_REG] + 0x0000);

                /* Validate expansion and reduction rates */
                if (0 == (altia_blit_op.ssize & 0xff000000))
                {
                    /* X-Scaling disabled -- enable it to a safe value */
                    if (0 == (altia_blit_op.dxdst & 0xf0000000))
                    {
                        /* Reduction is also off so use a 1/2 * 2 factor */
                        altia_blit_op.dxdst |= 0x10000000;
                        __raw_writel(altia_blit_op.dxdst, altia_reg[ALTIA_LCDDA_REG] + 0x001C);
                        altia_blit_op.ssize |= 0x80000000;
                    }
                    else
                    {


                        /* We have reduction so use minimal expansion */
                        altia_blit_op.ssize |= 0xff000000;
                    }
                    __raw_writel(altia_blit_op.ssize, altia_reg[ALTIA_LCDDA_REG] + 0x0024);
                }
                if (0 == (altia_blit_op.dsize & 0xff000000))
                {
                    /* Y-Scaling disabled -- enable it to a safe value */
                    if (0 == (altia_blit_op.dydst & 0xf0000000))
                    {
                        /* Reduction is also off so use a 1/2 * 2 factor */
                        altia_blit_op.dydst |= 0x10000000;
                        __raw_writel(altia_blit_op.dydst, altia_reg[ALTIA_LCDDA_REG] + 0x0020);
                        altia_blit_op.dsize |= 0x80000000;
                    }
                    else
                    {
                        /* We have reduction so use minimal expansion */
                        altia_blit_op.dsize |= 0xff000000;
                    }
                    __raw_writel(altia_blit_op.dsize, altia_reg[ALTIA_LCDDA_REG] + 0x0028);
                }
#if ALTIA_DEBUG
                altia_dump_lcdda("IOUPATE (scaled)");
#endif
                /* Start the DMA */
                altia_dma();
            }
            else
            {
                /* DMA is not active */
                altia_dma_op.active = 0;

                /* Set blit status to active (line count) */
                altia_blit_status = 1;

                /* Enable the interrupts */
                altia_blit_op.cr0 |= ALTIA_BLIT_ISR;
                __raw_writel(altia_blit_op.cr0, altia_reg[ALTIA_LCDDA_REG] + 0x0000);
#if ALTIA_DEBUG
                altia_dump_lcdda("IOUPATE (normal/blend)");
#endif
                /* Start the operation */
                altia_blit();
            }
        }
        break;

        case ALTIA_IOSTATUS:
        {
            ALTIA_IO_STATUS_T statblk;
            if (!access_ok(VERIFY_WRITE, (void *)arg, sizeof(ALTIA_IO_STATUS_T)))
            {
                printk(KERN_INFO "altia: ioctl argument not accessible\r\n");
                return -EPROTO;
            }

            /* Check status of blit routine */
            spin_lock(&altia_lock);
            statblk.operation = 0;
            if (altia_blit_status)
                statblk.pending = 1;
            else
                statblk.pending = 0;
            spin_unlock(&altia_lock);

            __cpuc_flush_user_all();

            size = copy_to_user((void *)arg, &statblk, sizeof(ALTIA_IO_STATUS_T));
            if (size)
            {
                printk(KERN_INFO "altia: cannot copy to ioctl argument (%d bytes missed)\r\n", size);
                return -EBADE;
            }
        }
        break;

        case ALTIA_IOALLOC:
        case ALTIA_IOFREE:
        {
            ALTIA_IO_MEMORY_T memblk;

            if (!access_ok(VERIFY_WRITE, (void *)arg, sizeof(ALTIA_IO_MEMORY_T)))
            {
                printk(KERN_INFO "altia: ioctl argument not accessible\r\n");
                return -EPROTO;
            }

            size = copy_from_user(&memblk, (void *)arg, sizeof(ALTIA_IO_MEMORY_T));
            if (size)
            {
                printk(KERN_INFO "altia: cannot copy from ioctl argument (%d bytes missed)\r\n", size);
                return -EBADE;
            }

            if (cmd == ALTIA_IOALLOC)
            {
                /* Save size */
                memblk.size = (block_size * 1048576);

                /* Grab next VRAM block */
                memblk.physical = 0;
                while (altia_mem_count < ALTIA_BLOCK_COUNT)
                {
                    if(altia_mem_block[altia_mem_count].addr)
                    {
                        memblk.physical = virt_to_phys(altia_mem_block[altia_mem_count].addr);
                        altia_mem_count++;
                        break;
                    }
                    else
                    {
                        /* Try next block */
                        altia_mem_count++;
                    }
                }

                size = copy_to_user((void *)arg, &memblk, sizeof(ALTIA_IO_MEMORY_T));
                if (size)
                {
                    printk(KERN_INFO "altia: cannot copy to ioctl argument (%d bytes missed)\r\n", size);
                    return -EBADE;
                }
            }
            else
            {
                /* Reset the block counter */
                altia_mem_count = 0;
            }
        }
        break;

        case ALTIA_IORESET:
            __raw_writel(0x80000000, altia_reg[ALTIA_LCDDA_REG] + 0x0034);
            break;

        case ALTIA_IOVER:
            return ALTIA_MODULE_VERSION;

        default:
            printk(KERN_INFO "altia: ioctl unrecognized cmd (%08x)\r\n",cmd);
            return -ENOTTY;
    }

    return 0;
}

static int altia_mmap(struct file *filp, struct vm_area_struct *vma)
{
    unsigned long phys = ALTIA_MMAP_PHYS;

    /* Do not cache this memory */
    vma->vm_flags |= VM_RESERVED | VM_IO;

#if ALTIA_DEBUG_MEM
    printk(KERN_INFO "altia: mmap %ld bytes at 0x%08lx\r\n", (vma->vm_end-vma->vm_start), ALTIA_MMAP_PHYS);
#endif

    if(remap_pfn_range(vma,                                 /* VMA structure */
                        vma->vm_start,                      /* Virtual address start */
                        (phys >> PAGE_SHIFT),               /* Physical address page start */
                        (vma->vm_end-vma->vm_start),        /* Size */
                        vma->vm_page_prot))                 /* Protection */
        return -EAGAIN;

    return 0;
}

static int altia_fasync(int fd, struct file *filp, int mode)
{
#if ALTIA_DEBUG
    printk(KERN_INFO "altia: altia_fasync: %d\r\n", fd);
#endif

    return fasync_helper(fd, filp, mode, &altia_sync);
}


/***************************************************************************
** private functions
***************************************************************************/

static void altia_blit(void)
{
    /* If scaled op, then add appropriate scale flags to CR1 */
    if (0x01000000 == (altia_blit_op.cr1 & 0x1F000000))
    {
        if (altia_dma_op.line_count == altia_blit_status)
        {
            /* First line */
            altia_blit_op.cr0 |= ALTIA_BLIT_FIRST_LINE;
            __raw_writel(altia_blit_op.cr0, altia_reg[ALTIA_LCDDA_REG]);
        }
        else if (1 == altia_blit_status)
        {
            /* Last line */
            altia_blit_op.cr0 |= ALTIA_BLIT_LAST_LINE;
            __raw_writel(altia_blit_op.cr0, altia_reg[ALTIA_LCDDA_REG]);
        }
        else if (altia_blit_op.cr0 & ALTIA_BLIT_FIRST_LINE)
        {
            /* Second line */
            altia_blit_op.cr0 &= ~ALTIA_BLIT_FIRST_LINE;
            __raw_writel(altia_blit_op.cr0, altia_reg[ALTIA_LCDDA_REG]);
        }
    }

    altia_blit_op.cr1 |= ALTIA_BLIT_START;
    __raw_writel(altia_blit_op.cr1, altia_reg[ALTIA_LCDDA_REG] + 0x0034);
}

static int altia_dma(void)
{
    /* altia_dma_op.line == 0                               dummy first line
    ** altia_dma_op.line == 1 + altia_dma_op.line_count     dummy last line
    **                   == Inbetween values                source image lines
    */
    if (altia_dma_op.line > (altia_dma_op.line_count+1))
    {
        /* We've finished, return the done flag */
        altia_dma_op.active = 0;
        return 0;
    }

    /* Skip first line */
    if (!altia_dma_op.line)
        altia_dma_op.line++;

    /* Enable the DMA circuit */
    ALTIA_DMA_ENABLE();

    /* Set source */
    ALTIA_DMA_SRC(altia_dma_op.saddr);

    /* Set destination */
    ALTIA_DMA_DST(altia_internal_ram[altia_dma_op.line & 0x7] + altia_dma_op.sshift);

    /* Linked List (not used) */
    ALTIA_DMA_LLI(0);

    /* Control flags */
    if (2 == altia_dma_op.sshift)
    {
        /* 16-bit source color fmt (2-byte source) */
        ALTIA_DMA_CTL(0xEE29B000 + altia_dma_op.dot_count);
    }
    else
    {
        /* 32-bit source color fmt (4-byte source) */
        ALTIA_DMA_CTL(0xEE49B000 + altia_dma_op.dot_count);
    }

    /* Set next source address (for next DMA op) 
    ** Duplicate last source row copy
    */
    if (altia_dma_op.line < altia_dma_op.line_count)
        altia_dma_op.saddr += altia_dma_op.sstep;

    /* Increment the line counter (for next DMA op) */
    altia_dma_op.line++;

    /* Start the DMA */
    ALTIA_DMA_CFG(0x8001);

    /* DMA active */
    return 1;
}

static irqreturn_t altia_isr(int irq, void * data)
{
    /* Shared so make sure this interrupt is for us */
    if (data == ALTIA_ISR_DEV)
    {
        if (ALTIA_LCDDA_IRQ == irq)
        {
            /* Capture status */
            unsigned long status = __raw_readl(altia_reg[ALTIA_LCDDA_REG]);

            if (status & ALTIA_ISR_MASK)
            {
                /* Clear interrupt flags */
                __raw_writel((status & ~ALTIA_ISR_MASK), altia_reg[ALTIA_LCDDA_REG]);

                spin_lock(&altia_lock);

                /* GA ISR */
                if (status & ALTIA_ISR_LINE)
                {
                    altia_ga_cnt++;
                }

                /* Error ISR */
                if (status & ALTIA_ISR_ERROR)
                {
                    printk("LCDDA ERROR!");
                    altia_err_cnt++;
                }

                /* Save ISR status */
                altia_ist = status;

                /* Decrement status (line count) */
                altia_blit_status--;

                /* Activate DMA if there are more lines to process */
                if (altia_dma_op.active)
                {
                    if (!altia_dma())
                    {
                        /* We're done with the DMA -- continue with blit */
                        altia_blit();
                    }
                }
                else if (altia_blit_status)
                {
                    /* No more DMA activity -- finish blit */
                    altia_blit();
                }
                else
                {
                    /* Done with operation -- reset hardware */
                    __raw_writel(0x80000000, altia_reg[ALTIA_LCDDA_REG] + 0x0034);
                }

                spin_unlock(&altia_lock);

                if (altia_sync)
                    kill_fasync(&altia_sync, SIGIO, POLL_PRI);
            }

            return IRQ_HANDLED;
        }
    }

    return IRQ_NONE;
}

static void tmpa9xx_altia_dma_handler(int dma_ch, void *data)
{
	ALTIA_DMA_CLEAR_INT();

        spin_lock(&altia_lock);

        /* Need to fill internal ram until we have four lines */
        if (altia_dma_op.line < 4)
        {
            if (!altia_dma())
            {
                /* We don't have enough lines -- move on to blit */
                altia_blit();
            }
        }
        else
        {
            /* We finished a line, do the blit */
            altia_blit();
        }

        spin_unlock(&altia_lock);

        return;
}

static void tmpa9xx_altia_dma_error_handler(int dma_ch, void *data)
{
	printk(KERN_ERR "DMA Error happens at DMA channel %d\n", dma_ch);
	return;
}

#if ALTIA_TIMESTAMP
static unsigned long altia_getMS(void)
{
    struct timeval now;
    unsigned long ms;

    do_gettimeofday(&now);

    ms = now.tv_sec * 1000 + now.tv_usec / 1000;

    return ms;
}
#endif

static void altia_reg_write(int block, unsigned long reg, unsigned long value, int size)
{
    if (block >=0 && block < ALTIA_REG_COUNT)
    {
        unsigned long offset = reg - altia_reg_block[block].offset;
        volatile void __iomem * mem = altia_reg[block];

        switch(size)
        {
            case 8:
                __raw_writeb(value, (mem + offset));
                break;

            case 16:
                __raw_writew(value, (mem + offset));
                break;

            case 32:
            default:
                __raw_writel(value, (mem + offset));
                break;
        }
    }
}

static unsigned long altia_reg_read(int block, unsigned long reg, int size)
{
    unsigned long value = 0;

    if (block >=0 && block < ALTIA_REG_COUNT)
    {
        unsigned long offset = reg - altia_reg_block[block].offset;
        volatile void __iomem * mem = altia_reg[block];

        switch(size)
        {
            case 8:
                value = __raw_readb((mem + offset));
                break;

            case 16:
                value = __raw_readw((mem + offset));
                break;

            case 32:
            default:
                value = __raw_readl((mem + offset));
                break;
        }
    }

    return value;
}

#if ALTIA_DEBUG
static void altia_dump_lcdda(char * msg)
{
    if (msg)
        printk("LCDDA registers from %s\r\n", msg);
    printk("   CR0:   0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x0000), altia_blit_op.cr0);
    printk("   DR0:   0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x0008), altia_blit_op.dr0);
    printk("   DR1:   0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x0004), altia_blit_op.dr1);
    printk("   FCP:   0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x000C), altia_blit_op.fcp);
    printk("   EFCP:  0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x0010), altia_blit_op.efcp);
    printk("   DV0:   0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x0038), altia_blit_op.dv0);
    printk("   DV1:   0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x0014), altia_blit_op.dv1);
    printk("   CR2:   0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x0018), altia_blit_op.cr2);
    printk("   DXDST: 0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x001C), altia_blit_op.dxdst);
    printk("   DYDST: 0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x0020), altia_blit_op.dydst);
    printk("   SSIZE: 0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x0024), altia_blit_op.ssize);
    printk("   DSIZE: 0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x0028), altia_blit_op.dsize);
    printk("   S0ADR: 0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x002C), altia_blit_op.s0adr);
    printk("   DADR:  0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x0030), altia_blit_op.dadr);
    printk("   CR1:   0x%08x (0x%08lx)\r\n", __raw_readl(altia_reg[ALTIA_LCDDA_REG] + 0x0034), altia_blit_op.cr1);
}
#endif

/***************************************************************************
** Declare our standard module functions
***************************************************************************/
module_init(altia_init);
module_exit(altia_exit);

