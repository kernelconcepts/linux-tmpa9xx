
#ifndef _TMPA910_NAND_H
#define _TMPA910_NAND_H

#define	NDFMCR0_ECCRST	(0x1 << 0)
#define	NDFMCR0_BUSY	(0x1 << 1)
#define	NDFMCR0_ECCE	(0x1 << 2)
#define	NDFMCR0_CE1	(0x1 << 3)
#define	NDFMCR0_CE0	(0x1 << 4)
#define	NDFMCR0_CLE	(0x1 << 5)
#define	NDFMCR0_ALE	(0x1 << 6)
#define	NDFMCR0_WE	(0x1 << 7)
#define	NDFMCR0_RSEDN	(0x1 << 10)

#define	NDFMCR1_ECCS	(0x1 << 1)
#define	NDFMCR1_SELAL	(0x1 << 9)
#define	NDFMCR1_ALS	(0x1 << 8)

#define	NAND_DMAC_STATUS	(0x1 << 5)
#define	NAND_DMAC_CLEAR		(0x1 << 5)

// added for MLC
#define NDFMCR0_ECC_RSM_ON		0x394
#define NDFMCR0_ECC_RSECGW_ON	0x0194
#define NDFMCR0_ECC_RSECGW_OFF	0x094
#define NDFINTC_LATCH_CLEAR		0x99
#define NDFINTC_RSERIS			0x20
#define NDFINTC_RSEIC			0x80
#define INTC_EN					1
#define INTC_DIS				0

/* The hw ecc generator provides a syndrome instead a ecc value on read
 * This can only work if we have the ecc bytes directly behind the
 * data bytes. Applies for DOC and AG-AND Renesas HW Reed Solomon generators */
#define NAND_HWECC_SYNDROME	0x00020000
#define NAND_HWECC_ON 0x00080000 //for MLC

#define ECC_SIZE	40

#endif

