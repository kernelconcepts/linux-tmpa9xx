#ifndef TMPA9XX_UDC_H
#define TMPA9XX_UDC_H

#define UD2AB_INTSTS		0x000
#define UD2AB_INTENB		0x004
#define UD2AB_MWTOUT		0x008
#define UD2C2STSET		0x00c
#define UD2AB_UDMSTSET		0x010
#define UD2AB_DMA_CRDREQ	0x014
#define UD2AB_DMA_CRDVL		0x018
#define UD2AB_UDC2RDREQ		0x01c
#define UD2AB_UDC2RDVL		0x020
#define UD2AB_ARBT_SET		0x03c
#define UD2AB_MWSADR		0x040
#define UD2AB_MWEADR		0x044
#define UD2AB_MWCADR		0x048
#define UD2AB_MWAHBADR		0x04c
#define UD2AB_MRSADR		0x050
#define UD2AB_MREADR		0x054
#define UD2AB_MRCADR		0x058
#define UD2AB_MRAHBADR		0x05c
#define UD2AB_PWCTL		0x080
#define UD2AB_MSTSTS		0x084
#define UD2AB_TOUTCNT		0x088
#define UD2AB_TSTSET		0x08c
#define UD2AB_TSTOUT		0x090

#define UD2ADR			0x200
#define UD2FRM			0x204
#define UD2TMD			0x208
#define UD2CMD			0x20c
#define UD2BRQ			0x210
#define UD2VAL			0x214
#define UD2IDX			0x218
#define UD2LEN			0x21c
#define UD2INT			0x220
#define UD2INT_EP		0x224
#define UD2INT_EP_MASK		0x228
#define UD2_RX_DATA_0		0x22C
#define UD2EP0_MaxPacketSize	0x230
#define UD2EP0_Status		0x234
#define UD2EP0_DataSize		0x238
#define UD2EP0_FIFO		0x23c
#define UD2EP1_MaxPacketSize	0x240
#define UD2EP1_Status		0x244
#define UD2EP1_DataSize		0x248
#define UD2EP1_FIFO		0x24c
#define UD2EP2_MaxPacketSize	0x250
#define UD2EP2_Status		0x254
#define UD2EP2_DataSize		0x258
#define UD2EP2_FIFO		0x25c
#define UD2EP3_MaxPacketSize	0x260
#define UD2EP3_Status		0x264
#define UD2EP3_DataSize		0x268
#define UD2EP3_FIFO		0x26c
#define UD2INTNAK		0x330
#define UD2INTNAKMSK		0x334

/* MAX. PACKET SIZE DEFINItION */
#define EP_MAX_PACKET_SIZE_FS	0x0040
#define EP_MAX_PACKET_SIZE_HS	0x0200

#define EP0 (0 << 4)
#define EP1 (1 << 4)
#define EP2 (2 << 4)
#define EP3 (3 << 4)

#define EP_SETUP_FIN		0x1
#define EP_SET_DATA0		0x2
#define EP_RESET		0x3
#define EP_STALL		0x4
#define EP_INVALID		0x5
#define EP_DISABLE		0x7
#define EP_ENABLE		0x8
#define EP_ALL_INVALID		0x9
#define EP_USB_READY		0xa
#define EP_SETUP_RECEIVED	0xb
#define EP_EOP			0xc
#define EP_FIFO_CLEAR		0xd
#define EP_TX_0DATA		0xe

/* sate parameters	*/
#define DEFAULT			0x0100
#define ADDRESSED		0x0200
#define CONFIGURED		0x0400
#define CONFIGURED_DONE		0x0800

/* interrupt */
#define INT_SETUP		0x0001
#define INT_STATUSNAK		0x0002
#define INT_STATUS		0x0004
#define INT_DATA		0x0008
#define INT_RX_DATA0		0x0008
#define INT_SOF			0x0010
#define INT_EP0			0x0020
#define INT_EP			0x0040
#define INT_NAK			0x0080

/* interrupt bits */
#define INT_SUSPEND		0x00000100L
#define INT_RESET		0x00000200L
#define INT_RESET_END		0x00000400L
#define INT_MW_SET_ADD		0x00020000L
#define INT_MW_END_ADD		0x00040000L
#define INT_MW_TIMEOUT		0x00080000L
#define INT_MW_AHBERR		0x00100000L
#define INT_MR_END_ADD		0x00200000L
#define INT_MR_EP_DSET		0x00400000L
#define INT_MR_AHBERR		0x00800000L
#define INT_UDC2REG_RD		0x01000000L
#define INT_DMACREG_RD		0x02000000L
#define INT_PW_DETECT		0x10000000L
#define INT_MW_RD_ERR		0x20000000L
#define UDC2AB_READ_RQ		0x80000000L
#define UDC2AB_READ_ADDRESS	0x000003fcL
#define UDC2AB_MR_RESET		0x00000040L
#define UDC2AB_MW_RESET		0x00000004L
#define UDC2AB_MR_ENABLE	0x00000010L
#define UDC2AB_MW_ENABLE	0x00000001L
#define UDC2AB_MR_EP_EMPTY	0x00000010L

/* pwctl settings */
#define	PWCTL_PHY_SUSPEND_ON		0x00000008
#define	PWCTL_PHY_SUSPEND_OFF		0x000000f7
#define	PWCTL_PHY_POWER_RESET_ON	0x000000dd
#define	PWCTL_PHY_RESET_OFF		0x00000028

#define	PWCTL_POWER_RESET		0x00000002

/* misc (clean me up, please) */
#define EP_DUAL_BULK_IN		0xC088
#define EP_DUAL_BULK_OUT	0xC008
#define FULL_SPEED		0x1000
#define HIGH_SPEED		0x2000

#define	NUM_ENDPOINTS	4

struct tmpa9xx_request {
	struct usb_request req;
	struct list_head queue;
};

struct tmpa9xx_ep {
	struct tmpa9xx_udc *udc;
	const struct usb_endpoint_descriptor *desc;
	struct usb_ep ep;

	struct list_head queue;
	struct tmpa9xx_request *cur;
	spinlock_t s;

	int stopped;
	int ackwait;
	int maxpacket;
	int is_in;
	int is_iso;
};

struct tmpa9xx_udc {
	struct device *dev;
	struct usb_gadget gadget;
	struct usb_gadget_driver	*driver;
	void __iomem *base;
	int irq;
	struct clk *clk;

	struct tmpa9xx_ep ep[NUM_ENDPOINTS];

	struct work_struct ws;
	struct workqueue_struct *wq;
	struct semaphore sem;
	uint16_t ud2int;

	int state;
	unsigned char addr;

	unsigned char *r_buf;
	dma_addr_t phy_r_buf;
	int r_len;

	unsigned char *w_buf;
	dma_addr_t phy_w_buf;
	int w_len;
	int w_pending;

	int ignore_status_nak;
	int ignore_status_ack;
};

static inline struct tmpa9xx_udc *to_udc(struct usb_gadget *g)
{
	return container_of(g, struct tmpa9xx_udc, gadget);
}

#endif

