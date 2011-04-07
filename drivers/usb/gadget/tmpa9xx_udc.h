#ifndef tmpa9xx_UDC_H
#define tmpa9xx_UDC_H

/*
 * USB Device Port (UDP) registers.
 * Based on tmpa9xxRM9200 datasheet revision E.
 */

#define   UD2AB_INTSTS		0x000
#define   UD2AB_INTENB           0x004
#define   UD2AB_MWTOUT           0x008
#define   UD2C2STSET             0x00c
#define   UD2AB_UDMSTSET         0x010
#define   UD2AB_DMA_CRDREQ       0x014
#define   UD2AB_DMA_CRDVL        0x018
#define   UD2AB_UDC2RDREQ        0x01c
#define   UD2AB_UDC2RDVL         0x020

#define   UD2AB_ARBT_SET         0x03c 
#define   UD2AB_MWSADR           0x040
#define   UD2AB_MWEADR           0x044
#define   UD2AB_MWCADR           0x048
#define   UD2AB_MWAHBADR         0x04c
#define   UD2AB_MRSADR           0x050
#define   UD2AB_MREADR           0x054
#define   UD2AB_MRCADR           0x058
#define   UD2AB_MRAHBADR         0x05c
#define   UD2AB_PWCTL            0x080
#define   UD2AB_MSTSTS           0x084
#define   UD2AB_TOUTCNT          0x088
#define   UD2AB_TSTSET           0x08c
#define   UD2AB_TSTOUT           0x090

/* ADDRESS ACCESS */            
#define   UD2ADR                 0x200
#define   UD2FRM                 0x204
#define   UD2TMD                 0x208
#define   UD2CMD                 0x20c
#define   UD2BRQ                 0x210
#define   UD2VAL                 0x214
#define   UD2IDX                 0x218
#define   UD2LEN                 0x21c
#define   UD2INT                 0x220
#define   UD2INT_EP              0x224
#define   UD2INT_EP_MASK         0x228
#define   UD2_RX_DATA_0          0x22C

#define   UD2EP0_MaxPacketSize   0x230
#define   UD2EP0_Status          0x234
#define   UD2EP0_DataSize        0x238
#define   UD2EP0_FIFO            0x23c

#define   UD2EP1_MaxPacketSize   0x240
#define   UD2EP1_Status          0x244
#define   UD2EP1_DataSize        0x248
#define   UD2EP1_FIFO            0x24c

#define   UD2EP2_MaxPacketSize   0x250
#define   UD2EP2_Status          0x254
#define   UD2EP2_DataSize        0x258
#define   UD2EP2_FIFO            0x25c

#define   UD2INTNAK              0x330
#define   UD2INTNAKMSK           0x334

/*

 *********************************************************************
 *   MACRO DEFINITIONS
 *********************************************************************
*/
/* PACKET SIZE DEFINISION */
#define EP_MAX_PACKET_SIZE_FS	0x0040
#define EP_MAX_PACKET_SIZE_HS	0x0200

/* MASKDEFINISION */
#define MASK_UCHAR_UPPER_6BIT	0xfc
#define MASK_UINT16_LOWER_8BIT	0x00ff
#define MASK_UINT32_LOWER_16BIT	0x0000ffff
#define SHIFT_8BIT		8
#define WORD_SIZE		2

/* USB DEVICE CONTROLLER HARDWARE */
#define EP_SUPPORT_NO		0x03	/* ENDPOINT0 Support Number for UDC [0-2]*/
#define EP0			0x00	/* Endpoint0*/
#define EP1			0x01	/* Endpoint1*/
#define EP2			0x02	/* Endpoint2*/

/* SPEED CHECK */
#define	CURRENT_SPEED_CHECK	0x3000 	/* UD2ADDRSTATE REG CHECK */
#define	FULL_SPEED		0x1000
#define	HIGH_SPEED		0x2000

/* ATTRIBUTE(bmAttributes) */
#define ATTRIBUTE_CHECK		0xe0
#define SELF_POWERED_BIT	0x40
#define REMOTE_WAKEUP_BIT	0x20

/* 	COMMAND DEFINE for UDC COMMAND REGISTER(16bit).			*/
#define SETUP_FIN		0x0001	/* to set SETUP FIN*/

/* Endpoint reset command.*/
#define EP0_RESET		0x0003	/* ENDPOINT0 RESET*/
#define EP1_RESET		0x0013	/* ENDPOINT1 RESET*/
#define EP2_RESET		0x0023	/* ENDPOINT2 RESET*/

/* Endpoint stall command.*/
#define EP0_STALL		0x0004	/* to set STALL for Endpoint0*/
#define EP1_STALL		0x0014	/* to set STALL for Endpoint1*/
#define EP2_STALL		0x0024	/* to set STALL for Endpoint2*/
#define EP3_STALL		0x0044	/* to set STALL for Endpoint3*/

#define STATUS_NAK_E		0xfdff	/* STATUS_NAK Interrupt Enaable */
#define STATUS_NAK_D		0x0200	/* STATUS_NAK Interrupt Desable */

#define All_EP_INVALID		0x0009	/* to set All ENDPOINT Invalid*/
#define USB_READY		0x000A	/* to set USB Ready*/
#define SETUP_RECEIVED		0x000B	/* to set Setup Received. ENDPOINT-0 Only*/

/* EP EOP command.*/
#define EP0_EOP			0x000C	/* to set ENDPOINT0 EOP*/
#define EP1_EOP			0x001C	/* to set ENDPOINT1 EOP*/
#define EP2_EOP			0x002C	/* to set ENDPOINT2 EOP*/
#define EP3_EOP			0x004C	/* to set ENDPOINT3 EOP*/

/* UDC Stage parameters */
#define IDLE_STAGE		0x00	/* Idle Stage*/
#define SETUP_STAGE		0x01	/* Setup Stage*/
#define DATA_STAGE		0x02	/* Data Stage*/
#define STATUS_STAGE		0x03	/* status Stage*/

/* Define Judgement of Function	*/
#define NORMAL			0x00	/* Nomally End*/
#define ERROR_1			0x01	/* Abnormally End*/
#define ERROR_2			0x02	/* Abnormally End*/

/* UDC State parameters	*/
#define	CURRENT_STATUS_CHECK	0x0f00 	/* */
#define IDLE			0x0000	/* Idle State*/
#define DEFAULT			0x0100	/* Default State*/
#define ADDRESSED		0x0200	/* Address State*/
#define CONFIGURED		0x0400	/* Configured State*/

/* wIndex CHECK */
#define	INDEX_CHECK		0x000f

#define NUM_CONFIG		0x01	/* Maximum configuration index of device*/
#define NUM_CONFIG1_INTERFACE	0x01	/* maximum interface index of config1*/

/* USB INTERRUPT SETTING */
#define INT_ADDRESS_DEFAULT	0x00000000	/* Interrupt Bit */
#define INT_USB_ENABLE		0x00200000	/* Interrupt Bit */
#define INT_USB_DISABLE		0x00200000	/* Interrupt Bit */
#define UDC2_INT_MASK		0x90ff		/* Interrupt Bit */
#define UDC2AB_INT_MASK		0x002407ff	/* Interrupt Bit */
#define UDC2AB_INT_ALL_CLEAR	0x33fe07ff	/* Interrupt Bit */

/*UDC2  INTERRUPT */
#define INT_SETUP		0x0001	/* Interrupt Bit of INT_SETUP*/
#define INT_STATUSNAK		0x0002	/* Interrupt Bit of INT_STATUSNAK*/
#define INT_STATUS		0x0004	/* Interrupt Bit of INT_STATUS*/
#define INT_DATA		0x0008	/* Interrupt Bit of INT_ENDPOINT0*/
#define INT_RX_DATA0		0x0008	/* Interrupt Bit of INT_RX_DATA0*/
#define INT_SOF			0x0010	/* Interrupt Bit of INT_SOF*/
#define INT_EP0			0x0020	/* Interrupt Bit of INT_EP0*/
#define INT_EP			0x0040	/* Interrupt Bit of INT_EP*/
#define INT_NAK			0x0080	/* Interrupt Bit of INT_NAK*/

/*UDC2  INTERRUPT CLEAR*/
#define INT_MASK_VALUE		0x9000	/* Interrupt Bit of INT_SETUP*/
#define INT_SETUP_CLEAR		INT_SETUP | INT_MASK_VALUE	/* Interrupt Bit of INT_SETUP*/
#define INT_STATUSNAK_CLEAR	INT_STATUSNAK | INT_MASK_VALUE	/* Interrupt Bit of INT_STATUSNAK*/
#define INT_STATUS_CLEAR	INT_STATUS | INT_MASK_VALUE	/* Interrupt Bit of INT_STATUS*/
#define INT_DATA_CLEAR		INT_DATA | INT_MASK_VALUE	/* Interrupt Bit of INT_ENDPOINT0*/
#define INT_RX_DATA0_CLEAR	INT_RX_DATA0 | INT_MASK_VALUE	/* Interrupt Bit of INT_RX_DATA0*/
#define INT_SOF_CLEAR		INT_SOF | INT_MASK_VALUE	/* Interrupt Bit of INT_SOF*/
#define INT_EP0_CLEAR		INT_EP0 | INT_MASK_VALUE	/* Interrupt Bit of INT_EP0*/
#define INT_EP_CLEAR		INT_EP | INT_MASK_VALUE		/* Interrupt Bit of INT_EP*/
#define INT_NAK_CLEAR		INT_NAK | INT_MASK_VALUE	/* Interrupt Bit of INT_NAK*/

/*UDC2  INTERRUPT MASK */
#define INT_SETUP_MASK		INT_SETUP << 8
#define INT_STATUSNAK_MASK	INT_STATUSNAK << 8
#define INT_STATUS_MASK		INT_STATUS << 8
#define INT_DATA_MASK		INT_DATA << 8
#define INT_RX_DATA0_MASK	INT_RX_DATA0 << 8
#define INT_SOF_MASK		INT_SOF << 8
#define INT_EP0_MASK		INT_EP0 << 8
#define INT_EP_MASK		INT_EP << 8
#define INT_NAK_MASK		INT_NAK << 8

/*UDC2  AHB INTERRUPT */
#define INT_SUSPEND		0x00000100L	/* Interrupt Bit of INT_SUSPEND*/
#define INT_RESET		0x00000200L	/* Interrupt Bit of INT_RESET_START*/
#define INT_RESET_END		0x00000400L	/* Interrupt Bit of INT_RESET_END*/

#define INT_MW_SET_ADD		0x00020000L	/* Interrupt Bit of INT_EP0*/
#define INT_MW_END_ADD		0x00040000L	/* Interrupt Bit of INT_EP*/
#define INT_MW_TIMEOUT		0x00080000L	/* Interrupt Bit of INT_NAK*/
#define INT_MW_AHBERR		0x00100000L	/* Interrupt Bit of INT_SOF*/
#define INT_MR_END_ADD		0x00200000L	/* Interrupt Bit of INT_EP0*/
#define INT_MR_EP_DSET		0x00400000L	/* Interrupt Bit of INT_EP*/
#define INT_MR_AHBERR		0x00800000L	/* Interrupt Bit of INT_NAK*/
#define INT_UDC2REG_RD		0x01000000L	/* Interrupt Bit of INT_SOF*/
#define INT_DMACREG_RD		0x02000000L	/* Interrupt Bit of INT_EP0*/
#define INT_PW_DETECT		0x10000000L	/* Interrupt Bit of INT_SOF*/
#define INT_MW_RD_ERR		0x20000000L	/* Interrupt Bit of INT_EP0*/
#define UDC2AB_READ_RQ		0x80000000L	/* UDC2RQ 32BIT */
#define UDC2AB_READ_ADDRESS	0x000003fcL	/* UDC2RQ 32BIT */
#define UDC2AB_MR_RESET		0x00000040L
#define UDC2AB_MW_RESET		0x00000004L
#define UDC2AB_MR_ENABLE	0x00000010L	/* MASTER READ ENABLE */
#define UDC2AB_MW_ENABLE	0x00000001L	/* MASTER WRITE ENABLE */
#define UDC2AB_MR_EP_EMPTY	0x00000010L	/* MASTER WRITE ENABLE */

#define UD2INT_EP_EP1		0x0002
#define UD2INT_EP_EP2		0x0004
#define UD2EP12_ENABLE		0xFFF8
#define UD2EP1_ENABLE		0xFFFD
#define UD2EP2_ENABLE		0xFFFB
#define UD2EP_DSET		0x1000
#define UD2EP_DATASIZE_MASK	0x07FF

#define UD2C2STSET_EOP_D	0x00000000L 

#define EP_DUAL_BULK_IN		0xC088
#define EP_DUAL_BULK_OUT	0xC008
#define EP_SINGLE_BULK_IN	0x4088
#define EP_SINGLE_BULK_OUT	0x4008
#define EP_SINGLE_BULK_OUT_C	0x0008

/* status Register Result */
#define STALL			0x0600	/* EP STALL*/
#define STALL_FEATURE		0x0001
#define STALL_FALL_CLEAR	0x00

#define USB_INIT		0x00
#define USB_MASK		0x1a00 /* sof, rx_data0, status_nak dissable */
#define USB_ADDRESS_MAX		0x007f

/* UDC2AB PWCTL SETTING */
#define	PWCTL_PHY_SUSPEND_ON		0x00000008
#define	PWCTL_PHY_SUSPEND_OFF		0x000000f7
#define	PWCTL_PHY_POWER_RESET_ON	0x000000dd
#define	PWCTL_PHY_RESET_OFF		0x00000028			
#define	PWCTL_POWER_RESET_OFF		0x00000002
#define	PWCTL_POWER_RESET_ON		0xfffffffd

#define DMA_READ_IDLE	0
#define DMA_READ_START	1
#define DMA_READ_COMPLETE	2

/*
 * controller driver data structures
 */

#define	NUM_ENDPOINTS	4

/*
 * hardware won't disable bus reset, or resume while the controller
 * is suspended ... watching suspend helps keep the logic symmetric.
 */
#define	MINIMUS_INTERRUPTUS \
	(tmpa9xx_UDP_ENDBUSRES | tmpa9xx_UDP_RXRSM | tmpa9xx_UDP_RXSUSP)

struct tmpa9xx_ep {
	struct usb_ep			ep;
	struct list_head		queue;
	struct tmpa9xx_udc		*udc;
	void __iomem			*creg;

	unsigned			maxpacket:16;
	u8				int_mask;
	unsigned			is_pingpong:1;

	unsigned			stopped:1;
	unsigned			is_in:1;
	unsigned			is_iso:1;
	unsigned			fifo_bank:1;

	const struct usb_endpoint_descriptor
					*desc;
	unsigned			datasize;
};

struct tmpa9xx_udc {
	struct usb_gadget		gadget;
	struct tmpa9xx_ep		ep[NUM_ENDPOINTS];
	struct usb_gadget_driver	*driver;
	unsigned			vbus:1;
	unsigned			enabled:1;
	unsigned			clocked:1;
	unsigned			suspended:1;
	unsigned			req_pending:1;
	unsigned			wait_for_addr_ack:1;
	unsigned			wait_for_config_ack:1;
	unsigned			selfpowered:1;
	unsigned			active_suspend:1;
	unsigned char			addr;
	unsigned char			config_bak;
	unsigned char			interface_bak;
	unsigned char			config;
	unsigned char			interface;
	unsigned int			state_bak;
	unsigned int			state;
	unsigned char			stage;
	struct platform_device		*pdev;
	void __iomem			*udp_baseaddr;
	int				udp_irq;
        int 				dma_ch;
        struct completion  		dma_completion;
    	unsigned char		 	*buf;
	unsigned int 			phy_buf;
	unsigned char 			dma_status;
	struct clk 			*clk;
};

static inline struct tmpa9xx_udc *to_udc(struct usb_gadget *g)
{
	return container_of(g, struct tmpa9xx_udc, gadget);
}

struct tmpa9xx_request {
	struct usb_request		req;
	struct list_head		queue;
};

/*-------------------------------------------------------------------------*/

#ifdef DEBUG
#define DBG(stuff...)		printk(KERN_DEBUG "udc: " stuff)
#else
#define DBG(stuff...)		do{}while(0)
#endif

#ifdef VERBOSE
#    define VDBG		DBG
#else
#    define VDBG(stuff...)	do{}while(0)
#endif

#ifdef PACKET_TRACE
#    define PACKET		VDBG
#else
#    define PACKET(stuff...)	do{}while(0)
#endif

#endif

