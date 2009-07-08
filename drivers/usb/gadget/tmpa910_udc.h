/*
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
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef TMPA910_UDC_H
#define TMPA910_UDC_H

/* USB Device Controller Registers */
#define   UD2AB_INTSTS_OFFSET		0x000
#define   UD2AB_INTENB_OFFSET           0x004
#define   UD2AB_MWTOUT_OFFSET           0x008
#define   UD2C2STSET_OFFSET             0x00c
#define   UD2AB_UDMSTSET_OFFSET         0x010
#define   UD2AB_DMA_CRDREQ_OFFSET       0x014
#define   UD2AB_DMA_CRDVL_OFFSET        0x018
#define   UD2AB_UDC2RDREQ_OFFSET        0x01c
#define   UD2AB_UDC2RDVL_OFFSET         0x020

#define   UD2AB_ARBT_SET_OFFSET         0x03c 
#define   UD2AB_MWSADR_OFFSET           0x040
#define   UD2AB_MWEADR_OFFSET           0x044
#define   UD2AB_MWCADR_OFFSET           0x048
#define   UD2AB_MWAHBADR_OFFSET         0x04c
#define   UD2AB_MRSADR_OFFSET           0x050
#define   UD2AB_MREADR_OFFSET           0x054
#define   UD2AB_MRCADR_OFFSET           0x058
#define   UD2AB_MRAHBADR_OFFSET         0x05c
#define   UD2AB_PWCTL_OFFSET            0x080
#define   UD2AB_MSTSTS_OFFSET           0x084
#define   UD2AB_TOUTCNT_OFFSET          0x088
#define   UD2AB_TSTSET_OFFSET           0x08c
#define   UD2AB_TSTOUT_OFFSET           0x090

/* ADDRESS ACCESS */            
#define   UD2ADR_OFFSET                 0x200
#define   UD2FRM_OFFSET                 0x204
#define   UD2TMD_OFFSET                 0x208
#define   UD2CMD_OFFSET                 0x20c
#define   UD2BRQ_OFFSET                 0x210
#define   UD2VAL_OFFSET                 0x214
#define   UD2IDX_OFFSET                 0x218
#define   UD2LEN_OFFSET                 0x21c
#define   UD2INT_OFFSET                 0x220
#define   UD2INT_EP_OFFSET              0x224
#define   UD2INT_EP_MASK_OFFSET         0x228
#define   UD2_RX_DATA_0_OFFSET          0x22C

#define   UD2EP0_MaxPacketSize_OFFSET   0x230
#define   UD2EP0_Status_OFFSET          0x234
#define   UD2EP0_DataSize_OFFSET        0x238
#define   UD2EP0_FIFO_OFFSET            0x23c

#define   UD2EP1_MaxPacketSize_OFFSET   0x240
#define   UD2EP1_Status_OFFSET          0x244
#define   UD2EP1_DataSize_OFFSET        0x248
#define   UD2EP1_FIFO_OFFSET            0x24c

#define   UD2EP2_MaxPacketSize_OFFSET   0x250
#define   UD2EP2_Status_OFFSET          0x254
#define   UD2EP2_DataSize_OFFSET        0x258
#define   UD2EP2_FIFO_OFFSET            0x25c

#define   UD2INTNAK_OFFSET              0x330
#define   UD2INTNAKMSK_OFFSET           0x334

//
#define FCLK_FREQ              		192	     	/* FCLK frequency [MHz] */
#define TIMCLK_FREQ            		FCLK_FREQ/4      /* TIMCLK frequency [MHz] */
#define TIMER1_1MS			TIMCLK_FREQ*1000		
#define	TIMER1_720uS			TIMCLK_FREQ*720
#define	TIMER1_2000CYCLE		2000/4

#define	TIMER1_INITIAL_VALUE		0x00000000L
#define	TIMER1_ENABLE			0x000000c3L
#define	TIMER1_TIMEOUT			0x00000000L

/* SYSTEM CLOCK DEFINE */
#define	SYSCR0_USB_MASK			0x0000003f
#define	SYSCR0_SEL_X1USB		0x00000080
#define	SYSCR0_SEL_X1			0x00000040
#define	USB_ENABLE			0x00000001
#define	SYSCR1_FC			0x00000000
#define	SYSCR1_FC_4			0x00000002
#define	SYSCR2_SEL_PLL 			0x00000002
#define	SYSCR2_LUPFLAG			0x00000001
#define	SYSCR3_PLL_ON			0x00000087
#define	SYSCR3_CLOCK_8			0x00000007
#define	SYSCR4_INIT			0x00000065
#define	DMAC_DISABLE	 		0x00000000

/* debug */
#define	PT7_SEL_X1USB	 		0x00000080

//usb_desc_format.h.h
/*
 *********************************************************************
 *   TYPE DEFINITIONS
 *********************************************************************
 */
/*  DEVICE DESCRIPTOR STRUCTURE*/
struct device_desc{
	unsigned char	bLength;
	unsigned char	bDescripterType;
	unsigned short int bcdUSB;
	unsigned char	bDeviceClass;
	unsigned char	bDeviceSubClass;
	unsigned char	bDeviceProtocol;
	unsigned char	bMaxPacketSize0;
	unsigned short int idVendor;
	unsigned short int idProduct;
	unsigned short int bcdDevice;
	unsigned char	iManufacture;
	unsigned char	iProduct;
	unsigned char	iSerialNumber;
	unsigned char	bNumConfiguration;
};

/* CONFIGURATION DESCRIPTOR STRUCTURE*/
struct  config_desc{
	unsigned char	bLength;
	unsigned char	bDescripterType;
	unsigned short int		wTotalLength;
	unsigned char	bNumInterfaces;
	unsigned char	bConfigurationValue;
	unsigned char	iConfiguration;
	unsigned char	bmAttributes;
	unsigned char	MaxPower;
};
/* STRING DESCRIPTOR STRUCTURE*/
struct  string_desc{
	unsigned char		bLength;
	unsigned char		bDescripterType;
};
/* INTERFACE DESCRIPTOR STRUCTURE*/
struct  interface_desc{
	unsigned char	bLength;
	unsigned char	bDescripterType;
	unsigned char	bInterfaceNumber;
	unsigned char	bAltemateSetting;
	unsigned char	bNumEndPoints;
	unsigned char	bInterfaceClass;
	unsigned char	bInterfaceSubClass;
	unsigned char	bInterfaceProtocol;
	unsigned char	iInterface;
};

/* ENDPOINT DESCRIPTOR STRUCTURE*/
struct  endpoint_desc{
	unsigned char	bLength;
	unsigned char	bDescripterType;
	unsigned char	bEndPointAddress;
	unsigned char	bmAttributes;
	unsigned short int wMaxPacketSize;
	unsigned char	bInterval;
	};

/* USB.2.0 Specification*/
/* DEVICE QUALIFIER DESCRIPTOR STRUCTURE*/
struct device_qualifier{
	unsigned char	bLength;
	unsigned char	bDescriptorType;
	unsigned short int bcdUSB;
	unsigned char	bDeviceClass;
	unsigned char	bDeviceSubClass;
	unsigned char	bDeviceProtocol;
	unsigned char	bMaxPacketSize0;
	unsigned char	bNumConfigurations;
	unsigned char	bReserved;
};

/* OTHER SPEED CONFIGRATION DESCRIPTOR STRUCTURE*/
struct other_speed_config{
	unsigned char	bLength;
	unsigned char	bDescriptorType;
	unsigned short int wTotalLength;
	unsigned char	bNumInterfaces;
	unsigned char	bConfigurationValue;
	unsigned char	iConfiguraion;
	unsigned char	bmAttributes;
	unsigned char	bMaxPower;
};

//usb_desc_format.h
/*
 *********************************************************************
 *   MACRO DEFINITIONS
 *********************************************************************
*/
/* PACKET SIZE DEFINISION */
#define EP_MAX_PACKET_SIZE_FS	0x0040
#define EP_MAX_PACKET_SIZE_HS	0x0200
#define BUFSIZE			(EP_MAX_PACKET_SIZE_HS*4)
#define RINGBUFSIZE		(EP_MAX_PACKET_SIZE_HS*4)

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

/* STANDARD REQUEST */
#define	RQ_GET_STATUS		0x00 	/* Request Type = Get_Status()*/
#define	RQ_CLEAR_FEATURE	0x01 	/* Request Type = Clear_Feature()*/
#define	RQ_SET_FEATURE		0x03 	/* Request Type = Set_Feature()*/
#define	RQ_SET_ADDRESS		0x05 	/* managed by Hardware*/
#define	RQ_GET_DESCRIPTOR	0x06 	/* Request Type = Get_Descriptor()*/
#define	RQ_SET_DESCRIPTOR	0x07 	/* Request Type = Set_Descriptor()*/
#define	RQ_GET_CONFIGURATION	0x08 	/* Request Type = Get_Configuration()*/
#define	RQ_SET_CONFIGURATION	0x09 	/* Request Type = Set_Configuration()*/
#define	RQ_GET_INTERFACE	0x0A 	/* Request Type = Get_Interface()*/
#define	RQ_SET_INTERFACE	0x0B 	/* Request Type = Set_Interface()*/
#define	RQ_SYNCH_FRAME		0x0C 	/* Request Type = Synch Frame()*/

/* DESCRIPTOR TYPE */
#define TYPE_DEVICE		0x01	/* Recipient in Get_Descriptor = Device*/
#define TYPE_CONFIG		0x02	/* Recipient in Get_Descriptor = Config*/
#define TYPE_STRING		0x03	/* Recipient in Get_Descriptor = String*/
#define TYPE_DEVICE_QUALIFIER 	0x06
#define TYPE_OTHER_SPEED 	0x07	/* Recipient in Get_Descriptor = OTHER SPEED*/

/* SPEED CHECK */
#define	CURRENT_SPEED_CHECK	0x3000 	/* UD2ADDRSTATE REG CHECK */
#define	FULL_SPEED		0x1000
#define	HIGH_SPEED		0x2000

/* DIRECTON (bmRequestType:7bit) */
#define	DIRECTION_TYPE_CHECK	0x80	/* Reruest Direction */
#define	REQ_GET			0x80	/* Reruest Direction */
#define	REQ_SET			0x00	/* Reruest Direction */
#define	SET			0	/* Type of Set Request(bit field) */
#define	GET			1	/* Type of Get Request(bit field) */

/* TYPE (bmRequestType:5-6bit)*/
#define REQUEST_TYPE_CHECK	0x60	/* Request Type Mask*/
#define STANDARD_RQ		0x00	/* Standard Request*/
#define	USBCLASS_RQ		0x20	/* Class request*/
#define	VENDOR_RQ		0x40	/* Vendor Request*/

/* RECEIVE TYPE (bmRequestType:0-4bit) */
#define RECEIVE_TYPE_CHECK	0x1f
#define RQ_DEVICE		0x00	/* Recipient=Device*/
#define RQ_INTERFACE		0x01	/* Recipient=Interface*/
#define RQ_ENDPOINT		0x02	/* Recipient=Endpoint*/
#define RECEIVE_ERROR		0x03	/* Error Request Type*/
	
#define CONFIG_DESC_TYPE					1
#define CONFIG_DESC_ATTRIBUTE	7
#define CONFIG_EP1_SIZE_LOW	22
#define CONFIG_EP1_SIZE_HIGH	23
#define CONFIG_EP2_SIZE_LOW	29
#define CONFIG_EP2_SIZE_HIGH	30

#define STRING_DESC_INIT	0xff
#define STRING_DESC_INDEX_0	0
#define STRING_DESC_INDEX_1	1
#define STRING_DESC_INDEX_2	2
#define STRING_DESC_INDEX_3	3

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

#define STATUS_NAK_E		0xfdff	/* STATUS_NAK Interrupt Enaable */
#define STATUS_NAK_D		0x0200	/* STATUS_NAK Interrupt Desable */

#define All_EP_INVALID		0x0009	/* to set All ENDPOINT Invalid*/
#define USB_READY		0x000A	/* to set USB Ready*/
#define SETUP_RECEIVED		0x000B	/* to set Setup Received. ENDPOINT-0 Only*/

/* EP EOP command.*/
#define EP0_EOP			0x000C	/* to set ENDPOINT0 EOP*/
#define EP1_EOP			0x001C	/* to set ENDPOINT1 EOP*/
#define EP2_EOP			0x002C	/* to set ENDPOINT2 EOP*/

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

/*		      - DESCRIPTOR INFORMATION  -			*/

/* CONFIG DESCRIPTOR INFOMATION */
#define TOTAL_STRING_DESC	0x58	/* Total length of String Descriptor*/
#define STR0_ADDRESS		0x00	/* Start address of String0 descriptor*/
#define STR1_ADDRESS		0x11	/* Start address of String1 descriptor*/
#define STR2_ADDRESS		0x14	/* Start address of String2 descriptor*/
#define NUM_CONFIG		0x01	/* Maximum configuration index of device*/
#define NUM_TOTAL_ENDPOINTS	0x04	/* maximum endpoint index of config1*/
#define NUM_BREQRUEST_MAX	0x0100	/* maximum value of bRequest */
#define NUM_CONFIG1_INTERFACE	0x01	/* maximum interface index of config1*/
#define NUM_C1IO_ALT0		0x00	/* Config1-Interface0-AlternateSettig0*/
#define OTHER_SPEED_ADD		0x40	/* Start address of String3 descriptor*/

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

/* DESCRIPTOR SIZE*/
#define	WLENGTH_MAX		2
#define	DEVICE_DESC_SIZE  	18
#define	CONFIG_DESC_SIZE  	32
#define	QUALFIER_DESC_SIZE 	10

/* UDC2AB PWCTL SETTING */
#define	PWCTL_PHY_SUSPEND_ON		0x00000008
#define	PWCTL_PHY_SUSPEND_OFF		0x000000f7
#define	PWCTL_PHY_POWER_RESET_ON	0x000000dd
#define	PWCTL_PHY_RESET_OFF		0x00000028			
#define	PWCTL_POWER_RESET_OFF		0x00000002

/*
 *********************************************************************
 *   TYPE DEFINITIONS
 *********************************************************************
 */
/* FLAG INFORMATION */
#define	FLAG_OFF			0
#define	FLAG_ON				1
typedef struct byte_field {
	unsigned short int bitF:1;
	unsigned short int bitE:1;
	unsigned short int bitD:1;
	unsigned short int bitC:1;
	unsigned short int bitB:1;
	unsigned short int bitA:1;
	unsigned short int bit9:1;
	unsigned short int bit8:1;

	unsigned short int bit7:1;
	unsigned short int bit6:1;
	unsigned short int bit5:1;
	unsigned short int bit4:1;
	unsigned short int bit3:1;
	unsigned short int bit2:1;
	unsigned short int bit1:1;
	unsigned short int bit0:1;
} Byte_Field;

typedef union _byte_io {
	unsigned short int	byte;
	Byte_Field bit;
} FlagByte;

//usb_vender_class.h
/*
 *********************************************************************
 *   MACRO DEFINITIONS
 *********************************************************************
 */
/* DRIVER STATUS	*/
#define	USB_STS_IDOL			0
#define	USB_STS_SUSPEND			1
#define	USB_STS_RESUME			2

/* BULK IN&OUT MODE	*/
#define	BULKINMODE_USB			0
#define	BULKINMODE_DMA			1
#define	BULKOUTMODE_USB			0
#define	BULKOUTMODE_DMA			1

/* g_USB_Stage ERROR	*/
#define	STAGE_NORMAL			0x00	/* Normal operation	*/
#define	STAGE_ERROR			0x01	/* Stage Error 		*/

/* REQUEST PROCESSING	*/
#define	ERR_REQUEST			0x02

/* RECEIVE RING BUFFER STATUS	*/
#define	RRBST_NORMAL			0x00	/* Normal STATUS	*/
#define	RRBST_FULL			0x01	/* FULL STATUS		*/

/* USB DOWNLOAD STATUS	*/
#define	USBDPG_IDOL			0x00
#define	USBDPG_MINF			0x01
#define	USBDPG_TSTART			0x02
#define	USBDPG_TRESULT			0x03
#define	USBDPG_TRESULT_DMA		0x04
#define	USBDPG_END			0x05
#define		USBDPG_E_NOR		0x00
#define		USBDPG_E_NOT		0x02
#define		USBDPG_E_FERR		0x04
#define		USBDPG_E_SOVER		0x06
#define		USBDPG_E_AERR		0x08
#define		USBDPG_E_PERR		0x0A

/* KIND VENDOR COMMAND	*/
#define		REQ_VEN_MINF		0x00
#define		MINF_SIZE		0x0F
#define		MINF_ADDRESS		0x02
#define		REQ_VEN_TSTART		0x02
#define		REQ_VEN_TRESULT		0x04

/* RAM AREA DEFINE	*/
#define	USB_DLTOP			0xF8002000L
#define	USB_DLBTM			0xF800DFFFL

/* BULK PROCESSING DEFINE	*/
#define	BULK_OUT_SUCCESS		0x00	/* Bulk-out processing success	*/
#define	BULK_OUT_ERROR			0xFF	/* Bulk-out processing error	*/
#define	BULK_IN_SUCCESS			0x00	/* Bulk-in processing success	*/
#define	BULK_IN_ERROR			0xFF	/* Bulk-in processing error		*/

/* S RECORD DECODE STATE */
#define		STATE0			0
#define		STATE1			1
#define		STATE2			2
#define		STATE3			3
#define		STATE4			4
#define		STATE5			5
#define		NORMAL_RECORD		3
#define		END_RECORD		7
#define		ADDRESS_GET		4
#define		SUM_RESULT		0xff

/*
 *********************************************************************
 *   EXTERNAL FUNCTION DECLARATION
 *********************************************************************
 */

//usb_ram.h
/*
 *********************************************************************
 *   EXTERNAL VARIABLE DECLARATION
 *********************************************************************
 */
extern unsigned char		g_USB_Send_Buf[BUFSIZE];					/* Send Buffer */
extern unsigned char		g_USB_Recv_Buf[BUFSIZE];					/* Receive Buffer */
extern unsigned char		g_USB_Recv_Rbuf[RINGBUFSIZE];

extern unsigned char		*g_Recv_Rbuf_Wpt;
extern unsigned char		*g_Recv_Rbuf_Rpt;
extern unsigned char		g_USB_Rbuf_Status;
extern unsigned char		g_USB_Status;
extern unsigned char		g_USB_Bulk_In_Mode;
extern unsigned char		g_USB_Bulk_Out_Mode;

/* For INTERRUPT CHECK*/
extern unsigned long int		g_Interrupt_Stasus; 

/* For ENDPOINT0	*/
extern unsigned char		g_EP0_Recv_Buff[EP_MAX_PACKET_SIZE_FS];		/* EP0 receive buffer */
extern unsigned char		g_EP0_Send_Buff[EP_MAX_PACKET_SIZE_FS];		/* EP0 sending buffer */
extern unsigned char		g_USB_Stage;
extern unsigned char		g_USB_Stage_Error;
extern unsigned char		g_EP0_Recv_Length;

/* For BULK MODE	*/
extern unsigned char		*g_DMA_Send_Buff_Address;	
extern unsigned char		*g_DMA_Recv_Buff_Address;
extern unsigned long int		g_DMA_Send_Length;
extern unsigned long int		g_DMA_Recv_Length;
extern unsigned short int		g_Bulk_Out_EP_Size;

/* For USB MAIN	*/
extern unsigned long int		g_USB_Dpg_Adress;

extern unsigned short int		g_USB_Dpg_Size;
extern unsigned short int		g_USB_Dpg_Size_cnt;
extern unsigned char		g_USB_Dpg_Stt;
extern unsigned char		g_USB_Dpg_Error_Stt;

/* Descriptor Infomation */
extern unsigned short int		g_EP_PAYLOAD_SIZE[EP_SUPPORT_NO];
extern unsigned char		g_Num_StringDesc;
extern unsigned short int		g_USB_Address;

/* Request Parameter of Setup Data */
extern unsigned char		g_bmRequestType;
extern unsigned char		g_bRequest;
extern unsigned short int		g_wValue;
extern unsigned short int		g_wIndex;
extern unsigned short int		g_wLength;

/* UDC State parameter */
extern unsigned short int		g_Current_State;
extern unsigned char		g_Current_Config;
extern unsigned char		g_Current_Interface;
extern unsigned char		g_Current_Alternate;
extern unsigned short int		g_Buf_Current_State;
extern unsigned char		g_Buf_Current_Config;
extern unsigned char		g_Buf_Current_Interface;
extern unsigned char		g_Buf_Current_Alternate;
extern unsigned char		g_Self_Powered_Bit;

/* Endpoint Fifo Access */
extern unsigned short int		g_Remain_TotalLength;
extern unsigned short int		g_Expected_Length;
extern unsigned long int		g_Start_Address;


extern const unsigned char Config_Desc[CONFIG_DESC_SIZE];
extern const unsigned char Dev_Desc[DEVICE_DESC_SIZE];
extern const unsigned char Qualifier_Desc[QUALFIER_DESC_SIZE];
extern const unsigned char Str_Desc_ROM[TOTAL_STRING_DESC];				/* String Descriptor Data */

/* FLAG DEFINE */
extern FlagByte			Dev_Status;
#define Dev_PowerStatus		Dev_Status.byte
#define	fSelf_Powered		Dev_Status.bit.bit0
#define	fRemote_Wakeup		Dev_Status.bit.bit1
#define	fDirection		Dev_Status.bit.bit2
#define	fErrorTrasfer		Dev_Status.bit.bit3
#define fBODirection		Dev_Status.bit.bit4

/* DEFINE FEATURE FLAG. */
extern FlagByte			EP_ST;
#define ST_Feature		EP_ST.byte
#define	fEP0_Stall_Feature	EP_ST.bit.bit0
#define	fEP1_Stall_Feature	EP_ST.bit.bit1
#define	fEP2_Stall_Feature	EP_ST.bit.bit2
#define	fEP3_Stall_Feature	EP_ST.bit.bit3

/*
 * controller driver data structures
 */

#define	NUM_ENDPOINTS	4

/*
 * hardware won't disable bus reset, or resume while the controller
 * is suspended ... watching suspend helps keep the logic symmetric.
 */
#define	MINIMUS_INTERRUPTUS \
	(AT91_UDP_ENDBUSRES | AT91_UDP_RXRSM | AT91_UDP_RXSUSP)

struct tmpa910_ep {
	struct usb_ep			ep;
	struct list_head		queue;
	struct tmpa910_udc		*udc;
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
};

/*
 * driver is non-SMP, and just blocks IRQs whenever it needs
 * access protection for chip registers or driver state
 */
struct tmpa910_udc {
	struct usb_gadget		gadget;
	struct tmpa910_ep			ep[NUM_ENDPOINTS];
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
	u8				addr;
//	struct tmpa910_udc_data		board;
//	struct clk			*iclk, *fclk;
	struct platform_device		*pdev;
	struct proc_dir_entry		*pde;
	void __iomem			*udp_baseaddr;
	int				udp_irq;
};

static inline struct tmpa910_udc *to_udc(struct usb_gadget *g)
{
	return container_of(g, struct tmpa910_udc, gadget);
}

struct tmpa910_request {
	struct usb_request		req;
	struct list_head		queue;
};

/*-------------------------------------------------------------------------*/

#define DEBUG
#ifdef DEBUG
#define DBG(stuff...)		printk("udc: " stuff)
#else
#define DBG(stuff...)		do{}while(0)
#endif

#ifdef VERBOSE
#    define VERBOSED		DBG
#else
#    define VDBG(stuff...)	do{}while(0)
#endif

#ifdef PACKET_TRACE
#    define PACKET		VDBG
#else
#    define PACKET(stuff...)	do{}while(0)
#endif

#define ERR(stuff...)		printk(KERN_ERR "udc: " stuff)
#define WARN(stuff...)		printk(KERN_WARNING "udc: " stuff)
#define INFO(stuff...)		printk(KERN_INFO "udc: " stuff)

#endif


