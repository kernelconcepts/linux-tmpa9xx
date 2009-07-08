/*
 * tmpa910_udc -- driver for tmpa910-series USB peripheral controller
 *
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
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA  02111-1307, USA.
 */

//#undef	DEBUG
#define	DEBUG
//#undef	VERBOSE
#define VERBOSE
#undef	PACKET_TRACE

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <mach/tmpa910_regs.h>

#include "tmpa910_udc.h"


const unsigned char Dev_Desc[DEVICE_DESC_SIZE] = {
	0x12, 	/* bLength*/
	0x01, 	/* bDescriptorType*/
	0x00, 	/* bcdUSB(Low) */
	0x02, 	/* bcdUSB(High) */
	0x00, 	/* bDeviceClass*/
	0x00, 	/* bDeviceSubClass*/
	0x00, 	/* bDeviceProtocol*/
	0x40, 	/* bMaxPacketSize0*/
	0x30, 	/* idVendor(Low) */
	0x09, 	/* idVendor(High) */
	0x05, 	/* idProduct(Low) */
	0x65, 	/* idProduct(High) */
	0x00, 	/* bcdDevice(Low) */
	0x01, 	/* bcdDevice(High) */
	0x00, 	/* iManufacturer*/
	0x00, 	/* iProduct*/
	0x00, 	/* iSerialNumber*/
	0x01	/* bNumConfigrations*/
};

//const unsigned char __align(4) Config_Desc[CONFIG_DESC_SIZE] = {
const unsigned char Config_Desc[CONFIG_DESC_SIZE] = {
/* Configuration Descriptor*/
	0x09, 	/* bLength*/
	0x02, 	/* bDescriptorType*/
	0x20, 	/* wTotalLength(Low) */
	0x00, 	/* wTotalLength(High) */
	0x01, 	/* bNumInterfaces*/
	0x01, 	/* bConfigurationValue */
	0x00, 	/* iConfiguration */
	0x80, 	/* bmAttributes */
	0x41, 	/* bMaxPower*/

/* Interface Descriptor*/
	0x09, 	/* bLength*/
	0x04, 	/* bDescriptorType*/
	0x00, 	/* bInterfaceNumber*/
	0x00, 	/* bAlternateSetting*/
	0x02, 	/* bNumEndpoints*/
	0x08, 	/* bInterfaceClass*/
	0x06, 	/* bInterfaceSubClass*/
	0x50, 	/* bInterfaceProtocol*/
	0x00, 	/* iInterface*/

/* EndPoint[1] Descriptor*/
	0x07, 	/* bLength*/
	0x05, 	/* bDescriptorType*/
	0x81, 	/* bEndPointAddress*/
	0x02, 	/* bmAttributes*/
	0x00, 	/* wMaxPacketSize(Low) */
	0x02, 	/* wMaxPacketSize(High) */
	0x01, 	/* bInterval*/

/* EndPoint[2] Descriptor*/
	0x07, 	/* bLength*/
	0x05, 	/* bDescriptorType*/
	0x02, 	/* bEndPointAddress*/
	0x02, 	/* bmAttributes*/
	0x00, 	/* wMaxPacketSize(Low) */
	0x02, 	/* wMaxPacketSize(High) */
	0x01	/* bInterval*/
};

//const unsigned char __align(4) Qualifier_Desc[QUALFIER_DESC_SIZE] = {
const unsigned char Qualifier_Desc[QUALFIER_DESC_SIZE] = {
	0x0A, 	/*bLength, (10byte) */
	0x06, 	/*bDescriptorType */
	0x00, 	/* bcdUSB(Low) */
	0x02, 	/* bcdUSB(High)*/
	0x00, 	/* bDeviceClass */
	0x00, 	/* bDeviceSubClass */
	0x00, 	/* bDeviceProtocol*/
	0x40, 	/* bMaxPacketSize0(64byte) */
	0x01, 	/* bNumConfiguration(2Configurations) */
	0x00, 	/* Reserve*/
};

//const unsigned char	__align(4) Str_Desc_ROM[TOTAL_STRING_DESC] = {
const unsigned char	Str_Desc_ROM[TOTAL_STRING_DESC] = {
	/* STRING DESCRIPTOR(Micro controller)	*/
	0x11		/* Size of Descriptor(byte)				*/
	, 0x03		/* Descriptor Type						*/
	, 0x54		/* 'T'									*/
	, 0x4d		/* 'M'									*/
	, 0x50		/* 'P'									*/
	, 0x41		/* 'A'									*/
	, 0x39		/* '9'									*/
	, 0x31		/* '1'									*/
	, 0x30		/* '0'									*/
	, 0x43		/* 'C'									*/
	, 0x52		/* 'R'									*/
	, 0x20		/* ''									*/
	, 0x20		/* ''									*/
	, 0x20		/* ''									*/
	, 0x20		/* ''									*/
	, 0x20		/* ''									*/
	, 0x20		/* ''									*/

	/* STRING DESCRIPTOR(Manufacturer)		*/
	, 0x03		/* Size of Descriptor(byte)				*/
	, 0x03		/* Descriptor Type						*/
	, 0x00		/* String Descriptor Infomation			*/

	/* STRING DESCRIPTOR(Product)			*/
	, 0x03		/* Size of Descriptor(byte)				*/
	, 0x03		/* Descriptor Type						*/
	, 0x00		/* String Descriptor Infomation			*/

	/* STRING DESCRIPTOR(Serial Number)		*/
	, 0x03		/* Size of Descriptor(byte)				*/
	, 0x03		/* Descriptor Type						*/
	, 0x00		/* String Descriptor Infomation			*/

};

/*
 * This controller is
 *
 * This driver expects the board has been wired with two GPIOs suppporting
 * a VBUS sensing IRQ, and a D+ pullup.  (They may be omitted, but the
 * testing hasn't covered such cases.)
 *
 * The pullup is most important (so it's integrated on sam926x parts).  It
 * provides software control over whether the host enumerates the device.
 *
 * The VBUS sensing helps during enumeration, and allows both USB clocks
 * (and the transceiver) to stay gated off until they're necessary, saving
 * power.  During USB suspend, the 48 MHz clock is gated off in hardware;
 * it may also be gated off by software during some Linux sleep states.
 */

#define DRIVER_VERSION  "26 July 2008"

static const char driver_name [] = "tmpa910-usb";
static const char ep0name[] = "ep0";


#define tmpa910_udp_read(dev, reg) \
        __raw_readl((dev)->udp_baseaddr + (reg))
#define tmpa910_udp_write(dev, reg, val) \
        __raw_writel((val), (dev)->udp_baseaddr + (reg))

static const struct usb_ep_ops tmpa910_ep_ops = {
        .enable         = NULL,		//tmpa910_ep_enable,
        .disable        = NULL,         //tmpa910_ep_disable,
        .alloc_request  = NULL,         //tmpa910_ep_alloc_request, 
        .free_request   = NULL,         //tmpa910_ep_free_request,
        //.alloc_buffer   = NULL,         //tmpa910_ep_alloc_buffer,
        //.free_buffer    = NULL,         //tmpa910_ep_free_buffer,
        .queue          = NULL,         //tmpa910_ep_queue,
        .dequeue        = NULL,         //tmpa910_ep_dequeue,
        .set_halt       = NULL,         //tmpa910_ep_set_halt,
        // there's only imprecise fifo status reporting
};              

static const struct usb_gadget_ops tmpa910_udc_ops = {
        .get_frame              = NULL,         //tmpa910_get_frame,
        .wakeup                 = NULL,         //tmpa910_wakeup,
        .set_selfpowered        = NULL,         //tmpa910_set_selfpowered,
        .vbus_session           = NULL,         //tmpa910_vbus_session,
        .pullup                 = NULL,         //tmpa910_pullup,
};

static void nop_release(struct device *dev){}

static struct tmpa910_udc controller = {
        .gadget = {
                .ops    = &tmpa910_udc_ops,
                .ep0    = &controller.ep[0].ep,
                .name   = driver_name,
                .dev    = {
                        .release = nop_release,
                }       
        },      
        .ep[0] = {
                .ep = {
                        .name   = ep0name,
                        .ops    = &tmpa910_ep_ops,
                },      
                .udc            = &controller,
                .maxpacket      = 64,
                .int_mask       = 1 << 0,
        },
        .ep[1] = {
                .ep = {
                        .name   = "ep1",
                        .ops    = &tmpa910_ep_ops,
                },
                .udc            = &controller,
                .is_pingpong    = 1,
                .maxpacket      = 512,
                .int_mask       = 1 << 1,
        },
        .ep[2] = {
                .ep = {
                        .name   = "ep2",
                        .ops    = &tmpa910_ep_ops,
                },
                .udc            = &controller,
                .is_pingpong    = 1,
                .maxpacket      = 512,
                .int_mask       = 1 << 2,
	},
};

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_USB_GADGET_DEBUG_FILES

#include <linux/seq_file.h>

static const char debug_filename[] = "driver/udc";

#define FOURBITS "%s%s%s%s"
#define EIGHTBITS FOURBITS FOURBITS
#endif

#define CLKCR4          __REG(0xf0050050)
/*
 *********************************************************************
 *   VARIABLE DEFINITIONS
 *********************************************************************
 */

/*==========================*/
/* Table Definitions		*/
/*==========================*/
//unsigned char	__align(4)	g_USB_Send_Buf[BUFSIZE];	/* Send Buffer */
//unsigned char	__align(4)	g_USB_Recv_Buf[BUFSIZE];	/* Receive Buffer */
//unsigned char	__align(4)	g_USB_Recv_Rbuf[RINGBUFSIZE];

unsigned char	g_USB_Send_Buf[BUFSIZE];			/* Send Buffer */
unsigned char	g_USB_Recv_Buf[BUFSIZE];			/* Receive Buffer */
unsigned char	g_USB_Recv_Rbuf[RINGBUFSIZE];

unsigned char		*g_Recv_Rbuf_Wpt;
unsigned char		*g_Recv_Rbuf_Rpt;
unsigned char		g_USB_Rbuf_Status;			/* USB RING BUFFER status */

unsigned char		g_USB_Status;				/* USB Driver status */
unsigned char		g_USB_Bulk_In_Mode;			/* Bulk-in mode	*/
unsigned char		g_USB_Bulk_Out_Mode;			/* Bulk-out mode */

/* For INTERRUPT CHECK*/
unsigned long int	g_Interrupt_Stasus; 

/* For ENDPOINT0	*/
//unsigned char	__align(4)	g_EP0_Recv_Buff[EP_MAX_PACKET_SIZE_FS];		/* EP0庴怣僶僢僼傽 */
//unsigned char	__align(4)	g_EP0_Send_Buff[EP_MAX_PACKET_SIZE_FS];		/* EP0憲怣僶僢僼傽 */
unsigned char	g_EP0_Recv_Buff[EP_MAX_PACKET_SIZE_FS];		/* EP0庴怣僶僢僼傽 */
unsigned char	g_EP0_Send_Buff[EP_MAX_PACKET_SIZE_FS];		/* EP0憲怣僶僢僼傽 */
unsigned char		g_USB_Stage;				/* 僗僥乕僕忬懺 */
unsigned char		g_USB_Stage_Error;			/* 僗僥乕僕僄儔乕僼儔僌 */
unsigned char		g_EP0_Recv_Length;			/* EP0庴怣僨乕僞挿 */

/* For BULK MODE	*/
unsigned char		*g_DMA_Send_Buff_Address;		/* 憲怣僨乕僞奿擺愭傾僪儗僗	*/
unsigned char		*g_DMA_Recv_Buff_Address;		/* 庴怣僨乕僞奿擺愭傾僪儗僗	*/
unsigned long int	g_DMA_Send_Length;			/* 巆憲怣僨乕僞挿 */
unsigned long int	g_DMA_Recv_Length;			/* 巆庴怣僨乕僞挿 */
unsigned short int	g_Bulk_Out_EP_Size;			/* DMA庴怣僨乕僞挿 */

/* Descriptor Infomation */
//unsigned short int	__align(4)	g_EP_PAYLOAD_SIZE[EP_SUPPORT_NO];
unsigned short int	g_EP_PAYLOAD_SIZE[EP_SUPPORT_NO];
unsigned char		g_Num_StringDesc;
unsigned short int	g_USB_Address;

/* Request Parameter of Setup Data */
unsigned char		g_bmRequestType;
unsigned char		g_bRequest;
unsigned short int	g_wValue;
unsigned short int	g_wIndex;
unsigned short int	g_wLength;

/* UDC State parameter */
unsigned short int	g_Current_State;
unsigned char		g_Current_Config;
unsigned char		g_Current_Interface;
unsigned char		g_Current_Alternate;

unsigned short int	g_Buf_Current_State;
unsigned char		g_Buf_Current_Config;
unsigned char		g_Buf_Current_Interface;
unsigned char		g_Buf_Current_Alternate;

unsigned char		g_Self_Powered_Bit;

/* Endpoint Fifo Access */
unsigned short int	g_Remain_TotalLength;
unsigned short int	g_Expected_Length;
unsigned long int	g_Start_Address;

/* FLAG DEFINE */
FlagByte		Dev_Status;
FlagByte		EP_ST;

/*
 *********************************************************************
 *   FUNCTION DECLARATIONS
 *********************************************************************
 */
static short int  	USB_Receive_Request(void);		/* Function of Request type Judegement */
static short int 	Rq_Clear_Feature(void);		/* Function of Clear_Feature Request management */
static short int 	Rq_Set_Feature(void);		/* Function of Set_Feature Request management */
static short int 	Rq_Set_Address(void);		/* Function of Set_Address Request management */
static short int 	Rq_Get_Status(void);		/* Function of Get_Status Request management */
static short int 	Rq_Set_Configuration(void);	/* Function of Set_Configuration Request management */
static short int 	Rq_Get_Configuration(void);	/* Function of Get_Configuration Request management */
static short int 	Rq_Set_Interface(void);		/* Function of Set_Interface Request management */
static short int 	Rq_Get_Interface(void);		/* Function of Get_Interface Request management */
static short int 	Rq_Get_Descriptor(void);	/* Function of Get_Descriptor Request management */
static void EP0_FIFO_Read(void);	/* Function of reading data from Endpointo's FIFO */
static void EP0_FIFO_Write(void); 	/* Function of writing data to Endpoint0's FIFO */

static void USB_Bulk_Out(void);
static void USB_Bulk_In(void);
static unsigned short int USB_Send_Length_Chk(const unsigned long int Length);
static unsigned short int USB_Recv_Length_Chk(void);
static void USB_Bulk_Out_Start(void);

static void UDC2_Reg_Read(const unsigned long int, unsigned short int*);
static void UDC2_Reg_Write(const unsigned long int, const unsigned short int);

static void USB_Ctl_Init(void);
static short int USB_Receive_Request(void);
static unsigned char Rbuf_Stchk(void);
static void MemcpyToRecvBufWithRewind(void **pdest, const void *psrc, unsigned long int cnt);
//static void MemcpyFromRecvBufWithRewind(void *pdest, const void **prbf_rpt, unsigned long int cnt);
static void USB_Int_Setup(void);
static void USB_Int_Rx_Zero(void);
static void USB_Int_Status(void);
static void USB_Int_Status_NAK(void);
static void USB_Int_EP0(void);
static void USB_Int_EPx(void);
static void USB_Int_SOF(void);
static void USB_Int_Supend_Resume(void);
static void USB_Int_USB_Reset_End(void);
static void USB_Int_USB_Reset(void);
static void USB_Int_MW_End(void);
static void USB_Int_MR_End(void);
//char usb_bulkin_mass_storage(void);
//char usb_bulkout_mass_storage(void);

/*
 *********************************************************************
 *   FUNCTION DEFINITIONS
 *********************************************************************
 */
/* DEFINE STANDARD DEVICE REQUEST FUNCTIONS */

/*
 *********************************************************************
 * NAME  :USB_Receive_Request
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : status
 * DESCRIPTION  : 僙僢僩傾僢僾僐儅儞僪庴怣屻丄偙偺娭悢偵偰僨僶僀僗梫媮傪夝愅
 *
 *********************************************************************
 */
static short int USB_Receive_Request(void)
{
	short int		status;
	unsigned short int	request_reg;
	
	status = 1;
	UDC2_Reg_Read(UD2BRQ_OFFSET, &request_reg);

	g_bmRequestType = (unsigned char) (request_reg & MASK_UINT16_LOWER_8BIT);
	g_bRequest = (unsigned char) ((request_reg >> SHIFT_8BIT) & MASK_UINT16_LOWER_8BIT);

	UDC2_Reg_Read(UD2VAL_OFFSET, &g_wValue);
	UDC2_Reg_Read(UD2IDX_OFFSET, &g_wIndex);
	UDC2_Reg_Read(UD2LEN_OFFSET, &g_wLength);

	if( (g_bmRequestType & DIRECTION_TYPE_CHECK) == REQ_GET ){
		fDirection = GET;
	}
	else{
		fDirection = SET;
	}

	UDC2_Reg_Write(UD2CMD_OFFSET, SETUP_RECEIVED);			/* Recieved Device Request. */

	if( (g_bmRequestType & REQUEST_TYPE_CHECK) == STANDARD_RQ ){
		switch( g_bRequest ){
		case RQ_GET_STATUS:
			status = Rq_Get_Status();
			break;
		case RQ_CLEAR_FEATURE:
			status = Rq_Clear_Feature();
			break;
		case RQ_SET_FEATURE:
			status = Rq_Set_Feature();
			break;
		case RQ_SET_ADDRESS:
			status = Rq_Set_Address();
			break;
		case RQ_GET_DESCRIPTOR:
			status = Rq_Get_Descriptor();
			break;
		case RQ_GET_CONFIGURATION:
			status = Rq_Get_Configuration();
			break;
		case RQ_SET_CONFIGURATION:
			status = Rq_Set_Configuration();
			break;
		case RQ_GET_INTERFACE:
			status = Rq_Get_Interface();
			break;
		case RQ_SET_INTERFACE:
			status = Rq_Set_Interface();
			break;
		case RQ_SYNCH_FRAME:
			break;

		default:
			status = 0;
			break;
		}
	}
	else if( (g_bmRequestType & REQUEST_TYPE_CHECK) == USBCLASS_RQ ){
		DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
//		status =  usb_AnalyzeClassRequest_MassStorage();
	}
	else{
		DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
		status = 0;
	}
	
	return status;
}

/*
 *********************************************************************
 * NAME  : Rq_Get_Status
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : status
 * DESCRIPTION  : GET STATUS梫媮張棟
 *
 *********************************************************************
 */
static short int Rq_Get_Status(void)
{
	unsigned char	index;
	unsigned char	attribute;

	index = 0;
	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);

	if( (fDirection == SET) || ((g_bmRequestType & RECEIVE_TYPE_CHECK) >= RECEIVE_ERROR)
	   || (g_wValue != 0) || (g_wIndex >= NUM_BREQRUEST_MAX) || (g_wLength != WLENGTH_MAX) ){
		return 0;
	}
	else{
		g_USB_Stage = DATA_STAGE;

		switch( g_bmRequestType & RECEIVE_TYPE_CHECK ){
		case RQ_DEVICE:
			switch( g_Current_State & CURRENT_STATUS_CHECK ){
			case DEFAULT:
				return 0;
				/* FALL THROUGH */
			case ADDRESSED:
				break;
			case CONFIGURED:
				break;
			default:
				return 0;
				/* FALL THROUGH */
			}
			attribute = Config_Desc[CONFIG_DESC_ATTRIBUTE] & ATTRIBUTE_CHECK;

			if( (attribute & SELF_POWERED_BIT) == SELF_POWERED_BIT ){
				fSelf_Powered = FLAG_ON;
			}
			else{
				fSelf_Powered = FLAG_OFF;
			}

			if( (attribute & REMOTE_WAKEUP_BIT) == REMOTE_WAKEUP_BIT ){
				fRemote_Wakeup = FLAG_ON;
			}
			else{
				fRemote_Wakeup = FLAG_OFF;
			}

			UDC2_Reg_Write(UD2EP0_FIFO_OFFSET, Dev_PowerStatus);
			break;
		case RQ_INTERFACE:
			switch( g_Current_State & CURRENT_STATUS_CHECK ){
			case DEFAULT:
				break;
			case ADDRESSED:
				return 0;
				/* FALL THROUGH */
			case CONFIGURED:
				break;
			default:
				return 0;
				/* FALL THROUGH */
			}

			index = (unsigned char) (g_wIndex & INDEX_CHECK);
			if( (g_Current_Config == 1) && (index > NUM_CONFIG1_INTERFACE) ){
				return 0;
			}
			/* DO NOTHING */

			UDC2_Reg_Write(UD2EP0_FIFO_OFFSET, 0);

			break;
		case RQ_ENDPOINT:
			switch( g_Current_State & CURRENT_STATUS_CHECK){
			case DEFAULT:
				return 0;
				/* FALL THROUGH */
			case ADDRESSED:
				if( index == EP0 ){
					break;
				}
				else{
					return 0;
				}
				/* FALL THROUGH */
			case CONFIGURED:
				break;
			default:
				return 0;
				/* FALL THROUGH */
			}

			index = (unsigned char) (g_wIndex & INDEX_CHECK);
			if( ((g_Current_Config == 0) && (index > 0)) ||
			    ((g_Current_Config == 1) && (index > NUM_TOTAL_ENDPOINTS)) ){
				return 0;
			}
			else{
				if( ST_Feature > 0 ){
					UDC2_Reg_Write(UD2EP0_FIFO_OFFSET, STALL_FEATURE);
				}
				else{
					UDC2_Reg_Write(UD2EP0_FIFO_OFFSET, 0);
				}
			}
			break;
		default :
			return 0;
			/* FALL THROUGH */
		}

		g_USB_Stage = STATUS_STAGE;
		UDC2_Reg_Write(UD2CMD_OFFSET, EP0_EOP);				/* process of EOP */
	}

	return 1;
}

/*
 *********************************************************************
 * NAME  : Rq_Clear_Feature
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : status
 * DESCRIPTION  : CLEAR FEATURE梫媮張棟
 *
 *********************************************************************
 */
static short int Rq_Clear_Feature(void)
{
	unsigned char	index;
	unsigned char	attribute;

	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);

	if( ((g_bmRequestType & MASK_UCHAR_UPPER_6BIT) != 0) || (g_wLength != 0) ){
		return 0;
	}
	else{
		index = (unsigned char) (g_wIndex & INDEX_CHECK);

		switch( g_bmRequestType & RECEIVE_TYPE_CHECK ){
		case RQ_DEVICE:
			switch( g_Current_State & CURRENT_STATUS_CHECK ){
			case DEFAULT:
				return 0;
				/* FALL THROUGH */
			case ADDRESSED:
				return 0;
				/* FALL THROUGH */
			case CONFIGURED:
				break;
			default:
				/* DO NOTHING */
				break;
			}
			
			if( g_wIndex == 0 && g_wValue == 1 ){
				attribute = Config_Desc[CONFIG_DESC_ATTRIBUTE] & ATTRIBUTE_CHECK;
				
				if( (attribute & REMOTE_WAKEUP_BIT) == REMOTE_WAKEUP_BIT ){
					fRemote_Wakeup = FLAG_OFF;
				}
				else{
					return 0;
				}
			}
			else{
				return 0;
			}
			break;
		case RQ_ENDPOINT:
			switch( g_Current_State & CURRENT_STATUS_CHECK ){
			case DEFAULT:
				return 0;
				/* FALL THROUGH */
			case ADDRESSED:
				if( index != EP0 ){
					return 0;
				}
				/* DO NOTHING */
				break;
			case CONFIGURED:
				break;
			default:
				/* DO NOTHING */
				break;
			}

			if( (g_Current_Config == 1) && (index > NUM_TOTAL_ENDPOINTS) ){
				return 0;
			}
			/* DO NOTHING */
			if( g_wValue != 0 ){
				return 0;
			}
			else{
				switch( index ){
				case EP0:
					fEP0_Stall_Feature = FLAG_OFF; 
					break;
				case EP1: fEP1_Stall_Feature = FLAG_OFF;
					UDC2_Reg_Write(UD2CMD_OFFSET, EP1_RESET);
					break;
				case EP2:
					fEP2_Stall_Feature = FLAG_OFF;
					UDC2_Reg_Write(UD2CMD_OFFSET, EP2_RESET);
					break;
				default:
					return 0;
					/* FALL THROUGH */
				}
			}
			break;
		default:
			return 0;
			/* FALL THROUGH */
		}
	}

	g_USB_Stage = STATUS_STAGE;
	UDC2_Reg_Write(UD2CMD_OFFSET, SETUP_FIN);

	return 1;
}

/*
 *********************************************************************
 * NAME  : Rq_Set_Feature
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : status
 * DESCRIPTION  : SET FEATURE梫媮張棟
 *
 *********************************************************************
 */
static short int Rq_Set_Feature(void)
{
	unsigned char	index;
	unsigned char	attribute;

	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);

	if( ((g_bmRequestType & MASK_UCHAR_UPPER_6BIT) != 0) || (g_wLength != 0) ){
																	/* g_wLength 0 fix check */
		return 0;
	}
	else{
		switch( g_bmRequestType & RECEIVE_TYPE_CHECK ){				/* g_bmRequestType check */
		case RQ_DEVICE:
			switch( g_Current_State & CURRENT_STATUS_CHECK ){
			case DEFAULT:
				return 0;
				/* FALL THROUGH */
			case ADDRESSED:
				return 0;
				/* FALL THROUGH */
			case CONFIGURED:
				break;
			default:
				/* DO NOTHING */
				break;
			}
			if( (g_wIndex == 0) && (g_wValue == 1) ){
				attribute = Config_Desc[CONFIG_DESC_ATTRIBUTE] & ATTRIBUTE_CHECK;
				if( (attribute & REMOTE_WAKEUP_BIT) == REMOTE_WAKEUP_BIT ){
					fRemote_Wakeup = FLAG_ON;
				}
				else{
					return 0;
				}
			}
			else
				return 0;
			break;
		case RQ_ENDPOINT:
			index = (unsigned char) (g_wIndex & INDEX_CHECK);

			switch( g_Current_State & CURRENT_STATUS_CHECK){
			case DEFAULT:
				return 0;
				/* FALL THROUGH */
			case ADDRESSED:
				if( index != EP0 ){
					return 0;
				}
				/* DO NOTHING */
				/* FALL THROUGH */
			case CONFIGURED:
				break;
			default:
				/* DO NOTHING */
				break;
			}

			if( (g_Current_Config == 1) && (index > NUM_TOTAL_ENDPOINTS) ){
				return 0;
			}
			/* DO NOTHING */

			if( (g_wValue != 0) || (g_wIndex >= NUM_BREQRUEST_MAX) ){
				return 0;
			}
			else{
				switch( index ){
				case EP0:
					fEP0_Stall_Feature = FLAG_ON;
					break;
				case EP1:
					fEP1_Stall_Feature = FLAG_ON;
					UDC2_Reg_Write(UD2CMD_OFFSET, EP1_STALL);
					break;
				case EP2:
					fEP2_Stall_Feature = FLAG_ON;
					UDC2_Reg_Write(UD2CMD_OFFSET, EP2_STALL);
					break;
				default:
					return 0;
					/* FALL THROUGH */
				}
			}
			break;
		default:
			return 0;
			/* FALL THROUGH */
		}
	}

	g_USB_Stage = STATUS_STAGE;

	return 1;
}

/*
 *********************************************************************
 * NAME  : Rq_Set_Address
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : status
 * DESCRIPTION  : SET ADDRESS梫媮張棟
 *
 *********************************************************************
 */
static short int Rq_Set_Address(void)
{
	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);

	if( (g_bmRequestType != (REQ_SET|STANDARD_RQ|RQ_DEVICE)) || (g_wValue >= NUM_BREQRUEST_MAX) ||
	    (g_wIndex != 0) || (g_wLength != 0) ){
		return 0;
	}
	else{
		if( fEP0_Stall_Feature == 1 ){
			return 0;
		}
		/* DO NOTHING */

		g_USB_Address = g_wValue;
		
		if( g_USB_Address > USB_ADDRESS_MAX ){
			return 0;
		}

		switch( g_Current_State & CURRENT_STATUS_CHECK ){
		case DEFAULT:
			if( g_USB_Address == 0 ){
				g_Buf_Current_State = DEFAULT;
			}
			else{
				g_Buf_Current_State = ADDRESSED;
			}
			break;
		case ADDRESSED:
			if( g_USB_Address == 0 ){
				g_Buf_Current_State = DEFAULT;
			}
			else{
				g_Buf_Current_State = ADDRESSED;
			}
			break;
		case CONFIGURED:
			if( g_USB_Address == 0 ){
				g_Buf_Current_State = DEFAULT;
				return 0;
			}
			else{
				g_Buf_Current_State = CONFIGURED;
			}
			break;
		default:
			return 0;
			/* FALL THROUGH */
		}

		g_USB_Stage = STATUS_STAGE;
	}

	return 1;
}

/*
 *********************************************************************
 * NAME  : Rq_Get_Configuration
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : status
 * DESCRIPTION  : GET CONFIGURATION梫媮張棟
 *
 *********************************************************************
 */
static short int Rq_Get_Configuration(void)
{
	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);

	if( ((g_bmRequestType & (DIRECTION_TYPE_CHECK | RECEIVE_TYPE_CHECK)) 
	   != (REQ_GET | STANDARD_RQ | RQ_DEVICE) )
	   || (g_wIndex != 0) || (g_wValue != 0) || (g_wLength != 1) ){
		return 0;
	}
	else{
		switch( g_Current_State & CURRENT_STATUS_CHECK ){
		case DEFAULT:
			return 0;
			/* FALL THROUGH */
		case ADDRESSED:
			UDC2_Reg_Write(UD2EP0_FIFO_OFFSET, 0);
			break;
		case CONFIGURED:
			UDC2_Reg_Write(UD2EP0_FIFO_OFFSET, g_Current_Config);
			break;
		default:
			return 0;
			/* FALL THROUGH */
		}

		g_USB_Stage = STATUS_STAGE;
		UDC2_Reg_Write(UD2CMD_OFFSET, EP0_EOP);
	}

	return 1;
}

/*
 *********************************************************************
 * NAME  : Rq_Set_Configuration
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : status
 * DESCRIPTION  : SET CONFIGURATION梫媮張棟
 *
 *********************************************************************
 */
static short int Rq_Set_Configuration(void)
{
	unsigned char	index;

	index = 0;
	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);

	if( (g_bmRequestType != (REQ_SET|STANDARD_RQ|RQ_DEVICE) ) 
	   	|| (g_wValue >= NUM_BREQRUEST_MAX) || (g_wLength != 0) ){
		return 0;
	}
	else{
		index = (unsigned char) (g_wValue & MASK_UINT16_LOWER_8BIT);
		
		if( index > NUM_CONFIG ){
			return 0;
		}

		switch( g_Current_State & CURRENT_STATUS_CHECK ){
		case DEFAULT:
			return 0;
			/* FALL THROUGH */
		case ADDRESSED:
			break;
		case CONFIGURED:
			break;
		default:
			/* DO NOTHING */
			break;
		}

		if( index == 0 ){		/* Config 0 */
			UDC2_Reg_Write(UD2CMD_OFFSET, All_EP_INVALID);			/*  INVALID */

			g_Buf_Current_State = ADDRESSED;
			g_Buf_Current_Config = index;
			g_Buf_Current_Interface = 0;
			g_Buf_Current_Alternate = 0;
		}
		else{
			UDC2_Reg_Write(UD2CMD_OFFSET, All_EP_INVALID);			/*  INVALID */
	
			if( index == 1 ){
				UDC2_Reg_Write(UD2EP1_MaxPacketSize_OFFSET, g_EP_PAYLOAD_SIZE[EP1]);
				UDC2_Reg_Write(UD2EP1_Status_OFFSET, EP_DUAL_BULK_IN);
				UDC2_Reg_Write(UD2EP2_MaxPacketSize_OFFSET, g_EP_PAYLOAD_SIZE[EP2]);
				UDC2_Reg_Write(UD2EP2_Status_OFFSET, EP_DUAL_BULK_OUT);

				UDC2_Reg_Write(UD2CMD_OFFSET, EP1_RESET);		/*EP1 Reset */
				UDC2_Reg_Write(UD2CMD_OFFSET, EP2_RESET);		/*EP2 Reset */
			}
			else{
				return 0;
			}

			g_Buf_Current_State = CONFIGURED;
			g_Buf_Current_Config = index;
			g_Buf_Current_Interface = 0;
			g_Buf_Current_Alternate = 0;

		}

		ST_Feature = STALL_FALL_CLEAR;	/* Stall Feature All Clear */
		g_USB_Stage = STATUS_STAGE;
	}

	return 1;
}

/*
 *********************************************************************
 * NAME  : Rq_Get_Interface
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : status
 * DESCRIPTION  : GET INTERFACE梫媮張棟
 *
 *********************************************************************
 */
static short int Rq_Get_Interface(void)
{
	unsigned char	index;

	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	if( ((g_bmRequestType & (DIRECTION_TYPE_CHECK | RECEIVE_TYPE_CHECK)) 
		 != (REQ_GET | STANDARD_RQ | RQ_INTERFACE) ) || (g_wValue != 0) || (g_wLength != 1) )	{
		return 0;
	}
	else{
		g_USB_Stage = DATA_STAGE;

		switch( g_Current_State & CURRENT_STATUS_CHECK){
		case DEFAULT:
			return 0;
			/* FALL THROUGH */
		case ADDRESSED:
			return 0;
			/* FALL THROUGH */
		case CONFIGURED:	
			break;
		default:
			/* DO NOTHING */
			break;
		}

		index = (unsigned char) (g_wIndex & MASK_UINT16_LOWER_8BIT);

		if( (g_Current_Config == 1) && (index > NUM_CONFIG1_INTERFACE) ){
			return 0;
		}
		/* DO NOTHING */

		UDC2_Reg_Write(UD2EP0_FIFO_OFFSET, g_Current_Alternate);
		g_USB_Stage = STATUS_STAGE;
		UDC2_Reg_Write(UD2CMD_OFFSET, EP0_EOP);					/* process of EOP */
	}

	return 1;
}

/*
 *********************************************************************
 * NAME  : Rq_Set_Interface
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : status
 * DESCRIPTION  : SET INTERFACE梫媮張棟
 *
 *********************************************************************
 */
static short int Rq_Set_Interface(void)
{
	unsigned char	index_interface;
	unsigned char	value_alternate;

	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);

	if( (g_bmRequestType != (REQ_SET|STANDARD_RQ|RQ_INTERFACE) ) || (g_wValue >= NUM_BREQRUEST_MAX)
		|| (g_wIndex >= NUM_BREQRUEST_MAX) || (g_wLength != 0) ){
		return 0;
	}
	else{
		
		switch( g_Current_State & CURRENT_STATUS_CHECK ){
		case DEFAULT:
			return 0;
			/* FALL THROUGH */
		case ADDRESSED:
			return 0;
			/* FALL THROUGH */
		case CONFIGURED:
			break;
		default:
			/* DO NOTHING */
			break;
		}

		index_interface = (unsigned char) (g_wIndex & MASK_UINT16_LOWER_8BIT);
		value_alternate = (unsigned char) (g_wValue & MASK_UINT16_LOWER_8BIT);

		if( g_Current_Config == 1 ){
			if( index_interface > NUM_CONFIG1_INTERFACE ){
				return 0;
			}
			if( value_alternate > NUM_C1IO_ALT0 ){
				return 0;
			}
		}
		else{
			return 0;
		}

		UDC2_Reg_Write(UD2CMD_OFFSET, All_EP_INVALID);		 /* INVALID */

		UDC2_Reg_Write(UD2EP1_MaxPacketSize_OFFSET, g_EP_PAYLOAD_SIZE[EP1]);
		UDC2_Reg_Write(UD2EP1_Status_OFFSET, EP_DUAL_BULK_IN);
		UDC2_Reg_Write(UD2EP2_MaxPacketSize_OFFSET, g_EP_PAYLOAD_SIZE[EP2]);
		UDC2_Reg_Write(UD2EP2_Status_OFFSET, EP_DUAL_BULK_OUT);

		UDC2_Reg_Write(UD2CMD_OFFSET, EP1_RESET);					/*EP1 Reset */
		UDC2_Reg_Write(UD2CMD_OFFSET, EP2_RESET);					/*EP2 Reset */

		g_Buf_Current_Interface = index_interface;
		g_Buf_Current_Alternate = value_alternate;

		ST_Feature = STALL_FALL_CLEAR;	/* Stall Feature All Clear */
		g_USB_Stage = STATUS_STAGE;
	}

	return 1;
}

/*
 *********************************************************************
 * NAME  : Rq_Get_Descriptor
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : status
 * DESCRIPTION  : GET DESCRIPTOR梫媮張棟
 *
 *********************************************************************
 */
static short int Rq_Get_Descriptor(void)
{
	unsigned char		type; /*status_reg; */
	unsigned char		index; /*status_reg; */
	struct config_desc*	config=NULL;
	struct string_desc*	string;
	unsigned char*		data1;
	int 			i;
	const unsigned char*	table;
	unsigned char 		ConfigDescData[CONFIG_DESC_SIZE];
	unsigned short int	Other_Speed_Payload_Size;
	unsigned short int	interrupt_status;

	if( fDirection == SET ){
		return 0;
	}
	else{
		g_USB_Stage = DATA_STAGE;

		type = (unsigned char) (g_wValue >> SHIFT_8BIT) ;		/* Descriptor type */
		index = (unsigned char) (g_wValue & MASK_UINT16_LOWER_8BIT) ;	/* String Descriptor Index */

		switch( g_Current_State & CURRENT_STATUS_CHECK ){
		case DEFAULT:
			UDC2_Reg_Write(UD2INT_OFFSET, USB_MASK);
			break;
		case ADDRESSED:
			break;
		case CONFIGURED:
			break;
		default:
			return 0;
			/* FALL THROUGH */
		}

		switch( type ){
		case TYPE_DEVICE:		/* Treatment of Device Descriptor */
			if( (g_wIndex == 0) && (index == 0) ){
				g_Expected_Length = DEVICE_DESC_SIZE;
				g_Start_Address = (unsigned long int) Dev_Desc;
				if( g_wLength <= g_Expected_Length ){
					g_Remain_TotalLength = g_wLength;
				}
				else{
					g_Remain_TotalLength = g_Expected_Length;
				}
				DBG("DESCDEV -- g_wIdx: %d; idx: %d; g_wLen: %d; g_ExpLen: %d; g_RemTotLen: %d\n",
					g_wIndex, index, g_wLength, g_Expected_Length, g_Remain_TotalLength);
				EP0_FIFO_Write();
			}
			else
				return 0;
			break;
		case TYPE_CONFIG:		/* Treatment of Config Descriptor */
			DBG("Function: %s, Line: %d, index: %d, g_wIndex: %d\n", __FUNCTION__, __LINE__, index, g_wIndex);
			if( (index > NUM_CONFIG) || (g_wIndex != 0) ){
				return 0;
			}
			else{
				data1 = ConfigDescData;
				table = Config_Desc;
				for( i=CONFIG_DESC_SIZE; i>0; data1++, table++, i--){
					*data1 = *table;
				}
				ConfigDescData[CONFIG_DESC_TYPE] = TYPE_CONFIG;				
				ConfigDescData[CONFIG_EP1_SIZE_LOW] = 
					(unsigned char) (g_EP_PAYLOAD_SIZE[EP1] & MASK_UINT16_LOWER_8BIT);
				ConfigDescData[CONFIG_EP1_SIZE_HIGH] = 
					(unsigned char) (g_EP_PAYLOAD_SIZE[EP1] >> SHIFT_8BIT);
				ConfigDescData[CONFIG_EP2_SIZE_LOW] = 
					(unsigned char) (g_EP_PAYLOAD_SIZE[EP2] & MASK_UINT16_LOWER_8BIT);
				ConfigDescData[CONFIG_EP2_SIZE_HIGH] = 
					(unsigned char) (g_EP_PAYLOAD_SIZE[EP2] >> SHIFT_8BIT);

				config =(struct	config_desc  *)&ConfigDescData[0];

				g_Expected_Length = config->wTotalLength;
				g_Start_Address = (unsigned long int) config;

				if( g_wLength <= g_Expected_Length ){
					g_Remain_TotalLength = g_wLength;
				}
				else{
					g_Remain_TotalLength = g_Expected_Length;
				}
				
				DBG("CFGDEV -- g_wIdx: %d; idx: %d; g_wLen: %d; g_ExpLen: %d; g_RemTotLen: %d\n",
					g_wIndex, index, g_wLength, g_Expected_Length, g_Remain_TotalLength);
				EP0_FIFO_Write();
			}
			break;
		case TYPE_STRING:		/* Treatment of String Descriptor */
			DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
			if( index < g_Num_StringDesc ){
				switch(index){
				case STRING_DESC_INDEX_0:
					string = (struct string_desc *)&Str_Desc_ROM[STR0_ADDRESS]; 
					break;
				case STRING_DESC_INDEX_1:
					string = (struct string_desc *)&Str_Desc_ROM[STR1_ADDRESS];
					break;
				case STRING_DESC_INDEX_2:
					string = (struct string_desc *)&Str_Desc_ROM[STR2_ADDRESS];
					break;
				case STRING_DESC_INDEX_3:
					string = (struct string_desc *)&Str_Desc_ROM[STR1_ADDRESS];
					break;
				default:
					return 0;
					/* FALL THROUGH */
				}

				if( string->bLength == 0 ){
					return 0;
				}
				else{
					g_Start_Address = (unsigned long int) string;
					g_Expected_Length = string->bLength;
					if( g_wLength <= g_Expected_Length ){
						g_Remain_TotalLength = g_wLength;
					}
					else{
						g_Remain_TotalLength = g_Expected_Length;
					}

					EP0_FIFO_Write();
				}
			}
			else{
				return 0;
			}
			break;
		case TYPE_DEVICE_QUALIFIER:
			DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
			if( index > 0 ){
				return 0;
			}
			else{
				config = (struct config_desc*) &Qualifier_Desc[0];

				g_Expected_Length = config->wTotalLength;
				g_Start_Address = (unsigned long int) config;

				if( g_wLength <= g_Expected_Length ){
					g_Remain_TotalLength = g_wLength;
				}
				else{
					g_Remain_TotalLength = g_Expected_Length;
				}

				EP0_FIFO_Write();
			}
			break;
		case TYPE_OTHER_SPEED:		/* Treatment of OTHER SPEED Descriptor */
			DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
			if( index > 0 ){
				return 0;
			}
			else{
				data1 = ConfigDescData;
				table = Config_Desc;
				for( i=CONFIG_DESC_SIZE; i>0; data1++, table++, i--){
					*data1 = *table;
				}
				ConfigDescData[CONFIG_DESC_TYPE] = TYPE_OTHER_SPEED;				
				Other_Speed_Payload_Size = 
					EP_MAX_PACKET_SIZE_HS + EP_MAX_PACKET_SIZE_FS - g_EP_PAYLOAD_SIZE[EP1];
				ConfigDescData[CONFIG_EP1_SIZE_LOW] = 
					(unsigned char) (Other_Speed_Payload_Size & MASK_UINT16_LOWER_8BIT);
				ConfigDescData[CONFIG_EP1_SIZE_HIGH] = 
					(unsigned char) (Other_Speed_Payload_Size >> SHIFT_8BIT);

				Other_Speed_Payload_Size = 
					EP_MAX_PACKET_SIZE_HS + EP_MAX_PACKET_SIZE_FS - g_EP_PAYLOAD_SIZE[EP2];
				ConfigDescData[CONFIG_EP2_SIZE_LOW] = 
					(unsigned char) (Other_Speed_Payload_Size & MASK_UINT16_LOWER_8BIT);
				ConfigDescData[CONFIG_EP2_SIZE_HIGH] = 
					(unsigned char) (Other_Speed_Payload_Size >> SHIFT_8BIT);

				g_Expected_Length = config->wTotalLength;
				g_Start_Address = (unsigned long int) config;

				if( g_wLength <= g_Expected_Length ){
					g_Remain_TotalLength = g_wLength;
				}
				else{
					g_Remain_TotalLength = g_Expected_Length;
				}

				EP0_FIFO_Write();
			}

			break;

		default:
			DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
			return 0;
			/* FALL THROUGH */
		}
		
		/* STATUS NAK Interrupt DISABLE. */
		UDC2_Reg_Read(UD2INT_OFFSET, &interrupt_status);
		interrupt_status &= 0x00ff;
		interrupt_status |= STATUS_NAK_D;
		UDC2_Reg_Write(UD2INT_OFFSET, interrupt_status);
	}

	return 1;
}

/*
 *********************************************************************
 * NAME		: USB_Bulk_Out
 *--------------------------------------------------------------------
 * DESCRIPTION	: Mass Storage Class Bulk-Out Process @ EP2
 *
 * PARAMETER	: 
 *
 * RETURN VALUE	: 
 *
 * Comment		: Called by USB_Int_EPx interrupt routine
 *********************************************************************
 */
static void USB_Bulk_Out(void)
{
	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);

	if( ( g_USB_Bulk_Out_Mode != BULKOUTMODE_DMA) && ( g_USB_Rbuf_Status == RRBST_NORMAL) ){
		USB_Bulk_Out_Start();
	}
}

/*
 *********************************************************************
 * NAME		: USB_Bulk_In
 *--------------------------------------------------------------------
 * DESCRIPTION	: Mass Storage Class Bulk-In Process @ EP1
 *
 * PARAMETER	: 
 *
 * RETURN VALUE	: 
 *
 * Comment		: Called by USB_Int_EPx interrupt routine
 *********************************************************************
 */
static void USB_Bulk_In(void)
{
	unsigned char*		buff;							/* 憲怣僶僢僼傽億僀儞僞 */
	unsigned short int	epsize;							/* EP憲怣壜擻僨乕僞挿 */
	unsigned long int	reg_data;
	struct tmpa910_udc 	*udc = &controller;

	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);

	if( g_DMA_Send_Length > 0 ){
		/*--------------------------*/
		/* DMA揮憲忬懺僠僃僢僋		*/
		/*--------------------------*/
		reg_data = tmpa910_udp_read(udc, UD2AB_INTSTS_OFFSET);
		if( (reg_data&INT_MR_AHBERR ) == INT_MR_AHBERR){
			tmpa910_udp_write(udc, UD2AB_UDMSTSET_OFFSET, UDC2AB_MR_RESET);	/* 儅僗僞乕儕乕僪儕僙僢僩 */
		}
		else {
			g_USB_Bulk_In_Mode = BULKINMODE_DMA;
			epsize = USB_Send_Length_Chk(g_DMA_Send_Length);
			g_DMA_Send_Length -= epsize;
//			epsize = g_DMA_Send_Length;

			buff = g_DMA_Send_Buff_Address;			/* 憲怣僨乕僞奿擺愭傪庢摼 */
			g_DMA_Send_Buff_Address += epsize;

			tmpa910_udp_write(udc, UD2AB_MRSADR_OFFSET, (unsigned long int) buff);
			tmpa910_udp_write(udc, UD2AB_MREADR_OFFSET, (unsigned long int) (buff + epsize-1));
			tmpa910_udp_write(udc, UD2AB_UDMSTSET_OFFSET, UDC2AB_MR_ENABLE);/* DMA揮憲奐巒 */
		}
	}
	
	return;
}

/*
 *********************************************************************
 * NAME		: USB_Send_Length_Chk
 *--------------------------------------------------------------------
 * DESCRIPTION	: Check DMA sending data size according to EP payload
 *
 * PARAMETER	: Length	-- Current sent data length
 *
 * RETURN VALUE	: size		-- Data length to be allowed to send at EP
 *
 * Comment		: Called by USB_Int_EPx interrupt routine
 *********************************************************************
 */
static unsigned short int USB_Send_Length_Chk(const unsigned long int Length)
{
	unsigned short int size;

	if( Length > 0 ){
		if( Length > g_EP_PAYLOAD_SIZE[EP1] ){
			size = g_EP_PAYLOAD_SIZE[EP1];
		}
		else{
			size = (unsigned short int) Length;
		}
	}
	else{
		size = 0;
	}
	return size;
}

/*
 *********************************************************************
 * NAME		: USB_Recv_Length_Chk
 *--------------------------------------------------------------------
 * DESCRIPTION	: Check DMA receiving data size according to EP payload
 *
 * PARAMETER	:
 *
 * RETURN VALUE	: size		-- Data length to be allowed to receive at EP
 *
 * Comment		: Called by USB_Int_EPx interrupt routine
 *********************************************************************
 */
static unsigned short int USB_Recv_Length_Chk(void)
{
	unsigned short int data_size;
	unsigned short int size;

	UDC2_Reg_Read(UD2EP2_DataSize_OFFSET, &data_size);

	data_size &= UD2EP_DATASIZE_MASK;
	if( data_size > 0 ){
		if( data_size > g_EP_PAYLOAD_SIZE[EP2] ){
			size = g_EP_PAYLOAD_SIZE[EP2];
		}
		else{
			size = data_size;
		}
	}
	else{
		size = 0;
	}
	return size;
}

/*
 *********************************************************************
 * NAME		: USB_Bulk_Out_Start
 *--------------------------------------------------------------------
 * DESCRIPTION	: Start the current Bulk-Out course @ EP2
 *
 * PARAMETER	: 
 *
 * RETURN VALUE	: 
 *
 * Comment		:
 *********************************************************************
 */
static void USB_Bulk_Out_Start(void)
{
	unsigned char	*buff;						/* 庴怣僶僢僼傽億僀儞僞	*/
	unsigned short int epsize;					/* EP庴怣僨乕僞挿 */
	unsigned long int reg_data;
	struct tmpa910_udc *udc = &controller;

	/*--------------------------*/
	/* 僨乕僞偺桳柍傪僠僃僢僋	*/
	/*--------------------------*/
	epsize = USB_Recv_Length_Chk();

	/* 庴怣僨乕僞偁傝	*/
	if( epsize > 0 ){
		/* 僶儖僋傾僂僩儌乕僪偵DMA傪愝掕	*/
		g_USB_Bulk_Out_Mode = BULKOUTMODE_DMA;

		/*--------------------------*/
		/* DMA揮憲僒僀僘傪寛傔傞	*/
		/*--------------------------*/
		if( g_DMA_Recv_Length < epsize ){
			g_DMA_Recv_Length = 0;
		}
		else{
			g_DMA_Recv_Length -= epsize;
		}


		/*--------------------------*/
		/* DMA揮憲忬懺僠僃僢僋		*/
		/*--------------------------*/
		reg_data = tmpa910_udp_read(udc, UD2AB_INTSTS_OFFSET);
		if( ((reg_data&INT_MW_AHBERR ) == INT_MW_AHBERR) || ((reg_data&INT_MW_RD_ERR ) == INT_MW_RD_ERR) ){
			tmpa910_udp_write(udc, UD2AB_UDMSTSET_OFFSET, UDC2AB_MW_RESET);
		}
		else	{
			/*--------------------------------------*/
			/* FIFO偐傜庴怣僶僢僼傽傊DMA揮憲傪峴偆	*/
			/*--------------------------------------*/
			buff = g_DMA_Recv_Buff_Address;	/* 庴怣僨乕僞彂偒崬傒愭傪庢摼	*/
			g_Bulk_Out_EP_Size = epsize;	/* 庴怣僨乕僞挿傪曐懚			*/
			tmpa910_udp_write(udc, UD2AB_MWSADR_OFFSET, (unsigned long int)buff);
			tmpa910_udp_write(udc, UD2AB_MWEADR_OFFSET,(unsigned long int)(buff + g_Bulk_Out_EP_Size-1));
			//UD2AB_MWTOUT = 0xC9;		/* DMA transfer timeout setting: CLK_U*200ns */
			tmpa910_udp_write(udc, UD2AB_UDMSTSET_OFFSET, UDC2AB_MW_ENABLE);	/* DMA揮憲奐巒 */
		}
	}
	else{ 
		/* 嵞搙丄USB偺庴怣妱崬傒偱娔帇偡傞	*/ 
		g_USB_Bulk_Out_Mode = BULKOUTMODE_USB;
	}
	
	return;
}

/*
 *********************************************************************
 * NAME  : EP0_FIFO_Read
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : status
 * DESCRIPTION  : EP0偐傜偺僨乕僞庢摼張棟
 *
 *********************************************************************
 */
static void EP0_FIFO_Read(void)
{
	unsigned short int	PacketSize;
	unsigned char*		data_p;
	unsigned short int	length;
	unsigned short int	interrupt_status;
	unsigned short int*	iaddress;

	data_p = NULL;

	if( g_Remain_TotalLength > g_EP_PAYLOAD_SIZE[EP0] ){ /* judgement of Remain TotalLength */
		length = g_EP_PAYLOAD_SIZE[EP0];
		iaddress = (unsigned short int *)g_Start_Address;

		/* STATUS NAK Interrupt Enable. */
		UDC2_Reg_Read(UD2INT_OFFSET, &interrupt_status);
		interrupt_status &= STATUS_NAK_E;
		UDC2_Reg_Write(UD2INT_OFFSET, interrupt_status);

		DBG("0x");
		while( length > 0 ){				/* write transmit data to endpoint0's Fifo */
			UDC2_Reg_Read(UD2EP0_FIFO_OFFSET, iaddress);
			DBG("%02x %02x ", *((unsigned char*)iaddress), *((unsigned char*)iaddress+1));
			iaddress++;
			length -= WORD_SIZE;
		}
		DBG("\n");

		g_Remain_TotalLength -= g_EP_PAYLOAD_SIZE[EP0]; /*increment of Remain TotalLength */
		g_Start_Address = (unsigned long int) iaddress;		/* increment of g_Start_Address */
	}
	else{
		length = g_Remain_TotalLength;
		iaddress = (unsigned short int *)g_Start_Address;

		DBG("0x");
		while( length != 0 ){				/* read data from endpoint0's Fifo */
			if( length == 1 ){
				UDC2_Reg_Read(UD2EP0_MaxPacketSize_OFFSET, &PacketSize);
				UDC2_Reg_Write(UD2EP0_MaxPacketSize_OFFSET, 1);		/* process of EOP */

                               	DBG("Function: %s, Line: %d, g_Remain_TotalLength: %d\n",
			          	__FUNCTION__, __LINE__, g_Remain_TotalLength);
				UDC2_Reg_Read(UD2EP0_FIFO_OFFSET, iaddress);
				*data_p = (unsigned char)*iaddress;
				length = 0;

				DBG("%02x %02x ", *((unsigned char*)iaddress), *((unsigned char*)iaddress+1));	

				UDC2_Reg_Write(UD2EP0_MaxPacketSize_OFFSET, PacketSize);/* process of EOP */
	
			}
			else{
				UDC2_Reg_Read(UD2EP0_FIFO_OFFSET, iaddress);
				DBG("%02x %02x ", *((unsigned char*)iaddress), *((unsigned char*)iaddress+1));
				iaddress++;
				length -=WORD_SIZE;
			}
		}
		DBG("\n");

		g_USB_Stage = STATUS_STAGE;						/* shift of Stage to STATUS_STAGE */

		/* STATUS NAK Interrupt Disable. */
		UDC2_Reg_Read(UD2INT_OFFSET, &interrupt_status);
		interrupt_status |= STATUS_NAK_D;
		UDC2_Reg_Write(UD2INT_OFFSET, interrupt_status);
	}

	return;
}

/*
 *********************************************************************
 * NAME  : EP0_FIFO_Write
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : status
 * DESCRIPTION  : EP0傊偺僨乕僞憲怣張棟
 *
 *********************************************************************
 */
static void EP0_FIFO_Write(void)
{
	unsigned short int	length;
	unsigned short int	interrupt_status;
	unsigned char		chardata;
	unsigned short int*	iaddress;		/* 2byte Write data to EP0 FIFO */
	unsigned short int	PacketSize;

	DBG("EP0_FIFO_Write\n");
	if( g_Remain_TotalLength > g_EP_PAYLOAD_SIZE[EP0] ){ /* judgement of Remain TotalLength */
		iaddress = (unsigned short int*)g_Start_Address;

		/* STATUS NAK Interrupt Enable. */
		UDC2_Reg_Read(UD2INT_OFFSET, &interrupt_status);
		interrupt_status &= STATUS_NAK_E;
		UDC2_Reg_Write(UD2INT_OFFSET, interrupt_status);

		length = g_EP_PAYLOAD_SIZE[EP0];
		DBG("%s: Int-Status: 0x%x; length:%d; RemainLen: %d; StartAddr:0x%lx\n",
			__FUNCTION__, interrupt_status,	length, g_Remain_TotalLength, g_Start_Address);
		
		DBG("0x");
		while( length > 0 ){			/* write transmit data to endpoint0's Fifo */
			UDC2_Reg_Write(UD2EP0_FIFO_OFFSET, *iaddress);
			DBG("%02x %02x ", *((unsigned char*)iaddress), *((unsigned char*)iaddress+1));
			iaddress++;
			length -= WORD_SIZE;
		}
		DBG("\n");

		g_Remain_TotalLength -= g_EP_PAYLOAD_SIZE[EP0]; /* increment of RemainTotalLength */
		g_Start_Address = (unsigned long int) iaddress;	/* increment of g_Start_Address */
	}
	else{
		length = g_Remain_TotalLength;
		iaddress = (unsigned short int *)g_Start_Address;
		DBG("0x");
		while( length != 0 ){				/* write transmit data to endpoint0's Fifo */
			if( length == 1 ){
				UDC2_Reg_Read(UD2EP0_MaxPacketSize_OFFSET, &PacketSize);
				UDC2_Reg_Write(UD2EP0_MaxPacketSize_OFFSET, 1);			/* process of EOP */

				chardata = (unsigned char) (*iaddress & MASK_UINT16_LOWER_8BIT);
				UDC2_Reg_Write(UD2EP0_FIFO_OFFSET, (unsigned long int)chardata);
				length = 0;

				DBG("%02x ", chardata);
				UDC2_Reg_Write(UD2EP0_MaxPacketSize_OFFSET, PacketSize);	/* process of EOP */
	
			}
			else{
				UDC2_Reg_Write(UD2EP0_FIFO_OFFSET, *iaddress);
				DBG("%02x %02x ", *((unsigned char*)iaddress), *((unsigned char*)iaddress+1));	
				iaddress++;
				length -= WORD_SIZE;
			}
		}
		DBG("\n");

		g_USB_Stage = STATUS_STAGE;					/* shift of Stage to STATUS_STAGE */
		UDC2_Reg_Write(UD2CMD_OFFSET, EP0_EOP);			/* process of EOP */

		/* STATUS NAK Interrupt Disable. */
		UDC2_Reg_Read(UD2INT_OFFSET, &interrupt_status);
		interrupt_status |= STATUS_NAK_E;
		UDC2_Reg_Write(UD2INT_OFFSET, interrupt_status);

	}

	return;
}

/*
 *********************************************************************
 * NAME  : UDC2_Reg_Read
 *--------------------------------------------------------------------
 * PARAMETER  : ReqAddr:儗僕僗僞傾僪儗僗丄Data_p:僨乕僞奿擺愭億僀儞僞
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : UDC2儗僕僗僞偐傜偺僨乕僞儕乕僪
 *
 *********************************************************************
 */
static void UDC2_Reg_Read(const unsigned long int reqaddr, unsigned short int* data_p)
{
	unsigned long int read_addr;
	unsigned long int reg_data;
	struct tmpa910_udc *udc;

	udc = &controller;
	read_addr = (reqaddr & UDC2AB_READ_ADDRESS) | UDC2AB_READ_RQ;
	tmpa910_udp_write(udc, UD2AB_UDC2RDREQ_OFFSET, read_addr);
	ndelay(1);	
	
	do{
		reg_data = tmpa910_udp_read(udc, UD2AB_UDC2RDREQ_OFFSET);
		ndelay(1);
	}while( (reg_data & UDC2AB_READ_RQ) == UDC2AB_READ_RQ );
	
	tmpa910_udp_write(udc, UD2AB_INTSTS_OFFSET, INT_UDC2REG_RD);
	ndelay(1);

	reg_data = tmpa910_udp_read(udc, UD2AB_UDC2RDVL_OFFSET);
	*data_p = (unsigned short int) (reg_data & MASK_UINT32_LOWER_16BIT);
	ndelay(1);	
	
	return;
}

/*
 *********************************************************************
 * NAME  : UDC2_Reg_Write
 *--------------------------------------------------------------------
 * PARAMETER  :  ReqAddr:儗僕僗僞傾僪儗僗丄Data:僨乕僞
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : UDC2儗僕僗僞傊偺僨乕僞儕乕僪
 *				  (UDC2傾僋僙僗傪柧帵揑偵掕媊)
 *********************************************************************
 */
static void UDC2_Reg_Write(const unsigned long int reqAddr, const unsigned short int data)
{
	unsigned long int reg_data;
        struct tmpa910_udc *udc;
	
	udc = &controller;
	reg_data = (unsigned long int) data;
	ndelay(1);

	tmpa910_udp_write(udc, reqAddr, reg_data);
	ndelay(1);

	return;
}

/*
 *********************************************************************
 * NAME  : MemcpyToRecvBufWithRewind
 *--------------------------------------------------------------------
 * PARAMETER	: prbf_wpt	-- Pointer to the write pointer of receive buffer
 *				  psrc		-- Pointer to the source data to be copied
 *				  cnt		-- Length of source data
 * RETURN VALUE  : 
 * DESCRIPTION  : memory copy operation with rewind to target buffer
 *
 *********************************************************************
 */
static void MemcpyToRecvBufWithRewind(void **prbf_wpt, const void *psrc, unsigned long int cnt)
{
	unsigned long int	a, b, delta1, delta2, p;

	p = (unsigned long int) *prbf_wpt;
	a = (unsigned long int) (p+cnt);
	b = (unsigned long int) g_USB_Recv_Rbuf + sizeof( g_USB_Recv_Rbuf);
	
	if( a >= b ){
		//If rewind to start of receive buffer, just divide the source data into two parts
		delta1 = b - (unsigned long int)*prbf_wpt;
		delta2 = a - b;

		//Copy source data from current write point to the end of receive buffer
		memcpy((void*)*prbf_wpt, psrc, delta1);

		//Copy remaining source data to receive buffer rewinding from beginning of receive buffer
		memcpy((void*)g_USB_Recv_Rbuf, (void*)((unsigned long int)psrc + delta1), delta2);
		
		//Modify the write pointer of receive buffer
		*prbf_wpt = g_USB_Recv_Rbuf + a - b;
	}
	else {
		//Totally copy source data to target receive buffer with no need of rewinding
		memcpy(*prbf_wpt, psrc, cnt);

		//Modify the write pointer of receive buffer
		p += cnt;
		*prbf_wpt = (void *)p;
	}
}

/*
 *********************************************************************
 * NAME  : MemcpyFromRecvBufWithRewind
 *--------------------------------------------------------------------
 * PARAMETER	: pdest		-- Pointer to the destination data
 *				  prbf_rpt	-- Pointer to the read pointer of receive buffer
 *				  cnt		-- Length of source data
 * RETURN VALUE  : 
 * DESCRIPTION  : memory copy operation with rewind from target buffer
 *
 *********************************************************************
 */
#if 0
static void MemcpyFromRecvBufWithRewind(void *pdest, const void **prbf_rpt, unsigned long int cnt)
{
	unsigned long int	a, b, delta1, delta2, p;

	p = (unsigned long int) *prbf_rpt;
	a = (unsigned long int) (p+cnt);
	b = (unsigned long int) g_USB_Recv_Rbuf + sizeof( g_USB_Recv_Rbuf);
	
	if( a >= b ){
		//If rewind to start of receive buffer, just divide the source data into two parts
		delta1 = b - (unsigned long int)*prbf_rpt;
		delta2 = a - b;

		//Copy source data from current read point to the end of receive buffer
		memcpy((void*)pdest, *prbf_rpt, delta1);

		//Copy remaining source data to receive buffer rewinding from beginning of receive buffer
		memcpy((void*)((unsigned long int)pdest + delta1), (void*)g_USB_Recv_Rbuf, delta2);
		
		//Modify the read pointer of receive buffer
		*prbf_rpt = g_USB_Recv_Rbuf + a - b;
	}
	else {
		//Totally copy source data to target receive buffer with no need of rewinding
		memcpy(pdest, *prbf_rpt, cnt);

		//Modify the read pointer of receive buffer
		p += cnt;
		*prbf_rpt = (void *)p;
	}
}
#endif

/*
 *********************************************************************
 * NAME  : Rbuf_Stchk
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 
 * DESCRIPTION  : 儕儞僌僶僢僼傽偺僨乕僞庴怣僗僥乕僞僗傪妋擣
 *
 *********************************************************************
 */
static unsigned char Rbuf_Stchk(void)
{
	unsigned long int	w1;
	unsigned long int	w2;
	unsigned long int	bt;
	unsigned long int	bs;

	w1 = (unsigned long int) g_Recv_Rbuf_Rpt;
	w2 = (unsigned long int) g_Recv_Rbuf_Wpt;
	if( w1 == w2){
		return RRBST_NORMAL;
	}
	else{
		if( w1 > w2 ){
			w1 = w1 - w2 -1;
		}
		else{
			bt = (unsigned long int) g_USB_Recv_Rbuf;
			bs = sizeof( g_USB_Recv_Rbuf);
			w1 = ( (bt + bs) - w2) + (w1 - bt - 1);
		}
		
		if( w1 > g_EP_PAYLOAD_SIZE[EP2]){
			return RRBST_NORMAL;
		}
		else{
			return RRBST_FULL;
		}
	}

}

/*
 *********************************************************************
 * NAME  : USB_Ctl_Init
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : 撪晹曄悢弶婜壔
 *
 *********************************************************************
 */
static void USB_Ctl_Init(void)
{
	/*------------------------------*/
	/* Stamdard Class initialize			*/
	/*------------------------------*/
	g_Current_State = DEFAULT;
	g_Current_Config = USB_INIT;
	g_Current_Interface = USB_INIT;
	g_Current_Alternate = USB_INIT;
	g_Buf_Current_Config = USB_INIT;
	g_Buf_Current_Interface = USB_INIT;
	g_Buf_Current_Alternate = USB_INIT;

	g_USB_Stage = IDLE_STAGE;
	g_EP_PAYLOAD_SIZE[EP0] = EP_MAX_PACKET_SIZE_FS ;
	g_EP_PAYLOAD_SIZE[EP1] = EP_MAX_PACKET_SIZE_HS ;
	g_EP_PAYLOAD_SIZE[EP2] = EP_MAX_PACKET_SIZE_HS ;

	/*------------------------------*/
	/* Vender Class initialize			*/
	/*------------------------------*/
	g_DMA_Recv_Buff_Address = (unsigned char *)g_USB_Recv_Buf;
	g_Recv_Rbuf_Wpt = g_USB_Recv_Rbuf;
	g_Recv_Rbuf_Rpt = g_USB_Recv_Rbuf;
	g_USB_Rbuf_Status = RRBST_NORMAL;		/* Normal status Set				*/
	g_USB_Status = USB_STS_IDOL;			/* USB僪儔僀僶忬懺傪弶婜壔			*/
	g_USB_Stage_Error = STAGE_NORMAL;		/* 僗僥乕僕僄儔乕僼儔僌弶婜壔		*/
	g_DMA_Recv_Length = 0;					/* 巆庴怣僨乕僞弶婜壔				*/
	g_DMA_Send_Length = 0;					/* 巆憲怣僨乕僞弶婜壔				*/
	g_Num_StringDesc = STRING_DESC_INIT;	/* string descriptor悢弶婜壔		*/ 
	
	return;

}

/*
 *********************************************************************
 * NAME  : USB_Int_Setup
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : SETUP僗僥乕僕廔椆庴怣妱傝崬傒
 *
 *********************************************************************
 */
static void USB_Int_Setup(void)
{
	short int	status;
	
	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	UDC2_Reg_Write(UD2INT_OFFSET, INT_SETUP_CLEAR);

	if( g_USB_Stage == IDLE_STAGE ){
		g_USB_Stage = SETUP_STAGE;	/* g_STAGE : IDLE -> SETUP */
	}
	/* DO NOTHING */

	status = USB_Receive_Request();
	if( status == 0 ){
		UDC2_Reg_Write(UD2CMD_OFFSET, EP0_STALL);			/* EP0 STALL */
		UDC2_Reg_Write(UD2CMD_OFFSET, EP1_STALL);			/* EP1 STALL */
		UDC2_Reg_Write(UD2CMD_OFFSET, EP2_STALL);			/* EP2 STALL */
	}
	/* DO NOTHING */

	return;
}

/*
 *********************************************************************
 * NAME  : USB_Int_Rx_Zero
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : 0僶僀僩僨乕僞庴怣妱傝崬傒
 *
 *********************************************************************
 */
static void USB_Int_Rx_Zero(void)
{
	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	UDC2_Reg_Write(UD2INT_OFFSET, INT_DATA_CLEAR);
	if( g_USB_Stage == DATA_STAGE ){
		
		if( fDirection == GET ){
			EP0_FIFO_Write();
		}
		else{
			EP0_FIFO_Read();
		}
	}
	/* DO NOTHING */
	
	return;
}

/*
 *********************************************************************
 * NAME  : USB_Int_Status
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : 僗僥乕僞僗僗僥乕僕廔椆庴怣妱傝崬傒
 *
 *********************************************************************
 */
static void USB_Int_Status(void)
{
	unsigned short int	interrupt_status;

	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	UDC2_Reg_Write(UD2INT_OFFSET, INT_STATUS_CLEAR);

	if( g_USB_Stage == STATUS_STAGE ){
		/* STATUS NAK Interrupt Enable. */
		UDC2_Reg_Read(UD2INT_OFFSET, &interrupt_status);
		interrupt_status |= INT_STATUSNAK_MASK;
		UDC2_Reg_Write(UD2INT_OFFSET, interrupt_status);
				
		if( (g_bRequest == RQ_SET_CONFIGURATION ) || (g_bRequest == RQ_SET_ADDRESS) ){
			g_Current_State = g_Buf_Current_State;		/* increment of state */
			g_Current_State |= g_USB_Address;
			UDC2_Reg_Write(UD2ADR_OFFSET, g_Current_State);
		}
		/* DO NOTHING */

		g_Current_Config = g_Buf_Current_Config;
		g_Current_Interface = g_Buf_Current_Interface;
		g_Current_Alternate = g_Buf_Current_Alternate;
		g_USB_Stage = IDLE_STAGE;	/* Stage Information initialyze. */
	}

	return;

}

/*
 *********************************************************************
 * NAME  : USB_Int_Status_NAK
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : STATUS NAK庴怣妱傝崬傒
 *
 *********************************************************************
 */
static void USB_Int_Status_NAK(void)
{
	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	UDC2_Reg_Write(UD2INT_OFFSET, INT_STATUSNAK_CLEAR);
	UDC2_Reg_Write(UD2CMD_OFFSET, SETUP_FIN);
	
	return;

}

/*
 *********************************************************************
 * NAME  : USB_Int_EP0
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : EP0僨乕僞憲庴怣妱傝崬傒
 *
 *********************************************************************
 */
static void USB_Int_EP0(void)
{
	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	UDC2_Reg_Write(UD2INT_OFFSET, INT_EP0_CLEAR);

	if( fDirection == GET ){		/*Write */
		EP0_FIFO_Write();
	}
	else{					/*Read */
		EP0_FIFO_Read();
	}

	return;

}

/*
 *********************************************************************
 * NAME  : USB_Int_EPx
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : EP0&1僨乕僞庴怣妱傝崬傒
 *
 *********************************************************************
 */
static void USB_Int_EPx(void)
{
	unsigned short int EP1_RegData;
	unsigned short int EP2_RegData;

	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	UDC2_Reg_Write(UD2INT_OFFSET, INT_EP_CLEAR);

	UDC2_Reg_Read(UD2EP1_MaxPacketSize_OFFSET, &EP1_RegData);
	UDC2_Reg_Read(UD2EP2_MaxPacketSize_OFFSET, &EP2_RegData);

	if( (EP1_RegData & UD2EP_DSET) == UD2EP_DSET ){			/*dset? */
		USB_Bulk_In();
//		usb_bulkin();
	}
	/* DO NOTHING */

	if( (EP2_RegData & UD2EP_DSET) == UD2EP_DSET ){			/*dset? */
		USB_Bulk_Out();
//		usb_bulkout();
	}
	/* DO NOTHING */

	
	return;

}

/*
 *********************************************************************
 * NAME  : USB_Int_SOF
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : SOF僷働僢僩庴怣妱傝崬傒
 *
 *********************************************************************
 */
static void USB_Int_SOF(void)
{
	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	UDC2_Reg_Write(UD2INT_OFFSET, INT_SOF_CLEAR);

	return;

}

/*
 *********************************************************************
 * NAME  : USB_Int_Supend_Resume
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : SUSPEND RESUME妱傝崬傒庴怣張棟
 *
 *********************************************************************
 */
static void USB_Int_Supend_Resume(void)
{
        struct tmpa910_udc *udc = &controller;
	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	tmpa910_udp_write(udc, UD2AB_INTSTS_OFFSET, INT_SUSPEND);
	g_USB_Status = USB_STS_SUSPEND;			/* 忬懺傪僒僗儁儞僪拞偵慗堏			*/

	return;
}

/*
 *********************************************************************
 * NAME  : USB_Int_USB_Reset_End
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : 儕僙僢僩廋椆妱傝崬傒庴怣
 *
 *********************************************************************
 */
static void USB_Int_USB_Reset_End(void)
{
	unsigned short int state;
        struct tmpa910_udc *udc = &controller;

	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	tmpa910_udp_write(udc, UD2AB_INTSTS_OFFSET, INT_RESET_END);
	
	UDC2_Reg_Read(UD2ADR_OFFSET, &state);
	state  &= CURRENT_SPEED_CHECK; 				/* Current_Speed check*/
	UDC2_Reg_Write(UD2INT_OFFSET, UDC2_INT_MASK);		/*INT EPx MASK & Refresh; */

	if( state == HIGH_SPEED ){
		g_EP_PAYLOAD_SIZE[EP1] = EP_MAX_PACKET_SIZE_HS ; /* Payload Size = 512byte(HIGH SPEED) */
		g_EP_PAYLOAD_SIZE[EP2] = EP_MAX_PACKET_SIZE_HS ; /* Payload Size = 512byte(HIGH SPEED) */
	}
	else{
		g_EP_PAYLOAD_SIZE[EP1] = EP_MAX_PACKET_SIZE_FS ; /* Payload Size = 64byte(FULL SPEED) */
		g_EP_PAYLOAD_SIZE[EP2] = EP_MAX_PACKET_SIZE_FS ; /* Payload Size = 64byte(FULL SPEED) */
	}
	
	g_USB_Rbuf_Status = 0;
	g_USB_Bulk_Out_Mode = 0;
	g_Recv_Rbuf_Wpt = g_USB_Recv_Rbuf;
	g_Recv_Rbuf_Rpt = g_USB_Recv_Rbuf;
	
	tmpa910_udp_write(udc, UD2AB_UDMSTSET_OFFSET, UDC2AB_MR_RESET | UDC2AB_MW_RESET);
	return;
}

/*
 *********************************************************************
 * NAME  : USB_Int_USB_Reset
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : 儕僙僢僩廋椆妱傝崬傒庴怣
 *
 *********************************************************************
 */
static void USB_Int_USB_Reset(void)
{
        struct tmpa910_udc *udc = &controller;

	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	tmpa910_udp_write(udc, UD2AB_INTSTS_OFFSET, INT_RESET);

	return;
}

/*
 *********************************************************************
 * NAME  : USB_Int_MR_End
 *--------------------------------------------------------------------
 * PARAMETER  : 側偟
 * RETURN VALUE  : 側偟
 * DESCRIPTION  : 儅僗僞乕儕乕僪揮憲廔椆妱傝崬傒
 *
 *********************************************************************
 */
static void USB_Int_MR_End(void)
{
        struct tmpa910_udc *udc = &controller;

	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	tmpa910_udp_write(udc, UD2AB_INTSTS_OFFSET, INT_MR_END_ADD);
	
	if( g_USB_Bulk_In_Mode == BULKINMODE_DMA ){
		/* USB偺憲怣壜擻妱崬傒娔帇	*/
		if(g_DMA_Send_Length == 0) {
			g_USB_Bulk_In_Mode = BULKINMODE_USB;
		
			//Trigger next phase data transfer
//			usb_bulkin_mass_storage();
		}
		else{
			//Continue next phase bulk-in transfer
			USB_Bulk_In();
		}
	}
	
	return;
}

static void USB_Int_MW_End(void)
{
	unsigned long int intsts;
        struct tmpa910_udc *udc = &controller;

	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
	tmpa910_udp_write(udc, UD2AB_INTSTS_OFFSET, INT_MW_END_ADD);

	//Optimize the speed using memcpy instead of "for" loop
	MemcpyToRecvBufWithRewind((void **)&g_Recv_Rbuf_Wpt, (void *)g_USB_Recv_Buf, (unsigned long int)g_Bulk_Out_EP_Size);
	
	g_USB_Rbuf_Status = Rbuf_Stchk();

	//Check for last write error
	intsts = tmpa910_udp_read(udc, UD2AB_INTSTS_OFFSET);
	if( ( intsts & INT_MW_AHBERR ) == INT_MW_AHBERR ){
		tmpa910_udp_write(udc, UD2AB_INTSTS_OFFSET, INT_MW_AHBERR);
		tmpa910_udp_write(udc, UD2AB_UDMSTSET_OFFSET, UDC2AB_MW_RESET);
		return;
	}
	if( (intsts&INT_MW_RD_ERR ) == INT_MW_RD_ERR ){
		tmpa910_udp_write(udc, UD2AB_INTSTS_OFFSET, INT_MW_RD_ERR);
		tmpa910_udp_write(udc, UD2AB_UDMSTSET_OFFSET, UDC2AB_MW_RESET);
		return;
	}
	
	if( g_USB_Rbuf_Status == RRBST_NORMAL ){

		//Here we've got the data and then begin bulk-out packet analysis
//		usb_bulkout_mass_storage();

		//Trigger next phase data transfer
		USB_Bulk_Out_Start();
	}

	return;
}

static inline void create_debug_file(struct tmpa910_udc *udc) {}
static inline void remove_debug_file(struct tmpa910_udc *udc) {}

/* reinit == restore inital software state */
static void udc_reinit(struct tmpa910_udc *udc)
{
        u32 i;

        INIT_LIST_HEAD(&udc->gadget.ep_list);
        INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);

        for (i = 0; i < NUM_ENDPOINTS; i++) {
                struct tmpa910_ep *ep = &udc->ep[i];

                if (i != 0)
                        list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
                
		ep->desc = NULL;
                ep->stopped = 0;
                ep->fifo_bank = 0;
                ep->ep.maxpacket = ep->maxpacket;
		ep->creg = (void __iomem *) udc->udp_baseaddr + 0x230 + 0x10*i + 0x00C;
		
		// initialiser une queue par endpoint
                INIT_LIST_HEAD(&ep->queue);
        }
        DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);
}

static irqreturn_t tmpa910_udc_irq (int irq, void *_udc)
{
        struct tmpa910_udc *udc = &controller;
        g_Interrupt_Stasus = tmpa910_udp_read(udc, UD2AB_INTSTS_OFFSET);      /* read UDC Interrupt Register. */
                
        /* Select Interrupt Process. */
        if( ( g_Interrupt_Stasus & INT_SETUP ) == INT_SETUP ){
		USB_Int_Setup();
        }
        else if( ( g_Interrupt_Stasus & INT_DATA ) == INT_DATA ){
                USB_Int_Rx_Zero(); 
        }       
        else if( ( g_Interrupt_Stasus & INT_STATUS ) == INT_STATUS ){
		USB_Int_Status();
        }
        else if( ( g_Interrupt_Stasus & INT_STATUSNAK ) == INT_STATUSNAK ){
		USB_Int_Status_NAK();
        }
        else if( ( g_Interrupt_Stasus & INT_EP0 ) == INT_EP0 ){
                USB_Int_EP0();
        }
        else if( ( g_Interrupt_Stasus & INT_EP ) == INT_EP ){
                USB_Int_EPx();
        }
        else if( ( g_Interrupt_Stasus & INT_SOF ) == INT_SOF ){
                USB_Int_SOF();
        }
        else if( ( g_Interrupt_Stasus & INT_SUSPEND ) == INT_SUSPEND ){
                USB_Int_Supend_Resume();
        }
        else if( ( g_Interrupt_Stasus & INT_RESET ) == INT_RESET ){
                USB_Int_USB_Reset();
        }
        else if( ( g_Interrupt_Stasus & INT_RESET_END ) == INT_RESET_END ){
                USB_Int_USB_Reset_End();
        }
        else if( ( g_Interrupt_Stasus & INT_MW_END_ADD ) == INT_MW_END_ADD ){
                USB_Int_MW_End();
        }
        else if( ( g_Interrupt_Stasus & INT_MR_END_ADD ) == INT_MR_END_ADD ){
                USB_Int_MR_End();
        }
        else {
		DBG("In UDC irq, unexpected irq occured!\n");
        }

        if( ( g_Interrupt_Stasus & INT_MW_RD_ERR ) == INT_MW_RD_ERR ){
		DBG("In UDC irq, master write or read error found!\n");
                tmpa910_udp_write(udc, UD2AB_INTSTS_OFFSET, INT_MW_RD_ERR);
                tmpa910_udp_write(udc, UD2AB_UDMSTSET_OFFSET, UDC2AB_MW_RESET);
        }

       // VICADDRESS = 0x1234567;

	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/

int usb_gadget_register_driver (struct usb_gadget_driver *driver)
{
	struct tmpa910_udc	*udc;
	int			retval;

        DBG("usb_gadget_register_driver");

	udc = &controller;
	if (!driver	|| driver->speed < USB_SPEED_FULL
			|| !driver->bind
			|| !driver->setup) {
		DBG("bad parameter.\n");
		return -EINVAL;
	}

	if (udc->driver) {
		DBG("UDC already has a gadget driver\n");
		return -EBUSY;
	}

	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	udc->gadget.dev.driver_data = &driver->driver;
	udc->enabled = 1;
	udc->selfpowered = 1;

	retval = driver->bind(&udc->gadget);
	if (retval) {
		DBG("driver->bind() returned %d\n", retval);
		udc->driver = NULL;
		udc->gadget.dev.driver = NULL;
		udc->gadget.dev.driver_data = NULL;
		udc->enabled = 0;
		udc->selfpowered = 0;
		return retval;
	}

	local_irq_disable();
	//pullup(udc, 1);
	local_irq_enable();

	DBG("bound to %s\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL (usb_gadget_register_driver);

int usb_gadget_unregister_driver (struct usb_gadget_driver *driver)
{
	struct tmpa910_udc *udc;

        DBG("usb_gadget_unregister_driver");

	udc = &controller;
	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;

	local_irq_disable();
	udc->enabled = 0;
//	tmpa910_udp_write(udc, AT91_UDP_IDR, ~0);
	//pullup(udc, 0);
	local_irq_enable();

	driver->unbind(&udc->gadget);
	udc->driver = NULL;

	DBG("unbound from %s\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL (usb_gadget_unregister_driver);

/*-------------------------------------------------------------------------*/

static void tmpa910udc_shutdown(struct platform_device *dev)
{
	DBG("tmpa910udc_shutdown");
	//pullup(platform_get_drvdata(dev), 0);
}

static int __init tmpa910udc_probe(struct platform_device *pdev)
{
	unsigned long int reg_data;
        struct device   *dev;
        struct tmpa910_udc *udc;
        int             retval;
        struct resource *res;

	printk("tmpa910udc_probe\n");


        dev = &pdev->dev;

        USB_Ctl_Init();

        if (pdev->num_resources != 2) {
                DBG("invalid num_resources");
                return -ENODEV;
        }
        if ((pdev->resource[0].flags != IORESOURCE_MEM)
                        || (pdev->resource[1].flags != IORESOURCE_IRQ)) {
                DBG("invalid resource type");
                return -ENODEV;
        }
        
        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!res)
                return -ENXIO;
                
        if (!request_mem_region(res->start,
                        res->end - res->start + 1,
                        driver_name)) {
                DBG("someone's using UDC memory\n");
                return -EBUSY;
        }

	/* init software state */
        udc = &controller;
        udc->gadget.dev.parent = dev;
        udc->pdev = pdev;
        udc->enabled = 0;

        udc->udp_baseaddr = ioremap(res->start, res->end - res->start + 1);
        if (!udc->udp_baseaddr) {
                release_mem_region(res->start, res->end - res->start + 1);
                return -ENOMEM;
        }


		printk("udp_baseaddr=0x%x / phy = 0x%x\n", (int) udc->udp_baseaddr, (int) res->start);

	udc_reinit(udc);

        retval = device_register(&udc->gadget.dev);
        if (retval < 0)
                goto fail0;

#if 0
	reg_data = SYSCR0;
        reg_data &= 0x3f;
        reg_data |= (1<<6);                 		// [7:6]
        SYSCR0 = reg_data;			      	// USBCLKSEL = X1

        CLKCR4 = USB_ENABLE;     			/* EN_USB */
#endif
	reg_data = tmpa910_udp_read(udc, UD2AB_PWCTL_OFFSET);
 	reg_data &= PWCTL_PHY_POWER_RESET_ON;		// [5][1] <= 0
 	reg_data |= PWCTL_PHY_SUSPEND_ON;		// [3] <= 1
 	tmpa910_udp_write(udc, UD2AB_PWCTL_OFFSET, reg_data);
	mdelay(1);

	reg_data = tmpa910_udp_read(udc, UD2AB_PWCTL_OFFSET);
        reg_data |= PWCTL_PHY_RESET_OFF;            	// [5][3] <= 1
        tmpa910_udp_write(udc, UD2AB_PWCTL_OFFSET, reg_data);
        mdelay(1);

	reg_data = tmpa910_udp_read(udc, UD2AB_PWCTL_OFFSET);
        reg_data &= PWCTL_PHY_SUSPEND_OFF;          // [3] <= 0
        tmpa910_udp_write(udc, UD2AB_PWCTL_OFFSET, reg_data);
        mdelay(1);
        mdelay(1);
        
	reg_data = tmpa910_udp_read(udc, UD2AB_PWCTL_OFFSET);
	reg_data |= PWCTL_POWER_RESET_OFF;          // [2] <= 1
        tmpa910_udp_write(udc, UD2AB_PWCTL_OFFSET, reg_data);
        mdelay(1);
        mdelay(1);
        mdelay(1);
       
	UDC2_Reg_Write(UD2INT_OFFSET,UDC2_INT_MASK);   /*INT EPx MASK & Refresh; */

        tmpa910_udp_write(udc, UD2AB_INTSTS_OFFSET, UDC2AB_INT_ALL_CLEAR);
        tmpa910_udp_write(udc, UD2AB_INTENB_OFFSET, UDC2AB_INT_MASK);
        tmpa910_udp_write(udc, UD2AB_UDMSTSET_OFFSET, UDC2AB_MR_RESET | UDC2AB_MW_RESET);

	/* request UDC and maybe VBUS irqs */
        udc->udp_irq = platform_get_irq(pdev, 0);
        if (request_irq(udc->udp_irq, tmpa910_udc_irq,
                        IRQF_DISABLED, driver_name, udc)) {
                DBG("request irq %d failed\n", udc->udp_irq);
                retval = -EBUSY;
                goto fail1;
        }

	UDC2_Reg_Write(UD2CMD_OFFSET, All_EP_INVALID);

        UDC2_Reg_Write(UD2CMD_OFFSET, USB_READY);  // Pull-Up of DP will be made only after this command is issued,

	DBG("Function: %s, Line: %d\n", __FUNCTION__, __LINE__);

        udc->vbus = 1;

        dev_set_drvdata(dev, udc);
        device_init_wakeup(dev, 1);
        create_debug_file(udc);

        INFO("%s version %s\n", driver_name, DRIVER_VERSION);

	return 0;

fail1:
        device_unregister(&udc->gadget.dev);
fail0:
        release_mem_region(res->start, res->end - res->start + 1);
        return retval;
}

#define tmpa910udc_remove	NULL
#define	tmpa910udc_suspend	NULL
#define	tmpa910udc_resume	NULL

static struct platform_driver tmpa910_udc_driver = {
	.remove		= __exit_p(tmpa910udc_remove),
	.shutdown	= tmpa910udc_shutdown,
	.suspend	= tmpa910udc_suspend,
	.resume		= tmpa910udc_resume,
	.driver		= {
	.name	= (char *) driver_name,
	.owner	= THIS_MODULE,
	},
};

static int __init udc_init_module(void)
{
        printk("udc_init_module\n");
        return platform_driver_probe(&tmpa910_udc_driver, tmpa910udc_probe);
}
module_init(udc_init_module);

static void __exit udc_exit_module(void)
{
        printk("udc_exit_module\n");
	platform_driver_unregister(&tmpa910_udc_driver);
}
module_exit(udc_exit_module);

MODULE_DESCRIPTION("TMPA910 udc driver");
MODULE_AUTHOR("Wang Liguang");
MODULE_LICENSE("GPL");
