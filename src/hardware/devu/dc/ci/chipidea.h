/*
 * $QNXLicenseC: 
 * Copyright 2008, QNX Software Systems.  
 *  
 * Licensed under the Apache License, Version 2.0 (the "License"). You  
 * may not reproduce, modify or distribute this software except in  
 * compliance with the License. You may obtain a copy of the License  
 * at: http://www.apache.org/licenses/LICENSE-2.0  
 *  
 * Unless required by applicable law or agreed to in writing, software  
 * distributed under the License is distributed on an "AS IS" basis,  
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied. 
 * 
 * This file may contain contributions from others, either as  
 * contributors under the License or as licensors under other terms.   
 * Please review this entire file for other proprietary rights or license  
 * notices, as well as the QNX Development Suite License Guide at  
 * http://licensing.qnx.com/license-guide/ for other information. 
 * $
 */

/* 
	Header file for the ChipIdea USB OTG Device Controller
*/

#ifndef _CHIP_IDEA_OTG_H_INCLUDED
#define _CHIP_IDEA_OTG_H_INCLUDED

#include <stdio.h>
#include <stdint.h>
#include <sys/cache.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/slog.h>
#include <hw/inout.h>
#include <sys/mman.h>
#include <gulliver.h>

#include <sys/io-usb_dcd.h>
#include <queue.h>

/* 
 * Endpoint Queue Head data struct
 * Controller requires address to 64 byte aligned
*/
typedef struct _chip_idea_qh {
	uint32_t		max_pkt_length;		/* Mult(31-30), Zlt(29), Max Pkt len and IOS(15) 			*/
	uint32_t		curr_dtd_ptr; 		/* Current dTD Pointer(31-5) 								*/
	uint32_t 		next_dtd_ptr;		/* Next dTD Pointer(31-5), T(0) 							*/
	uint32_t 		size_ioc_int_sts;	/* Total bytes (30-16), IOC (15), MultO(11-10), STS (7-0)  	*/
	uint32_t		buff_ptr0;			/* Buffer pointer Page 0 (31-0) 							*/
	uint32_t		buff_ptr1;			/* Buffer pointer Page 1 (31-12) 							*/
	uint32_t		buff_ptr2;			/* Buffer pointer Page 2 (31-12) 							*/
	uint32_t		buff_ptr3;			/* Buffer pointer Page 3 (31-12) 							*/
	uint32_t 		buff_ptr4;			/* Buffer pointer Page 4 (31-12) 							*/
	uint32_t 		reserved;
	uint8_t			setup_buffer[8];	/* Setup data 8 bytes */
	uint32_t		reserved2[4]; 		/* reserved - Must be 64 byte Aligned  */
} chip_idea_qh_t;

/* Endpoint Queue Head Bit Masks */
#define  EP_QUEUE_HEAD_MULT_POS               		30
#define  EP_QUEUE_HEAD_ZLT_SEL                		0x20000000
#define  EP_QUEUE_HEAD_MAX_PKT_LEN_POS        		16
#define  EP_QUEUE_HEAD_MAX_PKT_LEN(ep_info)   		(((ep_info)>>16)&0x07ff)
#define  EP_QUEUE_HEAD_IOS                    		0x00008000
#define  EP_QUEUE_HEAD_NEXT_TERMINATE         		0x00000001
#define  EP_QUEUE_HEAD_IOC                    		0x00008000
#define  EP_QUEUE_HEAD_MULTO                  		0x00000C00
#define  EP_QUEUE_HEAD_STATUS_HALT	      			0x00000040
#define  EP_QUEUE_HEAD_STATUS_ACTIVE          		0x00000080
#define  EP_QUEUE_CURRENT_OFFSET_MASK         		0x00000FFF
#define  EP_QUEUE_HEAD_NEXT_POINTER_MASK     		0xFFFFFFE0
#define  EP_QUEUE_FRINDEX_MASK                		0x000007FF
#define  EP_MAX_LENGTH_TRANSFER               		0x4000

/*
 * Endpoint Transfer Descriptor data struct 
*/
typedef struct _chip_idea_dtd {
	uint32_t 		next_td_ptr;	/* Next dTD pointer(31-5), T(0) set  indicate invalid 		*/
	uint32_t	 	size_ioc_sts;	/* Total bytes (30-16), IOC (15), MultO(11-10), STS (7-0)  	*/
	uint32_t 		buff_ptr0;		/* Buffer pointer Page 0 									*/
	uint32_t	 	buff_ptr1;		/* Buffer pointer Page 1 									*/
	uint32_t	 	buff_ptr2;		/* Buffer pointer Page 2 									*/
	uint32_t		buff_ptr3;		/* Buffer pointer Page 3 									*/
	uint32_t		buff_ptr4;		/* Buffer pointer Page 4 									*/
	uint32_t		res;			/* make it an even 8 Dwords 								*/
} chip_idea_dtd_t;

/*
 * Endpoint Transfer Descriptor bit Masks 
*/
#define  DTD_NEXT_TERMINATE                		0x00000001
#define  DTD_IOC                             	0x00008000
#define  DTD_STATUS_ACTIVE                   	0x00000080
#define  DTD_STATUS_HALTED                  	0x00000040
#define  DTD_STATUS_DATA_BUFF_ERR     			0x00000020
#define  DTD_STATUS_TRANSACTION_ERR        		0x00000008
#define  DTD_RESERVED_FIELDS                 	0x80007300
#define  DTD_ADDR_MASK                        	0xFFFFFFFE
#define  DTD_PACKET_SIZE                      	0x7FFF0000
#define  DTD_LENGTH_BIT_POS                   	16
#define  DTD_ERROR_MASK                       	(DTD_STATUS_HALTED | DTD_STATUS_DATA_BUFF_ERR | DTD_STATUS_TRANSACTION_ERR)


#define CHIP_IDEA_EP_DIR_IN   1
#define CHIP_IDEA_EP_DIR_OUT  0

/*
 * Endpoints
*/
 #define USB_ENDPOINT_NUMBER_MASK			0x0f    /* in bEndpointAddress */
 #define USB_ENDPOINT_DIR_MASK           	0x80
 
 #define USB_ENDPOINT_XFERTYPE_MASK			0x03    /* in bmAttributes */
 #define USB_ENDPOINT_XFER_CONTROL       	0
 #define USB_ENDPOINT_XFER_ISOC          	1
 #define USB_ENDPOINT_XFER_BULK          	2
 #define USB_ENDPOINT_XFER_INT           	3

#define CHIP_IDEA_EP0_STAGE_STATUS_ADDR   	1

/*
 * internal used help routines.
*/
#define ep_index(EP)			((EP)->edesc.bEndpointAddress & 0xF)
#define ep_maxpacket(EP)     	((EP)->->edesc.wMaxPacketSize)


#define ep_is_in(EP)			((ep_index(EP) == 0) ? (EP->udc->ep0_dir == \
									CHIP_IDEA_EP_DIR_IN ):((EP)->edesc.bEndpointAddress  \
									& 0x80)== CHIP_IDEA_EP_DIR_IN)


#define chip_idea_pthread_mutex_lock(_mutex) do {                             \
        if ( pthread_mutex_lock( _mutex )) {                                \
            fprintf( stderr, "mutex lock %s %d\n", __FILE__, __LINE__ );    \
        }                                                                   \
    } while(0) 


#define chip_idea_pthread_mutex_unlock(_mutex) do {                           \
        if ( pthread_mutex_unlock( _mutex )) {                              \
            fprintf( stderr, "mutex unlock %s %d\n", __FILE__, __LINE__ );  \
        }                                                                   \
    } while(0) 

typedef struct _chip_idea_isoch_urb {
   SIMPLEQ_ENTRY(_chip_idea_isoch_urb) link;
   iousb_transfer_t                    *urb;
   usbd_isoch_frame_request_t          *isoch_flist;
   int                                 isoch_nframes;
   int                                 isoch_frame_idx;
   int                                 status;
   uint32_t                            flags;
} chip_idea_isoch_urb_t;

typedef struct _chip_idea_endpoint {
	SIMPLEQ_HEAD(, _chip_idea_isoch_urb) urb_pending_q;   /* Queue of pending URBs to be transfered */
	SIMPLEQ_HEAD(, _chip_idea_isoch_urb) urb_free_q;      /* Queue of free URBs */
	void					*urb_mem;
	iousb_endpoint_t 		*ep;			/* back pointer 					*/ 
	uint16_t				ep_num; 		/* endpoint number i				*/
	uint16_t 		    	dir; 			/* direction 1 - in, 0 - out 		*/
	uint32_t				prime_bit;		/* prime bit mask for ep			*/
	uint32_t				type;			/* endpoint type					*/
	uint32_t				mps;			/* Endpoint max packet size 		*/

	/* Static Members - set during initialization */ 
	chip_idea_qh_t 			*qh; 					
	chip_idea_dtd_t			*dtd_head;			/* dTD link list head 				*/
	uint32_t 				dtd_addr; 			/* physical address of dtd_head 	*/

	/* ISOCH specific members - set during endpoint_init */
	chip_idea_dtd_t         *isoch_dtds;        /* ISOCH dTD list pointer */
	uint32_t                isoch_dtd_head;     /* ISOCH dTD head index (we complete from the head) */
	uint32_t                isoch_dtd_tail;     /* ISOCH dTD tail index	(we schedule from the tail) */
	uint32_t                isoch_dtd_addr;     /* physical address of ISOCH dTD list */
	void                    *isoch_dtd_mem;
	size_t                  isoch_dtd_mem_sz;

	/* Set during transfer */
	uint32_t 				dtd_count;     	/* count of active dtd entries(1-4) */
	uint32_t 				transfer_len;  	/* total transfer length 			*/
	uint32_t				flags;
	iousb_transfer_t 		*active_urb;
	pthread_mutex_t			mutex;
} chip_idea_endpoint_t;


typedef struct _otg_transceiver {
	uint32_t     		I2CBase;
	int                	chid;
	int                	coid;
	int					irq;
	int					iid;
	int					tid;
	int					tstate;
	int					i2c_fd;
	struct sigevent     event;
} OTG_TRANSCEIVER;

typedef enum {
	USBPHY_PARAMETER_OVERRIDE_A,
	USBPHY_PARAMETER_OVERRIDE_B,
	USBPHY_PARAMETER_OVERRIDE_C,
	USBPHY_PARAMETER_OVERRIDE_D,
	USBPHY_MAX_REG
} phy_tuning_t;

typedef struct _chip_ideadc {
	usbdc_device_t 			*udc;
	uint32_t     			IoBase;
	pthread_mutex_t			usb_mutex;

	uint32_t				flags;
	int						verbosity;
	int						int_thresh;

	void					*qh_mem;
	size_t					qh_mem_sz;
	void					*dtd_mem;
	size_t					dtd_mem_sz;
	int						num_ep;
	chip_idea_qh_t 	 		*ep_qh;			/* Endpoints Queue-Head */
	chip_idea_endpoint_t	ep[32];	 		/* Core supports 16 bidirectional Rx=0-15,TX=16-31, actual implementation may be less */

	uint32_t				device_speed;
	char 					*serial_string;

	OTG_TRANSCEIVER			*otg;

	uint32_t				phy_tuning[USBPHY_MAX_REG];
	void                    *extra;                 /* Extra area for variant specific data */
	int						reset_state;
	int                     itds;    /* Max isoch dTDs per endpoint */
} chip_ideadc;

/* internal defines */
#define CHIP_IDEA_MAX_PRIME_RETRIES					4

/* defines for flags */
#define CHIP_IDEA_FLAGS_INITIAL_LINK_DISCONNECTED	0x1

extern ssize_t chip_idea_slogf( chip_ideadc *dc, int verbosity, const char *fmt, ... );

#include <chipidea_bs.h>

/* Controller Initialization defines */
#define CHIP_IDEA_QH_ALIGN					0x800
#define CHIP_IDEA_MAX_DTD_PER_ENDPOINT		4
#define MAX_ISOCH_DTD_PER_ENDPOINT          512

#define CHIP_IDEA_MAX_TRANSFER_SIZE 		(64 * 1024)
#define CHIP_IDEA_MAX_LINKED_DTD			4

/* Controller information */
#define UDC_SIZE							0x300  
#define USB_MAX_CTRL_PAYLOAD				64

#define CHIP_IDEA_UOG_ID					0x000
#define CHIP_IDEA_UOG_HWDEVICE				0x00C


/* Controller register/bit defines */
#define CHIP_IDEA_USB_CTRL		        		0x600		/* offset of USB control register from USBOTG base */
#define CHIP_IDEA_USB_OTG_MIRROR     			0x604		/* offset of USB control register from USBOTG base */

#define UCTRL_OWIR          (1 << 31)       /* OTG wakeup intr request received */
#define UCTRL_OSIC_MASK     (3 << 29)       /* OTG  Serial Interface Config: */
#define UCTRL_OSIC_DU6      (0 << 29)       /* Differential/unidirectional 6 wire */
#define UCTRL_OSIC_DB4      (1 << 29)       /* Differential/bidirectional  4 wire */
#define UCTRL_OSIC_SU6      (2 << 29)       /* single-ended/unidirectional 6 wire */
#define UCTRL_OSIC_SB3      (3 << 29)       /* single-ended/bidirectional  3 wire */

#define UCTRL_OUIE          (1 << 28)       /* OTG ULPI intr enable */
#define UCTRL_OWIE          (1 << 27)       /* OTG wakeup intr enable */
#define UCTRL_OBPVAL_RXDP   (1 << 26)       /* OTG RxDp status in bypass mode */
#define UCTRL_OBPVAL_RXDM   (1 << 25)       /* OTG RxDm status in bypass mode */
#define UCTRL_OPM           (1 << 24)       /* OTG power mask */
#define UCTRL_H2WIR         (1 << 23)       /* HOST2 wakeup intr request received */
#define UCTRL_H2SIC_MASK    (3 << 21)       /* HOST2 Serial Interface Config: */
#define UCTRL_H2SIC_DU6     (0 << 21)       /* Differential/unidirectional 6 wire */
#define UCTRL_H2SIC_DB4     (1 << 21)       /* Differential/bidirectional  4 wire */
#define UCTRL_H2SIC_SU6     (2 << 21)       /* single-ended/unidirectional 6 wire */
#define UCTRL_H2SIC_SB3     (3 << 21)       /* single-ended/bidirectional  3 wire */


#define UCTRL_BPE           (1 <<  0)       /* bypass mode enable */


#define CHIP_IDEA_UOG_DCCPARAMS					0x124
	#define CHIP_IDEA_MASK_DEN				0x1f			/* Number of endpoints supported 	*/
	#define CHIP_IDEA_DEVICE_CAPABLE		0x80			/* Is capable ofe Device Mode		*/
	#define CHIP_IDEA_HOST_CAPABLE			0x100			/* Is capable of Host Mode 			*/

#ifndef CHIP_IDEA_UOG_USBOP_BASE
#define	CHIP_IDEA_UOG_USBOP_BASE				0x140		/* USB Operation Register Base*/
#endif
#define	CHIP_IDEA_UOG_USBCMD				(CHIP_IDEA_UOG_USBOP_BASE + 0x0) /* USB Command Register */

	/* USB Command  Register Bit Masks */
	#define USB_CMD_RUN_STOP				(1<<0)
	#define USB_CMD_CTRL_RESET				(1<<1)
	#define USB_CMD_PERIODIC_SCHEDULE_EN	(1<<4)
	#define USB_CMD_ASYNC_SCHEDULE_EN		(1<<5)
	#define USB_CMD_INT_AA_DOORBELL			(1<<6)
	#define USB_CMD_ASP						(0x3<<8)
	#define USB_CMD_ASYNC_SCH_PARK_EN		(1<<11)
#ifndef USB_CMD_ATDTW 
	#define USB_CMD_ATDTW					(1<<12)
#endif
	#define USB_CMD_SUTW					(1<<13)
	#define USB_CMD_ITC						(0xFF<<16)

	/* bit 15,3,2 are frame list size */
	#define USB_CMD_FRAME_SIZE_1024			(0<<15 | 0<<2)
	#define USB_CMD_FRAME_SIZE_512			(0<<15 | 1<<2)
	#define USB_CMD_FRAME_SIZE_256			(0<<15 | 2<<2)
	#define USB_CMD_FRAME_SIZE_128			(0<<15 | 3<<2)
	#define USB_CMD_FRAME_SIZE_64			(1<<15 | 0<<2)
	#define USB_CMD_FRAME_SIZE_32			(1<<15 | 1<<2)
	#define USB_CMD_FRAME_SIZE_16			(1<<15 | 2<<2)
	#define USB_CMD_FRAME_SIZE_8			(1<<15 | 3<<2)

	/* bit 9-8 are async schedule park mode count */
	#define USB_CMD_ASP_00				(0<<8)
	#define USB_CMD_ASP_01				(1<<8)
	#define USB_CMD_ASP_10				(2<<8)
	#define USB_CMD_ASP_11				(3<<8)
	#define USB_CMD_ASP_BIT_POS			(8)

	/* bit 23-16 are interrupt threshold control */
	#define USB_CMD_ITC_MASK			(0xff<<16)
	#define USB_CMD_ITC_NO_THRESHOLD		(0x00<<16)
	#define USB_CMD_ITC_1_MICRO_FRM			(0x01<<16)
	#define USB_CMD_ITC_2_MICRO_FRM			(0x02<<16)
	#define USB_CMD_ITC_4_MICRO_FRM			(0x04<<16)
	#define USB_CMD_ITC_8_MICRO_FRM			(0x08<<16)
	#define USB_CMD_ITC_16_MICRO_FRM		(0x10<<16)
	#define USB_CMD_ITC_32_MICRO_FRM		(0x20<<16)
	#define USB_CMD_ITC_64_MICRO_FRM		(0x40<<16)
	#define USB_CMD_ITC_BIT_POS				16
	
	
#define	CHIP_IDEA_UOG_USBSTS			(CHIP_IDEA_UOG_USBOP_BASE+4)	/* USB Status Register */
	/* USB Status Register Bit Masks */
	#define USB_STS_INT					(1<<0)
	#define USB_STS_ERR					(1<<1)
	#define USB_STS_PORT_CHANGE			(1<<2)
	#define USB_STS_FRM_LST_ROLL		(1<<3)
	#define USB_STS_SYS_ERR				(1<<4)
	#define USB_STS_IAA					(1<<5)
	#define USB_STS_RESET_RECEIVED		(1<<6)
	#define USB_STS_SOF					(1<<7)
	#define USB_STS_SUSPEND				(1<<8)
	#define USB_STS_HC_HALTED			(1<<12)
	#define USB_STS_RCL					(1<<13)
	#define USB_STS_PERIODIC_SCHEDULE	(1<<14)
	#define USB_STS_ASYNC_SCHEDULE		(1<<15)

#define	CHIP_IDEA_UOG_USBINTR			(CHIP_IDEA_UOG_USBOP_BASE+0x8)	/* Interrupt Enable Register */
	/* USB INTR Register Bit Masks */
	#define  USB_INTR_INT_EN						(1<<0)
	#define  USB_INTR_ERR_INT_EN                  	(1<<1)
	#define  USB_INTR_PTC_DETECT_EN               	(1<<2)
	#define  USB_INTR_FRM_LST_ROLL_EN             	(1<<3)
	#define  USB_INTR_SYS_ERR_EN                  	(1<<4)
	#define  USB_INTR_ASYN_ADV_EN                 	(1<<5)
	#define  USB_INTR_RESET_EN                    	(1<<6)
	#define  USB_INTR_SOF_EN                      	(1<<7)
	#define  USB_INTR_DEVICE_SUSPEND              	(1<<8)

#define CHIP_IDEA_DEFAULT_IRQ_EN_MASK 			(USB_INTR_INT_EN | USB_INTR_ERR_INT_EN | USB_INTR_PTC_DETECT_EN |\
													USB_INTR_RESET_EN |  USB_INTR_DEVICE_SUSPEND | USB_INTR_SYS_ERR_EN)

#define	CHIP_IDEA_UOG_FRINDEX			(CHIP_IDEA_UOG_USBOP_BASE+0xc)	/* USB Frame Index Register */
#define	CHIP_IDEA_UOG_DEVICEADDR		(CHIP_IDEA_UOG_USBOP_BASE+0x14)	/* device mode: device address Register */
	/* Device Address bit masks */
	#define USB_DEVICE_ADDRESS_MASK		0xFE000000
	#define USB_DEVICE_ADDRESS_BIT_POS	25  
	#define USB_DEVICE_ADDRESS_USBADRA  (1<<24)

#define	CHIP_IDEA_UOG_ENDPOINTLISTADDR	(CHIP_IDEA_UOG_USBOP_BASE+0x18)	/* device mode: endpoint list address Register */
	/* endpoint list address bit masks */
	#define USB_EP_LIST_ADDRESS_MASK        0xFFFFF800

#define	CHIP_IDEA_UOG_ULPIVIEW			(CHIP_IDEA_UOG_USBOP_BASE+0x30)	/* ULPI Viewport Register */
#define	CHIP_IDEA_UOG_CFGFLAG			(CHIP_IDEA_UOG_USBOP_BASE+0x40)	/* Config Flag Register */
#ifndef CHIP_IDEA_UOG_PORTSCX
#define	CHIP_IDEA_UOG_PORTSCX			(CHIP_IDEA_UOG_USBOP_BASE+0x44)	/* Port Status and Control Register*/
#endif
	/* PORTSCX  Register Bit Masks */
	#define  PORTSCX_CURRENT_CONNECT_STATUS      	(1<<0)
	#define  PORTSCX_CONNECT_STATUS_CHANGE  		(1<<1)
	#define  PORTSCX_PORT_ENABLE                  	(1<<2)
	#define  PORTSCX_PORT_EN_DIS_CHANGE           	(1<<3)
	#define  PORTSCX_OVER_CURRENT_ACT             	(1<<4)
	#define  PORTSCX_OVER_CURRENT_CHG             	(1<<5)
	#define  PORTSCX_PORT_FORCE_RESUME            	(1<<6)
	#define  PORTSCX_PORT_SUSPEND                 	(1<<7)
	#define  PORTSCX_PORT_RESET                   	(1<<8)
	#define  PORTSCX_LINE_STATUS_BITS             	(0x3<<10)
	#define  PORTSCX_PORT_POWER                   	(1<12)
	#define  PORTSCX_PORT_INDICTOR_CTRL           	(0x3<<14)
	#define  PORTSCX_PORT_TEST_CTRL               	(0xF<<16)
	#define  PORTSCX_WAKE_ON_CONNECT_EN           	(1<<20)
	#define  PORTSCX_WAKE_ON_CONNECT_DIS          	(1<<21)
	#define  PORTSCX_WAKE_ON_OVER_CURRENT         	(1<<22)
	#define  PORTSCX_PHY_LOW_POWER_SPD            	(1<<23)
	#define  PORTSCX_PORT_FORCE_FULL_SPEED        	(1<<24)
	#define  PORTSCX_PORT_SPEED_MASK              	(0x3<<26)
	#define  PORTSCX_PORT_WIDTH                   	(1<<28)
	#define  PORTSCX_PHY_TYPE_SEL                 	(0x3<<30)

	/* bit 11-10 are line status */
	#define  PORTSCX_LINE_STATUS_SE0 	            0
	#define  PORTSCX_LINE_STATUS_JSTATE           	(0x01<<10)
	#define  PORTSCX_LINE_STATUS_KSTATE           	(0x2<10)
	#define  PORTSCX_LINE_STATUS_UNDEF            	(0x3<<10)
	#define  PORTSCX_LINE_STATUS_BIT_POS          	10

	/* bit 15-14 are port indicator control */
	#define  PORTSCX_PIC_OFF                      		0
	#define  PORTSCX_PIC_AMBER                    		(0x01<<14)
	#define  PORTSCX_PIC_GREEN                    		(0x2<<14)
	#define  PORTSCX_PIC_UNDEF                    		(0x3<<14)
	#define  PORTSCX_PIC_BIT_POS                  		14
	
	/* bit 19-16 are port test control */
	#define  PORTSCX_PTC_DISABLE                  		0
	#define  PORTSCX_PTC_JSTATE                   		(0x1<<16)
	#define  PORTSCX_PTC_KSTATE                   		(0x2<<16)
	#define  PORTSCX_PTC_SEQNAK                   		(0x3<<16)
	#define  PORTSCX_PTC_PACKET                   		(0x4<<16)
	#define  PORTSCX_PTC_FORCE_EN                 		(0x5<<16)
	#define  PORTSCX_PTC_BIT_POS                  		16


	/* bit 27-26 are port speed */
	#define  PORTSCX_PORT_SPEED_FULL             		(0x00)
	#define  PORTSCX_PORT_SPEED_LOW               		(0x01)
	#define  PORTSCX_PORT_SPEED_HIGH              		(0x02)
	#define  PORTSCX_PORT_SPEED_UNDEF             		(0x03)
	#define  PORTSCX_SPEED_BIT_POS                		(26)
	
	/* bit 28 is parallel transceiver width for UTMI interface */
	#define  PORTSCX_PTW                          		(1<<28)
	#define  PORTSCX_PTW_8BIT                     		0
	#define  PORTSCX_PTW_16BIT                    		(1<<28)
	
	/* bit 31-30 are port transceiver select */
	#define  PORTSCX_PTS_UTMI                     		0
	#define  PORTSCX_PTS_ULPI                     		(0x2<<30)
	#define  PORTSCX_PTS_FSLS                     		(0x3<<30)
	#define  PORTSCX_PTS_BIT_POS                  		(30)
	
#ifndef CHIP_IDEA_UOG_OTGSC
#define	CHIP_IDEA_UOG_OTGSC				(CHIP_IDEA_UOG_USBOP_BASE+0x64)	/* OTG Status and Control Register */
#endif
	#define OTG_VBUS_DISCHARGE				(0x1<<0)
	#define OTG_VBUS_CHARGE					(0x1<<1)
	#define OTG_HAAR						(0x1<<2)
	#define OTG_OT							(0x1<<3)
	#define OTG_DP							(0x1<<4)
	#define OTG_IDPU						(0x1<<5)
	#define OTG_HADP						(0x1<<6)
	#define OTG_HABA						(0x1<<7)
	#define OTG_ID							(0x1<<8)
	#define OTG_AVV							(0x1<<9)
	#define OTG_ASV							(0x1<<10)
	#define OTG_BSV							(0x1<<11)
	#define OTG_BSE							(0x1<<12)
	#define OTG_1MST						(0x1<<13)
	#define OTG_DPS							(0x1<<14)
	#define OTG_RESERVED1					(0x1<<15)
	#define OTG_IDIS						(0x1<<16)
	#define OTG_AVVIS						(0x1<<17)
	#define OTG_ASVIS						(0x1<<18)
	#define OTG_BSVIS						(0x1<<19)
	#define OTG_BSEIS						(0x1<<20)
	#define OTG_1MSS						(0x1<<21)
	#define OTG_DPIS						(0x1<<22)
	#define OTG_RESERVED2					(0x1<<23)
	#define OTG_IDIE						(0x1<<24)
	#define OTG_AVVIE						(0x1<<25)
	#define OTG_ASVIE						(0x1<<26)
	#define OTG_BSVIE						(0x1<<27)
	#define OTG_BSEIE						(0x1<<28)
	#define OTG_1MSE						(0x1<<29)
	#define OTG_DPEI						(0x1<<30)
	#define OTG_RESERVED3					(0x1<<31)

#define CHIP_IDEA_OTG_IRQ_EN_MASK 					(OTG_IDIE | OTG_AVVIE | OTG_ASVIE | OTG_BSVIE | OTG_BSEIE | OTG_1MSE | OTG_DPEI)
#define CHIP_IDEA_DEFAULT_OTG_IRQ_EN_MASK 			(OTG_IDIE | OTG_AVVIE | OTG_BSVIE | OTG_BSEIE )
#define CHIP_IDEA_DEFAULT_OTG_STATUS_MASK 			(OTG_IDIS | OTG_AVVIS | OTG_ASVIS | OTG_BSVIS | OTG_BSEIS | OTG_1MSS | OTG_DPIS )


#define	CHIP_IDEA_UOG_USBMODE				0x1A8	/* USB Device Mode */
	/* USB MODE Register Bit Masks */
	#define  USB_MODE_CTRL_MODE_IDLE		(0x0<<0)
	#define  USB_MODE_CTRL_MODE_DEVICE		(0x2<<0)
	#define  USB_MODE_CTRL_MODE_HOST		(0x3<<0)
	#define  USB_MODE_CTRL_MODE_RSV			(0x1<<0)
	#define  USB_MODE_ENDIAN_SELECT			(1<<2)
	#define  USB_MODE_SETUP_LOCK_OFF		(1<<3)
	#define  USB_MODE_STREAM_DISABLE		(1<<4)

#ifndef CHIP_IDEA_ENDPREG_BASE
#define CHIP_IDEA_ENDPREG_BASE				0x1AC	/*Endpoint Register Base */
#endif
#define	CHIP_IDEA_UOG_ENDPTSETUPSATA		(CHIP_IDEA_ENDPREG_BASE+0x0)	/* Endpoint Setup Status Register */
	/* Endpoint Setup Status bit masks */
	#define  EP_SETUP_STATUS_MASK              	(0x0000003F)
	#define  EP_SETUP_STATUS_EP0		      	(0x00000001)

#define	CHIP_IDEA_UOG_ENDPTPRIME			(CHIP_IDEA_ENDPREG_BASE+0x4)	/* Endpoint Initialization Register*/
#define	CHIP_IDEA_UOG_ENDPTFLUSH			(CHIP_IDEA_ENDPREG_BASE+0x8)	/* Endpoint De-Initialization Register*/
	/* Endpoint Flush Register */
	#define EPFLUSH_TX_OFFSET	 				(0X00010000)
	#define EPFLUSH_RX_OFFSET		      		(0X00000000)
	
#define	CHIP_IDEA_UOG_ENDPTSTAT			(CHIP_IDEA_ENDPREG_BASE+0xc)	/* Endpoint Status Register */
#define	CHIP_IDEA_UOG_ENDPTCOMPLETE		(CHIP_IDEA_ENDPREG_BASE+0x10)	/* Endpoint Complete Register */

#define	CHIP_IDEA_UOG_ENDPTCTRL0		(CHIP_IDEA_ENDPREG_BASE+0x14)	/* Endpoint Control0 */
#define	CHIP_IDEA_UOG_ENDPTCTRL(x)		(CHIP_IDEA_UOG_ENDPTCTRL0 +(4*(x)))	/* Endpoint Control1 */
	/* ENDPOINTCTRLx  Register Bit Masks */ 
	#define  EPCTRL_TX_BITS_MASK				(0xffff0000)
	#define  EPCTRL_TX_ENABLE               	(0x00800000)
	#define  EPCTRL_TX_DATA_TOGGLE_RST         	(0x00400000)	/* Not EP0 */
	#define  EPCTRL_TX_DATA_TOGGLE_INH          (0x00200000)	/* Not EP0 */

	#define  EPCTRL_TX_EP_TYPE_MASK				(0x000C0000)
	#define  EPCTRL_TX_TYPE_CONTROL         	(0x00000000)
	#define  EPCTRL_TX_TYPE_ISOC				(0x00010000)
	#define  EPCTRL_TX_TYPE_BULK				(0x00080000)
	#define  EPCTRL_TX_TYPE_INTR            	(0x000C0000)

	#define  EPCTRL_TX_DATA_SOURCE              (0x00020000)	/* Not EP0 */
	#define  EPCTRL_TX_EP_STALL                 (0x00010000)

	#define  EPCTRL_RX_BITS_MASK				(0x0000ffff)
	#define  EPCTRL_RX_ENABLE                   (0x00000080)
	#define  EPCTRL_RX_DATA_TOGGLE_RST          (0x00000040)	/* Not EP0 */
	#define  EPCTRL_RX_DATA_TOGGLE_INH          (0x00000020)	/* Not EP0 */

	#define  EPCTRL_RX_EP_TYPE_MASK				(0x0000000C)
	#define  EPCTRL_RX_TYPE_CONTROL         	(0x00000000)
	#define  EPCTRL_RX_TYPE_ISOC            	(0x00000001)
	#define  EPCTRL_RX_TYPE_BULK            	(0x00000008)	
	#define  EPCTRL_RX_TYPE_INTR  		   		(0x0000000C)

	#define  EPCTRL_RX_DATA_SINK                (0x00000002)	/* Not EP0 */
	#define  EPCTRL_RX_EP_STALL                 (0x00000001)

	/* bit 19-18 and 3-2 are endpoint type */
	#define  EPCTRL_EP_TYPE_CONTROL               	(0)
	#define  EPCTRL_EP_TYPE_ISO                   	(1)
	#define  EPCTRL_EP_TYPE_BULK                  	(2)
	#define  EPCTRL_EP_TYPE_INTERRUPT             	(3)
	#define  EPCTRL_TX_EP_TYPE_SHIFT              	(18)
	#define  EPCTRL_RX_EP_TYPE_SHIFT              	(2)

/* CHIP IDEA EHCI controller specific register offset (from base address)*/
#define CHIP_IDEA_USBGENCTRL                      0x200
#define CHIP_IDEA_ISIPHYCTRL                      0x204
#define CHIP_IDEA_USB_MODE                        (CHIP_IDEA_UOG_USBMODE)
#define CHIP_IDEA_USB_OTGSC                       (CHIP_IDEA_UOG_OTGSC)

/* USBGENCTRL register bit definition */
#define CHIP_IDEA_USBGENCTRL_WU_IE                (1UL << 0)
#define CHIP_IDEA_USBGENCTRL_WU_ULPI_EN           (1UL << 1)
#define CHIP_IDEA_USBGENCTRL_PFP                  (1UL << 2)
#define CHIP_IDEA_USBGENCTRL_PPP                  (1UL << 3)
#define CHIP_IDEA_USBGENCTRL_ULPI_SEL             (1UL << 4)
#define CHIP_IDEA_USBGENCTRL_WU_INT_CLR           (1UL << 5)

/* ISIPHYCTRL register bit definition */
#define CHIP_IDEA_ISIPHYCTRL_PXE                  (1UL << 0)
#define CHIP_IDEA_ISIPHYCTRL_LSFE                 (1UL << 1)
#define CHIP_IDEA_ISIPHYCTRL_BSEN                 (1UL << 2)
#define CHIP_IDEA_ISIPHYCTRL_BSENH                (1UL << 3)
#define CHIP_IDEA_ISIPHYCTRL_PHYE                 (1UL << 4)
#define CHIP_IDEA_ISIPHYCTRL_OCO                  (1UL << 5)

/* USBMODE register bit definition */
#define CHIP_IDEA_USBMODE_CM(val)                 ((val & 3UL) << 0)
    #define CHIP_IDEA_USBMODE_CM_IDLE             (0UL << 0)
    #define CHIP_IDEA_USBMODE_CM_RESV             (1UL << 0)
    #define CHIP_IDEA_USBMODE_CM_DEVICE           (2UL << 0)
    #define CHIP_IDEA_USBMODE_CM_HOST             (3UL << 0)
#define CHIP_IDEA_USBMODE_ES                      (1UL << 2)
#define CHIP_IDEA_USBMODE_SLOM                    (1UL << 3)
#define CHIP_IDEA_USBMODE_SDIS                    (1UL << 4)
#define CHIP_IDEA_USBMODE_VBPS                    (1UL << 5)

/* USBOTGSC register bit definition */
#define CHIP_IDEA_USBOTGSC_OT                     (1UL << 3)
#define CHIP_IDEA_USBOTGSC_DP                     (1UL << 4)
#define CHIP_IDEA_USBOTGSC_IDPU                   (1UL << 5)
#define CHIP_IDEA_USBOTGSC_ID                     (1UL << 8)
#define CHIP_IDEA_USBOTGSC_AVV                    (1UL << 9)
#define CHIP_IDEA_USBOTGSC_ASV                    (1UL << 10)
#define CHIP_IDEA_USBOTGSC_BSV                    (1UL << 11)
#define CHIP_IDEA_USBOTGSC_BSE                    (1UL << 12)
#define CHIP_IDEA_USBOTGSC_1msT                   (1UL << 13)
#define CHIP_IDEA_USBOTGSC_DSP                    (1UL << 14)
#define CHIP_IDEA_USBOTGSC_IDIS                   (1UL << 16)
#define CHIP_IDEA_USBOTGSC_AVVIS                  (1UL << 17)
#define CHIP_IDEA_USBOTGSC_ASVIS                  (1UL << 18)
#define CHIP_IDEA_USBOTGSC_BSVIS                  (1UL << 19)
#define CHIP_IDEA_USBOTGSC_BSEIS                  (1UL << 20)
#define CHIP_IDEA_USBOTGSC_1msS                   (1UL << 21)
#define CHIP_IDEA_USBOTGSC_DSPIS                  (1UL << 22)
#define CHIP_IDEA_USBOTGSC_IDIE                   (1UL << 24)
#define CHIP_IDEA_USBOTGSC_AVVIE                  (1UL << 25)
#define CHIP_IDEA_USBOTGSC_ASVIE                  (1UL << 26)
#define CHIP_IDEA_USBOTGSC_BSVIE                  (1UL << 27)
#define CHIP_IDEA_USBOTGSC_BSEIE                  (1UL << 28)
#define CHIP_IDEA_USBOTGSC_1msE                   (1UL << 29)
#define CHIP_IDEA_USBOTGSC_DSPIE                  (1UL << 30)

#define EHCI_M_HOST_CONTOLLER_RESET             0x002

/* overidable defines */

#ifndef USBDC_DLL_NAME
	#define USBDC_DLL_NAME		"devu-chipidea.so"
#endif

#ifndef CHIP_IDEA_DEFAULT_CTRL_NAME
	#define CHIP_IDEA_DEFAULT_CTRL_NAME 	"CIDEA"
#endif

#ifndef CHIP_IDEA_MAX_NUM_USB_ENDPOINTS
	#define CHIP_IDEA_MAX_NUM_USB_ENDPOINTS		4
#endif

#ifndef CHIP_IDEA_BOARD_SPECIFIC_INIT
	#define CHIP_IDEA_BOARD_SPECIFIC_INIT
#endif

#ifndef CHIP_IDEA_BOARD_SPECIFIC_FINI
	#define CHIP_IDEA_BOARD_SPECIFIC_FINI
#endif

#ifndef CHIP_IDEA_BOARD_SPECIFIC_TRANSCEIVER_INIT
	#define CHIP_IDEA_BOARD_SPECIFIC_TRANSCEIVER_INIT
#endif

/* override default link control - default uses RUNSTOP bit */
#ifndef CHIP_IDEA_BOARD_SPECIFIC_LINK_UP
	#define CHIP_IDEA_BOARD_SPECIFIC_LINK_UP
#endif

#ifndef CHIP_IDEA_BOARD_SPECIFIC_LINK_DOWN 
	#define CHIP_IDEA_BOARD_SPECIFIC_LINK_DOWN
#endif

#ifndef CHIP_IDEA_BOARD_SPECIFIC_SYNC_FLUSH 
	#define CHIP_IDEA_BOARD_SPECIFIC_SYNC_FLUSH
#endif

#ifndef CHIP_IDEA_EXTRA_ALLOC_CALLOUT
    #define CHIP_IDEA_EXTRA_ALLOC_CALLOUT
#endif

#ifndef CHIP_IDEA_EXTRA_FREE_CALLOUT
    #define CHIP_IDEA_EXTRA_FREE_CALLOUT
#endif

#ifndef CHIP_IDEA_EXTRA_PROCESS_ARGS_CALLOUT
    #define CHIP_IDEA_EXTRA_PROCESS_ARGS_CALLOUT
#endif

#ifndef CHIP_IDEA_EXTRA_INTERRUPT_CALLOUT
    #define CHIP_IDEA_EXTRA_INTERRUPT_CALLOUT
#endif

#ifndef CHIPIDEA_GET_SPEED
#define	CHIPIDEA_GET_SPEED() ((in32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX ) & PORTSCX_PORT_SPEED_MASK)>>PORTSCX_SPEED_BIT_POS)
#endif

#ifndef CHIPIDEA_SET_MODE
#define CHIPIDEA_SET_MODE() (out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBMODE, in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBMODE ) | USB_MODE_CTRL_MODE_DEVICE | USB_MODE_SETUP_LOCK_OFF ))
#endif

/* define functions external called externally */
extern uint32_t 	chip_idea_init( usbdc_device_t *udc, io_usbdc_self_t *udc_self, char *args);
extern uint32_t 	chip_idea_start( usbdc_device_t *udc );
extern uint32_t 	chip_idea_stop( usbdc_device_t *udc );
extern uint32_t 	chip_idea_shutdown( usbdc_device_t *udc );
extern uint32_t 	chip_idea_set_bus_state( usbdc_device_t *udc, uint32_t device_state );
extern uint32_t 	chip_idea_set_device_feature( usbdc_device_t *udc, uint32_t feature, uint16_t index );
extern uint32_t 	chip_idea_clear_device_feature( usbdc_device_t *udc, uint32_t feature );

extern uint32_t 	chip_idea_set_device_address(  usbdc_device_t *udc, uint32_t device_address );
extern uint32_t 	chip_idea_select_configuration( usbdc_device_t *udc, uint8_t config );
extern uint32_t 	chip_idea_get_descriptor( usbdc_device_t *udc, uint8_t type, uint8_t index, uint16_t lang_id, uint8_t **ddesc, uint32_t speed );
extern uint32_t 	chip_idea_get_device_descriptor( usbdc_device_t *udc, uint8_t **ddesc, uint32_t speed );
extern uint32_t		chip_idea_get_config_descriptor( usbdc_device_t *udc, uint8_t **cdesc, uint8_t config_num, uint32_t speed );
extern uint32_t 	chip_idea_get_string_descriptor( usbdc_device_t *udc, uint8_t **sdesc, uint8_t index , uint32_t speed);

extern uint32_t 	chip_idea_set_endpoint_state( usbdc_device_t *udc, iousb_endpoint_t *ep, uint32_t ep_state );
extern uint32_t 	chip_idea_clear_endpoint_state( usbdc_device_t *udc, iousb_endpoint_t *ep, uint32_t ep_state );
extern uint32_t 	chip_idea_interrupt( usbdc_device_t *udc );
extern uint32_t 	chip_idea_endpoint_enable( void *chdl, iousb_device_t *device, iousb_endpoint_t *ep );
extern uint32_t 	chip_idea_endpoint_disable( void *chdl, iousb_endpoint_t *ep );
extern uint32_t 	chip_idea_transfer_abort( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *ED );
extern uint32_t 	chip_idea_transfer( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *endp, uint8_t *buffer, _uint32 length, _uint32 flags );

/* Isoch pipe methods */
extern uint32_t 	chip_idea_isoch_endpoint_enable( void *chdl, iousb_device_t *device, iousb_endpoint_t *ep );
extern uint32_t 	chip_idea_isoch_endpoint_disable( void *chdl, iousb_endpoint_t *ep );
extern uint32_t 	chip_idea_isoch_transfer_abort( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *ED );
extern uint32_t 	chip_idea_isoch_transfer( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *endp, uint8_t *buffer, _uint32 length, _uint32 flags );

extern paddr_t		mphys(const void *);
extern int 			chip_idea_otg_transceiver_init( chip_ideadc *dcctrl );
extern int 			chip_idea_board_specfic_init( chip_ideadc *dcctrl );

#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/devu/dc/ci/chipidea.h $ $Rev: 741231 $")
#endif
