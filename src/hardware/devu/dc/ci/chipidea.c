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
	Module : Hardware driver for ChipIdea/ARC/TDI - USB device controller
*/

#include <drvr/common.h>
#include <descriptors.h>
#include "chipidea.h"


extern char usbdc_config_descriptor[];

static iousb_pipe_methods_t chip_idea_pipe_methods = {
    chip_idea_endpoint_enable,
    chip_idea_endpoint_disable,
    chip_idea_transfer,
    chip_idea_transfer_abort,
    NULL
};

static iousb_pipe_methods_t chip_idea_isoch_pipe_methods = {
    chip_idea_isoch_endpoint_enable,
    chip_idea_isoch_endpoint_disable,
    chip_idea_isoch_transfer,
    chip_idea_isoch_transfer_abort,
    NULL
};

dc_methods_t chip_idea_controller_methods = {
					20,
					chip_idea_init,
					chip_idea_start,
					chip_idea_stop,
					chip_idea_shutdown,
					NULL,
					NULL,	
					chip_idea_set_bus_state,
					chip_idea_set_device_feature,
					chip_idea_clear_device_feature,
					chip_idea_set_device_address,
					chip_idea_get_descriptor,
					chip_idea_select_configuration,
					chip_idea_interrupt,
					chip_idea_set_endpoint_state,
					chip_idea_clear_endpoint_state,
					NULL,
					&chip_idea_pipe_methods,
					&chip_idea_pipe_methods,
					&chip_idea_pipe_methods,
					&chip_idea_isoch_pipe_methods,
};

usb_controller_methods chip_idea_usb_controller_methods = {
	NULL,
	&chip_idea_controller_methods,
	NULL,
	NULL
};

io_usb_dll_entry_t io_usb_dll_entry = {
    USBDC_DLL_NAME,
    0xffff,  // pci device id
    0xffff,
    0xffff,
    USB_CONTROLLER_DEVICE,
    NULL,
    NULL,
    &chip_idea_usb_controller_methods
};

ssize_t chip_idea_slogf( chip_ideadc* dc, int verbosity, const char *fmt, ...)
{
	ssize_t 	rc;
	va_list 	arglist;

	if ( verbosity > dc->verbosity ) {
		return( 0 ); 
	}	

	va_start( arglist, fmt );
	rc = vslogf( 12, verbosity, fmt, arglist );
	va_end( arglist );

	return( rc );
}

uint32_t
chip_idea_set_device_address(  usbdc_device_t *udc, uint32_t address )
{
	/* 
	 * server sets the address too soon when we short circuit the 
	 * status phase. So we do it in complete_irq... 
	 */
	return( EOK );
}

uint32_t
chip_idea_select_configuration( usbdc_device_t *udc, uint8_t config )
{
	return( EOK );
}

/*
 * Get a transfer descriptor - (Expected that the calling function locks the endpoint mutex)
 */
chip_idea_isoch_urb_t *
chip_idea_get_urb( usbdc_device_t *udc, iousb_transfer_t *urb, chip_idea_endpoint_t *ep, _uint32 flags )
{
	chip_ideadc            *dcctrl = udc->dc_data;
	chip_idea_isoch_urb_t  *isoch_urb;

	if ( (isoch_urb = SIMPLEQ_FIRST( &ep->urb_free_q ) ) == NULL )
	{
		chip_idea_slogf( dcctrl, 5, "%s - No CI_URB structures available, growing list by one", __FUNCTION__ );
		if ((isoch_urb = calloc( 1, sizeof( chip_idea_isoch_urb_t ))) == NULL )
		{
			int status = errno;
			chip_idea_slogf( dcctrl, 5, "%s - Failed to allocate isoch_urb - %s", __FUNCTION__, strerror(status));
			return (NULL) ;
		}
	}
	else
		SIMPLEQ_REMOVE_HEAD( &ep->urb_free_q, link );

	isoch_urb->status          = 0;
	isoch_urb->flags           = flags;
	isoch_urb->urb             = urb;
	isoch_urb->isoch_flist     = NULL;
	isoch_urb->isoch_nframes   = 0;

	return( isoch_urb );
}


uint32_t
chip_idea_abort_transfer( usbdc_device_t *udc, chip_idea_endpoint_t *ep ) 
{
	chip_ideadc 				*dcctrl = udc->dc_data;
	volatile chip_idea_dtd_t	*dtd;
	int							i;
	uint32_t					timeout;
	uint32_t					flush_retry = 5;

	if ( !ep || !dcctrl ) {
		return( EOK );
	}

	chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s(): ep %d", __func__, ep->ep_num );

	chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );

	if ( ep->active_urb ) {
		do  {
			out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTFLUSH, ep->prime_bit );

			timeout = 1000000;
			while ( in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTFLUSH ) && --timeout ) {

			}

			if ( !timeout ) {
				chip_idea_slogf(dcctrl, _SLOG_ERROR, "%s : Timeout waiting for endpoint flush on ep %d", __func__, ep->ep_num );
			}
		} while ( ( in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSTAT ) & ep->prime_bit) && ( --flush_retry ) );

		if ( !flush_retry ) {
				chip_idea_slogf(dcctrl, _SLOG_ERROR, "%s : Error flushing endpoint ep %d", __func__, ep->ep_num );
		}

		// stop the endpoint 
		ep->qh->size_ioc_int_sts &= ~(EP_QUEUE_HEAD_STATUS_ACTIVE | EP_QUEUE_HEAD_STATUS_HALT);

		delay( 5 );

		// disable all active transfers
		ep->qh->next_dtd_ptr = DTD_NEXT_TERMINATE; 

		if (ep->type == USB_ENDPOINT_XFER_ISOC)
		{
			dtd = ep->isoch_dtds;

			for( i = 0; i < dcctrl->itds; i++, dtd++ ) {
				dtd->next_td_ptr = DTD_NEXT_TERMINATE;
				dtd->size_ioc_sts = 0;
			}
		}
		else
		{
			dtd = ep->dtd_head;

			for( i = 0; i < CHIP_IDEA_MAX_LINKED_DTD; i++, dtd++ ) {
				dtd->next_td_ptr = DTD_NEXT_TERMINATE;
				dtd->size_ioc_sts = 0;
			}
		}

//		chip_idea_set_endpoint_state( udc, iousb_ep, IOUSB_ENDPOINT_STATE_NAK );

		ep->active_urb = NULL;
	}

	chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );

	return( EOK );
}

void
chip_idea_send_empty_packet( chip_ideadc *dcctrl )
{
	chip_idea_endpoint_t 	*ep;
	chip_idea_dtd_t			*dtd;

	ep = &dcctrl->ep[16];
	dtd = ep->dtd_head;

	/* build a dTD  */
	dtd->next_td_ptr = DTD_NEXT_TERMINATE;
	dtd->size_ioc_sts = DTD_IOC | DTD_STATUS_ACTIVE;
		
	/* write dQH next pointer and dQH terminate bit to 0 */			
	ep->qh->next_dtd_ptr = ep->dtd_addr; 

	/* clear active &halt bit in dQH */
	ep->qh->size_ioc_int_sts &= ~(EP_QUEUE_HEAD_STATUS_ACTIVE | EP_QUEUE_HEAD_STATUS_HALT);

	/* Memory barrier between writing TDs and hitting the prime bit */
	CHIP_IDEA_BOARD_SPECIFIC_SYNC_FLUSH

	/* prime endpoint */		
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME ) |
		ep->prime_bit );

	return;
}

static void
chip_idea_ep_qh_setup( usbdc_device_t *udc, chip_idea_endpoint_t *ep, uint32_t ep_type, uint32_t max_pkt_len )
{
	chip_ideadc 			*dcctrl = udc->dc_data;
	chip_idea_qh_t		 	*qh = NULL;

	/* set the Endpoint Capabilites Reg of dQH */
	switch ( ep_type ) {
		case USB_ENDPOINT_XFER_CONTROL:
			/* Interrupt On Setup (IOS). for control ep  */
 			qh = &dcctrl->ep_qh[ (ep->ep_num * 2) + ep->dir ];
			qh->max_pkt_length = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS) | EP_QUEUE_HEAD_IOS;
			break;

		case USB_ENDPOINT_XFER_ISOC:
			ep->qh->max_pkt_length = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS) | EP_QUEUE_HEAD_ZLT_SEL | ( 1 << 30 );
			break;

		case USB_ENDPOINT_XFER_BULK:
		case USB_ENDPOINT_XFER_INT:
			/* Always disable zlp transmission ... */
			ep->qh->max_pkt_length = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS) | EP_QUEUE_HEAD_ZLT_SEL;
			break;

		default:
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s - error invalid ep type %d", __FUNCTION__, ep_type );
			ep->qh->max_pkt_length = 0;
			return;
	}

	return;
}

uint32_t
chip_idea_set_endpoint_state( usbdc_device_t *udc, iousb_endpoint_t *iousb_ep, uint32_t ep_state )
{
	chip_ideadc 			*dcctrl = udc->dc_data;
	chip_idea_endpoint_t 	*ep;
	uint32_t 				tmp;

	chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );

	if ( !( ep = (chip_idea_endpoint_t *) iousb_ep->user ) ) {
		chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
		return( ENOENT );
	}

	switch ( ep_state ) {
		case IOUSB_ENDPOINT_STATE_READY :
			break;

		case IOUSB_ENDPOINT_STATE_ENABLE :
			break;

		case IOUSB_ENDPOINT_STATE_DISABLED :
			break;

		case IOUSB_ENDPOINT_STATE_NAK :
			break;

		case IOUSB_ENDPOINT_STATE_STALLED :
			if ( ep->ep_num == 0 ) {
				/* Stall both rx & tx when stalling ep 0 ... */
				tmp = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL0 );
				out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL0, tmp | ( EPCTRL_TX_EP_STALL | EPCTRL_RX_EP_STALL ) );
			}
			else {
				tmp = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ) );
				if( ep->dir == CHIP_IDEA_EP_DIR_IN ) {
					out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ), tmp | EPCTRL_TX_EP_STALL );
				}
				else {
					out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ), tmp | EPCTRL_RX_EP_STALL );
				}
			}
			break;

		default :
			break;
	}

	chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
	
	return( EOK );
}

uint32_t
chip_idea_clear_endpoint_state( usbdc_device_t *udc, iousb_endpoint_t *iousb_ep, uint32_t ep_state )
{
	chip_ideadc 			*dcctrl = udc->dc_data;
	chip_idea_endpoint_t 	*ep;
	uint32_t 				tmp;
	
	chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );

	if ( ! ( ep = (chip_idea_endpoint_t *)iousb_ep->user ) ) {
		chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
		return( ENOENT );
	}

	switch ( ep_state ) {
		case IOUSB_ENDPOINT_STATE_READY :
			break;

		case IOUSB_ENDPOINT_STATE_ENABLE :
			break;

		case IOUSB_ENDPOINT_STATE_DISABLED :
			break;

		case IOUSB_ENDPOINT_STATE_NAK :
			break;

		case IOUSB_ENDPOINT_STATE_STALLED :
			if ( ep->ep_num == 0 ) {
				out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL0, ~(EPCTRL_TX_EP_STALL | EPCTRL_RX_EP_STALL) );
			}
			else {
				tmp = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ) );
				if( ep->dir == CHIP_IDEA_EP_DIR_IN ) {
					out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ), (tmp & ~EPCTRL_TX_EP_STALL) | EPCTRL_TX_DATA_TOGGLE_RST );
				}
				else {
					out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ), (tmp & ~EPCTRL_RX_EP_STALL) | EPCTRL_RX_DATA_TOGGLE_RST );
				}
			}
			break;

		default :
			break;
	}

	chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
	
	return( EOK );
}

uint32_t
chip_idea_set_device_state( usbdc_device_t *udc, uint32_t device_state )
{
	return( EOK );
}

uint32_t
chip_idea_set_bus_state( usbdc_device_t *udc, uint32_t device_state )
{
	chip_ideadc 	*dcctrl = udc->dc_data;
	uint32_t		val;

	switch ( device_state ) {
		case IOUSB_BUS_STATE_DISCONNECTED :
			CHIP_IDEA_BOARD_SPECIFIC_LINK_DOWN
			out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) & ~USB_CMD_RUN_STOP );
			udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_REMOVED);
			break;

		case IOUSB_BUS_STATE_CONNECTED :
			CHIP_IDEA_BOARD_SPECIFIC_LINK_UP
			val = (in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) & ~USB_CMD_ITC_MASK) |
				 USB_CMD_RUN_STOP | dcctrl->int_thresh;
			out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, val );
			break;	

		case IOUSB_BUS_STATE_RESUME :
		default :
			return( ENOTSUP );
			break;	
	}

	return( EOK );
}


/*
	Set port test mode. This must be done witin 3ms of status phase of request.
*/

uint32_t
chip_idea_set_test_mode( chip_ideadc *dcctrl, uint16_t wIndex ) 
{

	chip_idea_slogf(dcctrl, _SLOG_DEBUG2, "%s : 0x%x", __func__, wIndex );

	switch( wIndex ) {	
		case USB_TEST_MODE_TEST_J :
		case USB_TEST_MODE_TEST_K :
		case USB_TEST_MODE_TEST_SE0_NAK :
		case USB_TEST_MODE_TEST_PACKET :
		case USB_TEST_MODE_TEST_FORCE_ENABLE :
			out32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX, (in32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX ) | (wIndex << 16) ) );
			break;

		default :	
			return( ENOTSUP );	
	}

	return( EOK );
}

/*
	Enable a device feature
*/
uint32_t
chip_idea_set_device_feature( usbdc_device_t *udc, uint32_t feature, uint16_t wIndex )
{
	chip_ideadc 	*dcctrl = udc->dc_data;

	switch ( feature ) {
		case USB_FEATURE_DEV_WAKEUP :
			return( ENOTSUP );
			break;

		case USB_FEATURE_TEST_MODE :
			return( chip_idea_set_test_mode( dcctrl, wIndex ) );
			break;

		default :
			return( ENOTSUP );
			break;
	}

	return( EOK );
}

/*
	clear a feature of a device
*/
uint32_t
chip_idea_clear_device_feature( usbdc_device_t *udc, uint32_t feature )
{
	switch ( feature ) {
		case USB_FEATURE_DEV_WAKEUP :
			return( ENOTSUP );
			break;

		case USB_FEATURE_TEST_MODE : // don't support test modes
			return( ENOTSUP );
			break;
	}

	return( ENOTSUP );
}

uint32_t
chip_idea_get_descriptor ( usbdc_device_t *udc, uint8_t type, uint8_t index, uint16_t lang_id, uint8_t **desc, uint32_t speed )
{
	chip_ideadc 	*dcctrl = udc->dc_data;
	uint8_t  		*descriptor;

	switch ( type ) {
		case USB_DESC_DEVICE :
			*desc = (speed == IOUSB_DEVICE_FULL_SPEED ) ? ((uint8_t *) USBDC_FS_DEVICE_DESCRIPTOR) : ((uint8_t *) USBDC_HS_DEVICE_DESCRIPTOR);
			chip_idea_slogf(dcctrl, _SLOG_DEBUG2, "%s : get USBDC_DEVICE_DESCRIPTOR", __func__);
			break;

		case USB_DESC_CONFIGURATION :
		case USB_DESC_OTHER_SPEED_CONF :
			if ( speed == IOUSB_DEVICE_FULL_SPEED ) {	
				*desc = descriptor = (uint8_t *) USBDC_FS_CONFIG_DESCRIPTOR[index];
			}
			else {
				*desc = descriptor = (uint8_t *) USBDC_HS_CONFIG_DESCRIPTOR[index];
			}

			break;

		case USB_DESC_STRING :
			if( index <= USBDC_MAX_STRING_DESCRIPTOR ) {
				*desc = (speed == IOUSB_DEVICE_FULL_SPEED ) ? ((uint8_t *) USBDC_FS_STRING_DESCRIPTOR[index]) : ((uint8_t *) USBDC_HS_STRING_DESCRIPTOR[index]);
				chip_idea_slogf( dcctrl, _SLOG_DEBUG2, "%s : get USBDC_STRING_DESCRIPTOR - index %d", __func__, index );
			}
			else {
				return( ENOTSUP );
			}
			break;

		case USB_DESC_DEVICE_QUALIFIER :
			*desc = (speed == IOUSB_DEVICE_FULL_SPEED ) ? (uint8_t *) USBDC_FS_DEVICE_QUALIFIER_DESCRIPTOR : (uint8_t *) USBDC_HS_DEVICE_QUALIFIER_DESCRIPTOR;
			chip_idea_slogf( dcctrl, _SLOG_DEBUG2, "%s : get USBDC_DEVICE_QUALIFIER_DESCRIPTOR", __func__ );
			break;

		case USB_DESC_INTERFACE_POWER :
		case USB_DESC_INTERFACE :
		case USB_DESC_ENDPOINT :

		default :
			return( ENOTSUP );
			break;
	}
	
	return( EOK );
}

uint32_t
chip_idea_endpoint_init( usbdc_device_t *udc, iousb_endpoint_t *iousb_ep )
{
	chip_ideadc 			*dcctrl = udc->dc_data;
	chip_idea_endpoint_t 	*ep;
	int 					ep_num, direction;
	uint32_t 				epctrl; 

	ep_num = iousb_ep->edesc.bEndpointAddress & 0x7f;

	if ( ep_num > (dcctrl->num_ep - 1) ) {
		chip_idea_slogf( dcctrl, _SLOG_CRITICAL, "%s : Endpoint address %x not supported", __func__, ep_num );
		return( ENOTSUP );
	}

	if ( ep_num == 0 ) {
		/* Endpoint 0 is split in half ... i.e. server only has one handle, 
		 * but we have two */
		dcctrl->ep[0].ep = dcctrl->ep[16].ep 			= iousb_ep; 
		dcctrl->ep[0].ep_num = dcctrl->ep[16].ep_num 	= 0; 

		dcctrl->ep[0].dir 			= CHIP_IDEA_EP_DIR_OUT;
		dcctrl->ep[16].dir  		= CHIP_IDEA_EP_DIR_IN;
		dcctrl->ep[0].prime_bit		= 1;
		dcctrl->ep[16].prime_bit 	= (1<<16);

		iousb_ep->user = &dcctrl->ep[0];

		return( EOK );
	}

	direction = ((iousb_ep->edesc.bEndpointAddress & 0x80) ? CHIP_IDEA_EP_DIR_IN : CHIP_IDEA_EP_DIR_OUT);

	ep = &dcctrl->ep[ep_num + (direction ? 16 : 0)];

	ep->ep 		= iousb_ep;
	ep->ep_num 	= ep_num; 
	ep->dir 	= direction;
	ep->prime_bit 	= (direction == CHIP_IDEA_EP_DIR_IN ) ? (1 << (ep->ep_num + 16)) : (1 << ep->ep_num);

	ep->mps		= iousb_ep->edesc.wMaxPacketSize;
	ep->type	= iousb_ep->edesc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	epctrl = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ) );

	if( ep->dir == CHIP_IDEA_EP_DIR_IN ) {
		/* Opposite direction EP type must be changed from default if it is unused. (30.8.1.5.21) */
		if ( !( epctrl & EPCTRL_RX_EP_TYPE_MASK ) ) {
			epctrl |= (ep->type << EPCTRL_RX_EP_TYPE_SHIFT);
		}
		/* don't clobber Rx bits */
		out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ), (epctrl & ~EPCTRL_TX_BITS_MASK) |
			 EPCTRL_TX_ENABLE | EPCTRL_TX_DATA_TOGGLE_RST | (ep->type) << EPCTRL_TX_EP_TYPE_SHIFT );
	}
	else {
		/* Opposite direction EP type must be changed from default if it is unused. (30.8.1.5.21) */
		if ( !( epctrl & EPCTRL_TX_EP_TYPE_MASK ) ) {
			epctrl |= (ep->type << EPCTRL_TX_EP_TYPE_SHIFT );
		}
		/* don't clobber Tx bits */
		out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ), (epctrl & ~EPCTRL_RX_BITS_MASK) |
			EPCTRL_RX_ENABLE | EPCTRL_RX_DATA_TOGGLE_RST | (ep->type) << EPCTRL_RX_EP_TYPE_SHIFT );
	}

	iousb_ep->user = ep; 	

	chip_idea_ep_qh_setup( udc, ep, ep->type, ep->mps );

	return( EOK );
}

/*
 * Initialize Isoch URB queues and allocate isoch dTD memory 
 * Actual Isoch URB entries will be allocated as needed in the isoch_transfer function
 */
int
chip_idea_init_isoch_endpoint( usbdc_device_t *udc, iousb_endpoint_t *iousb_ep )
{
	int                   status = EOK;
	chip_ideadc           *dcctrl = udc->dc_data;
	chip_idea_endpoint_t  *ep;

	if ((status = chip_idea_endpoint_init( udc, iousb_ep )) != EOK)
		return (status);

	if ( (ep = (chip_idea_endpoint_t *)iousb_ep->user ) == NULL )
		return( ENOENT );

	chip_idea_slogf(dcctrl, 3, "%s : ep %d", __func__, ep->ep_num );

	SIMPLEQ_INIT( &ep->urb_free_q );
	SIMPLEQ_INIT( &ep->urb_pending_q );

	/* alloc 8-Dword dTD block of memory aligned to 32-byte boundaries */
	ep->isoch_dtd_mem_sz = (sizeof( chip_idea_dtd_t ) * dcctrl->itds);

	if ( (ep->isoch_dtd_mem = mmap( NULL, ep->isoch_dtd_mem_sz, PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_PRIVATE | MAP_ANON | MAP_PHYS, NOFD, 0 )) == MAP_FAILED ) {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: mmap failed to alloc isoch dtd_mem for endpoint 0x%x", __func__, 
			(ep->ep_num +  (ep->dir ? 16 : 0)));
		return(ENOMEM);
	}
	memset( ep->isoch_dtd_mem, 0, ep->isoch_dtd_mem_sz );
	ep->isoch_dtds = (chip_idea_dtd_t *) (((_uint32) ep->isoch_dtd_mem + 31) & 0xFFFFFFE0);
	ep->isoch_dtd_head = ep->isoch_dtd_tail = 0;
	ep->isoch_dtd_addr = mphys( ep->isoch_dtds );

	return EOK ;
}

_uint32
chip_idea_isoch_endpoint_enable( void *chdl, iousb_device_t *device, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t *udc = ( usbdc_device_t * ) chdl;

	return ( chip_idea_init_isoch_endpoint( udc, iousb_ep ) );
}

_uint32
chip_idea_endpoint_enable( void *chdl, iousb_device_t *device, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t *udc = ( usbdc_device_t * ) chdl;

	return( chip_idea_endpoint_init( udc, iousb_ep ) );
}

uint32_t
chip_idea_endpoint_stop( usbdc_device_t *udc, iousb_endpoint_t *iousb_ep )
{
	//chip_ideadc 	*dcctrl = udc->dc_data;
	//iousb_transfer_t *urb = NULL;
	chip_idea_endpoint_t *ep =(chip_idea_endpoint_t *)iousb_ep->user;

	if ( !ep ) {
		return( EOK );
	}

#if 0 
	pthread_mutex_lock(&ep->mutex);
	urb = ep->active_urb;
	pthread_mutex_unlock(&ep->mutex);
#endif 
	
	ep->active_urb = 0; 

	if ( iousb_ep->user ) {
		iousb_ep->user = 0;
	}
	
	return( EOK );
}

_uint32
chip_idea_isoch_endpoint_disable( void *chdl, iousb_endpoint_t *iousb_ep )
{
	int                    status = EOK, cnt = 0;
	chip_idea_isoch_urb_t  *isoch_urb;
	usbdc_device_t         *udc = ( usbdc_device_t * ) chdl;
	chip_ideadc            *dcctrl = udc->dc_data;
	chip_idea_endpoint_t   *ep =(chip_idea_endpoint_t *)iousb_ep->user;

	chip_idea_pthread_mutex_lock( &ep->mutex );
	chip_idea_slogf(dcctrl, 3, "%s : ep %d", __func__, ep->ep_num );

	status = chip_idea_endpoint_stop( udc, iousb_ep );

	while((isoch_urb = SIMPLEQ_FIRST(&ep->urb_free_q)))
	{
		SIMPLEQ_REMOVE_HEAD( &ep->urb_free_q, link );
		free(isoch_urb);
		chip_idea_slogf( dcctrl, 5, "%s - Free'ing URB %d on ep %d dir %s", __FUNCTION__, cnt, ep->ep_num, ep->dir ? "In" : "Out" );
	}
	munmap( ep->isoch_dtd_mem, ep->isoch_dtd_mem_sz );
	chip_idea_pthread_mutex_unlock( &ep->mutex );

	return (status);
}

_uint32
chip_idea_endpoint_disable( void *chdl, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t *udc = ( usbdc_device_t * ) chdl;

	return( chip_idea_endpoint_stop( udc, iousb_ep ) );
}

_uint32
chip_idea_isoch_transfer_abort( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t         *udc = ( usbdc_device_t * ) chdl;
	chip_ideadc            *dcctrl = udc->dc_data;
	chip_idea_endpoint_t   *ep;
	chip_idea_isoch_urb_t  *isoch_urb;

	if ( (ep = (chip_idea_endpoint_t *)iousb_ep->user ) == NULL )
	{
		return( ENOENT );
	}

	chip_idea_slogf( dcctrl, 3, "%s - Abort ep 0x%x %s", __FUNCTION__, ep->ep_num, ep->dir == 1 ? "In" : "Out");

	chip_idea_abort_transfer( udc, ep );

	chip_idea_pthread_mutex_lock( &ep->mutex );
	while ((isoch_urb = SIMPLEQ_FIRST( &ep->urb_pending_q )))
	{
		/* Move  isoch_urb to the free queue */
		chip_idea_slogf( dcctrl, 5, "%s - Remove isoch_urb from pending_q 0x%x", __FUNCTION__,  isoch_urb);
		SIMPLEQ_REMOVE_HEAD( &ep->urb_pending_q, link );
		SIMPLEQ_INSERT_TAIL( &ep->urb_free_q, isoch_urb, link );
	}
	chip_idea_pthread_mutex_unlock( &ep->mutex );

	return( EOK );
}

_uint32
chip_idea_transfer_abort( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t			*udc = ( usbdc_device_t * ) chdl;
	chip_ideadc 			*dcctrl = udc->dc_data;
	chip_idea_endpoint_t 	*ep;

	if ( (ep = (chip_idea_endpoint_t *)iousb_ep->user ) == NULL ) {
		return( ENOENT );
	}
	
	if ( ep->ep_num == 0 ) {
		// abort both in and out for control endpoint
		chip_idea_abort_transfer( udc, &dcctrl->ep[ 0 ] );
		chip_idea_abort_transfer( udc, &dcctrl->ep[ 16 ] );
	}
	else {
		chip_idea_abort_transfer( udc, ep );
	}
	
	return( EOK );
}

// io-usb-dcd ensures that the endpoint is never active when chip_idea_transfer is called.
// Case two in chipidea documentation section 6.6.3 "Executing a trasnfer descriptor" 
// does not need to be handled in this code.  For more information, see:
//   "CI13611-a High-Speed USB On-The-Go Controller Core Programmer's Manual"
//   Document date: 2007-09-11
//   Page 83

_uint32
chip_idea_transfer( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *iousb_ep, uint8_t *buffer, _uint32 length, _uint32 flags )
{
	usbdc_device_t 					*udc = ( usbdc_device_t * ) chdl;
	chip_ideadc 					*dcctrl = udc->dc_data;
	chip_idea_endpoint_t 			*ep;
	volatile chip_idea_dtd_t		*dtd;
	uint32_t 						dtd_length, mphys_buffer, next_dtd_addr, offset;
	int 							i; 
	uint32_t						timeout, retries;
	uint32_t						epstat;

	chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );

	if ( (ep = (chip_idea_endpoint_t *) iousb_ep->user) == 0 ) {
		chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
		return( ENODEV ); 
	}
	
	if ( ep->ep_num == 0 ) {
		/* select correct half of ep0 ... */
		ep = &dcctrl->ep[(flags & PIPE_FLAGS_TOKEN_IN) ? 16 : 0]; 

		// check if we need to send a zero length packet
		if ( ( urb->buffer_len == 0 ) && (flags & PIPE_FLAGS_TOKEN_IN) ) { 
			ep->active_urb = urb;
			ep->flags = flags;
			chip_idea_send_empty_packet( dcctrl ); 
			chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
			return( EOK );
		}
		// fall through for normal transfer
	}

	ep->active_urb = urb;
	ep->flags = flags;
	mphys_buffer = ((uint32_t) (urb->buffer + urb->actual_len) & 0xfffff000); 
	offset = ((uint32_t) (urb->buffer + urb->actual_len) & 0xfff);

	// there is a problem for token out (receive) that the controller can not properly terminate
	// the transfer in case of short packet (no interrupt fired or controller automatically advance
	// to the next DTD), so we only allow one DTD.
	// ideally, we should only restrict transfer that have URB_SHORT_XFER_OK flag set,
	// we'll rework that once the stack can pass down that flag.
	if(/*(urb->flags & PIPE_FLAGS_SHORT_XFER_OK) &&*/ (flags & PIPE_FLAGS_TOKEN_OUT)){
		length = min(length, EP_MAX_LENGTH_TRANSFER);
	}else{
		length = min(length, CHIP_IDEA_MAX_TRANSFER_SIZE);
	}
	ep->transfer_len = length;

	dtd = ep->dtd_head; 
	next_dtd_addr = ep->dtd_addr;

	for ( i = 0 ; i < CHIP_IDEA_MAX_LINKED_DTD ; i++, dtd++ ) {
		dtd_length = min( EP_MAX_LENGTH_TRANSFER, length );
		dtd->buff_ptr0	= mphys_buffer + offset;

		dtd->buff_ptr1	= mphys_buffer + 0x1000;
		dtd->buff_ptr2	= mphys_buffer + 0x2000;
		dtd->buff_ptr3	= mphys_buffer + 0x3000;
		dtd->buff_ptr4	= mphys_buffer + 0x4000;

		// shouldn't need to interupt on every TD
		dtd->size_ioc_sts	= (dtd_length << DTD_LENGTH_BIT_POS) /* | DTD_IOC*/ | DTD_STATUS_ACTIVE; 
		if ( (dtd_length - length) <= 0 ) {  
			dtd->next_td_ptr 	= DTD_NEXT_TERMINATE; 
			dtd->size_ioc_sts 	|= DTD_IOC; 
			break; 
		}

		dtd->next_td_ptr = (next_dtd_addr += sizeof( chip_idea_dtd_t )); 

		mphys_buffer += dtd_length; 
		length -= dtd_length; 
	}

	ep->dtd_count = i + 1; 

	/* Active the dtd chain */
	ep->qh->next_dtd_ptr = ep->dtd_addr; 
	ep->qh->size_ioc_int_sts &= ~(EP_QUEUE_HEAD_STATUS_ACTIVE | EP_QUEUE_HEAD_STATUS_HALT);

	/* Make sure the descriptors have been written to main memory before hitting the prime bit */
	/* to ensure the controller has a coherent view of the descriptors */
	CHIP_IDEA_BOARD_SPECIFIC_SYNC_FLUSH

	retries = CHIP_IDEA_MAX_PRIME_RETRIES;
	do {
		/* Initiate priming operation */
		out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME, 
			   in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME ) | ep->prime_bit );
		CHIP_IDEA_BOARD_SPECIFIC_SYNC_FLUSH

		// Wait for prime bit to go low
		timeout = 1000;
		while( ( in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME ) & ep->prime_bit )  && --timeout ) { }
		if( timeout == 0 ) {
			/* Priming is taking an unusually long time */
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "CHIP_IDEA_UOG_ENDPTPRIME timeout - prime_bit = 0x%08X, ENDPTPRIME = 0x%08X",
							 ep->prime_bit, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME ) );
		}

		/* Confirm priming operation */
		timeout = 10;

		/* Do a memory fence before reading the dtd */
		CHIP_IDEA_BOARD_SPECIFIC_SYNC_FLUSH
		epstat = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSTAT );
		while( ( !( epstat & ep->prime_bit ) ) && ( ep->dtd_head->size_ioc_sts & DTD_STATUS_ACTIVE  ) && --timeout ) {
			epstat = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSTAT );
			CHIP_IDEA_BOARD_SPECIFIC_SYNC_FLUSH
		}
		if( timeout == 0 ) {
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "CHIP_IDEA_UOG_ENDPTSTAT timeout - prime_bit = 0x%08X, ENDPTSTAT = 0x%08X, ENDPTPRIME = 0x%08X",
							 ep->prime_bit, epstat, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME ) );
		}

	} while ( ( timeout == 0 ) && --retries );

	if( retries == 0 ) {
		/* At this point re-enumeration would likely be the only way to recover */
		chip_idea_slogf( dcctrl, _SLOG_CRITICAL, "CI reprime FAILED timeout - prime_bit = 0x%08X, ENDPTSTAT = 0x%08X, ENDPTPRIME = 0x%08X",
						 ep->prime_bit, epstat, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME ) );
	} else if( retries != CHIP_IDEA_MAX_PRIME_RETRIES ) {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "CI reprime success - retries = %d prime_bit = 0x%08X, ENDPTSTAT = 0x%08X, ENDPTPRIME = 0x%08X",
						 retries, ep->prime_bit, epstat, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME ) );
	}

	chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );

	return( EOK );
}

_uint32
chip_idea_prime_isoch_endpoint (chip_ideadc *dcctrl, chip_idea_endpoint_t *ep)
{
	uint32_t epstat;
	int timeout;
	int retries = CHIP_IDEA_MAX_PRIME_RETRIES;

	chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );

	/* Prime the endpoint starting from the head offset (Head points to the start of uncompleted dTDs),
	 * this ensures that if we are re-priming a stalled endpoint any uncompleted dTDs that were already
	 * on the chain are re-scheduled.
	 */
	ep->qh->next_dtd_ptr = ep->isoch_dtd_addr + (ep->isoch_dtd_head * sizeof( chip_idea_dtd_t ));
	ep->qh->size_ioc_int_sts &= ~(EP_QUEUE_HEAD_STATUS_ACTIVE | EP_QUEUE_HEAD_STATUS_HALT);

	/* Make sure the descriptors have been written to main memory before hitting the prime bit */
	/* to ensure the controller has a coherent view of the descriptors */
	CHIP_IDEA_BOARD_SPECIFIC_SYNC_FLUSH

	do
	{
		/* Initiate priming operation */
		out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME,
			in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME ) | ep->prime_bit );
		CHIP_IDEA_BOARD_SPECIFIC_SYNC_FLUSH

		/* Confirm priming operation */
		timeout = 10;

		/* Do a memory fence before reading the dtd */
		CHIP_IDEA_BOARD_SPECIFIC_SYNC_FLUSH
		epstat = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSTAT );
		while( ( !( epstat & ep->prime_bit ) ) && ( ep->isoch_dtds->size_ioc_sts & DTD_STATUS_ACTIVE  ) && --timeout )
		{
			epstat = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSTAT );
			CHIP_IDEA_BOARD_SPECIFIC_SYNC_FLUSH
		}
		if( timeout == 0 )
		{
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s timeout waiting for endpoint prime status on ep %d %s - ENDPTSTAT = 0x%08X, next_ptr = 0x%x",
				__FUNCTION__, ep->ep_num, ep->dir ? "In" : "Out", epstat, ep->qh->next_dtd_ptr);
		}
	}while ( ( timeout == 0 ) && --retries );

	chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );

	if( retries == 0 )
	{
		/* At this point re-enumeration would likely be the only way to recover */
		chip_idea_slogf( dcctrl, _SLOG_CRITICAL, "%s reprime FAILED - prime_bit = 0x%08X, ENDPTSTAT = 0x%08X, ENDPTPRIME = 0x%08X",
			__FUNCTION__, ep->prime_bit, epstat, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME ) );
	}

	return (EOK);
}

_uint32
chip_idea_isoch_transfer( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *iousb_ep, uint8_t *buffer, _uint32 length, _uint32 flags )
{
	usbdc_device_t            *udc = ( usbdc_device_t * ) chdl;
	chip_ideadc               *dcctrl = udc->dc_data;
	chip_idea_endpoint_t      *ep;
	volatile chip_idea_dtd_t  *dtd;
	uint32_t                  dtd_length = 0, mphys_buffer, next_dtd_addr, offset = 0, transfer_length = 0, dtd_idx, multo = 0;
	chip_idea_isoch_urb_t     *isoch_urb = NULL;
	int                       i;

	chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );

	if ( (ep = (chip_idea_endpoint_t *) iousb_ep->user) == 0 )
	{
		chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
		return( ENODEV ); 
	}
	chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );

	chip_idea_pthread_mutex_lock( &ep->mutex );
	/* Look to see if this URB is already on the pending list */
	for (isoch_urb = SIMPLEQ_FIRST( &ep->urb_pending_q ); isoch_urb != NULL && isoch_urb->urb != urb; isoch_urb = SIMPLEQ_NEXT(isoch_urb, link));
	if (isoch_urb == NULL)
	{
		/* URB is not yet on the list, add it */
		if ((isoch_urb = chip_idea_get_urb (udc, urb, ep, flags)) == NULL)
		{
			chip_idea_pthread_mutex_unlock( &ep->mutex );
			return (ENOMEM);
		}
		if ( flags & PIPE_FLAGS_MULTI_XFER)
		{
			isoch_urb->isoch_flist = ((usbd_urb_isoch_stream_xfer_t *) isoch_urb->urb->xdata_ptr)->frame_list;
			isoch_urb->isoch_nframes = ((usbd_urb_isoch_stream_xfer_t *) isoch_urb->urb->xdata_ptr)->nframes;
		}
	}
	else
	{
		chip_idea_slogf( dcctrl, 3,"%s - URB (0x%x:0x%x)  found on pending list - nframes = %d",
			 __FUNCTION__,  isoch_urb, isoch_urb->urb, isoch_urb->isoch_nframes);
	}
	ep->flags = isoch_urb->flags;

	if ( isoch_urb->urb->actual_len >= isoch_urb->urb->buffer_len )
	{
		chip_idea_slogf( dcctrl, 3, "%s - urb actual_len= %d, urb buffer_len = %d",
			 __FUNCTION__, isoch_urb->urb->actual_len, isoch_urb->urb->buffer_len);
	}

	/* If there is no active urb, then this is the first transfer, so reset head and tail */
	if (ep->active_urb == NULL)
	{
		ep->isoch_dtd_tail = ep->isoch_dtd_head = 0;
		ep->dtd_count = 0;
	}

	mphys_buffer = ((uint32_t) isoch_urb->urb->buffer & 0xfffff000);
	offset = ((uint32_t) isoch_urb->urb->buffer & 0xfff);

	dtd = ep->isoch_dtds;
	next_dtd_addr = ep->isoch_dtd_addr + (ep->isoch_dtd_tail * sizeof( chip_idea_dtd_t ));

	/* Fill out the dTD to be 1 frame per TD */
	dtd_length = 0;
	if (dcctrl->itds - ep->dtd_count < isoch_urb->isoch_nframes)
	{
		chip_idea_slogf( dcctrl, 3,"%s - Not enough dTDs to schedule the frame list req = %d, available = %d",
			 __FUNCTION__, isoch_urb->isoch_nframes, dcctrl->itds - ep->dtd_count);
		SIMPLEQ_INSERT_TAIL( &ep->urb_free_q, isoch_urb, link );
		chip_idea_pthread_mutex_unlock( &ep->mutex );
		return (ENOMEM);
	}
	SIMPLEQ_INSERT_TAIL( &ep->urb_pending_q, isoch_urb, link );

	for ( dtd_idx = ep->isoch_dtd_tail, i = 0; i < isoch_urb->isoch_nframes; i++)
	{
		dtd[dtd_idx].buff_ptr0 = mphys_buffer + offset;

		dtd[dtd_idx].buff_ptr1 = mphys_buffer + 0x1000;
		dtd[dtd_idx].buff_ptr2 = mphys_buffer + 0x2000;
		dtd[dtd_idx].buff_ptr3 = mphys_buffer + 0x3000;
		dtd[dtd_idx].buff_ptr4 = mphys_buffer + 0x4000;

		dtd_length = isoch_urb->isoch_flist[i].frame_len;
		/* For Transmit (isoch IN) set the MULTO bits to describe the number of packets per dTD */
		multo = (ep->dir == CHIP_IDEA_EP_DIR_IN) ? (1<<10) : 0;
		dtd[dtd_idx].size_ioc_sts = (dtd_length << DTD_LENGTH_BIT_POS) | multo | DTD_STATUS_ACTIVE;

		/* Interrupt on the last frame of the URB
		 * Allways terminate the dTD chain.
		 */
		if ( i == (isoch_urb->isoch_nframes - 1))
		{
			dtd[dtd_idx].size_ioc_sts |= DTD_IOC;
			dtd[dtd_idx].next_td_ptr = DTD_NEXT_TERMINATE;
		}
		else
		{
			/* Link the current dTD to the next, account for wrapping around the dTD buffer */
			if (dtd_idx == dcctrl->itds - 1)
				dtd[dtd_idx].next_td_ptr = next_dtd_addr = ep->isoch_dtd_addr;
			else
				dtd[dtd_idx].next_td_ptr = (next_dtd_addr += sizeof( chip_idea_dtd_t ));

			transfer_length += dtd_length;
			mphys_buffer = ((uint32_t) (isoch_urb->urb->buffer + transfer_length)) & 0xfffff000;
			offset = ((uint32_t) (isoch_urb->urb->buffer + transfer_length)) & 0xfff;
		}

		/* Advance the dTD index to point to the next available dTD (account for dTD buffer wrap) */
		dtd_idx++;
		if (dtd_idx >= dcctrl->itds)
			dtd_idx = 0;
	}

	/* There is an active urb so we must hook our dTD list (frame list) to the end of the active dTD chain */
	if ( ep->active_urb )
	{
		uint32_t last_dtd_idx;

		if (ep->isoch_dtd_tail == 0)
			last_dtd_idx = dcctrl->itds - 1;	/* handle buffer wrap */
		else
			last_dtd_idx = ep->isoch_dtd_tail - 1;

		/* Link the last active dTD's next pointer to the next dTD */
		dtd[last_dtd_idx].next_td_ptr = ep->isoch_dtd_addr + (ep->isoch_dtd_tail * sizeof( chip_idea_dtd_t ));

		chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );	/* We are going to hit hardware registers so lock the main hardware mutex */
		/* To safely add to an active dTD chain, we must use the ATDTW trip wire bit */
		if (!(in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME ) & ep->prime_bit ))
		{
			uint32_t tmp;
			int timeout = 10000;
			do 
			{
				out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) | USB_CMD_ATDTW );
				tmp = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSTAT );
			}while (!(in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) & USB_CMD_ATDTW) && timeout--);
			/* Clear ATDTW bit */
			out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, (in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) & ~(USB_CMD_ATDTW)) );
			if (!(tmp & ep->prime_bit))
			{
				ep->active_urb = NULL; /* Force  Re-prime of endpoint (Below) */
				chip_idea_slogf( dcctrl, 3,"%s - Force Re-prime of endpoint = %d %s - next_ptr = 0x%x",
					 __FUNCTION__, ep->ep_num, ep->dir ? "In" : "Out", ep->qh->next_dtd_ptr);
			}
		}
		chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
	}

	/* Update the tail index to next available dTD (i.e jump past the dTDs we just added to the chain) */
	ep->isoch_dtd_tail = dtd_idx;
	ep->dtd_count += i;

	if (ep->active_urb == NULL)
	{
		chip_idea_prime_isoch_endpoint(dcctrl, ep);
		ep->active_urb = isoch_urb->urb;
	}

	chip_idea_pthread_mutex_unlock( &ep->mutex );

	return( EOK );
}

static void
chip_idea_ep0_setup_irq(usbdc_device_t *udc )
{
	chip_ideadc 			*dcctrl = udc->dc_data;
	chip_idea_qh_t	 		*qh;
	uint8_t 				buf[8];	
	uint32_t 				timeout;

	/* Clear bit in ENDPTSETUPSTAT */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSETUPSATA, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSETUPSATA ) | 1 );

	qh = &dcctrl->ep_qh[0];

	/* while a hazard exists when setup package arrives */
	do {
		/* Set Setup Tripwire */
		out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) | USB_CMD_SUTW );

#if defined(__LITTLEENDIAN__)
		/* Copy the dQH.SetupBuffer into local buffer urb */
		memcpy( buf, qh->setup_buffer, sizeof( usb100_setup_packet_t ) );
#else
		/* setup buffer needs to swapped */
		{
			int i;	
			/* Copy the dQH.SetupBuffer into local buffer urb*/
			for ( i=0 ; i < 4 ; i++ ) {
			        buf[i] = qh-> setup_buffer[3-i];
			}
			for ( i = 0; i < 4 ; i++ ) {
			        buf[4+i]= qh-> setup_buffer[7-i];
			}
		}
#endif

	} while( !(in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) & USB_CMD_SUTW) );

	/* Clear Setup Tripwire */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) & ~USB_CMD_SUTW );

	/* 
	 * Due to missing code in server, we handle the set address request (0x5) ourselves, 
	 * and ignore the set_address call from server. 
	 */

	if ( (buf[0] == 0) && (buf[1] == 0x5) ) {
		chip_idea_slogf( dcctrl, _SLOG_INFO, "%s - assigning address %d", __FUNCTION__, buf[2] );
		out32( dcctrl->IoBase + CHIP_IDEA_UOG_DEVICEADDR,  buf[2] << USB_DEVICE_ADDRESS_BIT_POS | USB_DEVICE_ADDRESS_USBADRA );
		chip_idea_send_empty_packet( dcctrl ); 
	}

	/* send setup packet up to server */
	chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
	udc->usbdc_self->usbdc_setup_packet_process( udc, buf );
	chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );

	timeout = 10000000UL;
	while ( (in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSETUPSATA ) & 1 ) && --timeout ) {
		continue;
	}

	return; 
}

static uint32_t
dtd_check_errors( chip_ideadc *dcctrl, volatile chip_idea_dtd_t *curr_td, uint32_t error )
{
	if ( error & DTD_STATUS_DATA_BUFF_ERR ) {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "dtd buffer error" );
		return( USBD_STATUS_CMP_ERR | USBD_STATUS_DATA_OVERRUN );
	}
	else if ( error & DTD_STATUS_TRANSACTION_ERR ) {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "transaction error" );
		return( USBD_STATUS_CMP_ERR | USBD_STATUS_BITSTUFFING );
	}
	else {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "dtd error %x", error );
		return( USBD_STATUS_CMP_ERR | USBD_STATUS_ABORTED );
	}
}

static int
chip_idea_dtd_isoch_complete_irq( usbdc_device_t *udc, uint32_t ep_complete_mask )
{
	chip_ideadc                *dcctrl = udc->dc_data;
	chip_idea_endpoint_t       *ep;
	int                        i, epnum = 0;
	volatile chip_idea_dtd_t   *dtd;
	uint32_t                   remainder = 0;
	uint32_t                   actual_len;
	uint32_t                   status = EOK;
	uint32_t                   error;
	uint32_t                   urb_error = EOK;
	chip_idea_isoch_urb_t      *isoch_urb = NULL;

	for ( ; ep_complete_mask ; ep_complete_mask >>= 1, epnum++ )
	{
		if ( !(ep_complete_mask & 1) )
			continue;

		ep = &dcctrl->ep[ epnum ];
		/* Only process ISOCH completions */
		if (ep->type != USB_ENDPOINT_XFER_ISOC)
			continue;

		status = EOK;

		chip_idea_pthread_mutex_lock( &ep->mutex );
		if ( (isoch_urb = SIMPLEQ_FIRST( &ep->urb_pending_q ) ) == NULL )
		{
			chip_idea_slogf( dcctrl, 5, "%s - No pending ISOCH URBs for %s ep %d", __FUNCTION__,  ep->dir ? "In" : "Out", ep->ep_num );
			chip_idea_pthread_mutex_unlock( &ep->mutex );
			continue;
		}
		else
		{
			/* The ENDPTCOMPLETE (ep_complete_mask) bit will be set when any dTD completes not just when the dTD marked 
			 * with IOC completes, so we cannot rely on the ENDPTCOMLETE bit to know whether this endpoint actually completed
			 * the dTDs for the full frame list or not. Check the ACTIVE bit of the dTD marked for IOC to determine if the 
			 * completion interrupt is for this endpoint or not. For ISOC we always mark the last frame in the frame list with IOC.
			 */
			int ioc_idx = ep->isoch_dtd_head + (isoch_urb->isoch_nframes - 1);
			/* Handle buffer wrap */
			if (ioc_idx >= dcctrl->itds)
				ioc_idx = ioc_idx - dcctrl->itds;

			/* If the last frame in the list is still marked active then wait for the next interrupt */
			if (ep->isoch_dtds[ioc_idx].size_ioc_sts & DTD_STATUS_ACTIVE)
			{
				chip_idea_pthread_mutex_unlock( &ep->mutex );
				continue;
			}
			SIMPLEQ_REMOVE_HEAD( &ep->urb_pending_q, link );
		}

		dtd = ep->isoch_dtds;
		actual_len = 0;

		if ( isoch_urb->isoch_nframes > ep->dtd_count)
		{
			chip_idea_slogf( dcctrl, 1, "%s - WARNING: Trying to complete more frames then outstanding dTDs on ep %d %s",
				 __FUNCTION__,  ep->ep_num, ep->dir ? "In" : "Out");
		}

		for (i = 0; i < isoch_urb->isoch_nframes; i++)
		{
			if ( ( error = (dtd[ep->isoch_dtd_head].size_ioc_sts & DTD_ERROR_MASK) ) )
			{
				if ( (status = dtd_check_errors( dcctrl, &dtd[ep->isoch_dtd_head], error )) != EOK )
				{
					/* If any TD/Frame in the URB has an error, save it away and
					 * continue processing the rest of the TDs/Frames. When
					 * we complete the URB, pass this saved error code
					 * to give the upper layers an indication of a bad
					 * frame within the URB Frame list.
					 */
					urb_error = status;
				}
			}

			remainder = (dtd[ep->isoch_dtd_head].size_ioc_sts & 0x7FFF0000) >> 16;
			isoch_urb->isoch_flist[i].frame_len = isoch_urb->isoch_flist[i].frame_len - remainder;
			actual_len += isoch_urb->isoch_flist[i].frame_len;

			if ( (dtd[ep->isoch_dtd_head].next_td_ptr & DTD_NEXT_TERMINATE) )
			{
				if (i < (isoch_urb->isoch_nframes - 1))
				{
					chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: Unexpected termination of dTD list on ep %d %s, break", __func__,
						ep->ep_num, ep->dir ? "In" : "Out");
					urb_error = USBD_STATUS_DATA_UNDERRUN;
				}
				else
					chip_idea_slogf( dcctrl, 3, "%s: Reached end of dTD list on ep %d %s, break", __func__,
						ep->ep_num, ep->dir ? "In" : "Out");

				/* Transfer has stopped clear active_urb */
				ep->active_urb = NULL;
				break; 
			}

			/* Advance the dTD head index (accounting for dTD buffer wrap */
			ep->isoch_dtd_head++;
			if (ep->isoch_dtd_head >= dcctrl->itds)
				ep->isoch_dtd_head = 0;
		}

		isoch_urb->urb->actual_len += actual_len;
		ep->dtd_count -= i;				/* Decrement the active DTD count */

		if (i >= isoch_urb->isoch_nframes || ep->active_urb == NULL)
		{
			chip_idea_pthread_mutex_unlock( &ep->mutex );
			isoch_urb->urb->urb_complete_cbf( ep->ep, isoch_urb->urb, urb_error, 0 );
			chip_idea_pthread_mutex_lock( &ep->mutex );
			/* Now that we have completed the urb we can place it on the free_q to be re-used */
			SIMPLEQ_INSERT_TAIL( &ep->urb_free_q, isoch_urb, link );

			if ((isoch_urb = SIMPLEQ_FIRST( &ep->urb_pending_q )) == NULL )
			{
				chip_idea_slogf( dcctrl, 5, "%s - Reached end of isoch urb pending list on ep %d %s", __FUNCTION__,  ep->ep_num, ep->dir ? "In" : "Out");
				ep->active_urb = NULL;
			}
			else
				ep->active_urb = isoch_urb->urb;
		}
		else
		{
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: Incomplete Framelist on ep %d - %s, index = %d, nframes = %d, dtd_count = %d",
				__func__ , ep->ep_num, ep->dir ? "In" : "Out", i, isoch_urb->isoch_nframes, ep->dtd_count);
		}
		chip_idea_pthread_mutex_unlock( &ep->mutex );
	}

	return( EOK );
}

static int
chip_idea_dtd_complete_irq( usbdc_device_t *udc, uint32_t ep_complete_mask )
{
	chip_ideadc 					*dcctrl = udc->dc_data;
	chip_idea_endpoint_t 			*ep;
	int								epnum = 0;
	volatile chip_idea_dtd_t		*dtd;
	int 							i;
	uint32_t 						remainder = 0;
	uint32_t 						transfer_length, actual_len; 
	uint32_t 						status = EOK; 
	iousb_transfer_t 				*urb = 0;
	uint32_t 						error; 
	int								complete;

	for ( ; ep_complete_mask ; ep_complete_mask >>= 1, epnum++ ) {
		if ( !(ep_complete_mask & 1) ) {
			continue;
		}
		
		ep = &dcctrl->ep[ epnum ];
		/* Only process NON-ISOCH completions */
		if (ep->type == USB_ENDPOINT_XFER_ISOC)
			continue;
		status = EOK;

		if ( !ep->active_urb ) {
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "chip_idea_dtd_complete_irq : no active URB for %s ep %d", ep->dir ? "in" : "out", ep->ep_num ); 
			continue;
		}	
		dtd = ep->dtd_head; 
		transfer_length = ep->transfer_len; 
		actual_len = 0;
		complete = 0;

		for ( i = 0 ; i < ep->dtd_count ; i++, dtd++ ) {
			if ( dtd->size_ioc_sts & DTD_STATUS_ACTIVE ) {
				// There is a known issue where the endpoint completion interrupt gets processed
				// before updates to the TD become visible.  To handle this case, the TD is
				// polled until the active bit becomes low.
				chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s()%d: TD %d still active on ep %d ",
								__func__, __LINE__, i, epnum );

				// Wait and check again for TD to become inactive
				usleep(10);
				if ( dtd->size_ioc_sts & DTD_STATUS_ACTIVE ) {
					chip_idea_slogf( dcctrl, _SLOG_CRITICAL, 
									"%s()%d: ERROR TD %d still active.  Giving up and dropping transfer.",
									__func__, __LINE__, i );
					complete = 0;
					break;
				}

				chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: Recovered", __func__ );

			} 
		
			if ( ( error = (dtd->size_ioc_sts & DTD_ERROR_MASK) ) ) {
				if ( (status = dtd_check_errors( dcctrl, dtd, error )) != EOK ) {
					chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: dtd error on ep: %d", __func__, epnum );
					complete = 1;
					break; 
				}
			}
		
			remainder = (dtd->size_ioc_sts & 0x7FFF0000) >> 16;
			actual_len += min( transfer_length, EP_MAX_LENGTH_TRANSFER ) - remainder; 
			transfer_length -= min( transfer_length, EP_MAX_LENGTH_TRANSFER ) - remainder; 

			if ( remainder ) {
				/* short packet - complete the transfer */
				status |= USBD_STATUS_DATA_UNDERRUN; 
				complete = 1;
	
#if 0
				if ( ep->dir == CHIP_IDEA_EP_DIR_OUT ) {
					/* XXX do I really want to be flushing here? */
					out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTFLUSH, 1 << ep->ep_num );
				}
				else {
					chip_idea_slogf( dcctrl, 3,"%s : Transmit dtd remaining length not zero, remain = 0x%x", __func__,  remainder );
				}
#endif
	
				break; 
			}
	
			if ( (dtd->next_td_ptr & DTD_NEXT_TERMINATE) ) {
				break; 
			}
		}

		if ( (urb = ep->active_urb) == NULL ) {
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s()%d: NULL urb on ep %d, %d, len %d, compl=%d",
							__func__, __LINE__, epnum, ep->ep_num,	transfer_length, complete);
			continue;
		}

		urb->actual_len += actual_len;
		ep->active_urb = 0;  

		//if short pkt, error, or full buffer_len is done, finish it
		// otherwise, enqueue another transfer
		if ( complete || actual_len == 0 || urb->actual_len>=urb->buffer_len){
			/* clear the dtd queue */
			dtd = ep->dtd_head;
	
			for( i = 0 ; i < CHIP_IDEA_MAX_LINKED_DTD ; i++, dtd++ ) {
				dtd->next_td_ptr = DTD_NEXT_TERMINATE;
				dtd->size_ioc_sts = 0;
			}

			chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
			urb->urb_complete_cbf( ep->ep, urb, status, 0 );
			chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );
		}else{
			// enque another transfer, 
			// we have recursive mutex, don't need to unlock it
			chip_idea_transfer( dcctrl->udc, urb, ep->ep, 
				urb->buffer + urb->actual_len, urb->buffer_len - urb->actual_len, ep->flags );
		}
	}
	
	return( EOK );
}

static void
chip_idea_portchange_irq( usbdc_device_t *udc )
{
	chip_ideadc 	*dcctrl = udc->dc_data;
	uint32_t 		speed;
	uint32_t 		portstatus;
	
	portstatus = in32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX );


	if( dcctrl->reset_state ) {
		/* Bus resetting is finished */
		if ( !(portstatus & PORTSCX_PORT_RESET) ) {
			/* Get the speed */
			speed = CHIPIDEA_GET_SPEED();

				switch ( speed ) {
					case PORTSCX_PORT_SPEED_HIGH:
						chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s : High speed device", __func__  );
						udc->usbdc_self->usbdc_set_device_speed( udc, IOUSB_DEVICE_HIGH_SPEED );
						break;
					case PORTSCX_PORT_SPEED_FULL:
						chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s : Full speed device", __func__ );
						udc->usbdc_self->usbdc_set_device_speed( udc, IOUSB_DEVICE_FULL_SPEED );
						break;
					case PORTSCX_PORT_SPEED_LOW:
						chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s : Low speed device", __func__  );
						udc->usbdc_self->usbdc_set_device_speed( udc, IOUSB_DEVICE_LOW_SPEED );
						break;
					default:
						break;
				}
				/* Reset state must be reported after speed has been set */
				udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_RESET );
				dcctrl->reset_state = 0;
			}
	} else {
		/* Portchange occurs after reset on resume.
		 * If the controller was not in reset, the portchange is due to a resume. */
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: USB device resume.... ", __func__ );
		udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_RESUMED );
	}
}

/*
 *  Interrupt handler for USB reset received
 */

static void
chip_idea_reset_irq( usbdc_device_t *udc )
{
	chip_ideadc 	*dcctrl = udc->dc_data;
	uint32_t 		timeout;
	uint32_t		portstatus;

	portstatus = in32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX );

	/* Clear the device address */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_DEVICEADDR, 
					in32( dcctrl->IoBase + CHIP_IDEA_UOG_DEVICEADDR ) & ( ~USB_DEVICE_ADDRESS_MASK) );

	/* Clear all the setup token semaphores */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSETUPSATA, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSETUPSATA ) );

	/* Clear all the endpoint complete status bits */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCOMPLETE, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCOMPLETE ) );

	timeout = 10000000UL;
	/* Wait until all endptprime bits cleared */
	while ( (in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME )) && --timeout ) {
		continue;
	}
	if ( timeout == 0 ) {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: endptprime timeout", __func__ );
	}

	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTFLUSH, 0xFFFFFFFF );

	/* enable ep0 */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL0, EPCTRL_TX_ENABLE | EPCTRL_RX_ENABLE );

	/* setup endpoint0 dQH */
	chip_idea_ep_qh_setup( udc, &dcctrl->ep[0], USB_ENDPOINT_XFER_CONTROL, USB_MAX_CTRL_PAYLOAD );
	chip_idea_ep_qh_setup( udc, &dcctrl->ep[16], USB_ENDPOINT_XFER_CONTROL, USB_MAX_CTRL_PAYLOAD );

	if ( portstatus & PORTSCX_PORT_RESET )  {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: Bus reset...", __func__ );
	}
	else {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: controller reset...", __func__ );
	}

	dcctrl->reset_state = 1;

	return; 
}

uint32_t
chip_idea_interrupt( usbdc_device_t *udc )
{
	chip_ideadc 	*dcctrl = udc->dc_data;
	uint32_t 		otg_sc;
	uint32_t 		irq_src;
	uint32_t		ec_mask;
	uint32_t 		portstatus;

	chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );

	irq_src = in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBSTS );// & CHIP_IDEA_DEFAULT_IRQ_EN_MASK;

	// read OTG Status/Control
	otg_sc = in32( dcctrl->IoBase + CHIP_IDEA_UOG_OTGSC );
	
	// clear OTG status bits
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_OTGSC, otg_sc );

	/* Clear notification bits */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBSTS, irq_src );

	/* USB interrupt */
	if ( irq_src & (USB_STS_INT | USB_STS_ERR ) ) {
		/* completion of dTD */
		if ( (ec_mask = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCOMPLETE ) ) ) {
			/* Clear the bits in the register */
			out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCOMPLETE, ec_mask );

			/* Process isoch completions first as they are more time sensitive */
			chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
			chip_idea_dtd_isoch_complete_irq( udc, ec_mask );
			chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );

			chip_idea_dtd_complete_irq( udc, ec_mask );
		}

		/* Setup packet, we only support ep0 as control ep */
		if( in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSETUPSATA ) & 1 ) {
			/* receive setup packet */
			chip_idea_ep0_setup_irq( udc );
		}
	}

	/* Reset Received */
	if ( irq_src & USB_STS_RESET_RECEIVED ) {
		chip_idea_reset_irq( udc );
	}

	/* Port Change */
	if ( irq_src & USB_STS_PORT_CHANGE ) {
		chip_idea_portchange_irq( udc );
	}
    
	/* Sleep Enable (Suspend) */
	if ( irq_src & USB_STS_SUSPEND ) {
		portstatus = in32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX );
		if( (portstatus & PORTSCX_PORT_SUSPEND) && !( otg_sc & OTG_BSV ) ) {
			/* power management from needs proper diconnect notification - controller not reporting disconnected state */
			/* this may need to change to report SUSPEND */
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "USB device disconnect ( Suspend ) ... " );
			udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_REMOVED );
		}
		else if ( udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_SUSPEND_REQUEST ) == EOK ) {
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "USB device suspend ...." );
			udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_SUSPENDED );
		}
	}

	/* USB cable disconnected: B Session End Interrupt */
	if ( ( otg_sc & OTG_BSEIS ) && ( otg_sc & OTG_BSE ) ) {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "USB device disconnect ( Session End ) ... " );
		udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_REMOVED );
	}

	/* USB cable connected: B Session Valid Interrupt */
	if ( ( otg_sc & OTG_BSVIS ) && ( otg_sc & OTG_BSV ) ) {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "USB device connect.... " );
		udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_INSERTED );
	}
    
	/* error */
	if (irq_src & USB_STS_SYS_ERR) {
		chip_idea_slogf( dcctrl, _SLOG_CRITICAL, "%s: Error interrupt %x", __func__, irq_src );
	} 

	CHIP_IDEA_EXTRA_INTERRUPT_CALLOUT

	chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );

	return( EOK );
}


/*
	Initialize the hardware
*/
int
chip_idea_chip_init( chip_ideadc *dcctrl )
{
	uint32_t 			tmp, timeout;
	int 				error;
	chip_idea_dtd_t 	*dtd;
	int 				i,j;
	pthread_mutexattr_t mattr;

	chip_idea_slogf( dcctrl, _SLOG_NOTICE, "%s: id %x dev %x portscx %x", __func__,
			in32( dcctrl->IoBase + CHIP_IDEA_UOG_ID), 	
			in32( dcctrl->IoBase + CHIP_IDEA_UOG_HWDEVICE ),
			in32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX) ); 

	/*1. stop and reset the USB controller */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD) & ~ USB_CMD_RUN_STOP );
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD) |  USB_CMD_CTRL_RESET );
	
	/* wait for reset to complete */
	timeout = 10000000	;
	while ( (in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD) & USB_CMD_CTRL_RESET) && --timeout ) {
	} 	

	if ( timeout == 0 ) {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: TIMEOUT", __func__ );
		error = ETIMEDOUT;
		goto fail;
	}

	/* 2. Set Controller Mode in the USBMODE register to device mode */
	CHIPIDEA_SET_MODE();

	// force to full speed if specified at command line
	if ( !(dcctrl->udc->hw_ctrl.capabilities & DC_CAP_HIGH_SPEED ) ) {
		out32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX, (in32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX ) | PORTSCX_PORT_FORCE_FULL_SPEED) );
	}

	CHIP_IDEA_BOARD_SPECIFIC_INIT

	/* 3.Clear the USB status register */
	tmp = in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBSTS );
 	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBSTS, tmp );

	/*4. allocate and Initialize device queue heads in system memory */
	
	/* initialized QHs, take care the 2K align */
	dcctrl->qh_mem_sz = (dcctrl->num_ep * 2) * sizeof( chip_idea_qh_t ) + CHIP_IDEA_QH_ALIGN;

	if ( (dcctrl->qh_mem = mmap( NULL, dcctrl->qh_mem_sz, PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_SHARED | MAP_ANON | MAP_PHYS, NOFD, 0 )) == MAP_FAILED ) {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: mmap failed to alloc qh_mem", __func__ );
		error = ENOMEM;
		goto fail;
	}

	memset( dcctrl->qh_mem, 0, dcctrl->qh_mem_sz );
	dcctrl->ep_qh = (chip_idea_qh_t *) ( ((_uint32) dcctrl->qh_mem + (CHIP_IDEA_QH_ALIGN - 1) ) & USB_EP_LIST_ADDRESS_MASK);


	/*5. Configure ENDPOINTLISTADDR pointer */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPOINTLISTADDR, mphys( dcctrl->ep_qh) );

	/*6 allocate and Initialize dTDs in system memory and link to dQHs */

	/* alloc 8-Dword dTD block of memory aligned to 32-byte boundaries, alloc 4 dTDs(64K) for each endpoint */
	dcctrl->dtd_mem_sz = (dcctrl->num_ep * 2) * (sizeof( chip_idea_dtd_t ) * CHIP_IDEA_MAX_DTD_PER_ENDPOINT) + 1;

	if ( (dcctrl->dtd_mem = mmap( NULL, dcctrl->dtd_mem_sz, PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_SHARED | MAP_ANON | MAP_PHYS, NOFD, 0 )) == MAP_FAILED ) {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: mmap failed to alloc dtd_mem", __func__ );
		error = ENOMEM;
		goto fail2;
	}

	memset( dcctrl->dtd_mem, 0, dcctrl->dtd_mem_sz );
	dtd = (chip_idea_dtd_t *) (((_uint32) dcctrl->dtd_mem + 31) & 0xFFFFFFE0);

	/* Initialize endpoint structures */

	pthread_mutexattr_init( &mattr );
	pthread_mutexattr_setrecursive( &mattr, PTHREAD_RECURSIVE_ENABLE );

	/* init EP dTD and link 4 dTDs to each dQH */
	tmp = mphys( dtd );
	for( i = 0 ; i < dcctrl->num_ep ; i++ ) {
		if( pthread_mutex_init( &dcctrl->ep[i].mutex, &mattr ) == -1 ) {
			error = errno;
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: pthread_mutex_init() failed for ep %d Out error = %d", __func__, i, error );
			goto fail3;
		}

		// init RX
		dcctrl->ep[i].qh 		= &dcctrl->ep_qh[(i * 2)];		
		dcctrl->ep[i].dtd_head 	= dtd;		
		dcctrl->ep[i].dtd_addr 	= tmp;
		dtd += CHIP_IDEA_MAX_DTD_PER_ENDPOINT;
		tmp += CHIP_IDEA_MAX_DTD_PER_ENDPOINT * sizeof(chip_idea_dtd_t);

		// init TX
		if( pthread_mutex_init( &dcctrl->ep[i+16].mutex, &mattr ) == -1 ) {
			error = errno;
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: pthread_mutex_init() failed for ep %d In error = %d", __func__, i, error );
			pthread_mutex_destroy(&dcctrl->ep[i].mutex);	
			goto fail3;
		}

		dcctrl->ep[i+16].qh 		= &dcctrl->ep_qh[(i*2)+1];
		dcctrl->ep[i+16].dtd_head 	= dtd;		
		dcctrl->ep[i+16].dtd_addr 	= tmp;
		dtd += CHIP_IDEA_MAX_DTD_PER_ENDPOINT;
		tmp += CHIP_IDEA_MAX_DTD_PER_ENDPOINT * sizeof(chip_idea_dtd_t);
	}

	return( EOK );

fail3:
	for (j = 0; j < i; j++)
	{
		pthread_mutex_destroy(&dcctrl->ep[j].mutex);	
		pthread_mutex_destroy(&dcctrl->ep[j+16].mutex);	
	}	
fail2:
	munmap( dcctrl->qh_mem, dcctrl->qh_mem_sz );		
fail:
	return error;	
}

void
chip_idea_chip_fini( chip_ideadc *dcctrl ) {

	int i;

	CHIP_IDEA_BOARD_SPECIFIC_FINI

	for (i = 0; i < dcctrl->num_ep; i++)
	{
		pthread_mutex_destroy(&dcctrl->ep[i].mutex);	
		pthread_mutex_destroy(&dcctrl->ep[i+16].mutex);	
	}	

	if ( dcctrl->qh_mem != MAP_FAILED ) {
		munmap( dcctrl->qh_mem, dcctrl->qh_mem_sz );
	}
	
	if ( dcctrl->dtd_mem != MAP_FAILED ) {
		munmap( dcctrl->dtd_mem, dcctrl->dtd_mem_sz );
	}		
}



int
chip_idea_process_args( chip_ideadc *dcctrl, char *args )
{
	char 		*value;
	char 		*c;
	int 		len;
	int			cnt;
	uint32_t	val;
	static char *chip_idea_opts[] = { 
		"verbose",
		"ser",
		"linkup",
		"linkdown",
		"fullspeed",
		"int_thresh",
		"PARAMETER_OVERRIDE_A",
		"PARAMETER_OVERRIDE_B",
		"PARAMETER_OVERRIDE_C",
		"PARAMETER_OVERRIDE_D",
		"itds",
		NULL
	};

	dcctrl->itds = MAX_ISOCH_DTD_PER_ENDPOINT;

	if( !args ) {
		return( EOK );	
	}

	// convert args
	len = strlen( args );
	while ( (c = strchr( args, ':' ) ) ) {
		if ( c - args > len ) {
			break;
		}
		*c = ',';
	}

	while( args && *args != '\0' ) {
		switch ( getsubopt( &args, chip_idea_opts, &value ) ) {
			case 0 :
				if ( value ) {
					dcctrl->verbosity = strtoul( value, 0, 0 );
				}
				else  {
					dcctrl->verbosity = 5;
				}
				break;

			case 1 : 	// this arg should move up for now we build a proper string desc
				if ( value ) {
					uint8_t		slen;
					uint32_t 	x;

					slen = min( strlen( value ), 127 ); // max 8bit length each char requires 2 bytes for encoding plus 2 byte header.
					dcctrl->udc->serial_string = calloc( 1, 3 + 2 * slen );
					if (dcctrl->udc->serial_string) {
						dcctrl->udc->serial_string[0] = 2 + slen * 2; // string header is 2 bytes
						dcctrl->udc->serial_string[1] = USB_DESC_STRING;

						for ( x = 1 ; x < slen + 1 ; x ++ ) {
							dcctrl->udc->serial_string[x * 2] = value[x - 1];
						}
					}
					else {
						chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: ser argument error.", __func__ );
					}
				}
				break;

			case 2 :
        		dcctrl->flags &= ~CHIP_IDEA_FLAGS_INITIAL_LINK_DISCONNECTED;
				break;

			case 3 :
        		dcctrl->flags |= CHIP_IDEA_FLAGS_INITIAL_LINK_DISCONNECTED;
				break;

			case 4 :
				dcctrl->udc->hw_ctrl.capabilities &= ~DC_CAP_HIGH_SPEED;	// only fullspeed link is supported or force linkspeed.
				break;

			case 5 :
				if ( value ) {
					val = min( USB_CMD_ITC_64_MICRO_FRM, strtol( value, 0, 10 ) );
					if ( !val )  {
						dcctrl->int_thresh = USB_CMD_ITC_NO_THRESHOLD; // immeadiate interrupts
					}
					else {
						for ( cnt = 6 ; cnt ; cnt-- ) {
							if ( val & USB_CMD_ITC_64_MICRO_FRM )
								break;
							val = val << 1;
						}
						dcctrl->int_thresh = 1 << cnt; // set thresh to power of 2
					}
				}
				break;

			case 6 : // PARAMETER_OVERRIDE_A
				if ( value ) {
					dcctrl->phy_tuning[USBPHY_PARAMETER_OVERRIDE_A] = strtoul( value, NULL, 16 );
				}
				break;
			case 7 : // PARAMETER_OVERRIDE_B
				if ( value ) {
					dcctrl->phy_tuning[USBPHY_PARAMETER_OVERRIDE_B] = strtoul( value, NULL, 16 );
				}
				break;
			case 8 : // PARAMETER_OVERRIDE_C
				if ( value ) {
					dcctrl->phy_tuning[USBPHY_PARAMETER_OVERRIDE_C] = strtoul( value, NULL, 16 );
				}
				break;
			case 9 : // PARAMETER_OVERRIDE_D
				if ( value ) {
					dcctrl->phy_tuning[USBPHY_PARAMETER_OVERRIDE_D] = strtoul( value, NULL, 16 );
				}
				break;
			
			case 10: // Max isoch dTDs per endpoint
				dcctrl->itds = strtoul( value, NULL, 0 );
				break;
			default :
				if ( value )
					CHIP_IDEA_EXTRA_PROCESS_ARGS_CALLOUT
				break;
		}
	}

	return( EOK );
}

uint32_t
chip_idea_init( usbdc_device_t *udc, io_usbdc_self_t *udc_self, char *args )
{
	chip_ideadc				*dcctrl;
	pthread_mutexattr_t 	mattr;
	uint32_t				dcparams;
	int						error;

	udc->hw_ctrl.cname 					= CHIP_IDEA_DEFAULT_CTRL_NAME;
	//there is no real limitation now since we can break down large transfer into small
	//transfer, make it 1MB which should be enough.
	udc->hw_ctrl.max_transfer_size 		= 1<<20; 
	udc->hw_ctrl.max_unaligned_xfer 	= CHIP_IDEA_MAX_TRANSFER_SIZE;
	udc->hw_ctrl.buff_alignment_mask 	= 0x00;
	udc->hw_ctrl.capabilities 			= DC_CAP_FULL_SPEED | DC_CAP_HIGH_SPEED | DC_CAP_TEST_MODES_SUPPORTED | USBD_HCD_CAP_ISOCH_STREAM;

	/* allocate memory for our chip handle  */
	if ( (udc->dc_data = dcctrl = (void *) calloc( 1, sizeof( *dcctrl ) ) ) == NULL ) {
		error = ENOMEM;
		goto fail;
	}
	CHIP_IDEA_EXTRA_ALLOC_CALLOUT

	//get the back pointer
	dcctrl->udc 					= udc;
	dcctrl->flags 					= CHIP_IDEA_FLAGS_INITIAL_LINK_DISCONNECTED;
	dcctrl->verbosity 				= _SLOG_ERROR;
//	dcctrl->num_ep 		= CHIP_IDEA_MAX_NUM_USB_ENDPOINTS; 	// read DCCPARMS to ge number of supported endpoints.. (see below)

	chip_idea_process_args( dcctrl, args );

	pthread_mutexattr_init( &mattr );
	pthread_mutexattr_setrecursive( &mattr, PTHREAD_RECURSIVE_ENABLE );

	if( pthread_mutex_init( &dcctrl->usb_mutex, &mattr ) == -1 ) {
		error = errno;
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: pthread_mutex_init() failed. error = %d", __func__, error );
		goto fail2;
	}

	/* map in device registers */
	if ( (dcctrl->IoBase = (uint32_t) mmap_device_memory( 0, UDC_SIZE,
			PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_SHARED | MAP_PHYS,
			PCI_MEM_ADDR( udc->hw_ctrl.pci_inf->CpuBaseAddress[0] ) ) ) == (uint32_t) MAP_FAILED ) {
		error = errno;
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: mmap() failed to alloc IoBase regs. error = %d ", __func__, error );
		goto fail3;
	}

	dcparams = in32( dcctrl->IoBase + CHIP_IDEA_UOG_DCCPARAMS );

	if ( !( dcparams & CHIP_IDEA_DEVICE_CAPABLE ) ) {
		error = ENOTSUP;
		goto fail4;
	}

	// get number of supported endpoint
	dcctrl->num_ep = dcparams & CHIP_IDEA_MASK_DEN;
	
	chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );
		
	CHIP_IDEA_BOARD_SPECIFIC_TRANSCEIVER_INIT
	if ( ( error = chip_idea_chip_init( dcctrl ) ) != EOK ) {
		chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s: chip_idea_chip_init() failed. error = %d ", __func__, error );
		chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
		goto fail4;
	}
	
	chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );

	return( EOK );

fail4:
	munmap_device_io( dcctrl->IoBase, UDC_SIZE);
fail3:
	pthread_mutex_destroy( &dcctrl->usb_mutex );	
fail2:

	CHIP_IDEA_EXTRA_FREE_CALLOUT

	if (udc->serial_string != NULL) 
		free(udc->serial_string);
	free( udc->dc_data );		
fail:
	return error;	
}

uint32_t
chip_idea_start( usbdc_device_t *udc )
{
	chip_ideadc		*dcctrl = udc->dc_data;

	chip_idea_pthread_mutex_lock( &dcctrl->usb_mutex );

	/* Set interrupt enables */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBINTR, CHIP_IDEA_DEFAULT_IRQ_EN_MASK );
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_OTGSC, (CHIP_IDEA_DEFAULT_OTG_IRQ_EN_MASK | in32( dcctrl->IoBase + CHIP_IDEA_UOG_OTGSC ) ) );

	/* Set Run/Stop bit to Run Mode */
	if ( !(dcctrl->flags & CHIP_IDEA_FLAGS_INITIAL_LINK_DISCONNECTED) ) { 
		chip_idea_set_bus_state( udc, IOUSB_BUS_STATE_CONNECTED );
	}
	else {
		chip_idea_set_bus_state( udc, IOUSB_BUS_STATE_DISCONNECTED );
	}

//	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, in32(dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD) ); // | USB_CMD_RUN_STOP );

	chip_idea_pthread_mutex_unlock( &dcctrl->usb_mutex );
	
	return( EOK );
}

uint32_t
chip_idea_stop( usbdc_device_t *udc )
{
	chip_ideadc 	*dcctrl = udc->dc_data;
	
	/* clear interrupt enables */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBINTR, 0 );
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_OTGSC, (~(CHIP_IDEA_OTG_IRQ_EN_MASK) & in32( dcctrl->IoBase + CHIP_IDEA_UOG_OTGSC )) );
	/* disable all INTR */
	
	/* disconnect from bus and stop the controller */
	chip_idea_set_bus_state( udc, IOUSB_BUS_STATE_DISCONNECTED );
	
	return( EOK );
}

uint32_t
chip_idea_shutdown( usbdc_device_t *udc )
{
	chip_ideadc 	*dcctrl = udc->dc_data;

	// io-usb-dcd not calling stop function currently 
	chip_idea_stop( udc );

	chip_idea_chip_fini( dcctrl );
	
	// Add board specific shutdown
	munmap_device_io( dcctrl->IoBase, UDC_SIZE);
	
	pthread_mutex_destroy( &dcctrl->usb_mutex );

	if ( udc->serial_string ) { 				// serial string override should move to upper layer.
		free( udc->serial_string );
	}

	CHIP_IDEA_EXTRA_FREE_CALLOUT
	free( udc->dc_data );

	return( EOK );
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/devu/dc/ci/chipidea.c $ $Rev: 741231 $")
#endif
