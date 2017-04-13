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

#include "../chipidea.h"
#include <unistd.h>
#include <atomic.h>
#include <arm/mx6x.h>

struct mx6_extra_data {
	_uint32 phy_base;
	_uint32 *phy_virt;
	_uint32 vbus_enable;
	struct sigevent	event;
	int    iid;
	int    tid;
	int    chid;
	int    coid;
	int    phy_irq;
	unsigned connected;
};


#define MX_USBPHY1_BASE      0x020C9000
#define MX_USBPHY1_SIZE      0x84
#define MX_USBPHY1_IRQ       76
#define MX6_USBPHY1_CTRL     0x030
#define MX6_USBPHY1_CTRL_SET 0x034
#define MX6_USBPHY1_CTRL_CLR 0x038
#define MX6_USBPHY1_CTRL_TOG 0x03c
#define USBPHY_CTRL_DEVPLUGIN_IRQ      (1<<12)
#define USBPHY_CTRL_ENIRQDEVPLUGIN     (1<<11)
#define USBPHY_CTRL_DEVPLUGIN_POLARITY (1<<5)
#define USBPHY_CTRL_ENDEVPLUGINDETECT  (1<<4)

#define MX6_USBPHY1_STATUS   0x040
#define USBPHY_STATUS_DEVPLUGIN_STATUS (1<<6)

#define USBPHY_PULSE_CODE_INTR (_PULSE_CODE_MINAVAIL+1)
#define USBPHY_PULSE_CODE_EXIT (_PULSE_CODE_MINAVAIL+2)

void
mx6sabrelite_extra_alloc_callout(usbdc_device_t *udc)
{
	chip_ideadc      *dcctrl = (void *)udc->dc_data;
	struct mx6_extra_data *extra_data = NULL;

	dcctrl->extra = (struct mx6_extra_data *) calloc ( 1, sizeof (struct mx6_extra_data) );
	extra_data = (struct mx6_extra_data *) dcctrl->extra;
	extra_data->tid = -1;
	extra_data->iid = -1;
}

void
mx6sabrelite_extra_free_callout(usbdc_device_t *udc)
{
	chip_ideadc           *dcctrl = (void *)udc->dc_data;
	struct mx6_extra_data *extra_data = (struct mx6_extra_data *) dcctrl->extra;
	struct sigevent       event;

	if (extra_data->tid != -1)
	{
		uint32_t base=(uint32_t)(extra_data->phy_virt);

		/* Disable USBPHY Plugin detection */
		out32(base+MX6_USBPHY1_CTRL_CLR, USBPHY_CTRL_ENDEVPLUGINDETECT | USBPHY_CTRL_ENIRQDEVPLUGIN | USBPHY_CTRL_DEVPLUGIN_POLARITY);

		InterruptDetach( extra_data->iid );

		/* Terminate USBPHY interrupt thread */
		memcpy(&event, &extra_data->event, sizeof(struct sigevent));
		event.sigev_code = USBPHY_PULSE_CODE_EXIT;
		MsgDeliverEvent(extra_data->tid, &event);
		pthread_join(extra_data->tid, NULL);

		ConnectDetach( extra_data->coid );
		ChannelDestroy( extra_data->chid );
		extra_data->iid = -1;
		extra_data->tid = -1;
	}

	if(extra_data->phy_virt)
		munmap_device_memory( extra_data->phy_virt, MX_USBPHY1_SIZE);

	free (dcctrl->extra);
}

void
mx6sabrelite_board_specific_link_up( chip_ideadc *dcctrl)
{
	struct mx6_extra_data *extra_data = (struct mx6_extra_data *) dcctrl->extra;
	uint32_t              base = (uint32_t)(extra_data->phy_virt);

	if (extra_data->vbus_enable && base != 0)
	{
		/* Clear polarity to detect insertion */
		out32(base+MX6_USBPHY1_CTRL_CLR, USBPHY_CTRL_DEVPLUGIN_POLARITY);
		/* Enable Plugin status detect interrupt */
		out32(base+MX6_USBPHY1_CTRL_SET, USBPHY_CTRL_ENIRQDEVPLUGIN);
	}
}

void
mx6sabrelite_board_specific_link_down( chip_ideadc *dcctrl)
{
	struct mx6_extra_data *extra_data = (struct mx6_extra_data *) dcctrl->extra;
	uint32_t base = (uint32_t)(extra_data->phy_virt);

	if (extra_data->vbus_enable && base != 0)
	{
		/* Disable Plugin status detect interrupt */
		out32(base+MX6_USBPHY1_CTRL_CLR, USBPHY_CTRL_ENIRQDEVPLUGIN);
	}
}

void *
mx6sabrelite_extra_interrupt_handler(chip_ideadc *dcctrl)
{
	struct _pulse          pulse;
	struct mx6_extra_data  *extra_data = (struct mx6_extra_data *) dcctrl->extra;
	int                    base = (uint32_t)(extra_data->phy_virt);

	if (base == 0)
	{
		chip_idea_slogf( dcctrl, _SLOG_INFO, "%s: Failed to attach handler, no phy address", __func__);
		return (NULL);
	}

	while( 1 )
	{
		if( MsgReceivePulse( extra_data->chid, &pulse, sizeof(pulse), NULL ) != EOK )
		{
			chip_idea_slogf( dcctrl, _SLOG_INFO, "%s: MsgReceivePulse() failed - %s", __func__, strerror(errno));
			break;
		}

		switch( pulse.code )
		{
			case USBPHY_PULSE_CODE_INTR:
				if ( extra_data->vbus_enable)
				{
					uint32_t phystatus = in32(base+MX6_USBPHY1_STATUS);

					if (!extra_data->connected && phystatus & USBPHY_STATUS_DEVPLUGIN_STATUS)
					{
						chip_idea_slogf( dcctrl, _SLOG_INFO, "%s: USB device connect ... ", __func__ );
						atomic_set(&extra_data->connected, 1);
						dcctrl->udc->usbdc_self->usbdc_device_state_change( dcctrl->udc, IOUSB_DEVICE_STATE_INSERTED );
					}
					else if (extra_data->connected && !(phystatus & USBPHY_STATUS_DEVPLUGIN_STATUS))
					{
						chip_idea_slogf( dcctrl, _SLOG_INFO, "%s: USB device disconnect ... ", __func__ );
						atomic_clr(&extra_data->connected, 1);
						dcctrl->udc->usbdc_self->usbdc_device_state_change( dcctrl->udc, IOUSB_DEVICE_STATE_REMOVED );
					}
				}
				/* Clear the interrupt status
				 * Note: The polarity dictates whether we are are interrupting
				 *       on the insertion or the removal, so we must toggle
				 *       DEVPLUGIN_POLARITY to clear the interrupt condition and
				 *       to detect the inverse condition.
				 */
				out32(base+MX6_USBPHY1_CTRL_TOG, USBPHY_CTRL_DEVPLUGIN_POLARITY);
				out32(base+MX6_USBPHY1_CTRL_CLR, USBPHY_CTRL_DEVPLUGIN_IRQ);
				InterruptUnmask( extra_data->phy_irq, extra_data->iid );
				break;

			case USBPHY_PULSE_CODE_EXIT:
				chip_idea_slogf( dcctrl, _SLOG_INFO, "%s: Got exit pulse ... ", __func__ );
				pthread_exit(NULL);

			default:
				break;
		}
	}

	return NULL;
}

int 
mx6sabrelite_create_extra_interrupt_handlers( chip_ideadc *dcctrl)
{
	pthread_attr_t         pattr;
	struct sched_param     param;
	int                    retval;
	struct mx6_extra_data  *extra_data = (struct mx6_extra_data *) dcctrl->extra;
	int                    base = (uint32_t)(extra_data->phy_virt);

	if (base == 0)
	{
		return (ENOTSUP);
	}

	if( (extra_data->chid = ChannelCreate( _NTO_CHF_DISCONNECT )) < 0 )
	{
		chip_idea_slogf( dcctrl, _SLOG_INFO, "%s: - Unable to create channel", __func__);
		retval = errno;
		return(retval);
	}

	if( (extra_data->coid = ConnectAttach( 0, 0, extra_data->chid, _NTO_SIDE_CHANNEL, 0 )) < 0 )
	{
		chip_idea_slogf( dcctrl, _SLOG_INFO, "%s: - Unable to connect to channel", __func__);
		retval = errno;
		ChannelDestroy( extra_data->chid );
		return(retval);
	}

	/* Create interrupt event thread */
	pthread_attr_init(&pattr);
	pthread_attr_setschedpolicy(&pattr, SCHED_RR);
	param.sched_priority = 10;
	pthread_attr_setschedparam(&pattr, &param);
	pthread_attr_setinheritsched(&pattr, PTHREAD_EXPLICIT_SCHED);

	if( pthread_create( &extra_data->tid, &pattr, (void *) mx6sabrelite_extra_interrupt_handler, dcctrl ) )
	{
		chip_idea_slogf(dcctrl, _SLOG_INFO, "%s: Unable to create interrupt event thread for USBPHY interrupt", __func__);
		retval = errno;
		ConnectDetach( extra_data->coid );
		ChannelDestroy( extra_data->chid );
		return(retval);
	}

	/* Attach Interrupt Handler for USBPHY interrupt */
	extra_data->event.sigev_notify          = SIGEV_PULSE;
	extra_data->event.sigev_coid            = extra_data->coid;
	extra_data->event.sigev_code            = USBPHY_PULSE_CODE_INTR;
	extra_data->event.sigev_priority        = 10;
	extra_data->event.sigev_value.sival_ptr = NULL;

	if( ( extra_data->iid = InterruptAttachEvent( extra_data->phy_irq, &extra_data->event, _NTO_INTR_FLAGS_TRK_MSK ) ) == -1 )
	{
		chip_idea_slogf( dcctrl, _SLOG_INFO, "%s: InterruptAttachEvent failed for USBPHY interrupt", __func__);
		retval = errno;
		pthread_cancel( extra_data->tid );
		extra_data->tid = -1;
		ConnectDetach( extra_data->coid );
		ChannelDestroy( extra_data->chid );
		return(retval);
	}

	/* Enable plugin detect pull up */
	out32(base+MX6_USBPHY1_CTRL_SET, USBPHY_CTRL_ENDEVPLUGINDETECT);

	return EOK;
}

int
mx6sabrelite_board_specfic_init( chip_ideadc *dcctrl )
{
	int      status = EOK;
	struct mx6_extra_data *extra_data = (struct mx6_extra_data *) dcctrl->extra;

	/* If the VBUS is enabled then we need to map in the phy to use its
	 * plugin status to detect cable removal.
	 */
	if(extra_data->vbus_enable && extra_data->phy_base != 0)
	{
		if ( ( extra_data->phy_virt = mmap_device_memory( 0, MX_USBPHY1_SIZE,
			PROT_READ | PROT_WRITE | PROT_NOCACHE,
			MAP_SHARED | MAP_PHYS, extra_data->phy_base ) ) == MAP_FAILED )
		{
			status = errno;
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s : Error Mapping %x", __func__, extra_data->phy_base );
			extra_data->phy_virt=0;
			extra_data->phy_base=0;
			return (status);
		}

		if ((status = mx6sabrelite_create_extra_interrupt_handlers( dcctrl )) != EOK)
		{
			munmap_device_memory( extra_data->phy_virt, MX_USBPHY1_SIZE);
			extra_data->phy_virt=0;
			extra_data->phy_base=0;
		}
	}

	return( status );
}

int mx6sabrelite_otg_init( chip_ideadc *dcctrl )
{
	uint32_t gpio_addr;
	struct mx6_extra_data *extra_data = (struct mx6_extra_data *) dcctrl->extra;

	/* Toggle GPIO to turn off VBUS for OTG Controller in DEVICE mode */
	if (!extra_data->vbus_enable)
	{

#if defined(CONFIG_MACH_MX6Q_DMS_BA16) 
                if ( (gpio_addr = (uint32_t) mmap_device_memory( 0, sizeof(uint32_t),
                                PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_SHARED | MAP_PHYS,
                                MX6X_GPIO4_BASE + MX6X_GPIO_DR ) ) == (uint32_t) MAP_FAILED )
                {
                        return (errno);
                }

                out32(gpio_addr, in32(gpio_addr) & ~(1<<15));
#else
		if ( (gpio_addr = (uint32_t) mmap_device_memory( 0, sizeof(uint32_t),
				PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_SHARED | MAP_PHYS,
				MX6X_GPIO3_BASE + MX6X_GPIO_DR ) ) == (uint32_t) MAP_FAILED )
		{
			return (errno);
		}

		out32(gpio_addr, in32(gpio_addr) & ~(1<<22));
#endif
		munmap_device_io( gpio_addr, sizeof(uint32_t) );
	}

	return( EOK );
}

int mx6sabrelite_otg_fini( chip_ideadc *dcctrl )
{
	uint32_t gpio_addr;
	struct mx6_extra_data *extra_data = (struct mx6_extra_data *) dcctrl->extra;

	/* Startup code defaults the VBUS enabled for the OTG port in HOST mode. Restore the default
	 * if we toggle the GPIO off in the init routine, so that the HOST side driver can run without
	 * needing to hit the GPIO.
	 */
	if (!extra_data->vbus_enable)
	{
#if defined(CONFIG_MACH_MX6Q_DMS_BA16)
                if ( (gpio_addr = (uint32_t) mmap_device_memory( 0, sizeof(uint32_t),
                                PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_SHARED | MAP_PHYS,
                                MX6X_GPIO4_BASE + MX6X_GPIO_DR ) ) == (uint32_t) MAP_FAILED )
                {
                        return (errno);
                }

                out32(gpio_addr, in32(gpio_addr) | (1<<15));
#else
		if ( (gpio_addr = (uint32_t) mmap_device_memory( 0, sizeof(uint32_t),
				PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_SHARED | MAP_PHYS,
				MX6X_GPIO3_BASE + MX6X_GPIO_DR ) ) == (uint32_t) MAP_FAILED )
		{
			return (errno);
		}

		out32(gpio_addr, in32(gpio_addr) | (1<<22));
#endif
		munmap_device_io( gpio_addr, sizeof(uint32_t) );
	}
	return( EOK );
}

void
mx6sabrelite_extra_process_args_callout(chip_ideadc *dcctrl, char *options)
{
	char *value;
	struct mx6_extra_data *extra_data = (struct mx6_extra_data *) dcctrl->extra;
	extra_data->phy_base = MX_USBPHY1_BASE;
	extra_data->phy_irq = MX_USBPHY1_IRQ;
	extra_data->phy_virt = 0;
	char *mx6_opts[] = {
	#define MX_OPT_PHY_BASE     0
	"phy",
	#define MX_OPT_VBUS_ENABLE  1
	"vbus_enable",
	#define MX_OPT_PHY_IRQ      2
	"phy_irq",
	NULL};

	if ( !options || !extra_data)
		return;

	switch ( getsubopt( &options, mx6_opts, &value) )
	{
		case MX_OPT_PHY_BASE:
			if ( value )
			{
				extra_data->phy_base = strtoul( value, 0, 0 );
				chip_idea_slogf( dcctrl, _SLOG_INFO, "%s : phy_base = 0x%x", __func__, extra_data->phy_base);
			}
			break;
		case MX_OPT_VBUS_ENABLE:
			extra_data->vbus_enable = 1;
			break;
		case MX_OPT_PHY_IRQ:
			if ( value )
			{
				extra_data->phy_irq = strtoul( value, 0, 0 );
				chip_idea_slogf( dcctrl, _SLOG_INFO, "%s : phy_irq = %d", __func__, extra_data->phy_irq);
			}
			break;
		default:
			chip_idea_slogf( dcctrl, _SLOG_ERROR, "%s : unknown option - %s", __func__, value == NULL ? "NULL" : value);
			break;
	}
}

void inline mx6sabrelite_sync_flush( void ) 
{
	asm("dsb");
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION( "$URL: http://svn/product/branches/6.6.0/trunk/hardware/devu/dc/ci/mx6sabrelite/mx6sabrelite.c $ $Rev: 741231 $" )
#endif
