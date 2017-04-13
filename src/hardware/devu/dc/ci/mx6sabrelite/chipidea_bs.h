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

#define CHIP_IDEA_DEFAULT_CTRL_NAME   "mx6sabrelite"

extern int mx6sabrelite_board_specfic_init( chip_ideadc *dcctrl );
extern int mx6sabrelite_otg_init( chip_ideadc *dcctrl );
extern int mx6sabrelite_otg_fini( chip_ideadc *dcctrl );
extern void mx6sabrelite_extra_alloc_callout(usbdc_device_t *udc);
extern void mx6sabrelite_extra_free_callout(usbdc_device_t *udc);
extern void mx6sabrelite_extra_process_args_callout(chip_ideadc *dcctrl, char *options);
extern void mx6sabrelite_board_specific_link_up( chip_ideadc *dcctrl );
extern void mx6sabrelite_board_specific_link_down( chip_ideadc *dcctrl);
extern void inline mx6sabrelite_sync_flush( void );

#define CHIP_IDEA_BOARD_SPECIFIC_INIT                mx6sabrelite_board_specfic_init( dcctrl );
#define CHIP_IDEA_BOARD_SPECIFIC_FINI                mx6sabrelite_otg_fini( dcctrl );
#define CHIP_IDEA_BOARD_SPECIFIC_TRANSCEIVER_INIT    mx6sabrelite_otg_init( dcctrl );
#define CHIP_IDEA_EXTRA_ALLOC_CALLOUT                mx6sabrelite_extra_alloc_callout( udc );
#define CHIP_IDEA_EXTRA_FREE_CALLOUT                 mx6sabrelite_extra_free_callout( udc );
#define CHIP_IDEA_EXTRA_PROCESS_ARGS_CALLOUT         mx6sabrelite_extra_process_args_callout(dcctrl, value);
#define CHIP_IDEA_BOARD_SPECIFIC_LINK_UP             mx6sabrelite_board_specific_link_up( dcctrl );
#define CHIP_IDEA_BOARD_SPECIFIC_LINK_DOWN           mx6sabrelite_board_specific_link_down( dcctrl );
#define CHIP_IDEA_BOARD_SPECIFIC_SYNC_FLUSH          mx6sabrelite_sync_flush();

#define CHIPIDEA_SET_MODE() (out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBMODE, (in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBMODE ) & ~0x3) | USB_MODE_CTRL_MODE_DEVICE | USB_MODE_SETUP_LOCK_OFF | USB_MODE_STREAM_DISABLE))

#define USB_CMD_ATDTW					(1<<14)

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION( "$URL: http://svn/product/branches/6.6.0/trunk/hardware/devu/dc/ci/mx6sabrelite/chipidea_bs.h $ $Rev: 741231 $" )
#endif
