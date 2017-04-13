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
	Descriptor information for USB MSC device (Mass Storage Class)
*/

#ifndef _USB_MSC_MX6SABRELITE_DESCRIPTORS_H_INCLUDED
#define _USB_MSC_MX6SABRELITE_DESCRIPTORS_H_INCLUDED

#define USBDC_DLL_NAME "devu-usbumass-mx6sabrelite-ci.so"

/* Override Vendor/Device ID 			*/

/* Override String Descriptor Table 	*/

/* Include Base Descriptor Information 	*/
#include <hw/usbdc_desc_msc.h>


#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION( "$URL: http://svn/product/branches/6.6.0/trunk/hardware/devu/dc/ci/mx6sabrelite/usbumass/descriptors.h $ $Rev: 735296 $" )
#endif
