/*
 * $QNXLicenseC:
 * Copyright 2007, 2008, QNX Software Systems.
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

//#include <sys/mman.h>
#include <string.h>
//#include <arm/mx6x.h>
//#include <sys/slog.h>
//#include <sys/slogcodes.h>

#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <malloc.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/neutrino.h>
#include <termios.h>
#include <devctl.h>
#include <sys/dcmd_chr.h>
#include <sys/iomsg.h>
#include <atomic.h>
#include <hw/inout.h>
//#include <arm/mx1.h>
#include <sys/io-char.h>
#include <sys/hwinfo.h>
#include <drvr/hwinfo.h>
#include <pthread.h>
#include <sys/rsrcdbmgr.h>
#include <sys/dispatch.h>
#include <sys/slog.h>
#include <sys/slogcodes.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/resmgr.h>
#include <sys/neutrino.h>
#include <hw/inout.h>
#include <sys/slog.h>
#include <sys/slogcodes.h>
#include <errno.h>
//#include <arm/mx35.h>
#include <sys/procmgr.h>
#include <drvr/hwinfo.h>



#define MX6X_IOMUXC_BASE                        0x020E0000

int
main(int argc, char *argv[])
{
	size_t   len = 8192;
	uint32_t base = mmap_device_io(len, MX6X_IOMUXC_BASE);
	if (base == MAP_DEVICE_FAILED) {
		printf("Failed to map registers\n");
		return EXIT_FAILURE;
	}
	unsigned int reg1 = in32(base + 0x90);
	unsigned int reg2 = in32(base + 0x94);
	//unsigned int reg3 = in32(base + 0x96);
	//unsigned int reg4 = in32(base + 0x3a0);
	//out32(base + 0x90, 0x1);
	printf("weilun@adv debug EIM_D16 = 0x%x\n", reg1);
	printf("weilun@adv debug EIM_D17 = 0x%x\n", reg2);
	//printf("weilun@adv debug EIM_D18 = 0x%x\n", reg3);
	//printf("weilun@adv debug EIM_EB2 = 0x%x\n", reg4);
	munmap_device_io(base,len);
	return 0;
}

