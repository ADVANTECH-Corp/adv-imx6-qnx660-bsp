/*
 * $QNXLicenseC:
 * Copyright 2007, 2008, 2014 QNX Software Systems.
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
#ifdef __USAGE
%C - Serial driver for MC9328MX1/i.MX21/i.MX31/i.MX51/i.MX53/i.MX6 UARTs

%C [options] [port[,irq]] &
Options:
 -b number    Define initial baud rate (default 115200)
 -B number,number  First number defines baud rate for script downloading
                   Second number defines baud rate for streaming
                   If only one number defined, it serves both purposes
 -c clk       Set the input clock rate (default 96000000)
 -C number    Size of canonical input buffer (default 256)
 -e           Set options to "edit" mode
 -E           Set options to "raw" mode (default)
 -I number    Size of raw input buffer (default 2048)
 -f           Enable hardware flow control (default)
 -F           Disable hardware flow control
 -O number    Size of output buffer (default 2048)
 -s           Enable software flow control
 -S           Disable software flow control (default)
 -t number    Set receive FIFO trigger level ( 0 - 32; default 24)
 -T number    Set number of characters to send to transmit FIFO
                                             ( 2 - 32; default 2)
 -u unit      Set serial unit number (default 1)
 -m           MX1 type UART
 -d (evt_num) use dma (DMA is only supported on i.MX53 and i.MX6x SOCs).
              Note that on i.MX6x platform according to Freescale DMA should
			                 only be enabled if HW Flow Control is also enabled.
 -i (0|1)     Interrupt mode (0 = event, 1 = isr) (default = 1)
 -n           BT init script file name (*.bts or *.hcd)
 -r number    BT init retries, 0 = infinite (default 5)
 -g gpio_base,gpio_pin  GPIO base and GPIO pin used for Bluetooth Enable

Examples:

# Enable DMA and HW Flow Control for UART5 on i.MX6x - example for
# UART connection to BT chip.
devc-sermx1_hci-mx6x -d33 -E -f -S -c80000000 0x021f4000,62

#endif
*/
#include "externs.h"

/*
* Specify parameters for default devices from hwi_info tags.
*/
int query_hwi_device(TTYINIT_MX1 * dip, unsigned unit)
{
	unsigned hwi_off = hwi_find_device("uart", unit);
	if (hwi_off != HWI_NULL_OFF)
	{
		hwi_tag *tag_location = hwi_tag_find(hwi_off, HWI_TAG_NAME_location, 0);
		if (tag_location)
		{
			dip->tty.port = tag_location->location.base;
		}
		hwi_tag *tag_irq = hwi_tag_find(hwi_off, HWI_TAG_NAME_irq, 0);
		if (tag_irq)
		{
			dip->intr[0] = tag_irq->irq.vector;
		}
		return 1;
	}
	/*
	 * No default device, the base address and irq have been specified
	 */
	return 0;
}

unsigned options(int argc, char *argv[])
{
	int opt;
	int numports = 0;
	void *link;
	unsigned unit;
	unsigned rx_fifo = 24;		// default
	unsigned tx_fifo = 2;		// default
	TTYINIT_MX1 devinit = {
		{0,										 // port
		 0,										 // port_shift
		 0,										 // intr
		 DEFAULT_BAUD_RATE,						 // baud
		 2048,									 // isize
		 2048,									 // osize
		 256,									 // csize
		 0,										 // c_cflag
		 0,										 // c_iflag
		 0,										 // c_lflag
		 0,										 // c_oflag
		 0,										 // fifo
		 96000000,								 // pfclk
		 16,									 // div
		 "/dev/ser"								 // name
		 },
		5,										 // max_retry (BT init)
		0,										 // not MX1 type UART
		{-1, -1},								 // intr
		0,										 // usedma
		0,										 // rx_dma_evt
		0,										 // tx_dma_evt
		1,										 // isr
		{NULL},
		GPIO_BASE_ADDR,							// GPIO base address (used for bluetooth enable)
		GPIO_PIN,								// GPIO pin (used for bluetooth enable)
		DEFAULT_BAUD_RATE_MID,					// used for downloading script file
		DEFAULT_BAUD_RATE_HIGH					// used for streaming
	};

	int found_hwi_device = -1;

	/*
	 * Initialize the devinit to raw mode
	 */
	ttc(TTC_INIT_RAW, &devinit, 0);

	unit = 0;

	/* Getting the UART Base addresss and irq from the Hwinfo Section if available */
	{
		unsigned hwi_off = hwi_find_device("uart", 0);
		if (hwi_off != HWI_NULL_OFF)
		{
			hwi_tag *tag_inputclk = hwi_tag_find(hwi_off, HWI_TAG_NAME_inputclk, 0);
			if (tag_inputclk)
			{
				devinit.tty.clk = tag_inputclk->inputclk.clk;
			}
		}
	}

	while (optind < argc)
	{
		/*
		 * Process dash options.
		 * Options already used by io-char (do not use these!): b,e,E,f,F,s,S,C,I,O,o,v
		 */
		while ((opt = getopt(argc, argv, IO_CHAR_SERIAL_OPTIONS "t:T:c:u:md:i:n:g:r:B:")) != -1)
		{
			switch (ttc(TTC_SET_OPTION, &devinit, opt))
			{

				case 't':
					rx_fifo = strtoul(optarg, NULL, 0);
					if (rx_fifo > 32)
					{
						fprintf(stderr, "FIFO trigger must be <= 32.\n");
						fprintf(stderr, "Will disable FIFO.\n");
						rx_fifo = 0;
					}
					break;

				case 'T':
					tx_fifo = strtoul(optarg, NULL, 0);
					if ((tx_fifo > 32) || (tx_fifo < 2))
					{
						fprintf(stderr, "Tx fifo size must be >= 2 and <= 32.\n");
						fprintf(stderr, "Using tx fifo size of 32\n");
						tx_fifo = 2;
					}

					break;

				case 'c':
					devinit.tty.clk = strtoul(optarg, &optarg, 0);
					break;

				case 'u':
					unit = strtoul(optarg, NULL, 0);
					break;

				case 'm':
					devinit.mx1 = 1;
					break;

				case 'd':
#if USE_DMA
					devinit.usedma = 1;
					devinit.rx_dma_evt = strtoul(optarg, NULL, 0);
					devinit.tx_dma_evt = devinit.rx_dma_evt + 1;
#else
					fprintf(stderr, "DMA Not supported\n");
#endif
					break;
				case 'i':
					devinit.isr = strtoul(optarg, NULL, 0);
					break;
				case 'n':
					strncpy(devinit.bt_script_name, optarg, _POSIX_PATH_MAX);
					break;
				case 'r':
					devinit.max_retry = strtoul(optarg, NULL, 0);
					break;
				case 'g':
					devinit.gpio_base = strtoul(optarg, &optarg, 0);
					if (*optarg == ',')
						devinit.gpio_num = strtoul(optarg + 1, &optarg, 0);
					break;
				case 'B':
					devinit.baud_mid = strtoul(optarg, &optarg, 0);
					if (*optarg == ',')
						devinit.baud_high = strtoul(optarg + 1, &optarg, 0);
					else
						devinit.baud_high = devinit.baud_mid;
					break;
			}
		}

		devinit.tty.fifo = rx_fifo | (tx_fifo << 10);

		/*
		 * Process ports and interrupts.
		 */
		while (optind < argc && *(optarg = argv[optind]) != '-')
		{
			devinit.tty.port = strtoul(optarg, &optarg, 16);
			if (*optarg == ',')
			{
				devinit.intr[0] = strtoul(optarg + 1, &optarg, 0);
				if (*optarg == ',')
					devinit.intr[1] = strtoul(optarg + 1, &optarg, 0);
			}

			if (devinit.tty.port != 0 && devinit.intr[0] != -1)
			{
				create_device(&devinit, unit++);
				++numports;
			}
			++optind;
		}
	}

	if (numports == 0)
	{
		unit = 0;
		link = NULL;
		devinit.tty.fifo = rx_fifo | (tx_fifo << 10);
		while (1)
		{
			found_hwi_device = query_hwi_device(&devinit, unit);
			if (!found_hwi_device)
				break;
			create_device(&devinit, unit++);
			++numports;
		}
		while (1)
		{
			link = query_default_device(&devinit, link);
			if (link == NULL)
				break;
			create_device(&devinit, unit++);
			++numports;
		}
	}

	return numports;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/devc/sermx1_hci/options.c $ $Rev: 757029 $")
#endif
