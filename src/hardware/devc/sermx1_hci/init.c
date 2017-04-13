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

#include "externs.h"
#include <sys/mman.h>
#include <string.h>

/*
 * Specify parameters for default devices.
 */
void *query_default_device(TTYINIT_MX1 * dip, void *link)
{
	/*
	 * No default device, the base address and irq have be be specified
	 */
	return NULL;
}

void destroy_virtual_devices(DEV_MX1 * dev)
{
	int cnt;

	for (cnt = 0; cnt < NUM_DEVS; cnt++)
	{
		if (dev->vdev[cnt].tty.ibuf.buff != NULL)
		{
			free(dev->vdev[cnt].tty.ibuf.buff);
			dev->vdev[cnt].tty.ibuf.buff = NULL;
		}
		if (dev->vdev[cnt].tty.obuf.buff != NULL)
		{
			free(dev->vdev[cnt].tty.obuf.buff);
			dev->vdev[cnt].tty.obuf.buff = NULL;
		}
		if (dev->vdev[cnt].tty.cbuf.buff != NULL)
		{
			free(dev->vdev[cnt].tty.cbuf.buff);
			dev->vdev[cnt].tty.cbuf.buff = NULL;
		}
	}
}

int mx1_devctl(resmgr_context_t * ctp, io_devctl_t * msg, iofunc_ocb_t * ocb)
{
	virtual_dev_t *vdev = (virtual_dev_t *) ocb->attr;
	DEV_MX1 *dev = vdev->dev;
	int status = EOK;
	union
	{
		unsigned int udata32;
	} *dcp = _DEVCTL_DATA (msg->i);

	switch (msg->i.dcmd)
	{
		case DCMD_CHR_RESET:
			run_init_script(dev);
			break;
		case DCMD_CHR_GETOBAND_EXTENDED:
			dcp->udata32 = vdev->oband_data;
			vdev->oband_data = 0;	/* Clear data */
			break;
		default:
			status = EINVAL;
			break;
	}

	return (status);
}

void create_device(TTYINIT_MX1 * dip, unsigned unit)
{
	DEV_MX1 *dev;
	unsigned i, cnt;
	uint32_t sr1, sr2;
	/*
	 * Get a device entry and the input/output buffers for it.
	 */
	dev = calloc(1, sizeof (*dev));

	if (dev == NULL)
	{
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "MX1 UART: Unable to allocate device entry - %s", strerror(errno));
		exit(1);
	}

	//copy the max retry value
	dev->max_retry = dip->max_retry;

	/* GPIO base and number used for BT enable */
	dev->gpio_base = dip->gpio_base;
	dev->gpio_num = dip->gpio_num;

	dev->baud_mid = dip->baud_mid;
	dev->baud_high = dip->baud_high;

	if ((dev->gpio =
		mmap_device_memory(0, sizeof (struct _gpio), PROT_READ | PROT_WRITE | PROT_NOCACHE, 0,
							dev->gpio_base)) == MAP_FAILED)
	{
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "%s: Unable to map GPIO registers - %s", __FUNCTION__,
				strerror(errno));
		free(dev);
		return;
	}

	if (dip->usedma && dip->tty.isize < (DMA_XFER_SIZE * 2))
	{
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "MX1 UART: Invalid input buffer size");
		dip->tty.isize = DMA_XFER_SIZE * 2;
	}

	/* Create virtual devices */
	for (cnt = 0; cnt < NUM_DEVS; cnt++)
	{
		// Get buffers.
		dev->vdev[cnt].tty.ibuf.head = dev->vdev[cnt].tty.ibuf.tail = dev->vdev[cnt].tty.ibuf.buff =
			malloc(dev->vdev[cnt].tty.ibuf.size = dip->tty.isize);
		if (dev->vdev[cnt].tty.ibuf.buff == NULL)
		{
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "io-char: Allocation of input buffer failed (%d)", errno);
			destroy_virtual_devices(dev);
			munmap_device_memory(dev->gpio, sizeof (struct _gpio));
			free(dev);
			return;
		}

		dev->vdev[cnt].tty.obuf.head = dev->vdev[cnt].tty.obuf.tail = dev->vdev[cnt].tty.obuf.buff =
			malloc(dev->vdev[cnt].tty.obuf.size = dip->tty.osize);
		if (dev->vdev[cnt].tty.obuf.buff == NULL)
		{
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "io-char: Allocation of output buffer failed (%d)",
				  errno);
			destroy_virtual_devices(dev);
			munmap_device_memory(dev->gpio, sizeof (struct _gpio));
			free(dev);
			return;
		}

		dev->vdev[cnt].tty.cbuf.head = dev->vdev[cnt].tty.cbuf.tail = dev->vdev[cnt].tty.cbuf.buff =
			malloc(dev->vdev[cnt].tty.cbuf.size = dip->tty.csize);
		if (dev->vdev[cnt].tty.cbuf.buff == NULL)
		{
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "io-char: Allocation of canonical buffer failed (%d)",
				  errno);
			destroy_virtual_devices(dev);
			munmap_device_memory(dev->gpio, sizeof (struct _gpio));
			free(dev);
			return;
		}

		if (dip->usedma)
			dev->vdev[cnt].tty.highwater = dev->vdev[cnt].tty.ibuf.size + 1;    // when DMA is enabled never reach the RX FIFO highwater mark.
		else
			dev->vdev[cnt].tty.highwater = dev->vdev[cnt].tty.ibuf.size - 64;	//(dev->vdev[cnt].tty.ibuf.size < 128 ? dev->vdev[cnt].tty.ibuf.size/4 : 100);

		switch (cnt)
		{
			case 0:
				strcpy(dev->vdev[cnt].tty.name, "/dev/serbt");
				break;
			case 1:
				strcpy(dev->vdev[cnt].tty.name, "/dev/serfm");
				break;
			case 2:
				strcpy(dev->vdev[cnt].tty.name, "/dev/sergps");
				break;
			default:
				break;
		}
		dev->vdev[cnt].tty.baud = dip->tty.baud;

		/*
		 * The i.MX SOCs don't technically require the LOSES_TX_INTR flag,
		 * but the timer mechanism acts as a failsafe in case we ever miss a TX interrupt.
		 */
		dev->vdev[cnt].tty.flags = EDIT_INSERT | LOSES_TX_INTR;
		dev->vdev[cnt].tty.c_cflag = dip->tty.c_cflag;
		dev->vdev[cnt].tty.c_iflag = dip->tty.c_iflag;
		dev->vdev[cnt].tty.c_lflag = dip->tty.c_lflag;
		dev->vdev[cnt].tty.c_oflag = dip->tty.c_oflag;
		dev->vdev[cnt].tty.verbose = dip->tty.verbose;
		dev->vdev[cnt].tty.fifo = dip->tty.fifo;

		/*
		 * Currently io-char has a limitation that the TX timeout is hard coded to 150ms.
		 * At low baud rates the timer could potentially expire before the DMA transfer
		 * naturally completes. So when DMA is enabled we disable the LOSES_TX_INTR flag
		 * and let the driver specify the timeout value in tto().
		 */
#if USE_DMA
		if(dev->usedma)
			dev->vdev[cnt].tty.flags &= ~LOSES_TX_INTR;
#endif

		/*
		 * Initialize termios cc codes to an ANSI terminal.
		 */
		ttc(TTC_INIT_CC, &dev->vdev[cnt].tty, 0);

		/* The driver defaults the CREAD terminal flag to be off so un-used virtual ports
		 * will not accumulate data and trigger flow control halt.
		 * Client application must explicitly set the CREAD terminal flag if the want
		 * to use the virtual device.
		 */
		dev->vdev[cnt].tty.c_cflag &= ~(CREAD);

		/*
		 * Initialize the device's name.
		 * Assume that the basename is set in device name.  This will attach
		 * to the path assigned by the unit number/minor number combination
		 */
		unit = SET_NAME_NUMBER(unit) | NUMBER_DEV_FROM_USER;
		ttc(TTC_INIT_TTYNAME, &dev->vdev[cnt].tty, unit);

		dev->vdev[cnt].dev = dev;
	}

	dev->fifo = dip->tty.fifo;
	dev->intr[0] = dip->intr[0];
	dev->intr[1] = dip->intr[1];
	dev->clk = dip->tty.clk;
	dev->div = dip->tty.div;
	dev->mx1 = dip->mx1;

	dev->usedma = dip->usedma;
	dev->rx_dma_evt = dip->rx_dma_evt;
	dev->tx_dma_evt = dip->tx_dma_evt;
	dev->isr = dip->isr;
	dev->rx_dma.buffer0 = 1;
	dev->rx_dma.status = 0;
	dev->rx_dma.bytes_read = 0;
	dev->rx_dma.key = 0;
	dev->rx_active = -1;
	dev->tx_active = -1;

	/*
	 * Map device registers
	 */
	dev->base = mmap_device_io(MX1_UART_SIZE, dip->tty.port);
	if (dev->base == (uintptr_t) MAP_FAILED)
	{
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "MX1 UART: Failed to mmap device - %s", strerror(errno));
		goto fail1;
	}
#if USE_DMA
	if (dev->usedma)
	{
		char str[250];
		int watermark;
		unsigned channel;

		dev->tx_dma.xfer_size = DMA_XFER_SIZE;
		if ((dev->tx_dma.buf =
			 mmap(NULL, dev->tx_dma.xfer_size, PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_ANON | MAP_PHYS, NOFD,
				  0)) == MAP_FAILED)
		{
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "MX1 UART: Failed allocate DMA memory - %s", strerror(errno));
			goto fail2;
		}

		mem_offset64(dev->tx_dma.buf, NOFD, 1, &dev->tx_dma.phys_addr, 0);
		msync(dev->tx_dma.buf, dev->tx_dma.xfer_size, MS_INVALIDATE);

		/* Allocte 2x the transfer size for ping-pong buffer (only required for recv size) */
		dev->rx_dma.xfer_size = DMA_XFER_SIZE;
		if ((dev->rx_dma.buf =
			 mmap(NULL, dev->rx_dma.xfer_size * 2, PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_ANON | MAP_PHYS, NOFD,
				  0)) == MAP_FAILED)
		{
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "MX1 UART: Failed allocate DMA memory - %s", strerror(errno));
			goto fail3;
		}

		mem_offset64(dev->rx_dma.buf, NOFD, 1, &dev->rx_dma.phys_addr, 0);
		msync(dev->rx_dma.buf, dev->rx_dma.xfer_size * 2, MS_INVALIDATE);

		my_attach_pulse(&dev->rx_dma.pulse, &dev->rx_dma.sdma_event, mx53_rx_pulse_hdlr, dev);
		my_attach_pulse(&dev->tx_dma.pulse, &dev->tx_dma.sdma_event, mx53_tx_pulse_hdlr, dev);

		if (get_dmafuncs(&dev->sdmafuncs, sizeof (dma_functions_t)) == -1)
		{
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "MX1 UART: Failed to get DMA lib functions - %s", strerror(errno));
			goto fail4;
		}

		if (dev->sdmafuncs.init(NULL) == -1)
		{
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "MX1 UART: Failed to init DMA - %s", strerror(errno));
			goto fail4;
		}

		// water-mark is set to 1 less than fifo threshold.
		sprintf(str, "eventnum=%d,watermark=%d,fifopaddr=0x%llx", dev->rx_dma_evt, ((dev->fifo & MX1_UFCR_RXTL_MASK) - 1),
				dip->tty.port + 0x0);

#if defined(VARIANT_mx53)
		// For i.MX53 we only support shared UART modules so the UARTSH_2_MCU script is used.
		channel = 7;    // SDMA_CHTYPE_UARTSH_2_MCU
#else
		// For i.MX6x Freescale recommends using the UART_2_MCU SDMA script for all UART modules (shared UARTs and ARM Platform UARTs)
		channel = 8;    // SDMA_CHTYPE_UART_2_MCU
#endif

		if ((dev->rx_dma.dma_chn = dev->sdmafuncs.channel_attach(str, &dev->rx_dma.sdma_event, &channel,
																 DMA_ATTACH_PRIORITY_HIGHEST,
																 DMA_ATTACH_EVENT_PER_SEGMENT)) == NULL)
		{
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "MX1 UART: RX DMA attach failed - %s", strerror(errno));
			goto fail5;
		}

		/* DMA watermark is in free space, but fifo threadhold is in bytes used */
		watermark = 32 - ((dev->fifo >> 10) & MX1_UFCR_RXTL_MASK);
		if (watermark == 0)						 /* watermark of 0 is invalid */
			watermark = 1;
		sprintf(str, "eventnum=%d,watermark=%d,fifopaddr=0x%llx\n", dev->tx_dma_evt, watermark,
				dip->tty.port + MX1_UART_TXDATA);

#if defined(VARIANT_mx53)
        channel = 3;    // SDMA_CHTYPE_MCU_2_SHP
#else
		channel = 1;    // SDMA_CHTYPE_MCU_2_AP
#endif

		if ((dev->tx_dma.dma_chn = dev->sdmafuncs.channel_attach(str, &dev->tx_dma.sdma_event, (unsigned *) &channel,
																 DMA_ATTACH_PRIORITY_HIGHEST,
																 DMA_ATTACH_EVENT_PER_SEGMENT)) == NULL)
		{
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "MX1 UART: TX DMA attach failed - %s", strerror(errno));
			goto fail6;
		}

		dev->tx_xfer_active = FALSE;
	}
#endif

	/* Clear UART configuration, which won't be cleared by resetting UART device */
	out32(dev->base + MX1_UART_CR1, 0);
	out32(dev->base + MX1_UART_CR2, 0);

	/* Assert DSR/DTR */
	out32(dev->base + MX1_UART_CR3, in32(dev->base + MX1_UART_CR3) | MX1_UCR3_DSR);

	/*
	 * Only setup IRQ handler for non-pcmcia devices.
	 * Pcmcia devices will have this done later when card is inserted.
	 */
	if (dip->tty.port != 0 && dev->intr[0] != -1)
	{
		ser_stty(&dev->vdev[0]);
		ser_attach_intr(dev);
	}

	sr1 = in32(dev->base + MX1_UART_SR1);
	sr2 = in32(dev->base + MX1_UART_SR2);
	for (i = 0; i < NUM_DEVS; i++)
	{
		if (sr2 & (1 << 6))
			tti(&dev->vdev[i].tty, (sr2 & (1 << 5)) ? TTI_HANGUP : TTI_CARRIER);

		if (sr1 & MX1_USR1_RTSD && dev->vdev[i].tty.c_cflag & OHFLOW)
			tti(&dev->vdev[i].tty, (sr1 & MX1_USR1_RTSS) ? TTI_OHW_CONT : TTI_OHW_STOP);
	}

   /* Enable UART */
   out32(dev->base + MX1_UART_CR1, in32(dev->base + MX1_UART_CR1) | MX1_UCR1_UARTEN);

	/*
	 * Attach the resource manager
	 */
	for (cnt = 0; cnt < NUM_DEVS; cnt++)
	{
		dev->vdev[cnt].tty.io_devctlext = mx1_devctl;
		dev->vdev[cnt].tty.custom_event_handler = mx1_custom_event_handler;
		ttc(TTC_INIT_ATTACH, &dev->vdev[cnt].tty, 0);
	}

#if USE_DMA
	if (dev->usedma)
	{
		dma_transfer_t tinfo;
		dma_addr_t dma_addr;

		// Schedule an RX Transfer (upto MAX DMA SIZE)
		tinfo.xfer_bytes = dma_addr.len = dev->rx_dma.xfer_size;
		dma_addr.paddr = dev->rx_dma.phys_addr;
		tinfo.dst_addrs = &dma_addr;
		tinfo.src_addrs = NULL;
		tinfo.xfer_unit_size = 8;
		tinfo.dst_fragments = 1;

		dev->sdmafuncs.setup_xfer(dev->rx_dma.dma_chn, &tinfo);
		dev->sdmafuncs.xfer_start(dev->rx_dma.dma_chn);
        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "sermx1_hci: DMA is enabled for device %s", dev->vdev[0].tty.name);
    }
    else
    {
        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "sermx1_hci: DMA is disabled for device %s", dev->vdev[0].tty.name);
    }
#endif
	strncpy(dev->bt_script_name, dip->bt_script_name, _POSIX_PATH_MAX);
	run_init_script(dev);
	return;
#if USE_DMA
  fail6:
	dev->sdmafuncs.channel_release(dev->rx_dma.dma_chn);
  fail5:
	dev->sdmafuncs.fini();
  fail4:
	my_detach_pulse(&dev->rx_dma.pulse);
	my_detach_pulse(&dev->tx_dma.pulse);
	munmap(dev->rx_dma.buf, dev->rx_dma.xfer_size);
  fail3:
	munmap(dev->tx_dma.buf, dev->tx_dma.xfer_size);
  fail2:
	munmap_device_io(dev->base, MX1_UART_SIZE);
#endif
  fail1:
	for (cnt = 0; cnt < NUM_DEVS; cnt++)
	{
		free(dev->vdev[cnt].tty.obuf.buff);
		free(dev->vdev[cnt].tty.ibuf.buff);
		free(dev->vdev[cnt].tty.cbuf.buff);
	}
	free(dev);
	exit(1);
}

void dinit()
{
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/devc/sermx1_hci/init.c $ $Rev: 758023 $")
#endif
