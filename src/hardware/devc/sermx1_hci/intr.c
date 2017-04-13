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

#define MX1_RXERR (MX1_URXD_ERR | MX1_URXD_OVERRUN | MX1_URXD_FRMERR | MX1_URXD_BRK | MX1_URXD_PRERR)

#if USE_DMA

/* This function is in charge of process a given block transferred by
 * the dma engine
 */
int process_rx_block(DEV_MX1 *dev)
{
	unsigned char *pBuf;
	int bytes_to_read = 0;
	static int idx = 0;

	/* set pointer to begining of DMA buffer */
	if(dev->rx_dma.buffer0)
		pBuf = dev->rx_dma.buf;
	else
		pBuf = dev->rx_dma.buf + dev->rx_dma.xfer_size;

	while (idx < dev->rx_dma.bytes_read)
	{
		if (dev->rx_active != -1 && dev->vdev[dev->rx_active].rx_length > 0)
		{
			bytes_to_read = min (dev->vdev[dev->rx_active].rx_length, dev->rx_dma.bytes_read - idx);
			bytes_to_read = min ( bytes_to_read, dev->vdev[dev->rx_active].tty.ibuf.size - dev->vdev[dev->rx_active].tty.ibuf.cnt );
			if (dev->vdev[dev->rx_active].tty.c_cflag & CREAD)
				dev->rx_dma.status |= tti2 ( &dev->vdev[dev->rx_active].tty, &pBuf[idx], bytes_to_read, dev->rx_dma.key);
			dev->vdev[dev->rx_active].rx_length -= bytes_to_read;
			idx += bytes_to_read;
			if(dev->vdev[dev->rx_active].rx_length <= 0 || (dev->flags & RX_ERROR))
			{
				if(dev->flags & SCRIPT_MODE || (dev->flags & RX_ERROR))
				{
					/* Set EVENT_CUSTOM to signal error (re-init BT chip at thread time) */
					atomic_set(&dev->vdev[dev->rx_active].tty.eflags, EVENT_CUSTOM);
				}
				dev->rx_active = -1;
			}
		}
		else
		{
			/* Discard zero length HCI packets */
			if (dev->rx_active != -1 && dev->vdev[dev->rx_active].rx_length == 0)
			{
				dev->rx_active = -1;
				dev->header_cnt = 0;
			}

			dev->header[dev->header_cnt++] = pBuf[idx++];

			if (dev->flags & RX_ERROR)
			{
				int rx_idx = (dev->rx_active != -1) ? dev->rx_active : 0;
				/* Set EVENT_CUSTOM to signal error (re-init BT chip at thread time) */
				atomic_set(&dev->vdev[rx_idx].tty.eflags, EVENT_CUSTOM);
			}
			else
				dev->rx_dma.status |= parse_rx_header(dev);
		}
	}
	if (idx >= dev->rx_dma.bytes_read)
		idx = 0;

	return EOK;
}

/*
 * TX Pulse Handler - Gets notified once TX DMA is done
 */
void mx53_tx_pulse_hdlr(DEV_MX1 *dev, struct sigevent *event)
{
    int    status = 0;
    int    tx_idx;

    tx_idx = (dev->tx_active != -1)? dev->tx_active:0;

    dev->sdmafuncs.xfer_complete(dev->tx_dma.dma_chn);
	dev->tx_xfer_active = FALSE;

    dev->vdev[0].tty.un.s.tx_tmr = 0;
    dev->vdev[1].tty.un.s.tx_tmr = 0;
    dev->vdev[2].tty.un.s.tx_tmr = 0;

    /* Send event to io-char, tto() will be processed at thread time */
    atomic_set(&dev->vdev[0].tty.flags, EVENT_TTO);
    status |= 1;

    if (status)
    {
        iochar_send_event (&dev->vdev[tx_idx].tty);
    }
}

/*
 * RX Pulse Handler - Get notified once RX DMA is complete
 */
void mx53_rx_pulse_hdlr(DEV_MX1 *dev, struct sigevent *event)
{
    uintptr_t    base = dev->base;
    int error = 0;
    uint32_t sr1, sr2;
    int rx_idx, i;

    dma_transfer_t tinfo;
    dma_addr_t dma_addr;

    dev->rx_dma.bytes_read = dev->sdmafuncs.bytes_left(dev->rx_dma.dma_chn);
    error = dev->sdmafuncs.xfer_complete(dev->rx_dma.dma_chn);
    dev->rx_dma.key = 0;
    dev->rx_dma.status = 0;

    rx_idx = (dev->rx_active != -1)? dev->rx_active : 0;
    if(error)
    {
        sr1 = in32(base + MX1_UART_SR1);
        sr2 = in32(base + MX1_UART_SR2);

        if(sr2 & MX1_USR2_BRCD)
            dev->rx_dma.key |= TTI_BREAK;
        else if(sr1 & MX1_USR1_FRAMERR)
            dev->rx_dma.key |= TTI_FRAME;
        else if(sr1 & MX1_USR1_PARITYERR)
            dev->rx_dma.key |= TTI_PARITY;
        else if(sr2 & MX1_USR2_ORE)
            dev->rx_dma.key |= TTI_OVERRUN;
        dev->flags = RX_ERROR;

        dev->vdev[rx_idx].tty.oband_data |= dev->rx_dma.key;
        atomic_set(&dev->vdev[rx_idx].tty.flags, OBAND_DATA);
    }

    if((dev->vdev[rx_idx].tty.ibuf.size - dev->vdev[rx_idx].tty.ibuf.cnt) < dev->rx_dma.bytes_read)
    {
        /* no enough spage in ibuf. Return!
         * This is okay because auto-cts is enabled and another transfer will
         * get scheduled once we receive the signal in tto.c
         */

        /* Set IHW_PAGED otherwise io-char flow control code will not call tto */
        atomic_set (&dev->vdev[rx_idx].tty.flags, IHW_PAGED);

        /* Force READ to make more room in io-char buffer */
        atomic_set (&dev->vdev[rx_idx].tty.flags, EVENT_READ);
        iochar_send_event (&dev->vdev[rx_idx].tty);
        return;
    }

    /* Process the block of data received by the DMA engine */
    process_rx_block(dev);
	/* Schedule the next transfer */
    memset(&tinfo, 0, sizeof(tinfo));
    tinfo.xfer_bytes = dma_addr.len = dev->rx_dma.xfer_size;
    tinfo.dst_addrs = &dma_addr;
    tinfo.src_addrs= NULL;
    tinfo.xfer_unit_size = 8;
    tinfo.dst_fragments = 1;

	if(dev->rx_dma.buffer0)
    {
		dma_addr.paddr = dev->rx_dma.phys_addr + dev->rx_dma.xfer_size;
    }
    else
    {
    	dma_addr.paddr = dev->rx_dma.phys_addr;
	}

	dev->rx_dma.bytes_read = 0; /* Reset bytes read to give us an indication that a transfer is scheduled */
    dev->rx_dma.buffer0 ^= 1;
	dev->sdmafuncs.setup_xfer(dev->rx_dma.dma_chn, &tinfo);
	dev->sdmafuncs.xfer_start(dev->rx_dma.dma_chn);

    for(i = 0; i < NUM_DEVS; i++)
        iochar_send_event (&dev->vdev[i].tty);
}
#endif

static inline int ms_interrupt(DEV_MX1 *dev)
{
    int status = 0;
    uintptr_t       base = dev->base;
    uint32_t sr1, sr2;
    int i;

    sr1 = in32(base + MX1_UART_SR1);
    sr2 = in32(base + MX1_UART_SR2);

    for(i = 0; i < NUM_DEVS; i++)
    {
        if(sr2 & (MX1_USR2_DCDDELT))
        {
            status |= tti(&dev->vdev[i].tty, (sr2 & (MX1_USR2_DCDIN)) ?  TTI_HANGUP:TTI_CARRIER);
        }

        if(dev->vdev[i].tty.c_cflag & OHFLOW)
            status |= tti(&dev->vdev[i].tty, (sr1 & MX1_USR1_RTSS) ? TTI_OHW_CONT : TTI_OHW_STOP);

        /* OBAND notification of Modem status change */
        dev->vdev[i].tty.oband_data |= _OBAND_SER_MS;
        atomic_set(&dev->vdev[i].tty.flags, OBAND_DATA);
        atomic_set(&dev->vdev[i].tty.flags, EVENT_NOTIFY_OBAND);
    }
    status |= 1;

    out32(base + MX1_UART_SR1, MX1_USR1_RTSD);
    out32(base + MX1_UART_SR2, (MX1_USR2_DCDDELT));

	return (status);
}

static inline int tx_interrupt(DEV_MX1 *dev)
{
    int    status = 0;
    uintptr_t        base = dev->base;
    int cr1;

    cr1 = in32(base + MX1_UART_CR1);
    out32(base + MX1_UART_CR1, cr1 & ~MX1_UCR1_TRDYEN);

    dev->vdev[0].tty.un.s.tx_tmr = 0;
    dev->vdev[1].tty.un.s.tx_tmr = 0;
    dev->vdev[2].tty.un.s.tx_tmr = 0;
    /* Send event to io-char, tto() will be processed at thread time */
    atomic_set(&dev->vdev[0].tty.flags, EVENT_TTO);
    status |= 1;

    return (status);
}

static inline int rx_interrupt(DEV_MX1 *dev)
{
    int            status = 0;
    unsigned       key, rxdata;
    uintptr_t      base = dev->base;
    int cnt;

    in32(base + MX1_UART_SR2);
    out32(base + MX1_UART_SR1, (1<<8));

    cnt = 0;

	/* limit loop iterations by FIFO size to prevent ISR from running too long */
    while (in32(base + MX1_UART_SR2) & MX1_USR2_RDR && cnt++ < (dev->fifo  & MX1_UFCR_RXTL_MASK))
    {
        /*
        * Read next character from FIFO
        */
        rxdata = in32(base + MX1_UART_RXDATA);
        key = rxdata & 0xFF;

        if(dev->rx_active != -1 && dev->vdev[dev->rx_active].rx_length > 0)
        {
            if (rxdata & MX1_RXERR)
            {
                /*
                 * Save error as out-of-band data which can be read via devctl()
                 */
                dev->vdev[dev->rx_active].tty.oband_data |= rxdata;
                atomic_set(&dev->vdev[dev->rx_active].tty.flags, OBAND_DATA);

                if (rxdata & MX1_URXD_BRK)
                    key |= TTI_BREAK;
                else if (rxdata & MX1_URXD_FRMERR)
                    key |= TTI_FRAME;
                else if (rxdata & MX1_URXD_PRERR)
                    key |= TTI_PARITY;
                else if (rxdata & MX1_URXD_OVERRUN)
                    key |= TTI_OVERRUN;
                dev->flags |= RX_ERROR;
            }
			if (dev->vdev[dev->rx_active].tty.c_cflag & CREAD)
            	status |= tti(&dev->vdev[dev->rx_active].tty, key);
            if(--dev->vdev[dev->rx_active].rx_length <= 0 || (dev->flags & RX_ERROR))
            {
                if(dev->flags & SCRIPT_MODE || (dev->flags & RX_ERROR))
                {
                    atomic_set(&dev->vdev[dev->rx_active].tty.eflags, EVENT_CUSTOM);
                }
                dev->rx_active = -1;
            }
        }
        else
        {
			/* Discard zero length HCI packets */
			if (dev->rx_active != -1 && dev->vdev[dev->rx_active].rx_length == 0)
			{
				dev->rx_active = -1;
				dev->header_cnt = 0;
			}

            int rx_idx = (dev->rx_active != -1) ? dev->rx_active : 0;
            dev->header[dev->header_cnt++] = key;
            if (rxdata & MX1_RXERR)
            {
                /*
                 * Save error as out-of-band data which can be read via devctl()
                 */
                dev->vdev[rx_idx].tty.oband_data |= rxdata;
                atomic_set(&dev->vdev[rx_idx].tty.flags, OBAND_DATA);

                if (rxdata & MX1_URXD_BRK)
                    key |= TTI_BREAK;
                else if (rxdata & MX1_URXD_FRMERR)
                    key |= TTI_FRAME;
                else if (rxdata & MX1_URXD_PRERR)
                    key |= TTI_PARITY;
                else if (rxdata & MX1_URXD_OVERRUN)
                    key |= TTI_OVERRUN;
                dev->flags |= RX_ERROR;
                /* Set EVENT_CUSTOM to signal error (re-init BT chip at thread time) */
                atomic_set(&dev->vdev[rx_idx].tty.eflags, EVENT_CUSTOM);
            }
            else
            {
                status |= parse_rx_header(dev);
            }
        }
    }

	/*
	 * Note that as soon the RX FIFO data level drops below the RXTL threshold
	 * the Receiver Ready (RRDY) interrupt will automatically clear
	 */

	/* Clear the ageing timer interrupt if it caused this interrupt */
	if (in32(base + MX1_UART_SR1) & MX1_USR1_AGTIM)
		out32(base + MX1_UART_SR1, MX1_USR1_AGTIM);

    return status;
}

static inline int do_interrupt(DEV_MX1 *dev, int id)
{
    int sts=0;

    if(!dev->usedma) // do not need to process tx and rx_interrupt in DMA mode
    {
        sts = rx_interrupt(dev);
		/* If interrupt is enabled and FIFO is ready */
        if ((in32(dev->base + MX1_UART_CR1) & MX1_UCR1_TRDYEN) && (in32(dev->base + MX1_UART_SR1) & MX1_USR1_TRDY))
            sts |= tx_interrupt(dev);
    }

    if (in32(dev->base + MX1_UART_SR1) & MX1_USR1_RTSD)
        sts |= ms_interrupt(dev);

    return sts;
}

/*
 * Serial interrupt handler
 */
static const struct sigevent * ser_intr(void *area, int id)
{
    int i;
    DEV_MX1    *dev = area;
    struct sigevent *event = NULL;

    if (do_interrupt(dev,id))
    {
        for(i = 0; i < NUM_DEVS; i++)
        {
            if((dev->vdev[i].tty.flags & EVENT_QUEUED) == 0)
            {
                event = &ttyctrl.event;
                dev_lock(&ttyctrl);
                ttyctrl.event_queue[ttyctrl.num_events++] = &dev->vdev[i].tty;
                atomic_set(&dev->vdev[i].tty.flags, EVENT_QUEUED);
                dev_unlock(&ttyctrl);
            }
        }
    }
    return event;
}

int
interrupt_event_handler (message_context_t * msgctp, int code, unsigned flags, void *handle)
{
    uint32_t status = EOK;
    DEV_MX1 *dev = (DEV_MX1 *) handle;
    int i;

    status = do_interrupt (dev, dev->iid[0]);

    if (status)
    {
        for(i = 0; i < NUM_DEVS; i++)
            iochar_send_event (&dev->vdev[i].tty);
    }

    InterruptUnmask (dev->intr[0], dev->iid[0]);
    return (EOK);
}

void
ser_attach_intr(DEV_MX1 *dev)
{
    struct sigevent event;
    if(dev->isr)
    {
        dev->iid[0] = InterruptAttach(dev->intr[0], ser_intr, dev, 0, _NTO_INTR_FLAGS_TRK_MSK);
        if (dev->intr[1] != -1)
            dev->iid[1] = InterruptAttach(dev->intr[1], ser_intr, dev, 0, _NTO_INTR_FLAGS_TRK_MSK);
    }
    else
    {
        // Associate a pulse which will call the event handler.
        if ((event.sigev_code =
                pulse_attach (ttyctrl.dpp, MSG_FLAG_ALLOC_PULSE, 0, &interrupt_event_handler,
                    dev)) == -1)
        {
            fprintf (stderr, "Unable to attach event pulse.%s\n", strerror(errno));
            return;
        }

        /* Init the pulse for interrupt event */
        event.sigev_notify = SIGEV_PULSE;
        event.sigev_coid = ttyctrl.coid;

		/*
		 * If the interrupt is being handled by an event, then set the event priority
		 * to the io-char event priority+1. The io-char event priority can be configured
		 * by the "-o priority=X" parameter.
		 */
		event.sigev_priority = ttyctrl.event.sigev_priority + 1;
        event.sigev_value.sival_int = 0;

        dev->iid[0] = InterruptAttachEvent (dev->intr[0], &event, _NTO_INTR_FLAGS_TRK_MSK);
        if (dev->iid[0] == -1)
            fprintf (stderr, "Unable to attach InterruptEvent. %s\n", strerror(errno));
    }

    /* Enable UART, Ageing DMA timer interrupt, Receive Ready DMA Interrupt Enable, Transmitter Ready DMA Enable */
    if(dev->usedma)
        out32(dev->base + MX1_UART_CR1, in32(dev->base + MX1_UART_CR1) | MX53_UART_UCR1_ATDMAEN | MX1_UCR1_UARTEN | MX1_UCR1_RDMAEN | MX1_UCR1_TDMAEN);
    else
    {
        /* Ensure that DMA interrupts are all disabled */
        out32(dev->base + MX1_UART_CR1, in32(dev->base + MX1_UART_CR1) & ~(MX53_UART_UCR1_ATDMAEN
                | MX1_UCR1_RDMAEN | MX1_UCR1_TDMAEN));

        /* Enable aging timer interrupt */
        out32(dev->base + MX1_UART_CR2, in32(dev->base + MX1_UART_CR2) | MX1_UCR2_ATEN);
    }
}

#if 0
void
ser_detach_intr(DEV_MX1 *dev)
{
    uintptr_t    base = dev->base;

    /* Disable UART */
    out32(base + MX1_UART_CR1, 0x04);
    out32(base + MX1_UART_CR2, 0x00);

    InterruptDetach(dev->iid[0]);
    dev->intr[0] = -1;
    if (dev->intr[1] != -1) {
        InterruptDetach(dev->iid[1]);
        dev->intr[1] = -1;
    }

}
#endif


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/devc/sermx1_hci/intr.c $ $Rev: 761666 $")
#endif
