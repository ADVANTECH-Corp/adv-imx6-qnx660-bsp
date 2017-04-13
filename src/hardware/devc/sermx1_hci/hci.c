/*
 * $QNXLicenseC:
 * Copyright 2007, 2014, QNX Software Systems.
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

#define SCRIPT_TYPE_BTS                 1
#define SCRIPT_TYPE_HCD                 2

/* Definitions for BTS file parsing */
#define FILE_HEADER_MAGIC               0x42535442
#define MAX_ACTION_SIZE                 300

#define ACTION_SEND_COMMAND             1		 // Send out raw data (as is)
#define ACTION_WAIT_EVENT               2		 // Wait for data
#define ACTION_SERIAL_PORT_PARAMETERS   3
#define ACTION_DELAY                    4
#define ACTION_RUN_SCRIPT               5
#define ACTION_REMARKS                  6

int mx1_custom_event_handler(TTYDEV * tty)
{
	virtual_dev_t *vdev = (virtual_dev_t *) tty;
	DEV_MX1 *dev = vdev->dev;

	if (dev->flags & SCRIPT_MODE)
	{
		pthread_sleepon_lock();
		if (tty->ibuf.cnt > 0 || (dev->flags & RX_ERROR))
		{
			dev->response_pending = 1;
			pthread_sleepon_signal(&tty->ibuf);
		}
		pthread_sleepon_unlock();
	}
	else
	{
		/* Re-init BT Chip */
		run_init_script(dev);
	}
	return EOK;
}

int parse_tx_packet(virtual_dev_t * vdev)
{
	if (vdev->tty.obuf.cnt > 0)
	{
		int packet_type, room;
		uint16_t length;
		TTYBUF *bup = &vdev->tty.obuf;

		bup = &vdev->tty.obuf;

		packet_type = bup->tail[0];
		switch (packet_type)
		{
			case 1:							 /* BT Channel -> Host to send HCI commands to chip */
				if (vdev->tty.obuf.cnt >= 4)
				{
					room = &bup->buff[bup->size] - bup->tail;
					if (room >= 4)
						vdev->tx_length = bup->tail[3];
					else
						vdev->tx_length = bup->buff[3 - room];
					vdev->tx_length += 4;
				}
				break;
			case 2:							 /* BT Channel -> Host and chip to exchange ACL data packets */
				if (vdev->tty.obuf.cnt >= 5)
				{
					room = &bup->buff[bup->size] - bup->tail;
					/* LSB First */
					if (room >= 4)
						length = bup->tail[3];
					else
						length = bup->buff[3 - room];
					if (room >= 5)
						length |= bup->tail[4] << 8;
					else
						length |= bup->buff[4 - room] << 8;
					vdev->tx_length = ENDIAN_LE16(length);
					vdev->tx_length += 5;
				}
				break;
			case 3:							 /* BT Channel -> Host and chip to exchange SCO data packets */
				if (vdev->tty.obuf.cnt >= 4)
				{
					room = &bup->buff[bup->size] - bup->tail;
					if (room >= 4)
						vdev->tx_length = bup->tail[3];
					else
						vdev->tx_length = bup->buff[3 - room];
					vdev->tx_length += 4;
				}
				break;
			case 4:							 /* BT Channel -> Chip to send HCI event packets to host */
				if (vdev->tty.obuf.cnt >= 3)
				{
					room = &bup->buff[bup->size] - bup->tail;
					if (room >= 3)
						vdev->tx_length = bup->tail[2];
					else
						vdev->tx_length = bup->buff[2 - room];
					vdev->tx_length += 3;
				}
				break;
			case 8:							 /* FM Channel */
				if (vdev->tty.obuf.cnt >= 2)
				{
					room = &bup->buff[bup->size] - bup->tail;
					if (room >= 2)
						vdev->tx_length = bup->tail[1];
					else
						vdev->tx_length = bup->buff[0];
					vdev->tx_length += 2;
				}
				break;
			case 9:							 /* GPS Channel */
				if (vdev->tty.obuf.cnt >= 4)
				{
					room = &bup->buff[bup->size] - bup->tail;
					/* LSB First */
					if (room >= 3)
						length = bup->tail[2];
					else
						length = bup->buff[2 - room];
					if (room >= 4)
						length |= bup->tail[3] << 8;
					else
						length |= bup->buff[3 - room] << 8;
					vdev->tx_length = ENDIAN_LE16(length);
					vdev->tx_length += 4;
				}
				break;
			default:
				slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "%s: Unknown Packet type %c", __FUNCTION__,
					  *(vdev->tty.obuf.tail));
				break;
		}
	}
	return (vdev->tx_length);
}

int parse_rx_header(DEV_MX1 * dev)
{
	int status = EOK;

	switch (dev->header[0])
	{
		case 0x1:								 /* BT Channel -> Host to send HCI commands to chip */
			if (dev->header_cnt == 4)
			{
				dev->rx_active = 0;
				dev->vdev[dev->rx_active].rx_length = dev->header[dev->header_cnt - 1];
				if (dev->vdev[dev->rx_active].tty.c_cflag & CREAD)
					tti2(&dev->vdev[dev->rx_active].tty, dev->header, dev->header_cnt, 0);
				dev->header_cnt = 0;
			}
			break;
		case 0x2:								 /* BT Channel -> Host and chip to exchange ACL data packets */
			if (dev->header_cnt == 5)
			{
				uint16_t length;
				dev->rx_active = 0;
				length = dev->header[3];
				length |= dev->header[4] << 8;
				dev->vdev[dev->rx_active].rx_length = ENDIAN_LE16(length);
				if (dev->vdev[dev->rx_active].tty.c_cflag & CREAD)
					tti2(&dev->vdev[dev->rx_active].tty, dev->header, dev->header_cnt, 0);
				dev->header_cnt = 0;
			}
			break;
		case 0x3:								 /* BT Channel -> Host and chip to exchange SCO data packets */
			if (dev->header_cnt == 4)
			{
				dev->rx_active = 0;
				dev->vdev[dev->rx_active].rx_length = dev->header[dev->header_cnt - 1];
				if (dev->vdev[dev->rx_active].tty.c_cflag & CREAD)
					tti2(&dev->vdev[dev->rx_active].tty, dev->header, dev->header_cnt, 0);
				dev->header_cnt = 0;
			}
			break;
		case 0x4:								 /* BT Channel -> Chip to send HCI event packets to host */
			if (dev->header_cnt == 3)
			{
				dev->rx_active = 0;
				dev->vdev[dev->rx_active].rx_length = dev->header[dev->header_cnt - 1];
				if (dev->vdev[dev->rx_active].tty.c_cflag & CREAD)
					tti2(&dev->vdev[dev->rx_active].tty, dev->header, dev->header_cnt, 0);
				dev->header_cnt = 0;
			}
			break;
		case 0x8:								 /* FM Channel */
			if (dev->header_cnt == 2)
			{
				dev->rx_active = 1;
				dev->vdev[dev->rx_active].rx_length = dev->header[dev->header_cnt - 1];
				if (dev->vdev[dev->rx_active].tty.c_cflag & CREAD)
					tti2(&dev->vdev[dev->rx_active].tty, dev->header, dev->header_cnt, 0);
				dev->header_cnt = 0;
			}
			break;
		case 0x9:								 /* GPS Channel */
			if (dev->header_cnt == 4)
			{
				uint16_t length;
				dev->rx_active = 2;
				length = dev->header[2];
				length |= dev->header[3] << 8;
				dev->vdev[dev->rx_active].rx_length = ENDIAN_LE16(length);
				if (dev->vdev[dev->rx_active].tty.c_cflag & CREAD)
					tti2(&dev->vdev[dev->rx_active].tty, dev->header, dev->header_cnt, 0);
				dev->header_cnt = 0;
			}
			break;
			/* HCILL Channels */
		case 0x1E:								 /* HCILL Channel -> Chip to send sleep indication to host */
			dev->header_cnt = 0;
			break;
		case 0x20:								 /* HCILL Channel -> Host to send sleep acknowledge to chip */
			dev->header_cnt = 0;
			break;
		case 0x21:								 /* HCILL Channel -> Host/Chip to send wakeup indication to chip/host */
			dev->header_cnt = 0;
			break;
		case 0x22:								 /* HCILL Channel -> Host/Chip to send wakeup acknowledgement to chip/host */
			dev->header_cnt = 0;
			break;
		default:
			dev->header_cnt = 0;
			break;
	}
	return (status);
}

FILE *open_script_file(const char *filename, int *script_type, unsigned long *version)
{
	FILE *fptr;
	char *extension = NULL;
	struct tagCHeader
	{
		unsigned long m_nMagicNumber;
		unsigned long m_nVersion;
		unsigned char m_nFuture[24];
	} header;

	if ((fptr = fopen(filename, "rb")) == NULL)
	{
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "Unable to open script file %s - %s)", filename,
			  strerror(errno));
		return (NULL);
	}

	/* Check the file extension to see if we're using a TI BTS file or a Broadcom HCD file */
	extension = (char*)filename + (strlen(filename) - 3);
	if (stricmp(extension, "bts") == 0)
		*script_type = SCRIPT_TYPE_BTS;
	else if (stricmp(extension, "hcd") == 0)
		*script_type = SCRIPT_TYPE_HCD;

	switch (*script_type)
	{
		case SCRIPT_TYPE_BTS:
			if (fread(&header, sizeof (header), 1, fptr) == 1)
			{
				if (header.m_nMagicNumber == FILE_HEADER_MAGIC)
				{
					if (version != NULL)
					{
						*version = header.m_nVersion;
					}
					return (fptr);
				}
			}
			fclose(fptr);
			break;
		case SCRIPT_TYPE_HCD:
			return (fptr);
		default:
			break;
	}
	return (NULL);
}

/* Extact the next action from a TI BTS fle */
int get_next_action(DEV_MX1 *dev, const FILE *fptr, uint16_t *type)
{
	/*
	 * Each Action contains:
	 * uint16   type (action type)
	 * uint16   size (size of data to follow)
	 * uint8[]  data (Full HCI packet)
	 */
	uint16_t header[2];
	uint16_t size = 0;

	if (fptr == NULL)
		return 0;

	/* Read action header */
	while (fread(header, sizeof (header), 1, (FILE *) fptr) == 1)
	{
		*type = ENDIAN_LE16(header[0]);
		size = header[1];

		/* Read in Data */
		if (size > 0 && size <= MAX_ACTION_SIZE)
		{
			dev->cmd_nbytes = fread(dev->cmd_buf, sizeof (uint8_t), size, (FILE *) fptr);
			return (dev->cmd_nbytes);
		}
		else
			size = 0;
	}

	return (size);
}

/* Extract the next command from a Broadcom HCD file */
int get_next_command(DEV_MX1 *dev, const FILE *fptr)
{
	uint16_t size = 0;

	if (fptr == NULL)
		return 0;

	/* Read opcode and payload length leaving room to prepend the HCI header*/
	while (fread(&dev->cmd_buf[1], 3, 1, (FILE *) fptr) == 1)
	{
		dev->cmd_buf[0] = 0x01;	/* pre-pend HCI Header -  BT Channel -> Host to send HCI commands to chip */
		size = dev->cmd_buf[3];

		/* Read in Data
		 * Account for the 4 byte header when validating the command size.
		 */
		if (size > 0 && size <= (MAX_ACTION_SIZE - 4))
		{
			dev->cmd_nbytes = fread(&dev->cmd_buf[4], sizeof (uint8_t), size, (FILE *) fptr);
			dev->cmd_nbytes += 4;	/* Add HCI header, opcode and size onto data payload length */
			return (dev->cmd_nbytes);
		}
		else
			size = 0;
	}

	return (size);
}

int write_command(DEV_MX1 *dev, int size)
{
	unsigned      cr1;
	static int    cnt = 0;
	uintptr_t     base = dev->base;

	if (dev->usedma)
	{
#ifdef USE_DMA
		dma_transfer_t   tinfo;
		dma_addr_t       dma_addr;

		/* DMA Transaction in progress, wait for it to finish */
		if(!(in32(base + MX1_UART_SR2) & MX1_USR2_TXDC))
		{
			dev->vdev[0].tty.un.s.tx_tmr = 3;		 /* Timeout 3 */
			return 0;
		}

		memcpy(dev->tx_dma.buf, dev->cmd_buf, dev->cmd_nbytes);
		/* Configure DMA buffer address and transfer size */
		memset(&tinfo, 0, sizeof(tinfo));
		tinfo.xfer_bytes = dma_addr.len = dev->cmd_nbytes;
		dma_addr.paddr = dev->tx_dma.phys_addr;
		tinfo.src_addrs = &dma_addr;
		tinfo.dst_addrs = NULL;
		tinfo.xfer_unit_size = 8;
		tinfo.src_fragments = 1;

		dev->cmd_nbytes = 0;
		dev->sdmafuncs.setup_xfer(dev->tx_dma.dma_chn, &tinfo);
		dev->sdmafuncs.xfer_start(dev->tx_dma.dma_chn);
#endif
	}
	else
	{
		while (dev->cmd_nbytes > 0 && (in32(base + MX1_UART_SR1) & MX1_USR1_TRDY))
		{

			/* Print the character */
			dev->vdev[0].tty.un.s.tx_tmr = 3;		 /* Timeout 3 */
			out32(base + MX1_UART_TXDATA, dev->cmd_buf[cnt++]);
			dev->cmd_nbytes--;
		}

		if (dev->cmd_nbytes <= 0)
		{
			dev->vdev[0].tty.un.s.tx_tmr = 0;
			pthread_sleepon_lock();
			cnt = 0;
			pthread_sleepon_signal(&dev->cmd_buf);
			pthread_sleepon_unlock();
		}
		else
		{
			/* If there is still data in the cmd_buf turn TX interrupts back
			 * on to notify us when the hardware is ready for more characters.
			 */
			cr1 = in32(base + MX1_UART_CR1);
			out32(base + MX1_UART_CR1, cr1 | MX1_UCR1_TRDYEN);
		}
	}

	return (0);
}

int wait_tx_complete(DEV_MX1 *dev)
{
	int status = EBUSY;

	if (dev->cmd_nbytes)
	{
		pthread_sleepon_lock();
		if (dev->vdev[0].tty.verbose >= EVENT)
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "Waiting for SEND_CMD completion - %s", dev->vdev[0].tty.name);
		pthread_sleepon_wait(&dev->cmd_buf);
		if (dev->vdev[0].tty.verbose >= EVENT)
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "SEND_CMD completed - %s", dev->vdev[0].tty.name);
		status = EOK;
		pthread_sleepon_unlock();
	}
	else
		status = EOK;

	return (status);
}

#define HCI_COMMAND_COMPLETE 0xE

int wait_rx_ack(DEV_MX1 *dev, uint32_t ms_timeout)
{
	int status = EBUSY;
	uint64_t ns_timeout = ms_timeout * 1000000LL;

	pthread_sleepon_lock();
	if (dev->vdev[0].tty.verbose >= EVENT)
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "Waiting for command Acknowledgment - %s",
			  dev->vdev[0].tty.name);

	if (!dev->response_pending)
	{
		if ((status = pthread_sleepon_timedwait(&dev->vdev[0].tty.ibuf, ns_timeout)) == ETIMEDOUT)
		{
			dev->flags |= RX_ERROR;
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "Command Acknowledgment Timed out - %s",
				  dev->vdev[0].tty.name);
		}
		else
		{
			if (dev->vdev[0].tty.verbose >= EVENT)
				slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "Command Acknowledged1 - %s", dev->vdev[0].tty.name);
		}
	}
	else
	{
		if (dev->vdev[0].tty.verbose >= EVENT)
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "Command Acknowledged2 - %s", dev->vdev[0].tty.name);
	}
	dev->response_pending = 0;
	pthread_sleepon_unlock();

	if (dev->vdev[0].tty.verbose > EVENT)
		display_data_bytes(dev->vdev[0].tty.name, dev->vdev[0].tty.ibuf.tail, dev->vdev[0].tty.ibuf.cnt, "RESPONSE");
	if (!(dev->flags & RX_ERROR) && dev->vdev[0].tty.ibuf.tail[1] == HCI_COMMAND_COMPLETE)
		status = EOK;
	else
	{
		if (dev->flags & RX_ERROR)
		{
			dev->flags &= ~(RX_ERROR);
			status = EIO;
		}
		else
			status = EINVAL;
	}
	dev_lock(&dev->vdev[0].tty);
	dev->vdev[0].tty.ibuf.cnt = 0;
	dev->vdev[0].tty.ibuf.tail = dev->vdev[0].tty.ibuf.head;
	dev->vdev[0].tty.cbuf.cnt = 0;
	dev->vdev[0].tty.cbuf.tail = dev->vdev[0].tty.cbuf.head = dev->vdev[0].tty.cbuf.buff;
	/* flushing buffers, reset OVERRUN slog suppression flags */
	atomic_clr(&dev->vdev[0].tty.shush, EVENT_OVERRUN | EVENT_OVERRUN_SW);
	dev_unlock(&dev->vdev[0].tty);
	return (status);
}

static void
change_soc_baud_rate(DEV_MX1 *dev, int baud_rate)
{
	dev->vdev[0].tty.baud = baud_rate;
	dev->vdev[1].tty.baud = baud_rate;
	dev->vdev[2].tty.baud = baud_rate;
	tto(&dev->vdev[0].tty, TTO_STTY, 0);

	slogf(  _SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "Baud Rate update successfully to %d", baud_rate);
}

static void
change_baud_rate(DEV_MX1 *dev, int baud_rate )
{
	uint8_t update_baud_rate[] = { 0x01, 0x18, 0xfc, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint8_t hci_write_uart_clock_setting_48Mhz[] = { 0x01, 0x45, 0xfc, 0x01, 0x01 };
	uint32_t value;
	int status = -1;

	if (baud_rate == dev->vdev[0].tty.baud)
		return;

	/* TODO: When should we select the higher frequency clock on the BT chip??? */
//	if (baud_rate > 3000000)
	{
		/* If baud rate is bigger, make sure that BCM4325 uses 48MHz clk for its UART */
		memcpy( dev->cmd_buf, hci_write_uart_clock_setting_48Mhz, sizeof( hci_write_uart_clock_setting_48Mhz ) );
		dev->cmd_nbytes = sizeof( hci_write_uart_clock_setting_48Mhz );

		if (dev->vdev[0].tty.verbose > EVENT)
			display_data_bytes(dev->vdev[0].tty.name, dev->cmd_buf, dev->cmd_nbytes, "SEND_CMD");

		write_command(dev, dev->cmd_nbytes);
		wait_tx_complete(dev);
		if ((status = wait_rx_ack(dev, DEFAULT_CMD_TIMEOUT)) != EOK)
			slogf(  _SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "Failed to select BT high frequency clock - %s)", strerror(status));
	}

	value = ENDIAN_LE32(baud_rate);
	memcpy(&update_baud_rate[6], &value, sizeof(uint32_t));

	memcpy( dev->cmd_buf, update_baud_rate, sizeof( update_baud_rate ) );
	dev->cmd_nbytes = sizeof( update_baud_rate );

	if (dev->vdev[0].tty.verbose > EVENT)
		display_data_bytes(dev->vdev[0].tty.name, dev->cmd_buf, dev->cmd_nbytes, "SEND_CMD");

	write_command(dev, dev->cmd_nbytes);
	wait_tx_complete(dev);
	if ((status = wait_rx_ack(dev, DEFAULT_CMD_TIMEOUT)) != EOK)
		slogf(  _SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "Failed to update the baud rate of the BT chip - %s)", strerror(status));
	else
		change_soc_baud_rate(dev, baud_rate);
}

void *script_writer(void *data)
{
	DEV_MX1 *dev = (DEV_MX1 *) data;
	int status = EOK;
	uint16_t type = 0, size = 0;
	uint16_t opcode;
	uint32_t value;
	uint32_t retry_cnt = 0;
	int old_cread[NUM_DEVS] = { 0 };
	uint8_t hcc_reset[] = { 0x01, 0x03, 0x0c, 0x00 };
	uint8_t download_minidriver[] = { 0x01, 0x2e, 0xfc, 0x00 };	/* For Broadcom chips */
	int x, script_type = 0;

	/* Set CREAD for script load */
	for (x = 0; x < NUM_DEVS; x++)
	{
		old_cread[x] = (dev->vdev[x].tty.c_cflag & CREAD) ? 1 : 0;
		dev->vdev[x].tty.c_cflag |= CREAD;
		ser_stty(&dev->vdev[x]);	/* Apply CREAD change */
	}

	do
	{
		/* HCC_RESET */
		dev->cmd_nbytes = sizeof(hcc_reset);
		memcpy(dev->cmd_buf, hcc_reset, dev->cmd_nbytes);
		if (dev->vdev[0].tty.verbose > EVENT)
			display_data_bytes(dev->vdev[0].tty.name, dev->cmd_buf, dev->cmd_nbytes, "SEND_CMD");
		write_command(dev, dev->cmd_nbytes);
		wait_tx_complete(dev);
		if ((status = wait_rx_ack(dev, DEFAULT_CMD_TIMEOUT)) != EOK)
		{
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "HCC_RESET Failed - %s)", strerror(status));
			retry_cnt++;
			reset_device(dev);
			continue;
		}

		if (dev->bt_script_name[0] != NULL)
			dev->bt_script = open_script_file(dev->bt_script_name, &script_type, NULL);

		if (dev->bt_script)
		{
			void *buf = &dev->cmd_buf[0];
			switch (script_type)
			{
				case SCRIPT_TYPE_BTS:
					do
					{
						size = get_next_action(dev, dev->bt_script, &type);
						if (size == 0)						 /* EOF */
							break;
						switch (type)
						{
							case ACTION_SEND_COMMAND:
								opcode = dev->cmd_buf[1];
								opcode |= (dev->cmd_buf[2] << 8);
								if (dev->vdev[0].tty.verbose > EVENT)
									display_data_bytes(dev->vdev[0].tty.name, dev->cmd_buf, dev->cmd_nbytes, "SEND_CMD");
								write_command(dev, size);
								wait_tx_complete(dev);
								break;
							case ACTION_SERIAL_PORT_PARAMETERS:
								value = ENDIAN_LE32(*(uint32_t *) buf);
								change_soc_baud_rate(dev, value);
								break;
							case ACTION_DELAY:
								value = ENDIAN_LE32(*(uint32_t *) buf);
								delay(value);
								break;
							case ACTION_RUN_SCRIPT:
							case ACTION_REMARKS:
								break;
							case ACTION_WAIT_EVENT:
								value = ENDIAN_LE32(*(uint32_t *) buf);
								status = wait_rx_ack(dev, value);
								break;
							default:
								break;
						}
					} while (status == EOK);

					if (status != EOK)
					{
						slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "BT Script load failed - Reset - (%s)", strerror(errno));
						fclose(dev->bt_script);
						reset_device(dev);
						retry_cnt++;
						continue; /* Loop back and attempt to load the script again*/
					}

					break;
				case SCRIPT_TYPE_HCD:
					/* If user specifies higher baud rate for downloading .HCD file */
					change_baud_rate(dev, dev->baud_mid);

					/* Broadcom Download MiniDriver command */
					dev->cmd_nbytes = sizeof(download_minidriver);
					memcpy(dev->cmd_buf, download_minidriver, dev->cmd_nbytes);
					if (dev->vdev[0].tty.verbose > EVENT)
						display_data_bytes(dev->vdev[0].tty.name, dev->cmd_buf, dev->cmd_nbytes, "SEND_CMD");
					write_command(dev, dev->cmd_nbytes);
					wait_tx_complete(dev);
					if ((status = wait_rx_ack(dev, DEFAULT_CMD_TIMEOUT)) != EOK)
					{
						slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "Download MiniDriver Command Failed - %s)", strerror(status));
						fclose(dev->bt_script);
						reset_device(dev);
						retry_cnt++;
						continue; /* Loop back and attempt to load the script again*/
					}
					delay(10);	/* Give Broadcom chip time to except HCD upload */

					do
					{
						size = get_next_command(dev, dev->bt_script);
						if (size == 0)						 /* EOF */
							break;
						if (dev->vdev[0].tty.verbose > EVENT)
							display_data_bytes(dev->vdev[0].tty.name, dev->cmd_buf, dev->cmd_nbytes, "SEND_CMD");
						write_command(dev, size);
						wait_tx_complete(dev);
						status = wait_rx_ack(dev, DEFAULT_CMD_TIMEOUT);
					} while (status == EOK);

					if (status != EOK)
					{
						slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "BT HCD Script load failed - Reset - %s)", strerror(errno));
						fclose(dev->bt_script);
						reset_device(dev);
						retry_cnt++;
						continue; /* Loop back and attempt to load the script again*/
					}

					/* HCD script sets BT chip's baud rate back to 115200 */
					change_soc_baud_rate(dev, DEFAULT_BAUD_RATE);

					/* Issue another HCC_RESET */
					dev->cmd_nbytes = sizeof(hcc_reset);
					memcpy(dev->cmd_buf, hcc_reset, dev->cmd_nbytes);
					if (dev->vdev[0].tty.verbose > EVENT)
						display_data_bytes(dev->vdev[0].tty.name, dev->cmd_buf, dev->cmd_nbytes, "SEND_CMD");
					write_command(dev, dev->cmd_nbytes);
					wait_tx_complete(dev);
					if ((status = wait_rx_ack(dev, DEFAULT_CMD_TIMEOUT)) != EOK)
					{
						slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "HCC_RESET Failed - %s)", strerror(status));
						fclose(dev->bt_script);
						reset_device(dev);
						retry_cnt++;
						continue; /* Loop back and attempt to load the script again*/
					}

					/* Now that HCD file has been transferred to BT chip increase the baud rate */
					change_baud_rate(dev, dev->baud_high);
					break;
				default:
					break;
			}
			/* Done sending the script file commands */
			fclose(dev->bt_script);
		}
		/* If the max_retry is 0 (meaning infinity), then loop forever until this succeeds
		 * If the max_retry is set, then loop until we reach the count */
	}
	while (status != EOK && (dev->max_retry == 0 || retry_cnt < dev->max_retry));

	dev->flags &= ~(SCRIPT_MODE);
	for (x = 0; x < NUM_DEVS; x++)
	{
		/* Restore CREAD state */
		if (!old_cread[x])
		{
			dev->vdev[x].tty.c_cflag &= ~(CREAD);
			ser_stty(&dev->vdev[x]);	/* Apply CREAD change */
		}

		if(status == EOK)
		{
			/* Carrier is only asserted if we were able to successfully HCC_RESET and upload the script */
			dev->flags |= HCI_CARRIER;
			tti(&dev->vdev[x].tty, TTI_CARRIER);
			dev->vdev[x].tty.oband_data |= _OBAND_SER_MS;
			atomic_set(&dev->vdev[x].tty.flags, OBAND_DATA);
			atomic_set(&dev->vdev[x].tty.flags, EVENT_NOTIFY_OBAND);
			iochar_send_event(&dev->vdev[x].tty);
		}
		else
		{
			/* HCC_RESET/Script load failed
			 * NOTE: Use _OBAND_SER_EXTENDED to notify client to
			 *       use driver specific devctl to read out extended
			 *       oband data flags.
			 */
			dev->vdev[x].tty.oband_data = _OBAND_SER_EXTENDED;
			dev->vdev[x].oband_data |= HCI_INIT_FAILED; /* Driver specific oband_data */
			atomic_set(&dev->vdev[x].tty.flags, OBAND_DATA);
			atomic_set(&dev->vdev[x].tty.flags, EVENT_NOTIFY_OBAND);
			iochar_send_event(&dev->vdev[x].tty);
       }
	}

	if(status == EOK)
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "BT initialization complete");
	else
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "BT initialization failed");

	return (NULL);
}

int reset_device(DEV_MX1 *dev)
{
	int x;

	for (x = 0; x < NUM_DEVS; x++)
	{
		dev->vdev[x].tty.oband_data = 0;
		dev->vdev[x].oband_data = 0;
	}
	dev->flags &= ~(HCI_CARRIER);

	dev->flags |= SCRIPT_MODE;
	dev->gpio->data_reg &= ~(1 << dev->gpio_num);		/* Disable BT */

	/* Clear flow control */
	tto(&dev->vdev[0].tty, TTO_CTRL, _SERCTL_RTS_CHG | _SERCTL_RTS);

	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "init script hanging up");
	for (x = 0; x < NUM_DEVS; x++)
	{
		tti(&dev->vdev[x].tty, TTI_HANGUP);
		dev->vdev[x].tty.oband_data |= _OBAND_SER_MS;
		atomic_set(&dev->vdev[x].tty.flags, OBAND_DATA);
		atomic_set(&dev->vdev[x].tty.flags, EVENT_NOTIFY_OBAND);
		iochar_send_event(&dev->vdev[x].tty);
	}

	/* Default to 115200 */
	change_soc_baud_rate(dev, DEFAULT_BAUD_RATE);

	dev->header_cnt = 0;
	dev->rx_active = -1;
	dev->response_pending = 0;
	for (x = 0; x < NUM_DEVS; x++)
	{
		dev_lock(&dev->vdev[x].tty);
		dev->vdev[x].rx_length = 0;
		dev->vdev[x].tx_length = 0;
		dev->vdev[x].tty.obuf.cnt = 0;
		dev->vdev[x].tty.obuf.tail = dev->vdev[x].tty.obuf.head = dev->vdev[x].tty.obuf.buff;
		dev->vdev[x].tty.ibuf.cnt = 0;
		dev->vdev[x].tty.ibuf.tail = dev->vdev[x].tty.ibuf.head = dev->vdev[x].tty.ibuf.buff;
		dev->vdev[x].tty.cbuf.cnt = 0;
		dev->vdev[x].tty.cbuf.tail = dev->vdev[x].tty.cbuf.head = dev->vdev[x].tty.cbuf.buff;
		/* flushing buffers, reset OVERRUN slog suppression flags */
		atomic_clr(&dev->vdev[x].tty.shush, EVENT_OVERRUN | EVENT_OVERRUN_SW);
		dev_unlock(&dev->vdev[x].tty);
	}

	delay(100);
	dev->gpio->data_reg |= (1 << dev->gpio_num);		/* Enable BT */
	delay(260);
	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "BT enabled");
	return EOK;
}

int run_init_script(DEV_MX1 *dev)
{
	int status = EOK;
	pthread_attr_t tattr;

	if (dev->flags & SCRIPT_MODE)
		return (EOK);

	reset_device(dev);

	pthread_attr_init(&tattr);
	pthread_attr_setdetachstate(&tattr, PTHREAD_CREATE_DETACHED);
	if (pthread_create(NULL, &tattr, (void *) script_writer, dev) != EOK)
	{
		status = errno;
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "Failed to create script_writer thread");
		return (status);
	}
	return 0;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/devc/sermx1_hci/hci.c $ $Rev: 767861 $")
#endif
