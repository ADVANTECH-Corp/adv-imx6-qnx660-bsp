/* 
 * Copyright (c) 1998  Metro Link Incorporated
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, cpy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE X CONSORTIUM BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Except as contained in this notice, the name of the Metro Link shall not be
 * used in advertising or otherwise to promote the sale, use or other dealings
 * in this Software without prior written authorization from Metro Link.
 *
 */
/* 
 * Based, in part, on code with the following copyright notice:
 *
 * Copyright 1996 by Patrick Lecoanet, France. <lecoanet@cenaath.cena.dgac.fr>       
 *                                                                            
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is  hereby granted without fee, provided that
 * the  above copyright   notice appear  in   all  copies and  that both  that
 * copyright  notice   and   this  permission   notice  appear  in  supporting
 * documentation, and that   the  name of  Patrick  Lecoanet not  be  used  in
 * advertising or publicity pertaining to distribution of the software without
 * specific,  written      prior  permission.     Patrick Lecoanet   makes  no
 * representations about the suitability of this software for any purpose.  It
 * is provided "as is" without express or implied warranty.                   
 *                                                                            
 * PATRICK LECOANET DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT  SHALL PATRICK LECOANET BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA  OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS  ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 *
 */
/* $XFree86: xc/programs/Xserver/hw/xfree86/input/microtouch/microtouch.c,v 1.11 1999/08/28 10:43:36 dawes Exp $ */

#define _microtouch_C_
/*****************************************************************************
 *	Standard Headers
 ****************************************************************************/

//#include <exevents.h>	/* Needed for InitValuator/Proximity stuff	*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <fcntl.h>


/*****************************************************************************
 *	Local Headers
 ****************************************************************************/
#include "microtouch.h"

#define Success   1

#define TRUE 	1
#define FALSE 	0

#if 0
#define DEBUGP printf
#else
#define DEBUGP(...)
#endif

/*****************************************************************************
 *	Variables without includable headers
 ****************************************************************************/

/*****************************************************************************
 *	Local Variables
 ****************************************************************************/

#if 0
static const char *default_options[] =
{
	"BaudRate", "9600",
	"StopBits", "1",
	"DataBits", "8",
	"Parity", "None",
	"Vmin", "5",
	"Vtime", "1",
	"FlowControl", "None"
};

static const char *fallback_options[] =
{
	"BaudRate", "9600",
	"StopBits", "2",
	"DataBits", "7",
	"Parity", "None",
	"Vmin", "1",
	"Vtime", "1",
	"FlowControl", "None"
};
#endif

LocalDevicePtr public_device_ptr = NULL;
extern int calibrate;

/*****************************************************************************
 *	Function Definitions
 ****************************************************************************/

static int
Calibrate (LocalDevicePtr local)
{
        MuTPrivatePtr priv = (MuTPrivatePtr) local->priv;
	int ret = Success;

	if (!MuTSendCommand ( (unsigned char *)MuT_CALIBRATE_EXT, priv))
	{
		ret = -1;
		goto done;
	}

	printf("Touch the sensor at a lower left target\n");

	if (MuTGetPacket (priv) != Success) {
		ret = -1;
		goto done;
	}

	printf("Touch the sensor at an upper right target\n");

	if (MuTGetPacket (priv) != Success) {
		ret = -1;
		goto done;
	}


  done:
	return (ret);
}



InputInfoPtr
MuTouchInit(char *ser_node)
{
	LocalDevicePtr local;
	MuTPrivatePtr priv;
	printf("MicroTouch driver MuTouchInit\n");

	public_device_ptr = (LocalDevicePtr)malloc(sizeof (LocalDevicePtr));
	if (!public_device_ptr) 
		return NULL;
	local = public_device_ptr;

	local->priv = malloc (sizeof (MuTPrivateRec));
	if (!local->priv) {
		if (local)
			free(local);
		return NULL;
	}

	priv = local->priv;
	local->read_input = ReadInput;
	//local->close_proc = CloseProc; /* don't use it, it can not "/dev/devi/touch0" file */

	local->cSerial = CSerial_open();
	if (local->cSerial == NULL)
	{
		printf("MicroTouch driver unable to open device\n");
		goto SetupProc_fail;
	}
	DEBUGP("serial port opened successfully\n" );

	priv->min_x = 0;
	priv->max_x = 640;
	priv->min_y = 0;
	priv->max_y = 480;
	priv->button_number = 1;

	priv->buffer = malloc(200);
	priv->proximity = 0;
	priv->button_down = 0;

	MuTNewPacket (priv);

	if (QueryHardware(local) != Success)
	{
		printf("Unable to query/initialize MicroTouch hardware.\n");
		goto SetupProc_fail;
	}

	if (calibrate) {
		if (Calibrate (local) != Success)
			printf("Calibrate Extended failed\n");
		printf("Calibration successful...\n");
		goto SetupProc_fail;
	}

	/* prepare to process touch packets */
	MuTNewPacket (priv);

	if (!local->cSerial)
	{ 
		if (priv->buffer)
		{
			free(priv->buffer);
			priv->buffer = NULL;
		}
		//if (local->cSerial) {
		//	CSerial_close(local->cSerial); 
		//	local->cSerial = NULL;
		//}
	}

	return (local);

  SetupProc_fail:
	//if ((local) && (local->cSerial))
	//	CSerial_close(local->cSerial); 
	if ((priv) && (priv->buffer))
		free(priv->buffer);
	if (priv)
		free (priv);
	return NULL;
}

#define WORD_ASSEMBLY(byte1, byte2) (((byte2) << 7) | (byte1))

static int
ReadInput (LocalDevicePtr local, int *x, int *y, int *button_down)
{
	int type;
	MuTPrivatePtr priv = (MuTPrivatePtr) (local->priv);

	if (MuTGetPacket (priv) == Success)
	{

		type = priv->packet[0];
		*x = WORD_ASSEMBLY (priv->packet[1], priv->packet[2]);
		*y = WORD_ASSEMBLY (priv->packet[3], priv->packet[4]);

		if ((priv->proximity == FALSE) && (type & MuT_CONTACT))
		{
			priv->proximity = TRUE;
		}

		/* 
		 * Emit a button press or release.
		 */
		if ((priv->button_down == FALSE) && (type & MuT_CONTACT))

		{
			priv->button_down = TRUE;
			*button_down = TRUE;
		}
		if ((priv->button_down == TRUE) && !(type & MuT_CONTACT))
		{
			priv->button_down = FALSE;
			*button_down = FALSE;
		}
		/* 
		 * the untouch should always come after the button release
		 */
		if ((priv->proximity == TRUE) && !(type & MuT_CONTACT))

		{
			priv->proximity = FALSE;
		}

		DEBUGP("TouchScreen: x(%d), y(%d), %d %d %s\n",
						*x, *y, type, type & MuT_CONTACT,
						(type & MuT_CONTACT) ? "Press" : "Release" );
	}

	return Success;
}

static void
CloseProc (LocalDevicePtr local)
{
	if (local->cSerial) {
		CSerial_close(local->cSerial); 
		local->cSerial = NULL;
	}
}

static int
MuTSendCommand (unsigned char *type, MuTPrivatePtr priv)
{
	int r;
	int retries = MuT_RETRIES;

	while (retries--)
	{
		if (MuTSendPacket (type, strlen ( (char *)type), priv) != Success) {
			continue;
		}
		r = MuTWaitReply ( (unsigned char *)MuT_OK, priv);
		if (r == ACK)
			return (TRUE);
		else if (r == NACK)
			return (FALSE);
	}
	return (FALSE);
}


/* 
 * The microtouch SMT3 factory default is 72N, but the recommended operating
 * mode is 81N. This code first tries 81N, but if that fails it switches to
 * 72N and puts the controller in 81N before proceeding
 */
static int
QueryHardware (LocalDevicePtr local)
{
        MuTPrivatePtr priv = (MuTPrivatePtr) local->priv;
	int ret = Success;

	if (!MuTSendCommand ( (unsigned char *)MuT_RESET, priv))
	{
		ret = 0;
		goto done;
	}

	if (!MuTSendCommand ( (unsigned char *)MuT_ABDISABLE, priv))
	{
		ret = 0;
		goto done;
	}
	if (!MuTSendCommand ( (unsigned char *)MuT_SETRATE, priv))
	{
		ret = 0;
		goto done;
	}

	if (!MuTSendCommand ( (unsigned char *)MuT_FORMAT_TABLET, priv))
	{
		ret = 0;
		goto done;
	}
	if (!MuTSendCommand ( (unsigned char *)MuT_MODE_STREAM, priv))
	{
		ret = 0;
		goto done;
	}
	if (!MuTSendCommand ( (unsigned char *)MuT_PARAM_LOCK, priv))
	{
		ret = 0;
		goto done;
	}

	if (MuTSendPacket ( (unsigned char *)MuT_OUTPUT_IDENT, strlen (MuT_OUTPUT_IDENT),
							   priv) == Success)
	{
		if (MuTGetPacket (priv) == Success)
			MuTPrintIdent (priv->packet);
	}

	/* some microtouch controllers support one command, some support the
	 * * other. If the first one get's a NACK, try the second. They both
	 * * return the same packet. */
	if (MuTSendPacket ( (unsigned char *)MuT_UNIT_VERIFY, strlen (MuT_UNIT_VERIFY),
						   priv) == Success)
	{
		if ((MuTGetPacket (priv) == Success) &&
			(strcmp ( (char *)&(priv->packet[1]), MuT_ERROR) == 0))
		{
			if (MuTSendPacket ( (unsigned char *)MuT_UNIT_TYPE,
							   strlen (MuT_UNIT_TYPE), priv) == Success)
			{
				if ((MuTGetPacket (priv) != Success))
				{
					ret = FALSE;
					goto done;
				}
			}
		}
		ret = MuTPrintHwStatus (priv->packet);
	}

  done:
	return (ret);
}

static void
MuTNewPacket (MuTPrivatePtr priv)
{
	priv->lex_mode = microtouch_normal;
	priv->packeti = 0;
	priv->binary_pkt = FALSE;
}

/* 
 ***************************************************************************
 *
 * MuTSendPacket --
 *  Emit a variable length packet to the controller.
 *  The function expects a valid buffer containing the
 *  command to be sent to the controller.  The command
 *  size is in len
 *  The buffer is filled with the leading and trailing
 *  character before sending.
 *
 ***************************************************************************
 */
static int
MuTSendPacket (unsigned char *type, int len, MuTPrivatePtr priv)
{
	int result;
	unsigned char req[MuT_PACKET_SIZE];

	memset (req, 0, MuT_PACKET_SIZE);
	strncpy ((char *) &req[1],  (char *)type, strlen ( (char *)type));
	req[0] = MuT_LEAD_BYTE;
	req[len + 1] = MuT_TRAIL_BYTE;

   	result = public_device_ptr->cSerial->serWrite(public_device_ptr->cSerial, req, len + 2);
	if (result != len + 2)
	{
		printf("System error while sending to MicroTouch touchscreen.\n" );
		return FALSE;
	}
	else
		return TRUE;
}

/* 
 ***************************************************************************
 *   0 ACK
 *  -1 NACK
 *  -2 timeout
 *  -3 wrong packet type
 ***************************************************************************
 */
static int
MuTWaitReply (unsigned char *type, MuTPrivatePtr priv)
{
	int ok;
	int wrong, empty;

	wrong = MuT_MAX_WRONG_PACKETS;
	empty = MuT_MAX_EMPTY_PACKETS;
	do
	{
		ok = 0;

		/* 
		 * Wait half a second for the reply. The fuse counts down each
		 * timeout and each wrong packet.
		 */
		DEBUGP("Waiting %d ms for data from port\n", MuT_MAX_WAIT / 1000 );
		MuTNewPacket (priv);
		ok = MuTGetPacket (priv);
		/* 
		 * type is a NULL terminated string of 0 - 2 characters. An empty
		 * string for type indicates that any reply type is acceptable.
		 */
		if (ok != Success)
		{
			DEBUGP("Recieved empty packet.\n" );
			empty--;
			continue;
		}
		if (ok == Success)
		{
			/* 
			 * this bit of weirdness attempts to detect an ACK from the
			 * controller when it is in 7bit mode and the computer is in 8bit
			 * mode. If we see this pattern and send a NACK here the next
			 * level up will switch to 7 bit mode and try again
			 */
			if (priv->cs7flag && (priv->packet[1] == MuT_OK7) &&
				(priv->packet[2] == '\0'))
			{
				DEBUGP("Detected the 7 bit ACK in 8bit mode.\n" );
				return (NACK);
			}
			if (strcmp ( (char *)&(priv->packet[1]),  (char *)type) == 0)
			{
				DEBUGP("\t\tgot an ACK\n" );
				return (ACK);
			}
			else if (strcmp ( (char *)&(priv->packet[1]), MuT_ERROR) == 0)
			{
				DEBUGP("\t\tgot a NACK\n" );
				return (NACK);
			}
			else
			{
				DEBUGP("Wrong reply received\n" );
				ok = 0;
				wrong--;
			}
		}
	}
	while (ok != Success && wrong && empty);

	if (wrong)
		return (TIMEOUT);
	else
		return (WRONG_PACKET);
}

static int
MuTGetPacket (MuTPrivatePtr priv)
{
	int count = 0;
	unsigned char c;

	while (public_device_ptr->cSerial->serGetChar(public_device_ptr->cSerial, &c) >= 0)
	{
		if (count++ > 100)
		{
			MuTNewPacket (priv);
			return FALSE;
		}

		switch (priv->lex_mode)
		{
		case microtouch_normal:
			if ((c == MuT_LEAD_BYTE) ||
				(priv->cs7flag && ((c & 0x7f) == MuT_LEAD_BYTE)))
			{
				DEBUGP("Saw MuT_LEAD_BYTE\n" );
				priv->packet[priv->packeti++] = (unsigned char) c;
				priv->lex_mode = microtouch_body;
			}
			/* 
			 * binary touch packets do not have LEAD_BYTE or TRAIL_BYTE
			 * Instead, only the first byte has the 8th bit set.
			 */
			if (c & 0x80)
			{
				DEBUGP("Saw BINARY start\n" );
				priv->packet[priv->packeti++] = (unsigned char) c;
				priv->lex_mode = mtouch_binary;
				priv->bin_byte = 0;
			}
			break;

		case mtouch_binary:
			priv->packet[priv->packeti++] = (unsigned char) c;
			priv->bin_byte++;
			if (priv->bin_byte == 4)
			{
				DEBUGP("got a good BINARY packet\n" );
				MuTNewPacket (priv);
				priv->binary_pkt = TRUE;
				return (Success);
			}
			break;

		case microtouch_body:
			/* 
			 * apparently a new packet can start in the middle of another
			 * packet if they host sends something at the right time to
			 * trigger it.
			 */
			if ((c == MuT_LEAD_BYTE) ||
				(priv->cs7flag && ((c & 0x7f) == MuT_LEAD_BYTE)))
			{
				priv->packeti = 0;
			}
			if ((c == MuT_TRAIL_BYTE) ||
				(priv->cs7flag && ((c & 0x7f) == MuT_TRAIL_BYTE)))
			{
				/* null terminate the packet */
				priv->packet[priv->packeti++] = '\0';
				DEBUGP("got a good packet\n" );
				MuTNewPacket (priv);
				return (Success);
			}
			else
				priv->packet[priv->packeti++] = (unsigned char) c;
			break;

		}
	}
	return FALSE;
}

/* 
 ***************************************************************************
 *
 * MuTPrintIdent --
 *  Print type of touchscreen and features on controller board.
 *
 ***************************************************************************
 */
static void
MuTPrintIdent (unsigned char *packet)
{
	int vers, rev;

	if (strlen ( (char *)packet) < 6)
		return;
	printf(" MicroTouch touchscreen is " );
	if (strncmp ((char *) &packet[1], MuT_TOUCH_PEN_IDENT, 2) == 0)
		printf( "a TouchPen.\n" );
	else if (strncmp ((char *) &packet[1], MuT_SMT3_IDENT, 2) == 0)
		printf( "a Serial/SMT3.\n" );
	else if (strncmp ((char *) &packet[1], MuT_GENERAL_IDENT, 2) == 0)
		printf( "an SMT2, SMT3V or SMT3RV.\n" );
	else
		printf( "Unknown Type %c%c.\n", packet[1], packet[2] );
	sscanf ((char *) &packet[3], "%2d%2d", &vers, &rev);
	printf(" MicroTouch controller firmware revision is %d.%d.\n", vers, rev);
}

/* 
 ***************************************************************************
 *
 * MuTPrintHwStatus --
 *  Print status of hardware. That is if the controller report errors,
 *  decode and display them.
 *
 ***************************************************************************
 */
static int
MuTPrintHwStatus (unsigned char *packet)
{
	int i;
	int err;

	for (i = 3; i < 7; i++)
	{
		if (packet[i] == 'R')
		{
			printf(" MicroTouch controller is a resistive type.\n" );
		}

	}
	if (packet[7] == '1')
	{
		printf(" MicroTouch controller reports the following errors:\n" );
		err = packet[8];
		if (err & 0x01)
			printf( "\tReserved\n" );
		if (err & 0x02)
			printf( "\tROM error. Firmware checksum verification error.\n" );
		if (err & 0x04)
			printf( "\tPWM error. Unable to establish PWM operating range at power-up.\n" );
		if (err & 0x08)
			printf( "\tNOVRAM error. The operating parameters in the controller NOVRAM are invalid.\n" );
		if (err & 0x10)
			printf( "\tHWD error. The controller hardware failed.\n" );
		if (err & 0x20)
			printf( "\tReserved\n" );
		if (err & 0x40)
			printf( "\tCable NOVRAM error. The linearization data in the cable NOVRAM is invalid.\n" );
		if (err & 0x80)
			printf( "\tNOVRAM2 error. The linearization data in the controller NOVRAM is invalid.\n" );
		return (!Success);
	}

	return (Success);
}
