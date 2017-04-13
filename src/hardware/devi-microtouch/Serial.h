#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ipc.h>
#include <termios.h>
#include <sys/time.h>
#include <assert.h>
#include <errno.h>
#include <syslog.h>
#include <pthread.h>
#include <time.h>

#define	BAUDRATE            	B9600
#define	SERIAL_CTL_TIMEOUT     	2
#define	SERIAL_DEF_CHANNEL     	"/dev/ser4"

#define WAIT_COMMAND_TIMEOUT_1SEC  1000000	  //  1 sec

#define SER_TIMEOUT		150 /*Timeout value in milliseconds*/

#define TRUE  1
#define FALSE 0

#define OutputDebugString printf

struct _CSerial;

/* CSerial */
typedef struct _CSerial
{
	int fd;
	struct termios old_termios;
	fd_set input_fdset;
    	pthread_mutex_t driver_mutex_lock;

	void (*serPutChar)(struct _CSerial *, unsigned char c);
	unsigned int (*serWrite)(struct _CSerial *, unsigned char *buf, int len);
	unsigned int (*serGetChar)(struct _CSerial *, unsigned char *c);
	unsigned int (*serRead)(struct _CSerial *, unsigned char *buf, int len);
	void (*serClose)(struct _CSerial *);
	int (*serOpen)(struct _CSerial *);
	char (*serGetLastError)(void);

} CSerial;

CSerial *cSerial;

CSerial *CSerial_open(void);
void CSerial_close(CSerial *driver); 

#endif
