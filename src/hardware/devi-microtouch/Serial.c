
#include "Serial.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

#if 1
#define DEBUGP printf
//#define DEBUG_TXRX
//#define DEBUGP(s, args...) syslog(LOG_INFO, s, ##args)
#else
#define DEBUGP(...)
#endif


#define SER_TIMEOUT		150 /*Timeout value in milliseconds*/

#define FNDELAY O_NONBLOCK
#define CRTSCTS (IHFLOW | OHFLOW)

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

static char lastError=0;

/****************************************************************************
 *
 * Function:  serGetLastError
 *
 *   Get the error code for the last serial.c function that failed.
 *
 * Input: Nothing
 *
 * Return: The error code
 *
 ***************************************************************************/
static char serGetLastError()
{
	return lastError;
}

/****************************************************************************
 *
 * Function:  serOpen
 *
 *   Opens a serial port in a given mode.
 *
 * Input: port, string describing with port to open (ie. COM1, COM2..)
 *        mode, the bitrate plus the number of databits,stopbits and
 *        the parity. Should be given as one of the serial_xxxx_yyy
 *        constants (see serial.h)
 *
 * Return: A win32 HANDLE to the file corresponding to the serial port.
 *
 ***************************************************************************/
static int serOpen(CSerial *driver)
{
      /*O_NONBLOCK:Open for synchronizing two processes
        O_RDWR:Open for reading and writing
		O_NOCTTY:It doen't want to be the "controlling terminal" for that port.
	         	 If you don't specify this then any input (such as keyboard abort
		 	 	 singls and so forth) wiill affert your process.
		O_NDELAY:It tells UNIX that this program doesn't care what state the DCD signal
		 		 line is in whether the other end of the port is up and running. If you
		 		 do not specify this flag, your process will be put to sleep until the DCD
		 		 singal line is the space voltage.
      */

      /*Reading data from a port is a little trickier. When you operate the port in raw
        data mode, each read system call will return however many characters are actually
		available in the serial input buffers. If no characters are avaiable, the call will
		block (wait) until characters come in, an interval timer expires, or an error occrus.
	 	The read function can be made to return immediately by doing the following:
		fcntl(fd,F_SETFL,FNDELAY)
      */

      struct termios Termios;

	  if (driver == NULL)
	  	return -1;

      bzero(&Termios, sizeof(Termios));
      openlog( "CSerial", LOG_NDELAY, LOG_DAEMON );

#if 0
      driver->fd  = open(SERIAL_DEF_CHANNEL, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
#else
      driver->fd  = open(SERIAL_DEF_CHANNEL, O_RDWR | O_NOCTTY);
#endif

      if(driver->fd < 0) {
         DEBUGP("Unable to open /dev/ser4\n");
         return -1;
      }
#if 0
      else
      	fcntl(driver->fd,F_SETFL,FNDELAY);
#endif

      /*tcgetattr:It writes the current values of the terminal interface variables
                  into the structure pointed to by &old_termios
      */
      tcgetattr(driver->fd, &driver->old_termios);


      /*CLOCAL:Ignore any modem status lines
      	CREAD :Enable the receipt of characters
		CS8   :Use eight bits in sent or received characters
		IGNPAR:Ignore characters with parity errors
		ICRNL :Convert a receivd carriage return to a new line
		ICANON:Enable canonical input processing
	       Canonical input is line-oriented.Input characters are put
	       into a buffer which can be edited interactively by the user
	       until a CR(carriage return) or LF(line feed)character is received.
      */

      /*c_cflag:Control options
        c_iflag:Input options
		c_oflag:Output options
		c_lflag:Line options
		c_cc   :Control characters
      */
 	// Insure not owner of port (CLOCAL) enable reading(CREAD).
 	Termios.c_cflag |= (CLOCAL | CREAD);
     	// Set parity, stop bit, bit size (8N1).
    	Termios.c_cflag &= ~PARENB;
    	Termios.c_cflag &= ~CSTOPB;
    	Termios.c_cflag &= ~CSIZE;
    	Termios.c_cflag |= CS8;
    	// Disable hardware flow control.
    	Termios.c_cflag &= ~CRTSCTS;
 
    	// Choose Raw Input Input characters are passed through exactly as
    	// they are received, when they are received.
	 Termios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	 // Disable software flow control.
	 Termios.c_iflag &= ~(IXON | IXOFF | IXANY);
	 // Choose Raw Output, all other option bits in  c_oflag are ignored.
	 Termios.c_oflag &= ~OPOST;
 
	 //Termios.c_cc[VMIN] = 1; /* blocking read until 1 chars received */
 	 //Termios.c_cc[VTIME] = 50; /* inter-character timer unused */
	 Termios.c_cc[VMIN] = 5; /* blocking read until 1 chars received */
 	 Termios.c_cc[VTIME] = 1; /* inter-character timer unused */

#if 0
      Termios.c_cc[VINTR]    = 0;     /* Ctrl-C Interrupt 			*/
      Termios.c_cc[VQUIT]    = 0;     /* Ctrl-Z Quit	  			*/
      Termios.c_cc[VERASE]   = 0;     /* Backspace(BS)	  			*/
      Termios.c_cc[VKILL]    = 0;     /* Ctrul-U Kill-line			*/
      Termios.c_cc[VEOF]     = 4;     /* Ctrl-D End-of-file			*/
      Termios.c_cc[VEOL]     = 0;     /* Carriage return (CR) End-of-Line	*/
      Termios.c_cc[VEOL2]    = 0;     /* Second end-of-line Line feed (LF)	*/
      Termios.c_cc[VSTART]   = 0;     /* Ctrl-q used for software flow control	*/
      Termios.c_cc[VSTOP]    = 0;     /* Ctrl-s used for software flow control	*/
      Termios.c_cc[VSUSP]    = 0;     /* Ctrl-z SUSP character			*/
      Termios.c_cc[VREPRINT] = 0;     /* Ctrl-r */
      Termios.c_cc[VDISCARD] = 0;     /* Ctrl-u */
      Termios.c_cc[VWERASE]  = 0;     /* Ctrl-w */
      Termios.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
#endif

      /*TCSANOW:Change values immediately
        tcsetattr:It reconfigures the terminal interface
        tcflush:It can be used to flush input, ouput or both
      */

      cfsetispeed(&Termios,BAUDRATE);
      cfsetospeed(&Termios,BAUDRATE);
      tcflush(driver->fd, TCIFLUSH);

      tcsetattr(driver->fd, TCSANOW, &Termios);

      /*
		The select system call allows a program to wait for input to arrive (or output to complete)
		on a number of low-level file descriptors at once. This means that the terminal emulator
		program can block until there is something to so. The select function operates on data
		structure, fd_set, that are sets of open file descriptos.

      	FD_ZERO:It initializes an fd_set to the empty set.
        FD_SET:It sets elements of the set corresponding to the file descriptor passed as fd.
      */

#if 0
      FD_ZERO(&driver->input_fdset);
      FD_SET(driver->fd, &driver->input_fdset);
#endif

	  return driver->fd;
}

/****************************************************************************
 *
 * Function:  serClose
 *
 *   Free a serial port after use.
 *
 * Input: h, the win32 handle returned by serOpen().
 *
 * Return: Nothing
 *
 ***************************************************************************/
void serClose(CSerial *driver)
{
	if (!close(driver->fd))
    {
		printf("can't close\n");
	}
}

/****************************************************************************
 *
 * Function:  serRead
 *
 *   Reads a number of bytes from a serial port. Will wait until 
 *   all request bytes are available.
 *
 * Input: h, the win32 handle returned by serOpen
 *        buf, pointer to where the data should be stored
 *        len, the number of bytes to receive
 *
 * Return: The number of bytes actually read.
 *
 ***************************************************************************/
static unsigned int serRead(CSerial *driver, unsigned char *buf, int len)
{
	int bytes;
   	bytes = read(driver->fd, buf, len);
	return bytes;
}

/****************************************************************************
 *
 * Function:  serGetChar
 *
 *   Read a single byte from a serial port.
 *
 * Input: h, the win32 handle returned by serOpen()
 *        bStop, pointer to boolean indicating if the thread is stopping
 *
 * Return: The read byte.
 *
 ***************************************************************************/
static unsigned int serGetChar(CSerial *driver, unsigned char *c)
{
	int res = serRead(driver, c, 1);
#ifdef DEBUG_TXRX
	if (res > 0)
  	{
		char ba[10];
		sprintf(ba,"0x%02X ",*c);
		OutputDebugString(ba);
	}
#endif
	return res;
}

/****************************************************************************
 *
 * Function:  serWrite
 *
 *   Writes a number of bytes to a serial port. Will wait until 
 *   all bytes are sent.
 *
 * Input: h, the win32 handle returned by serOpen
 *        buf, pointer to where the data are be stored
 *        len, the number of bytes to send
 *
 * Return: The number of bytes actually send.
 *
 ***************************************************************************/
static unsigned int serWrite(CSerial *driver, unsigned char *buf, int len)
{
   	int rc = write(driver->fd, buf, len);

#ifdef DEBUG_TXRX
  	int i;
  	char ba[30];
	OutputDebugString("Serial API: ");
  	for (i = 0; i < len; i++)
  	{
	  sprintf(ba, "0x%02X ", ((unsigned char *)buf)[i]);
	  OutputDebugString(ba);
  	}
	OutputDebugString("\n");
#endif

	return rc;	
} 

/****************************************************************************
 *
 * Function:  serPutChar
 *
 *   Write a single byte to a serial port.
 *
 * Input: h, the win32 handle returned by serOpen()
 *        c, the char to write/send
 *
 * Return: Nothing.
 *
 ***************************************************************************/
static void serPutChar(CSerial *driver, unsigned char c)
{
	serWrite(driver, &c, 1);
}

CSerial *CSerial_open()
{
  CSerial  *driver = NULL;
  driver = (CSerial *) malloc(sizeof(CSerial));

  if (driver) {
  	driver->serOpen  		= serOpen;
  	driver->serClose 		= serClose;
  	driver->serWrite 		= serWrite;
  	driver->serRead	 		= serRead;
	driver->serGetChar 		= serGetChar;
	driver->serPutChar  	= serPutChar;
	driver->serGetLastError = serGetLastError;

  	if (driver->serOpen(driver) <= 0) {
		free(driver);
		driver = NULL;
		return NULL;
	}
  }

  return driver;
}    

void CSerial_close(CSerial *driver) 
{
	if (driver) {
		driver->serClose(driver);
		free(driver);
		driver = NULL;
	}
}

