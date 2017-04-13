#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>

#define THE_DEV	"/dev/devi/touch%d"

#define _POINTER_BUTTON_1           0x00000001       // right
#define _POINTER_BUTTON_2           0x00000002       // middle
#define _POINTER_BUTTON_3           0x00000004       // left

struct _pointer_packet_hdr {                /* start of struct returned from read() */
    struct timespec         time;
    unsigned long           buttons;        /* Currently depressed buttons */
};

struct _touch2_packet {                     /* common form for "touch" pointing devices (with pressure) */
    struct _pointer_packet_hdr hdr;
    long                    x;              /* num_coord = 2, num_pressure = 1 */
    long                    y;
    long                    pressure;
};

void printPkt( struct _touch2_packet *pkt )
{
	//printf( "buttons %ul\n", pkt->hdr.buttons );
	if (pkt->hdr.buttons & _POINTER_BUTTON_3)
		printf( "click %d  %d\n", (int)pkt->x, (int)pkt->y );
	else
		printf("release\n");
}

/*
 * Read and display touch packets from a devi-* touch driver.
 *
 * The /dev/devi/touch? unit number may be specified on the command line.
 */

int main(int argc, char *argv[])
{
int fd;
int len, unit = 0;
unsigned char buf[256];
char the_dev[32];

    if ( argc > 1 )
    	unit = atoi( argv[1] );
    sprintf( the_dev, THE_DEV, unit );

    //printf( "sizeof( struct _touch2_packet) is %d\n\n", sizeof( struct _touch2_packet ) );
    fd = open( the_dev, O_RDWR );  // was O_RDWR
    while ( fd > -1 )
    {
   		len = read(fd, buf, sizeof( struct _touch2_packet ) );
   		if ( len == sizeof( struct _touch2_packet ) )
   	   		printPkt( (struct _touch2_packet *) buf );
   		else
   		if ( len )
   		{
   		    printf( "error - read length %d\n", len );
   		    break;
   		}
    }
    if ( fd < 0 )
    	printf( "bad file descriptor\n" );
    printf( "exit\n" );
    return EXIT_SUCCESS;
}
