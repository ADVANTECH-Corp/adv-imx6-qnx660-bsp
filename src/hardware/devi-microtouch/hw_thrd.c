#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include "hw_thrd.h"

#include "microtouch.h"

/****************************************************************************
 *      hw_thrd
 *
 *      Hardware thread routine to detect and return touch points.
 *
 *      microtouch touchpanel "calculates" x/y for 640x480 touchscreen.
 *      This is much simpler than resistive touchscreen controllers
 *      where the driver must wait for an analog-to-digital conversion.
 *
 *     We would prefer to be notified by interrupt, but lacking the
 *     details, this thread will "poll" for the data - accessed via serial.
 *
 *     We will maintain two packets in memory (pktClick and pktRelease)
 *     and alternate use of each.
 *
 *     Synchronization between main resmgr and this thread is maintained
 *     vi "rd_sem".  Most of the time, this thread holds rd_sem.
 *
 ***************************************************************************/

extern int calibrate;

void *hw_thrd( void *p )
{

    struct _touch2_packet pktClick;
    struct _touch2_packet pktRelease;
    int coord_x;
    int coord_y;
    int status;
    int press_or_release;

    char devname[] = "/dev/ser4";
    char *thrdname = "hw_thrd";        // for diagnostic messages
    struct sched_param schedp;

    // we must bump hw_thrd priority ABOVE main() to assure semaphore works as planned
    pthread_getschedparam( pthread_self(), NULL, &schedp );
    pthread_setschedprio( pthread_self(), max(schedp.sched_priority + 1, HIGH_PRIO) );

    pthread_setname_np( 0, thrdname );

#if 1 
    // devi-hid does not update the time with each pkt - so it will not be done
    // in this driver
    // set time once here for use with every event
    clock_gettime( CLOCK_REALTIME, &pktClick.hdr.time );

    // setup pkts for passing touch events to libmtouch-devi.so via our device
    memset( &pktClick, 0, sizeof(pktClick) );
    memcpy( &pktRelease, &pktClick, sizeof(pktClick) );
    pktClick.hdr.buttons = _POINTER_BUTTON_3;  // indicates a touch event
    pktRelease.hdr.buttons = 0;                // indicates a release
    
    // set this global (constant)
    g_dataBufferLen = sizeof( pktClick );

    InputInfoPtr local_input_ptr = MuTouchInit(devname);

    if (calibrate)
	exit(0);
	
    if (local_input_ptr == NULL)
    {
        printf( "%s:  open(%s) error: %s\n", thrdname, devname, strerror(errno));
        exit(-1);
    }

    while ( 1 )
    {
        // wait for signal that a read is pending
    	// WE STAT BLOCKED UNTIL THERE IS A READER!!!
        if ( verbose )
    	  printf( "%s:  blocked\n", thrdname );
        sem_wait( &rd_sem );
        if ( verbose )
          printf( "%s:  unblocked - watch h/w\n", thrdname );

        // wait for data here
        while ( 1 )
        {
	    status = local_input_ptr->read_input(local_input_ptr, &coord_x, &coord_y, &press_or_release); 

            if ( status && press_or_release)
            {
                 pktClick.x = coord_x;
                 pktClick.y = coord_y;

		if (g_dataBuffer)
                 	memcpy( g_dataBuffer, &pktClick, sizeof( pktClick ) );
                 if ( verbose )
                   printf( "%s:  click  x: %d, y: %d\n", thrdname, coord_x, coord_y );
                 break;
            }

            if ( status && !press_or_release )
            {
		if (g_dataBuffer)
                 	memcpy( g_dataBuffer, &pktRelease, sizeof( pktRelease ) );  // a release
                 if ( verbose )
                   printf( "%s:  release\n", thrdname  );
                 break;
            }

            // poll at approx frequency of touch panel logic (60 hz)
            delay(16);
        }  // inner while

        // signal now that data is ready
        sem_post( &rd_sem );
    }  // while
#else  // simulation code
int state = 0;   // 0:waiting for click    1:waiting for release

    pktClick.hdr.buttons = _POINTER_BUTTON_3;

    // these coordinates are known active touch coordinates
    // (i.e. an app icon - to prove the injected "touch" works)
    pktClick.x = 880;
    pktClick.y = 465;

    memcpy( &pktRelease, &pktClick, sizeof(pktClick) );
    pktRelease.hdr.buttons = 0;
    while ( 1 )
    {
        // wait for signal that read is requested
    	if ( verbose )
          printf( "%s:  blocked\n", thrdname );
        sem_wait( &rd_sem );

        // now that data is available, put it in the buffer
        // and set length of data in buffer
        if ( state++ & 0x01 )
        {
            delay(150);
            memcpy( g_dataBuffer, &pktRelease, sizeof( pktRelease ) );
        }
        else
        {
            // simulate wait for data from the hardware - with a sleep()
        	if ( verbose )
              printf( "%s:  sleeping\n", thrdname );
            sleep(2);
            memcpy( g_dataBuffer, &pktClick, sizeof( pktClick ) );
        }
        g_dataBufferLen = sizeof( pktClick );

        // signal that data is ready
        sem_post( &rd_sem );
    }  // while
#endif

    return NULL;
}
