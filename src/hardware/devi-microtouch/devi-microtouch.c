/*************************************************************************
 *
 *  devi-microtouch.c
 *
 *  This module contains a touch "Resource Manager" designed to interface
 *  with screen via libmtouch-devi.so.
 *
 *  It interfaces with the microtouch touch panel.  Interactions
 *  are via serial.
 *
 *  It writes a 24-byte "struct _touch2_packet" of touch data on each touch
 *  event to /dev/devi/touch0.  libmtouch-devi.so read our device and
 *  forwards the events to screen.
 *
 *  This code makes "resmgr library" calls - but all are contained in libc
 *  so no extra libraries need be linked.
 *
 *  All hw-specific code is placed in hw_thrd.c.  For a pure-software
 *  usde of this resource manager, see devi-simtouch project.
 *
 *************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <string.h>
#include <pthread.h>
#include "hw_thrd.h"

// extend the ocb and the attr variables (sim_ocb_s and sim_attr_s)
// they are really sim_ocb_t and sim_attr_t below.
struct sim_attr_s;
#define IOFUNC_ATTR_T struct sim_attr_s

struct sim_ocb_s;
#define IOFUNC_OCB_T struct sim_ocb_s

#include <sys/iofunc.h>  // must follow the above def's

#define TRUE    1
#define FALSE   0

// define our device attributes structure.
typedef struct sim_attr_s {
    iofunc_attr_t attr;
    int           device;     // always UNIT_ORG (see below)
} sim_attr_t;

// define our open context block structure.
typedef struct sim_ocb_s {
    iofunc_ocb_t ocb;      // this has 'IOFUNC_ATTR_T *attr' at its top
                           // (ocb.offset is the current position in the buffer)
    char         *buffer;  // the data returned to the client
    int          bufsize;  // the size of the buffer
} sim_ocb_t;

#define NUM_DEV         1  // only support one device supported
#define MAX_DEVNAM_SIZ  32
#define UNIT_ORG        0  // unit is always zero regardless of external name touchUnitNbr

// declare the tables used by the resource manager.
#define SIM_DIR "/dev/devi"

// device names table
char devNameBuffer[MAX_DEVNAM_SIZ];
char *devnames[NUM_DEV] =
{
    devNameBuffer   // default is "/dev/devi/touch0"
};

// pathname ID table
int pathnameID[NUM_DEV];

// device information table
sim_attr_t  sim_attrs[NUM_DEV];

// read semaphore
sem_t rd_sem;

// i/o functions
int io_read(resmgr_context_t *ctp, io_read_t *msg, sim_ocb_t *tocb);
int io_write(resmgr_context_t *ctp, io_write_t *msg, sim_ocb_t *tocb);
sim_ocb_t *sim_ocb_calloc(resmgr_context_t *ctp, sim_attr_t *tattr);
void sim_ocb_free(sim_ocb_t *tocb);

// connect and I/O functions
resmgr_connect_funcs_t connect_funcs;
resmgr_io_funcs_t      io_funcs;

// ocb allocating and freeing functions
iofunc_funcs_t sim_ocb_funcs = {
    _IOFUNC_NFUNCS,
    sim_ocb_calloc,
    sim_ocb_free
};

// mount structure, we only have one so we statically declare it
iofunc_mount_t          sim_mount = { 0, 0, 0, 0, &sim_ocb_funcs };

// dispatch, resource manager and iofunc variables
dispatch_t          *dpp;
resmgr_attr_t       rattr;
dispatch_context_t  *ctp;

// forward
void options(int argc, char **argv);
void usage( int err );

// some misc global variables
char *progname = "devi-microtouch";  // for diagnostic messages
int verbose = FALSE;             // -v for verbose operation
/* weilun@adv - begin */
int calibrate = FALSE;             // -c for calibrate operation
/* weilun@adv - end */
int touchUnitNbr = 0;
pthread_t hw_tid;
pthread_attr_t hw_attr;

// shared by io_read and thread - access sychronized by rd_sem
char *g_dataBuffer;
int g_dataBufferLen;

/*************************************************************************/

int main(int argc, char **argv)
{
int ii;

    options(argc, argv);
    if ( opterr )
        usage( -1 );  // does not return

    if ( verbose )
      printf("%s:  starting...\n", progname);

    // now that options() has run, set devNameBuffer
    sprintf( devNameBuffer, "%s/touch%d" , SIM_DIR, touchUnitNbr );

    if ( verbose )
      printf( "%s:  %s\n", progname, devnames[0] );

    dpp = dispatch_create();
    memset(&rattr, 0, sizeof(rattr));

    // intialize the connect functions and I/O functions tables to
    // their defaults and then override the defaults with the
    // functions that we are providing.
    iofunc_func_init( _RESMGR_CONNECT_NFUNCS, &connect_funcs,
                      _RESMGR_IO_NFUNCS, &io_funcs );
    io_funcs.read = io_read;
    io_funcs.write = io_write;

     // call resmgr_attach to register our prefix with the process manager,
     // and also to let it know about our connect and I/O functions.
     // on error, returns -1 and errno is set
     for (ii = 0; ii < NUM_DEV; ii++)
     {
        if ( verbose )
            printf( "%s:  attaching pathname %s\n", progname, devnames[ii] );

        // for this sample program we are using the same mount structure
        // for all devices, we are using it solely for providing the
        // addresses of our ocb calloc and free functions.
        iofunc_attr_init(&sim_attrs[ii].attr, S_IFCHR | 0666, NULL, NULL);
        sim_attrs[ii].attr.mount = &sim_mount;
        sim_attrs[ii].device = ii;  /* 0 is our device */
        pathnameID[ii] = resmgr_attach( dpp, &rattr, devnames[ii],
                                        _FTYPE_ANY, 0, &connect_funcs, &io_funcs, &sim_attrs[ii] );

        if (pathnameID[ii] == -1)
        {
            printf( "%s:  couldn't attach pathname %s, errno %d\n", progname, devnames[ii], errno);
            exit(1);
        }
    }


    if ( verbose )
        printf("%s:  entering dispatch loop\n", progname);

    ctp = dispatch_context_alloc(dpp);

    // setup the semaphore
    sem_init( &rd_sem, 0, 0 );

    // setup the thread
    pthread_attr_init( &hw_attr );
    pthread_attr_setdetachstate( &hw_attr, PTHREAD_CREATE_DETACHED );
    pthread_create( &hw_tid, &hw_attr, hw_thrd, NULL );

    while ( TRUE )
    {
        if ((ctp = dispatch_block(ctp)) == NULL)
        {
            printf( "%s:  dispatch_block failed: %s\n", progname, strerror(errno));
            break;
        }
        dispatch_handler(ctp);
    }
    exit(1);
}  /* main */

/*************************************************************************
 *  sim_ocb_calloc
 *
 *  This routine gives us a place to allocate our own ocb.
 *  It is called as a result of the open being done
 *  (e.g. iofunc_open_default causes it to be called).  We register
 *  it through the mount structure.
*/

sim_ocb_t *sim_ocb_calloc( resmgr_context_t *ctp, sim_attr_t *tattr )
{
sim_ocb_t   *tocb;

    if ( verbose )
        printf("%s:  in sim_ocb_calloc for device %d\n", progname, tattr->device);

    if ((tocb = calloc(1, sizeof(sim_ocb_t))) == NULL)
    {
        if ( verbose )
            printf("%s:  couldn't allocate %d bytes\n", progname, sizeof(sim_ocb_t));
        return NULL;
    }

    // do anything else to the ocb
    tocb->ocb.offset = 0;
    tocb->buffer = NULL;
    return tocb;
}  /* sim_ocb_calloc */

/*************************************************************************
 *  sim_ocb_free
 *
 *  This routine gives us a place to free our ocb.
 *  It is called as a result of the close being done
 *  (e.g. iofunc_close_ocb_default causes it to be called).  We register
 *  it through the mount structure.
*/

void sim_ocb_free(sim_ocb_t *tocb)
{
    if ( verbose )
        printf("%s:  in sim_ocb_free for device %d\n", progname, tocb->ocb.attr->device);

    // return the memory associated with the buffer.
    if (tocb->buffer)
        free(tocb->buffer);

    free(tocb);
}  /* sim_ocb_free */

/*************************************************************************
 *  io_read
 *
 *  At this point, the client has called their library "read"
 *  function, and expects zero or more bytes.
 *
 *  Get the device number for this request from the
 *  "tocb->ocb.attr->device" parameter.
*/

int io_read( resmgr_context_t *ctp, io_read_t *msg, sim_ocb_t *tocb )
{
int nleft;
int nbytes;
int status;

    if ( verbose )
        printf("%s:  in io_read, offset is %lld, nbytes %d\n", progname, tocb->ocb.offset, msg->i.nbytes);

    if ((status = iofunc_read_verify(ctp, msg, &tocb->ocb, NULL)) != EOK)
        return status;

    // no special xtypes
    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE)
        return ENOSYS;

    // normally we would figure out which device based on ocb - but we manage only one!
    // device = tocb->ocb.attr->device;

    // if first read call, buffer will be null - needs allocating
    // subsequent call will occur with a previously allocated buffer
    if (tocb->buffer == NULL)
    {
        // allocate the buffer the size of the input buffer
        tocb->buffer = g_dataBuffer = malloc(msg->i.nbytes);
    }
    else  // thread alway copies to zero!
        tocb->ocb.offset = 0;

    // signal the hw_thrd that we need data
    sem_post( &rd_sem );

    // now wait for a response
    // g_dataBufferLen will be set by thread
    sem_wait( &rd_sem );

    // g_dataBuffer and length are now ready
    tocb->bufsize = g_dataBufferLen;

    // calculate how many bytes we can return to the client, based upon
    // the number of bytes available (nleft) and the client's buffer size
    nleft = tocb->bufsize - tocb->ocb.offset;
    nbytes = min(msg->i.nbytes, nleft);

    // MsgReply here instead of having the resmgr API do it
    if (nbytes)
        MsgReply(ctp->rcvid, nbytes, tocb->buffer + tocb->ocb.offset, nbytes);
    else
        MsgReply(ctp->rcvid, 0, NULL, 0);

    // advance the offset to reflect the number of bytes returned to the client
    tocb->ocb.offset += nbytes;

    if ( msg->i.nbytes > 0 )
        tocb->ocb.attr->attr.flags |= IOFUNC_ATTR_ATIME;

     // always return _RESMGR_NOREPLY because we did the MsgReply() above
    return _RESMGR_NOREPLY;
}  /* io_read */

/*************************************************************************
 *  io_write
 *
 *  At this point, the client has called their library "write"
 *  function.  Writing is not defined for /dev/devi/touch0, so we just
 *  ignore any bytes that they need to be written.
 *  We cannot return error for writing at open time because
 *  libmtouch-devi (unnecessaryily) opens the device for r/w.
*/

int io_write(resmgr_context_t *ctp, io_write_t *msg, sim_ocb_t *tocb)
{
int status;

    if ( verbose )
        printf("%s:  in io_write\n", progname);

    if ((status = iofunc_write_verify(ctp, msg, &tocb->ocb, NULL)) != EOK)
        return status;

    // no special xtypes
    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE)
        return ENOSYS;

    _IO_SET_WRITE_NBYTES(ctp, msg->i.nbytes); // indicate how many we wrote (discarded)

    if (msg->i.nbytes > 0)
        tocb->ocb.attr->attr.flags |= IOFUNC_ATTR_MTIME | IOFUNC_ATTR_CTIME;

    return _RESMGR_NPARTS(0);
}  /* io_write */

/*************************************************************************
 *  options
 *
 *  This routine handles the command line options.
 *  For our /dev/time family, we support:
 *      -u #   /dev/devi/touch#
 *      -v     verbose operation
 *      -c     calibration operation
*/

void options(int argc, char **argv)
{
int opt;

    opterr = 0;
    while ( (opt = getopt(argc, argv, "u:v:c")) != -1 )
    {
       switch (opt)
       {
       case 'u':
           touchUnitNbr = atoi( optarg );
           break;
       case 'v':
           verbose++;
           break;
	/* weilun@adv - begin */
       case 'c':
	   calibrate++;
	   break;
	/* weilun@adv - end */
       default:
           opterr = 1;
           break;
       }
    }
}  /* options */

/*************************************************************************/

void usage( int err )
{
    printf( "\n%s\n\n", progname );
    printf( "    Usage:\n\n" );
    printf( "\t-u <nbr>   unit number, i.e. /dev/devi/touch0\n" );
    printf( "\t-v         verbose\n" );
    /* weilun@adv - begin */
    printf( "\t-c         calibrate\n\n" );
    /* weilun@adv - end */
    exit( err );
}  /* usage */
