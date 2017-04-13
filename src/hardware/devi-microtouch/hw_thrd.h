#ifndef HW_THRD_H_
#define HW_THRD_H_

#include <semaphore.h>

#define	HIGH_PRIO	22		// must be higher than set by screen (default 21)

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

extern sem_t rd_sem;
extern char *progname;
extern int verbose;

// hw_thrd writes to buffer and sets g_dataBufferLen
extern char *g_dataBuffer;
extern int g_dataBufferLen;

extern void *hw_thrd( void *p );

#endif /* HW_THRD_H_ */
