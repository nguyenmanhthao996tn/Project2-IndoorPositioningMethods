#ifndef __POS_DATA_TYPE__
#define __POS_DATA_TYPE__

/* LIBRARIES */
#include <time.h>

/* STRUCT */
typedef struct PosDataType_Tag {
    char *data;
    struct timespec timeReceived;
} PosDataType;
typedef struct PosDataType_Tag * PosDataType_p;

/* METHODS */


#endif /* __POS_DATA_TYPE__ */
