
#ifdef NBIOT_DEBUG
#include <utils.h>
#include <stdlib.h>
#include <platform.h>
#include "malloc.h"

static size_t s_total = 0;
static size_t s_last = 100;

void *nbiot_malloc( size_t size )
{
    size_t *ptr;

    ptr = (size_t*)mymalloc( sizeof( size_t ) + size );
    *ptr = size;
    s_total += size;
    if ( s_total > s_last )
    {
        nbiot_printf( "nbiot_malloc() %dbytes memories.\n", (int)s_total );
        s_last += 100;
    }

    return (++ptr);
}

void nbiot_free( void *ptr )
{
    if ( NULL != ptr )
    {
        size_t *tmp;

        tmp = (size_t*)ptr;
        s_total -= *(--tmp);
        myfree( tmp );
    }
}
#else
#include <stdlib.h>
#include <platform.h>
#include "malloc.h"

void *nbiot_malloc( size_t size )
{
    return mymalloc( size );
}

void nbiot_free( void *ptr )
{
    myfree( ptr );
}
#endif
