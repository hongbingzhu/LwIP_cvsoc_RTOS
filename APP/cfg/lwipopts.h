#ifndef __LWIPOPTS_WRAPPER_H__
#define __LWIPOPTS_WRAPPER_H__

#if LWIP_VER == 1
#include "lwipopts1.h"
#elif LWIP_VER == 2
#include "lwipopts2.h"
#endif

extern int TracePuts(const char * msg, int len);
#define PUTS(msg)	TracePuts(msg, strlen(msg));

extern int TracePrintf(const char * fmt, ...);
#define PRINTF(...)		TracePrintf(__VA_ARGS__)
// non-fatal, print a message.
#define LWIP_PLATFORM_DIAG(x)                     do { PRINTF x; PRINTF("\r\n"); } while(0)

extern void sys_assert(char *msg);	// in sys_arch.c
// fatal, print message and abandon execution.
#define LWIP_PLATFORM_ASSERT(x)                   sys_assert( x )

#endif
/* EOF */
