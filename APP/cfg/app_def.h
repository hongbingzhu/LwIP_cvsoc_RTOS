#ifndef __APP_DEF_H__
#define __APP_DEF_H__

#include "lwipopts.h"

#if NO_SYS

#define PRINTF(...)		printf(__VA_ARGS__)

#define OSTimeDly(ms)  do { \
	u32_t start = G_IPnetTime; \
	while (G_IPnetTime - start < ms) ; \
} while (0)

#define OSTimeGet()		G_IPnetTime

#else

extern int TracePrintf(const char * fmt, ...);
#define PRINTF(...)		TracePrintf(__VA_ARGS__)

#endif

#endif // __APP_DEF_H__
