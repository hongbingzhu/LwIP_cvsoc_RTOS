#include "lwipopts.h"

#if !NO_SYS

#if defined(__ABASSI_H__) || defined(__MABASSI_H__) || defined(__UABASSI_H__)
  #include "../../Abassi/sys_arch.h"
#endif

//#include "os_cpu.h"
#include "os.h"
#include "../../uCOS-II/sys_arch.h"

#else

static inline void OSTimeDly(uint32_t ms)
{
	u32_t start = G_IPnetTime;
	while (G_IPnetTime - start < ms) ;
}

static inline uint32_t OSTimeGet(void)
{
	return G_IPnetTime;
}

#endif

