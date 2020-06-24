#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__

#include "lwip/err.h"
#include "lwip/netif.h"

#ifdef __cplusplus
 extern "C" {
#endif

err_t ethernetif_init(struct netif *netif);
void  ethernetif_input(void *Arg);

#ifdef __cplusplus
}
#endif

#endif

/* EOF */
