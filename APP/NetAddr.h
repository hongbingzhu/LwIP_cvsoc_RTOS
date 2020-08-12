/* ------------------------------------------------------------------------------------------------ */
/* FILE :		NetAddr.h																			*/
/*																									*/
/* CONTENTS :																						*/
/*				lwIP network address configuration & DHCP client task								*/
/*				DHCP client task																	*/
/*																									*/
/*																									*/
/* Copyright (c) 2013-2014, Code-Time Technologies Inc. All rights reserved.						*/
/*																									*/
/* Code-Time Technologies retains all right, title, and interest in and to this work				*/
/*																									*/
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS							*/
/* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF										*/
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL							*/
/* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR								*/
/* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,							*/
/* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR							*/
/* OTHER DEALINGS IN THE SOFTWARE.																	*/
/*																									*/
/*																									*/
/*	$Revision: 1.8 $																				*/
/*	$Date: 2013/10/03 15:55:16 $																	*/
/*																									*/
/* ------------------------------------------------------------------------------------------------ */

#ifndef __NETADDR_H
#define __NETADDR_H

#include "ethernetif.h"
#include "lwip/debug.h"
#include "lwip/dhcp.h"
#include "lwip/ip_addr.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#if LWIP_VER == 1
#include "lwip/tcp_impl.h"
#elif LWIP_VER ==2
#include "lwip/timeouts.h"
#endif
#include "lwip/udp.h"
#include "netif/etharp.h"
#include <stdio.h>
#include <stdint.h>


/* ------------------------------------------------------------------------------------------------ */

#if BYTE_ORDER == BIG_ENDIAN
 #define IP4_INT32(a,b,c,d) (((a)<<24)|((b)<<16)|((c)<<8)|(d))
#else
 #define IP4_INT32(a,b,c,d) (((d)<<24)|((c)<<16)|((b)<<8)|(a))
#endif

#define IP_ADDR0   192								/* IP Address									*/
#define IP_ADDR1   168								/* Lands in G_IPnetDefIP located in the file	*/
#define IP_ADDR2   68								/* NetAddr.c									*/
#define IP_ADDR3   10								/* Currently set in main()						*/

#define NETMASK_ADDR0   255							/* Net Mask										*/
#define NETMASK_ADDR1   255							/* Lands in G_IPnetDefNM located in the file	*/
#define NETMASK_ADDR2   255							/* NetAddr.c									*/
#define NETMASK_ADDR3   0							/* Currently set in main()						*/

#define GATEWAY_ADDR0   192							/* Gateway address								*/
#define GATEWAY_ADDR1   168							/* Lands in G_IPnetDefGW located in the file	*/
#define GATEWAY_ADDR2   68							/* NetAddr.c									*/
#define GATEWAY_ADDR3   1  							/* Currently set in main()						*/

#define MAC_ADDR0   0xFE							/* MAC address									*/
#define MAC_ADDR1   0x23							/* If available from hardware, modify the		*/
#define MAC_ADDR2   0x34							/* the function GetMACaddr() located in the		*/
#define MAC_ADDR3   0x45							/* file NetAddr.c								*/
#define MAC_ADDR4   0x56
#define MAC_ADDR5   0x67

#define MII_MODE

/* ------------------------------------------------------------------------------------------------ */

void httpd_ssi_init(void);
void httpd_cgi_init(void);
void LwIP_Init(void);
void LwIP_Packet(void);
void LwIP_Periodic(volatile u32_t localtime);

const char * ip4_ntop(u32_t);
#define SYSTICK_MS  10		// timer interval for baremetal mode
void Time_Update(void);		// Timer ISR for baremetal mode

extern u32_t          G_IPnetDefGW;
extern u32_t          G_IPnetDefIP;
extern u32_t          G_IPnetDefNM;
extern int            G_IPnetStatic;
extern volatile u32_t G_IPnetTime;

#ifdef __cplusplus
}
#endif

#endif

/* EOF */
