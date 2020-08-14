/* ------------------------------------------------------------------------------------------------ */
/* FILE :		NetAddr.c																			*/
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

#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/dhcp.h"
#include "lwip/tcpip.h"
#include "alt_ethernet.h"
#include "ethernetif.h"
#include "WebServer.h"	// hardware interface, for LED.
#include "NetAddr.h"

/* ------------------------------------------------------------------------------------------------ */

#define MAX_DHCP_TRIES		10						/* Number of time to try until declared timeout	*/

#define DHCP_START			0						/* States of the DHCP process					*/
#define DHCP_WAIT_ADDRESS	1
#define DHCP_GOT_ADDRESS	2
#define DHCP_TIMEOUT		2
#define DHCP_DONE			3

/* ------------------------------------------------------------------------------------------------ */

u32_t G_IPnetDefGW;								/* Default static default gateway address		*/
u32_t G_IPnetDefIP;								/* Default static default IP address			*/
u32_t G_IPnetDefNM;								/* Default net mask								*/
int   G_IPnetStatic = 0;						/* If using static or dynamic IP address		*/
volatile u32_t G_IPnetTime=0;

static u32_t ARPTimer       __attribute__((unused));
static int   DHCPstate      __attribute__((unused));
static u32_t DHCPtimeCoarse __attribute__((unused));
static u32_t DHCPtimeFine   __attribute__((unused));
static u32_t TCPTimer       __attribute__((unused));

static struct netif g_NetIF;						/* lwIP network interface descriptor			*/

/* ------------------------------------------------------------------------------------------------ */

void GetMACaddr(u8_t MACaddr[NETIF_MAX_HWADDR_LEN])
{
	MACaddr[0]  = MAC_ADDR0;
	MACaddr[1]  = MAC_ADDR1;
	MACaddr[2]  = MAC_ADDR2;
	MACaddr[3]  = MAC_ADDR3;
	MACaddr[4]  = MAC_ADDR4;
	MACaddr[5]  = MAC_ADDR5;

	return;
}

/* ------------------------------------------------------------------------------------------------ */

#if LWIP_VER == 1

void LwIP_Init(void)
{
	ip_addr_t IPaddr;
	ip_addr_t NetMask;
	ip_addr_t GateWay;

	G_IPnetTime     = 0;

	ARPTimer        = 0;
	DHCPstate       = DHCP_START;
	DHCPtimeCoarse  = 0;
	DHCPtimeFine    = 0;
	TCPTimer        = 0;

	mem_init();										/* Init dynamic memory (size is MEM_SIZE)		*/
	memp_init();									/* Init the mempry pools (number is MEMP_NUM_x)	*/

	if (G_IPnetStatic == 0) {						/* Request for dynamic address					*/
		IPaddr.addr  = 0;
		NetMask.addr = 0;
		GateWay.addr = 0;
	}
	else {											/* Request for static address					*/
		IPaddr.addr  = G_IPnetDefIP;
		NetMask.addr = G_IPnetDefNM;
		GateWay.addr = G_IPnetDefGW;

		PRINTF("Static IP address : %s\n", ip4_ntop(G_IPnetDefIP));
		PRINTF("The webserver is ready\n");
		PRINTF("The Ethernet interface is ready\n");

		GPIO_SET(LED_0, 0);							/* Turn ON LED #0								*/
	}

	netif_add(&g_NetIF, &IPaddr, &NetMask, &GateWay, NULL, &ethernetif_init, &ethernet_input);

	netif_set_default(&g_NetIF);					/* This is the default network interface		*/

	netif_set_up(&g_NetIF);							/* Doc states this must be called				*/

  #if !NO_SYS
	if (G_IPnetStatic == 0) dhcp_start(&g_NetIF);
  #endif
}

#if NO_SYS
void LwIP_Packet(void)
{
	ethernetif_input(&g_NetIF);
	return;
}
#endif

void DHCPprocess(void);
void LwIP_Periodic(volatile u32_t Time)
{
	if ((Time-TCPTimer) >= TCP_TMR_INTERVAL) {		/* TCP is processed periodicaly					*/
		TCPTimer =  Time;
		tcp_tmr();
	}
  
	if ((Time-ARPTimer) >= ARP_TMR_INTERVAL) {		/* ARP is processed periodicaly					*/
		ARPTimer =  Time;
		etharp_tmr();
	}

	if (G_IPnetStatic == 0) {						/* DHCP processing								*/
		if ((Time-DHCPtimeFine) >= DHCP_FINE_TIMER_MSECS) {
			DHCPtimeFine =  Time;
			dhcp_fine_tmr();
			if ((DHCPstate != DHCP_GOT_ADDRESS)
			&&  (DHCPstate != DHCP_TIMEOUT)) {
				DHCPprocess();						/* DHCP state machine							*/
			}										/* Located in this file							*/
		}
		if ((Time-DHCPtimeCoarse) >= DHCP_COARSE_TIMER_MSECS) {
			DHCPtimeCoarse =  Time;					/* Coarse DHCP processing						*/
			dhcp_coarse_tmr();
		}
  	}
}

void DHCPprocess(void)
{
	ip_addr_t IPaddr;
	u32_t     DynamicIP;
	ip_addr_t GateWay;
	ip_addr_t NetMask;

	switch (DHCPstate) {
		case DHCP_START:
			dhcp_start(&g_NetIF);
			DynamicIP = 0;
			DHCPstate = DHCP_WAIT_ADDRESS;
			break;

		case DHCP_WAIT_ADDRESS:
			//GPIO_SET(LED_0, Toggle);
			//if (Toggle == 0) {						/* Blink the "waiting for DHCP" message			*/
			//	PRINTF("Waiting for DHCP server\r");
			//}
			//else {
			//	PRINTF("                       \r");
			//}
			//Toggle = (Toggle+1) & 1;

			DynamicIP = g_NetIF.ip_addr.addr;		/* Grab the current address						*/
			if (DynamicIP != 0) {					/* If the address is not 0 anymore, DHCP server	*/
				DHCPstate = DHCP_GOT_ADDRESS;		/* gave us our address							*/
				dhcp_stop(&g_NetIF);				/* We can stop the DCHP client					*/

				PRINTF("Dynamic IP address : %s\n", ip4_ntop(DynamicIP));
				PRINTF("The Ethernet interface is ready\n");
				GPIO_SET(LED_0, 0);				/* Keep LED #0 ON								*/
				DHCPstate = DHCP_DONE;
			}
			else {									/* Check for timeout: too long wait for reply	*/
				if (g_NetIF.dhcp->tries > MAX_DHCP_TRIES)	{
					DHCPstate = DHCP_TIMEOUT;		/* We are done, fall back to static address		*/
					dhcp_stop(&g_NetIF);			/* We can stop the DCHP client					*/

					IP4_ADDR(&IPaddr,  IP_ADDR0,      IP_ADDR1,      IP_ADDR2,      IP_ADDR3 );
					IP4_ADDR(&NetMask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
					IP4_ADDR(&GateWay, GATEWAY_ADDR0, GATEWAY_ADDR1, GATEWAY_ADDR2, GATEWAY_ADDR3);

					netif_set_addr(&g_NetIF, &IPaddr , &NetMask, &GateWay);

					PRINTF("DHCP timeout, fallback to static address : %d.%d.%d.%d\n",
					                        IP_ADDR0 ,IP_ADDR1 , IP_ADDR2 , IP_ADDR3);
					PRINTF("The Ethernet interface is ready\n");
					GPIO_SET(LED_0, 0);				/* Keep LED #0 ON								*/
					dhcp_stop(&g_NetIF);			/* After stop, we can ask again for an address	*/

					DHCPstate = DHCP_DONE;
				}
			}
			break;

		case DHCP_DONE:
			break;

		default:
			break;
	}
}

#elif LWIP_VER == 2

void LwIP_Init(void)
{
	ip_addr_t IPaddr;
	ip_addr_t NetMask;
	ip_addr_t GateWay;

	G_IPnetTime     = 0;

	mem_init();										/* Init dynamic memory (size is MEM_SIZE)		*/
	memp_init();									/* Init the mempry pools (number is MEMP_NUM_x)	*/

	tcpip_init(NULL, NULL);

	if (G_IPnetStatic == 0) {						/* Request for dynamic address					*/
		ip_addr_set_zero_ip4(&IPaddr);
		ip_addr_set_zero_ip4(&NetMask);
		ip_addr_set_zero_ip4(&GateWay);
	}
	else {											/* Request for static address					*/
		ip4_addr_set_u32(&IPaddr,  G_IPnetDefIP);
		ip4_addr_set_u32(&NetMask, G_IPnetDefNM);
		ip4_addr_set_u32(&GateWay, G_IPnetDefGW);

		PRINTF("Static IP address : %s\n", ip4_ntop(G_IPnetDefIP));
		PRINTF("The webserver is ready\n");
		PRINTF("The Ethernet interface is ready\n");

		GPIO_SET(LED_0, 0);							/* Turn ON LED #0								*/
	}

	netif_add(&g_NetIF, &IPaddr, &NetMask, &GateWay, NULL, &ethernetif_init, &tcpip_input);

	netif_set_default(&g_NetIF);					/* This is the default network interface		*/

	if (netif_is_link_up(&g_NetIF))
	{
		/* When the netif is fully configured this function must be called */
		netif_set_up(&g_NetIF);
	}
	else
	{
		/* When the netif link is down this function must be called */
		netif_set_down(&g_NetIF);
	}

	if (G_IPnetStatic == 0)
	{
		/*  Creates a new DHCP client for this interface on the first call.
		    Note: you must call dhcp_fine_tmr() and dhcp_coarse_tmr() at
		    the predefined regular intervals after starting the client.
		    You can peek in the netif->dhcp struct for the actual DHCP status.*/
		int cnt = 0;
		int err = dhcp_start(&g_NetIF);
		PRINTF("LwIP DHCP init %s.\n", (err == ERR_OK) ? "success" : "fail");
		/*PRINTF("Waiting DHCP ready ...");
		while (ip_addr_cmp(&(g_NetIF.ip_addr), &IPaddr))
		{
			OSTimeDly(1);
			if (cnt++ % 1000 == 0) PUTS(".");
		}
		PRINTF("\nDynamic IP address : %s\n", ip4_ntop(g_NetIF.ip_addr.addr));
		PRINTF("The Ethernet interface is ready\n"); */
	}
}

#endif

/* ------------------------------------------------------------------------------------------------ */

/* ------------------------------------------------------------------------------------------------ */

const char * ip4_ntop(u32_t Addr)
{
	static char buff[20];
	u8_t *Baddr;

	Baddr = (u8_t *)&Addr;
  #if BYTE_ORDER == BIG_ENDIAN
	snprintf(buff, sizeof(buff), "%d.%d.%d.%d", Baddr[3], Baddr[2], Baddr[1], Baddr[0]);
  #else
	snprintf(buff, sizeof(buff), "%d.%d.%d.%d", Baddr[0], Baddr[1], Baddr[2], Baddr[3]);
  #endif
	return buff;
}

// Under baremetal, Update system tick, main loop will call LwIP_Periodic()
void Time_Update(void)
{
	G_IPnetTime += SYSTICK_MS;	// when no OS
}

#include "arch/sys_arch.h"

// Under uC/OS, use time tick hook to call LwIP_Periodic()
void App_TimeTickHook(void)
{
	G_IPnetTime = OSTimeGet();
	if (g_NetIF.flags & NETIF_FLAG_UP)	// if input function assigned, lwip should start work
	{
#if LWIP_VER == 1
		LwIP_Periodic(G_IPnetTime);
#elif LWIP_VER == 2
		sys_check_timeouts();
#endif
	}
}

u32_t sys_now(void)
{
	return OSTimeGet();
}

/* EOF */

