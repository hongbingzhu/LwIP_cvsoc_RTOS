/**
 * @file
 * Ethernet Interface for standalone applications (without RTOS) - works only for 
 * ethernet polling mode (polling for ethernet frame reception)
 *
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#include "lwip/mem.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "alt_eth_dma.h"
#include <string.h>

#include "app_def.h"

#if NO_SYS

#ifndef NETIF_MTU
  #define NETIF_MTU					1500		/* Maximum transfer unit							*/
#endif
#ifndef MY_HOSTNAME
  #define MY_HOSTNAME				"lwIP"
#endif
#ifndef IFNAME0
  #define IFNAME0 					's'			/* Define those to better describe your network I/F	*/
#endif
#ifndef IFNAME1
  #define IFNAME1 					't'
#endif

#if ETH_PAD_SIZE
  #define DROP_PAD(x)	pbuf_header((x), -ETH_PAD_SIZE);	/* Drop the padding word				*/
  #define CLAIM_PAD(x)	pbuf_header((x),  ETH_PAD_SIZE);	/* Reclaim the padding word				*/
#else
  #define DROP_PAD(x)
  #define CLAIM_PAD(x)
#endif
extern ETH_DMADESCTypeDef      DMARxDscrTab[ETH_RXBUFNB], DMATxDscrTab[ETH_TXBUFNB];
												/* Ethernet RX and TX  buffers						*/
extern uint8_t                 Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE];
extern uint8_t                 Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE];
extern ETH_DMADESCTypeDef     *DMARxDescToGet;	/* Pointer tracking current RX descriptors			*/
extern ETH_DMADESCTypeDef     *G_DMATxDescToSet;/* Pointer tracking current TX descriptors			*/
extern ETH_DMA_Rx_Frame_infos *G_DMArxFrameInfo;/* Global pointer for last received frame infos	*/

void GetMACaddr(u8_t MACaddr[NETIF_MAX_HWADDR_LEN]);

/*--------------------------------------------------------------------------------------------------*/
/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{
#ifdef CHECKSUM_BY_HARDWARE
  int ii; 
#endif

	netif->hwaddr_len = ETHARP_HWADDR_LEN;		/* Set the netif MAC hardware address length		*/
	GetMACaddr(&netif->hwaddr[0]);				/* Use a fct to allow reading from dardware			*/
	netif->mtu        = NETIF_MTU;				/* Set the netif maximum transfer unit				*/
	netif->flags      = NETIF_FLAG_BROADCAST	/* Accept broadcast address and ARP traffic			*/
	                  | NETIF_FLAG_ETHARP
	                  | NETIF_FLAG_LINK_UP;

	ETH_MACAddressConfig(ETH_MAC_Address0, netif->hwaddr);	/* I MAC address in ethernet MAC		*/
	ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);/* Init Tx Desc list: Chain	*/
	ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);/* Init Rx Desc list: Chain	*/

  #ifdef CHECKSUM_BY_HARDWARE
	for(ii=0 ; ii<ETH_TXBUFNB ; ii++) {			/* Enable the checksum insertion for the Tx frames	*/
		ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[ii], ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
	}
  #endif

	/* Note: TCP, UDP, ICMP checksum checking for received frame are enabled in DMA config */

	alt_eth_start();							/* Enable MAC and DMA TX and RX						*/

	return;
}

/*--------------------------------------------------------------------------------------------------*/
/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
u8_t        *DMAbuf;
struct pbuf *InBuf8;
u32_t        Nbytes;

	netif  = netif;								/* To remove compiler warning						*/
	DMAbuf = (u8_t *)(G_DMATxDescToSet->Buffer1Addr);
	Nbytes = 0;

	DROP_PAD(p);								/* Drop the padding word if needed					*/
	for(InBuf8=p ; InBuf8!=NULL ; InBuf8=InBuf8->next) {
		SMEMCPY((u8_t*)&DMAbuf[Nbytes], InBuf8->payload, InBuf8->len);
		Nbytes += InBuf8->len;
	}
	ETH_Prepare_Transmit_Descriptors(Nbytes);

	CLAIM_PAD(p);								/* Reclaim the padding word if needed				*/

	return(ERR_OK);
}

/*--------------------------------------------------------------------------------------------------*/
/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf * low_level_input(struct netif *netif)
{
struct pbuf *Bptr;
u8_t        *Buf8;
struct pbuf *BufDst;
FrameTypeDef Frame;
int          ii;
int          Len;
int          RetSize;

volatile  ETH_DMADESCTypeDef *DMARxNextDesc;

	netif   = netif;						/* To remove compiler warning						*/
	BufDst  = NULL;
	RetSize = 0;
	Frame   = ETH_Get_Received_Frame();		/* Grab the received frame							*/
	Len     = (int)Frame.length;			/* Size of the packet								*/

  #if ETH_PAD_SIZE
	Len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
  #endif

	Buf8    = (u8_t *)Frame.buffer;			/* Pointer to the packet							*/
	BufDst  = pbuf_alloc(PBUF_RAW, Len, PBUF_POOL);	/* Get the buffer to return from the pool	*/
	if (BufDst != NULL) { 					/* Copy RX frame (linked list) in dst buffer		*/
		DROP_PAD(BufDst);					/* Drop the padding word if needed					*/

		for (Bptr=BufDst ; Bptr!=NULL ; Bptr=Bptr->next) {
			SMEMCPY((u8_t*)Bptr->payload, (u8_t*)&Buf8[RetSize], Bptr->len);
			RetSize += Bptr->len;			/* Adjust the write index in destination buffer		*/
		}

		CLAIM_PAD(BufDst);					/* Reclaim the padding word if needed				*/

	}
	if (G_DMArxFrameInfo->Seg_Count > 1) {/* Release the descriptor back to the DMA			*/
		DMARxNextDesc = G_DMArxFrameInfo->FS_Rx_Desc;
	}
	else {
		DMARxNextDesc = Frame.descriptor;
	}

	for (ii=0; ii<G_DMArxFrameInfo->Seg_Count; ii++) {	/* Give the buffers back to the DMA		*/
		DMARxNextDesc->Status = ETH_DMARxDesc_OWN;
		DMARxNextDesc = (ETH_DMADESCTypeDef *)(DMARxNextDesc->Buffer2NextDescAddr);
	}
	G_DMArxFrameInfo->Seg_Count =0;			/* Clear the segment count							*/

	if (EMAC_DMA_STATUS & DMA_STATUS_RU) {	/* When the RX unvailable flag is set, resume RX	*/
		EMAC_DMAclearPendIT(DMA_STATUS_RU);	/* Clear the RX unvailable flag						*/
		EMAC_DMA_RECEIVE_POLL_DEMAND = 0;	/* And resume the DMA reception						*/
	}

	return(BufDst);
}

/*--------------------------------------------------------------------------------------------------*/
/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
err_t ethernetif_input(struct netif *netif)
{
err_t Err;
struct pbuf *p;

	Err = ERR_MEM;								/* Assume no pbuf available							*/
	p   = low_level_input(netif);				/* Move received packet into a new pbuf				*/
	if (p != (struct pbuf *)NULL) {				/* OK, we got a valid buffer						*/
		Err = netif->input(p, netif);
		if (Err != ERR_OK) {
			LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
			pbuf_free(p);
		}
	}

	return(Err);
}

/*--------------------------------------------------------------------------------------------------*/
/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
	LWIP_ASSERT("netif != NULL", (netif != NULL));

  #if LWIP_NETIF_HOSTNAME
	netif->hostname   = MY_HOSTNAME;			/* Initialize interface hostname					*/
  #endif
	netif->name[0]    = IFNAME0;
	netif->name[1]    = IFNAME1;
	netif->output     = etharp_output;
	netif->linkoutput = low_level_output;

	low_level_init(netif);						/* Initialize the hardware							*/

	return(ERR_OK);
}

#endif // NO_SYS
/* EOF */
