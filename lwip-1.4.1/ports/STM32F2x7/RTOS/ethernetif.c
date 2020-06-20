/**
 * @file
 * Ethernet Interface Skeleton
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

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "netif/etharp.h"
#include "err.h"
#include "ethernetif.h"
#include "mac.h"
#include "stm32f2x7_eth.h"
#include "stm32f2x7_eth_bsp.h"

#ifndef NETIF_MTU
  #define NETIF_MTU					1500		/* Maximum transfer unit							*/
#endif
#ifndef NETIF_TASK_STACK_SIZE
  #define NETIF_TASK_STACK_SIZE		1024
#endif
#ifndef NETIF_TASK_PRIORITY
  #define NETIF_TASK_PRIORITY		1
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

static struct netif *s_pxNetIf = NULL;
static sys_sem_t     EthRXsem  = NULL;
												/* Ethernet RX and Tx DMA Descriptors				*/
extern ETH_DMADESCTypeDef      DMARxDscrTab[ETH_RXBUFNB], DMATxDscrTab[ETH_TXBUFNB];
												/* Ethernet RX and TX  buffers						*/
extern uint8_t                 Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE];
extern uint8_t                 Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE];
extern ETH_DMADESCTypeDef     *DMATxDescToSet;	/* Pointer tracking current TX descriptors			*/
extern ETH_DMADESCTypeDef     *DMARxDescToGet;	/* Pointer tracking current RX descriptors			*/
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;/* Global pointer for last received frame infos	*/

static void ethernetif_input(void);

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
uint32_t ii;

	netif->hwaddr_len = ETHARP_HWADDR_LEN;		/* Set the netif MAC hardware address length		*/
	netif->hwaddr[0]  = MAC_ADDR0;				/* Set the netif MAC hardware address				*/
	netif->hwaddr[1]  = MAC_ADDR1;
	netif->hwaddr[2]  = MAC_ADDR2;
	netif->hwaddr[3]  = MAC_ADDR3;
	netif->hwaddr[4]  = MAC_ADDR4;
	netif->hwaddr[5]  = MAC_ADDR5;
    netif->mtu        = NETIF_MTU;				/* Set the netif maximum transfer unit				*/
	netif->flags      = NETIF_FLAG_BROADCAST	/* Accept broadcast address and ARP traffic			*/
	                  | NETIF_FLAG_ETHARP;
	s_pxNetIf         = netif;

	if (EthRXsem == NULL) {						/* Create semaphore ethernetif_input() blocks on	*/
		(void)sys_sem_new(&EthRXsem, 0);
	}

	ETH_MACAddressConfig(ETH_MAC_Address0, netif->hwaddr);	/* I MAC address in ethernet MAC		*/
  	ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);/* Init Tx Desc list: Chain	*/
	ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);/* Init Rx Desc list: Chain	*/

	for(ii=0 ; ii<ETH_RXBUFNB ; ii++) {			/* Enable Ethernet Rx interrrupt					*/
		ETH_DMARxDescReceiveITConfig(&DMARxDscrTab[ii], ENABLE);
	}

  #ifdef CHECKSUM_BY_HARDWARE
	for(ii=0 ; ii<ETH_TXBUFNB ; ii++) {			/* Enable the checksum insertion for the Tx frames	*/
		ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[ii], ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
	}
  #endif
  												/* Create the task that handles the ETH_MAC			*/
	TSKcreate("Ethernet I/F", NETIF_TASK_PRIORITY, NETIF_TASK_STACK_SIZE, ethernetif_input,  1);

	ETH_Start();								/* Enable MAC and DMA TX and RX						*/

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
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
u8_t        *DMAbuf;
struct pbuf *InBuf8;
u32_t        Nbytes;

static sys_mutex_t MyMutex = NULL;				/* To protect against multi-threading				*/

	netif = netif;								/* To remove compiler warning						*/

	if (MyMutex == (sys_mutex_t)NULL) {			/* If the mutex hasn't been allocated				*/
		if (ERR_MEM == sys_mutex_new(&MyMutex)) {
		  #if ((SYS_LIGHTWEIGHT_PROT) != 0)
			sys_arch_protect();					/* No more mutex available, trap the error			*/
		  #endif
			for(;;);
		}
	}

	sys_mutex_lock(&MyMutex);

	DMAbuf = (u8_t *)(DMATxDescToSet->Buffer1Addr);
	Nbytes = 0;
	for(InBuf8=p ; InBuf8!=NULL ; InBuf8=InBuf8->next) {
		memcpy((u8_t*)&DMAbuf[Nbytes], InBuf8->payload, InBuf8->len);
		Nbytes += InBuf8->len;
   	}
	ETH_Prepare_Transmit_Descriptors(Nbytes);

	sys_mutex_unlock(&MyMutex);

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

__IO ETH_DMADESCTypeDef *DMARxNextDesc;

	netif   = netif;							/* To remove compiler warning						*/
	BufDst  = NULL;
	RetSize = 0;
	Frame   = ETH_Get_Received_Frame_interrupt();	/* Grab the received frame						*/
												/* Make sure there is no error						*/
	if ((Frame.descriptor->Status & ETH_DMARxDesc_ES) == (uint32_t)RESET) {
		Len  = (int)Frame.length;				/* Size of the packet								*/
		Buf8 = (u8_t *)Frame.buffer;			/* Pointer to the packet							*/

	    BufDst = pbuf_alloc(PBUF_RAW, Len, PBUF_POOL);	/* Get the buffer to return from the pool	*/
 	    if (BufDst != NULL) { 					/* Copy RX frame (linked list) in dst buffer		*/
			for (Bptr=BufDst ; Bptr!=NULL ; Bptr=Bptr->next) {
				memcpy((u8_t*)Bptr->payload, (u8_t*)&Buf8[RetSize], Bptr->len);
				RetSize += Bptr->len;			/* Adjust the write index in destination buffer		*/
			}
		}
	}

	if (DMA_RX_FRAME_infos->Seg_Count > 1) {	/* Release the descritor back to the DMA			*/
		DMARxNextDesc = DMA_RX_FRAME_infos->FS_Rx_Desc;
	}
	else {
		DMARxNextDesc = Frame.descriptor;
	}

	for (ii=0; ii<DMA_RX_FRAME_infos->Seg_Count; ii++) {	/* Give buffers back to the DMA			*/
		DMARxNextDesc->Status = ETH_DMARxDesc_OWN;
		DMARxNextDesc = (ETH_DMADESCTypeDef *)(DMARxNextDesc->Buffer2NextDescAddr);
	}
	DMA_RX_FRAME_infos->Seg_Count =0;			/* Clear the segment count							*/
  	if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET) {	/* When RX unqavail flag set, resume RX		*/
		ETH->DMASR = ETH_DMASR_RBUS;			/* Clear the RBUS ETHERNET DMA flag					*/
		ETH->DMARPDR = 0;						/* Resume the DMA reception							*/
	}

	return(BufDst);
}

/*--------------------------------------------------------------------------------------------------*/
/**
 * This function is the ethernetif_input task, it is processed when a packet 
 * is ready to be read from the interface. It uses the function low_level_input() 
 * that should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void ethernetif_input(void)
{
struct pbuf *p;
  
	for(;;) {
		if (SYS_ARCH_TIMEOUT != sys_arch_sem_wait(&EthRXsem, 0)) { //EMAC_BLOCK_TIME)) {
			p = low_level_input(s_pxNetIf);
			if (ERR_OK != s_pxNetIf->input(p, s_pxNetIf)) {
				pbuf_free(p);
				p = (struct pbuf *)NULL;
			}
		}
	}
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

	etharp_init();
	etharp_tmr();

	return(ERR_OK);
}

/*--------------------------------------------------------------------------------------------------*/
/**
  * @brief  This function handles External line 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(ETH_LINK_EXTI_LINE) != RESET) {
		Eth_Link_ITHandler(DP83848_PHY_ADDRESS);
		EXTI_ClearITPendingBit(ETH_LINK_EXTI_LINE);	/* Clear interrupt pending bit					*/
	}

	return;
}

/*--------------------------------------------------------------------------------------------------*/
/**
  * @brief  This function handles ethernet DMA interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void)
{
	if (ETH_GetDMAFlagStatus(ETH_DMA_FLAG_R) == SET) {  /* A new frame was received					*/
		SEMpost(EthRXsem);						/* Post the semaphore ethernetif_input() blocks on	*/
	}

	ETH_DMAClearITPendingBit(ETH_DMA_IT_R);  	/* Clear the interrupt flags						*/
	ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);	/* Clear the Eth DMA Rx IT pending bits				*/

	return;
}

/* EOF */
