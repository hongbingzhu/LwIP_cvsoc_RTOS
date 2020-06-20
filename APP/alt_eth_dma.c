/* ------------------------------------------------------------------------------------------------ */
/* FILE :		alt_eth_dma.c																		*/
/*																									*/
/* CONTENTS :																						*/
/*				Driver for the Ethernet DMA on the Cyclone V										*/
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

#include "alt_eth_dma.h"
#include <string.h>

#if defined (__CC_ARM)
  #define DMA_ATTRIB __attribute__((section(".uncached"))) __align(4)
  DMA_ATTRIB ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB];		/* Ethernet Rx MA Descriptor		*/
  DMA_ATTRIB ETH_DMADESCTypeDef  DMATxDscrTab[ETH_TXBUFNB];		/* Ethernet Tx DMA Descriptor		*/
  DMA_ATTRIB uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE];		/* Ethernet Receive Buffer			*/
  DMA_ATTRIB uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE];		/* Ethernet Transmit Buffer			*/
#elif defined (__GNUC__)
  #define DMA_ATTRIB __attribute__ ((aligned (4))) __attribute__ ((section (".uncached")))
  ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB] DMA_ATTRIB;		/* Ethernet Rx DMA Descriptor		*/
  ETH_DMADESCTypeDef  DMATxDscrTab[ETH_TXBUFNB] DMA_ATTRIB;		/* Ethernet Tx DMA Descriptor		*/
  uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] DMA_ATTRIB;		/* Ethernet Receive Buffer			*/
  uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] DMA_ATTRIB;		/* Ethernet Transmit Buffer			*/
#endif

volatile ETH_DMADESCTypeDef     *G_DMATxDescToSet;	/* Global TX & RX pointers to use				*/
volatile ETH_DMADESCTypeDef     *G_DMARxDescToGet;
         ETH_DMA_Rx_Frame_infos  g_RXframeDsc;
volatile ETH_DMA_Rx_Frame_infos *G_DMArxFrameInfo;

/* ------------------------------------------------------------------------------------------------ */
/* Configure the MAC address																		*/

void ETH_MACAddressConfig(uint32_t MacAddr, uint8_t *Addr)
{
	(*(volatile uint32_t *)(ETH_MAC_ADDR_HBASE+MacAddr)) = ((uint32_t)Addr[5] <<  8)
	                                                 |  (uint32_t)Addr[4];
	(*(volatile uint32_t *)(ETH_MAC_ADDR_LBASE+MacAddr)) = ((uint32_t)Addr[3] << 24)
	                                                 | ((uint32_t)Addr[2] << 16)
	                                                 | ((uint32_t)Addr[1] <<  8)
	                                                 |  (uint32_t)Addr[0];
	return;
}

/* ------------------------------------------------------------------------------------------------ */
/* Receive a frame (polling / not interrupt based)													*/

FrameTypeDef ETH_Get_Received_Frame(void)
{
uint32_t Len;
FrameTypeDef frame; 

												/* Length of the RXed packet						*/
	Len               = ((G_DMARxDescToGet->Status & ETH_DMARxDesc_FL)
	                    >> ETH_DMARxDesc_FrameLengthShift)
	                 - 4;						/* -4 is for the 4 CRC bytes						*/
	frame.length = Len;
												/* Get address, take into account multiple segments	*/
	frame.buffer     = (G_DMArxFrameInfo->Seg_Count > 1)
	                 ? (G_DMArxFrameInfo->FS_Rx_Desc)->Buffer1Addr
	                 :  G_DMARxDescToGet->Buffer1Addr;
	frame.descriptor = G_DMARxDescToGet;
												/* DMA will use the next desciptor					*/
	G_DMARxDescToGet = (ETH_DMADESCTypeDef*)(G_DMARxDescToGet->Buffer2NextDescAddr);    

	return (frame);  
}

/* ------------------------------------------------------------------------------------------------ */
/* Receive a frame (interrupt based / not polling)													*/

FrameTypeDef ETH_Get_Received_Frame_interrupt(void)
{
FrameTypeDef  Frame;
volatile uint32_t DscCnt; 

	DscCnt = 0;									/* Scan all the descriptors							*/
	while (((G_DMARxDescToGet->Status & ETH_DMARxDesc_OWN) == (uint32_t)RESET)
	&&     (DscCnt < ETH_RXBUFNB)) {
		DscCnt++;
												/* First one or middle one							*/
		if ((G_DMARxDescToGet->Status & ETH_DMARxDesc_LS) == (uint32_t)RESET) {
			if  ((G_DMARxDescToGet->Status & ETH_DMARxDesc_FS) != (uint32_t)RESET) {
				G_DMArxFrameInfo->FS_Rx_Desc = G_DMARxDescToGet;
				G_DMArxFrameInfo->Seg_Count  = 1;   
			}
			else {								/* Middle ones										*/
				(G_DMArxFrameInfo->Seg_Count) ++;
			}
			G_DMARxDescToGet = (ETH_DMADESCTypeDef*) (G_DMARxDescToGet->Buffer2NextDescAddr);
		}
		else {									/* Last one											*/
			G_DMArxFrameInfo->LS_Rx_Desc = G_DMARxDescToGet;
			(G_DMArxFrameInfo->Seg_Count)++;
			if ((G_DMArxFrameInfo->Seg_Count) == 1) {
				G_DMArxFrameInfo->FS_Rx_Desc = G_DMARxDescToGet;
			}
			Frame.length = ((G_DMARxDescToGet->Status & ETH_DMARxDesc_FL)
			                 >> ETH_DMARxDesc_FrameLengthShift)
			             - 4;
			Frame.buffer = (G_DMArxFrameInfo->Seg_Count > 1)
			             ? (G_DMArxFrameInfo->FS_Rx_Desc)->Buffer1Addr
			             : G_DMARxDescToGet->Buffer1Addr;
			Frame.descriptor = G_DMARxDescToGet;

			G_DMARxDescToGet = (ETH_DMADESCTypeDef*) (G_DMARxDescToGet->Buffer2NextDescAddr);

			return (Frame);  
		}
	}

	Frame.length     = 0;
	Frame.buffer     = (uint32_t)NULL;
	Frame.descriptor = NULL;

	return(Frame);
}

/* ------------------------------------------------------------------------------------------------ */
/* Prepare the transmission of an Ethernet frame													*/

uint32_t ETH_Prepare_Transmit_Descriptors(u16 FrameLength)
{
uint32_t Size;
volatile ETH_DMADESCTypeDef *DscLast;
volatile ETH_DMADESCTypeDef *DscNext;
  
	if ((G_DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (u32)RESET) {
		return ETH_ERROR;						/* OWN bit is set, DMA owns it, CPU can't use it	*/
	}
  
	if (FrameLength <= ETH_TX_BUF_SIZE) {		/* The frame fits in a single packet				*/
		G_DMATxDescToSet->Status |= ETH_DMATxDesc_FS		/* Set first segment					*/
		                         |  ETH_DMATxDesc_LS;		/* Set last segment						*/
		G_DMATxDescToSet->ControlBufferSize = (FrameLength& ETH_DMATxDesc_TBS1);
		G_DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;
		G_DMATxDescToSet          = (ETH_DMADESCTypeDef *)(G_DMATxDescToSet->Buffer2NextDescAddr);
	}
	else {										/* Multiple packets required to hold the frame		*/
		Size             = FrameLength;			/* Size is used to control the looping				*/
		DscNext          = G_DMATxDescToSet;
		DscNext->Status |= ETH_DMATxDesc_FS;	/* 1st segment, set the info						*/
		while (Size > ETH_TX_BUF_SIZE) {
			DscNext->ControlBufferSize = (ETH_TX_BUF_SIZE & ETH_DMATxDesc_TBS1);
			DscNext->Status       |= ETH_DMATxDesc_OWN;	/* DMA owns the descriptor now				*/
			DscLast                = DscNext;	/* Keep a copy to set last segment info				*/
			DscNext                = (ETH_DMADESCTypeDef *)(DscNext->Buffer2NextDescAddr);
			Size                  -= ETH_TX_BUF_SIZE;	/* This is what is left to transmit			*/
		}
		DscLast->Status           |= ETH_DMATxDesc_LS;		/* Set info for the last segment		*/
		DscLast->ControlBufferSize = (Size & ETH_DMATxDesc_TBS1);
		G_DMATxDescToSet           = DscNext;
	}

	if (EMAC_DMA_STATUS & DMA_STATUS_TU) {
		EMAC_DMAclearPendIT(DMA_STATUS_TU);				/* Clear TBUS Ethernet flag					*/
		EMAC_DMA_TRANSMIT_POLL_DEMAND = 0;				/* Resume TX								*/
	}

	return(ETH_SUCCESS);
}

/* ------------------------------------------------------------------------------------------------ */
/* Init the DMA descriptor chain																	*/

void ETH_DMARxDescChainInit(ETH_DMADESCTypeDef *DMARxDescTab, uint8_t *RxBuff, uint32_t RxBuffCount)
{
int ii;
ETH_DMADESCTypeDef *DMARxDesc;

	G_DMARxDescToGet = DMARxDescTab;			/* Start at first in the list						*/
	DMARxDesc        = DMARxDescTab;			/* Will traverse the array							*/
	for(ii=0 ; ii<RxBuffCount ; ii++) {			/* Put the value in all descriptors					*/
		DMARxDesc->Status              = ETH_DMARxDesc_OWN;
		DMARxDesc->ControlBufferSize   = ETH_DMARxDesc_RCH
		                               | (uint32_t)ETH_RX_BUF_SIZE;  
		DMARxDesc->Buffer1Addr         = (uint32_t)(&RxBuff[ii*ETH_RX_BUF_SIZE]);
		DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDesc+1);	/* Set next						*/
		DMARxDesc++;
	}
	DMARxDesc--;								/* Last descriptor, loop back: circular				*/
	DMARxDesc->Buffer2NextDescAddr = (uint32_t)DMARxDescTab;

	EMAC_DMA_RECEIVE_DESCRIPTOR_LIST_ADDRESS = (uint32_t) DMARxDescTab;
	G_DMArxFrameInfo                        = &g_RXframeDsc;

	return;
}

/* ------------------------------------------------------------------------------------------------ */
/* Poll for a RX frame																				*/
/* return:																							*/
/*        == 0 Not ready																			*/
/*        != 0 success																				*/

uint32_t ETH_CheckFrameReceived(void)
{
	if ((G_DMARxDescToGet->Status & ETH_DMARxDesc_OWN) == (uint32_t)RESET) {
												/* Last segment										*/
		if (((G_DMARxDescToGet->Status & ETH_DMARxDesc_LS)  != (uint32_t)RESET)) {
			G_DMArxFrameInfo->LS_Rx_Desc = G_DMARxDescToGet;
			G_DMArxFrameInfo->Seg_Count++;
			return(1);
		}										/* First segment									*/
		else if ((G_DMARxDescToGet->Status & ETH_DMARxDesc_FS)  != (uint32_t)RESET) {
			G_DMArxFrameInfo->FS_Rx_Desc = G_DMARxDescToGet;
			G_DMArxFrameInfo->LS_Rx_Desc = NULL;
			G_DMArxFrameInfo->Seg_Count  = 1;   
			G_DMARxDescToGet = (ETH_DMADESCTypeDef*)(G_DMARxDescToGet->Buffer2NextDescAddr);
		}
		else {									/* Not first, not last, in the middle				*/
			G_DMArxFrameInfo->Seg_Count++;
			G_DMARxDescToGet = (ETH_DMADESCTypeDef*)(G_DMARxDescToGet->Buffer2NextDescAddr);
		}
	}
	return(0);
}

/* ------------------------------------------------------------------------------------------------ */

void ETH_DMATxDescChainInit(ETH_DMADESCTypeDef *DMATxDescTab, uint8_t* TxBuff, uint32_t TxBuffCount)
{
int ii;
ETH_DMADESCTypeDef *DMATxDesc;

	G_DMATxDescToSet = DMATxDescTab;			/* Start at first in the list						*/
	DMATxDesc        = DMATxDescTab;			/* Will traverse the array							*/
	for(ii=0 ; ii<TxBuffCount ; ii++) {			/* Put the value in all descriptors					*/
		DMATxDesc->Status = ETH_DMATxDesc_TCH;	/* Set Second Address Chained bit					*/
												/* Set Buffer1 address pointer						*/
		DMATxDesc->Buffer1Addr         = (uint32_t)(&TxBuff[ii*ETH_TX_BUF_SIZE]);
												/* Set next											*/
		DMATxDesc->Buffer2NextDescAddr = (uint32_t)(DMATxDesc+1);
		DMATxDesc++;
	}
	DMATxDesc--;								/* Last descriptor, loop back: circular				*/
	DMATxDesc->Buffer2NextDescAddr = (uint32_t) DMATxDescTab;
												/* Set Transmit Desciptor List Address Register		*/
	EMAC_DMA_TRANSMIT_DESCRIPTOR_LIST_ADDRESS = (uint32_t)DMATxDescTab;

	return;
}

/* ------------------------------------------------------------------------------------------------ */
/* Enable / disable the DMA receive interrupt														*/

void ETH_DMARxDescReceiveITConfig(ETH_DMADESCTypeDef *DMARxDesc, FunctionalState NewState)
{
	if (NewState != DISABLE) {					/* Enable											*/
		DMARxDesc->ControlBufferSize &=(~(uint32_t)ETH_DMARxDesc_DIC);
	}
	else {										/* Disable											*/
		DMARxDesc->ControlBufferSize |= ETH_DMARxDesc_DIC;
	}
	return;
}

/* EOF */
