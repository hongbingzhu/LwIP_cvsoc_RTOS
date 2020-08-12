/* ------------------------------------------------------------------------------------------------ */
/* FILE :		alt_eth_dma.h																		*/
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

#ifndef __ALT_ETH_DMA_H
#define __ALT_ETH_DMA_H

#include "alt_ethernet.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define u32 uint32_t
#define u16 uint32_t

// here and alt_ethernet.c, httpserver.c's BSP_IntVectSet() and BSP_IntSrcEn()
#define EMAC0_BASE									0xFF700000
#define EMAC1_BASE									0xFF702000
#define EMAC_BASE									EMAC0_BASE

#define EMAC_GMAC_INTERRUPT_STATUS					(*(volatile int *)(EMAC_BASE+0x38))
 #define INTERRUPT_STATUS_RGSMIIIS					(1<<0)
 #define INTERRUPT_STATUS_PCSLCHGIS					(1<<1)
 #define INTERRUPT_STATUS_PCSANCIS					(1<<2)
 #define INTERRUPT_STATUS_MMCIS						(1<<4)
 #define INTERRUPT_STATUS_MMCRXIS					(1<<5)
 #define INTERRUPT_STATUS_MMCTXIS					(1<<6)
 #define INTERRUPT_STATUS_MMCRXIPIS					(1<<7)
 #define INTERRUPT_STATUS_TSIS						(1<<9)
 #define INTERRUPT_STATUS_LPIIS						(1<<10)
#define EMAC_GMACS_GMII_RGMII_SMII_CONTROL_STATUS	(*(volatile int *)(EMAC_BASE+0xD8))
 #define LINK_MOD									(1<<0)
  #define LINK_MOD_HALFDUP							(0<<0)
  #define LINK_MOD_FULLDUP							(1<<0)
 #define LINK_SPEED_MASK							(3<<1)
  #define LINK_SPEED_SPEED2POINT5MHZ				(0<<1)
  #define LINK_SPEED_SPEED25MHZ						(1<<1)
  #define LINK_SPEED_SPEED125MHZ					(2<<1)
 #define LINK_STS									(1<<3)
  #define LINK_STS_LINKDOWN							(0<<3)
  #define LINK_STS_LINKUP							(1<<3)
#define EMAC_DMA_TRANSMIT_POLL_DEMAND				(*(volatile int *)(EMAC_BASE+0x1004))
#define EMAC_DMA_RECEIVE_POLL_DEMAND				(*(volatile int *)(EMAC_BASE+0x1008))
#define EMAC_DMA_RECEIVE_DESCRIPTOR_LIST_ADDRESS	(*(volatile int *)(EMAC_BASE+0x100C))
#define EMAC_DMA_TRANSMIT_DESCRIPTOR_LIST_ADDRESS	(*(volatile int *)(EMAC_BASE+0x1010))
#define EMAC_DMA_STATUS								(*(volatile int *)(EMAC_BASE+0x1014))
 #define DMA_STATUS_TI								(1<<0)
 #define DMA_STATUS_TPS								(1<<1)
 #define DMA_STATUS_TU								(1<<2)
 #define DMA_STATUS_TJT								(1<<3)
 #define DMA_STATUS_OVF								(1<<4)
 #define DMA_STATUS_UNF								(1<<5)
 #define DMA_STATUS_RI								(1<<6)
 #define DMA_STATUS_RU								(1<<7)
 #define DMA_STATUS_RPS								(1<<8)
 #define DMA_STATUS_RWT								(1<<9)
 #define DMA_STATUS_ETI								(1<<10)
 #define DMA_STATUS_FBI								(1<<13)
 #define DMA_STATUS_ERI								(1<<14)
 #define DMA_STATUS_AIS								(1<<15)
 #define DMA_STATUS_NIS								(1<<16)
 #define DMA_STATUS_RS_MASK							(7<<17)
 #define DMA_STATUS_TS_MASK							(7<<20)
 #define DMA_STATUS_EB_MASK							(7<<23)
 #define DMA_STATUS_GLI								(1<<26)
 #define DMA_STATUS_GMI								(1<<29)
 #define DMA_STATUS_GPLII							(1<<30)


#define EMAC_DMAclearPendIT(Mask)					do {										\
														EMAC_DMA_STATUS = (Mask);				\
													} while (EMAC_DMA_STATUS & (Mask))

#define ETH_DMATxDescChecksumInsertionConfig(Dsc, ChkSum) 	do {(Dsc)->Status |= (ChkSum); } while(0)


#define ETH_DMARxDesc_FrameLengthShift		16

#define ETH_MAC_Address0     ((uint32_t)0x00000000)
#define ETH_MAC_Address1     ((uint32_t)0x00000008)
#define ETH_MAC_Address2     ((uint32_t)0x00000010)
#define ETH_MAC_Address3     ((uint32_t)0x00000018)
							   /* TCP/UDP/ICMP checksum fully in hardware including pseudo header	*/
#define ETH_DMATxDesc_ChecksumTCPUDPICMPFull     ((uint32_t)0x00C00000)

#define ETH_MAC_ADDR_HBASE		(EMAC_BASE + 0x40)		/* ETHERNET MAC address high offset			*/
#define ETH_MAC_ADDR_LBASE		(EMAC_BASE + 0x44)		/* ETHERNET MAC address low offset			*/

#define  ETH_ERROR              ((uint32_t)0)
#define  ETH_SUCCESS            ((uint32_t)1)

/*--------------------------------------------------------------------------------------------------*/

typedef struct  {
  volatile uint32_t   Status;						/* Status											*/
  uint32_t   ControlBufferSize;					/* Control and Buffer1, Buffer2 lengths				*/
  uint32_t   Buffer1Addr;						/* Buffer1 address pointer							*/
  uint32_t   Buffer2NextDescAddr;				/* Buffer2 or next descriptor address pointer		*/
#ifdef USE_ENHANCED_DMA_DESCRIPTORS				/* Enhanced ETHERNET DMA PTP Descriptors			*/
  uint32_t   ExtendedStatus;					/* Extended status for PTP receive descriptor		*/
  uint32_t   Reserved1;							/* Reserved											*/
  uint32_t   TimeStampLow;						/* Time Stamp Low value for transmit and receive	*/
  uint32_t   TimeStampHigh;						/* Time Stamp High value for transmit and receive	*/
#endif
} ETH_DMADESCTypeDef;

typedef struct{
  u32 length;
  u32 buffer;
  volatile ETH_DMADESCTypeDef *descriptor;
}FrameTypeDef;

typedef struct  {
  volatile ETH_DMADESCTypeDef *FS_Rx_Desc;          /* First Segment Rx Desc */
  volatile ETH_DMADESCTypeDef *LS_Rx_Desc;          /* Last Segment Rx Desc */
  volatile uint32_t  Seg_Count;                     /* Segment count */
} ETH_DMA_Rx_Frame_infos;
  
/* ------------------------------------------------------------------------------------------------ */

void         ETH_MACAddressConfig                (uint32_t MacAddr, uint8_t *Addr);
FrameTypeDef ETH_Get_Received_Frame              (void);
FrameTypeDef ETH_Get_Received_Frame_interrupt    (void);
uint32_t     ETH_CheckFrameReceived              (void);
uint32_t     ETH_Prepare_Transmit_Descriptors    (u16 FrameLength);
void         ETH_DMARxDescChainInit              (ETH_DMADESCTypeDef *DMARxDescTab, uint8_t *RxBuff,
                                                  uint32_t RxBuffCount);
void         ETH_DMATxDescChainInit              (ETH_DMADESCTypeDef *DMATxDescTab, uint8_t* TxBuff,
                                                  uint32_t TxBuffCount);
void         ETH_DMARxDescReceiveITConfig        (ETH_DMADESCTypeDef *DMARxDesc,
                                                  FunctionalState NewState);
#ifdef __cplusplus
}
#endif

#endif
