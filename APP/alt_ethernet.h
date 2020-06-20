/*! \file
 *  Contains definitions for the Altera Hardware Libraries Ethernet
 *  Application Programming Interface
 */

/******************************************************************************
*
* Copyright 2013 Altera Corporation. All Rights Reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* 
* 3. The name of the author may not be used to endorse or promote products
* derived from this software without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ARE DISCLAIMED. IN NO
* EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
* 
******************************************************************************/

#ifndef __ALT_ETHERNET_H__
#define __ALT_ETHERNET_H__

#include "hwlib.h"
#include "socal/hps.h"
#include "socal/alt_emac.h"


#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */


typedef enum {RESET = 0, SET = !RESET} alt_eth_flag_status_t, alt_eth_irq_status_t, FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} alt_eth_functional_state_t, FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Uncomment the line below when using time stamping and/or IPv4 checksum offload */
#define USE_ENHANCED_DMA_DESCRIPTORS

/* Uncomment the line below to allow custom configuration of the Ethernet driver buffers */    
#define CUSTOM_DRIVER_BUFFERS_CONFIG   

#ifdef  CUSTOM_DRIVER_BUFFERS_CONFIG
/* Redefinition of the Ethernet driver buffers size and count */   
 #define ETH_RX_BUF_SIZE    ETH_MAX_PACKET_SIZE  /* buffer size for receive */
 #define ETH_TX_BUF_SIZE    ETH_MAX_PACKET_SIZE  /* buffer size for transmit */
 #define ETH_RXBUFNB        4                   /* 4 Rx buffers of size ETH_RX_BUF_SIZE */
 #define ETH_TXBUFNB        4                   /* 4  Tx buffers of size ETH_TX_BUF_SIZE */
#endif


 
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */  


/**--------------------------------------------------------------------------**/
/** 
  * @brief                           DMA descriptors types
  */ 
/**--------------------------------------------------------------------------**/

struct ALT_EMAC_DMA_TDESC0_s
{
    uint32_t        db    :  1;  /* Deferred Byte */
    uint32_t        uf    :  1;  /* Underflow Error */
    uint32_t        ed    :  1;  /* Excessive Defferal */
    uint32_t        cc    :  4;  /* Collision Count */
    uint32_t        vf    :  1;  /* VLAN Frame */
    uint32_t        ec    :  1;  /* Excessive Collision */
    uint32_t        lcol  :  1;  /* Late Collision */
    uint32_t        nc    :  1;  /* No Carrier */
    uint32_t        lc    :  1;  /* Loss of Carrier */
    uint32_t        ipe   :  1;  /* IP Payload Error */
    uint32_t        ff    :  1;  /* Frame Flushed */
    uint32_t        jt    :  1;  /* Jabber Timeout */
    uint32_t        es    :  1;  /* Error Summary */
    uint32_t        ihe   :  1;  /* IP Header Error */
    uint32_t        ttss  :  1;  /* Transmit timestamp Status */
    uint32_t        vlic  :  2;  /* VLAN Insertion Control */
    uint32_t        tch   :  1;  /* Second Address Chained */
    uint32_t        ter   :  1;  /* Transmit end of Ring */
    uint32_t        cic   :  2;  /* Checksum Insertion Control */
    uint32_t        crcr  :  1;  /* CRC Replacement Control */
    uint32_t        ttse  :  1;  /* Transmit Timestamp Enable */
    uint32_t        dp    :  1;  /* Disable Pad */
    uint32_t        dc    :  1;  /* Disable CRC */
    uint32_t        fs    :  1;  /* First Segment */
    uint32_t        ls    :  1;  /* Last Segment */
    uint32_t        ic    :  1;  /* Interrupt Completion */
    uint32_t        own   :  1;  /* Own Bit */
};

/* The typedef declaration for register ALT_EMAC_DMA_STAT. */
typedef volatile struct ALT_EMAC_DMA_TDESC0_s  ALT_EMAC_DMA_TDESC0_t;


/** 
  * @brief  ETH DMA Descriptors data structure definition
  */ 
typedef struct  {
  volatile uint32_t   Status;                /*!< Status */
  uint32_t   ControlBufferSize;     /*!< Control and Buffer1, Buffer2 lengths */
  uint32_t   Buffer1Addr;           /*!< Buffer1 address pointer */
  uint32_t   Buffer2NextDescAddr;   /*!< Buffer2 or next descriptor address pointer */
/* Enhanced ETHERNET DMA PTP Descriptors */
#ifdef USE_ENHANCED_DMA_DESCRIPTORS
  uint32_t   ExtendedStatus;        /* Extended status for PTP receive descriptor */
  uint32_t   Reserved1;             /* Reserved */
  uint32_t   TimeStampLow;          /* Time Stamp Low value for transmit and receive */
  uint32_t   TimeStampHigh;         /* Time Stamp High value for transmit and receive */
#endif /* USE_ENHANCED_DMA_DESCRIPTORS */
} alt_eth_dma_descriptor_t;

#if 0
typedef struct{
  uint32_t length;
  uint32_t buffer;
  volatile alt_eth_dma_descriptor_t *descriptor;
}FrameTypeDef;


typedef struct  {
  volatile alt_eth_dma_descriptor_t *FS_Rx_Desc;          /*!< First Segment Rx Desc */
  volatile alt_eth_dma_descriptor_t *LS_Rx_Desc;          /*!< Last Segment Rx Desc */
  volatile uint32_t  Seg_Count;                     /*!< Segment count */
} ETH_DMA_Rx_Frame_infos;
#endif  


 
/**--------------------------------------------------------------------------**/
/*                   Ethernet Frames defines								  */
/**--------------------------------------------------------------------------**/


/** @defgroup ENET_Buffers_setting 
  * @{
  */ 
#define ETH_MAX_PACKET_SIZE    1524    /*!< ETH_HEADER + ETH_EXTRA + VLAN_TAG + MAX_ETH_PAYLOAD + ETH_CRC */
#define ETH_HEADER               14    /*!< 6 byte Dest addr, 6 byte Src addr, 2 byte length/type */
#define ETH_CRC                   4    /*!< Ethernet CRC */
#define ETH_EXTRA                 2    /*!< Extra bytes in some cases */   
#define VLAN_TAG                  4    /*!< optional 802.1q VLAN Tag */
#define MIN_ETH_PAYLOAD          46    /*!< Minimum Ethernet payload size */
#define MAX_ETH_PAYLOAD        1500    /*!< Maximum Ethernet payload size */
#define JUMBO_FRAME_PAYLOAD    9000    /*!< Jumbo frame payload size */      


#ifdef USE_ENHANCED_DMA_DESCRIPTORS
/**--------------------------------------------------------------------------**/
/*				Ethernet DMA descriptors registers bits definition            */ 
/**--------------------------------------------------------------------------**/

/**
   DMA Tx Desciptor
  -----------------------------------------------------------------------------------------------
  TDES0 | OWN(31) | CTRL[30:26] | Reserved[25:24] | CTRL[23:20] | Reserved[19:17] | Status[16:0] |
  -----------------------------------------------------------------------------------------------
  TDES1 | Reserved[31:29] | Buffer2 ByteCount[28:16] | Reserved[15:13] | Buffer1 ByteCount[12:0] |
  -----------------------------------------------------------------------------------------------
  TDES2 |                         Buffer1 Address [31:0]                                         |
  -----------------------------------------------------------------------------------------------
  TDES3 |                   Buffer2 Address [31:0] / Next Descriptor Address [31:0]              |
  -----------------------------------------------------------------------------------------------
*/

/** 
  * @brief  Bit definition of TDES0 register: DMA Tx descriptor status register
  */ 
#define ETH_DMATxDesc_OWN                     ((uint32_t)0x80000000)  /*!< OWN bit: descriptor is owned by DMA engine */
#define ETH_DMATxDesc_IC                      ((uint32_t)0x40000000)  /*!< Interrupt on Completion */
#define ETH_DMATxDesc_LS                      ((uint32_t)0x20000000)  /*!< Last Segment */
#define ETH_DMATxDesc_FS                      ((uint32_t)0x10000000)  /*!< First Segment */
#define ETH_DMATxDesc_DC                      ((uint32_t)0x08000000)  /*!< Disable CRC */
#define ETH_DMATxDesc_DP                      ((uint32_t)0x04000000)  /*!< Disable Padding */
#define ETH_DMATxDesc_TTSE                    ((uint32_t)0x02000000)  /*!< Transmit Time Stamp Enable */
#define ETH_DMATxDesc_CIC                     ((uint32_t)0x00C00000)  /*!< Checksum Insertion Control: 4 cases */
#define ETH_DMATxDesc_CIC_ByPass              ((uint32_t)0x00000000)  /*!< Do Nothing: Checksum Engine is bypassed */ 
#define ETH_DMATxDesc_CIC_IPV4Header          ((uint32_t)0x00400000)  /*!< IPV4 header Checksum Insertion */ 
#define ETH_DMATxDesc_CIC_TCPUDPICMP_Segment  ((uint32_t)0x00800000)  /*!< TCP/UDP/ICMP Checksum Insertion calculated over segment only */ 
#define ETH_DMATxDesc_CIC_TCPUDPICMP_Full     ((uint32_t)0x00C00000)  /*!< TCP/UDP/ICMP Checksum Insertion fully calculated */ 
#define ETH_DMATxDesc_TER                     ((uint32_t)0x00200000)  /*!< Transmit End of Ring */
#define ETH_DMATxDesc_TCH                     ((uint32_t)0x00100000)  /*!< Second Address Chained */
#define ETH_DMATxDesc_TTSS                    ((uint32_t)0x00020000)  /*!< Tx Time Stamp Status */
#define ETH_DMATxDesc_IHE                     ((uint32_t)0x00010000)  /*!< IP Header Error */
#define ETH_DMATxDesc_ES                      ((uint32_t)0x00008000)  /*!< Error summary: OR of the following bits: UE || ED || EC || LCO || NC || LCA || FF || JT */
#define ETH_DMATxDesc_JT                      ((uint32_t)0x00004000)  /*!< Jabber Timeout */
#define ETH_DMATxDesc_FF                      ((uint32_t)0x00002000)  /*!< Frame Flushed: DMA/MTL flushed the frame due to SW flush */
#define ETH_DMATxDesc_PCE                     ((uint32_t)0x00001000)  /*!< Payload Checksum Error */
#define ETH_DMATxDesc_LCA                     ((uint32_t)0x00000800)  /*!< Loss of Carrier: carrier lost during transmission */
#define ETH_DMATxDesc_NC                      ((uint32_t)0x00000400)  /*!< No Carrier: no carrier signal from the transceiver */
#define ETH_DMATxDesc_LCO                     ((uint32_t)0x00000200)  /*!< Late Collision: transmission aborted due to collision */
#define ETH_DMATxDesc_EC                      ((uint32_t)0x00000100)  /*!< Excessive Collision: transmission aborted after 16 collisions */
#define ETH_DMATxDesc_VF                      ((uint32_t)0x00000080)  /*!< VLAN Frame */
#define ETH_DMATxDesc_CC                      ((uint32_t)0x00000078)  /*!< Collision Count */
#define ETH_DMATxDesc_ED                      ((uint32_t)0x00000004)  /*!< Excessive Deferral */
#define ETH_DMATxDesc_UF                      ((uint32_t)0x00000002)  /*!< Underflow Error: late data arrival from the memory */
#define ETH_DMATxDesc_DB                      ((uint32_t)0x00000001)  /*!< Deferred Bit */


/*					Bit definition of TDES1 register  			*/ 
#define ETH_DMATxDesc_TBS2  ((uint32_t)0x1FFF0000)  /*!< Transmit Buffer2 Size */
#define ETH_DMATxDesc_TBS1  ((uint32_t)0x00001FFF)  /*!< Transmit Buffer1 Size */

/*					Bit definition of TDES2 register  			*/ 
#define ETH_DMATxDesc_B1AP  ((uint32_t)0xFFFFFFFF)  /*!< Buffer1 Address Pointer */

/*					Bit definition of TDES3 register  			*/ 
#define ETH_DMATxDesc_B2AP  ((uint32_t)0xFFFFFFFF)  /*!< Buffer2 Address Pointer */

  /*---------------------------------------------------------------------------------------------
  TDES6 |                         Transmit Time Stamp Low [31:0]                                 |
  -----------------------------------------------------------------------------------------------
  TDES7 |                         Transmit Time Stamp High [31:0]                                |
  ----------------------------------------------------------------------------------------------*/

/* Bit definition of TDES6 register */
 #define ETH_DMAPTPTxDesc_TTSL  ((uint32_t)0xFFFFFFFF)  /* Transmit Time Stamp Low */

/* Bit definition of TDES7 register */
 #define ETH_DMAPTPTxDesc_TTSH  ((uint32_t)0xFFFFFFFF)  /* Transmit Time Stamp High */

/**
  DMA Rx Descriptor
  --------------------------------------------------------------------------------------------------------------------
  RDES0 | OWN(31) |                                             Status [30:0]                                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES1 | CTRL(31) | Reserved[30:29] | Buffer2 ByteCount[28:16] | CTRL[15:14] | Reserved(13) | Buffer1 ByteCount[12:0] |
  ---------------------------------------------------------------------------------------------------------------------
  RDES2 |                                       Buffer1 Address [31:0]                                                 |
  ---------------------------------------------------------------------------------------------------------------------
  RDES3 |                          Buffer2 Address [31:0] / Next Descriptor Address [31:0]                             |
  ---------------------------------------------------------------------------------------------------------------------
*/

/*	Bit definition of RDES0 register: DMA Rx descriptor status register  */ 
#define ETH_DMARxDesc_OWN         ((uint32_t)0x80000000)  /*!< OWN bit: descriptor is owned by DMA engine  */
#define ETH_DMARxDesc_AFM         ((uint32_t)0x40000000)  /*!< DA Filter Fail for the rx frame  */
#define ETH_DMARxDesc_FL          ((uint32_t)0x3FFF0000)  /*!< Receive descriptor frame length  */
#define ETH_DMARxDesc_ES          ((uint32_t)0x00008000)  /*!< Error summary: OR of the following bits: DE || OE || IPC || LC || RWT || RE || CE */
#define ETH_DMARxDesc_DE          ((uint32_t)0x00004000)  /*!< Descriptor error: no more descriptors for receive frame  */
#define ETH_DMARxDesc_SAF         ((uint32_t)0x00002000)  /*!< SA Filter Fail for the received frame */
#define ETH_DMARxDesc_LE          ((uint32_t)0x00001000)  /*!< Frame size not matching with length field */
#define ETH_DMARxDesc_OE          ((uint32_t)0x00000800)  /*!< Overflow Error: Frame was damaged due to buffer overflow */
#define ETH_DMARxDesc_VLAN        ((uint32_t)0x00000400)  /*!< VLAN Tag: received frame is a VLAN frame */
#define ETH_DMARxDesc_FS          ((uint32_t)0x00000200)  /*!< First descriptor of the frame  */
#define ETH_DMARxDesc_LS          ((uint32_t)0x00000100)  /*!< Last descriptor of the frame  */ 
#define ETH_DMARxDesc_IPV4HCE     ((uint32_t)0x00000080)  /*!< IPC Checksum Error: Rx Ipv4 header checksum error   */    
#define ETH_DMARxDesc_LC          ((uint32_t)0x00000040)  /*!< Late collision occurred during reception   */
#define ETH_DMARxDesc_FT          ((uint32_t)0x00000020)  /*!< Frame type - Ethernet, otherwise 802.3    */
#define ETH_DMARxDesc_RWT         ((uint32_t)0x00000010)  /*!< Receive Watchdog Timeout: watchdog timer expired during reception    */
#define ETH_DMARxDesc_RE          ((uint32_t)0x00000008)  /*!< Receive error: error reported by MII interface  */
#define ETH_DMARxDesc_DBE         ((uint32_t)0x00000004)  /*!< Dribble bit error: frame contains non int multiple of 8 bits  */
#define ETH_DMARxDesc_CE          ((uint32_t)0x00000002)  /*!< CRC error */
#define ETH_DMARxDesc_MAMPCE      ((uint32_t)0x00000001)  /*!< Rx MAC Address/Payload Checksum Error: Rx MAC address matched/ Rx Payload Checksum Error */

/*		Bit definition of RDES1 register  			*/ 
#define ETH_DMARxDesc_DIC   ((uint32_t)0x80000000)  /*!< Disable Interrupt on Completion */
#define ETH_DMARxDesc_RBS2  ((uint32_t)0x1FFF0000)  /*!< Receive Buffer2 Size */
#define ETH_DMARxDesc_RER   ((uint32_t)0x00008000)  /*!< Receive End of Ring */
#define ETH_DMARxDesc_RCH   ((uint32_t)0x00004000)  /*!< Second Address Chained */
#define ETH_DMARxDesc_RBS1  ((uint32_t)0x00001FFF)  /*!< Receive Buffer1 Size */

/*		Bit definition of RDES2 register    		*/ 
#define ETH_DMARxDesc_B1AP  ((uint32_t)0xFFFFFFFF)  /*!< Buffer1 Address Pointer */

/*		Bit definition of RDES3 register    		*/ 
#define ETH_DMARxDesc_B2AP  ((uint32_t)0xFFFFFFFF)  /*!< Buffer2 Address Pointer */

/*---------------------------------------------------------------------------------------------------------------------
  RDES4 |                   Reserved[31:15]              |             Extended Status [14:0]                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES5 |                                            Reserved[31:0]                                                    |
  ---------------------------------------------------------------------------------------------------------------------
  RDES6 |                                       Receive Time Stamp Low [31:0]                                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES7 |                                       Receive Time Stamp High [31:0]                                         |
  --------------------------------------------------------------------------------------------------------------------*/

/* Bit definition of RDES4 register */
#define ETH_DMAPTPRxDesc_PTPV     ((uint32_t)0x00002000)  /* PTP Version */
#define ETH_DMAPTPRxDesc_PTPFT    ((uint32_t)0x00001000)  /* PTP Frame Type */
#define ETH_DMAPTPRxDesc_PTPMT    ((uint32_t)0x00000F00)  /* PTP Message Type */
  #define ETH_DMAPTPRxDesc_PTPMT_Sync                      ((uint32_t)0x00000100)  /* SYNC message (all clock types) */
  #define ETH_DMAPTPRxDesc_PTPMT_FollowUp                  ((uint32_t)0x00000200)  /* FollowUp message (all clock types) */ 
  #define ETH_DMAPTPRxDesc_PTPMT_DelayReq                  ((uint32_t)0x00000300)  /* DelayReq message (all clock types) */ 
  #define ETH_DMAPTPRxDesc_PTPMT_DelayResp                 ((uint32_t)0x00000400)  /* DelayResp message (all clock types) */ 
  #define ETH_DMAPTPRxDesc_PTPMT_PdelayReq_Announce        ((uint32_t)0x00000500)  /* PdelayReq message (peer-to-peer transparent clock) or Announce message (Ordinary or Boundary clock) */ 
  #define ETH_DMAPTPRxDesc_PTPMT_PdelayResp_Manag          ((uint32_t)0x00000600)  /* PdelayResp message (peer-to-peer transparent clock) or Management message (Ordinary or Boundary clock)  */ 
  #define ETH_DMAPTPRxDesc_PTPMT_PdelayRespFollowUp_Signal ((uint32_t)0x00000700)  /* PdelayRespFollowUp message (peer-to-peer transparent clock) or Signaling message (Ordinary or Boundary clock) */           
#define ETH_DMAPTPRxDesc_IPV6PR   ((uint32_t)0x00000080)  /* IPv6 Packet Received */
#define ETH_DMAPTPRxDesc_IPV4PR   ((uint32_t)0x00000040)  /* IPv4 Packet Received */
#define ETH_DMAPTPRxDesc_IPCB  ((uint32_t)0x00000020)  /* IP Checksum Bypassed */
#define ETH_DMAPTPRxDesc_IPPE  ((uint32_t)0x00000010)  /* IP Payload Error */
#define ETH_DMAPTPRxDesc_IPHE  ((uint32_t)0x00000008)  /* IP Header Error */
#define ETH_DMAPTPRxDesc_IPPT  ((uint32_t)0x00000007)  /* IP Payload Type */
  #define ETH_DMAPTPRxDesc_IPPT_UDP                 ((uint32_t)0x00000001)  /* UDP payload encapsulated in the IP datagram */
  #define ETH_DMAPTPRxDesc_IPPT_TCP                 ((uint32_t)0x00000002)  /* TCP payload encapsulated in the IP datagram */ 
  #define ETH_DMAPTPRxDesc_IPPT_ICMP                ((uint32_t)0x00000003)  /* ICMP payload encapsulated in the IP datagram */

/* Bit definition of RDES6 register */
#define ETH_DMAPTPRxDesc_RTSL  ((uint32_t)0xFFFFFFFF)  /* Receive Time Stamp Low */

/* Bit definition of RDES7 register */
#define ETH_DMAPTPRxDesc_RTSH  ((uint32_t)0xFFFFFFFF)  /* Receive Time Stamp High */


#else       /**------------Non-Enhanced Descriptors-------------------------**/



/**--------------------------------------------------------------------------**/
/*		Ethernet DMA descriptors registers bits definition  				  */ 
/**--------------------------------------------------------------------------**/

/**
   DMA Tx Desciptor
  -----------------------------------------------------------------------------------------------
  TDES0 | OWN(31) | CTRL[30:26] | Reserved[25:24] | CTRL[23:20] | Reserved[19:17] | Status[16:0] |
  -----------------------------------------------------------------------------------------------
  TDES1 | Reserved[31:29] | Buffer2 ByteCount[28:16] | Reserved[15:13] | Buffer1 ByteCount[12:0] |
  -----------------------------------------------------------------------------------------------
  TDES2 |                         Buffer1 Address [31:0]                                         |
  -----------------------------------------------------------------------------------------------
  TDES3 |                   Buffer2 Address [31:0] / Next Descriptor Address [31:0]              |
  -----------------------------------------------------------------------------------------------
*/

/*		Bit definition of TDES0 register: DMA Tx descriptor status register  */ 
#define ETH_DMATxDesc_OWN                     ((uint32_t)0x80000000)  /*!< OWN bit: descriptor is owned by DMA engine */
// Status bits.
#define ETH_DMATxDesc_TTSS                    ((uint32_t)0x00020000)  /*!< Tx Time Stamp Status */
#define ETH_DMATxDesc_IHE                     ((uint32_t)0x00010000)  /*!< IP Header Error */
#define ETH_DMATxDesc_ES                      ((uint32_t)0x00008000)  /*!< Error summary: OR of the following bits: UE || ED || EC || LCO || NC || LCA || FF || JT */
#define ETH_DMATxDesc_JT                      ((uint32_t)0x00004000)  /*!< Jabber Timeout */
#define ETH_DMATxDesc_FF                      ((uint32_t)0x00002000)  /*!< Frame Flushed: DMA/MTL flushed the frame due to SW flush */
#define ETH_DMATxDesc_PCE                     ((uint32_t)0x00001000)  /*!< Payload Checksum Error */
#define ETH_DMATxDesc_LCA                     ((uint32_t)0x00000800)  /*!< Loss of Carrier: carrier lost during transmission */
#define ETH_DMATxDesc_NC                      ((uint32_t)0x00000400)  /*!< No Carrier: no carrier signal from the transceiver */
#define ETH_DMATxDesc_LCO                     ((uint32_t)0x00000200)  /*!< Late Collision: transmission aborted due to collision */
#define ETH_DMATxDesc_EC                      ((uint32_t)0x00000100)  /*!< Excessive Collision: transmission aborted after 16 collisions */
#define ETH_DMATxDesc_VF                      ((uint32_t)0x00000080)  /*!< VLAN Frame */
#define ETH_DMATxDesc_CC                      ((uint32_t)0x00000078)  /*!< Collision Count */
#define ETH_DMATxDesc_ED                      ((uint32_t)0x00000004)  /*!< Excessive Deferral */
#define ETH_DMATxDesc_UF                      ((uint32_t)0x00000002)  /*!< Underflow Error: late data arrival from the memory */
#define ETH_DMATxDesc_DB                      ((uint32_t)0x00000001)  /*!< Deferred Bit */


/*	Bit definition of TDES1 register: DMA Tx descriptor control-size register  */ 
#define ETH_DMATxDesc_IC                      ((uint32_t)0x80000000)  /*!< Interrupt on Completion */
#define ETH_DMATxDesc_LS                      ((uint32_t)0x40000000)  /*!< Last Segment */
#define ETH_DMATxDesc_FS                      ((uint32_t)0x20000000)  /*!< First Segment */


#define ETH_DMATxDesc_CIC                     ((uint32_t)0x18000000)  /*!< Checksum Insertion Control: 4 cases */
#define ETH_DMATxDesc_CIC_ByPass              ((uint32_t)0x00000000)  /*!< Do Nothing: Checksum Engine is bypassed */ 
#define ETH_DMATxDesc_CIC_IPV4Header          ((uint32_t)0x08000000)  /*!< IPV4 header Checksum Insertion */ 
#define ETH_DMATxDesc_CIC_TCPUDPICMP_Segment  ((uint32_t)0x10000000)  /*!< TCP/UDP/ICMP Checksum Insertion calculated over segment only */ 
#define ETH_DMATxDesc_CIC_TCPUDPICMP_Full     ((uint32_t)0x18000000)  /*!< TCP/UDP/ICMP Checksum Insertion fully calculated */ 


#define ETH_DMATxDesc_DC                      ((uint32_t)0x04000000)  /*!< Disable CRC */
#define ETH_DMATxDesc_TER                     ((uint32_t)0x02000000)  /*!< Transmit End of Ring */
#define ETH_DMATxDesc_TCH                     ((uint32_t)0x01000000)  /*!< Second Address Chained */
#define ETH_DMATxDesc_DP                      ((uint32_t)0x00800000)  /*!< Disable Padding */
#define ETH_DMATxDesc_TTSE                    ((uint32_t)0x00400000)  /*!< Transmit Time Stamp Enable */

#define ETH_DMATxDesc_TBS2                    ((uint32_t)0x003FF800)  /*!< Transmit Buffer2 Size */
#define ETH_DMATxDesc_TBS1                    ((uint32_t)0x000007FF)  /*!< Transmit Buffer1 Size */

/*		Bit definition of TDES2 register  		*/ 
#define ETH_DMATxDesc_B1AP  ((uint32_t)0xFFFFFFFF)  /*!< Buffer1 Address Pointer */

/*		Bit definition of TDES3 register  		*/ 
#define ETH_DMATxDesc_B2AP  ((uint32_t)0xFFFFFFFF)  /*!< Buffer2 Address Pointer */


/**
  DMA Rx Descriptor
  --------------------------------------------------------------------------------------------------------------------
  RDES0 | OWN(31) |                                             Status [30:0]                                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES1 | CTRL(31) | Reserved[30:29] | Buffer2 ByteCount[28:16] | CTRL[15:14] | Reserved(13) | Buffer1 ByteCount[12:0] |
  ---------------------------------------------------------------------------------------------------------------------
  RDES2 |                                       Buffer1 Address [31:0]                                                 |
  ---------------------------------------------------------------------------------------------------------------------
  RDES3 |                          Buffer2 Address [31:0] / Next Descriptor Address [31:0]                             |
  ---------------------------------------------------------------------------------------------------------------------
*/

/*	Bit definition of RDES0 register: DMA Rx descriptor status register  */ 
#define ETH_DMARxDesc_OWN         ((uint32_t)0x80000000)  /*!< OWN bit: descriptor is owned by DMA engine  */
#define ETH_DMARxDesc_AFM         ((uint32_t)0x40000000)  /*!< DA Filter Fail for the rx frame  */
#define ETH_DMARxDesc_FL          ((uint32_t)0x3FFF0000)  /*!< Receive descriptor frame length  */
#define ETH_DMARxDesc_ES          ((uint32_t)0x00008000)  /*!< Error summary: OR of the following bits: DE || OE || IPC || LC || RWT || RE || CE */
#define ETH_DMARxDesc_DE          ((uint32_t)0x00004000)  /*!< Descriptor error: no more descriptors for receive frame  */
#define ETH_DMARxDesc_SAF         ((uint32_t)0x00002000)  /*!< SA Filter Fail for the received frame */
#define ETH_DMARxDesc_LE          ((uint32_t)0x00001000)  /*!< Frame size not matching with length field */
#define ETH_DMARxDesc_OE          ((uint32_t)0x00000800)  /*!< Overflow Error: Frame was damaged due to buffer overflow */
#define ETH_DMARxDesc_VLAN        ((uint32_t)0x00000400)  /*!< VLAN Tag: received frame is a VLAN frame */
#define ETH_DMARxDesc_FS          ((uint32_t)0x00000200)  /*!< First descriptor of the frame  */
#define ETH_DMARxDesc_LS          ((uint32_t)0x00000100)  /*!< Last descriptor of the frame  */ 
#define ETH_DMARxDesc_IPV4HCE     ((uint32_t)0x00000080)  /*!< IPC Checksum Error: Rx Ipv4 header checksum error   */    
#define ETH_DMARxDesc_LC          ((uint32_t)0x00000040)  /*!< Late collision occurred during reception   */
#define ETH_DMARxDesc_FT          ((uint32_t)0x00000020)  /*!< Frame type - Ethernet, otherwise 802.3    */
#define ETH_DMARxDesc_RWT         ((uint32_t)0x00000010)  /*!< Receive Watchdog Timeout: watchdog timer expired during reception    */
#define ETH_DMARxDesc_RE          ((uint32_t)0x00000008)  /*!< Receive error: error reported by MII interface  */
#define ETH_DMARxDesc_DBE         ((uint32_t)0x00000004)  /*!< Dribble bit error: frame contains non int multiple of 8 bits  */
#define ETH_DMARxDesc_CE          ((uint32_t)0x00000002)  /*!< CRC error */
#define ETH_DMARxDesc_MAMPCE      ((uint32_t)0x00000001)  /*!< Rx MAC Address/Payload Checksum Error: Rx MAC address matched/ Rx Payload Checksum Error */

/*	Bit definition of RDES1 register			  */ 
#define ETH_DMARxDesc_DIC   ((uint32_t)0x80000000)  /*!< Disable Interrupt on Completion */

#define ETH_DMARxDesc_RER   ((uint32_t)0x02000000)  /*!< Receive End of Ring */
#define ETH_DMARxDesc_RCH   ((uint32_t)0x01000000)  /*!< Second Address Chained */

#define ETH_DMARxDesc_RBS2  ((uint32_t)0x003FF800)  /*!< Receive Buffer2 Size */
#define ETH_DMARxDesc_RBS1  ((uint32_t)0x000007FF)  /*!< Receive Buffer1 Size */


/*		Bit definition of RDES2 register    	*/ 
#define ETH_DMARxDesc_B1AP  ((uint32_t)0xFFFFFFFF)  /*!< Buffer1 Address Pointer */

/*		 Bit definition of RDES3 register  		*/ 
#define ETH_DMARxDesc_B2AP  ((uint32_t)0xFFFFFFFF)  /*!< Buffer2 Address Pointer */


#endif           //#ifdef USE_ENHANCED_DMA_DESCRIPTORS


#ifdef USE_ENHANCED_DMA_DESCRIPTORS

    /* ETHERNET DMA Tx descriptors Buffer2 Size Shift */
    #define  ETH_DMATXDESC_BUFFER2_SIZESHIFT           16
    
    /* ETHERNET DMA Rx descriptors Frame Length Shift */
    #define  ETH_DMARXDESC_FRAME_LENGTHSHIFT           16
    
    /* ETHERNET DMA Rx descriptors Buffer2 Size Shift */
    #define  ETH_DMARXDESC_BUFFER2_SIZESHIFT           16

#else

    /* ETHERNET DMA Tx descriptors Buffer2 Size Shift */
    #define  ETH_DMATXDESC_BUFFER2_SIZESHIFT           11
    
    /* ETHERNET DMA Rx descriptors Frame Length Shift */
    #define  ETH_DMARXDESC_FRAME_LENGTHSHIFT           16
    
    /* ETHERNET DMA Rx descriptors Buffer2 Size Shift */
    #define  ETH_DMARXDESC_BUFFER2_SIZESHIFT           11

#endif


/**--------------------------------------------------------------------------**/
/*			Description of common PHY registers  							  */ 
/**--------------------------------------------------------------------------**/



/* PHY_Read_write_Timeouts   */ 
#define PHY_READ_TO                     ((uint32_t)0x0004FFFF)
#define PHY_WRITE_TO                    ((uint32_t)0x0004FFFF)


/*		PHY_Register_address */ 
#define PHY_BCR                          0          /*!< Transceiver Basic Control Register */
#define PHY_BSR                          1          /*!< Transceiver Basic Status Register */

#define PHY_ID1                          2          /*!< PHY Identifier Register */
#define PHY_ID2                          3          /*!< PHY Identifier Register */

#define PHY_AUTON                        4          /*!< PHY AutoNegotiate Register */


#define PHY_1GCTL                        9          /*!< PHY 1000T Control Register */
#define PHY_1GSTS                        10         /*!< PHY 1000T Status Register */

#define PHY_EXTCTL                       11         /*!< PHY Extended Control Register */
#define PHY_EXTW                         12         /*!< PHY Extended Write Register */
#define PHY_EXTR                         13         /*!< PHY Extended Read Register */


#define IS_ETH_PHY_ADDRESS(ADDRESS) ((ADDRESS) <= 0x20)
#define IS_ETH_PHY_REG(REG) (((REG) == PHY_BCR) || \
                             ((REG) == PHY_BSR) || \
                             ((REG) == PHY_SR))
                             

/* PHY configuration section **************************************************/
void alt_eth_delay(volatile uint32_t delay);
/* PHY Reset delay */ 
#define PHY_RESET_DELAY    ((uint32_t)0x000FFFFF) 
/* PHY Configuration delay */ 
#define PHY_CONFIG_DELAY   ((uint32_t)0x00FFFFFF)

/* Delay approximately 3 seconds after re-initialzing link */
#define ETH_RECONFIG_DELAY ((uint32_t)0x002FFFFF)

/* The PHY status register value may change from one PHY to another, 
   This value may need to be changed depending on the PHY selected. */

#define PHY_SR    ((uint16_t)31) /* Value for KSZ9021 PHY */

/* The Speed and Duplex mask values change from a PHY to another, so the user
   have to update this value depending on the used external PHY */
#define PHY_SPEED_100               ((uint16_t)0x0020) /* Value for KSZ9021 PHY */
#define PHY_SPEED_MASK              ((uint16_t)0x0070) /* Value for KSZ9021 PHY */
#define PHY_DUPLEX_STATUS           ((uint16_t)0x0008) /* Value for KSZ9021 PHY */

// define default PHY address.
#define KSZ9021RL_PHY_ADDRESS       0x04 /* Relative to CycloneV Board */

/* ksz9021 PHY Registers */
#define MII_KSZ9021_EXTENDED_CTRL	0x0b
#define MII_KSZ9021_EXTENDED_DATAW	0x0c
#define MII_KSZ9021_EXTENDED_DATAR	0x0d
#define MII_KSZ9021_PHY_CTL		    0x1f
#define MIIM_KSZ9021_PHYCTL_1000	(1 << 6)
#define MIIM_KSZ9021_PHYCTL_100		(1 << 5)
#define MIIM_KSZ9021_PHYCTL_10		(1 << 4)
#define MIIM_KSZ9021_PHYCTL_DUPLEX	(1 << 3)


#define MII_KSZ9021_EXT_COMMON_CTRL		    0x100
#define MII_KSZ9021_EXT_STRAP_STATUS		0x101
#define MII_KSZ9021_EXT_OP_STRAP_OVERRIDE	0x102
#define MII_KSZ9021_EXT_OP_STRAP_STATUS		0x103
#define MII_KSZ9021_EXT_RGMII_CLOCK_SKEW	0x104
#define MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW	0x105
#define MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW	0x106
#define MII_KSZ9021_EXT_ANALOG_TEST		    0x107  
                             
                            
/** PHY_basic_Control_register   		*/ 
#define PHY_Reset                       ((uint16_t)0x8000)      /*!< PHY Reset */
#define PHY_Loopback                    ((uint16_t)0x4000)      /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M             ((uint16_t)0x2100)      /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M             ((uint16_t)0x2000)      /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M              ((uint16_t)0x0100)      /*!< Set the full-duplex mode at 10 Mb/s */
#define PHY_HALFDUPLEX_10M              ((uint16_t)0x0000)      /*!< Set the half-duplex mode at 10 Mb/s */
#define PHY_AutoNegotiation             ((uint16_t)0x1000)      /*!< Enable auto-negotiation function */
#define PHY_Restart_AutoNegotiation     ((uint16_t)0x0200)      /*!< Restart auto-negotiation function */
#define PHY_Powerdown                   ((uint16_t)0x0800)      /*!< Select the power down mode */
#define PHY_Isolate                     ((uint16_t)0x0400)      /*!< Isolate PHY from MII */


/** PHY_basic_status_register   		*/ 
#define PHY_AutoNego_Complete           ((uint16_t)0x0020)      /*!< Auto-Negotiation process completed */
#define PHY_Linked_Status               ((uint16_t)0x0004)      /*!< Valid link established */
#define PHY_Jabber_detection            ((uint16_t)0x0002)      /*!< Jabber condition detected */


/**--------------------------------------------------------------------------**/
/*			MAC/PHY/DMA Function Prototypes.    							  */ 
/**--------------------------------------------------------------------------**/

ALT_STATUS_CODE alt_eth_reset_mac(void);



ALT_STATUS_CODE alt_eth_init(uint16_t phy_address);

ALT_STATUS_CODE  alt_eth_software_reset(void);
void  alt_eth_start(void);
void  alt_eth_stop(void);

void alt_eth_dma_mac_config(void);

/**--------------------------------------------------------------------------**/
/*			PHY Specific Function Prototypes.    							  */ 
/**--------------------------------------------------------------------------**/
uint16_t alt_eth_read_phy_register(uint16_t phy_address, uint16_t phy_reg);
ALT_STATUS_CODE alt_eth_write_phy_register(uint16_t phy_address, uint16_t phy_reg, uint16_t phy_value);


ALT_STATUS_CODE alt_eth_write_phy_register_extended(uint16_t phy_address, uint16_t phy_reg, uint16_t phy_value);
uint16_t alt_eth_read_phy_register_extended(uint16_t phy_address, uint16_t phy_reg);

void dump_phy_regs(uint16_t phy_address);

/**--------------------------------------------------------------------------**/
/*			MAC Specific Function Prototypes.    							  */ 
/**--------------------------------------------------------------------------**/
void alt_eth_mac_tx_en(alt_eth_functional_state_t new_state);
void alt_eth_mac_rx_en(alt_eth_functional_state_t new_state);
alt_eth_flag_status_t alt_eth_mac_get_flow_control_busy_status(void);
alt_eth_flag_status_t alt_eth_mac_get_mii_link_status(void);
void alt_eth_mac_pause_ctrl_frame(void);  
void alt_eth_mac_back_pressure_activate(alt_eth_functional_state_t new_state); 
alt_eth_flag_status_t alt_eth_mac_get_status_flags(uint32_t ETH_MAC_FLAG);  

uint32_t alt_eth_mac_get_irq_status(void);
void alt_eth_mac_set_irq(uint32_t ETH_MAC_IT, alt_eth_functional_state_t new_state);
void alt_eth_mac_set_mac_addr(uint32_t MacAddr, uint8_t *Addr);
void alt_eth_mac_get_mac_addr(uint32_t MacAddr, uint8_t *Addr);

/**--------------------------------------------------------------------------**/
/*			DMA Specific Function Prototypes.    							  */ 
/**--------------------------------------------------------------------------**/
extern uint32_t alt_eth_dma_get_irq_status(void);
alt_eth_flag_status_t alt_eth_dma_get_status_flags(uint32_t ETH_DMA_FLAG);
extern void alt_eth_dma_clear_status_flag(uint32_t ETH_DMA_FLAG);
alt_eth_irq_status_t alt_eth_dma_get_irq(uint32_t ETH_DMA_IT);
extern void alt_eth_dma_clear_irq_flag(uint32_t ETH_DMA_IT);

void alt_eth_clear_tx_rx_irq_bits(void);


uint32_t alt_eth_dma_get_tx_process_state(void);
extern uint32_t alt_eth_dma_get_rx_process_state(void);
extern void alt_eth_dma_flush_tx_fifo(void);
alt_eth_flag_status_t alt_eth_dma_get_tx_fifo_flush_status(void);
void alt_eth_dma_tx_en(alt_eth_functional_state_t new_state);
void alt_eth_dma_rx_en(alt_eth_functional_state_t new_state);
void alt_eth_dma_set_irq(uint32_t ETH_DMA_IT, alt_eth_functional_state_t new_state);
alt_eth_flag_status_t alt_eth_dma_get_overflow_status(uint32_t ETH_DMA_Overflow);

uint32_t alt_eth_dma_get_curr_tx_desc_start_addr(void);
uint32_t alt_eth_dma_get_curr_rx_desc_start_addr(void);
uint32_t alt_eth_dma_get_curr_tx_buff_addr(void);
uint32_t alt_eth_dma_get_curr_rx_buff_addr(void);

void alt_eth_dma_set_tx_desc_list_addr(uint32_t tx_desc_list_addr);
void alt_eth_dma_set_rx_desc_list_addr(uint32_t rx_desc_list_addr);

extern void alt_eth_dma_resume_dma_tx(void);
extern void alt_eth_dma_resume_dma_rx(void);
void alt_eth_dma_set_rx_int_wdt(uint8_t Value);





#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __ALT_ETHERNET_H__ */
