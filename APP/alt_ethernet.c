/*******************************************************************************
*                                                                              *
* Copyright 2013 Altera Corporation. All Rights Reserved.                      *
*                                                                              *
* Redistribution and use in source and binary forms, with or without           *
* modification, are permitted provided that the following conditions are met:  *
*                                                                              *
* 1. Redistributions of source code must retain the above copyright notice,    *
*    this list of conditions and the following disclaimer.                     *
*                                                                              *
* 2. Redistributions in binary form must reproduce the above copyright notice, *
*    this list of conditions and the following disclaimer in the documentation *
*    and/or other materials provided with the distribution.                    *
*                                                                              *
* 3. The name of the author may not be used to endorse or promote products     *
*    derived from this software without specific prior written permission.     *
*                                                                              *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR *
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ARE DISCLAIMED. IN NO  *
* EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,       *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, *
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;  *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,     *
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR      *
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF       *
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                   *
*                                                                              *
*******************************************************************************/
#include "alt_ethernet.h"

#include "alt_rstmgr.h"
#include "alt_sysmgr.h"
#include "socal.h"

uint16_t 	u16_phy_regs[40];

// S-Board use EMAC0, while EVK-Board use EMAC1
// change here(all 4 lines) and alt_eth_dma.h, httpserver.c
#define ALT_EMAC_GMAC(reg)		ALT_EMAC0_GMAC_##reg
#define ALT_EMAC_DMA(reg)		ALT_EMAC0_DMA_##reg
#define ALT_RSTMGR_PERMODRST_EMAC_SET		ALT_RSTMGR_PERMODRST_EMAC0_SET
#define ALT_RSTMGR_PERMODRST_EMAC_RESET		ALT_RSTMGR_PERMODRST_EMAC0_RESET

void alt_eth_delay(volatile uint32_t delay)
{
//  volatile uint32_t index = 0;
//  for(index = delay; index != 0; index--);
	delay >>= 16;
	delay++;
	extern void OSTimeDly(uint32_t ms);
	OSTimeDly(delay);
}


void dump_phy_regs(uint16_t phy_address)
{
   int i;
   // Record the PHY values.
   for (i=0; i< 32; i++)
   {
      u16_phy_regs[i] = alt_eth_read_phy_register(phy_address, i);
   }
   for (i=0; i< 6; i++)
   {
      u16_phy_regs[32+i] = alt_eth_read_phy_register_extended(phy_address, 0x100+i);
   }
   u16_phy_regs[39] = alt_eth_read_phy_register_extended(phy_address, 0x107);
}

/******************************************************************************/
/*                           Global ETH PHY functions                         */
/******************************************************************************/
int16_t	alt_eth_find_phy(void)
{
   volatile int 		phy_addr = 0;
   volatile uint16_t ctrl, oldctrl;

   do
   {
      ctrl = alt_eth_read_phy_register(phy_addr, PHY_BCR);
      oldctrl = ctrl & PHY_AutoNegotiation;

      ctrl ^= PHY_AutoNegotiation;
      alt_eth_write_phy_register(phy_addr, PHY_BCR, ctrl);
      ctrl = alt_eth_read_phy_register(phy_addr, PHY_BCR);
      ctrl &= PHY_AutoNegotiation;

      if (ctrl == oldctrl)
      {
         phy_addr++;
      } else
      {
         ctrl ^= PHY_AutoNegotiation;
         alt_eth_write_phy_register(phy_addr, PHY_BCR, ctrl);

         return phy_addr;
      }
   } while (phy_addr < 32);

   return ALT_E_ERROR;
}



ALT_STATUS_CODE alt_eth_phy_config(uint16_t phy_address)
{
   uint16_t utemp1 __attribute__((unused));
   uint16_t utemp2 __attribute__((unused));
   uint16_t utemp3 __attribute__((unused));

   /*--------------------   Configure the PHY   ----------------*/
   alt_eth_write_phy_register_extended(phy_address, MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW,0);
   alt_eth_write_phy_register_extended(phy_address, MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW,0);
   alt_eth_write_phy_register_extended(phy_address, MII_KSZ9021_EXT_RGMII_CLOCK_SKEW,0xA0D0);

   utemp1 = alt_eth_read_phy_register_extended(phy_address, MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW);
   utemp2 = alt_eth_read_phy_register_extended(phy_address, MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW);
   utemp3 = alt_eth_read_phy_register_extended(phy_address, MII_KSZ9021_EXT_RGMII_CLOCK_SKEW);

   /* Configure the PHY for AutoNegotiate*/
   alt_eth_write_phy_register(phy_address, PHY_AUTON, 0x1E1);
   alt_eth_write_phy_register(phy_address, PHY_BCR, 0x1200);

   return ALT_E_SUCCESS;
}



ALT_STATUS_CODE alt_eth_phy_reset(uint16_t phy_address)
{
   int i;
   /*-------------------- PHY initialization and configuration ----------------*/
   /* Put the PHY in reset mode */
   if((alt_eth_write_phy_register(phy_address, PHY_BCR, PHY_Reset)) != ALT_E_SUCCESS)
   {
      /* Return ERROR in case of write timeout */
      return ALT_E_ERROR;
   }

   /* Wait for the reset to clear */
   for (i=0; i<10; i++)
   {
      alt_eth_delay(PHY_RESET_DELAY);
      if ((alt_eth_read_phy_register(phy_address, PHY_BCR) & PHY_Reset) == 0)
         break;
   }

   /* Spin here if there is a problem. Want to know quickly */
   if (i == 10)  while(1);

   /* Delay to assure PHY reset */
   alt_eth_delay(PHY_RESET_DELAY);

   return ALT_E_SUCCESS;
}



/******************************************************************************/
/*                           Global ETH MAC/DMA functions                     */
/******************************************************************************/


ALT_STATUS_CODE alt_eth_reset_mac(void)
{
   /*-------------------- Reset the EMAC and set the PHY mode  ----------------*/

   /* Reset the EMAC */
   alt_setbits_word(ALT_RSTMGR_PERMODRST_ADDR,
                   ALT_RSTMGR_PERMODRST_EMAC_SET(ALT_RSTMGR_PERMODRST_EMAC_RESET));

   /* Clear out the previous value and update these to RGMII */
   alt_clrbits_word(ALT_SYSMGR_EMAC_CTL_ADDR,  0x0F);
   alt_setbits_word(ALT_SYSMGR_EMAC_CTL_ADDR,
                   ALT_SYSMGR_EMAC_CTL_PHYSEL_0_SET(ALT_SYSMGR_EMAC_CTL_PHYSEL_0_E_RGMII) |
                   ALT_SYSMGR_EMAC_CTL_PHYSEL_1_SET(ALT_SYSMGR_EMAC_CTL_PHYSEL_1_E_RGMII) );
   /* Remove the reset. */
   alt_clrbits_word(ALT_RSTMGR_PERMODRST_ADDR,
                   ALT_RSTMGR_PERMODRST_EMAC_SET(ALT_RSTMGR_PERMODRST_EMAC_RESET));

   /* Delay to assure PHY reset */
   alt_eth_delay(PHY_RESET_DELAY);

   return ALT_E_SUCCESS;
}




ALT_STATUS_CODE alt_eth_init(uint16_t phy_address)
{
  uint32_t reg_value = 0, tmpreg = 0;
  volatile uint32_t alt_mac_config_reg_settings = 0;
  volatile uint32_t timeout = 0;


  /*-------------------------------- MAC Config ------------------------------*/
  /*-------------------- Reset the EMAC and set the PHY mode  ----------------*/
   /* Reset the entire Ethernet */
   alt_eth_software_reset();

   /* Reset the EMAC */
   alt_eth_reset_mac();

   /*--------------------------- Find the PHY Address -------------------------*/
   // TODO - Come back and fix this.
   //tmpreg = alt_eth_find_phy();

   alt_eth_phy_reset(phy_address);

   /*-------------------- PHY read and check     ----------------*/
   /* Check the PHY Identifier */
   reg_value = alt_eth_read_phy_register(phy_address, 2);
   tmpreg = alt_eth_read_phy_register(phy_address, 3);
   if ((reg_value != 0x22) && ((tmpreg & 0xFFF0) != 0x1610)) while (1);

   /*--------------------   Configure the PHY   ----------------*/
   alt_eth_phy_config(phy_address);

   /*--------------------   Configure the MAC   ----------------*/
   alt_mac_config_reg_settings = (ALT_EMAC_GMAC_MAC_CFG_IPC_SET_MSK |  /* Checksum Offload */
                                  ALT_EMAC_GMAC_MAC_CFG_JD_SET_MSK  |  /* Jabber Disable */
                                  ALT_EMAC_GMAC_MAC_CFG_PS_SET_MSK  |  /* Port Select = MII */
                                  ALT_EMAC_GMAC_MAC_CFG_BE_SET_MSK  |  /* Frame Burst Enable */
                                  ALT_EMAC_GMAC_MAC_CFG_WD_SET_MSK );  /* Watchdog Disable */


#ifdef ALT_ETH_SET_EMAC_RATE
    if(alt_eth_write_phy_register(phy_address, PHY_BCR, ((uint16_t)ALT_ETH_SET_EMAC_MODE |
                                                          (uint16_t)ALT_ETH_SET_EMAC_SPEED ))
                                          != ALT_E_SUCCESS)
    {
      /* Return ERROR in case of write timeout */
      return ALT_E_ERROR;
    }
    /* Delay to assure PHY configuration */
    alt_eth_delay(PHY_CONFIG_DELAY);

#else    //#ifdef ALT_ETH_SET_EMAC_RATE
   /* Determine the correct PHY settings if we are AutoNegotiating */

   /* First wait for linked status... */
   /* Reset Timeout counter */
   timeout = 0;
   do
   {
      timeout++;
   } while (!(alt_eth_read_phy_register(phy_address, PHY_BSR) & PHY_Linked_Status) && (timeout < PHY_READ_TO));

   /* Return ERROR in case of timeout */
   if(timeout == PHY_READ_TO)
   {
      return ALT_E_ERROR;
   }

   /* Reset Timeout counter */
   timeout = 0;
   /* Enable Auto-Negotiation */
   if((alt_eth_write_phy_register(phy_address, PHY_BCR, PHY_AutoNegotiation)) != ALT_E_SUCCESS)
   {
      /* Return ERROR in case of write timeout */
      return ALT_E_ERROR;
   }

   /* Wait until the auto-negotiation will be completed */
   do
   {
      timeout++;
   } while (!(alt_eth_read_phy_register(phy_address, PHY_BSR) & PHY_AutoNego_Complete) && (timeout < (uint32_t)PHY_READ_TO));

   /* Return ERROR in case of timeout */
   if(timeout == PHY_READ_TO)
   {
      return ALT_E_ERROR;
   }

   /* Reset Timeout counter */
   timeout = 0;

   /* Read the result of the auto-negotiation */
   reg_value = alt_eth_read_phy_register(phy_address, PHY_SR);

   /* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
   if((reg_value & PHY_DUPLEX_STATUS) != (uint32_t)RESET)
   {
      /* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
      alt_mac_config_reg_settings |= ALT_EMAC_GMAC_MAC_CFG_DM_SET_MSK;
   }

   /* Configure the MAC with the speed fixed by the auto-negotiation process */
   if ((reg_value & PHY_SPEED_MASK) == PHY_SPEED_100)
   {
      /* Set Ethernet speed to 100M following the auto-negotiation */
      alt_mac_config_reg_settings |= ALT_EMAC_GMAC_MAC_CFG_FES_SET_MSK;
   } else if ((reg_value & PHY_SPEED_MASK) > PHY_SPEED_100)
   {
      /* Set Ethernet speed to 1G following the auto-negotiation */
      alt_mac_config_reg_settings &= ALT_EMAC_GMAC_MAC_CFG_PS_CLR_MSK;
   }

   /* dump_phy_regs(phy_address); */

#endif      //#ifdef ALT_ETH_SET_EMAC_RATE


   /*------------------------    ETHERNET MAC Config   -----------------------*/
   /*----------------------- ETHERNET MACIM Configuration --------------------*/
   /* Read the SMII Status Register to clear the SMII changed flag */
   alt_eth_mac_get_mii_link_status();
   /* Disable MAC interrupts */
   alt_write_word(ALT_EMAC_GMAC(INT_MSK_ADDR), (ALT_EMAC_GMAC_INT_STAT_LPIIS_SET_MSK |   /* Disable Low Power IRQ */
                                                ALT_EMAC_GMAC_INT_STAT_TSIS_SET_MSK ));  /* Disable Timestamp IRQ */

  /*------------------------ ETHERNET MACCR Configuration --------------------*/
   /* Set the MAC Configureation Register */
   alt_write_word(ALT_EMAC_GMAC(MAC_CFG_ADDR), alt_mac_config_reg_settings);

  /*-------------------------------- DMA Config ------------------------------*/
  /*----------------------- ETHERNET DMABM Configuration --------------------*/

   /* Set the DMA Bus Mode Register */
   alt_write_word(ALT_EMAC_DMA(BUS_MOD_ADDR),  (ALT_EMAC_DMA_BUS_MOD_USP_SET_MSK   | /* Use separate PBL */
                                                ALT_EMAC_DMA_BUS_MOD_AAL_SET_MSK   | /* Address Aligned Beats */
#ifdef USE_ENHANCED_DMA_DESCRIPTORS
                                                ALT_EMAC_DMA_BUS_MOD_ATDS_SET_MSK  | /* Alternate Descriptor Size */
#endif
                                                ALT_EMAC_DMA_BUS_MOD_EIGHTXPBL_SET(1) |
                                                ALT_EMAC_DMA_BUS_MOD_PBL_SET(8)    | /* Programmable Burst Length */
                                                ALT_EMAC_DMA_BUS_MOD_RPBL_SET(8)));  /* Programmable Burst Length */

   /*----------------------- ETHERNET DMAOMR Configuration --------------------*/
   /* Set the DMA Operation Mode Register */
   alt_write_word(ALT_EMAC_DMA(OP_MOD_ADDR),   (ALT_EMAC_DMA_OP_MOD_OSF_SET_MSK    | /* Operate on Second Frame */
                                                ALT_EMAC_DMA_OP_MOD_TSF_SET_MSK    | /* Transmit Store and Forward */
                                                ALT_EMAC_DMA_OP_MOD_RSF_SET_MSK  )); /* Receive Store and Forward */

   /*----------------- ETHERNET DMA AXI Bus Mode Configuration -----------------*/
   /* Set the DMA Operation Mode Register */
   alt_write_word(ALT_EMAC_DMA(AXI_BUS_MOD_ADDR), 0); /* Clear everything */

  /* Return Ethernet configuration success */
  return ALT_E_SUCCESS;
}




void alt_eth_start(void)
{
  /* Enable transmit state machine of the MAC for transmission on the MII */
  alt_eth_mac_tx_en(ENABLE);
  /* Flush Transmit FIFO */
  alt_eth_dma_flush_tx_fifo();
  /* Enable receive state machine of the MAC for reception from the MII */
  alt_eth_mac_rx_en(ENABLE);

  /* Start DMA transmission */
  alt_eth_dma_tx_en(ENABLE);
  /* Start DMA reception */
  alt_eth_dma_rx_en(ENABLE);

  alt_eth_delay(PHY_RESET_DELAY);
}




void alt_eth_stop(void)
{

  /* Stop DMA transmission */
  alt_eth_dma_tx_en(DISABLE);
  /* Stop DMA reception */
  alt_eth_dma_rx_en(DISABLE);

  /* Disable transmit state machine of the MAC for transmission on the MII */
  alt_eth_mac_tx_en(DISABLE);
  /* Flush Transmit FIFO */
  alt_eth_dma_flush_tx_fifo();
  /* Disable receive state machine of the MAC for reception from the MII */
  alt_eth_mac_rx_en(DISABLE);

  /* Stop DMA transmission */
  alt_eth_dma_tx_en(DISABLE);
  /* Stop DMA reception */
  alt_eth_dma_rx_en(DISABLE);

  alt_eth_delay(PHY_RESET_DELAY);
}




void alt_eth_mac_tx_en(alt_eth_functional_state_t new_state)
{
  if (new_state != DISABLE)
  {
    /* Enable the MAC transmission */
    alt_setbits_word(ALT_EMAC_GMAC(MAC_CFG_ADDR),
                     ALT_EMAC_GMAC_MAC_CFG_TE_SET_MSK);
  }
  else
  {
    /* Disable the MAC transmission */
    alt_clrbits_word(ALT_EMAC_GMAC(MAC_CFG_ADDR),
                     ALT_EMAC_GMAC_MAC_CFG_TE_SET_MSK);

  }
}




void alt_eth_mac_rx_en(alt_eth_functional_state_t new_state)
{

  if (new_state != DISABLE)
  {
    /* Enable the MAC reception */
    alt_setbits_word(ALT_EMAC_GMAC(MAC_CFG_ADDR),
                     ALT_EMAC_GMAC_MAC_CFG_RE_SET_MSK);

  }
  else
  {
    /* Disable the MAC reception */
    alt_clrbits_word(ALT_EMAC_GMAC(MAC_CFG_ADDR),
                     ALT_EMAC_GMAC_MAC_CFG_RE_SET_MSK);
  }
}



alt_eth_flag_status_t alt_eth_mac_get_flow_control_busy_status(void)
{
  alt_eth_flag_status_t bitstatus = RESET;
  /* The Flow Control register should not be written to until this bit is cleared */
  if (ALT_EMAC_GMAC_FLOW_CTL_FCA_BPA_GET(alt_read_word(ALT_EMAC_GMAC(FLOW_CTL_ADDR))))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}



void alt_eth_mac_pause_ctrl_frame(void)
{
  /* When Set In full duplex MAC initiates pause control frame */
  alt_setbits_word(ALT_EMAC_GMAC(FLOW_CTL_ADDR),
                   ALT_EMAC_GMAC_FLOW_CTL_FCA_BPA_SET_MSK);

}


alt_eth_flag_status_t alt_eth_mac_get_mii_link_status(void)
{
  alt_eth_flag_status_t bitstatus = RESET;
  /* The Flow Control register should not be written to until this bit is cleared */
  if (ALT_EMAC_GMAC_MII_CTL_STAT_LNKSTS_GET(alt_read_word(ALT_EMAC_GMAC(SGMII_RGMII_SMII_CTL_STAT_ADDR))))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}



void alt_eth_mac_back_pressure_activate(alt_eth_functional_state_t new_state)
{

  if (new_state != DISABLE)
  {
    /* Activate the MAC BackPressure operation */
    /* In Half duplex: during backpressure, when the MAC receives a new frame,
    the transmitter starts sending a JAM pattern resulting in a collision */
    alt_setbits_word(ALT_EMAC_GMAC(FLOW_CTL_ADDR),
                   ALT_EMAC_GMAC_FLOW_CTL_FCA_BPA_SET_MSK);

  }
  else
  {
    /* Desactivate the MAC BackPressure operation */
    alt_clrbits_word(ALT_EMAC_GMAC(FLOW_CTL_ADDR),
                   ALT_EMAC_GMAC_FLOW_CTL_FCA_BPA_SET_MSK);

  }
}


alt_eth_flag_status_t alt_eth_mac_get_status_flags(uint32_t mac_flag_mask)
{
  alt_eth_flag_status_t bitstatus = RESET;

  if (alt_read_word(ALT_EMAC_GMAC(INT_STAT_ADDR)) & mac_flag_mask)

  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}


uint32_t alt_eth_mac_get_irq_status(void)
{
	return alt_read_word(ALT_EMAC_GMAC(INT_STAT_ADDR));
}



void alt_eth_mac_set_irq(uint32_t mac_irq_mask, alt_eth_functional_state_t new_state)
{

  if (new_state != DISABLE)
  {
    /* Enable the selected ETHERNET MAC interrupts */
    alt_clrbits_word(ALT_EMAC_GMAC(INT_MSK_ADDR),
                   mac_irq_mask);
  }
  else
  {
    /* Disable the selected ETHERNET MAC interrupts */
    alt_setbits_word(ALT_EMAC_GMAC(INT_MSK_ADDR),
                   mac_irq_mask);
  }
}



void alt_eth_mac_set_mac_addr(uint32_t mac_addr_num, uint8_t *address)
{
  uint32_t tmpreg;

  /* Calculate the selected MAC address high register */
  tmpreg = ((uint32_t)address[5] << 8) | (uint32_t)address[4];
  /* Load the selected MAC address high register */
  alt_write_word(ALT_EMAC_GMAC(MAC_ADDR0_HIGH_ADDR) + (8*mac_addr_num),
  						tmpreg);

  /* Calculate the selected MAC address low register */
  tmpreg = ((uint32_t)address[3] << 24) | ((uint32_t)address[2] << 16) |
  			((uint32_t)address[1] << 8) | address[0];
   /* Load the selected MAC address low register */
  alt_write_word(ALT_EMAC_GMAC(MAC_ADDR0_LOW_ADDR) + (8*mac_addr_num), tmpreg);

}



void alt_eth_mac_get_mac_addr(uint32_t mac_addr_num, uint8_t *address)
{
  uint32_t tmpreg;

  /* Get the selected MAC address high register */
  tmpreg = alt_read_word(ALT_EMAC_GMAC(MAC_ADDR0_HIGH_ADDR) + (8*mac_addr_num));
   /* Calculate the selected MAC address buffer */
  address[5] = ((tmpreg >> 8) & (uint8_t)0xFF);
  address[4] = (tmpreg & (uint8_t)0xFF);


  /* Load the selected MAC address low register */
  tmpreg = alt_read_word(ALT_EMAC_GMAC(MAC_ADDR0_LOW_ADDR) + (8*mac_addr_num));
  /* Calculate the selected MAC address buffer */
  address[3] = ((tmpreg >> 24) & (uint8_t)0xFF);
  address[2] = ((tmpreg >> 16) & (uint8_t)0xFF);
  address[1] = ((tmpreg >> 8 ) & (uint8_t)0xFF);
  address[0] = (tmpreg & (uint8_t)0xFF);
}


/******************************************************************************/
/*                           DMA Descriptors functions                        */
/******************************************************************************/

// TODO: Come back and clean this up.
uint32_t alt_eth_dma_get_rx_desc_frame_len(alt_eth_dma_descriptor_t *rx_dma_desc)
{
  /* Return the Receive descriptor frame length */
  return ((rx_dma_desc->Status & ETH_DMARxDesc_FL) >> ETH_DMARXDESC_FRAME_LENGTHSHIFT);
}




/******************************************************************************/
/*                           DMA functions                                    */
/******************************************************************************/

alt_eth_flag_status_t alt_eth_get_software_reset_status(void)
{
  alt_eth_flag_status_t bitstatus = RESET;

  if(ALT_EMAC_DMA_BUS_MOD_SWR_GET(alt_read_word(ALT_EMAC_DMA(BUS_MOD_ADDR))))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}



ALT_STATUS_CODE alt_eth_software_reset(void)
{
  int i;
  /* Set the SWR bit: resets all MAC subsystem internal registers and logic */
  /* After reset all the registers holds their respective reset values */
  alt_setbits_word(ALT_EMAC_DMA(BUS_MOD_ADDR),
                   ALT_EMAC_DMA_BUS_MOD_SWR_SET_MSK);

  /* Wait for the software reset to clear */
  for (i=0; i<10; i++)
  {
    alt_eth_delay(PHY_RESET_DELAY);
    if (alt_eth_get_software_reset_status() == RESET)
      break;
  }

  /* Spin here if there is a problem. Want to know quickly */
  if (i == 10)  while(1);

  return ALT_E_SUCCESS;
}




inline uint32_t alt_eth_dma_get_irq_status(void)
{
  return alt_read_word(ALT_EMAC_DMA(STAT_ADDR));
}



alt_eth_flag_status_t alt_eth_dma_get_status_flags(uint32_t dma_flag)
{
  alt_eth_flag_status_t bitstatus = RESET;

  if (alt_read_word(ALT_EMAC_DMA(STAT_ADDR)) & dma_flag)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}



inline void alt_eth_dma_clear_status_flag(uint32_t dma_flag)
{

  /* Clear the selected ETHERNET DMA FLAG */
  alt_write_word(ALT_EMAC_DMA(STAT_ADDR), dma_flag);
}



void alt_eth_dma_set_irq(uint32_t dma_irq_flag, alt_eth_functional_state_t new_state)
{

  if (new_state != DISABLE)
  {
    /* Enable the selected ETHERNET DMA interrupts */
    alt_setbits_word(ALT_EMAC_DMA(INT_EN_ADDR),
                     dma_irq_flag);
  }
  else
  {
    /* Disable the selected ETHERNET DMA interrupts */
    alt_clrbits_word(ALT_EMAC_DMA(INT_EN_ADDR),
                     dma_irq_flag);
  }
}



alt_eth_irq_status_t alt_eth_dma_get_irq(uint32_t dma_irq_flag)
{
  alt_eth_irq_status_t bitstatus = RESET;

  if (alt_read_word(ALT_EMAC_DMA(STAT_ADDR)) & dma_irq_flag)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}



inline void alt_eth_dma_clear_irq_flag(uint32_t dma_irq_flag)
{
  /* Clear the selected ETHERNET DMA IT */
  alt_write_word(ALT_EMAC_DMA(STAT_ADDR), dma_irq_flag);
}

/* Clear the Ethernet DMA Rx/Tx IT pending bits */
void alt_eth_clear_tx_rx_irq_bits(void)
{
  alt_eth_dma_clear_irq_flag( ALT_EMAC_DMA_INT_EN_NIE_SET_MSK |
                              ALT_EMAC_DMA_INT_EN_RIE_SET_MSK |
                              ALT_EMAC_DMA_INT_EN_TIE_SET_MSK);
}




uint32_t alt_eth_dma_get_tx_process_state(void)
{
  return ((uint32_t)(ALT_EMAC_DMA_STAT_TI_GET(
                     alt_read_word(ALT_EMAC_DMA(STAT_ADDR)) )
                    ) );
}



inline uint32_t alt_eth_dma_get_rx_process_state(void)
{
  return ((uint32_t)(ALT_EMAC_DMA_STAT_RI_GET(alt_read_word(ALT_EMAC_DMA(STAT_ADDR)))));
}



inline void alt_eth_dma_flush_tx_fifo(void)
{
  /* Set the Flush Transmit FIFO bit */
  alt_setbits_word(ALT_EMAC_DMA(OP_MOD_ADDR), ALT_EMAC_DMA_OP_MOD_FTF_SET_MSK);
}



alt_eth_flag_status_t alt_eth_dma_get_tx_fifo_flush_status(void)
{
  alt_eth_flag_status_t bitstatus = RESET;

  if (ALT_EMAC_DMA_OP_MOD_FTF_GET(alt_read_word(ALT_EMAC_DMA(OP_MOD_ADDR))))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}



void alt_eth_dma_tx_en(alt_eth_functional_state_t new_state)
{

  if (new_state != DISABLE)
  {
    /* Enable the DMA transmission */
    alt_setbits_word(ALT_EMAC_DMA(OP_MOD_ADDR),
                    ALT_EMAC_DMA_OP_MOD_ST_SET_MSK);

  }
  else
  {
    /* Disable the DMA transmission */
    alt_clrbits_word(ALT_EMAC_DMA(OP_MOD_ADDR),
                    ALT_EMAC_DMA_OP_MOD_ST_SET_MSK);

  }
}




void alt_eth_dma_rx_en(alt_eth_functional_state_t new_state)
{

  if (new_state != DISABLE)
  {
    /* Enable the DMA reception */
    alt_setbits_word(ALT_EMAC_DMA(OP_MOD_ADDR),
                   ALT_EMAC_DMA_OP_MOD_SR_SET_MSK);

  }
  else
  {
    /* Disable the DMA reception */
    alt_clrbits_word(ALT_EMAC_DMA(OP_MOD_ADDR),
                   ALT_EMAC_DMA_OP_MOD_SR_SET_MSK);

  }
}




alt_eth_flag_status_t alt_eth_dma_get_overflow_status(uint32_t dma_overflow_flag)
{
   alt_eth_flag_status_t bitstatus = RESET;

   if (alt_read_word(ALT_EMAC_DMA(MISSED_FRM_AND_BUF_OVF_CNTR_ADDR)) & dma_overflow_flag)
   {
      bitstatus = SET;
   }
   else
   {
      bitstatus = RESET;
   }
   return bitstatus;
}




uint32_t alt_eth_dma_get_curr_tx_desc_start_addr(void)
{
   return ((uint32_t)alt_read_word(ALT_EMAC_DMA(CUR_HOST_TX_DESC_ADDR)));
}




uint32_t alt_eth_dma_get_curr_rx_desc_start_addr(void)
{
   return ((uint32_t)alt_read_word(ALT_EMAC_DMA(CUR_HOST_RX_DESC_ADDR)));
}




uint32_t alt_eth_dma_get_curr_tx_buff_addr(void)
{
   return ((uint32_t)alt_read_word(ALT_EMAC_DMA(CUR_HOST_TX_BUF_ADDR_ADDR)));
}




uint32_t alt_eth_dma_get_curr_rx_buff_addr(void)
{
   return ((uint32_t)alt_read_word(ALT_EMAC_DMA(CUR_HOST_RX_BUF_ADDR_ADDR)));
}




void alt_eth_dma_set_tx_desc_list_addr(uint32_t tx_desc_list_addr)
{
   alt_write_word(ALT_EMAC_DMA(TX_DESC_LIST_ADDR_ADDR), tx_desc_list_addr);
}




void alt_eth_dma_set_rx_desc_list_addr(uint32_t rx_desc_list_addr)
{
   alt_write_word(ALT_EMAC_DMA(RX_DESC_LIST_ADDR_ADDR), rx_desc_list_addr);
}




inline void alt_eth_dma_resume_dma_tx(void)
{
   alt_write_word(ALT_EMAC_DMA(TX_POLL_DEMAND_ADDR), 0);
}




inline void alt_eth_dma_resume_dma_rx(void)
{
   alt_write_word(ALT_EMAC_DMA(RX_POLL_DEMAND_ADDR), 0);
}




void alt_eth_dma_set_rx_int_wdt(uint8_t value)
{
   /* Set the DMA Receive status watchdog timer register */
   alt_write_word(ALT_EMAC_DMA(RX_INT_WDT_ADDR), value);

}

/******************************************************************************/
/*                                PHY functions                               */
/******************************************************************************/

uint16_t alt_eth_read_phy_register(uint16_t phy_address, uint16_t phy_reg)
{
   volatile uint32_t tmpreg = 0;
   volatile uint32_t timeout = 0;

   /* Prepare the MII address register value */
   tmpreg = 0;
   /* Set the PHY device address */
   tmpreg |= ALT_EMAC_GMAC_GMII_ADDR_PA_SET(phy_address);
   /* Set the PHY register address */
   tmpreg |= ALT_EMAC_GMAC_GMII_ADDR_GR_SET(phy_reg);
   /* Set the read mode */
   tmpreg &= ALT_EMAC_GMAC_GMII_ADDR_GW_CLR_MSK;
   /* Set the clock divider */
   tmpreg |= ALT_EMAC_GMAC_GMII_ADDR_CR_SET(ALT_EMAC_GMAC_GMII_ADDR_CR_E_DIV102);
   /* Set the MII Busy bit */
   tmpreg |= ALT_EMAC_GMAC_GMII_ADDR_GB_SET(ALT_EMAC_GMAC_GMII_ADDR_GB_E_END);
   /* Write the result value into the MII Address register */
   alt_write_word(ALT_EMAC_GMAC(GMII_ADDR_ADDR), tmpreg);
   /* Check for the Busy flag */
   do
   {
      timeout++;
      tmpreg = alt_read_word(ALT_EMAC_GMAC(GMII_ADDR_ADDR));
   } while ((tmpreg & ALT_EMAC_GMAC_GMII_ADDR_GB_SET_MSK) && (timeout < (uint32_t)PHY_READ_TO));

   /* Return ERROR in case of timeout */
   if(timeout == PHY_READ_TO)
   {
      return (uint16_t)ALT_E_ERROR;
   }

   /* Return data register value */
   return alt_read_word(ALT_EMAC_GMAC(GMII_DATA_ADDR));

}





ALT_STATUS_CODE alt_eth_write_phy_register(uint16_t phy_address, uint16_t phy_reg, uint16_t phy_value)
{
   uint32_t tmpreg = 0;
   volatile uint32_t timeout = 0;

   /* Prepare the MII address register value */
   tmpreg = 0;
   /* Set the PHY device address */
   tmpreg |= ALT_EMAC_GMAC_GMII_ADDR_PA_SET(phy_address);
   /* Set the PHY register address */
   tmpreg |= ALT_EMAC_GMAC_GMII_ADDR_GR_SET(phy_reg);
   /* Set the write mode */
   tmpreg |= ALT_EMAC_GMAC_GMII_ADDR_GW_SET_MSK;
   /* Set the clock divider */
   tmpreg |= ALT_EMAC_GMAC_GMII_ADDR_CR_SET(ALT_EMAC_GMAC_GMII_ADDR_CR_E_DIV102);
   /* Set the MII Busy bit */
   tmpreg |= ALT_EMAC_GMAC_GMII_ADDR_GB_SET(ALT_EMAC_GMAC_GMII_ADDR_GB_E_END);
   /* Give the value to the MII data register */
   alt_write_word(ALT_EMAC_GMAC(GMII_DATA_ADDR), phy_value);
   /* Write the result value into the MII Address register */
   alt_write_word(ALT_EMAC_GMAC(GMII_ADDR_ADDR), tmpreg);

   /* Check the Busy flag */
   do
   {
      timeout++;
      tmpreg = alt_read_word(ALT_EMAC_GMAC(GMII_ADDR_ADDR));
   } while ((tmpreg & ALT_EMAC_GMAC_GMII_ADDR_GB_SET_MSK) && (timeout < (uint32_t)PHY_WRITE_TO));

   /* Return ERROR in case of timeout */
   if(timeout == PHY_WRITE_TO)
   {
      return ALT_E_ERROR;
   }

   /* Return SUCCESS */
   return ALT_E_SUCCESS;
}



ALT_STATUS_CODE alt_eth_write_phy_register_extended(uint16_t phy_address, uint16_t phy_reg, uint16_t phy_value)
{
    ALT_STATUS_CODE retcode;
    retcode = alt_eth_write_phy_register(phy_address,
                         MII_KSZ9021_EXTENDED_CTRL,
                         0x8000 | phy_reg);
    retcode |= alt_eth_write_phy_register(phy_address,
                         MII_KSZ9021_EXTENDED_DATAW,
                         phy_value);

    return retcode;
}



uint16_t alt_eth_read_phy_register_extended(uint16_t phy_address, uint16_t phy_reg)
{
    alt_eth_write_phy_register(phy_address,
                         MII_KSZ9021_EXTENDED_CTRL,
                         phy_reg);
    return alt_eth_read_phy_register(phy_address,
                               MII_KSZ9021_EXTENDED_DATAR);
}




ALT_STATUS_CODE ETH_PHYLoopBackCmd(uint16_t phy_address, alt_eth_functional_state_t new_state)
{
  uint16_t tmpreg = 0;

  /* Get the PHY configuration to update it */
  tmpreg = alt_eth_read_phy_register(phy_address, PHY_BCR);

  if (new_state != DISABLE)
  {
    /* Enable the PHY loopback mode */
    tmpreg |= PHY_Loopback;
  }
  else
  {
    /* Disable the PHY loopback mode: normal mode */
    tmpreg &= (uint16_t)(~(uint16_t)PHY_Loopback);
  }
  /* Update the PHY control register with the new configuration */
  if(alt_eth_write_phy_register(phy_address, PHY_BCR, tmpreg) != ALT_E_SUCCESS)
  {
    return ALT_E_SUCCESS;
  }
  else
  {
    /* Return SUCCESS */
    return ALT_E_ERROR;
  }
}




void alt_eth_dma_mac_config(void)
{

   /* Reset ETHERNET MAC */
   alt_eth_reset_mac();

   /* Software reset of DMA block */
   alt_eth_software_reset();

   /* Configure Ethernet */
   alt_eth_init(KSZ9021RL_PHY_ADDRESS);

   /* Enable the Ethernet Rx Interrupt, TX Interrupt & Normal Int Summary */
   alt_eth_dma_set_irq(ALT_EMAC_DMA_INT_EN_NIE_SET_MSK |
                  ALT_EMAC_DMA_INT_EN_RIE_SET_MSK |
                  ALT_EMAC_DMA_INT_EN_TIE_SET_MSK,
                  ENABLE);
}

