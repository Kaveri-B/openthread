/*! ****************************************************************************
 * @file:    ADF7242.c
 * @brief:   ADF7242 core driver implementation.
 * @details: 
 ------------------------------------------------------------------------------
Copyright (c) 2010-2015 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-
INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF
CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "ADF7242.h" 
#include "ADF7242_Config.h"
#include "adf7242-firmware.h"

   
ieee_802_15_4_mode_reg_t ieee_802_15_4_mode_regs = {
               ADF7242_CONF_PAN_ID, 
               ADF7242_CONF_MAC_SHORT_ADDR, 
               ADF7242_CONF_MAC_EXTENDED_ADDR, 
               0x00,
               0x00,
               (ADF7242_CONF_MAX_CCA_RETRY << 4) | ADF7242_CONF_MAX_FRAME_RETRY,  
               (ADF7242_CONF_MIN_BE << 4) | ADF7242_CONF_MAX_BE 
               };

 ADF7242_trx_config_t TRX_config_params = {ADF7242_CONF_PHY_MODE,
                                           ADF7242_CONF_CHANNEL_FREQ, 
                                           ADF7242_CONF_PACKET_OPR_MODE,
                                           ADF7242_CONF_DEFAULT_TX_POWER};
  
uint8_t rx_buff[ADF7242_CONF_RX_BUFF_SIZE];
uint8_t spi_txbuff[ADF7242_CONF_TX_BUFF_SIZE];

#ifdef ADF7242_DEBUG_d
uint8_t g_rx_pkt = 0;
uint8_t g_rx_sfd = 0;
uint8_t g_rx_frame_valid = 0;
uint8_t g_rx_addr_valid = 0;
#endif

/* Static function declarations */
static void ADF7242_PHY_init(ADF7242_phy_mode_t phy_mode);
static void ADf7242_upload_firmware(void);
static void ADF7242_issue_command(uint8_t command);
static void ADF7242_REG_write(uint16_t reg_addr, uint8_t data);
static void ADF7242_MEM_block_write(uint16_t mem_addr, uint8_t* pData, uint16_t size);
static void ADF7242_PKT_write(uint8_t* pTxbuff, uint16_t size);
static void ADF7242_PKT_read(uint8_t* pRxbuff, uint16_t* size);
static void ADF7242_MEM_read(uint16_t mem_addr, uint16_t size, uint8_t* pData);
static uint8_t ADF7242_wait_state(uint8_t state_mask);
static uint8_t ADF7242_read_status(void);

/********************* Public function definations ****************************/
/*!
 * @brief  This function initializes ADF7242 with default configured parameters. 
 *
 * @param    None 
 *
 * @return  Status
 *          - #ADF_SUCCESS              If ADF7242 radio initialization success.
 *          - #ADF_SPI_INIT_FAILED      If SPI initialization failed.
 *          - #ADF_IRQ_INIT_FAILED      If IRQ initialization failed.
 */
ADF7242_Result_t ADF7242_init(void)
{
  uint32_t i = 100000;
  ADF7242_Result_t eResult = ADF_SUCCESS;
  
  /* Initialize SPI */
  eResult = SPI_init();  
  if(eResult) {
    return eResult;
  }
  
  /* Initialize IRQ pin for ADf7242 */
  eResult = IRQ_init();
  if (eResult) {
    return eResult;
  }
  
  Disable_TRX_IRQ();
  
  /* Reset ADF7242 */
  ADF7242_issue_command(CMD_RC_RESET); 
  /* delay of 2ms*/
  //rf_delay(2);
  while(i)
  {
    i--;
  }
  /* wait untill MISO output becomes high*/
   if(PhyWakeUp() != 0) while(1);
  /* Wait untill the TRX state becomes RC_READY */
  ADF7242_wait_state(RC_READY_MASK);
  
  /* Initialize configured PHY mode */
  ADF7242_PHY_init(TRX_config_params.phy_mode);
  
  /* Initialize channel frequency */
  ADF7242_set_channel(TRX_config_params.channelFreq);
     
  /* Initialize TRX to ready state */
  ADF7242_issue_command(CMD_RC_PHY_RDY);
  ADF7242_wait_state(RC_PHY_RDY_MASK);
  
  Enable_TRX_IRQ();
	
  return eResult;
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Writes transmitting packet into ADF7242 tx buffer and starts 
 *         transmission. 
 *
 * @param[in]   pTxbuf          Pointer to tranmitting buffer, allocated by 
 *                              higher layer.
 *
 * @param[in]   txlen           Length of transmitting packet.
 *
 * @return  None
 */
int8_t ADF7242_tx(uint8_t *pTxbuf, uint16_t txlen)
{
	uint8_t irq_status;
	
  /*TBD: Need to check whether trx is in busy state. */
	
	ADF7242_issue_command(CMD_RC_PHY_RDY);
    /* Wait untill TRX state becomes PHY_RDY */
    ADF7242_wait_state(RC_PHY_RDY_MASK);
  
			ADF7242_MEM_read(REG_IRQ1_SRC1, 1, &irq_status);
		if((irq_status & IRQ_RX_PKT_RCVD)) {
			return -1;
		}
		
  /* Write transmitting packet into Tx buffer */
  ADF7242_PKT_write(pTxbuf, txlen);
  
  if(TRX_config_params.ieee_802_15_4_oper_mode & IEEE_802_15_4_AUTO_CSMACA){
    /* RC_CSMACA command is valid when the ADF7242 is in PHY_DRY state. Hence
     first issue PHY_RDY command. */
    //ADF7242_issue_command(CMD_RC_PHY_RDY);
    /* Wait untill TRX state becomes PHY_RDY */
    //ADF7242_wait_state(RC_PHY_RDY_MASK);
		
		
    /* Issue RC_CSMACA command to start transmission */
    ADF7242_issue_command(CMD_RC_CSMACA);
  }
  else {
    /* Issue RC_TX command to start transmission */
    ADF7242_issue_command(CMD_RC_TX);
  }
	return 0;
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Read received packet from ADF7242. Read RSSI and LQI of received 
 *         packet. Give indication to higher layer.
 *
 * @param   None.
 *
 * @return  None.
 */
void ADF7242_rx(void)
{
  uint8_t rssi = 0, lqi = 0;
  uint16_t rxlen;
  /* Read received packet from RX_buffer of ADF7242 */
  ADF7242_PKT_read(rx_buff, &rxlen);
    
  if(TRX_config_params.phy_mode == IEEE_802_15_4_packet_mode){
     /* Here rxlen is PHY_payload(including CRC). Here PHY payload starts from 
    rx_buff + 3. RSSI is stored in first byte of CRC. Hence to get rssi 
    rx_buff[rxlen + 2 + 1 -2].*/
    rssi = rx_buff[rxlen + 1];
    lqi = rx_buff[rxlen + 2];
  }  
	//MAC_ACK: 0x02
	if((rx_buff[3] & 0x07) != 0x02){
  /* Give indication to higher layer */
  ADF7242_rx_done(rx_buff + 2, rxlen, rssi, lqi);
	}
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  ADF7242 interrupt handler.
 *
 * @param   None.
 *
 * @return  None.
 */
void ADF7242_irq_handler(void)
{
  uint8_t irq_status, irq_status0;
 
  /* Read irq triggered event.*/
  ADF7242_MEM_read(REG_IRQ1_SRC1, 1, &irq_status);
	ADF7242_MEM_read(REG_IRQ1_SRC0, 1, &irq_status0);
  /* Clear irq */ 
  ADF7242_REG_write(REG_IRQ1_SRC1, irq_status);
	ADF7242_REG_write(REG_IRQ1_SRC0, irq_status0);
  /* Handle transmission complete interrupt. */
  if(((TRX_config_params.ieee_802_15_4_oper_mode & IEEE_802_15_4_AUTO_CSMACA) && (irq_status & IRQ_CSMA_CA) ) || 
		(!(TRX_config_params.ieee_802_15_4_oper_mode & IEEE_802_15_4_AUTO_CSMACA) && (irq_status & IRQ_TX_PKT_SENT) )) {
    uint8_t status = 0;
    if(TRX_config_params.ieee_802_15_4_oper_mode & IEEE_802_15_4_AUTO_CSMACA){
      /* Read auto_status register for tx status */
      ADF7242_MEM_read(REG_AUTO_STATUS, 1, &status);
    }
    /* If CSMA-CA auto mode enabled and RX on when idle is enabled, then no need 
     to issue RC_RX command beacuse while PHY_init we are enabling 
    csama_ca_turnaround. 
    If it is no auto CSMA-CA and RX on when idle then we need to issue RC_RX 
    command.*/
    if(TRX_config_params.ieee_802_15_4_oper_mode & IEEE_802_15_4_RX_ON_WHEN_IDLE) {
      if(!(TRX_config_params.ieee_802_15_4_oper_mode & IEEE_802_15_4_AUTO_CSMACA)){
        ADF7242_issue_command(CMD_RC_RX);
				ADF7242_wait_state(RC_RX_MASK);
			}
			if(TRX_config_params.ieee_802_15_4_oper_mode & IEEE_802_15_4_AUTO_CSMACA){
				if(!((status & 0x07) == TX_SUCCESS) &&
					!((status & 0x07) == TX_SUCCESS_DATA_PEND)){
					ADF7242_issue_command(CMD_RC_RX);
				  ADF7242_wait_state(RC_RX_MASK);
				}
			}
			
    }
    /* Indicate higher layer about transmission status */
    ADF7242_tx_done(status); 

  }
  /* Handle packet reception interrupt. */
  if((irq_status & IRQ_RX_PKT_RCVD)) {
    ADF7242_rx();
   
		if((rx_buff[3] & 0x07) != 0x02){
    if(TRX_config_params.ieee_802_15_4_oper_mode & IEEE_802_15_4_RX_ON_WHEN_IDLE){
			//uint8_t status;
			//status = ADF7242_read_status();
			//if(!(status & RC_RX_MASK)){
    /*After reading received packet, turn on the receiver.*/
    ADF7242_wait_state(RC_PHY_RDY_MASK);
    ADF7242_issue_command(CMD_RC_RX);
		ADF7242_wait_state(RC_RX_MASK);
		//}
    }
	}		
  }
#ifdef ADF7242_DEBUG_d
  if(irq_status & IRQ_RX_PKT_RCVD)   g_rx_pkt++;
  if(irq_status & IRQ_SFD_RX)   g_rx_sfd++;
  if(irq_status & IRQ_FRAME_VALID)  g_rx_frame_valid++;
  if(irq_status & IRQ_ADDRESS_VALID)  g_rx_addr_valid++;
#endif
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Turn on ADF7242 TRX receiver on.
 *
 * @param   None.
 *
 * @return  Status of trx receiver on.
 *          - #0  If able to on TRX receiver.
 */
ADF7242_Result_t ADF7242_radio_rx_on(void)
{
  /*TBD: Need to check whether trx is in busy state. */
  ADF7242_issue_command(CMD_RC_RX);

  return ADF_SUCCESS;
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Initializes ADF7242 TRX based on given PHY mode configurations.
 *
 * @param[phy_mode]   PHY mode.
 *
 * @return  None.
 */
static void ADF7242_PHY_init(ADF7242_phy_mode_t phy_mode)
{
  if(phy_mode == IEEE_802_15_4_packet_mode){
    uint8_t irq_en;
    
    ADF7242_REG_write(REG_BUFFERCFG, 0x00); //default setting
#ifdef FIRMWARE_ENABLE_d
    /* First upload firmware using CMD_SPI_PRAM_WR */
    ADf7242_upload_firmware();
    
    /* Set Automatic IEEE 802.15.4 Mode Registers. */
    ieee_802_15_4_mode_regs.ffilt_cfg = (ACCEPT_BEACON_FRAMES | 
                                        ACCEPT_DATA_FRAMES |  
                                        ACCEPT_MACCMD_FRAMES 
                                          
                                            ); //ACCEPT_RESERVED_FRAMESACCEPT_ACK_FRAMES | ACCEPT_ALL_ADDRESS Address filtering disables
    
     
    if(TRX_config_params.ieee_802_15_4_oper_mode & IEEE_802_15_4_RX_AUTO_ACK){
      ieee_802_15_4_mode_regs.auto_cfg = RX_AUTO_ACK_EN;
    }
    if(TRX_config_params.ieee_802_15_4_oper_mode & IEEE_802_15_4_RX_ON_WHEN_IDLE){
      ieee_802_15_4_mode_regs.auto_cfg |= CSMA_CA_TURNAROUND;
    }
  
    
    ADF7242_update_802_15_4_mode_regs();
     
    /* Write 0x8D to MCR Register 0x3FB and write 0xCA to MCR Register 0x3FC. */
    ADF7242_REG_write(0x3FB, 0x8D); 
    ADF7242_REG_write(0x3FC, 0xCA);
    
    /* Enable firmware add-on module by setting pkt_cfg.addon_en bit.
    By default Auto FCS appending and validation is enabled.*/
    ADF7242_REG_write(REG_PKT_CFG, (ADDON_EN | PKT_CFG_RESERVED) );
   
    /* Enable interrupt based on IEEE operating mode. */
    ADF7242_REG_write(REG_IRQ1_EN0, 0x00);
    if(TRX_config_params.ieee_802_15_4_oper_mode & IEEE_802_15_4_AUTO_CSMACA){
      irq_en = IRQ_CSMA_CA;
    }
    else {
      irq_en = IRQ_TX_PKT_SENT;
    }
   
#ifdef ADF7242_DEBUG_d
//    ADF7242_REG_write(REG_IRQ1_EN1, (IRQ_RX_PKT_RCVD | IRQ_SFD_RX | 
//                                     IRQ_FRAME_VALID | IRQ_ADDRESS_VALID | 
//                                     irq_en));
    ADF7242_REG_write(REG_IRQ1_EN1, (IRQ_RX_PKT_RCVD | irq_en));
#else
    ADF7242_REG_write(REG_IRQ1_EN1, (IRQ_RX_PKT_RCVD | irq_en));
#endif 
    
    /* Clear source interrupt registers */
    ADF7242_REG_write(REG_IRQ1_SRC1, 0xFF);
    ADF7242_REG_write(REG_IRQ1_SRC0, 0xFF);
 
/* If firmware not uploaded */
#else 
    ADF7242_REG_write(REG_IRQ1_EN0, 0x00);
    ADF7242_REG_write(REG_IRQ1_EN1, (IRQ_RX_PKT_RCVD | IRQ_TX_PKT_SENT));
    /* Clear source interrupt registers */
    ADF7242_REG_write(REG_IRQ1_SRC1, 0xFF);
    ADF7242_REG_write(REG_IRQ1_SRC0, 0xFF);
#endif /* FIRMWARE_ENABLE_d */
  }
  
#ifdef SUPPORT_GFSK_FSK_d  
  else if(mode == GFSK_FSK_packet_mode) {
    /* Initialize GFSK/FSK packet mode (rc_cfg.rc_mode) */
    ADF7242_MEM_write(rc_cfg);    
    /* Initialize all the registers required for GFSK/FSK mode. */
    ADF7242_update_GFSK_mode_regs(); 
    /* Enable interrupt */
    ADF7242_REG_write(REG_IRQ1_EN0, 0x00);
    ADF7242_REG_write(REG_IRQ1_EN1, IRQ_RX_PKT_RCVD | IRQ_SFD_RX | IRQ_TX_PKT_SENT);
    /* Clear source interrupt registers */
    ADF7242_REG_write(REG_IRQ1_SRC0, 0xFF);
    ADF7242_REG_write(REG_IRQ1_SRC1, 0xFF);
  }
#endif
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Uploads given firmware into ADF7242 program RAM.
 *
 * @param   None.
 *
 * @return  None.
 */
static void ADf7242_upload_firmware(void)
{
  uint16_t i;
  uint8_t page_num;
  uint16_t firmware_len = sizeof(adf7242_firmware);
  uint16_t block_size;
  
  for(page_num = 0; page_num <= (firmware_len/PRAM_PAGESIZE) ; page_num++){
    ADF7242_REG_write(REG_PRAMPG, page_num);
    
    block_size = (page_num < (firmware_len/PRAM_PAGESIZE))?
                        PRAM_PAGESIZE:(firmware_len - (page_num * PRAM_PAGESIZE));
    spi_txbuff[0] = CMD_SPI_PRAM_WR;
    spi_txbuff[1] = 0x00;
    for(i = 0; i < block_size ; i++){
      spi_txbuff[2 + i] = adf7242_firmware[i + page_num * PRAM_PAGESIZE ];
    }
    rf_spi_transaction(spi_txbuff, 2 + block_size , rx_buff, 0);
  }
  

#ifdef  ADF7242_DEBUG_d  
  /* Verify uploaded firmware */
  for(page_num = 0; page_num <= (firmware_len/PRAM_PAGESIZE) ; page_num++){
    ADF7242_REG_write(REG_PRAMPG, page_num);
    
    block_size = (page_num < (firmware_len/PRAM_PAGESIZE))?
                        PRAM_PAGESIZE:(firmware_len - (page_num * PRAM_PAGESIZE));
    spi_txbuff[0] = CMD_SPI_PRAM_RD;
    spi_txbuff[1] = 0x00;
    spi_txbuff[2] = CMD_SPI_NOP;
    for(i = 0; i < block_size ; i++){
      spi_txbuff[3 + i] = CMD_SPI_NOP;
    }
    rf_spi_transaction(spi_txbuff, 3 + block_size , rx_buff, 3 + block_size);
    for(i = 0; i < block_size; i++){
      if(rx_buff[3+i] != adf7242_firmware[i + page_num * PRAM_PAGESIZE ]){
        while(1);
      }
    }
  }
#endif /* ADF7242_DEBUG_d */
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Sets given channel frequency.
 *
 * @param[channel]   Channel frequency.
 *
 * @return  None.
 */
void ADF7242_set_channel(uint32_t channel)
{
  
  /* Convert frequency into 3 byte frequency format */
  channel = (uint32_t)(channel /10000);
  
  spi_txbuff[0] = CMD_SPI_MEM_WR(REG_CH_FREQ0);
  spi_txbuff[1] = REG_CH_FREQ0 & 0xFF;
  spi_txbuff[2] = (channel >> 0) & 0xFF;
  spi_txbuff[3] = (channel >> 8) & 0xFF;
  spi_txbuff[4] = (channel >> 16) & 0xFF;

  rf_spi_transaction(spi_txbuff, 5, rx_buff, 5);

#ifdef ADF7242_DEBUG_d  
  /* Read the channel back*/
  spi_txbuff[0] = CMD_SPI_MEM_RD(REG_CH_FREQ0);
  spi_txbuff[1] = REG_CH_FREQ0 & 0xFF;
  spi_txbuff[2] = CMD_SPI_NOP;
  spi_txbuff[3] = CMD_SPI_NOP;
  spi_txbuff[4] = CMD_SPI_NOP; 
  rf_spi_transaction(spi_txbuff, 5, rx_buff, 5);
#endif /* ADF7242_DEBUG_d */
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Writes ieee_802_15_4_mode_regs structure into automatic IEEE 802.15.4 
 *         mode registers staring from register address 0x112 to 0x121.
 *
 * @param   None.
 *
 * @return  None.
 */
void ADF7242_update_802_15_4_mode_regs(void)
{
  ADF7242_MEM_block_write(REG_PAN_ID0, 
                          (uint8_t *)&ieee_802_15_4_mode_regs, 
                          sizeof(ieee_802_15_4_mode_regs));
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Sets PA level.
 *
 * @param[power]   New PA Level to be set. 
 *                minimum power: 3. maximum power: 15.
 *                Nominal power step size 2 dB per LSB.
 *
 * @return  None.
 */
void ADF7242_SetPALevel(uint8_t power)
{
  uint8_t pa_level;
  
  pa_level =  (extpa_bias_mode << extpa_bias_mode_offset) |
    (extpa_bias_src << extpa_bias_src_offset) | 
      (power << pa_pwr_offset);
  ADF7242_REG_write(REG_EXTPA_MSC, pa_level);
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Put ADF7242 in RC_SLEEP state.
 *
 * @param   None.
 *
 * @return  None.
 */
void ADF7242_goto_sleep_state(void)
{
  uint32_t i = 100000;
  //@RF_IF_debug
  ADF7242_REG_write(REG_TMR_CFG1, 0x08);
  /*From any state by issuing RC_SLEEP command ADF7242 enters into sleep state*/
  ADF7242_issue_command(CMD_RC_SLEEP);
  /* wait for 2ms after issuing SLEEP command.*/
  //rf_delay(2);
  while(i)
  {
    i--;
  }
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Wake up from sleep state
 *
 * @param   None.
 *
 * @return  None.
 */
void ADF7242_wakeup_from_sleep(void)
{
  /* As per ADF7242 data sheet: The host MCU can bring CS low at any time to 
  wake the ADF7242 from the sleep state. */
  /* As per AN-1082: PRAM block is volatile memory and must be reloaded each 
  time the tranceiver is waking up from sleep state.*/
  
  /* Make CS as low and wait untill MISO output goes high.*/
  if(PhyWakeUp() != 0) while(1);
  
    /* Initialize configured PHY mode */
  ADF7242_PHY_init(TRX_config_params.phy_mode);
  
  /* Initialize channel frequency */
  ADF7242_set_channel(TRX_config_params.channelFreq);
     
  /* Initialize TRX to ready state */
  ADF7242_issue_command(CMD_RC_PHY_RDY);
  ADF7242_wait_state(RC_PHY_RDY_MASK);
}
/*----------------------------------------------------------------------------*/

#ifdef SUPPORT_GFSK_FSK_d
void ADF7242_update_GFSK_mode_regs(void)
{
  /* Use block write insted of writing each register, if possible */
  ADF7242_MEM_block_write(reg_addr, values, size);
}
#endif

#ifdef ADF7242_DEBUG_d

void ADF7242_get_BBRAM_dump(uint8_t *buff, uint16_t *buff_len){

  uint16_t i;
  /* REG_EXT_CTRL is start of BBRAM memory address.*/
  spi_txbuff[0] = CMD_SPI_MEM_RD(REG_EXT_CTRL);
  spi_txbuff[1] = REG_EXT_CTRL & 0xFF;
  spi_txbuff[2] = CMD_SPI_NOP;
  for(i =0; i < BBRAM_SIZE; i++){
    spi_txbuff[3+i] = CMD_SPI_NOP;
  }
  rf_spi_transaction(spi_txbuff, BBRAM_SIZE + 3, rx_buff, BBRAM_SIZE + 3);
  
  for(i = 0; i < BBRAM_SIZE; i++){
    buff[i] = rx_buff[i+3];
  }
  *buff_len = BBRAM_SIZE;
}

void ADF7242_get_MCR_dump(uint8_t *buff, uint16_t *buff_len){

  uint16_t i;
  
  /* REG_CH_FREQ0 is start of MCR register memory address.*/
  spi_txbuff[0] = CMD_SPI_MEM_RD(REG_CH_FREQ0);
  spi_txbuff[1] = REG_CH_FREQ0 & 0xFF;
  spi_txbuff[2] = CMD_SPI_NOP;
  for(i =0; i < MCR_REG_SIZE; i++){
    spi_txbuff[3+i] = CMD_SPI_NOP;
  }
  rf_spi_transaction(spi_txbuff, MCR_REG_SIZE + 3, rx_buff, MCR_REG_SIZE + 3);
  
  for(i = 0; i < MCR_REG_SIZE; i++){
    buff[i] = rx_buff[i+3];
  }
  *buff_len = MCR_REG_SIZE;
  
}
#endif
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Resets ADF7242.
 *
 * @param   None.
 *
 * @returns  None.   
 */
void ADF7242_Reset(void)
{
  uint32_t i = 100000;
  /* Issue reset command */
  ADF7242_issue_command(CMD_RC_RESET);
    /* delay of 2ms after issuing the command*/
  while(i)
  {
    i--;
  }
  /* Make CS as low and wait untill MISO output goes high.*/
  if(PhyWakeUp() != 0) while(1);
  /* Wait untill TRX becomes ready to access new commands.*/
  ADF7242_wait_state(RC_READY_MASK);
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Gives whether TRX is busy or not.
 *
 * @param   None.
 *
 * @returns  1  - If TRX is busy.
 *           0  - If TRX is not busy.
 */
uint8_t  ADF7242_is_trx_busy(void)
{
  uint8_t rc_status;
  uint8_t rx_status;
  
  /* Read current state of TRX. */
  rc_status = ADF7242_read_status();
  if ((rc_status & RC_TX_MASK) == RC_TX_MASK) {
    /* ADF7242 is busy in transmitting a packet. */
    return 1;
  }
  else if((rc_status & RC_RX_MASK) == RC_RX_MASK) {
    /* ADF7242 is in receiver mode check is it busy in receiving a packet.*/
    ADF7242_MEM_read(REG_802_15_4_STATE, 1, &rx_status);
    if((rx_status & DS_15d4_STATE_EXTRACT_DATA) == DS_15d4_STATE_EXTRACT_DATA){
      /* ADF7242 is busy in receiving a packet. */
      return 1;
    }
  }
  return 0;
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Gives AUTO_ACK_FRAMEPEND bit value.
 *
 * @param   None.
 *
 * @returns  1  - If AUTO_ACK_FRAMEPEND bit is 1.
 *           0  - If AUTO_ACK_FRAMEPEND bit is 0.
 */
uint8_t ADF7242_get_ack_frame_pend(void)
{
  uint8_t auto_cfg;
  ADF7242_MEM_read(REG_AUTO_CFG, 1, &auto_cfg); 
  return ((auto_cfg & AUTO_ACK_FRAMEPEND)?0x01:0x00);
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Sets RF driver to Energy detection mode.
 *
 * @param   None.
 *
 * @returns  None.   
 */
void ADF7242_set_ED_mode(void)
{
  /* Set rx_buffer_mode to 2. In this mode received frames are not written to RX 
  buffer.*/
  ADF7242_REG_write(REG_BUFFERCFG, 0x02);
  /* Turn on the receiver.*/
  ADF7242_issue_command(CMD_RC_PHY_RDY);
  ADF7242_wait_state(RC_PHY_RDY_MASK);
  ADF7242_issue_command(CMD_RC_RX);
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  This function reads RSSI and returns ED value.
 *
 * @param   None.
 *
 * @returns  ED value.   
 */
uint8_t ADF7242_get_channel_energy(void)
{
  int8_t rssi;
  uint8_t ed_val = 0;
  
  /* Read RSSI. */
  ADF7242_MEM_read(REG_RRB, 1, (uint8_t *)&rssi);
  /* Convert RSSI into ED value. */
  if ( rssi > (RX_SENSITIVITY + 10) )
  {
    /* Ed value is rssi in dbm above, 10dbm above rx sensitivity times ED 
    equivalent of 1 dbm RSSI. */
    ed_val = ( rssi - (RX_SENSITIVITY + 10) ) * MAX_ED_VAL/(RSSI_UL - RSSI_LL);
  }
  return ed_val;
}

/**********************Static function definations*****************************/

/*!
 * @brief  Writes given command into ADF7242.
 *
 * @param[in]   command   Command.
 *
 * @return  None.
 */
static void ADF7242_issue_command(uint8_t command)
{
  /* wait untill status becomes RC_READY */ 
  ADF7242_wait_state(RC_READY_MASK);
  /* Issue command to ADF7242 */
  spi_txbuff[0] = command;
  //rf_spi_write_fast(spi_txbuff, 1);
  rf_spi_transaction(spi_txbuff, 1, rx_buff, 0);
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Writes given data into specified register.
 *
 * @param[in]   reg_addr   Rgister address to which data needs to be write.
 *
 * @param[in]   data       Data which needs to write into ADF7242 register.
 *
 * @return  None.
 */
static void ADF7242_REG_write(uint16_t reg_addr, uint8_t data)
{
  spi_txbuff[0] = CMD_SPI_MEM_WR(reg_addr);
  spi_txbuff[1] = reg_addr;
  spi_txbuff[2] = data;
  //rf_spi_write_fast(spi_txbuff, 3);
	rf_spi_transaction(spi_txbuff, 3, rx_buff, 0);
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Writes given data from specified memory address.
 *
 * @param[in]   mem_addr   Memory address from where data needs to write.
 *
 * @param[in]   pData     Pointer to data which needs to write into ADF7242 mem.
 *
 * @param[in]   size      Number of bytes required to write into memory.
 *
 * @return  None.
 */
static void ADF7242_MEM_block_write(uint16_t mem_addr, 
                                    uint8_t* pData, uint16_t size)
{
  uint16_t i;
  
  spi_txbuff[0] = CMD_SPI_MEM_WR(mem_addr);
  spi_txbuff[1] = mem_addr;
  for(i = 0; i < size; i++){
    spi_txbuff[2 + i] = *pData++;
  }
  rf_spi_transaction(spi_txbuff, size + 2, rx_buff, 0);
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Writes given packet into ADF7242 transmit buffer. Here size is 
 *         considered as PHR of the transmitting packet and content of buffer 
 *         as PHY payload.
 *
 * @param[in]   pTxbuff   Pointer to tx buffer.
 *
 * @param[in]   size      Number of bytes required to write into frame buffer.
 *
 * @return  None.
 */
static void ADF7242_PKT_write(uint8_t* pTxbuff, uint16_t size)
{
  uint16_t i;
  /* @debug: Consider whether auto CRC is enabled. based on that pass length */
  spi_txbuff[0] = CMD_SPI_PKT_WR;
  spi_txbuff[1] = size;
  for(i = 0; i < (size -2); i++) {
    spi_txbuff[2 + i] = *pTxbuff++;
  }
  rf_spi_transaction(spi_txbuff, size, rx_buff, 0);
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Reads received packet from ADF7242 receiver buffer.
 *
 * @param[out]   pTxbuff   Pointer to rx buffer.
 *
 * @param[out]   size      Number of bytes read from ADF7242 receive buffer.
 *
 * @return  None.
 */
static void ADF7242_PKT_read(uint8_t* pRxbuff, uint16_t* size)
{
  uint16_t i;
  /* @debug: Consider whether auto CRC is enabled. based on that pass length */

  /* Read length */
  spi_txbuff[0] = CMD_SPI_PKT_RD;
  spi_txbuff[1] = CMD_SPI_NOP;
  spi_txbuff[2] = CMD_SPI_NOP;
  rf_spi_transaction(spi_txbuff, 3, pRxbuff, 3);
  *size = pRxbuff[2];

  spi_txbuff[0] = CMD_SPI_PKT_RD;
  spi_txbuff[1] = CMD_SPI_NOP;
  spi_txbuff[2] = CMD_SPI_NOP;
  for(i = 0; i <= *size; i++) {
    spi_txbuff[3 + i] = CMD_SPI_NOP;
  }
  rf_spi_transaction(spi_txbuff, *size + 4, pRxbuff, *size + 4);

}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Reads register or memory contents from specified address.
 *
 * @param[in]   mem_addr  Memory address.
 *
 * @param[in]   size      Number of bytes required to read from memory.
 *
 * @param[out]  pData     Output pointer which holds read data.
 *
 * @return  None.
 */
static void ADF7242_MEM_read(uint16_t mem_addr, uint16_t size, uint8_t* pData)
{
  uint16_t i;
  
  spi_txbuff[0] = CMD_SPI_MEM_RD(mem_addr);
  spi_txbuff[1] = mem_addr;
  spi_txbuff[2] = CMD_SPI_NOP;
  for (i = 0; i < size; i++) {
    spi_txbuff[3 + i] = CMD_SPI_NOP;
  }
  rf_spi_transaction(spi_txbuff, 3 + size, rx_buff, 3 + size);
  for (i = 0; i < size; i++) {
    *pData++ = rx_buff[3 + i];
  }
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Reads ADF7242 status and waits untill trx becomes given state.
 *
 * @param[in]   state_mask      Required ADF7242 status mask.
 *
 * @return  0   After reaching the specified state.
 */
static uint8_t ADF7242_wait_state(uint8_t state_mask)
{
  uint8_t status =0;
  do {
    status = ADF7242_read_status();
  }while((status & state_mask) != state_mask);
  return 0;
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Reads ADF7242 status.
 *
 * @param   None.
 *
 * @return  ADF7242 status.
 */
static uint8_t ADF7242_read_status(void)
{
  uint8_t status = 0;
  spi_txbuff[0] = CMD_SPI_NOP;
  rf_spi_transaction(spi_txbuff, 1, &status, 1);
  return status;
}
