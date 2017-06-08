/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for radio communication.
 *
 */

#include <openthread/types.h>
#include <openthread-config.h>
#include "openthread/openthread.h"
#include "openthread/platform/platform.h"
#include "openthread/platform/radio.h"
#include "openthread/platform/diag.h"
#include "utils/code_utils.h"
#include <string.h>

#include "platform-aducm3029.h"
#include "ADF7242_Config.h"
#include "ADF7242.h"

static uint8_t eui64[OT_EXT_ADDRESS_SIZE] = {0x00, 0x05, 0xf7, 0xfe, 0xff, 0x00, 0x00, 0x02};

static otRadioState sRadioState = OT_RADIO_STATE_DISABLED;
static otRadioFrame sTxFrame;
static otRadioFrame sRxFrame;
static bool sTxDone = false;
static bool sRxDone = false;
static bool sAckFramePend = false;
static otError sTxStatus;
static uint8_t sTxData[OT_RADIO_FRAME_MAX_SIZE];
static uint8_t sRxData[OT_RADIO_FRAME_MAX_SIZE];
static uint8_t sShortAddrCount = 0;
static uint8_t sExtAddrCount = 0;

static uint8_t getPaPowerFromdBm(int8_t txPower)
{
    uint8_t pa_power;
    
    switch(txPower)
    {
        case -22:
        case -21:
            pa_power = 3;
            break;

        case -20:
        case -19:
            pa_power = 4;
            break;

        case -18:
        case -17:
            pa_power = 5;
            break;

        case -16:
        case -15:
            pa_power = 6;
            break;

        case -14:
        case -13:
            pa_power = 7;
            break;

        case -12:
        case -11:
            pa_power = 8;
            break;

        case -10:
        case -9:
            pa_power = 9;
            break;

        case -8:
        case -7:
            pa_power = 10;
            break;

        case -6:
        case -5:
            pa_power = 11;
            break;

        case -4:
        case -3:
            pa_power = 12;
            break;

        case -2:
        case -1:
            pa_power = 13;
            break;

        case 0:
        case 1:
            pa_power = 14;
            break;

        case 2:
        case 3:
	default:
            pa_power = 15;
            break;
    }
    return pa_power;   
}

static void updateFramePendBit(void)
{
   /* Set the frame pending bit, if source match entry is present */
   if(sShortAddrCount > 0 || sExtAddrCount > 0)
   {
       /* Check whether already set, if not then set frame pending bit */
       if((ieee_802_15_4_mode_regs.auto_cfg & AUTO_ACK_FRAMEPEND) != AUTO_ACK_FRAMEPEND)
       {
           ieee_802_15_4_mode_regs.auto_cfg |= AUTO_ACK_FRAMEPEND;
           ADF7242_update_802_15_4_mode_regs();
       }
   }
   /* Clear the frame pending bit, if source match entry is empty */
   if(sShortAddrCount == 0 && sExtAddrCount == 0)
   {
       /* Check whether already cleared, if not then clear frame pending bit */
       if((ieee_802_15_4_mode_regs.auto_cfg & AUTO_ACK_FRAMEPEND) != 0)
       {
           ieee_802_15_4_mode_regs.auto_cfg &= ~(AUTO_ACK_FRAMEPEND);
           ADF7242_update_802_15_4_mode_regs();
       }

   }
}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    uint8_t i;
    (void)aInstance;

    /* Stored in network byte order */
    for (i = 0; i < OT_EXT_ADDRESS_SIZE; i++)
    {
        aIeeeEui64[i] = eui64[(OT_EXT_ADDRESS_SIZE - 1) - i];
    }
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t panid)
{
    (void)aInstance;
    if(sRadioState != OT_RADIO_STATE_DISABLED)
    {
        ieee_802_15_4_mode_regs.pan_id = panid; 
        ADF7242_update_802_15_4_mode_regs();
    }
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, uint8_t *address)
{
    (void)aInstance;
    if(sRadioState != OT_RADIO_STATE_DISABLED)
    {
        memcpy(ieee_802_15_4_mode_regs.ieee_addr, address, 8);
        ADF7242_update_802_15_4_mode_regs();
    }
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t address)
{
    (void)aInstance;
    if(sRadioState != OT_RADIO_STATE_DISABLED)
    {
        ieee_802_15_4_mode_regs.short_addr = address;
        ADF7242_update_802_15_4_mode_regs();
    }
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
    (void)aInstance;
    return sRadioState;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    (void)aInstance;
    
    ADF7242_goto_sleep_state();
    sRadioState = OT_RADIO_STATE_SLEEP;

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    (void)aInstance;

    ADF7242_Reset();
    sRadioState = OT_RADIO_STATE_DISABLED;

    return OT_ERROR_NONE;
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    (void)aInstance;
    return (sRadioState != OT_RADIO_STATE_DISABLED);
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    (void)aInstance;

    if(sRadioState == OT_RADIO_STATE_DISABLED) 
    {
        return OT_ERROR_INVALID_STATE;
    }
    if(ADF7242_is_trx_busy())
    {
        return OT_ERROR_BUSY;
    }
    ADF7242_goto_sleep_state();
    sRadioState = OT_RADIO_STATE_SLEEP; 
    return OT_ERROR_NONE;
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    (void)aInstance;
    uint32_t freq = 2400000000;
    
    if((aChannel >= 11) && (aChannel <= 26))
    {
      /* Convert from channel number to absolute frequency value. */
      freq = (2405 + 5 * (aChannel - 11)) * 1000000;
    }    
   
    if(sRadioState == OT_RADIO_STATE_RECEIVE && 
       TRX_config_params.channelFreq == freq) 
    {
        return OT_ERROR_NONE;
    }
  
    if(sRadioState != OT_RADIO_STATE_SLEEP)
    {
        return OT_ERROR_INVALID_STATE;
    }

    /* Wakeup the TRX from sleep state*/
    ADF7242_wakeup_from_sleep();

    /* Set channel frequency */   
    TRX_config_params.channelFreq = freq;
    ADF7242_set_channel(TRX_config_params.channelFreq);

    /* Turn on TRX receiver */
    ADF7242_radio_rx_on();
    
    sRadioState = OT_RADIO_STATE_RECEIVE; 
    return OT_ERROR_NONE;
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    (void)aInstance;
    (void)aEnable;
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    (void)aInstance;
    (void)aShortAddress;
    
    if(sShortAddrCount < 255) 
        sShortAddrCount++; 
    updateFramePendBit();

    return OT_ERROR_NONE;
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    (void)aInstance;
    (void)aExtAddress;

    if(sExtAddrCount < 255) 
        sExtAddrCount++; 
    updateFramePendBit();

    return OT_ERROR_NONE;
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    (void)aInstance;
    (void)aShortAddress;

    if(sShortAddrCount > 0) 
        sShortAddrCount--; 
    updateFramePendBit();

    return OT_ERROR_NONE;
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    (void)aInstance;
    (void)aExtAddress;

    if(sExtAddrCount > 0) 
        sExtAddrCount--; 
    updateFramePendBit();

    return OT_ERROR_NONE;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    (void)aInstance;
  
    sShortAddrCount = 0; 
    updateFramePendBit();

}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    (void)aInstance;

    sExtAddrCount = 0;
    updateFramePendBit();
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    (void)aInstance;
    return &sTxFrame;
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
    (void)aInstance;
    uint32_t channelFreq;

    if(sRadioState != OT_RADIO_STATE_RECEIVE)
    {
        return OT_ERROR_INVALID_STATE;
    }

    /* If Radio is busy in receiving the packet, then return error */
    if(ADF7242_is_trx_busy())
    {
        sTxDone= true;
	sTxStatus = OT_ERROR_CHANNEL_ACCESS_FAILURE;

        return OT_ERROR_NONE;
    }

    channelFreq = (2405 + 5 * (aFrame->mChannel - 11)) * 1000000;

    /* If channnel and TxPower is different than previous, then set it here. */
    if((channelFreq != TRX_config_params.channelFreq) ||
       (aFrame->mPower != TRX_config_params.txPower)
      )
    {
       
       TRX_config_params.channelFreq = channelFreq;
       ADF7242_set_channel(TRX_config_params.channelFreq);
       otPlatRadioSetDefaultTxPower(aInstance, aFrame->mPower); 
    }

    /* Initiate packet transmission */
    sRadioState = OT_RADIO_STATE_TRANSMIT;
    if(ADF7242_tx(aFrame->mPsdu, aFrame->mLength + 2) != 0)
    {
        /* Packet transmission failed due to TRX busy */
        sRadioState = OT_RADIO_STATE_RECEIVE;

        sTxDone= true;
	sTxStatus = OT_ERROR_CHANNEL_ACCESS_FAILURE;

        return OT_ERROR_NONE;
    }

    return OT_ERROR_NONE;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    (void)aInstance;
    return sRxFrame.mPower;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    (void)aInstance;
    return (OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_TRANSMIT_RETRIES);
}

void otPlatRadioSetDefaultTxPower(otInstance *aInstance, int8_t aPower)
{
    (void)aInstance;
    uint8_t pa_power;

    TRX_config_params.txPower = aPower;
    pa_power = getPaPowerFromdBm(aPower);
    ADF7242_SetPALevel(pa_power);
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    (void)aInstance;

    if(ieee_802_15_4_mode_regs.ffilt_cfg == ACCEPT_ALL_ADDRESS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    (void)aInstance;
    
    if(aEnable)
    {
        ieee_802_15_4_mode_regs.ffilt_cfg = ACCEPT_ALL_ADDRESS;
    }
    else
    {
        ieee_802_15_4_mode_regs.ffilt_cfg = ACCEPT_BEACON_FRAMES | 
                                            ACCEPT_DATA_FRAMES |  
                                            ACCEPT_MACCMD_FRAMES; 
    }
    ADF7242_update_802_15_4_mode_regs();
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    (void)aInstance;
    (void)aScanChannel;
    (void)aScanDuration;
    return OT_ERROR_NOT_IMPLEMENTED;
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    (void)aInstance;
    return RX_SENSITIVITY;
}
/*****************************************************************************/

void aducm3029RadioInit(void)
{
    ADF7242_init();
    sRadioState = OT_RADIO_STATE_DISABLED;
    /* Initialize Tx frame */
    sTxFrame.mPsdu = sTxData;
    sTxFrame.mLength = 0;
    sTxFrame.mChannel = (((TRX_config_params.channelFreq/1000000) / 5) - 2405) + 11;
    sTxFrame.mPower = TRX_config_params.txPower;
    sTxFrame.mLqi = 0;
    sTxFrame.mMaxTxAttempts = ADF7242_CONF_MAX_FRAME_RETRY;  
    sTxFrame.mSecurityValid = false;
    sTxFrame.mDidTX = false;
    sTxFrame.mIsARetx = false;
    /* Initialize Rx frame */
    sRxFrame.mPsdu = sRxData;
    sRxFrame.mLength = 0;
    sRxFrame.mChannel = (((TRX_config_params.channelFreq/1000000) / 5) - 2405) + 11;
    sRxFrame.mPower = TRX_config_params.txPower;
    sRxFrame.mLqi = 0;
    sRxFrame.mMaxTxAttempts = ADF7242_CONF_MAX_FRAME_RETRY;  
    sRxFrame.mSecurityValid = false;
    sRxFrame.mDidTX = false;
    sRxFrame.mIsARetx = false;

}
void aducm3029RadioProcess(otInstance *aInstance)
{
    if(sTxDone)
    {
        otPlatRadioTransmitDone(aInstance, &sTxFrame, sAckFramePend, sTxStatus);
        sTxDone = false;    
    }
    
    if(sRxDone)
    {
#if OPENTHREAD_ENABLE_DIAG

        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioReceiveDone(aInstance, &sRxFrame, OT_ERROR_NONE);
        }
        else
#endif
        {
            otPlatRadioReceiveDone(aInstance, &sRxFrame, OT_ERROR_NONE);
        }
        sRxDone = false;
    }
}

void ADF7242_rx_done(uint8_t *pRx_buf, uint16_t rxlen, int8_t rssi, uint8_t lqi)
{
    sRxDone = true;

    /* Copy data to Rx buffer */
    memcpy(sRxData, pRx_buf + 1, rxlen-2);

    /* Fill receive frame structure */
    sRxFrame.mPsdu = sRxData;
    sRxFrame.mLength = rxlen-2;
    sRxFrame.mChannel = (((TRX_config_params.channelFreq/1000000) / 5) - 2405) + 11;
    sRxFrame.mPower = rssi;
    sRxFrame.mLqi = lqi;
}

void ADF7242_tx_done(uint8_t status)
{
    (void)status;
    
    sTxDone= true;
    sRadioState = OT_RADIO_STATE_RECEIVE;
    sAckFramePend = false;
    
    switch(status)
    {
        case TX_SUCCESS:
            sTxStatus = OT_ERROR_NONE;
        break;
   
        case TX_SUCCESS_DATA_PEND:
            sTxStatus = OT_ERROR_NONE;
            sAckFramePend = true;
        break;

        case TX_FAILURE_CSMACA:
            sTxStatus = OT_ERROR_CHANNEL_ACCESS_FAILURE;
        break;

	case TX_FAILURE_NOACK:
            sTxStatus = OT_ERROR_NO_ACK;
        break;

	case TX_FAILURE_CFG:
        default:
            sTxStatus = OT_ERROR_ABORT;
        break;

    }
}
