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
 *   This file implements the OpenThread platform abstraction for UART communication.
 *
 */

#include <stddef.h>

#include "openthread/types.h"
#include "openthread/platform/uart.h"

#include "utils/code_utils.h"

#include "platform-aducm3029.h"
#include "drivers/uart/adi_uart.h"

#define UART_DEVICE_NUM	0
#define ADI_UART_MEMORY_SIZE	(ADI_UART_BIDIR_MEMORY_SIZE)
/* Select the boudrate divider for 115200 */
#define UART_DIV_C_115200       4
#define UART_DIV_M_115200       1
#define UART_DIV_N_115200       1563
#define UART_OSR_115200         3

enum
{
    kPlatformClock = 32000000,
    kBaudRate = 115200,
    kReceiveBufferSize = 128,
};

static void processReceive(void);
static void processTransmit(void);

static const uint8_t *sTransmitBuffer = NULL;
static uint16_t sTransmitLength = 0;

typedef struct RecvBuffer
{
    // The data buffer
    uint8_t mBuffer[kReceiveBufferSize];
    // The offset of the first item written to the list.
    uint16_t mHead;
    // The offset of the next item to be written to the list.
    uint16_t mTail;
} RecvBuffer;

static RecvBuffer sReceive;

static ADI_UART_HANDLE hUartDevice = NULL;
static uint8_t UartDeviceMem[ADI_UART_MEMORY_SIZE] __attribute__((aligned(4)));
static volatile bool uartTxDone = false;
static uint8_t uartRxbyte;

static void uartCallback(void *pCBParam, uint32_t Event, void *pArg)
{
    switch(Event){
        case ADI_UART_EVENT_RX_BUFFER_PROCESSED:

            // We can only write if incrementing mTail doesn't equal mHead
            if (sReceive.mHead != (sReceive.mTail + 1) % kReceiveBufferSize)
            {
                sReceive.mBuffer[sReceive.mTail] = uartRxbyte;
                sReceive.mTail = (sReceive.mTail + 1) % kReceiveBufferSize;
            }
            adi_uart_SubmitRxBuffer(hUartDevice, &uartRxbyte, 1, false);
            break;

        case ADI_UART_EVENT_TX_BUFFER_PROCESSED:
            uartTxDone = true;
            break;

        case ADI_UART_EVENT_NO_RX_BUFFER_EVENT:
            adi_uart_SubmitRxBuffer(hUartDevice, &uartRxbyte, 1, false);
            break;
        
        default:
            break;
    }
}

otError otPlatUartEnable(void)
{
    sReceive.mHead = 0;
    sReceive.mTail = 0;

    adi_uart_Open(UART_DEVICE_NUM, ADI_UART_DIR_BIDIRECTION, UartDeviceMem, ADI_UART_MEMORY_SIZE, &hUartDevice);
    adi_uart_SetConfiguration(hUartDevice, ADI_UART_NO_PARITY, ADI_UART_ONE_STOPBIT, ADI_UART_WORDLEN_8BITS);
    adi_uart_ConfigBaudRate(hUartDevice, UART_DIV_C_115200, UART_DIV_M_115200, UART_DIV_N_115200, UART_OSR_115200);       
    adi_uart_RegisterCallback(hUartDevice, uartCallback, NULL);
    adi_uart_SubmitRxBuffer(hUartDevice, &uartRxbyte, 1, false);
    return OT_ERROR_NONE;
}

otError otPlatUartDisable(void)
{
    adi_uart_Close(hUartDevice);
    return OT_ERROR_NONE;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(sTransmitBuffer == NULL, error = OT_ERROR_BUSY);

    sTransmitBuffer = aBuf;
    sTransmitLength = aBufLength;

exit:
    return error;
}

void processReceive(void)
{
   // Copy tail to prevent multiple reads
    uint16_t tail = sReceive.mTail;

    // If the data wraps around, process the first part
    if (sReceive.mHead > tail)
    {
        otPlatUartReceived(sReceive.mBuffer + sReceive.mHead, kReceiveBufferSize - sReceive.mHead);

        // Reset the buffer mHead back to zero.
        sReceive.mHead = 0;
    }

    // For any data remaining, process it
    if (sReceive.mHead != tail)
    {
        otPlatUartReceived(sReceive.mBuffer + sReceive.mHead, tail - sReceive.mHead);

        // Set mHead to the local tail we have cached
        sReceive.mHead = tail;
    }
}

void processTransmit(void)
{
    otEXPECT(sTransmitBuffer != NULL);
    
    uartTxDone = false;
    adi_uart_SubmitTxBuffer(hUartDevice, (uint8_t *)sTransmitBuffer, sTransmitLength, false);
    while(!uartTxDone);
    sTransmitLength = 0;                                       

    sTransmitBuffer = NULL;
    otPlatUartSendDone();

exit:
    return;
}

void aducm3029UartProcess(void)
{
    processReceive();
    processTransmit();
}

