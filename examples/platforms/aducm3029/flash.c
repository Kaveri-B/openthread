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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include <openthread-config.h>
#include <openthread-core-aducm3029-config.h>
#include "utils/code_utils.h"
#include "utils/flash.h"

#include "platform-aducm3029.h"
#include <drivers/flash/adi_flash.h>

#define ADI_START_ADDR_OF_FLASH		(0x800u)
#define ADI_END_ADDR_OF_FLASH		(ADI_START_ADDR_OF_FLASH + (SETTINGS_CONFIG_PAGE_SIZE * SETTINGS_CONFIG_PAGE_NUM))
#define ADI_SUPPORTED_FLASH_SIZE	(ADI_END_ADDR_OF_FLASH - ADI_START_ADDR_OF_FLASH)

static uint8_t FlashDeviceMem[ADI_FEE_MEMORY_SIZE]  __attribute__ ((aligned (4)));
static ADI_FEE_HANDLE hFlashDevice = NULL;
static uint8_t sFlashWriteBuf[SETTINGS_CONFIG_PAGE_SIZE];

static uint32_t flashAddrMap(uint32_t addr)
{
    return (addr + ADI_START_ADDR_OF_FLASH);
}

otError utilsFlashInit(void)
{
    adi_fee_Open(0, FlashDeviceMem, sizeof(FlashDeviceMem), &hFlashDevice);
    return OT_ERROR_NONE;
}

uint32_t utilsFlashGetSize(void)
{
    return ADI_SUPPORTED_FLASH_SIZE;
}

otError utilsFlashErasePage(uint32_t aAddress)
{
    otError error = OT_ERROR_NONE;
    uint32_t hwError = 0u;
    uint32_t nStartPage;

    /* Check address is out of range */
    otEXPECT_ACTION(flashAddrMap(aAddress) < ADI_END_ADDR_OF_FLASH, error = OT_ERROR_INVALID_ARGS);
    adi_fee_GetPageNumber(hFlashDevice, aAddress, &nStartPage);
    otEXPECT_ACTION(adi_fee_PageErase (hFlashDevice, nStartPage, nStartPage, &hwError) == ADI_FEE_SUCCESS, error = OT_ERROR_FAILED) ;

exit:
    return error;
}

otError utilsFlashStatusWait(uint32_t aTimeout)
{
    (void)aTimeout;

    return OT_ERROR_NONE;
}

uint32_t utilsFlashWrite(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
    uint32_t hwError = 0u;
    ADI_FEE_TRANSACTION transfer;
    uint32_t nStartPage, nEndPage;
    uint32_t size = aSize;
    uint32_t index;
    uint8_t* pDataInFlash;
    uint8_t* pDataToFlash;
    uint32_t temp_size;

    /* Check address is out of range */
    otEXPECT_ACTION(flashAddrMap(aAddress) < (ADI_END_ADDR_OF_FLASH - 8), size = 0);
    
    /* Check size exceeds range */
    otEXPECT_ACTION(size < (ADI_END_ADDR_OF_FLASH - flashAddrMap(aAddress)) , size = 0);

    adi_fee_GetPageNumber(hFlashDevice, flashAddrMap(aAddress), &nStartPage);
    adi_fee_GetPageNumber(hFlashDevice, (flashAddrMap(aAddress) + size - 1), &nEndPage);

    /* Read data before erase */
    index = 0;
    pDataInFlash = (uint8_t *)(nStartPage * SETTINGS_CONFIG_PAGE_SIZE);
    while(index < SETTINGS_CONFIG_PAGE_SIZE)
    {
        sFlashWriteBuf[index++] = *pDataInFlash++;
    }
    
    otEXPECT_ACTION((adi_fee_PageErase (hFlashDevice, nStartPage, nStartPage, &hwError)) == ADI_FEE_SUCCESS, size = 0);

    /* Copy actual write bytes */
    pDataToFlash = aData;
    index = aAddress;
    if(index >= SETTINGS_CONFIG_PAGE_SIZE)
    {
    	index = (index % SETTINGS_CONFIG_PAGE_SIZE);
    }
    
    temp_size = 0;
    while((temp_size != aSize) && (index < SETTINGS_CONFIG_PAGE_SIZE))
    {
        sFlashWriteBuf[index] = *pDataToFlash++;
        index++;
        temp_size++;
    }
    
    /* configure write buffer */
    transfer.pWriteAddr = (uint32_t *)(nStartPage * SETTINGS_CONFIG_PAGE_SIZE);
    transfer.pWriteData = (uint32_t *)&sFlashWriteBuf[0];
    transfer.nSize      = SETTINGS_CONFIG_PAGE_SIZE;
    transfer.bUseDma    = false;

    /* Submit a buffer for writing */
    otEXPECT_ACTION((adi_fee_SubmitBuffer(hFlashDevice, &transfer)) == ADI_FEE_SUCCESS, size = 0);
   
    /* Wait till flash write completes */
    otEXPECT_ACTION((adi_fee_GetBuffer(hFlashDevice, &hwError)) == ADI_FEE_SUCCESS, size = 0);
    size = aSize;
    
exit:   
    return size;
}

uint32_t utilsFlashRead(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
    uint32_t size;
    uint8_t* pDataInFlash = (uint8_t*)(flashAddrMap(aAddress));

    /* Check address is out of range */
    otEXPECT_ACTION(flashAddrMap(aAddress) < ADI_END_ADDR_OF_FLASH, size = 0);

    /* Check maximum available size*/ 
    if(aSize > (ADI_END_ADDR_OF_FLASH - flashAddrMap(aAddress)))
    {
        aSize = (ADI_END_ADDR_OF_FLASH - flashAddrMap(aAddress));
    }
    size = 0;
    /* Read data */
    while(size < aSize)
    {
        aData[size] = *pDataInFlash++;
        size++;
    }

exit:    
    return size;
}
