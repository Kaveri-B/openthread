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
 *   This file implements a random number generator.
 *
 */

#include "openthread/types.h"
#include "openthread/platform/radio.h"
#include "openthread/platform/random.h"

#include "platform-aducm3029.h"
#include "drivers/rng/adi_rng.h"

/* RNG Device number */
#define RNG_DEV_NUM              (0u)

#define RNG_DEV_LEN_PRESCALER    (1u)

#define RNG_DEV_LEN_RELOAD       (256u)

/* RNG Device Handle */
static ADI_RNG_HANDLE shRngDevice;
/* Memory to handle CRC Device */
static uint8_t sRngDevMem[ADI_RNG_MEMORY_SIZE] __attribute__((aligned(4)));

void aducm3029RandomInit(void)
{
    adi_rng_Open(RNG_DEV_NUM, sRngDevMem, sizeof(sRngDevMem), &shRngDevice);
    adi_rng_SetSampleLen(shRngDevice, RNG_DEV_LEN_PRESCALER, RNG_DEV_LEN_RELOAD);
}

uint32_t otPlatRandomGet(void)
{
    bool bRNGRdy;
    uint32_t nRandomNum;
    uint8_t num_bytes = 0;
    uint32_t random = 0;

    adi_rng_Enable(shRngDevice, true);
    
    while ( num_bytes < 4)
    {
        adi_rng_GetRdyStatus(shRngDevice, &bRNGRdy);
        if (bRNGRdy) 
        {
            adi_rng_GetRngData(shRngDevice, &nRandomNum);
            random |= (nRandomNum & 0xFF) << (num_bytes * 8);
            num_bytes++;
        }
    }    
    adi_rng_Enable(shRngDevice, false);
    
    return random;
}

otError otPlatRandomGetTrue(uint8_t *aOutput, uint16_t aOutputLength)
{
    bool bRNGRdy;
    uint32_t nRandomNum;
    uint16_t num_bytes = 0;

    adi_rng_Enable(shRngDevice, true);
    
    while ( num_bytes < aOutputLength)
    {
        adi_rng_GetRdyStatus(shRngDevice, &bRNGRdy);
        if (bRNGRdy)
        {
            adi_rng_GetRngData(shRngDevice, &nRandomNum);
            aOutput[num_bytes++] = nRandomNum & 0xFF;
        }
    }    
    adi_rng_Enable(shRngDevice, false);
   
    return OT_ERROR_NONE;
}
