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

#include "test_platform.h"
//#include <mac/mac_frame.hpp>

//#define DBG_FUZZ 1

bool g_fRadioEnabled = false;
uint8_t g_RecvChannel = 0;
uint8_t g_TransmitPsdu[128];
otRadioFrame g_TransmitRadioFrame;
bool g_fTransmit = false;

bool testFuzzRadioIsEnabled(otInstance *)
{
    return g_fRadioEnabled;
}

otError testFuzzRadioEnable(otInstance *)
{
#ifdef DBG_FUZZ
    Log("Radio enabled");
#endif
    g_fRadioEnabled = true;
    return OT_ERROR_NONE;
}

otError testFuzzRadioDisable(otInstance *)
{
#ifdef DBG_FUZZ
    Log("Radio disabled");
#endif
    g_fRadioEnabled = false;
    return OT_ERROR_NONE;
}

otError testFuzzRadioReceive(otInstance *, uint8_t aChannel)
{
#ifdef DBG_FUZZ
    Log("==> receive");
#endif
    g_RecvChannel = aChannel;
    return OT_ERROR_NONE;
}

otError testFuzzRadioTransmit(otInstance *)
{
#ifdef DBG_FUZZ
    Log("==> transmit");
#endif
    g_fTransmit = true;
    return OT_ERROR_NONE;
}

otRadioFrame *testFuzztRadioGetTransmitBuffer(otInstance *)
{
    return &g_TransmitRadioFrame;
}

void TestFuzz(uint32_t aSeconds)
{
    // Set the radio capabilities to disable any Mac related timer dependencies
    g_testPlatRadioCaps = (otRadioCaps)(OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_TRANSMIT_RETRIES);

    // Set the platform function pointers
    g_TransmitRadioFrame.mPsdu = g_TransmitPsdu;
    g_testPlatRadioIsEnabled = testFuzzRadioIsEnabled;
    g_testPlatRadioEnable = testFuzzRadioEnable;
    g_testPlatRadioDisable = testFuzzRadioDisable;
    g_testPlatRadioReceive = testFuzzRadioReceive;
    g_testPlatRadioTransmit = testFuzzRadioTransmit;
    g_testPlatRadioGetTransmitBuffer = testFuzztRadioGetTransmitBuffer;

    // Initialize our timing variables
    uint32_t tStart = otPlatAlarmGetNow();
    uint32_t tEnd = tStart + (aSeconds * 1000);

    otInstance *aInstance;

#ifdef _WIN32
    uint32_t seed = (uint32_t)time(NULL);
    srand(seed);
    Log("Initialized seed = 0x%X", seed);
#endif

#ifdef OPENTHREAD_MULTIPLE_INSTANCE
    size_t otInstanceBufferLength = 0;
    uint8_t *otInstanceBuffer = NULL;

    // Call to query the buffer size
    (void)otInstanceInit(NULL, &otInstanceBufferLength);

    // Call to allocate the buffer
    otInstanceBuffer = (uint8_t *)malloc(otInstanceBufferLength);
    VerifyOrQuit(otInstanceBuffer != NULL, "Failed to allocate otInstance");
    memset(otInstanceBuffer, 0, otInstanceBufferLength);

    // Initialize OpenThread with the buffer
    aInstance = otInstanceInit(otInstanceBuffer, &otInstanceBufferLength);
#else
    aInstance = otInstanceInit();
#endif

    VerifyOrQuit(aInstance != NULL, "Failed to initialize otInstance");

    // Start the Thread network
    otLinkSetPanId(aInstance, (otPanId)0xFACE);
    otIp6SetEnabled(aInstance, true);
    otThreadSetEnabled(aInstance, true);

    uint32_t countRecv = 0;

    while (otPlatAlarmGetNow() < tEnd)
    {
        otTaskletsProcess(aInstance);

        if (g_testPlatAlarmSet && otPlatAlarmGetNow() >= g_testPlatAlarmNext)
        {
            g_testPlatAlarmSet = false;
            otPlatAlarmFired(aInstance);
        }

        if (g_fRadioEnabled)
        {
            if (g_fTransmit)
            {
                g_fTransmit = false;
                otPlatRadioTxDone(aInstance, &g_TransmitRadioFrame, NULL, OT_ERROR_NONE);
#ifdef DBG_FUZZ
                Log("<== transmit");
#endif
            }

            if (g_RecvChannel != 0)
            {
                uint8_t fuzzRecvBuff[128];
                otRadioFrame fuzzPacket;

                // Initialize the radio packet with a random length
                memset(&fuzzPacket, 0, sizeof(fuzzPacket));
                fuzzPacket.mPsdu = fuzzRecvBuff;
                fuzzPacket.mChannel = g_RecvChannel;
                fuzzPacket.mLength = (uint8_t)(otPlatRandomGet() % 127);

                // Populate the length with random
                for (uint8_t i = 0; i < fuzzPacket.mLength; i++)
                {
                    fuzzRecvBuff[i] = (uint8_t)otPlatRandomGet();
                }

                // Clear the global flag
                g_RecvChannel = 0;

                // Indicate the receive complete
                otPlatRadioReceiveDone(aInstance, &fuzzPacket, OT_ERROR_NONE);

                countRecv++;
#ifdef DBG_FUZZ
                Log("<== receive (%llu, %u bytes)", countRecv, fuzzPacket.mLength);
#endif

                // Hack to get a receive poll immediately
                otLinkSetChannel(aInstance, 11);
            }
        }
    }

    Log("%u packets received", countRecv);

    // Clean up the instance
    otInstanceFinalize(aInstance);

#ifdef OPENTHREAD_MULTIPLE_INSTANCE
    free(otInstanceBuffer);
#endif
}

#ifdef ENABLE_TEST_MAIN
int main(void)
{
    TestFuzz(30);
    printf("All tests passed\n");
    return 0;
}
#endif
