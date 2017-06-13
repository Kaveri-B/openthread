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
 *   This file implements the OpenThread platform abstraction for the alarm.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include <openthread/config.h>
#include "openthread/openthread.h"
#include "openthread/platform/platform.h"
#include "openthread/platform/alarm.h"
#include "openthread/platform/diag.h"
#include "platform-aducm3029.h"
#include "drivers/rtc/adi_rtc.h"

#define RTC_DEV_NUM 1

static ADI_RTC_HANDLE hDeviceRTC = NULL;
static uint8_t rtc_dev_mem[ADI_RTC_MEMORY_SIZE];

static uint32_t sCounter = 0;
static uint32_t sAlarmT0 = 0;
static uint32_t sAlarmDt = 0;
static bool sIsRunning = false;

static void rtc_callback(void *hWut, uint32_t event, void *pArgs)
{
	//adi_rtc_Enable(hDeviceRTC, false);
	sCounter++;
	adi_rtc_SetCount(hDeviceRTC, 0);
	adi_rtc_SetAlarm(hDeviceRTC, 7);
	adi_rtc_EnableAlarm(hDeviceRTC, true);
	//adi_rtc_Enable(hDeviceRTC, true);
}

void aducm3029AlarmInit(void)
{
	ADI_RTC_INT_TYPE nInterrupts = ADI_RTC_ALARM_INT;
	 
	adi_rtc_Open(RTC_DEV_NUM, rtc_dev_mem, ADI_RTC_MEMORY_SIZE, &hDeviceRTC);
	adi_rtc_RegisterCallback(hDeviceRTC, rtc_callback, hDeviceRTC);
	adi_rtc_SetTrim(hDeviceRTC, ADI_RTC_TRIM_INTERVAL_15, ADI_RTC_TRIM_1, ADI_RTC_TRIM_SUB);
	adi_rtc_EnableTrim(hDeviceRTC, true);
	adi_rtc_EnableInterrupts(hDeviceRTC, nInterrupts, true);
	adi_rtc_SetPreScale(hDeviceRTC, 2);
	adi_rtc_SetCount(hDeviceRTC, 0);
	adi_rtc_SetAlarm(hDeviceRTC, 7);
	adi_rtc_EnableAlarm(hDeviceRTC, true);
	adi_rtc_Enable(hDeviceRTC, true);
}

uint32_t otPlatAlarmGetNow(void)
{
    return sCounter;
}

void otPlatAlarmStartAt(otInstance *aInstance, uint32_t t0, uint32_t dt)
{
    (void)aInstance;
    sAlarmT0 = t0;
    sAlarmDt = dt;
    sIsRunning = true;
}

void otPlatAlarmStop(otInstance *aInstance)
{
    (void)aInstance;
    adi_rtc_EnableAlarm(hDeviceRTC, false);
	adi_rtc_Enable(hDeviceRTC, false);
	sIsRunning = false;
}

void aducm3029AlarmProcess(otInstance *aInstance)
{
    uint32_t expires;
    bool fire = false;

    if (sIsRunning)
    {
        expires = sAlarmT0 + sAlarmDt;

        if (sAlarmT0 <= sCounter)
        {
            if (expires >= sAlarmT0 && expires <= sCounter)
            {
                fire = true;
            }
        }
        else
        {
            if (expires >= sAlarmT0 || expires <= sCounter)
            {
                fire = true;
            }
        }

        if (fire)
        {
            sIsRunning = false;

#if OPENTHREAD_ENABLE_DIAG

            if (otPlatDiagModeGet())
            {
                otPlatDiagAlarmFired(aInstance);
            }
            else
#endif
            {
                otPlatAlarmFired(aInstance);
            }
        }
    }
}

