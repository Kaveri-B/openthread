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
 * @brief
 *   This file includes the platform-specific initializers.
 */

#include "platform-aducm3029.h"

#include <system_ADuCM3029.h>
#include <adi_callback.h>
#include <drivers/wdt/adi_wdt.h>
#include <drivers/pwr/adi_pwr.h>
#include <drivers/flash/adi_flash.h>
#include <drivers/gpio/adi_gpio.h>


#define SPI2_SCLK_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<4))
#define SPI2_MISO_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<6))
#define SPI2_MOSI_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<8))
#define SPI2_CS_2_PORTP2_MUX  ((uint32_t) ((uint32_t) 2<<20))
#define SPI2_CS_3_PORTP2_MUX  ((uint16_t) ((uint16_t) 2<<14))
#define UART0_TX_PORTP0_MUX  ((uint32_t) ((uint32_t) 1<<20))
#define UART0_RX_PORTP0_MUX  ((uint32_t) ((uint32_t) 1<<22))
#define UART0_TX_PORTP0_MASK  ~((uint32_t) ((uint32_t) 3<<20)) 
#define UART0_RX_PORTP0_MASK  ~((uint32_t) ((uint32_t) 3<<22))

#define SPI0_SCLK_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<0))
#define SPI0_MOSI_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<2))
#define SPI0_MISO_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<4))
#define SPI0_CS_0_PORTP0_MUX  ((uint32_t) ((uint16_t) 1<<6))

#define SPI1_SCLK_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<12))
#define SPI1_MISO_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<14))
#define SPI1_MOSI_PORTP1_MUX  ((uint32_t) ((uint32_t) 1<<16))
#define SPI1_CS_0_PORTP1_MUX  ((uint32_t) ((uint32_t) 1<<18))
#define SYS_WAKEUP_SYS_WAKE1_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<0))

static uint8_t FlashDeviceMem[ADI_FEE_MEMORY_SIZE]  __attribute__ ((aligned (4)));
static ADI_FEE_HANDLE hFlashDevice = NULL;
static uint8_t sGPIOCallbackMem[ADI_GPIO_MEMORY_SIZE] __attribute__((aligned(4)));

static void adi_wdt_callback(void *pCBParam, uint32_t Event, void *pArg)
{
    
}

void adi_initpinmux(void)
{
    /* Port Control MUX registers */
    *((volatile uint32_t *)REG_GPIO0_CFG) = UART0_TX_PORTP0_MUX  | 
                                            UART0_RX_PORTP0_MUX  | 
                                            SPI0_SCLK_PORTP0_MUX | 
                                            SPI0_MOSI_PORTP0_MUX | 
                                            SPI0_MISO_PORTP0_MUX | 
                                            SPI0_CS_0_PORTP0_MUX;
    
    *((volatile uint32_t *)REG_GPIO1_CFG) = SPI1_SCLK_PORTP1_MUX | 
                                            SPI1_MISO_PORTP1_MUX | 
                                            SPI1_MOSI_PORTP1_MUX |
					    SPI1_CS_0_PORTP1_MUX;
    
    *((volatile uint32_t *)REG_GPIO2_CFG) = SPI2_CS_2_PORTP2_MUX | 
                                            SPI2_CS_3_PORTP2_MUX;
}

void disable_CS3(void)
{
    *((volatile uint32_t *)REG_GPIO1_CFG) &= ~(SPI1_CS_0_PORTP1_MUX);
    *((volatile uint32_t *)REG_GPIO1_CFG) &= ~SPI1_MISO_PORTP1_MUX;
    adi_gpio_OutputEnable(ADI_GPIO_PORT1, ADI_GPIO_PIN_9, true);
    adi_gpio_InputEnable(ADI_GPIO_PORT1, ADI_GPIO_PIN_8, true);
}

void PlatformInit(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    SystemInit();
    adi_initpinmux();
    adi_wdt_Enable(false, adi_wdt_callback);
    adi_pwr_Init();
    adi_pwr_SetLFClockMux(ADI_CLOCK_MUX_LFCLK_LFXTAL);
    adi_pwr_EnableClockSource(ADI_CLOCK_SOURCE_LFXTAL, true);
    adi_pwr_SetClockDivider(ADI_CLOCK_HCLK, 1);
    adi_pwr_SetClockDivider(ADI_CLOCK_PCLK, 1);
    adi_fee_Open(0, FlashDeviceMem, sizeof(FlashDeviceMem), &hFlashDevice);
    /* Initialize GPIO */
    adi_gpio_Init(sGPIOCallbackMem, ADI_GPIO_MEMORY_SIZE);

    /* Set priorities */
    NVIC_SetPriority(SPI1_EVT_IRQn,0);
    NVIC_SetPriority(XINT_EVT1_IRQn,1);
    NVIC_SetPriority(SYS_GPIO_INTA_IRQn,1);
      
    aducm3029AlarmInit();
    aducm3029RandomInit();
    aducm3029RadioInit();
}

void PlatformProcessDrivers(otInstance *aInstance)
{

    // should sleep and wait for interrupts here

    aducm3029UartProcess();
    aducm3029RadioProcess(aInstance); 
    aducm3029AlarmProcess(aInstance);
}
