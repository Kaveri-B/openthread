/*! ****************************************************************************
 * @file:    ADF7242_HAL.c
 * @brief:   ADF7242 driver interface with hardware platform.
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
#include "common.h"
#include <drivers/general/adi_drivers_general.h>
#include "ADF7242.h"
#include <drivers/spi/adi_spi.h>
#include <drivers/pwr/adi_pwr.h>
#include <drivers/gpio/adi_gpio.h>
#include <drivers/xint/adi_xint.h>

   
/* Only used if hardware CS is used */
#define SPI1_CS_0_PORTP1_MUX  ((uint32_t) ((uint32_t) 1<<18))
   
/* SPI bitrate, ADF7242 datasheet spec is 80ns min for SCLK period */
//#define ADF7242_SPI_BITRATE              6500000
#define ADF7242_SPI_BITRATE              2500000//4000000

/* Use SPI device 2 (SPIH), this is fixed because of board layout */
#define ADF7242_SPI_DEVICE_NUM          (1)

/* Radio IRQ number */
#define XINT_EVT_NUM			ADI_XINT_EVENT_INT1

uint8_t spi_devicemem[ADI_SPI_MEMORY_SIZE] __attribute__((aligned(4)));

static ADI_SPI_HANDLE hDevice;

/*----------------------------------------------------------------------------*/

/*!
 * @brief  Initializes SPI MOSI, MISO, SCLK and CS pins.
 *
 * @param  None
 *
 * @return            Status
 *                    - #ADF_SPI_INIT_FAILED       Initialization of SPI failed.
 *                    - #ADF_SUCCESS             SPI initialization successfull.
 */
ADF7242_Result_t SPI_init(void)
{
  ADF7242_Result_t eResult = ADF_SUCCESS;
  
  *((volatile uint32_t *)REG_GPIO1_CFG) |= SPI1_CS_0_PORTP1_MUX;   
  
  CALL_FUNC(adi_spi_Open(ADF7242_SPI_DEVICE_NUM, 
                         spi_devicemem, ADI_SPI_MEMORY_SIZE, &hDevice), 
            ADF_SUCCESS, 
            ADI_SPI_SUCCESS, 
            ADF_SPI_INIT_FAILED)
  /* Set SPI baud rate */
  CALL_FUNC(adi_spi_SetBitrate(hDevice, ADF7242_SPI_BITRATE), 
            ADF_SUCCESS, 
            ADI_SPI_SUCCESS, 
            ADF_SPI_INIT_FAILED)

    adi_spi_SetContinuousMode(hDevice, true);
    
    adi_spi_SetIrqmode(hDevice, 0u);
          
  CALL_FUNC(adi_spi_SetChipSelect (hDevice, ADI_SPI_CS0), 
            ADF_SUCCESS, 
            ADI_SPI_SUCCESS, 
            ADF_SPI_INIT_FAILED)

  return eResult;
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  IRQ handler for ADF7242 radio interrupt.
 *
 * @param[in]     pCBParam        Pointer to callback prameter.
 *
 * @param[in]     Event           Event ID specific to the Driver/Service.
 *
 * @param[in]     pArg            Pointer to the event specific argument.
 *
 * @return         None
 */
void trx_irq_cb(void *pCBParam, uint32_t Event, void *pArg)
{
  ADF7242_irq_handler();
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Enable external IRQ pin for ADF7242 and register a call back function
 *
 * @param  None
 *
 * @return            Status
 *                    - #ADF_SPI_INIT_FAILED       Initialization of SPI failed.
 *                    - #ADF_SUCCESS             SPI initialization successfull.
 */
ADF7242_Result_t IRQ_init(void)
{
  ADF7242_Result_t eResult = ADF_SUCCESS;

  IRQn_Type eIrq;
     /* init the GPIO service */
  adi_gpio_InputEnable(ADI_GPIO_PORT1, ADI_GPIO_PIN_0, true);
  adi_gpio_SetGroupInterruptPolarity(ADI_GPIO_PORT1, ADI_GPIO_PIN_0);
  /* Enable pin interrupt on group interrupt A */
  adi_gpio_SetGroupInterruptPins(ADI_GPIO_PORT1, ADI_GPIO_INTA_IRQ, ADI_GPIO_PIN_0);
  eIrq = SYS_GPIO_INTA_IRQn;
  /* Register the callback */
  adi_gpio_RegisterCallback (ADI_GPIO_INTA_IRQ, trx_irq_cb, (void*)&eIrq);
  return eResult;
}
/*----------------------------------------------------------------------------*/

/*!
 * @brief  Writes and Reads specified number of bytes using blocking mode and 
 *         without DMA. 
 *
 * @param[in]    pTxbuf    Pointer to 
 *
 * @return  None
 */
ADF7242_Result_t rf_spi_transaction(uint8_t *pTxbuf, uint16_t txlen, 
                                        uint8_t *pRxbuf, uint16_t rxlen)
{
 
  ADF7242_Result_t           eResult = ADF_SUCCESS;
  ADI_SPI_TRANSCEIVER     transceive;

    /* initialize data attributes */
  transceive.pTransmitter = pTxbuf;
  transceive.pReceiver = pRxbuf;

  
  
  /* auto increment both buffers */
  transceive.nTxIncrement = 1;
  transceive.nRxIncrement = 1;
  
  /* link transceive data size to the remaining count */
  transceive.TransmitterBytes = txlen;
  /* link transceive data size to the remaining count */
  if(rxlen == 0){
    rxlen = txlen;
    transceive.nRxIncrement = 0;
  }
  
  transceive.ReceiverBytes = rxlen;
  

  
  /* DMA false and RD_CTL false */
  transceive.bDMA = false;
  transceive.bRD_CTL = false;
  
    /* Submit data buffers for SPI Master-Mode transaction in Blocking mode */
  CALL_FUNC(adi_spi_MasterReadWrite(hDevice, &transceive), 
            ADF_SUCCESS, 
            ADI_SPI_SUCCESS, 
            ADF_SPI_ERROR)
      
  return eResult;

}  

void Enable_TRX_IRQ(void)
{
   NVIC_EnableIRQ(SYS_GPIO_INTA_IRQn);
}
void Disable_TRX_IRQ(void)
{
  NVIC_DisableIRQ(SYS_GPIO_INTA_IRQn);
}

 int8_t PhyWakeUp(void)
{

    volatile uint32_t i = 0;
    uint16_t value = 0;

    disable_CS3();
    /* FIXME use Murali's ADC delay function... not int counts!
    do we still need second try with new silicon?
    perhaps just wiggle select twice and then watch for miso? */
    /* assert ADF7023 select and watch for wakeup on MISO */
if(adi_gpio_SetLow (ADI_GPIO_PORT1, ADI_GPIO_PIN_9) != ADI_GPIO_SUCCESS)
    {
        adi_initpinmux();
        return -1;
    }
    do
    {
        adi_gpio_GetData(ADI_GPIO_PORT1, ADI_GPIO_PIN_8, &value);
        if (value)
            break;
        i++;
    } while(i < 5000);

    if(adi_gpio_SetHigh(ADI_GPIO_PORT1, ADI_GPIO_PIN_9) != ADI_GPIO_SUCCESS)
    {
        adi_initpinmux();
        return -1;
    }
	/* if at first you don't succeed... */
    if (i > 4999)
	{
        i = 0;
		if(adi_gpio_SetLow (ADI_GPIO_PORT1, ADI_GPIO_PIN_9) != ADI_GPIO_SUCCESS)
        {
            adi_initpinmux();
			return -1;
        }
		do
		{
			adi_gpio_GetData(ADI_GPIO_PORT1, ADI_GPIO_PIN_8, &value);
            if (value)
                break;
			i++;
		} while(i < 5000);

		if(adi_gpio_SetHigh(ADI_GPIO_PORT1, ADI_GPIO_PIN_9) != ADI_GPIO_SUCCESS)
        {
            adi_initpinmux();
			return -1;
        }
	}

	// if second try fails, return error
    if (i > 4999)
    {
        adi_initpinmux();
        return -1;
    }
    adi_initpinmux();
    return 0;
}
/*----------------------------------------------------------------------------*/

