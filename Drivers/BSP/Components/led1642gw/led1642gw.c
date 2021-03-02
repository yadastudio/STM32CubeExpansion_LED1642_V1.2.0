/**
******************************************************************************
* @file    led1642GW.c
* @author  CL
* @version V1.0.0
* @date    25-January-2016
* @brief   This file provides a set of functions needed to manage led1642gw.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "led1642gw.h"

/** @addtogroup BSP
* @{
*/

/** @addtogroup Components
* @{
*/

/** @addtogroup LED1642GW
* @{
*/

/** @defgroup LED1642GW_Private_Variables LED1642GW_Private_Variables
* @{
*/
uint16_t LED1642GW_ConfigurationRegister = 0x00; // Configuration Register Default Value
uint16_t LastConfigurationRegisterValue[MAXIMUM_NUMBER_OF_X_NUCLEO_BOARD]; // Configuration Value obtained or readed

extern uint8_t SPI_RxBuffer[];
extern uint8_t X_NUCLEO_LED1642GW_BoardPresent;

/*
CFG-0  // Current Gain Adjustment bit 1
CFG-1  // Current Gain Adjustment bit 2
CFG-2  // Current Gain Adjustment bit 3
CFG-3  // Current Gain Adjustment bit 4
CFG-4  // Current Gain Adjustment bit 5
CFG-5  // Current Gain Adjustment bit 6
CFG-6  // Current Range
CFG-7  // Error Detection Mode
CFG-8  // Shorted LED detection thresholds bit 1
CFG-9  // Shorted LED detection thresholds bit 2
CFG-10 // Auto OFF shutdown
CFG-11 // Output turn ON/OFF time
CFG-12 // Output turn ON/OFF time
CFG-13 // SDO Delay
CFG-14 // Gradual Output Delay
CFG-15 // 12/16 PWM Counter
*/

/**
* @brief  Configuration Register Read
* @param  None
* @retval None
*/
void ConfigurationRegisterRead(void)
{
  uint16_t TempCountXNucleoBoards = 0;
  
  /* Read Configuration Register */
  XNUCLEO_LED1642GW_READ_CONFIGRATION_REGISTER();
  
  /* One Extra Pulse on SPI Clock - SCK */
  XNUCLEO_LED1642GW_EXTRAPULSE();
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {    
  XNUCLEO_LED1642GW_DUMMY_DATA();
  
  LastConfigurationRegisterValue[(X_NUCLEO_LED1642GW_BoardPresent - 1) - TempCountXNucleoBoards] = SPI_RxBuffer[1];
  LastConfigurationRegisterValue[(X_NUCLEO_LED1642GW_BoardPresent - 1) - TempCountXNucleoBoards] = LastConfigurationRegisterValue[(X_NUCLEO_LED1642GW_BoardPresent - 1) - TempCountXNucleoBoards] << 8;
  LastConfigurationRegisterValue[(X_NUCLEO_LED1642GW_BoardPresent - 1) - TempCountXNucleoBoards] |= SPI_RxBuffer[0];
  }
}

/**
* @}
*/

/** @defgroup LED1642GW_Private_Functions LED1642GW_Private_Functions
* @{
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
