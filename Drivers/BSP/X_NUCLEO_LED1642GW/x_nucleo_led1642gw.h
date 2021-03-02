/**
  ******************************************************************************
  * @file    x_nucleo_led1642gw.h
  * @author  CL
  * @version V1.0.0
  * @date    25-January-2016
  * @brief   This file contains definitions for the x_nucleo_led1642gw.c 
  *          board specific functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __X_NUCLEO_LED1642GW_H
#define __X_NUCLEO_LED1642GW_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#ifdef STM32F401xE
#include "stm32f4xx_hal.h"
#endif
#ifdef STM32L053xx
#include "stm32l0xx_hal.h"
#endif

#include "led1642gw.h"
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup X_NUCLEO_LED1642GW
  * @{
  */

/** @defgroup X_NUCLEO_LED1642GW_Exported_Types X_NUCLEO_LED1642GW_Exported_Types
  * @{
  */

/**
  * @}
  */

/** @defgroup X_NUCLEO_1642GW_Exported_Defines X_NUCLEO_LED1642GW_Exported_Defines
  * @{
  */
       
#define NUMBER_OF_CHANNELS_PER_BOARD            16
#define MAXIMUM_NUMBER_OF_X_NUCLEO_BOARD        6    
#define X_NUCLEO_SIZE_OF_BUFFER                 (NUMBER_OF_CHANNELS_PER_BOARD*MAXIMUM_NUMBER_OF_X_NUCLEO_BOARD)   
#define HAL_SPI_SIZE_OF_BUFFER                  2   
#define NUMBER_OF_X_NUCLEO_RGB_LEDS             5
    
    
#define DEMO_LED_ON_ALL                         0
#define DEMO_LED_TRAIN_FWD                      1
#define DEMO_LED_TRAIN_REV                      2 
#define DEMO_LED_BRIGHTNESS                     3
#define DEMO_LED_ERROR_DETECTION                4   
#define DEMO_LED_DIFFERENT_COLOR_SAME_TIME      5
#define DEMO_LED_RANDOM                         6       
#define DEMO_LED_RANDOM_BRIGHTNESS              7    
#define DEMO_LED_LED_ON_ALL_LOW_CURRNET_RANGE   8 
    
#define DELAY_BETWEEN_DEMOS                     2047    
    
#define ALL_ON                                  0xFFFF    
#define ALL_OFF                                 0x0000    
   
#ifdef STM32L053xx
#define LE_TIMER                                &htim6
#define DELAY_TIMER                             &htim21   
#endif
   
#ifdef STM32F401xE
#define LE_TIMER                                &htim10
#define DELAY_TIMER                             &htim11
#endif    
        
#if (defined (USE_STM32F4XX_NUCLEO))
#endif

#if (defined (USE_STM32L0XX_NUCLEO))
#endif     
  
   
/** @defgroup X_NUCLEO_LED1642GW_Exported_Functions X_NUCLEO_LED1642GW_Exported_Functions
 * @{
 */
void XNUCLEO_LED1642GW_Demo(uint8_t);

void LED_ALL_ON(void);
void LED_TRAIN_FORWARD(void);
void LED_TRAIN_REVERSE_RGB(void); 
void LED_TRAIN_REVERSE_SINGLE_CHANNEL_LED(void);
void LED_ERROR_DETECTION(void);
void LED_ERROR_DETECTION_RGB_LED_BOARD(void);
void LED_RANDOM_PATTERN1(void);
void LED_RANDOM_PATTERN2(void);
void LED_DIFFERENT_COLOR_SAME_TIME(void);
void LED_BRIGHTNESS_RGB(void);
void LED_BRIGHTNESS(void);
void LED_RANDOM_BRIGHTNESS(void);
void LED_LED_ON_ALL_LOW_CURRNET_RANGE(void);

/*----Random Number Generation----*/
int LEDRandomNumberGenerator(int Limit);

/*----LED Indications----*/
void LEDGreenON(void);
void LEDRedON(void);

void XNUCLEO_LED1642GW_SendData_SPI(uint16_t);
HAL_StatusTypeDef XNUCLEO_LED1642GW_TransmitReceive_SPI(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef XNUCLEO_LED1642GW_WaitOnFlagUntilTimeout_SPI(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status, uint32_t Timeout);

void XNUCLEO_LED1642GW_BoardCount(void);

void XNUCLEO_LED1642GW_READ_CONFIGRATION_REGISTER(void);
void XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(uint16_t);
void XNUCLEO_LED1642GW_DATALATCH(uint16_t);
void XNUCLEO_LED1642GW_GLOBALLATCH(uint16_t);
void XNUCLEO_LED1642GW_SWITCH(uint16_t);
void XNUCLEO_LED1642GW_OPEN_CIRCUIT_ERROR_DETECTION(void);
void XNUCLEO_LED1642GW_SHORT_CIRCUIT_ERROR_DETECTION(void);
void XNUCLEO_LED1642GW_COMBINED_ERROR_DETECTION(void);
void XNUCLEO_LED1642GW_END_ERROR_DETECTION(void);

void XNUCLEO_LED1642GW_DUMMY_DATA(void);

void XNUCLEO_LED1642GW_EXTRAPULSE(void);

void DelayLoop(uint32_t);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
  }
#endif

#endif /* __X_NUCLEO_LED1642GW_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
