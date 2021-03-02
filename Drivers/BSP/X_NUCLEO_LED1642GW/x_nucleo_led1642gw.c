/**
******************************************************************************
* @file    x_nucleo_led1642gw.c
* @author  CL
* @version V1.0.0
* @date    25-January-2016
* @brief   This file provides X_NUCLEO_LED1642GW shield board specific functions
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
#include "x_nucleo_led1642gw.h"
#include "main.h"

#ifdef STM32F401xE
#include "stm32f4xx_hal_spi.h"
#endif
#ifdef STM32L053xx
#include "stm32l0xx_hal_spi.h"
#endif

/** @addtogroup BSP
* @{
*/

/** @addtogroup X_NUCLEO_LED1642GW
* @{
*/

/** @defgroup X_NUCLEO_LED1642GW_Private_Defines X_NUCLEO_LED1642GW_Private_Defines
* @{
*/

/**
* @}
*/
extern SPI_HandleTypeDef hspi1;

/** @defgroup X_NUCLEO_LED1642GW_Private_Variables X_NUCLEO_LED1642GW_Private_Variables
* @{
*/

/* Number of X-NUCLEO Boards */
uint8_t X_NUCLEO_LED1642GW_BoardPresent = 0;

/* SPI Buffer which is used for Transmit/Receicve */
uint8_t SPI_TxBuffer[HAL_SPI_SIZE_OF_BUFFER];
uint8_t SPI_RxBuffer[HAL_SPI_SIZE_OF_BUFFER];

/* Different Commadn values are assigned to these buffers and send to SPI Buffer*/
uint16_t TxBuffer[X_NUCLEO_SIZE_OF_BUFFER];
uint16_t RxBuffer[X_NUCLEO_SIZE_OF_BUFFER];
uint8_t SPIBufferSize = 0;

/* Array which will have Error Data*/
uint16_t ErrorRegisterValue[MAXIMUM_NUMBER_OF_X_NUCLEO_BOARD];

extern uint8_t DemoNumber;
extern uint8_t DemoChange;

#ifdef STM32L053xx
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim21;  
#endif
   
#ifdef STM32F401xE
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
#endif 

/**
* @}
*/


/**
* @brief  X-NUCLEO LED1642GW DEMO
* @param  Demo Number which is to executed is passed
* @retval None
*/
void XNUCLEO_LED1642GW_Demo(uint8_t DemoNumber)
{
  ConfigurationRegisterRead();
  switch (DemoNumber)
  { 
  case DEMO_LED_ON_ALL:                         LED_ALL_ON();                           break;
  case DEMO_LED_TRAIN_FWD:                      LED_TRAIN_FORWARD();                    break;
  case DEMO_LED_TRAIN_REV:                      LED_TRAIN_REVERSE_RGB();                break;
  case DEMO_LED_BRIGHTNESS:                     LED_BRIGHTNESS_RGB();                   break;
  case DEMO_LED_ERROR_DETECTION:                LED_ERROR_DETECTION_RGB_LED_BOARD();    break;
  case DEMO_LED_DIFFERENT_COLOR_SAME_TIME:      LED_DIFFERENT_COLOR_SAME_TIME();        break;
  case DEMO_LED_RANDOM:                         LED_RANDOM_PATTERN1();                  break;
  case DEMO_LED_RANDOM_BRIGHTNESS:              LED_RANDOM_BRIGHTNESS();                break;
  case DEMO_LED_LED_ON_ALL_LOW_CURRNET_RANGE:   LED_LED_ON_ALL_LOW_CURRNET_RANGE();     break;
  
  default:                                                                              break;
  } 
}

/**
* @brief  LED ALL_ON DEMO
* @param  None
* @retval None
*/
void LED_ALL_ON(void)
{
  uint16_t TempCountXNucleoBoards = 0;
  uint16_t TempBrighnessBufferCount  = 0;
  
  /* Configuration Register */
  /* For eg:
  In case of 4 drivers:
  The 0th element will corresponds to 4th driver */
  
  /* In case the number of boards increases TxBuffer need to assigned the configuration value based on number of board */
  /* Here only for 6 boards the assignment has been done */
  
  TxBuffer[0] = 0x007F;
  TxBuffer[1] = 0x007F;
  TxBuffer[2] = 0x007F;
  TxBuffer[3] = 0x007F;
  TxBuffer[4] = 0x007F;
  TxBuffer[5] = 0x007F;
  
  /* Based on the muner of the boards the data is sent and LE signal is given on last buffer byte */
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Brightness Register */
  /* For 1 LED1642GW 15 Data Byte Register need to be sent */
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  TxBuffer[6] = 0xFFFF;
  TxBuffer[7] = 0xFFFF;
  TxBuffer[8] = 0xFFFF;
  TxBuffer[9] = 0xFFFF;
  TxBuffer[10] = 0xFFFF;
  TxBuffer[11] = 0xFFFF;
  TxBuffer[12] = 0xFFFF;  
  TxBuffer[13] = 0xFFFF;
  TxBuffer[14] = 0xFFFF;
  TxBuffer[15] = 0xFFFF;  
  TxBuffer[16] = 0xFFFF;
  TxBuffer[17] = 0xFFFF;
  TxBuffer[18] = 0xFFFF;  
  TxBuffer[19] = 0xFFFF;
  TxBuffer[20] = 0xFFFF;
  TxBuffer[21] = 0xFFFF;    
  TxBuffer[22] = 0xFFFF;
  TxBuffer[23] = 0xFFFF;
  TxBuffer[24] = 0xFFFF;
  TxBuffer[25] = 0xFFFF;
  TxBuffer[26] = 0xFFFF;
  TxBuffer[27] = 0xFFFF;
  TxBuffer[28] = 0xFFFF;
  TxBuffer[29] = 0xFFFF;
  TxBuffer[30] = 0xFFFF;
  TxBuffer[31] = 0xFFFF;
  TxBuffer[32] = 0xFFFF;  
  TxBuffer[33] = 0xFFFF;
  TxBuffer[34] = 0xFFFF;
  TxBuffer[35] = 0xFFFF;  
  TxBuffer[36] = 0xFFFF;
  TxBuffer[37] = 0xFFFF;
  TxBuffer[38] = 0xFFFF;    
  TxBuffer[39] = 0xFFFF;
  TxBuffer[40] = 0xFFFF;
  TxBuffer[41] = 0xFFFF;
  TxBuffer[42] = 0xFFFF;
  TxBuffer[43] = 0xFFFF;
  TxBuffer[44] = 0xFFFF;
  TxBuffer[45] = 0xFFFF;
  TxBuffer[46] = 0xFFFF;
  TxBuffer[47] = 0xFFFF;
  TxBuffer[48] = 0xFFFF;
  TxBuffer[49] = 0xFFFF;  
  TxBuffer[50] = 0xFFFF;
  TxBuffer[51] = 0xFFFF;
  TxBuffer[52] = 0xFFFF;  
  TxBuffer[53] = 0xFFFF;
  TxBuffer[54] = 0xFFFF;
  TxBuffer[55] = 0xFFFF;    
  TxBuffer[56] = 0xFFFF;
  TxBuffer[57] = 0xFFFF;
  TxBuffer[58] = 0xFFFF;
  TxBuffer[59] = 0xFFFF;
  TxBuffer[60] = 0xFFFF;
  TxBuffer[61] = 0xFFFF;
  TxBuffer[62] = 0xFFFF;
  TxBuffer[63] = 0xFFFF;
  TxBuffer[64] = 0xFFFF;
  TxBuffer[65] = 0xFFFF;
  TxBuffer[66] = 0xFFFF;  
  TxBuffer[67] = 0xFFFF;
  TxBuffer[68] = 0xFFFF;
  TxBuffer[69] = 0xFFFF;  
  TxBuffer[70] = 0xFFFF;
  TxBuffer[71] = 0xFFFF;
  TxBuffer[72] = 0xFFFF;    
  TxBuffer[73] = 0xFFFF;
  TxBuffer[74] = 0xFFFF;
  TxBuffer[75] = 0xFFFF;
  TxBuffer[76] = 0xFFFF;
  TxBuffer[77] = 0xFFFF;
  TxBuffer[78] = 0xFFFF;
  TxBuffer[79] = 0xFFFF;
  TxBuffer[80] = 0xFFFF;
  TxBuffer[81] = 0xFFFF;
  TxBuffer[82] = 0xFFFF;
  TxBuffer[83] = 0xFFFF;
  TxBuffer[84] = 0xFFFF;
  TxBuffer[85] = 0xFFFF;  
  TxBuffer[86] = 0xFFFF;
  TxBuffer[87] = 0xFFFF;
  TxBuffer[88] = 0xFFFF;    
  TxBuffer[89] = 0xFFFF;
  TxBuffer[90] = 0xFFFF;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    if(TempBrighnessBufferCount == (X_NUCLEO_LED1642GW_BoardPresent - 1))
    {
      XNUCLEO_LED1642GW_DATALATCH(TxBuffer[TempCountXNucleoBoards]); 
      TempBrighnessBufferCount = 0;
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
      TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    }
  }

  /* Global Latch Register */
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_GLOBALLATCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Switch Control */
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  
  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,DELAY_BETWEEN_DEMOS);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
}

/**
* @brief  LED TRAIN DEMO FORWARD
* @param  None
* @retval None
*/
void LED_TRAIN_FORWARD(void)
{
  uint16_t TempCountXNucleoBoards = 0;
  uint16_t TempBrighnessBufferCount = 0;
  uint16_t TempCountXNucleoNumberOfChannelsPerBoard = 0;
  
  /* Configuration Register */
  /* Uncomment Number of Lines equal to number of LED1642GW drivers */
  /* For eg:
  In case of 4 drivers:
  The 0th element will corresponds to 4th driver*/
  
  TxBuffer[0] = 0x007F;
  TxBuffer[1] = 0x007F;
  TxBuffer[2] = 0x007F;
  TxBuffer[3] = 0x007F;
  TxBuffer[4] = 0x007F;
  TxBuffer[5] = 0x007F;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  }
  
  /* Brightness Register */  
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  TxBuffer[6] = 0xFFFF;
  TxBuffer[7] = 0xFFFF;
  TxBuffer[8] = 0xFFFF;
  TxBuffer[9] = 0xFFFF;
  TxBuffer[10] = 0xFFFF;
  TxBuffer[11] = 0xFFFF;
  TxBuffer[12] = 0xFFFF;  
  TxBuffer[13] = 0xFFFF;
  TxBuffer[14] = 0xFFFF;
  TxBuffer[15] = 0xFFFF;  
  TxBuffer[16] = 0xFFFF;
  TxBuffer[17] = 0xFFFF;
  TxBuffer[18] = 0xFFFF;  
  TxBuffer[19] = 0xFFFF;
  TxBuffer[20] = 0xFFFF;
  TxBuffer[21] = 0xFFFF;    
  TxBuffer[22] = 0xFFFF;
  TxBuffer[23] = 0xFFFF;
  TxBuffer[24] = 0xFFFF;
  TxBuffer[25] = 0xFFFF;
  TxBuffer[26] = 0xFFFF;
  TxBuffer[27] = 0xFFFF;
  TxBuffer[28] = 0xFFFF;
  TxBuffer[29] = 0xFFFF;
  TxBuffer[30] = 0xFFFF;
  TxBuffer[31] = 0xFFFF;
  TxBuffer[32] = 0xFFFF;  
  TxBuffer[33] = 0xFFFF;
  TxBuffer[34] = 0xFFFF;
  TxBuffer[35] = 0xFFFF;  
  TxBuffer[36] = 0xFFFF;
  TxBuffer[37] = 0xFFFF;
  TxBuffer[38] = 0xFFFF;    
  TxBuffer[39] = 0xFFFF;
  TxBuffer[40] = 0xFFFF;
  TxBuffer[41] = 0xFFFF;
  TxBuffer[42] = 0xFFFF;
  TxBuffer[43] = 0xFFFF;
  TxBuffer[44] = 0xFFFF;
  TxBuffer[45] = 0xFFFF;
  TxBuffer[46] = 0xFFFF;
  TxBuffer[47] = 0xFFFF;
  TxBuffer[48] = 0xFFFF;
  TxBuffer[49] = 0xFFFF;  
  TxBuffer[50] = 0xFFFF;
  TxBuffer[51] = 0xFFFF;
  TxBuffer[52] = 0xFFFF;  
  TxBuffer[53] = 0xFFFF;
  TxBuffer[54] = 0xFFFF;
  TxBuffer[55] = 0xFFFF;    
  TxBuffer[56] = 0xFFFF;
  TxBuffer[57] = 0xFFFF;
  TxBuffer[58] = 0xFFFF;
  TxBuffer[59] = 0xFFFF;
  TxBuffer[60] = 0xFFFF;
  TxBuffer[61] = 0xFFFF;
  TxBuffer[62] = 0xFFFF;
  TxBuffer[63] = 0xFFFF;
  TxBuffer[64] = 0xFFFF;
  TxBuffer[65] = 0xFFFF;
  TxBuffer[66] = 0xFFFF;  
  TxBuffer[67] = 0xFFFF;
  TxBuffer[68] = 0xFFFF;
  TxBuffer[69] = 0xFFFF;  
  TxBuffer[70] = 0xFFFF;
  TxBuffer[71] = 0xFFFF;
  TxBuffer[72] = 0xFFFF;    
  TxBuffer[73] = 0xFFFF;
  TxBuffer[74] = 0xFFFF;
  TxBuffer[75] = 0xFFFF;
  TxBuffer[76] = 0xFFFF;
  TxBuffer[77] = 0xFFFF;
  TxBuffer[78] = 0xFFFF;
  TxBuffer[79] = 0xFFFF;
  TxBuffer[80] = 0xFFFF;
  TxBuffer[81] = 0xFFFF;
  TxBuffer[82] = 0xFFFF;
  TxBuffer[83] = 0xFFFF;
  TxBuffer[84] = 0xFFFF;
  TxBuffer[85] = 0xFFFF;  
  TxBuffer[86] = 0xFFFF;
  TxBuffer[87] = 0xFFFF;
  TxBuffer[88] = 0xFFFF;    
  TxBuffer[89] = 0xFFFF;
  TxBuffer[90] = 0xFFFF;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    if(TempBrighnessBufferCount == (X_NUCLEO_LED1642GW_BoardPresent - 1))
    {
      XNUCLEO_LED1642GW_DATALATCH(TxBuffer[TempCountXNucleoBoards]); 
      TempBrighnessBufferCount = 0;
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
      TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    } 
  }
  
  /* Global Latch Register */  
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_GLOBALLATCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,DELAY_BETWEEN_DEMOS);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  while(DemoChange == 0)
  {
    /* Switch Control */  
    TxBuffer[0] = 0x0001;
    TxBuffer[1] = 0x0001;
    TxBuffer[2] = 0x0001;
    TxBuffer[3] = 0x0001;
    TxBuffer[4] = 0x0001;
    TxBuffer[5] = 0x0001;
    
    for (TempCountXNucleoNumberOfChannelsPerBoard = 0; TempCountXNucleoNumberOfChannelsPerBoard < NUMBER_OF_CHANNELS_PER_BOARD; TempCountXNucleoNumberOfChannelsPerBoard ++)
    {
      if(DemoNumber == DEMO_LED_TRAIN_FWD)
      {
        for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
        {
          if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
          {
            XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
            TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
          }
          else
          {
            XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
            TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
          } 
        }
      }
      DelayLoop(2);
    }
  }
}

/**
* @brief  LED TRAIN DEMO REVERSE
* @param  None
* @retval None
*/
void LED_TRAIN_REVERSE_RGB(void)
{
  uint16_t TempCountXNucleoBoards = 0;
  uint16_t TempBrighnessBufferCount = 0;
  uint16_t TempCountXNucleoNumberOfLEDs = 0;
  uint16_t TempCountLEDTrainReverse = 0;
  
  /* Configuration Register */
  /* Uncomment Number of Lines equal to number of LED1642GW drivers */
  /* For eg:
  In case of 4 drivers:
  The 0th element will corresponds to 4th driver*/
  
  TxBuffer[0] = 0x007F;
  TxBuffer[1] = 0x007F;
  TxBuffer[2] = 0x007F;
  TxBuffer[3] = 0x007F;
  TxBuffer[4] = 0x007F;
  TxBuffer[5] = 0x007F;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  }
  
  /* Brightness Register */  
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  TxBuffer[6] = 0xFFFF;
  TxBuffer[7] = 0xFFFF;
  TxBuffer[8] = 0xFFFF;
  TxBuffer[9] = 0xFFFF;
  TxBuffer[10] = 0xFFFF;
  TxBuffer[11] = 0xFFFF;
  TxBuffer[12] = 0xFFFF;  
  TxBuffer[13] = 0xFFFF;
  TxBuffer[14] = 0xFFFF;
  TxBuffer[15] = 0xFFFF;  
  TxBuffer[16] = 0xFFFF;
  TxBuffer[17] = 0xFFFF;
  TxBuffer[18] = 0xFFFF;  
  TxBuffer[19] = 0xFFFF;
  TxBuffer[20] = 0xFFFF;
  TxBuffer[21] = 0xFFFF;    
  TxBuffer[22] = 0xFFFF;
  TxBuffer[23] = 0xFFFF;
  TxBuffer[24] = 0xFFFF;
  TxBuffer[25] = 0xFFFF;
  TxBuffer[26] = 0xFFFF;
  TxBuffer[27] = 0xFFFF;
  TxBuffer[28] = 0xFFFF;
  TxBuffer[29] = 0xFFFF;
  TxBuffer[30] = 0xFFFF;
  TxBuffer[31] = 0xFFFF;
  TxBuffer[32] = 0xFFFF;  
  TxBuffer[33] = 0xFFFF;
  TxBuffer[34] = 0xFFFF;
  TxBuffer[35] = 0xFFFF;  
  TxBuffer[36] = 0xFFFF;
  TxBuffer[37] = 0xFFFF;
  TxBuffer[38] = 0xFFFF;    
  TxBuffer[39] = 0xFFFF;
  TxBuffer[40] = 0xFFFF;
  TxBuffer[41] = 0xFFFF;
  TxBuffer[42] = 0xFFFF;
  TxBuffer[43] = 0xFFFF;
  TxBuffer[44] = 0xFFFF;
  TxBuffer[45] = 0xFFFF;
  TxBuffer[46] = 0xFFFF;
  TxBuffer[47] = 0xFFFF;
  TxBuffer[48] = 0xFFFF;
  TxBuffer[49] = 0xFFFF;  
  TxBuffer[50] = 0xFFFF;
  TxBuffer[51] = 0xFFFF;
  TxBuffer[52] = 0xFFFF;  
  TxBuffer[53] = 0xFFFF;
  TxBuffer[54] = 0xFFFF;
  TxBuffer[55] = 0xFFFF;    
  TxBuffer[56] = 0xFFFF;
  TxBuffer[57] = 0xFFFF;
  TxBuffer[58] = 0xFFFF;
  TxBuffer[59] = 0xFFFF;
  TxBuffer[60] = 0xFFFF;
  TxBuffer[61] = 0xFFFF;
  TxBuffer[62] = 0xFFFF;
  TxBuffer[63] = 0xFFFF;
  TxBuffer[64] = 0xFFFF;
  TxBuffer[65] = 0xFFFF;
  TxBuffer[66] = 0xFFFF;  
  TxBuffer[67] = 0xFFFF;
  TxBuffer[68] = 0xFFFF;
  TxBuffer[69] = 0xFFFF;  
  TxBuffer[70] = 0xFFFF;
  TxBuffer[71] = 0xFFFF;
  TxBuffer[72] = 0xFFFF;    
  TxBuffer[73] = 0xFFFF;
  TxBuffer[74] = 0xFFFF;
  TxBuffer[75] = 0xFFFF;
  TxBuffer[76] = 0xFFFF;
  TxBuffer[77] = 0xFFFF;
  TxBuffer[78] = 0xFFFF;
  TxBuffer[79] = 0xFFFF;
  TxBuffer[80] = 0xFFFF;
  TxBuffer[81] = 0xFFFF;
  TxBuffer[82] = 0xFFFF;
  TxBuffer[83] = 0xFFFF;
  TxBuffer[84] = 0xFFFF;
  TxBuffer[85] = 0xFFFF;  
  TxBuffer[86] = 0xFFFF;
  TxBuffer[87] = 0xFFFF;
  TxBuffer[88] = 0xFFFF;    
  TxBuffer[89] = 0xFFFF;
  TxBuffer[90] = 0xFFFF; 
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    if(TempBrighnessBufferCount == (X_NUCLEO_LED1642GW_BoardPresent - 1))
    {
      XNUCLEO_LED1642GW_DATALATCH(TxBuffer[TempCountXNucleoBoards]); 
      TempBrighnessBufferCount = 0;
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
      TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    }
  }
  
  /* Global Latch */
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_GLOBALLATCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,DELAY_BETWEEN_DEMOS);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  while(DemoChange == 0)
  { 
    /* Switch Control */
    
    if (TempCountLEDTrainReverse == 0)
    {
    /* Blue Color */
    TxBuffer[0] = 0x4924;
    TxBuffer[1] = 0x4924;
    TxBuffer[2] = 0x4924;
    TxBuffer[3] = 0x4924;
    TxBuffer[4] = 0x4924;
    TxBuffer[5] = 0x4924;
    }
    else if (TempCountLEDTrainReverse == 1)
    {
      TxBuffer[0] = 0x1249;
      TxBuffer[1] = 0x1249;
      TxBuffer[2] = 0x1249;
      TxBuffer[3] = 0x1249;
      TxBuffer[4] = 0x1249;
      TxBuffer[5] = 0x1249;
    }
    else if (TempCountLEDTrainReverse == 2)
    {
      TxBuffer[0] = 0x2492;
      TxBuffer[1] = 0x2492;
      TxBuffer[2] = 0x2492;
      TxBuffer[3] = 0x2492;
      TxBuffer[4] = 0x2492;
      TxBuffer[5] = 0x2492;
    }
    
    for (TempCountXNucleoNumberOfLEDs = 0; TempCountXNucleoNumberOfLEDs <= NUMBER_OF_X_NUCLEO_RGB_LEDS; TempCountXNucleoNumberOfLEDs ++)
    {
      if(DemoNumber == DEMO_LED_TRAIN_REV)
      {
        for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
        {
          if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
          {
            XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
            TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 3; 
          }
          else
          {
            XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
            TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 3; 
          }
        }
      }
      DelayLoop(4);
    }  
    TempCountLEDTrainReverse  = TempCountLEDTrainReverse + 1;
    if(TempCountLEDTrainReverse > 2)
    {
      TempCountLEDTrainReverse = 0;
    }
  }
}

/**
* @brief  LED TRAIN DEMO REVERSE For Single Channel LED
* @param  None
* @retval None
*/
void LED_TRAIN_REVERSE_SINGLE_CHANNEL_LED(void)
{
  uint16_t TempCountXNucleoBoards = 0;
  uint16_t TempBrighnessBufferCount = 0;
  uint16_t TempCountXNucleoNumberOfChannelsPerBoard = 0;
  
  /* Configuration Register */
  /* Uncomment Number of Lines equal to number of LED1642GW drivers */
  /* For eg:
  In case of 4 drivers:
  The 0th element will corresponds to 4th driver*/
  
  TxBuffer[0] = 0x007F;
  TxBuffer[1] = 0x007F;
  TxBuffer[2] = 0x007F;
  TxBuffer[3] = 0x007F;
  TxBuffer[4] = 0x007F;
  TxBuffer[5] = 0x007F;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  }
  
  /* Brightness Register */  
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  TxBuffer[6] = 0xFFFF;
  TxBuffer[7] = 0xFFFF;
  TxBuffer[8] = 0xFFFF;
  TxBuffer[9] = 0xFFFF;
  TxBuffer[10] = 0xFFFF;
  TxBuffer[11] = 0xFFFF;
  TxBuffer[12] = 0xFFFF;  
  TxBuffer[13] = 0xFFFF;
  TxBuffer[14] = 0xFFFF;
  TxBuffer[15] = 0xFFFF;  
  TxBuffer[16] = 0xFFFF;
  TxBuffer[17] = 0xFFFF;
  TxBuffer[18] = 0xFFFF;  
  TxBuffer[19] = 0xFFFF;
  TxBuffer[20] = 0xFFFF;
  TxBuffer[21] = 0xFFFF;    
  TxBuffer[22] = 0xFFFF;
  TxBuffer[23] = 0xFFFF;
  TxBuffer[24] = 0xFFFF;
  TxBuffer[25] = 0xFFFF;
  TxBuffer[26] = 0xFFFF;
  TxBuffer[27] = 0xFFFF;
  TxBuffer[28] = 0xFFFF;
  TxBuffer[29] = 0xFFFF;
  TxBuffer[30] = 0xFFFF;
  TxBuffer[31] = 0xFFFF;
  TxBuffer[32] = 0xFFFF;  
  TxBuffer[33] = 0xFFFF;
  TxBuffer[34] = 0xFFFF;
  TxBuffer[35] = 0xFFFF;  
  TxBuffer[36] = 0xFFFF;
  TxBuffer[37] = 0xFFFF;
  TxBuffer[38] = 0xFFFF;    
  TxBuffer[39] = 0xFFFF;
  TxBuffer[40] = 0xFFFF;
  TxBuffer[41] = 0xFFFF;
  TxBuffer[42] = 0xFFFF;
  TxBuffer[43] = 0xFFFF;
  TxBuffer[44] = 0xFFFF;
  TxBuffer[45] = 0xFFFF;
  TxBuffer[46] = 0xFFFF;
  TxBuffer[47] = 0xFFFF;
  TxBuffer[48] = 0xFFFF;
  TxBuffer[49] = 0xFFFF;  
  TxBuffer[50] = 0xFFFF;
  TxBuffer[51] = 0xFFFF;
  TxBuffer[52] = 0xFFFF;  
  TxBuffer[53] = 0xFFFF;
  TxBuffer[54] = 0xFFFF;
  TxBuffer[55] = 0xFFFF;    
  TxBuffer[56] = 0xFFFF;
  TxBuffer[57] = 0xFFFF;
  TxBuffer[58] = 0xFFFF;
  TxBuffer[59] = 0xFFFF;
  TxBuffer[60] = 0xFFFF;
  TxBuffer[61] = 0xFFFF;
  TxBuffer[62] = 0xFFFF;
  TxBuffer[63] = 0xFFFF;
  TxBuffer[64] = 0xFFFF;
  TxBuffer[65] = 0xFFFF;
  TxBuffer[66] = 0xFFFF;  
  TxBuffer[67] = 0xFFFF;
  TxBuffer[68] = 0xFFFF;
  TxBuffer[69] = 0xFFFF;  
  TxBuffer[70] = 0xFFFF;
  TxBuffer[71] = 0xFFFF;
  TxBuffer[72] = 0xFFFF;    
  TxBuffer[73] = 0xFFFF;
  TxBuffer[74] = 0xFFFF;
  TxBuffer[75] = 0xFFFF;
  TxBuffer[76] = 0xFFFF;
  TxBuffer[77] = 0xFFFF;
  TxBuffer[78] = 0xFFFF;
  TxBuffer[79] = 0xFFFF;
  TxBuffer[80] = 0xFFFF;
  TxBuffer[81] = 0xFFFF;
  TxBuffer[82] = 0xFFFF;
  TxBuffer[83] = 0xFFFF;
  TxBuffer[84] = 0xFFFF;
  TxBuffer[85] = 0xFFFF;  
  TxBuffer[86] = 0xFFFF;
  TxBuffer[87] = 0xFFFF;
  TxBuffer[88] = 0xFFFF;    
  TxBuffer[89] = 0xFFFF;
  TxBuffer[90] = 0xFFFF; 
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    if(TempBrighnessBufferCount == (X_NUCLEO_LED1642GW_BoardPresent - 1))
    {
      XNUCLEO_LED1642GW_DATALATCH(TxBuffer[TempCountXNucleoBoards]); 
      TempBrighnessBufferCount = 0;
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
      TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    }
  }
  
  /* Global Latch */
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_GLOBALLATCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,DELAY_BETWEEN_DEMOS);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  while(DemoChange == 0)
  { 
    /* Switch Control */
    TxBuffer[0] = 0xFFFF;
    TxBuffer[1] = 0xFFFF;
    TxBuffer[2] = 0xFFFF;
    TxBuffer[3] = 0xFFFF;
    TxBuffer[4] = 0xFFFF;
    TxBuffer[5] = 0xFFFF;
    
    for (TempCountXNucleoNumberOfChannelsPerBoard = 0; TempCountXNucleoNumberOfChannelsPerBoard < NUMBER_OF_CHANNELS_PER_BOARD; TempCountXNucleoNumberOfChannelsPerBoard ++)
    {
      if(DemoNumber == DEMO_LED_TRAIN_REV)
      {
        for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
        {
          if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
          {
            XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
            TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
          }
          else
          {
            XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
            TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
          }
        }
      }
      DelayLoop(8);
    }   
  }
}

/**
* @brief  LED ERROR DETECTION MODE
* @param  None
* @retval None
*/
void LED_ERROR_DETECTION(void)
{    
  GPIO_InitTypeDef GPIO_InitStruct;
  uint16_t TempCountXNucleoBoards = 0;
  uint8_t TempLEDErrorIndicator = 0;
  
  /* LED ALL ON */
  LED_ALL_ON();
  
  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,DELAY_BETWEEN_DEMOS);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  while (DemoChange == 0)
  {     
    /* Starting Combined Error Detection */
    XNUCLEO_LED1642GW_COMBINED_ERROR_DETECTION();
    
    /* Delay */
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    
    /* Ending Error Detection Mode */
    XNUCLEO_LED1642GW_END_ERROR_DETECTION();  
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      /* Setting GPIO for one Extra Pulse on SPI Clock */
      GPIO_InitStruct.Pin = GPIO_PIN_5;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      
      /* One Extra Pulse on SPI Clock - SCK */
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      __NOP();
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  
      
      /* ReSetting GPIO as SCK */
      GPIO_InitStruct.Pin = GPIO_PIN_5;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
      
#ifdef USE_STM32F4XX_NUCLEO
      GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
#endif
      
#ifdef USE_STM32L0XX_NUCLEO
      GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
#endif
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      
      XNUCLEO_LED1642GW_DUMMY_DATA();
      
      ErrorRegisterValue[(X_NUCLEO_LED1642GW_BoardPresent - 1) - TempCountXNucleoBoards] = SPI_RxBuffer[1];
      ErrorRegisterValue[(X_NUCLEO_LED1642GW_BoardPresent - 1) - TempCountXNucleoBoards] = ErrorRegisterValue[(X_NUCLEO_LED1642GW_BoardPresent - 1) - TempCountXNucleoBoards] << 8;
      ErrorRegisterValue[(X_NUCLEO_LED1642GW_BoardPresent - 1) - TempCountXNucleoBoards] |= SPI_RxBuffer[0];
    }
    TempLEDErrorIndicator = 1;
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(ErrorRegisterValue[TempCountXNucleoBoards] != 0xFFFF)
      {
        TempLEDErrorIndicator = 0;
      }
    }
    if(TempLEDErrorIndicator == 0)
    {
      LEDRedON();
    }
    else
    {
      LEDGreenON();
    }
  }
}

/**
* @brief  LED ERROR DETECTION MODE: SPECIFIC FOR RGB LED BOARD (As 16th Channel is not connected)
* @param  None
* @retval None
*/
void LED_ERROR_DETECTION_RGB_LED_BOARD(void)
{    
  GPIO_InitTypeDef GPIO_InitStruct;
  uint16_t TempCountXNucleoBoards = 0;
  uint8_t TempLEDErrorIndicator = 0;
  uint16_t TempBrighnessBufferCount  = 0;
  
  /* LED ALL ON with 16th Channel Switched OFF */
  
/* Configuration Register */
  /* For eg:
  In case of 4 drivers:
  The 0th element will corresponds to 4th driver */
  
  /* In case the number of boards increases TxBuffer need to assigned the configuration value based on number of board */
  /* Here only for 6 boards the assignment has been done */
  
  TxBuffer[0] = 0x007F;
  TxBuffer[1] = 0x007F;
  TxBuffer[2] = 0x007F;
  TxBuffer[3] = 0x007F;
  TxBuffer[4] = 0x007F;
  TxBuffer[5] = 0x007F;
  
  /* Based on the muner of the boards the data is sent and LE signal is given on last buffer byte */
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Brightness Register */
  /* For 1 LED1642GW 15 Data Byte Register need to be sent */
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  TxBuffer[6] = 0xFFFF;
  TxBuffer[7] = 0xFFFF;
  TxBuffer[8] = 0xFFFF;
  TxBuffer[9] = 0xFFFF;
  TxBuffer[10] = 0xFFFF;
  TxBuffer[11] = 0xFFFF;
  TxBuffer[12] = 0xFFFF;  
  TxBuffer[13] = 0xFFFF;
  TxBuffer[14] = 0xFFFF;
  TxBuffer[15] = 0xFFFF;  
  TxBuffer[16] = 0xFFFF;
  TxBuffer[17] = 0xFFFF;
  TxBuffer[18] = 0xFFFF;  
  TxBuffer[19] = 0xFFFF;
  TxBuffer[20] = 0xFFFF;
  TxBuffer[21] = 0xFFFF;    
  TxBuffer[22] = 0xFFFF;
  TxBuffer[23] = 0xFFFF;
  TxBuffer[24] = 0xFFFF;
  TxBuffer[25] = 0xFFFF;
  TxBuffer[26] = 0xFFFF;
  TxBuffer[27] = 0xFFFF;
  TxBuffer[28] = 0xFFFF;
  TxBuffer[29] = 0xFFFF;
  TxBuffer[30] = 0xFFFF;
  TxBuffer[31] = 0xFFFF;
  TxBuffer[32] = 0xFFFF;  
  TxBuffer[33] = 0xFFFF;
  TxBuffer[34] = 0xFFFF;
  TxBuffer[35] = 0xFFFF;  
  TxBuffer[36] = 0xFFFF;
  TxBuffer[37] = 0xFFFF;
  TxBuffer[38] = 0xFFFF;    
  TxBuffer[39] = 0xFFFF;
  TxBuffer[40] = 0xFFFF;
  TxBuffer[41] = 0xFFFF;
  TxBuffer[42] = 0xFFFF;
  TxBuffer[43] = 0xFFFF;
  TxBuffer[44] = 0xFFFF;
  TxBuffer[45] = 0xFFFF;
  TxBuffer[46] = 0xFFFF;
  TxBuffer[47] = 0xFFFF;
  TxBuffer[48] = 0xFFFF;
  TxBuffer[49] = 0xFFFF;  
  TxBuffer[50] = 0xFFFF;
  TxBuffer[51] = 0xFFFF;
  TxBuffer[52] = 0xFFFF;  
  TxBuffer[53] = 0xFFFF;
  TxBuffer[54] = 0xFFFF;
  TxBuffer[55] = 0xFFFF;    
  TxBuffer[56] = 0xFFFF;
  TxBuffer[57] = 0xFFFF;
  TxBuffer[58] = 0xFFFF;
  TxBuffer[59] = 0xFFFF;
  TxBuffer[60] = 0xFFFF;
  TxBuffer[61] = 0xFFFF;
  TxBuffer[62] = 0xFFFF;
  TxBuffer[63] = 0xFFFF;
  TxBuffer[64] = 0xFFFF;
  TxBuffer[65] = 0xFFFF;
  TxBuffer[66] = 0xFFFF;  
  TxBuffer[67] = 0xFFFF;
  TxBuffer[68] = 0xFFFF;
  TxBuffer[69] = 0xFFFF;  
  TxBuffer[70] = 0xFFFF;
  TxBuffer[71] = 0xFFFF;
  TxBuffer[72] = 0xFFFF;    
  TxBuffer[73] = 0xFFFF;
  TxBuffer[74] = 0xFFFF;
  TxBuffer[75] = 0xFFFF;
  TxBuffer[76] = 0xFFFF;
  TxBuffer[77] = 0xFFFF;
  TxBuffer[78] = 0xFFFF;
  TxBuffer[79] = 0xFFFF;
  TxBuffer[80] = 0xFFFF;
  TxBuffer[81] = 0xFFFF;
  TxBuffer[82] = 0xFFFF;
  TxBuffer[83] = 0xFFFF;
  TxBuffer[84] = 0xFFFF;
  TxBuffer[85] = 0xFFFF;  
  TxBuffer[86] = 0xFFFF;
  TxBuffer[87] = 0xFFFF;
  TxBuffer[88] = 0xFFFF;    
  TxBuffer[89] = 0xFFFF;
  TxBuffer[90] = 0xFFFF;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    if(TempBrighnessBufferCount == (X_NUCLEO_LED1642GW_BoardPresent - 1))
    {
      XNUCLEO_LED1642GW_DATALATCH(TxBuffer[TempCountXNucleoBoards]); 
      TempBrighnessBufferCount = 0;
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
      TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    }
  }  
    /* Global Latch Register */
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_GLOBALLATCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Switch Control */ // 16th Channel is not used - So During Switch Control it is kept OFF
  TxBuffer[0] = 0xFFFE;
  TxBuffer[1] = 0xFFFE;
  TxBuffer[2] = 0xFFFE;
  TxBuffer[3] = 0xFFFE;
  TxBuffer[4] = 0xFFFE;
  TxBuffer[5] = 0xFFFE;
  
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  }
  /* LED ALL ON */
  
  
  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,DELAY_BETWEEN_DEMOS);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  while (DemoChange == 0)
  {     
    /* Starting Combined Error Detection */
    XNUCLEO_LED1642GW_COMBINED_ERROR_DETECTION();
    
    /* Delay */
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    
    /* Ending Error Detection Mode */
    XNUCLEO_LED1642GW_END_ERROR_DETECTION();  
    
      /* Setting GPIO for one Extra Pulse on SPI Clock */
      GPIO_InitStruct.Pin = GPIO_PIN_5;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      
      /* One Extra Pulse on SPI Clock - SCK */
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      __NOP();
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  
      
      /* ReSetting GPIO as SCK */
      GPIO_InitStruct.Pin = GPIO_PIN_5;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
      
#ifdef USE_STM32F4XX_NUCLEO
      GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
#endif
      
#ifdef USE_STM32L0XX_NUCLEO
      GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
#endif
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      
      for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
      {
        XNUCLEO_LED1642GW_DUMMY_DATA();
        
        ErrorRegisterValue[(X_NUCLEO_LED1642GW_BoardPresent - 1) - TempCountXNucleoBoards] = SPI_RxBuffer[1];
        ErrorRegisterValue[(X_NUCLEO_LED1642GW_BoardPresent - 1) - TempCountXNucleoBoards] = ErrorRegisterValue[(X_NUCLEO_LED1642GW_BoardPresent - 1) - TempCountXNucleoBoards] << 8;
        ErrorRegisterValue[(X_NUCLEO_LED1642GW_BoardPresent - 1) - TempCountXNucleoBoards] |= SPI_RxBuffer[0];
      }
      TempLEDErrorIndicator = 1;
      for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(ErrorRegisterValue[TempCountXNucleoBoards] != 0x7FFF)
      {
        TempLEDErrorIndicator = 0;
      }
    }
    if(TempLEDErrorIndicator == 0)
    {
      LEDRedON();
    }
    else
    {
      LEDGreenON();
    }
  }
}

/**
* @brief  LED Random Pattern 1 for RGB LEDs
* @param  None
* @retval None
*/
void LED_RANDOM_PATTERN1(void)
{
  uint16_t TempCountXNucleoBoards = 0;
  uint16_t TempBrighnessBufferCount  = 0;
  
  /* Configuration Register */
  /* Uncomment Number of Lines equal to number of LED1642GW drivers */
  /* For eg:
  In case of 4 drivers:
  The 0th element will corresponds to 4th driver*/
  
  TxBuffer[0] = 0x007F;
  TxBuffer[1] = 0x007F;
  TxBuffer[2] = 0x007F;
  TxBuffer[3] = 0x007F;
  TxBuffer[4] = 0x007F;
  TxBuffer[5] = 0x007F;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  }
  
  /* Brightness Register Buffer */
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    TxBuffer[TempCountXNucleoBoards] = 0xFFFF;
    if (TempBrighnessBufferCount == 16){TempBrighnessBufferCount = 0;}
  }
  
  
  /* Brightness Register */
  TempBrighnessBufferCount = 0;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    if(TempBrighnessBufferCount == (X_NUCLEO_LED1642GW_BoardPresent - 1))
    {
      XNUCLEO_LED1642GW_DATALATCH(TxBuffer[TempCountXNucleoBoards]); 
      TempBrighnessBufferCount = 0;
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
      TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    }
  }
  
  /* Global Latch Register Buffer */
  TempBrighnessBufferCount = X_NUCLEO_LED1642GW_BoardPresent;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    TxBuffer[TempCountXNucleoBoards] = 0xFFFF;
    TempBrighnessBufferCount = (TempBrighnessBufferCount - 1);
  } 
  
  /* Global Latch */  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_GLOBALLATCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,DELAY_BETWEEN_DEMOS);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  while(DemoChange == 0)
  {
    /* Switch Control */
    TxBuffer[0] = 0x1249;
    TxBuffer[1] = 0x1249;
    TxBuffer[2] = 0x1249;
    TxBuffer[3] = 0x1249;
    TxBuffer[4] = 0x1249;
    TxBuffer[5] = 0x1249;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);
    
    TxBuffer[0] = 0x4924;
    TxBuffer[1] = 0x4924;
    TxBuffer[2] = 0x4924;
    TxBuffer[3] = 0x4924;
    TxBuffer[4] = 0x4924;
    TxBuffer[5] = 0x4924;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);
    
    TxBuffer[0] = 0x2492;
    TxBuffer[1] = 0x2492;
    TxBuffer[2] = 0x2492;
    TxBuffer[3] = 0x2492;
    TxBuffer[4] = 0x2492;
    TxBuffer[5] = 0x2492;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);
    
    TxBuffer[0] = 0x5B6D;
    TxBuffer[1] = 0x5B6C;
    TxBuffer[2] = 0x5B6C;
    TxBuffer[3] = 0x5B6C;
    TxBuffer[4] = 0x5B6C;
    TxBuffer[5] = 0x5B6C;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);
    
    TxBuffer[0] = 0x36DB;
    TxBuffer[1] = 0x36DB;
    TxBuffer[2] = 0x36DB;
    TxBuffer[3] = 0x36DB;
    TxBuffer[4] = 0x36DB;
    TxBuffer[5] = 0x36DB;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);
    
    TxBuffer[0] = 0x6DB6;
    TxBuffer[1] = 0x6DB6;
    TxBuffer[2] = 0x6DB6;
    TxBuffer[3] = 0x6DB6;
    TxBuffer[4] = 0x6DB6;
    TxBuffer[5] = 0x6DB6;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);
    
    TxBuffer[0] = 0xFFFF;
    TxBuffer[1] = 0xFFFF;
    TxBuffer[2] = 0xFFFF;
    TxBuffer[3] = 0xFFFF;
    TxBuffer[4] = 0xFFFF;
    TxBuffer[5] = 0xFFFF;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);    
  }
}

/**
* @brief  RGB LED color mixing
* @param  None
* @retval None
*/
void LED_DIFFERENT_COLOR_SAME_TIME(void)
{
  uint16_t TempCountXNucleoBoards = 0;
  uint16_t TempBrighnessBufferCount  = 0;
  uint16_t TempRepeatCounter = 0;
  
  /* Configuration Register */
  /* Uncomment Number of Lines equal to number of LED1642GW drivers */
  /* For eg:
  In case of 4 drivers:
  The 0th element will corresponds to 4th driver*/
  
  TxBuffer[0] = 0x007F;
  TxBuffer[1] = 0x007F;
  TxBuffer[2] = 0x007F;
  TxBuffer[3] = 0x007F;
  TxBuffer[4] = 0x007F;
  TxBuffer[5] = 0x007F;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  }

  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,(DELAY_BETWEEN_DEMOS*2));
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  while(DemoChange == 0)
  {
  
   TempRepeatCounter = TempRepeatCounter + 0x00FF;
   if (TempRepeatCounter > 0xF000)
   {
     TempRepeatCounter = 0x1000;
   }
  /* Brightness Register Buffer */
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    TxBuffer[TempCountXNucleoBoards] = TempRepeatCounter;
    if (TempBrighnessBufferCount == 16){TempBrighnessBufferCount = 0;}
  }
  
  /* Brightness Register */
  TempBrighnessBufferCount = 0;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    if(TempBrighnessBufferCount == (X_NUCLEO_LED1642GW_BoardPresent - 1))
    {
      XNUCLEO_LED1642GW_DATALATCH(TxBuffer[TempCountXNucleoBoards]); 
      TempBrighnessBufferCount = 0;
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
      TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    }
  }
  
  /* Global Latch Register Buffer */
  TempBrighnessBufferCount = X_NUCLEO_LED1642GW_BoardPresent;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    TxBuffer[TempCountXNucleoBoards] = TempRepeatCounter;
    TempBrighnessBufferCount = (TempBrighnessBufferCount - 1);
  } 
  
  /* Global Latch */  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_GLOBALLATCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
    /* Switch Control */
    TxBuffer[0] = 0x62D5;
    TxBuffer[1] = 0x62D5;
    TxBuffer[2] = 0x62D5;
    TxBuffer[3] = 0x62D5;
    TxBuffer[4] = 0x62D5;
    TxBuffer[5] = 0x62D5;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(1);  
  }
}

/**
* @brief  RGB LED color mixing
* @param  None
* @retval None
*/
void LED_RANDOM_BRIGHTNESS(void)
{
  uint16_t TempCountXNucleoBoards = 0;
  uint16_t TempBrighnessBufferCount  = 0;
  uint16_t TempRandomBrightness = 0;
  
  /* Configuration Register */
  /* Uncomment Number of Lines equal to number of LED1642GW drivers */
  /* For eg:
  In case of 4 drivers:
  The 0th element will corresponds to 4th driver*/
  
  TxBuffer[0] = 0x007F;
  TxBuffer[1] = 0x007F;
  TxBuffer[2] = 0x007F;
  TxBuffer[3] = 0x007F;
  TxBuffer[4] = 0x007F;
  TxBuffer[5] = 0x007F;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  }

  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,(DELAY_BETWEEN_DEMOS*2));
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  while(DemoChange == 0)
  {
  
  /* Brightness Register Buffer */
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    TempRandomBrightness = LEDRandomNumberGenerator(TempRandomBrightness);
    TxBuffer[TempCountXNucleoBoards] = TempRandomBrightness;
    TempRandomBrightness = TempRandomBrightness + 0xABCD;
    if (TempBrighnessBufferCount == 16){TempBrighnessBufferCount = 0;}
  }
  
  /* Brightness Register */
  TempBrighnessBufferCount = 0;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    if(TempBrighnessBufferCount == (X_NUCLEO_LED1642GW_BoardPresent - 1))
    {
      XNUCLEO_LED1642GW_DATALATCH(TxBuffer[TempCountXNucleoBoards]); 
      TempBrighnessBufferCount = 0;
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
      TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    }
  }
  
  /* Global Latch Register Buffer */
  TempBrighnessBufferCount = X_NUCLEO_LED1642GW_BoardPresent;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    TempRandomBrightness = LEDRandomNumberGenerator(TempRandomBrightness);
    TxBuffer[TempCountXNucleoBoards] = TempRandomBrightness;
    TempRandomBrightness = TempRandomBrightness + 0xABCD;
    TempBrighnessBufferCount = (TempBrighnessBufferCount - 1);
  } 
  
  /* Global Latch */  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_GLOBALLATCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Switch Control Register Buffer */
  TempBrighnessBufferCount = X_NUCLEO_LED1642GW_BoardPresent;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    TempRandomBrightness = LEDRandomNumberGenerator(TempRandomBrightness);
    TxBuffer[TempCountXNucleoBoards] = TempRandomBrightness;
    TempRandomBrightness = TempRandomBrightness + 0xABCD;
    TempBrighnessBufferCount = (TempBrighnessBufferCount - 1);
  } 
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);  
  }
}

/**
* @brief  LED RANDOM PATTERN GNERATION for single LED
* @param  None
* @retval None
*/

void LED_RANDOM_PATTERN2(void)
{
  uint16_t TempCountXNucleoBoards = 0;
  uint16_t TempBrighnessBufferCount  = 0;
  
  /* Configuration Register */
  /* Uncomment Number of Lines equal to number of LED1642GW drivers */
  /* For eg:
  In case of 4 drivers:
  The 0th element will corresponds to 4th driver*/
  
  TxBuffer[0] = 0x007F;
  TxBuffer[1] = 0x007F;
  TxBuffer[2] = 0x007F;
  TxBuffer[3] = 0x007F;
  TxBuffer[4] = 0x007F;
  TxBuffer[5] = 0x007F;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  }
  
  /* Brightness Register Buffer */
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    TxBuffer[TempCountXNucleoBoards] = 0x0FFF * TempBrighnessBufferCount;
    if (TempBrighnessBufferCount == 16){TempBrighnessBufferCount = 0;}
  }
  
  
  /* Brightness Register */
  TempBrighnessBufferCount = 0;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    if(TempBrighnessBufferCount == (X_NUCLEO_LED1642GW_BoardPresent - 1))
    {
      XNUCLEO_LED1642GW_DATALATCH(TxBuffer[TempCountXNucleoBoards]); 
      TempBrighnessBufferCount = 0;
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
      TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    }
  }
  
  /* Global Latch Register Buffer */
  TempBrighnessBufferCount = X_NUCLEO_LED1642GW_BoardPresent;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    TxBuffer[TempCountXNucleoBoards] = 0x0FFF * TempBrighnessBufferCount;
    TempBrighnessBufferCount = (TempBrighnessBufferCount - 1);
  } 
  
  /* Global Latch */  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_GLOBALLATCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,DELAY_BETWEEN_DEMOS);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  while(DemoChange == 0)
  {
    /* Switch Control */
    TxBuffer[0] = 0xF000;
    TxBuffer[1] = 0xF000;
    TxBuffer[2] = 0xF000;
    TxBuffer[3] = 0xF000;
    TxBuffer[4] = 0xF000;
    TxBuffer[5] = 0xF000;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0xFF00;
    TxBuffer[1] = 0xFF00;
    TxBuffer[2] = 0xFF00;
    TxBuffer[3] = 0xFF00;
    TxBuffer[4] = 0xFF00;
    TxBuffer[5] = 0xFF00;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0xFFF0;
    TxBuffer[1] = 0xFFF0;
    TxBuffer[2] = 0xFFF0;
    TxBuffer[3] = 0xFFF0;
    TxBuffer[4] = 0xFFF0;
    TxBuffer[5] = 0xFFF0;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0xFFFF;
    TxBuffer[1] = 0xFFFF;
    TxBuffer[2] = 0xFFFF;
    TxBuffer[3] = 0xFFFF;
    TxBuffer[4] = 0xFFFF;
    TxBuffer[5] = 0xFFFF;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0x0FFF;
    TxBuffer[1] = 0x0FFF;
    TxBuffer[2] = 0x0FFF;
    TxBuffer[3] = 0x0FFF;
    TxBuffer[4] = 0x0FFF;
    TxBuffer[5] = 0x0FFF;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0x00FF;
    TxBuffer[1] = 0x00FF;
    TxBuffer[2] = 0x00FF;
    TxBuffer[3] = 0x00FF;
    TxBuffer[4] = 0x00FF;
    TxBuffer[5] = 0x00FF;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0x000F;
    TxBuffer[1] = 0x000F;
    TxBuffer[2] = 0x000F;
    TxBuffer[3] = 0x000F;
    TxBuffer[4] = 0x000F;
    TxBuffer[5] = 0x000F;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);    
  }
}

/**
* @brief  LED BRIGHTNESS for single channel LED
* @param  None
* @retval None
*/

void LED_BRIGHTNESS(void)
{
  uint16_t TempCountXNucleoBoards = 0;
  uint16_t TempBrighnessBufferCount  = 0;
  
  /* Configuration Register */
  /* Uncomment Number of Lines equal to number of LED1642GW drivers */
  /* For eg:
  In case of 4 drivers:
  The 0th element will corresponds to 4th driver*/
  
  TxBuffer[0] = 0x007F;
  TxBuffer[1] = 0x007F;
  TxBuffer[2] = 0x007F;
  TxBuffer[3] = 0x007F;
  TxBuffer[4] = 0x007F;
  TxBuffer[5] = 0x007F;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  }
  
  /* Brightness Register Buffer */
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    if(TempBrighnessBufferCount <= 8){
      TxBuffer[TempCountXNucleoBoards] = (TempBrighnessBufferCount * 0x1FFF);
    }
    else{
      TxBuffer[TempCountXNucleoBoards] = (0xFFF8 - ((TempBrighnessBufferCount - 9) * 0x1FFF));
      if (TempBrighnessBufferCount == 16){TempBrighnessBufferCount = 0;}
    }
  } 
  
  /* Brightness Register */
  TempBrighnessBufferCount = 0;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    if(TempBrighnessBufferCount == (X_NUCLEO_LED1642GW_BoardPresent - 1))
    {
      XNUCLEO_LED1642GW_DATALATCH(TxBuffer[TempCountXNucleoBoards]); 
      TempBrighnessBufferCount = 0;
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
      TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    }
  }
  
  /* Global Latch Register Buffer */
  TempBrighnessBufferCount = X_NUCLEO_LED1642GW_BoardPresent;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    TxBuffer[TempCountXNucleoBoards] = 0x1FFF * TempBrighnessBufferCount;
    TempBrighnessBufferCount = (TempBrighnessBufferCount - 1);
  } 
  
  /* Global Latch */  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_GLOBALLATCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,DELAY_BETWEEN_DEMOS);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  while(DemoChange == 0)
  {
    /* Switch Control */
    TxBuffer[0] = 0x0180;
    TxBuffer[1] = 0x0180;
    TxBuffer[2] = 0x0180;
    TxBuffer[3] = 0x0180;
    TxBuffer[4] = 0x0180;
    TxBuffer[5] = 0x0180;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0x03C0;
    TxBuffer[1] = 0x03C0;
    TxBuffer[2] = 0x03C0;
    TxBuffer[3] = 0x03C0;
    TxBuffer[4] = 0x03C0;
    TxBuffer[5] = 0x03C0;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0x07E0;
    TxBuffer[1] = 0x07E0;
    TxBuffer[2] = 0x07E0;
    TxBuffer[3] = 0x07E0;
    TxBuffer[4] = 0x07E0;
    TxBuffer[5] = 0x07E0;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0x0FF0;
    TxBuffer[1] = 0x0FF0;
    TxBuffer[2] = 0x0FF0;
    TxBuffer[3] = 0x0FF0;
    TxBuffer[4] = 0x0FF0;
    TxBuffer[5] = 0x0FF0;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0x1FF8;
    TxBuffer[1] = 0x1FF8;
    TxBuffer[2] = 0x1FF8;
    TxBuffer[3] = 0x1FF8;
    TxBuffer[4] = 0x1FF8;
    TxBuffer[5] = 0x1FF8;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0x3FFC;
    TxBuffer[1] = 0x3FFC;
    TxBuffer[2] = 0x3FFC;
    TxBuffer[3] = 0x3FFC;
    TxBuffer[4] = 0x3FFC;
    TxBuffer[5] = 0x3FFC;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0x7FFE;
    TxBuffer[1] = 0x7FFE;
    TxBuffer[2] = 0x7FFE;
    TxBuffer[3] = 0x7FFE;
    TxBuffer[4] = 0x7FFE;
    TxBuffer[5] = 0x7FFE;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);
    
    TxBuffer[0] = 0xFFFF;
    TxBuffer[1] = 0xFFFF;
    TxBuffer[2] = 0xFFFF;
    TxBuffer[3] = 0xFFFF;
    TxBuffer[4] = 0xFFFF;
    TxBuffer[5] = 0xFFFF;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(2);    
  }
}

/**
* @brief  LED BRIGHTNESS for RGB LED's
* @param  None
* @retval None
*/

void LED_BRIGHTNESS_RGB(void)
{
  uint16_t TempCountXNucleoBoards = 0;
  uint16_t TempBrighnessBufferCount  = 0;
  
  /* Configuration Register */
  /* Uncomment Number of Lines equal to number of LED1642GW drivers */
  /* For eg:
  In case of 4 drivers:
  The 0th element will corresponds to 4th driver*/
  
  TxBuffer[0] = 0x007F;
  TxBuffer[1] = 0x007F;
  TxBuffer[2] = 0x007F;
  TxBuffer[3] = 0x007F;
  TxBuffer[4] = 0x007F;
  TxBuffer[5] = 0x007F;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  }
  
  /* Brightness Register Buffer */
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    if(TempBrighnessBufferCount <= 8){
      TxBuffer[TempCountXNucleoBoards] = (TempBrighnessBufferCount * 0x1FFF);
    }
    else{
      TxBuffer[TempCountXNucleoBoards] = (0xFFF8 - ((TempBrighnessBufferCount - 9) * 0x1FFF));
      if (TempBrighnessBufferCount == 16){TempBrighnessBufferCount = 0;}
    }
  } 
  
  /* Brightness Register */
  TempBrighnessBufferCount = 0;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    if(TempBrighnessBufferCount == (X_NUCLEO_LED1642GW_BoardPresent - 1))
    {
      XNUCLEO_LED1642GW_DATALATCH(TxBuffer[TempCountXNucleoBoards]); 
      TempBrighnessBufferCount = 0;
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
      TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    }
  }
  
  /* Global Latch Register Buffer */
  TempBrighnessBufferCount = X_NUCLEO_LED1642GW_BoardPresent;
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    TxBuffer[TempCountXNucleoBoards] = 0x1FFF * TempBrighnessBufferCount;
    TempBrighnessBufferCount = (TempBrighnessBufferCount - 1);
  } 
  
  /* Global Latch */  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_GLOBALLATCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,DELAY_BETWEEN_DEMOS);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  while(DemoChange == 0)
  {
    /* Switch Control */
    TxBuffer[0] = 0x1249;
    TxBuffer[1] = 0x1249;
    TxBuffer[2] = 0x1249;
    TxBuffer[3] = 0x1249;
    TxBuffer[4] = 0x1249;
    TxBuffer[5] = 0x1249;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);
    
    TxBuffer[0] = 0x4924;
    TxBuffer[1] = 0x4924;
    TxBuffer[2] = 0x4924;
    TxBuffer[3] = 0x4924;
    TxBuffer[4] = 0x4924;
    TxBuffer[5] = 0x4924;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);
    
    TxBuffer[0] = 0x2492;
    TxBuffer[1] = 0x2492;
    TxBuffer[2] = 0x2492;
    TxBuffer[3] = 0x2492;
    TxBuffer[4] = 0x2492;
    TxBuffer[5] = 0x2492;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);
    
    TxBuffer[0] = 0x5B6D;
    TxBuffer[1] = 0x5B6D;
    TxBuffer[2] = 0x5B6D;
    TxBuffer[3] = 0x5B6D;
    TxBuffer[4] = 0x5B6D;
    TxBuffer[5] = 0x5B6D;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);
    
    TxBuffer[0] = 0x36DB;
    TxBuffer[1] = 0x36DB;
    TxBuffer[2] = 0x36DB;
    TxBuffer[3] = 0x36DB;
    TxBuffer[4] = 0x36DB;
    TxBuffer[5] = 0x36DB;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);
    
    TxBuffer[0] = 0x6DB6;
    TxBuffer[1] = 0x6DB6;
    TxBuffer[2] = 0x6DB6;
    TxBuffer[3] = 0x6DB6;
    TxBuffer[4] = 0x6DB6;
    TxBuffer[5] = 0x6DB6;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);
    
    TxBuffer[0] = 0xFFFF;
    TxBuffer[1] = 0xFFFF;
    TxBuffer[2] = 0xFFFF;
    TxBuffer[3] = 0xFFFF;
    TxBuffer[4] = 0xFFFF;
    TxBuffer[5] = 0xFFFF;
    
    for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
    {
      if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
      {
        XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }
      else
      {
        XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
        TxBuffer[TempCountXNucleoBoards] = TxBuffer[TempCountXNucleoBoards] << 1; 
      }   
    }
    DelayLoop(8);    
  }
}

/**
* @brief  X-NUCLEO LED1642GW LED ALL ON - LOW CURRENT RANGE (Configuration Register)
* @param  None
* @retval None
*/
void LED_LED_ON_ALL_LOW_CURRNET_RANGE(void)
{
  uint16_t TempCountXNucleoBoards = 0;
  uint16_t TempBrighnessBufferCount = 0;
  
  /* Configuration Register */
  /* For eg:
  In case of 4 drivers:
  The 0th element will corresponds to 4th driver */
  
  TxBuffer[0] = 0x003F;
  TxBuffer[1] = 0x003F;
  TxBuffer[2] = 0x003F;
  TxBuffer[3] = 0x003F;
  TxBuffer[4] = 0x003F;
  TxBuffer[5] = 0x003F;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Brightness Register */
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  TxBuffer[6] = 0xFFFF;
  TxBuffer[7] = 0xFFFF;
  TxBuffer[8] = 0xFFFF;
  TxBuffer[9] = 0xFFFF;
  TxBuffer[10] = 0xFFFF;
  TxBuffer[11] = 0xFFFF;
  TxBuffer[12] = 0xFFFF;  
  TxBuffer[13] = 0xFFFF;
  TxBuffer[14] = 0xFFFF;
  TxBuffer[15] = 0xFFFF;  
  TxBuffer[16] = 0xFFFF;
  TxBuffer[17] = 0xFFFF;
  TxBuffer[18] = 0xFFFF;  
  TxBuffer[19] = 0xFFFF;
  TxBuffer[20] = 0xFFFF;
  TxBuffer[21] = 0xFFFF;    
  TxBuffer[22] = 0xFFFF;
  TxBuffer[23] = 0xFFFF;
  TxBuffer[24] = 0xFFFF;
  TxBuffer[25] = 0xFFFF;
  TxBuffer[26] = 0xFFFF;
  TxBuffer[27] = 0xFFFF;
  TxBuffer[28] = 0xFFFF;
  TxBuffer[29] = 0xFFFF;
  TxBuffer[30] = 0xFFFF;
  TxBuffer[31] = 0xFFFF;
  TxBuffer[32] = 0xFFFF;  
  TxBuffer[33] = 0xFFFF;
  TxBuffer[34] = 0xFFFF;
  TxBuffer[35] = 0xFFFF;  
  TxBuffer[36] = 0xFFFF;
  TxBuffer[37] = 0xFFFF;
  TxBuffer[38] = 0xFFFF;    
  TxBuffer[39] = 0xFFFF;
  TxBuffer[40] = 0xFFFF;
  TxBuffer[41] = 0xFFFF;
  TxBuffer[42] = 0xFFFF;
  TxBuffer[43] = 0xFFFF;
  TxBuffer[44] = 0xFFFF;
  TxBuffer[45] = 0xFFFF;
  TxBuffer[46] = 0xFFFF;
  TxBuffer[47] = 0xFFFF;
  TxBuffer[48] = 0xFFFF;
  TxBuffer[49] = 0xFFFF;  
  TxBuffer[50] = 0xFFFF;
  TxBuffer[51] = 0xFFFF;
  TxBuffer[52] = 0xFFFF;  
  TxBuffer[53] = 0xFFFF;
  TxBuffer[54] = 0xFFFF;
  TxBuffer[55] = 0xFFFF;    
  TxBuffer[56] = 0xFFFF;
  TxBuffer[57] = 0xFFFF;
  TxBuffer[58] = 0xFFFF;
  TxBuffer[59] = 0xFFFF;
  TxBuffer[60] = 0xFFFF;
  TxBuffer[61] = 0xFFFF;
  TxBuffer[62] = 0xFFFF;
  TxBuffer[63] = 0xFFFF;
  TxBuffer[64] = 0xFFFF;
  TxBuffer[65] = 0xFFFF;
  TxBuffer[66] = 0xFFFF;  
  TxBuffer[67] = 0xFFFF;
  TxBuffer[68] = 0xFFFF;
  TxBuffer[69] = 0xFFFF;  
  TxBuffer[70] = 0xFFFF;
  TxBuffer[71] = 0xFFFF;
  TxBuffer[72] = 0xFFFF;    
  TxBuffer[73] = 0xFFFF;
  TxBuffer[74] = 0xFFFF;
  TxBuffer[75] = 0xFFFF;
  TxBuffer[76] = 0xFFFF;
  TxBuffer[77] = 0xFFFF;
  TxBuffer[78] = 0xFFFF;
  TxBuffer[79] = 0xFFFF;
  TxBuffer[80] = 0xFFFF;
  TxBuffer[81] = 0xFFFF;
  TxBuffer[82] = 0xFFFF;
  TxBuffer[83] = 0xFFFF;
  TxBuffer[84] = 0xFFFF;
  TxBuffer[85] = 0xFFFF;  
  TxBuffer[86] = 0xFFFF;
  TxBuffer[87] = 0xFFFF;
  TxBuffer[88] = 0xFFFF;    
  TxBuffer[89] = 0xFFFF;
  TxBuffer[90] = 0xFFFF;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < ((X_NUCLEO_LED1642GW_BoardPresent*NUMBER_OF_CHANNELS_PER_BOARD)-X_NUCLEO_LED1642GW_BoardPresent); TempCountXNucleoBoards ++)
  {
    if(TempBrighnessBufferCount == (X_NUCLEO_LED1642GW_BoardPresent - 1))
    {
      XNUCLEO_LED1642GW_DATALATCH(TxBuffer[TempCountXNucleoBoards]); 
      TempBrighnessBufferCount = 0;
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
      TempBrighnessBufferCount = TempBrighnessBufferCount + 1;
    }
  }
  
  /* Global Latch Register */
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_GLOBALLATCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  } 
  
  /* Switch Control */
  TxBuffer[0] = 0xFFFF;
  TxBuffer[1] = 0xFFFF;
  TxBuffer[2] = 0xFFFF;
  TxBuffer[3] = 0xFFFF;
  TxBuffer[4] = 0xFFFF;
  TxBuffer[5] = 0xFFFF;
  
  /* Setting Time Interval to switch to other Demo */
  __HAL_TIM_SET_AUTORELOAD(DELAY_TIMER,DELAY_BETWEEN_DEMOS);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(DELAY_TIMER);
  
  for (TempCountXNucleoBoards = 0; TempCountXNucleoBoards < X_NUCLEO_LED1642GW_BoardPresent; TempCountXNucleoBoards ++)
  {
    if(TempCountXNucleoBoards == (X_NUCLEO_LED1642GW_BoardPresent-1))
    {
      XNUCLEO_LED1642GW_SWITCH(TxBuffer[TempCountXNucleoBoards]); 
    }
    else
    {
      XNUCLEO_LED1642GW_SendData_SPI(TxBuffer[TempCountXNucleoBoards]);
    }
  }  
}


/**
* @brief  X-NUCLEO LED1642GW CONFIGRATION REGISTER READ
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_READ_CONFIGRATION_REGISTER(void)
{ 
  __HAL_TIM_SET_AUTORELOAD(LE_TIMER,READ_CONFIGURATION_REGISTER);
  XNUCLEO_LED1642GW_SendData_SPI(0xFFFF); 
}

/**
* @brief  X-NUCLEO LED1642GW CONFIGRATION REGISTER WRITE
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_WRITE_CONFIGRATION_REGISTER(uint16_t TempDataByte)
{ 
  __HAL_TIM_SET_AUTORELOAD(LE_TIMER,WRITE_CONFIGURATION_REGISTER);
  XNUCLEO_LED1642GW_SendData_SPI(TempDataByte); 
}

/**
* @brief  X-NUCLEO LED1642GW DATA LATCH
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_DATALATCH(uint16_t TempDataByte)
{ 
  __HAL_TIM_SET_AUTORELOAD(LE_TIMER,BRIGHTNESS_DATA_LATCH);
  XNUCLEO_LED1642GW_SendData_SPI(TempDataByte);
}

/**
* @brief  X-NUCLEO LED1642GW GLOBAL LATCH
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_GLOBALLATCH(uint16_t TempDataByte)
{ 
  __HAL_TIM_SET_AUTORELOAD(LE_TIMER,BRIGHTNESS_GLOBAL_LATCH);
  XNUCLEO_LED1642GW_SendData_SPI(TempDataByte);
}

/**
* @brief  X-NUCLEO LED1642GW SWITCH
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_SWITCH(uint16_t TempDataByte)
{ 
  __HAL_TIM_SET_AUTORELOAD(LE_TIMER,WRITE_SWITCH);
  XNUCLEO_LED1642GW_SendData_SPI(TempDataByte);
}

/**
* @brief  X-NUCLEO LED1642GW OPEN CIRCUIT ERROR DETECTION
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_OPEN_CIRCUIT_ERROR_DETECTION(void)
{ 
  __HAL_TIM_SET_AUTORELOAD(LE_TIMER,START_OPEN_ERROR_DETECTION_MODE);
  XNUCLEO_LED1642GW_SendData_SPI(0xFFFF);
}

/**
* @brief  X-NUCLEO LED1642GW SHORT CIRCUIT ERROR DETECTION
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_SHORT_CIRCUIT_ERROR_DETECTION(void)
{ 
  __HAL_TIM_SET_AUTORELOAD(LE_TIMER,START_SHORT_ERROR_DETECTION_MODE);
  XNUCLEO_LED1642GW_SendData_SPI(0xFFFF);
}

/**
* @brief  X-NUCLEO LED1642GW COMBINED OPEN AND SHORT CIRCUIT ERROR DETECTION
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_COMBINED_ERROR_DETECTION(void)
{ 
  __HAL_TIM_SET_AUTORELOAD(LE_TIMER,START_COMBINED_ERROR_DETECTION_MODE);
  XNUCLEO_LED1642GW_SendData_SPI(0xFFFF);
}

/**
* @brief  X-NUCLEO LED1642GW END ERROR DETECTION
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_END_ERROR_DETECTION(void)
{ 
  __HAL_TIM_SET_AUTORELOAD(LE_TIMER,END_ERROR_DETECTON_MODE);
  XNUCLEO_LED1642GW_SendData_SPI(0xFFFF);
}

/**
* @brief  X-NUCLEO LED1642GW DUMMY DATA
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_DUMMY_DATA(void)
{  
  __HAL_TIM_SET_AUTORELOAD(LE_TIMER,DUMMY_DATA_LED1642GW);
  XNUCLEO_LED1642GW_SendData_SPI(0xFFFF); 
}

/**
* @brief  X-NUCLEO LED1642GW DUMMY DATA
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_EXTRAPULSE(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Setting GPIO for one Extra Pulse on SPI Clock */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* One Extra Pulse on SPI Clock - SCK */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  __NOP();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  
  
  /* ReSetting GPIO as SCK */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  
#ifdef USE_STM32F4XX_NUCLEO
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
#endif
  
#ifdef USE_STM32L0XX_NUCLEO
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
#endif
  
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/**
* @brief  XNUCLEO LED1642GW Board Number Count
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_BoardCount(void)
{
  uint8_t TempBoardNumberCount = 0;
  
  SPI_TxBuffer[0] = 0x00;
  SPI_TxBuffer[1] = 0x00;   
  /* Clearing the RxBuffer */
  for(TempBoardNumberCount = 0; TempBoardNumberCount < MAXIMUM_NUMBER_OF_X_NUCLEO_BOARD; TempBoardNumberCount++)
  {
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)SPI_TxBuffer, (uint8_t *)SPI_RxBuffer, 1, 1000);
  }
  
  SPI_TxBuffer[0] = 23;  /* Parshwanath */
  SPI_TxBuffer[1] = 24;  /* Mahavir */ 
  
  for(TempBoardNumberCount = 0; TempBoardNumberCount < MAXIMUM_NUMBER_OF_X_NUCLEO_BOARD; TempBoardNumberCount++)
  {
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)SPI_TxBuffer, (uint8_t *)SPI_RxBuffer, 1, 1000);
    /* Wait for the end of the transfer */  
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
    
    SPI_TxBuffer[0] = 255;
    SPI_TxBuffer[1] = 255; 
    
    /* Checking number of X-NUCLEO boards present */
    if((SPI_RxBuffer[0] == 23) && (SPI_RxBuffer[1] == 24))
    {
      X_NUCLEO_LED1642GW_BoardPresent = TempBoardNumberCount;
    } 
  }
  if(X_NUCLEO_LED1642GW_BoardPresent == 0)
  {
    /* No board present or wrong jumper setting */
    LEDRedON(); 
  }
  else
  {
    LEDGreenON(); 
  }
}

/**
* @brief  XNUCLEO LED1642GW SPI Data
* @param  None
* @retval None
*/
void XNUCLEO_LED1642GW_SendData_SPI(uint16_t SPI_DataByte)
{
  SPI_TxBuffer[0] = SPI_DataByte;
  SPI_TxBuffer[1] = (SPI_DataByte >> 8);   
  
  SPIBufferSize = 1;
  XNUCLEO_LED1642GW_TransmitReceive_SPI(&hspi1, (uint8_t*)SPI_TxBuffer, (uint8_t *)SPI_RxBuffer, SPIBufferSize, 1000);
  /* Wait for the end of the transfer */  
  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
}

/**
  * @brief  Transmit and Receive an amount of data in blocking mode 
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for SPI module.
  * @param  pTxData: pointer to transmission data buffer
  * @param  pRxData: pointer to reception data buffer to be
  * @param  Size: amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef XNUCLEO_LED1642GW_TransmitReceive_SPI(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
  __IO uint16_t tmpreg = 0;

  if((hspi->State == HAL_SPI_STATE_READY) || (hspi->State == HAL_SPI_STATE_BUSY_RX))
  {
    if((pTxData == NULL ) || (pRxData == NULL ) || (Size == 0))
    {
      return  HAL_ERROR;
    }

    /* Check the parameters */
    assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

    /* Process Locked */
    __HAL_LOCK(hspi);
 
    /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
    if(hspi->State == HAL_SPI_STATE_READY)
    {
      hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
    }

     /* Configure communication */   
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

    hspi->pRxBuffPtr  = pRxData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;  
    
    hspi->pTxBuffPtr  = pTxData;
    hspi->TxXferSize  = Size; 
    hspi->TxXferCount = Size;

    /*Init field not used in handle to zero */
    hspi->RxISR = 0;
    hspi->TxISR = 0;

    /* Reset CRC Calculation */
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      SPI_RESET_CRC(hspi);
    }
    
    /* Check if the SPI is already enabled */ 
    if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
    {
      /* Enable SPI peripheral */
      __HAL_SPI_ENABLE(hspi);
    }
    /* Enable the Peripheral */
    __HAL_TIM_ENABLE(LE_TIMER);
    
    /* Transmit and Receive data in 16 Bit mode */
    if(hspi->Init.DataSize == SPI_DATASIZE_16BIT)
    {
      if((hspi->Init.Mode == SPI_MODE_SLAVE) || ((hspi->Init.Mode == SPI_MODE_MASTER) && (hspi->TxXferCount == 0x01)))
      {
        hspi->Instance->DR = *((uint16_t*)hspi->pTxBuffPtr);
        hspi->pTxBuffPtr+=2;
        hspi->TxXferCount--;
      }
      if(hspi->TxXferCount == 0)
      {
        /* Enable CRC Transmission */
        if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
          SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
        }
        
        /* Wait until RXNE flag is set */
        if(XNUCLEO_LED1642GW_WaitOnFlagUntilTimeout_SPI(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
        { 
          return HAL_TIMEOUT;
        }
        
#if (defined (USE_STM32F4XX_NUCLEO))
        GPIOA->BSRR = (uint32_t)GPIO_PIN_10 << 16U;  // Reset LE
#endif
        
#if (defined (USE_STM32L0XX_NUCLEO))
        GPIOA->BRR = GPIO_PIN_10;  // Reset LE
#endif 
        
        *((uint16_t*)hspi->pRxBuffPtr) = hspi->Instance->DR;
        hspi->pRxBuffPtr+=2;
        hspi->RxXferCount--;
      }
      else
      {

        /* Enable the Peripheral */
        __HAL_TIM_ENABLE(LE_TIMER); // LE
                
        while(hspi->TxXferCount > 0)
        {
          /* Wait until TXE flag is set to send data */
          if(XNUCLEO_LED1642GW_WaitOnFlagUntilTimeout_SPI(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
          { 
            return HAL_TIMEOUT;
          }

          hspi->Instance->DR = *((uint16_t*)hspi->pTxBuffPtr);
          
          hspi->pTxBuffPtr+=2;
          hspi->TxXferCount--;

          /* Enable CRC Transmission */
          if((hspi->TxXferCount == 0) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
          {
            SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
          }
          
          /* Wait until RXNE flag is set */
          if(XNUCLEO_LED1642GW_WaitOnFlagUntilTimeout_SPI(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
          { 
            return HAL_TIMEOUT;
          }
          
#if (defined (USE_STM32F4XX_NUCLEO))
          GPIOA->BSRR = (uint32_t)GPIO_PIN_10 << 16U;  // Reset LE
#endif
          
#if (defined (USE_STM32L0XX_NUCLEO))
          GPIOA->BRR = GPIO_PIN_10;  // Reset LE
#endif 
          
          *((uint16_t*)hspi->pRxBuffPtr) = hspi->Instance->DR;
          hspi->pRxBuffPtr+=2;
          hspi->RxXferCount--;
        }
        /* Receive the last byte */
        if(hspi->Init.Mode == SPI_MODE_SLAVE)
        {
          /* Wait until RXNE flag is set */
          if(XNUCLEO_LED1642GW_WaitOnFlagUntilTimeout_SPI(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }
          
          *((uint16_t*)hspi->pRxBuffPtr) = hspi->Instance->DR;
          hspi->pRxBuffPtr+=2;
          hspi->RxXferCount--;
        }
      }
    }
    /* Transmit and Receive data in 8 Bit mode */
    else
    {
      if((hspi->Init.Mode == SPI_MODE_SLAVE) || ((hspi->Init.Mode == SPI_MODE_MASTER) && (hspi->TxXferCount == 0x01)))
      {
        hspi->Instance->DR = (*hspi->pTxBuffPtr++);
        hspi->TxXferCount--;
      }
      if(hspi->TxXferCount == 0)
      {
        /* Enable CRC Transmission */
        if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
          SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
        }

        /* Wait until RXNE flag is set */
        if(XNUCLEO_LED1642GW_WaitOnFlagUntilTimeout_SPI(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        (*hspi->pRxBuffPtr) = hspi->Instance->DR;
        hspi->RxXferCount--;
      }
      else
      {
        while(hspi->TxXferCount > 0)
        {
          /* Wait until TXE flag is set to send data */
          if(XNUCLEO_LED1642GW_WaitOnFlagUntilTimeout_SPI(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }

          hspi->Instance->DR = (*hspi->pTxBuffPtr++);
          hspi->TxXferCount--;

          /* Enable CRC Transmission */
          if((hspi->TxXferCount == 0) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
          {
            SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
          }

          /* Wait until RXNE flag is set */
          if(XNUCLEO_LED1642GW_WaitOnFlagUntilTimeout_SPI(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }

          (*hspi->pRxBuffPtr++) = hspi->Instance->DR;
          hspi->RxXferCount--;
        }
        if(hspi->Init.Mode == SPI_MODE_SLAVE)
        {
          /* Wait until RXNE flag is set */
          if(XNUCLEO_LED1642GW_WaitOnFlagUntilTimeout_SPI(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }
          
          (*hspi->pRxBuffPtr++) = hspi->Instance->DR;
          hspi->RxXferCount--;
        }
      }
    }

    /* Read CRC from DR to close CRC calculation process */
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Wait until RXNE flag is set */
      if(XNUCLEO_LED1642GW_WaitOnFlagUntilTimeout_SPI(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
      {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
        return HAL_TIMEOUT;
      }
      /* Read CRC */
      tmpreg = hspi->Instance->DR;
      UNUSED(tmpreg);		/* avoid warning on tmpreg affectation with stupid compiler */
    }

    /* Wait until Busy flag is reset before disabling SPI */
    if(XNUCLEO_LED1642GW_WaitOnFlagUntilTimeout_SPI(hspi, SPI_FLAG_BSY, SET, Timeout) != HAL_OK)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
      return HAL_TIMEOUT;
    }
    
    hspi->State = HAL_SPI_STATE_READY;

    /* Check if CRC error occurred */
    if((hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR) != RESET))
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);

      SPI_RESET_CRC(hspi);

      /* Process Unlocked */
      __HAL_UNLOCK(hspi);
      
      return HAL_ERROR; 
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  This function handles SPI Communication Timeout.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for SPI module.
  * @param  Flag: SPI flag to check
  * @param  Status: Flag status to check: RESET or set
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef XNUCLEO_LED1642GW_WaitOnFlagUntilTimeout_SPI(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status, uint32_t Timeout)  
{
  uint32_t tickstart = 0;

  /* Get tick */ 
  tickstart = HAL_GetTick();

  /* Wait until flag is set */
  if(Status == RESET)
  {
    while(__HAL_SPI_GET_FLAG(hspi, Flag) == RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0) || ((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
          __HAL_SPI_DISABLE_IT(hspi, (uint32_t)(SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
          {
            SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while(__HAL_SPI_GET_FLAG(hspi, Flag) != RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0) || ((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
          __HAL_SPI_DISABLE_IT(hspi, (uint32_t)(SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
          {
            SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}

/**
* @brief  Delay
* @param  Loop Count for Delay
* @retval None
*/
void DelayLoop(uint32_t DelayLoopCount)
{
  uint32_t TempLoopCount = DelayLoopCount*25;
  HAL_Delay(TempLoopCount);
}

/**
* @brief  Green LED ON - OK
* @param  
* @retval None
*/
void LEDGreenON()
{
  HAL_GPIO_WritePin(LED_Indication_GPIO_Port, LED_Indication_Pin, GPIO_PIN_SET);
}

/**
* @brief  Red LED ON - FAULT
* @param  
* @retval None
*/
void LEDRedON()
{
  HAL_GPIO_WritePin(LED_Indication_GPIO_Port, LED_Indication_Pin, GPIO_PIN_RESET);
}

/**
* @brief  Random Number Generation
* @param  
* @retval Random Number
*/
int LEDRandomNumberGenerator(int Limit)
{
  /*Returns the random integer between 0 and lim*/
  static long var = 3;
  var = (((var * 214013L + 2531011L) >> 16) & 32767);
  return ((var % Limit) + 1);
}

/** @defgroup X_NUCLEO_LED1642GW_Exported_Functions X_NUCLEO_LED1642GW_Exported_Functions
* @{
*/

/** @defgroup X_NUCLEO_LED1642GW_Private_Functions X_NUCLEO_LED1642GW_Private_Functions
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
