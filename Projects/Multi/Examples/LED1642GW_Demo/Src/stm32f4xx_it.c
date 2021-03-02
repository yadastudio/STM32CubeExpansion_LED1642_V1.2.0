/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-March-2016
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "led1642gw.h"

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim10;   
extern TIM_HandleTypeDef htim11;

extern uint8_t DemoNumber;
extern uint8_t DemoChange;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
}

/**
  * @brief  This function handle TIM10 Interrupt.
  * @param  None
  * @retval None
*/
void TIM1_UP_TIM10_IRQHandler(void)
{ 
  /* Disable the Peripheral */
  __HAL_TIM_DISABLE(&htim10); 
  
  /* Setting LE PIN of LED Driver LED1642 */
  GPIOA->BSRR = GPIO_PIN_10;
  
  /* Reseting the Parameter */
  __HAL_TIM_SET_COUNTER(&htim10,0);
  __HAL_TIM_SET_AUTORELOAD(&htim10,DUMMY_DATA_LED1642GW);
  
  /* Clearing Interrupt */  
  HAL_TIM_IRQHandler(&htim10);
}

/**
* @brief This function handles TIM11 global interrupt.
*/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{ 
  /* Disable the Peripheral */
  __HAL_TIM_DISABLE(&htim11);
  
  /* Changing Demo Number */
  DemoNumber = DemoNumber + 1;
  if(DemoNumber == 9)
  {
    DemoNumber = 0;
  }
  /* Setting Demo Change Bit */
  DemoChange = 1;
  /* Clearing Interrupt */
  HAL_TIM_IRQHandler(&htim11);
}

/**
* @brief This function handles interrupt B1 Push Button on NUCLEO board.
  This function handles EXTI line[15:10] interrupt
*/
void EXTI15_10_IRQHandler(void)
{
  /* Changing Demo Number */
  DemoNumber = DemoNumber + 1;
  if(DemoNumber == 9)
  {
    DemoNumber = 0;
  }
  /* Setting Demo Change Bit */
  DemoChange = 1;
  
  /* Clearing Interrupt */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
