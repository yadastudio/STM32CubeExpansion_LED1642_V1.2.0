/**
 ******************************************************************************
 * @file    led1642gw.h
 * @author  CL
 * @version V1.0.0
 * @date    25-January-2016
 * @brief   This file contains definitions for the led1642gw.c
 *          firmware driver.
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
#ifndef __LED1642GW_H
#define __LED1642GW_H

#ifdef __cplusplus
extern "C" {
#endif
  
  /* Includes ------------------------------------------------------------------*/
#include "x_nucleo_led1642gw.h"
#include "main.h"  
  
  /** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
 * @{
 */

/** @addtogroup LED1642GW
 * @{
 */

/** @defgroup LED1642GW_Exported_Defines LED1642GW_Exported_Defines
 * @{
 */
 
#ifdef SPIBaudPreScaler_16
#define DUMMY_DATA_LED1642GW                            240  
#define WRITE_SWITCH                                    210
#define BRIGHTNESS_DATA_LATCH                           180
#define BRIGHTNESS_GLOBAL_LATCH                         150
#define WRITE_CONFIGURATION_REGISTER                    120
#define READ_CONFIGURATION_REGISTER                     105
#define START_OPEN_ERROR_DETECTION_MODE                 85
#define START_SHORT_ERROR_DETECTION_MODE                70
#define START_COMBINED_ERROR_DETECTION_MODE             55
#define END_ERROR_DETECTON_MODE                         40
#define RESERVED     
#endif     

#ifdef SPIBaudPreScaler_32
#define DUMMY_DATA_LED1642GW                            500  
#define WRITE_SWITCH                                    440
#define BRIGHTNESS_DATA_LATCH                           380
#define BRIGHTNESS_GLOBAL_LATCH                         320
#define WRITE_CONFIGURATION_REGISTER                    260
#define READ_CONFIGURATION_REGISTER                     230
#define START_OPEN_ERROR_DETECTION_MODE                 195
#define START_SHORT_ERROR_DETECTION_MODE                160
#define START_COMBINED_ERROR_DETECTION_MODE             130
#define END_ERROR_DETECTON_MODE                         100
#define THERMAL_ERROR_READING                           70
#define RESERVED  
#endif

   
/**
 * @}
 */

/** @defgroup LED1642GW_Imported_Functions LED1642GW_Imported_Functions
 * @{
 */   
  
/** @addtogroup LED1642GW_Exported_Variables LED1642GW_Exported_Variables
 * @{
 */

/** @addtogroup LED1642GW_Exported_Variables LED1642GW_Exported_Functions
 * @{
 */  
void ConfigurationRegisterRead(void);
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

#ifdef __cplusplus
}
#endif

#endif /* __LED1642GW_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
