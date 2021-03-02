/**
  @page RGB Demo Application based on X-NUCLEO LED1642GW expansion board and STM32 based STM32L053R8 and STM32F401RE Nucleo Boards
  
  @verbatim
  ******************** (C) COPYRIGHT 2017 STMicroelectronics *******************
  * @file    readme.txt  
  * @version V1.2.0
  * @date    5-April-2018
  * @brief   This application contains an example which shows LED1642GW functionality.
  ******************************************************************************
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
  @endverbatim

@par Example Description 

Main function is to demonstrates various features of LED1642GW LED Driver.
Default firmware has few LED1642GW DEMO to control various parameters of LED's.

@par Hardware and Software requirement

Hardware: This example runs on NUCLEO-L053R8 and NUCLEO-F401RE.

Software: The software has been tested on following compilers.
 - IAR Embedded Workbench for ARM (EWARM) toolchain V7.60 + ST-Link.
 - RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.18 + ST-LINK.
 - AC6 System Workbench for STM32 V4.4.2 + ST-LINK.
    
@par How to use it ?

To use X-nUCLEO-LED1642GW, the following connections of hardware is to be unsured:

 - Jumper Settings 
	* J4 - (1&2 - Power Source - USB) (2&3 - Power Source - External Power Supply).
	* J2 - (1&2 - SDI pin connected to MCU) (2&3 - SDI pin connected to SDO of adjacent below X-NUCLEO-LED1642GW).
	* J3 - (1&2 - SDO pin connected to MCU) (2&3 - SDO pin connected to SDI of adjacent above X-NUCLEO-LED1642GW).
Note - In case of Daisy chain configuration first X-NUCLEO-LED1642GW (adjacent to NUCLEO board) board J2 setting should be (1&2) and last board J3 setting should be (2&3).


 - X-NUCLEO-LED1642GW board can be fitted directly to NUCEO boards with the help of CN5,CN6, CN8 and CN9 connectors on X-NUCLEO-LED1642GW.

To use this example, the following steps have to be followed:
 - Open project file.
 - Rebuild all the project and load the image into the target memory.
 - Run the example.
 - User can modify and control the LED brightness and can also build new functions as per requirement.

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
