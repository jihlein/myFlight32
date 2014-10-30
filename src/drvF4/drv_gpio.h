/*
  August 2014

  myFlight32 Rev -

  Copyright (c) 2014 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Control Software

  Designed to run on Naze32Pro and AQ32 Flight Control Boards

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)MultiWii
  4)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX
  7)Me!!

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////////////
// GPIO Defines
////////////////////////////////////////////////////////////////////////////////

#define ONBOARD_LED1_GPIO   GPIOE
#define ONBOARD_LED1_PIN    GPIO_Pin_5

#define ONBOARD_LED2_GPIO   GPIOE
#define ONBOARD_LED2_PIN    GPIO_Pin_6

#define EXTERNAL_LED1_GPIO  GPIOD
#define EXTERNAL_LED1_PIN   GPIO_Pin_7

#define EXTERNAL_LED2_GPIO  GPIOE
#define EXTERNAL_LED2_PIN   GPIO_Pin_0

#define EXTERNAL_LED3_GPIO  GPIOE
#define EXTERNAL_LED3_PIN   GPIO_Pin_1

#define EXTERNAL_LED4_GPIO  GPIOD
#define EXTERNAL_LED4_PIN   GPIO_Pin_4

///////////////////////////////////////

#define ONBOARD_LED1_OFF     GPIO_ResetBits(ONBOARD_LED1_GPIO,   ONBOARD_LED1_PIN)
#define ONBOARD_LED1_ON      GPIO_SetBits(ONBOARD_LED1_GPIO,     ONBOARD_LED1_PIN)
#define ONBOARD_LED1_TOGGLE  GPIO_ToggleBits(ONBOARD_LED1_GPIO,  ONBOARD_LED1_PIN)

#define ONBOARD_LED2_OFF     GPIO_ResetBits(ONBOARD_LED2_GPIO,   ONBOARD_LED2_PIN)
#define ONBOARD_LED2_ON      GPIO_SetBits(ONBOARD_LED2_GPIO,     ONBOARD_LED2_PIN)
#define ONBOARD_LED2_TOGGLE  GPIO_ToggleBits(ONBOARD_LED2_GPIO,  ONBOARD_LED2_PIN)

#define EXTERNAL_LED1_OFF    GPIO_ResetBits(EXTERNAL_LED1_GPIO,  EXTERNAL_LED1_PIN)
#define EXTERNAL_LED1_ON     GPIO_SetBits(EXTERNAL_LED1_GPIO,    EXTERNAL_LED1_PIN)
#define EXTERNAL_LED1_TOGGLE GPIO_ToggleBits(EXTERNAL_LED1_GPIO, EXTERNAL_LED1_PIN)

#define EXTERNAL_LED2_OFF    GPIO_ResetBits(EXTERNAL_LED2_GPIO,  EXTERNAL_LED2_PIN)
#define EXTERNAL_LED2_ON     GPIO_SetBits(EXTERNAL_LED2_GPIO,    EXTERNAL_LED2_PIN)
#define EXTERNAL_LED2_TOGGLE GPIO_ToggleBits(EXTERNAL_LED2_GPIO, EXTERNAL_LED2_PIN)

#define EXTERNAL_LED3_OFF    GPIO_ResetBits(EXTERNAL_LED3_GPIO,  EXTERNAL_LED3_PIN)
#define EXTERNAL_LED3_ON     GPIO_SetBits(EXTERNAL_LED3_GPIO,    EXTERNAL_LED3_PIN)
#define EXTERNAL_LED3_TOGGLE GPIO_ToggleBits(EXTERNAL_LED3_GPIO, EXTERNAL_LED3_PIN)

#define EXTERNAL_LED4_OFF    GPIO_ResetBits(EXTERNAL_LED4_GPIO,  EXTERNAL_LED4_PIN)
#define EXTERNAL_LED4_ON     GPIO_SetBits(EXTERNAL_LED4_GPIO,    EXTERNAL_LED4_PIN)
#define EXTERNAL_LED4_TOGGLE GPIO_ToggleBits(EXTERNAL_LED4_GPIO, EXTERNAL_LED4_PIN)

#define BEEP_OFF
#define BEEP_ON
#define BEEP_TOGGLE

///////////////////////////////////////////////////////////////////////////////
// GPIO Initialization
///////////////////////////////////////////////////////////////////////////////

void gpioInit(void);

///////////////////////////////////////////////////////////////////////////////
