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

#define ONBOARD_LED1_PIN   GPIO_Pin_12
#define ONBOARD_LED1_GPIO  GPIOB

#define BEEP_PIN           GPIO_Pin_10
#define BEEP_GPIO          GPIOB

///////////////////////////////////////

#define ONBOARD_LED1_OFF     GPIO_SetBits(ONBOARD_LED1_GPIO,    ONBOARD_LED1_PIN)
#define ONBOARD_LED1_ON      GPIO_ResetBits(ONBOARD_LED1_GPIO,  ONBOARD_LED1_PIN)
#define ONBOARD_LED1_TOGGLE  GPIO_ToggleBits(ONBOARD_LED1_GPIO, ONBOARD_LED1_PIN)


#define ONBOARD_LED2_OFF
#define ONBOARD_LED2_ON
#define ONBOARD_LED2_TOGGLE

#define EXTERNAL_LED1_OFF
#define EXTERNAL_LED1_ON
#define EXTERNAL_LED1_TOGGLE

#define EXTERNAL_LED2_OFF
#define EXTERNAL_LED2_ON
#define EXTERNAL_LED2_TOGGLE

#define EXTERNAL_LED3_OFF
#define EXTERNAL_LED3_ON
#define EXTERNAL_LED3_TOGGLE

#define EXTERNAL_LED4_OFF
#define EXTERNAL_LED4_ON
#define EXTERNAL_LED4_TOGGLE

#define BEEP_OFF    GPIO_ResetBits(BEEP_GPIO,  BEEP_PIN)
#define BEEP_ON     GPIO_SetBits(BEEP_GPIO,    BEEP_PIN)
#define BEEP_TOGGLE GPIO_ToggleBits(BEEP_GPIO, BEEP_PIN)

///////////////////////////////////////////////////////////////////////////////
// GPIO Initialization
///////////////////////////////////////////////////////////////////////////////

void gpioInit(void);

///////////////////////////////////////////////////////////////////////////////
