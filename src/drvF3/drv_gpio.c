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

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// GPIO Initialization
///////////////////////////////////////////////////////////////////////////////

void gpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin   = BEEP_PIN | ONBOARD_LED1_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	ONBOARD_LED1_OFF;
	ONBOARD_LED2_OFF;

	EXTERNAL_LED1_OFF;
	EXTERNAL_LED2_OFF;
	EXTERNAL_LED3_OFF;
	EXTERNAL_LED4_OFF;

	BEEP_OFF;
}

///////////////////////////////////////////////////////////////////////////////
