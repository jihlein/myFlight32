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
// PWM Servo Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define SERVO_PULSE_1p5MS           3000  // 1.5 ms pulse width

static volatile uint32_t *OutputChannels[] = { &(TIM4->CCR1),
	                                           &(TIM4->CCR2),
	                                           &(TIM4->CCR3),
	                                           &(TIM4->CCR4), };

///////////////////////////////////////////////////////////////////////////////
// PWM Servo Initialization
///////////////////////////////////////////////////////////////////////////////

void pwmServoInit(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    // Outputs
    // SERVO PWM1 TIM4 CH1 PD12
    // SERVO PWM2 TIM4 CH2 PD13
    // SERVO PWM3 TIM4 CH3 PD14
    // SERVO PWM4 TIM4 CH4 PD15

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Output timers

    TIM_TimeBaseStructure.TIM_Period            = (uint16_t)(2000000 / eepromConfig.servoPwmRate) - 1;
    TIM_TimeBaseStructure.TIM_Prescaler         = 42 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = SERVO_PULSE_1p5MS;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);

    TIM_Cmd(TIM4, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// PWM Servo Write
///////////////////////////////////////////////////////////////////////////////

void pwmServoWrite(uint8_t channel, uint16_t value)
{
    *OutputChannels[channel] = value;
}

///////////////////////////////////////////////////////////////////////////////
