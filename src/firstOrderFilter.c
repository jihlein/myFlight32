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

// TAU = Filter Time Constant
// T   = Filter Sample Time

// A   = 2 * TAU / T

// Low Pass:
// GX1 = 1 / (1 + A)
// GX2 = 1 / (1 + A)
// GX3 = (1 - A) / (1 + A)

// High Pass:
// GX1 =  A / (1 + A)
// GX2 = -A / (1 + A)
// GX3 = (1 - A) / (1 + A)

///////////////////////////////////////

#define PRESSURE_ALT_LOWPASS_TAU         0.05f
#define PRESSURE_ALT_LOWPASS_SAMPLE_TIME 0.02f
#define PRESSURE_ALT_LOWPASS_A           (2.0f * PRESSURE_ALT_LOWPASS_TAU / PRESSURE_ALT_LOWPASS_SAMPLE_TIME)
#define PRESSURE_ALT_LOWPASS_GX1         (1.0f / (1.0f + PRESSURE_ALT_LOWPASS_A))
#define PRESSURE_ALT_LOWPASS_GX2         (1.0f / (1.0f + PRESSURE_ALT_LOWPASS_A))
#define PRESSURE_ALT_LOWPASS_GX3         ((1.0f - PRESSURE_ALT_LOWPASS_A) / (1.0f + PRESSURE_ALT_LOWPASS_A))

///////////////////////////////////////////////////////////////////////////////

void initFirstOrderFilter(void)
{
    firstOrderFilters[PRESSURE_ALT_LOWPASS].gx1 = PRESSURE_ALT_LOWPASS_GX1;
	firstOrderFilters[PRESSURE_ALT_LOWPASS].gx2 = PRESSURE_ALT_LOWPASS_GX2;
	firstOrderFilters[PRESSURE_ALT_LOWPASS].gx3 = PRESSURE_ALT_LOWPASS_GX3;
	firstOrderFilters[PRESSURE_ALT_LOWPASS].previousInput  = sensors.pressureAlt50Hz;
    firstOrderFilters[PRESSURE_ALT_LOWPASS].previousOutput = sensors.pressureAlt50Hz;
}

///////////////////////////////////////////////////////////////////////////////

float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters)
{
    float output;

    output = filterParameters->gx1 * input +
             filterParameters->gx2 * filterParameters->previousInput -
             filterParameters->gx3 * filterParameters->previousOutput;

    filterParameters->previousInput  = input;
    filterParameters->previousOutput = output;

    return output;
}

///////////////////////////////////////////////////////////////////////////////


