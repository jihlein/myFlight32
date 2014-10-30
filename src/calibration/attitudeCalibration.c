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

uint16_t attCalibrationCount = 0;

float rollSum  = 0.0f;
float pitchSum = 0.0f;

///////////////////////////////////////////////////////////////////////////////
// Attitude Calibration
///////////////////////////////////////////////////////////////////////////////

void attCalibration(void)
{
	if (attCalibrationCount == 500)
	{
	    cliPortPrint("\nStart Attitude Calibration....\n\n");

	    rollSum  = 0.0f;
	    pitchSum = 0.0f;

	    eepromConfig.attTrim[ROLL ] = 0.0f;
        eepromConfig.attTrim[PITCH] = 0.0f;
    }

    rollSum  += sensors.attitude500Hz[ROLL ];
    pitchSum += sensors.attitude500Hz[PITCH];

	if (attCalibrationCount == 1)
	{
	    eepromConfig.attTrim[ROLL ] = rollSum  / 500.0f;
        eepromConfig.attTrim[PITCH] = pitchSum / 500.0f;

        cliPortPrint("End Attitude Calibration....\n\n");
	}

    attCalibrationCount--;
}

///////////////////////////////////////////////////////////////////////////////
