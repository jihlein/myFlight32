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

uint16_t gyroRunTimeCalCount = 2520;

float rollGyroSum  = 0.0f;
float pitchGyroSum = 0.0f;
float yawGyroSum   = 0.0f;

///////////////////////////////////////////////////////////////////////////////
// Gyro Run Time Calibration
///////////////////////////////////////////////////////////////////////////////

void gyroRunTimeCal(void)
{
	if (gyroRunTimeCalCount <= 2500)
	{
		if (gyroRunTimeCalCount == 2500)
        {
	        rollGyroSum  = 0.0f;
			pitchGyroSum = 0.0f;
            yawGyroSum   = 0.0f;
        }

        rollGyroSum  += nonRotatedGyroData[ROLL ];
        pitchGyroSum += nonRotatedGyroData[PITCH];
        yawGyroSum   += nonRotatedGyroData[YAW  ];

	    if (gyroRunTimeCalCount == 1)
	    {
	    	gyroRTBias[ROLL ] = rollGyroSum  / 2500.0f;
            gyroRTBias[PITCH] = pitchGyroSum / 2500.0f;
            gyroRTBias[YAW  ] = yawGyroSum   / 2500.0f;

            gyroValid = true;;
	    }
    }

    gyroRunTimeCalCount--;
}

///////////////////////////////////////////////////////////////////////////////
