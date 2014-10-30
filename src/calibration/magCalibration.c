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

float * d;

uint16_t calibrationCounter = 0;

uint8_t magCalibrationState = 0;
uint8_t magCalibrating      = false;

///////////////////////////////////////////////////////////////////////////////
// Mag Calibration
///////////////////////////////////////////////////////////////////////////////

void magCalibration(void)
{
	uint16_t population[2][3];

	float    sphereOrigin[3];
	float    sphereRadius;

	switch(magCalibrationState)
	{
		///////////////////////////////

		case 0:  // Initialize
		    d = (float*)malloc(600 * 3 * sizeof(float));

		    if (d == NULL)
		    {
				cliPortPrint("Memory Allocation Error!!!!\n\n");
				cliPortPrint("Magnetometer Calibration Aborted....\n\n");

				magCalibrating = false;

				return;
			}

		    calibrationCounter = 0;

		    eepromConfig.magBias[XAXIS + eepromConfig.externalHMC5883] = 0.0f;
		    eepromConfig.magBias[YAXIS + eepromConfig.externalHMC5883] = 0.0f;
		    eepromConfig.magBias[ZAXIS + eepromConfig.externalHMC5883] = 0.0f;

		    cliPortPrint("\nMagnetometer Calibration....\n\n");

            cliPortPrint("Rotate magnetometer around all axes multiple times\n");
            cliPortPrint("Must complete within 60 seconds....\n\n");
            cliPortPrint("  Send a character when ready to begin and another when complete....\n\n");

		    magCalibrationState++;

		    break;

		///////////////////////////////

		case 1:  // Wait Until Ready
            if (cliPortAvailable() == true)
            {
				cliPortRead();
				cliPortPrint("    Start Rotations....\n\n");
				cliPortPrint("    Gathering Data....\n\n");

				magCalibrationState++;
			}

            break;

        ///////////////////////////////

        case 2:  // Gather Data
            if (calibrationCounter < 600)
        	{
			    d[calibrationCounter * 3 + XAXIS] = nonRotatedMagData[XAXIS];
			    d[calibrationCounter * 3 + YAXIS] = nonRotatedMagData[YAXIS];
			    d[calibrationCounter * 3 + ZAXIS] = nonRotatedMagData[ZAXIS];

			    calibrationCounter++;
		    }
            else
                magCalibrationState++;

            if (cliPortAvailable() == true)
            {
				cliPortRead();
				magCalibrationState++;
			}

			break;

		///////////////////////////////

		case 3:  // Perform Computations
		    cliPortPrintF("Magnetometer Bias Calculation, %3ld samples collected out of 600 max)\n\n", calibrationCounter);

	        sphereFit(d, calibrationCounter, 100, 0.0f, population, sphereOrigin, &sphereRadius);

	        eepromConfig.magBias[XAXIS + eepromConfig.externalHMC5883] = sphereOrigin[XAXIS];
	        eepromConfig.magBias[YAXIS + eepromConfig.externalHMC5883] = sphereOrigin[YAXIS];
	        eepromConfig.magBias[ZAXIS + eepromConfig.externalHMC5883] = sphereOrigin[ZAXIS];

            cliPortPrintF("Mag Bias: %9.4f, %9.4f, %9.4f\n", eepromConfig.magBias[XAXIS + eepromConfig.externalHMC5883],
                                                             eepromConfig.magBias[YAXIS + eepromConfig.externalHMC5883],
                                                             eepromConfig.magBias[ZAXIS + eepromConfig.externalHMC5883]);
	        free(d);

	        cliPortPrint("\nMagnetometer Calibration Complete....\n\n");

	        magCalibrating = false;

	        break;

	    //////////////////////////////
	}
}

///////////////////////////////////////////////////////////////////////////////
