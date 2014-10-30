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

uint16_t accelOneGCalcCount = 520;

float xAccelSum = 0.0f;
float yAccelSum = 0.0f;
float zAccelSum = 0.0f;

///////////////////////////////////////////////////////////////////////////////
// Accel One G Calculation
///////////////////////////////////////////////////////////////////////////////

void accelOneGCalc(void)
{
	if (accelOneGCalcCount <= 500)
	{
		if (accelOneGCalcCount == 500)
        {
	        xAccelSum = 0.0f;
	        yAccelSum = 0.0f;
	        zAccelSum = 0.0f;
        }

        xAccelSum += sensors.accel100Hz[XAXIS];
        yAccelSum += sensors.accel100Hz[YAXIS];
        zAccelSum += sensors.accel100Hz[ZAXIS];

	    if (accelOneGCalcCount == 1)
	    {
	        xAccelSum /= 500.0f;
            yAccelSum /= 500.0f;
            zAccelSum /= 500.0f;

	        accelOneG = sqrt(SQR(xAccelSum) + SQR(yAccelSum) + SQR(zAccelSum));

            accelValid = true;
	    }
    }

    accelOneGCalcCount--;
}

///////////////////////////////////////////////////////////////////////////////

float noseUpSum        = 0.0f;
float noseDownSum      = 0.0f;
float leftWingDownSum  = 0.0f;
float rightWingDownSum = 0.0f;
float upSideDownSum    = 0.0f;
float rightSideUpSum   = 0.0f;

float xBiasSum         = 0.0f;
float yBiasSum         = 0.0f;
float zBiasSum         = 0.0f;

uint16_t accelCalibrationCount;

uint8_t  accelCalibrationState = 0;
uint8_t  accelCalibrating      = false;

///////////////////////////////////////////////////////////////////////////////
// Accel Calibration
///////////////////////////////////////////////////////////////////////////////

void accelCalibration(void)
{
    arm_matrix_instance_f32 a;
	arm_matrix_instance_f32 b;
	arm_matrix_instance_f32 x;

	float nonRotatedAccelData[3];

	switch(accelCalibrationState)
    {
		///////////////////////////////

		case 0:  // Initialize
            noseUpSum        = 0.0f;
			noseDownSum      = 0.0f;
			leftWingDownSum  = 0.0f;
			rightWingDownSum = 0.0f;
			upSideDownSum    = 0.0f;
            rightSideUpSum   = 0.0f;

            xBiasSum         = 0.0f;
            yBiasSum         = 0.0f;
            zBiasSum         = 0.0f;

            eepromConfig.accelBias[XAXIS + eepromConfig.useMXR9150] = 0.0f;
            eepromConfig.accelBias[YAXIS + eepromConfig.useMXR9150] = 0.0f;
            eepromConfig.accelBias[ZAXIS + eepromConfig.useMXR9150] = 0.0f;

            eepromConfig.accelScaleFactor[XAXIS + eepromConfig.useMXR9150] = 1.0f;
            eepromConfig.accelScaleFactor[YAXIS + eepromConfig.useMXR9150] = 1.0f;
            eepromConfig.accelScaleFactor[ZAXIS + eepromConfig.useMXR9150] = 1.0f;

            cliPortPrint("\nAccelerometer Calibration....\n\n");

            accelCalibrationState++;

		    break;

		///////////////////////////////

		case 1:  // Setup Right Side Up Case
            cliPortPrint("Place accelerometer right side up\n");
            cliPortPrint("  Send a character when ready to proceed\n\n");

            accelCalibrationCount = 500;

            accelCalibrationState++;

		    break;

		///////////////////////////////

		case 2:  // Wait for Right Side Up Setup
            if (cliPortAvailable() == true)
            {
				cliPortRead();
				cliPortPrint("  Gathering Data....\n\n");
				accelCalibrationState++;
			}

		    break;

		///////////////////////////////

		case 3:  // Collect Right Side Up Data
            if (accelCalibrationCount > 0)
            {
				arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrixT);

				arm_mat_init_f32(&b, 3, 1, (float *)sensors.accel100Hz);   // Body Coordinate Frame

		        arm_mat_init_f32(&x, 3, 1,          nonRotatedAccelData);  // Sensor Coordinate Frame

		        arm_mat_mult_f32(&a, &b, &x);

		        rightSideUpSum += nonRotatedAccelData[ZAXIS];

                xBiasSum       += nonRotatedAccelData[XAXIS];
                yBiasSum       += nonRotatedAccelData[YAXIS];
			}
			else
			{
				accelCalibrationState++;
			}

			accelCalibrationCount--;

		    break;

		///////////////////////////////

		case 4:  // Setup Upside Down Case
            cliPortPrint("Place accelerometer up side down\n");
            cliPortPrint("  Send a character when ready to proceed\n\n");

            accelCalibrationCount = 500;

            accelCalibrationState++;

		    break;

		///////////////////////////////

		case 5:  // Wait for Up Side Down Setup
            if (cliPortAvailable() == true)
            {
				cliPortRead();
				cliPortPrint("  Gathering Data....\n\n");
				accelCalibrationState++;
			}

		    break;

		///////////////////////////////

		case 6:  // Collect Up Side Down Data
            if (accelCalibrationCount > 0)
            {
				arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrixT);

				arm_mat_init_f32(&b, 3, 1, (float *)sensors.accel100Hz);   // Body Coordinate Frame

		        arm_mat_init_f32(&x, 3, 1,          nonRotatedAccelData);  // Sensor Coordinate Frame

		        arm_mat_mult_f32(&a, &b, &x);

				upSideDownSum += nonRotatedAccelData[ZAXIS];

                xBiasSum      += nonRotatedAccelData[XAXIS];
                yBiasSum      += nonRotatedAccelData[YAXIS];
			}
			else
			{
				accelCalibrationState++;
			}

			accelCalibrationCount--;

		    break;

		///////////////////////////////

		case 7:  // Setup Left Edge Down Case
            cliPortPrint("Place accelerometer left edge down\n");
            cliPortPrint("  Send a character when ready to proceed\n\n");

            accelCalibrationCount = 500;

            accelCalibrationState++;

		    break;

		///////////////////////////////

		case 8:  // Wait for Left Edge Down Setup
            if (cliPortAvailable() == true)
            {
				cliPortRead();
				cliPortPrint("  Gathering Data....\n\n");
				accelCalibrationState++;
			}

		    break;

		///////////////////////////////

		case 9:  // Collect Left Edge Down Data
            if (accelCalibrationCount > 0)
            {
				arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrixT);

				arm_mat_init_f32(&b, 3, 1, (float *)sensors.accel100Hz);   // Body Coordinate Frame

		        arm_mat_init_f32(&x, 3, 1,          nonRotatedAccelData);  // Sensor Coordinate Frame

		        arm_mat_mult_f32(&a, &b, &x);

				leftWingDownSum += nonRotatedAccelData[YAXIS];

                zBiasSum        += nonRotatedAccelData[ZAXIS];
			}
			else
			{
				accelCalibrationState++;
			}

			accelCalibrationCount--;

		    break;

		///////////////////////////////

		case 10:  // Setup Right Edge Down Case
            cliPortPrint("Place accelerometer right edge down\n");
            cliPortPrint("  Send a character when ready to proceed\n\n");

            accelCalibrationCount = 500;

            accelCalibrationState++;

		    break;

		///////////////////////////////

		case 11:  // Wait for Right Edge Down Setup
            if (cliPortAvailable() == true)
            {
				cliPortRead();
				cliPortPrint("  Gathering Data....\n\n");
				accelCalibrationState++;
			}

		    break;

		///////////////////////////////

		case 12:  // Collect Right Edge Down Data
            if (accelCalibrationCount > 0)
            {
				arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrixT);

				arm_mat_init_f32(&b, 3, 1, (float *)sensors.accel100Hz);   // Body Coordinate Frame

		        arm_mat_init_f32(&x, 3, 1,          nonRotatedAccelData);  // Sensor Coordinate Frame

		        arm_mat_mult_f32(&a, &b, &x);

				rightWingDownSum += nonRotatedAccelData[YAXIS];

                zBiasSum         += nonRotatedAccelData[ZAXIS];
			}
			else
			{
				accelCalibrationState++;
			}

			accelCalibrationCount--;

		    break;

		///////////////////////////////

		case 13:  // Setup Rear Edge Down Case
            cliPortPrint("Place accelerometer rear edge down\n");
            cliPortPrint("  Send a character when ready to proceed\n\n");

            accelCalibrationCount = 500;

            accelCalibrationState++;

		    break;

		///////////////////////////////

		case 14:  // Wait for Rear Edge Down Setup
            if (cliPortAvailable() == true)
            {
				cliPortRead();
				cliPortPrint("  Gathering Data....\n\n");
				accelCalibrationState++;
			}

		    break;

		///////////////////////////////

		case 15:  // Collect Rear Edge Down Data
            if (accelCalibrationCount > 0)
            {
				arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrixT);

				arm_mat_init_f32(&b, 3, 1, (float *)sensors.accel100Hz);   // Body Coordinate Frame

		        arm_mat_init_f32(&x, 3, 1,          nonRotatedAccelData);  // Sensor Coordinate Frame

		        arm_mat_mult_f32(&a, &b, &x);

				noseUpSum += nonRotatedAccelData[XAXIS];

                zBiasSum  += nonRotatedAccelData[ZAXIS];
			}
			else
			{
				accelCalibrationState++;
			}

			accelCalibrationCount--;

		    break;

		///////////////////////////////

		case 16:  // Setup Front Edge Down Case
            cliPortPrint("Place accelerometer front edge down\n");
            cliPortPrint("  Send a character when ready to proceed\n\n");

            accelCalibrationCount = 500;

            accelCalibrationState++;

		    break;

		///////////////////////////////

		case 17:  // Wait for Front Edge Down Setup
            if (cliPortAvailable() == true)
            {
				cliPortRead();
				cliPortPrint("  Gathering Data....\n\n");
				accelCalibrationState++;
			}

		    break;

		///////////////////////////////

		case 18:  // Collect Front Edge Down Data
            if (accelCalibrationCount > 0)
            {
				arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrixT);

				arm_mat_init_f32(&b, 3, 1, (float *)sensors.accel100Hz);   // Body Coordinate Frame

		        arm_mat_init_f32(&x, 3, 1,          nonRotatedAccelData);  // Sensor Coordinate Frame

		        arm_mat_mult_f32(&a, &b, &x);

				noseDownSum += nonRotatedAccelData[XAXIS];

                zBiasSum    += nonRotatedAccelData[ZAXIS];
			}
			else
			{
				accelCalibrationState++;
			}

			accelCalibrationCount--;

		    break;

		///////////////////////////////

		case 19:  // Perform Computations
		    rightSideUpSum   /= 500.0f;
		    upSideDownSum    /= 500.0f;

		    leftWingDownSum  /= 500.0f;
		    rightWingDownSum /= 500.0f;

		    noseUpSum        /= 500.0f;
		    noseDownSum      /= 500.0f;

		    eepromConfig.accelBias[ZAXIS + eepromConfig.useMXR9150]        = zBiasSum / 2000.0f;
			eepromConfig.accelScaleFactor[ZAXIS + eepromConfig.useMXR9150] = (2.0f * 9.8065f) / fabsf(rightSideUpSum - upSideDownSum);

			eepromConfig.accelBias[YAXIS + eepromConfig.useMXR9150]        = yBiasSum / 1000.0f;
			eepromConfig.accelScaleFactor[YAXIS + eepromConfig.useMXR9150] = (2.0f * 9.8065f) / fabsf(leftWingDownSum - rightWingDownSum);

			eepromConfig.accelBias[XAXIS + eepromConfig.useMXR9150]        = xBiasSum / 1000.0f;
            eepromConfig.accelScaleFactor[XAXIS + eepromConfig.useMXR9150] = (2.0f * 9.8065f) / fabsf(noseUpSum - noseDownSum);

            cliPortPrintF("Accel Bias:         %9.3f, %9.3f, %9.3f\n", eepromConfig.accelBias[XAXIS + eepromConfig.useMXR9150],
			                                                		   eepromConfig.accelBias[YAXIS + eepromConfig.useMXR9150],
			                                                		   eepromConfig.accelBias[ZAXIS + eepromConfig.useMXR9150]);
			cliPortPrintF("Accel Scale Factor: %9.7f, %9.7f, %9.7f\n", eepromConfig.accelScaleFactor[XAXIS + eepromConfig.useMXR9150],
							                                           eepromConfig.accelScaleFactor[YAXIS + eepromConfig.useMXR9150],
			                                                		   eepromConfig.accelScaleFactor[ZAXIS + eepromConfig.useMXR9150]);
            cliPortPrint("\nAccel Calibration Complete....\n\n");

            accelCalibrating = false;

            break;

        ///////////////////////////////
	}

}
