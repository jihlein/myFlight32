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

#if defined(STM32F40XX)
    __attribute__((__section__(".eeprom"), used)) const int8_t eepromArray[16384];
#endif

float          t1, t2, t3, t4;

sensors_t      sensors;

homeData_t     homeData;

uint16_t       timerValue;

void           (*openLogPortPrintF)(const char * fmt, ...);

///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    ///////////////////////////////////////////////////////////////////////////

    uint32_t currentTime;

	arm_matrix_instance_f32 a;
	arm_matrix_instance_f32 b;
	arm_matrix_instance_f32 x;

    systemReady = false;

    systemInit();

    systemReady = true;

    evrPush(EVR_StartingMain, 0);

    while (1)
    {
        #if defined(STM32F40XX)
            checkUsbActive(false);
        #endif

        evrCheck();

        ///////////////////////////////

        if (frame_10Hz)
        {
            frame_10Hz = false;

            currentTime      = micros();
            deltaTime10Hz    = currentTime - previous10HzTime;
            previous10HzTime = currentTime;

            if (newMagData == true)
            {
			    nonRotatedMagData[XAXIS] = (rawMag[XAXIS].value * magScaleFactor[XAXIS]) - eepromConfig.magBias[XAXIS + eepromConfig.externalHMC5883];
			    nonRotatedMagData[YAXIS] = (rawMag[YAXIS].value * magScaleFactor[YAXIS]) - eepromConfig.magBias[YAXIS + eepromConfig.externalHMC5883];
			    nonRotatedMagData[ZAXIS] = (rawMag[ZAXIS].value * magScaleFactor[ZAXIS]) - eepromConfig.magBias[ZAXIS + eepromConfig.externalHMC5883];

			    arm_mat_init_f32(&a, 3, 3, (float *)hmcOrientationMatrix);

			    arm_mat_init_f32(&b, 3, 1, (float *)nonRotatedMagData);  // Sensor Coordinate Frame

			    arm_mat_init_f32(&x, 3, 1,          sensors.mag10Hz);    // Body Coordinate Frame

			    arm_mat_mult_f32(&a, &b, &x);

				newMagData = false;

				///////////////////////////

	            if (magCalibrating == true)
	                magCalibration();

	            ///////////////////////////

	            tasks500Hz_U.mag[XAXIS]    = sensors.mag10Hz[XAXIS];
				tasks500Hz_U.mag[YAXIS]    = sensors.mag10Hz[YAXIS];
				tasks500Hz_U.mag[ZAXIS]    = sensors.mag10Hz[ZAXIS];
			    tasks500Hz_U.magDataUpdate = true;

                if (magValid == false)
                	magValidCount++;

                if ((magValidCount >= 20) && (magValid == false))
                	magValid = true;
            }

            decodeUbloxMsg();

            batMonTick();

            if (eepromConfig.mavlinkEnabled == true)
            {
				mavlinkSendAttitude();
				mavlinkSendVfrHud();
			}

			if (gatheringCalibrationData == true)
			{
        		cliPortPrintF("%11.6f,%11.6f,%11.6f,%11.6f,%11.6f,%11.6f,%11.6f\n", t1,
        				                                                            nonRotatedGyroData[ROLL ],
        		                                                                    nonRotatedGyroData[PITCH],
        		                                                                    nonRotatedGyroData[YAW  ],
        		                                                                    nonRotatedAccelData[XAXIS],
        		            			                                            nonRotatedAccelData[YAXIS],
        		            			                                            nonRotatedAccelData[ZAXIS]);
			}

			cliCom();

			executionTime10Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_50Hz)
        {
            frame_50Hz = false;

            currentTime      = micros();
            deltaTime50Hz    = currentTime - previous50HzTime;
            previous50HzTime = currentTime;

            processFlightCommands();

            if (newTemperatureReading && newPressureReading)
            {
                d1Value = d1.value;
                d2Value = d2.value;

                calculateTemperature();
                calculatePressureAltitude();

                newTemperatureReading = false;
                newPressureReading    = false;

                if (pressureValid == false)
                	pressureValidCount++;

                if ((pressureValidCount >= 20) && (pressureValid == false))
                	pressureValid = true;
            }

            sensors.pressureAlt50Hz = firstOrderFilter(sensors.pressureAlt50Hz, &firstOrderFilters[PRESSURE_ALT_LOWPASS]);

            executionTime50Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_100Hz)
        {
            frame_100Hz = false;

            currentTime       = micros();
            deltaTime100Hz    = currentTime - previous100HzTime;
            previous100HzTime = currentTime;

			#if defined(STM32F30X)
			    TIM_Cmd(TIM7, DISABLE);
			    timerValue = TIM_GetCounter(TIM7);
			    TIM_SetCounter(TIM7, 0);
			    TIM_Cmd(TIM7, ENABLE);
			#endif

			#if defined(STM32F40XX)
                TIM_Cmd(TIM11, DISABLE);
                timerValue = TIM_GetCounter(TIM11);
                TIM_SetCounter(TIM11, 0);
                TIM_Cmd(TIM11, ENABLE);
            #endif

            dt100Hz = (float)timerValue * 0.0000005f;  // For integrations in 100 Hz loop

            ///////////////////////////

            t1 = (float)rawMPU6000Temperature.value * 0.00294118f + 35.0f;  // 0.00294118 = 1 / 340

            t1 = constrain(t1, eepromConfig.mpuTempMin, eepromConfig.mpuTempMax);

            t2 = SQR(t1);
            t3 = t2 * t1;
            t4 = t3 * t1;

            accelBias[XAXIS] = eepromConfig.accelBiasPolynomial[XAXIS * 5 + 0] * t4 +
            		           eepromConfig.accelBiasPolynomial[XAXIS * 5 + 1] * t3 +
                               eepromConfig.accelBiasPolynomial[XAXIS * 5 + 2] * t2 +
                               eepromConfig.accelBiasPolynomial[XAXIS * 5 + 3] * t1 +
                               eepromConfig.accelBiasPolynomial[XAXIS * 5 + 4];

            accelBias[YAXIS] = eepromConfig.accelBiasPolynomial[YAXIS * 5 + 0] * t4 +
                               eepromConfig.accelBiasPolynomial[YAXIS * 5 + 1] * t3 +
                               eepromConfig.accelBiasPolynomial[YAXIS * 5 + 2] * t2 +
                               eepromConfig.accelBiasPolynomial[YAXIS * 5 + 3] * t1 +
                               eepromConfig.accelBiasPolynomial[YAXIS * 5 + 4];

            accelBias[ZAXIS] = eepromConfig.accelBiasPolynomial[ZAXIS * 5 + 0] * t4 +
                               eepromConfig.accelBiasPolynomial[ZAXIS * 5 + 1] * t3 +
                               eepromConfig.accelBiasPolynomial[ZAXIS * 5 + 2] * t2 +
                               eepromConfig.accelBiasPolynomial[ZAXIS * 5 + 3] * t1 +
                               eepromConfig.accelBiasPolynomial[ZAXIS * 5 + 4];

            accelScaleFactor[XAXIS] = eepromConfig.accelScaleFactorPolynomial[XAXIS * 5 + 0] * t4 +
                                      eepromConfig.accelScaleFactorPolynomial[XAXIS * 5 + 1] * t3 +
                                      eepromConfig.accelScaleFactorPolynomial[XAXIS * 5 + 2] * t2 +
                                      eepromConfig.accelScaleFactorPolynomial[XAXIS * 5 + 3] * t1 +
                                      eepromConfig.accelScaleFactorPolynomial[XAXIS * 5 + 4];

            accelScaleFactor[YAXIS] = eepromConfig.accelScaleFactorPolynomial[YAXIS * 5 + 0] * t4 +
                                      eepromConfig.accelScaleFactorPolynomial[YAXIS * 5 + 1] * t3 +
                                      eepromConfig.accelScaleFactorPolynomial[YAXIS * 5 + 2] * t2 +
                                      eepromConfig.accelScaleFactorPolynomial[YAXIS * 5 + 3] * t1 +
                                      eepromConfig.accelScaleFactorPolynomial[YAXIS * 5 + 4];

            accelScaleFactor[ZAXIS] = eepromConfig.accelScaleFactorPolynomial[ZAXIS * 5 + 0] * t4 +
                                      eepromConfig.accelScaleFactorPolynomial[ZAXIS * 5 + 1] * t3 +
                                      eepromConfig.accelScaleFactorPolynomial[ZAXIS * 5 + 2] * t2 +
                                      eepromConfig.accelScaleFactorPolynomial[ZAXIS * 5 + 3] * t1 +
                                      eepromConfig.accelScaleFactorPolynomial[ZAXIS * 5 + 4];

            ///////////////////////////

            nonRotatedAccelData[XAXIS] = ((float)accelSummedSamples100Hz[XAXIS] * 0.1f - accelBias[XAXIS]) * accelScaleFactor[XAXIS];
            nonRotatedAccelData[YAXIS] = ((float)accelSummedSamples100Hz[YAXIS] * 0.1f - accelBias[YAXIS]) * accelScaleFactor[YAXIS];
            nonRotatedAccelData[ZAXIS] = ((float)accelSummedSamples100Hz[ZAXIS] * 0.1f - accelBias[ZAXIS]) * accelScaleFactor[ZAXIS];

		    arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrix);

		    arm_mat_init_f32(&b, 3, 1, (float *)nonRotatedAccelData);  // Sensor Coordinate Frame

		    arm_mat_init_f32(&x, 3, 1,          sensors.accel100Hz);   // Body Coordinate Frame

		    arm_mat_mult_f32(&a, &b, &x);

			///////////////////////////

            if (accelOneGCalcCount > 0)
			    accelOneGCalc();

			///////////////////////////

			if (attCalibrationCount > 0)
			    attCalibration();

			///////////////////////////

            tasks500Hz_U.accel[XAXIS]      = sensors.accel100Hz[XAXIS];
            tasks500Hz_U.accel[YAXIS]      = sensors.accel100Hz[YAXIS];
            tasks500Hz_U.accel[ZAXIS]      = sensors.accel100Hz[ZAXIS];
            tasks500Hz_U.accelDataUpdate   = true;

            ///////////////////////////

            tasks100Hz_U.q[0]              = tasks500Hz_Y.q[0];
            tasks100Hz_U.q[1]              = tasks500Hz_Y.q[1];
            tasks100Hz_U.q[2]              = tasks500Hz_Y.q[2];
            tasks100Hz_U.q[3]              = tasks500Hz_Y.q[3];
            tasks100Hz_U.accel[XAXIS]      = sensors.accel100Hz[XAXIS];
            tasks100Hz_U.accel[YAXIS]      = sensors.accel100Hz[YAXIS];
            tasks100Hz_U.accel[ZAXIS]      = sensors.accel100Hz[ZAXIS];
            tasks100Hz_U.accelOneG         = accelOneG;
            tasks100Hz_U.gpsLatitude       = sensors.gps.latitude;
          //tasks100Hz_U.homeLatitude      = set in 1 Hz Task
            tasks100Hz_U.gpsNdot           = sensors.gps.velN;
            tasks100Hz_U.gpsLongitude      = sensors.gps.longitude;
          //tasks100Hz_U.homeLongitude     = set in 1 Hz Task
            tasks100Hz_U.gpsEdot           = sensors.gps.velE;
            tasks100Hz_U.pressureAlt       = sensors.pressureAlt50Hz;
            tasks100Hz_U.systemOperational = systemOperational;
          //tasks100Hz_U.magVar            = set in 1 Hz Task

            tasks100Hz_step();

            ///////////////////////////

            if (armed == true)
            {
				if (eepromConfig.activeTelemetry == 1)
                {
            	    // Roll Loop
					openLogPortPrintF("1,%1d,%9.4f,%9.4f,%9.4f,%9.4f,%9.4f\n", flightMode,
					        			                                       tasks500Hz_U.rateCmds[ROLL],
					        			                                       tasks500Hz_U.gyro[ROLL],
					        			                                       tasks500Hz_U.attCmds[ROLL],
                                                                               tasks500Hz_Y.attitudes[ROLL],
		                                                                       tasks500Hz_Y.axisCmds[ROLL]);
                }

                if (eepromConfig.activeTelemetry == 2)
                {
            	    // Pitch Loop
					openLogPortPrintF("2,%1d,%9.4f,%9.4f,%9.4f,%9.4f,%9.4f\n", flightMode,
					        			                                       tasks500Hz_U.rateCmds[PITCH],
					        			                                       tasks500Hz_U.gyro[PITCH],
					        			                                       tasks500Hz_U.attCmds[PITCH],
                                                                               tasks500Hz_Y.attitudes[PITCH],
	                                                                           tasks500Hz_Y.axisCmds[PITCH]);
                }

                if (eepromConfig.activeTelemetry == 4)
                {
            	    // Sensors
					openLogPortPrintF("3,%8.4f,%8.4f,%8.4f,%1d,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%1d,%8.4f,%8.4f,%8.4f\n", tasks500Hz_U.accel[XAXIS],
					        			                                                                                     tasks500Hz_U.accel[YAXIS],
					        			                                                                                     tasks500Hz_U.accel[ZAXIS],
					        			                                                                                     tasks500Hz_U.accelDataUpdate,
					        			                                                                                     tasks500Hz_U.gyro[ROLL],
                                                                                                                             tasks500Hz_U.gyro[PITCH],
	                                                                                                                         tasks500Hz_U.gyro[YAW],
	                                                                                                                         tasks500Hz_U.mag[XAXIS],
	                                                                                                                         tasks500Hz_U.mag[YAXIS],
	                                                                                                                         tasks500Hz_U.mag[ZAXIS],
	                                                                                                                         tasks500Hz_U.magDataUpdate,
	                                                                                                                         tasks500Hz_Y.attitudes[ROLL],
	                                                                                                                         tasks500Hz_Y.attitudes[PITCH],
	                                                                                                                         tasks500Hz_Y.attitudes[YAW]);

                }

                if (eepromConfig.activeTelemetry == 8)
                {
                    openLogPortPrintF("4,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%12ld,%12ld,%12ld,%12ld,%8.4f,%8.4f,%8.4f,%8.4f\n", sensors.accel100Hz[XAXIS],
                    		                                                                                                     sensors.accel100Hz[YAXIS],
                                                                                                                                 sensors.accel100Hz[ZAXIS],
                                                                                                                                 tasks100Hz_Y.earthAxisAccels[XAXIS],
                                                                                                                                 tasks100Hz_Y.earthAxisAccels[YAXIS],
                                                                                                                                 tasks100Hz_Y.earthAxisAccels[ZAXIS],
                                                                                                                                 sensors.gps.latitude,
                                                                                                                                 sensors.gps.longitude,
                                                                                                                                 sensors.gps.velN,
                                                                                                                                 sensors.gps.velE,
                                                                                                                                 tasks100Hz_Y.velocityEstimates[XAXIS],
											                                                                                     tasks100Hz_Y.velocityEstimates[YAXIS],
											                                                                                     tasks100Hz_Y.positionEstimates[XAXIS],
											                                                                                     tasks100Hz_Y.positionEstimates[YAXIS]);
                }

                if (eepromConfig.activeTelemetry == 16)
                {
               	    // Vertical Variables
            	    openLogPortPrintF("5,%9.4f, %9.4f, %9.4f, %4ld, %1d, %9.4f\n", tasks500Hz_U.velCmds[ZAXIS],
            	    		                                                       tasks500Hz_U.velocities[ZAXIS],
            	    		                                                       tasks500Hz_U.positions[ZAXIS],
            	    		                                                       ms5611Temperature,
            	    		                                                       verticalModeState,
            	    		                                                       tasks500Hz_Y.axisCmds[VERTICAL]);
                }
		    }

            executionTime100Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_500Hz)
        {
            frame_500Hz = false;

            currentTime       = micros();
            deltaTime500Hz    = currentTime - previous500HzTime;
            previous500HzTime = currentTime;

       	    #if defined(STM32F30X)
       	        TIM_Cmd(TIM6, DISABLE);
       	 	    timerValue = TIM_GetCounter(TIM6);
       	 	    TIM_SetCounter(TIM6, 0);
       	 	    TIM_Cmd(TIM6, ENABLE);
       	 	#endif

       	 	#if defined(STM32F40XX)
                TIM_Cmd(TIM10, DISABLE);
                timerValue = TIM_GetCounter(TIM10);
                TIM_SetCounter(TIM10, 0);
                TIM_Cmd(TIM10, ENABLE);
            #endif

            dt500Hz = (float)timerValue * 0.0000005f;  // For integrations in 500 Hz loop

            ///////////////////////////

            t1 = (float)rawMPU6000Temperature.value * 0.00294118f + 35.0f;  // 0.00294118 = 1 / 340

            t1 = constrain(t1, eepromConfig.mpuTempMin, eepromConfig.mpuTempMax);

            t2 = SQR(t1);
            t3 = t2 * t1;
            t4 = t3 * t1;

            gyroBias[ROLL ] = eepromConfig.gyroBiasPolynomial[ROLL  * 5 + 0] * t4 +
                              eepromConfig.gyroBiasPolynomial[ROLL  * 5 + 1] * t3 +
                              eepromConfig.gyroBiasPolynomial[ROLL  * 5 + 2] * t2 +
                              eepromConfig.gyroBiasPolynomial[ROLL  * 5 + 3] * t1 +
                              eepromConfig.gyroBiasPolynomial[ROLL  * 5 + 4];

            gyroBias[PITCH] = eepromConfig.gyroBiasPolynomial[PITCH * 5 + 0] * t4 +
                              eepromConfig.gyroBiasPolynomial[PITCH * 5 + 1] * t3 +
                              eepromConfig.gyroBiasPolynomial[PITCH * 5 + 2] * t2 +
                              eepromConfig.gyroBiasPolynomial[PITCH * 5 + 3] * t1 +
                              eepromConfig.gyroBiasPolynomial[PITCH * 5 + 4];

            gyroBias[YAW  ] = eepromConfig.gyroBiasPolynomial[YAW   * 5 + 0] * t4 +
                              eepromConfig.gyroBiasPolynomial[YAW   * 5 + 1] * t3 +
                              eepromConfig.gyroBiasPolynomial[YAW   * 5 + 2] * t2 +
                              eepromConfig.gyroBiasPolynomial[YAW   * 5 + 3] * t1 +
                              eepromConfig.gyroBiasPolynomial[YAW   * 5 + 4];

		    nonRotatedGyroData[ROLL ] = ((float)gyroSummedSamples500Hz[ROLL ] * 0.5f - gyroBias[ROLL ]) * gyroScaleFactor;
            nonRotatedGyroData[PITCH] = ((float)gyroSummedSamples500Hz[PITCH] * 0.5f - gyroBias[PITCH]) * gyroScaleFactor;
            nonRotatedGyroData[YAW  ] = ((float)gyroSummedSamples500Hz[YAW  ] * 0.5f - gyroBias[YAW  ]) * gyroScaleFactor;

		    arm_mat_init_f32(&a, 3, 3, (float *)mpuOrientationMatrix);

		    arm_mat_init_f32(&b, 3, 1, (float *)nonRotatedGyroData);  // Sensor Coordinate Frame

		    arm_mat_init_f32(&x, 3, 1,          sensors.gyro500Hz);   // Body Coordinate Frame

		    arm_mat_mult_f32(&a, &b, &x);

		    ///////////////////////////

            if (gyroRunTimeCalCount > 0)
			    gyroRunTimeCal();

            ///////////////////////////

            tasks500Hz_U.gyro[ROLL ]       = sensors.gyro500Hz[ROLL ];
            tasks500Hz_U.gyro[PITCH]       = sensors.gyro500Hz[PITCH];
            tasks500Hz_U.gyro[YAW  ]       = sensors.gyro500Hz[YAW];
          //tasks500Hz_U.accel[3]          = set in 100 Hz Task
          //tasks500Hz_U.mag[3]            = set in 10 Hz Task
            tasks500Hz_U.dt                = dt500Hz;
            tasks500Hz_U.gyroValid         = gyroValid;
            tasks500Hz_U.accelValid        = accelValid;
            tasks500Hz_U.magValid          = magValid;
            tasks500Hz_U.accelOneG         = accelOneG;
          //tasks500Hz_U.accelDataUpdate   = set in 100 Hz Task
          //tasks500Hz_U.magDataUpdate     = set in 10 Hz Task
          //tasks500Hz_U.magVar            = set in 1 Hz Task
            tasks500Hz_U.velocities[XAXIS] = tasks100Hz_Y.bodyVelocityEstimates[XAXIS];
            tasks500Hz_U.velocities[YAXIS] = tasks100Hz_Y.bodyVelocityEstimates[YAXIS];
            tasks500Hz_U.velocities[ZAXIS] = tasks100Hz_Y.bodyVelocityEstimates[ZAXIS];
            tasks500Hz_U.positions[XAXIS]  = tasks100Hz_Y.bodyPositionEstimates[XAXIS];
            tasks500Hz_U.positions[YAXIS]  = tasks100Hz_Y.bodyPositionEstimates[YAXIS];
            tasks500Hz_U.positions[ZAXIS]  = tasks100Hz_Y.bodyPositionEstimates[ZAXIS];
          //tasks500Hz_U.rateModes[3]      = set in flightCommand.c
          //tasks500Hz_U.rateCmds[3]       = set in flightCommand.c
          //tasks500Hz_U.attModes[3]       = set in flightCommand.c
          //tasks500Hz_U.attCmds[3]        = set in flightCommand.c
          //tasks500Hz_U.velModes[3]       = set in flightCommand.c
          //tasks500Hz_U.velCmds[3]        = set in flightCommand.c
          //tasks500Hz_U.posModes[3]       = set in flightCommand.c
          //tasks500Hz_U.posCmds[3]        = set in flightCommand.c
          //tasks500Hz_U.resetPIDs         = set in flightCommand.c

            tasks500Hz_step();

            tasks500Hz_U.accelDataUpdate   = false;
            tasks500Hz_U.magDataUpdate     = false;

            sensors.attitude500Hz[ROLL ]   = tasks500Hz_Y.attitudes[ROLL ];
            sensors.attitude500Hz[PITCH]   = tasks500Hz_Y.attitudes[PITCH];
            sensors.attitude500Hz[YAW  ]   = tasks500Hz_Y.attitudes[YAW  ];

            ///////////////////////////

            limitMotorCmds();

            writeMotors();

            #if defined(STM32F30X)
                checkUsbActive(false);
            #endif

            executionTime500Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_5Hz)
        {
            frame_5Hz = false;

            currentTime     = micros();
            deltaTime5Hz    = currentTime - previous5HzTime;
            previous5HzTime = currentTime;

            setGpsValid();

            if (eepromConfig.mavlinkEnabled == true)
            {
				mavlinkSendGpsRaw();
			}

			if (batMonVeryLowWarning > 0)
			{
				ONBOARD_LED1_TOGGLE;
				batMonVeryLowWarning--;
			}

            if (systemOperational == true)
                ONBOARD_LED1_TOGGLE;

			executionTime5Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_1Hz)
        {
            frame_1Hz = false;

            currentTime     = micros();
            deltaTime1Hz    = currentTime - previous1HzTime;
            previous1HzTime = currentTime;

            if (systemOperational == true)
                ONBOARD_LED2_TOGGLE;

            if (accelValid           &&
               tasks500Hz_Y.attValid &&
               gyroValid             &&
               magValid              &&
               pressureValid         &&
               !systemOperational    &&
               (sensors.gps.valid || !eepromConfig.useGPS))
            {
                if (sensors.gps.valid && eepromConfig.useGPS)
                {
					computeGeoMagElements();

                    homeData.latitude   = sensors.gps.latitude;
	                homeData.longitude  = sensors.gps.longitude;
	                homeData.altitude   = tasks100Hz_Y.bodyPositionEstimates[ZAXIS];
	                homeData.geoMagX    = geoMagneticElements.X;
	                homeData.geoMagY    = geoMagneticElements.Y;
	                homeData.geoMagZ    = geoMagneticElements.Z;
	                homeData.geoMagH    = geoMagneticElements.H;
	                homeData.geoMagF    = geoMagneticElements.F;
	        	    homeData.geoMagIncl = geoMagneticElements.Incl;
	        	    homeData.geoMagDecl = geoMagneticElements.Decl;
	        	    homeData.hDop       = sensors.gps.hDop;                  // TEST

	                tasks100Hz_U.homeLatitude  = homeData.latitude;
	                tasks100Hz_U.homeLongitude = homeData.longitude;
	                tasks100Hz_U.magVar        = homeData.geoMagDecl * D2R;

	                tasks500Hz_U.magVar        = homeData.geoMagDecl * D2R;

	                openLogPortPrintF("Home %12ld,%12ld\n", homeData.latitude, homeData.longitude);
				}
				else
				{
					homeData.geoMagDecl = 0.0f;

					tasks100Hz_U.magVar = 0.0f;

	                tasks500Hz_U.magVar = 0.0f;
				}

	            homeData.magHeading = standardRadianFormat(tasks500Hz_Y.attitudes[YAW] + homeData.geoMagDecl * D2R);

	            pwmEscInit();
	            pwmServoInit();

	            systemOperational = true;
            }

			if (batMonLowWarning > 0)
			{
				EXTERNAL_LED1_TOGGLE;
				batMonLowWarning--;
			}

            if (eepromConfig.mavlinkEnabled == true)
            {
				mavlinkSendHeartbeat();
				mavlinkSendSysStatus();
			}

            executionTime1Hz = micros() - currentTime;
        }

        ////////////////////////////////
    }

    ///////////////////////////////////////////////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
