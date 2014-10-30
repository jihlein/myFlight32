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

uint32_t (*cliPortAvailable)(void);

void     (*cliPortClearBuffer)(void);

uint8_t  (*cliPortRead)(void);

void     (*cliPortPrint)(char *str);

void     (*cliPortPrintF)(const char * fmt, ...);

///////////////////////////////////////

uint8_t cliBusy = false;

static volatile uint8_t cliQuery        = 'x';
static volatile uint8_t validCliCommand = false;

uint8_t gpsDataType = 0;

///////////////////////////////////////////////////////////////////////////////
// Read Character String from CLI
///////////////////////////////////////////////////////////////////////////////

char *readStringCLI(char *data, uint8_t length)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;

    do
    {
        if (cliPortAvailable() == false)
        {
            delay(10);
            timeout++;
        }
        else
        {
            data[index] = cliPortRead();
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < length));

    data[index] = '\0';

    return data;
}

///////////////////////////////////////////////////////////////////////////////
// Read Float from CLI
///////////////////////////////////////////////////////////////////////////////

float readFloatCLI(void)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;
    char    data[13] = "";

    do
    {
        if (cliPortAvailable() == false)
        {
            delay(10);
            timeout++;
        }
        else
        {
            data[index] = cliPortRead();
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));

    data[index] = '\0';

    return stringToFloat(data);
}

///////////////////////////////////////////////////////////////////////////////
// CLI Communication
///////////////////////////////////////////////////////////////////////////////

void cliCom(void)
{
	uint8_t  index;
	char mvlkToggleString[5] = { 0, 0, 0, 0, 0 };

    if ((cliPortAvailable() && !validCliCommand))
    {
		cliQuery = cliPortRead();

        if (cliQuery == '#')                       // Check to see if we should toggle mavlink msg state
        {
	    	while (cliPortAvailable == false);

        	readStringCLI(mvlkToggleString, 5);

            if ((mvlkToggleString[0] == '#') &&
            	(mvlkToggleString[1] == '#') &&
                (mvlkToggleString[2] == '#') &&
                (mvlkToggleString[3] == '#'))
	    	{
	    	    if (eepromConfig.mavlinkEnabled == false)
	    	    {
	    	 	    eepromConfig.mavlinkEnabled  = true;
	    		    eepromConfig.activeTelemetry = 0x0000;
	    		}
	    		else
	    		{
	    		    eepromConfig.mavlinkEnabled = false;
	    	    }

	    	    if (mvlkToggleString[4] == 'W')
	    	    {
	                cliPortPrint("\nWriting EEPROM Parameters....\n");
	                writeEEPROM();
	    	    }
	    	}
	    }
	}

	validCliCommand = false;

    if ((eepromConfig.mavlinkEnabled == false) && (cliQuery != '#'))
    {
        switch (cliQuery)
        {
            ///////////////////////////////

            case 'a': // Rate PIDs
                cliPortPrintF("\nRoll Rate PID:  %8.4f, %8.4f, %8.4f, %8.4f\n", eepromConfig.rateP[ROLL],
                    		                                                    eepromConfig.rateI[ROLL],
                    		                                                    eepromConfig.rateD[ROLL],
                    		                                                    eepromConfig.rateLimit[ROLL]);

                cliPortPrintF(  "Pitch Rate PID: %8.4f, %8.4f, %8.4f, %8.4f\n", eepromConfig.rateP[PITCH],
                    		                                                    eepromConfig.rateI[PITCH],
                    		                                                    eepromConfig.rateD[PITCH],
                    		                                                    eepromConfig.rateLimit[PITCH]);

                cliPortPrintF(  "Yaw Rate PID:   %8.4f, %8.4f, %8.4f, %8.4f\n", eepromConfig.rateP[YAW],
                    		                                                    eepromConfig.rateI[YAW],
                    		                                                    eepromConfig.rateD[YAW],
                    		                                                    eepromConfig.rateLimit[YAW]);
                cliQuery = 'x';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'b': // Attitude PIDs
                cliPortPrintF("\nRoll Attitude PID:  %8.4f, %8.4f, %8.4f, %8.4f\n", eepromConfig.attitudeP[ROLL],
                   		                                                            eepromConfig.attitudeI[ROLL],
                   		                                                            eepromConfig.attitudeD[ROLL],
                   		                                                            eepromConfig.attitudeLimit[ROLL]);

                cliPortPrintF(  "Pitch Attitude PID: %8.4f, %8.4f, %8.4f, %8.4f\n", eepromConfig.attitudeP[PITCH],
                   		                                                            eepromConfig.attitudeI[PITCH],
                   		                                                            eepromConfig.attitudeD[PITCH],
                   		                                                            eepromConfig.attitudeLimit[PITCH]);

                cliPortPrintF(  "Heading PID:        %8.4f, %8.4f, %8.4f, %8.4f\n", eepromConfig.attitudeP[YAW],
                   		                                                            eepromConfig.attitudeI[YAW],
                   		                                                            eepromConfig.attitudeD[YAW],
                   		                                                            eepromConfig.attitudeLimit[YAW]);
                cliQuery = 'x';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'c': // Velocity PIDs
                cliPortPrintF("\nnDot PID:  %8.4f, %8.4f, %8.4f, %8.4f\n", eepromConfig.velocityP[XAXIS],
                   		                                                   eepromConfig.velocityI[XAXIS],
                   		                                                   eepromConfig.velocityD[XAXIS],
                   		                                                   eepromConfig.velocityLimit[XAXIS]);

                cliPortPrintF(  "eDot PID:  %8.4f, %8.4f, %8.4f, %8.4f\n", eepromConfig.velocityP[YAXIS],
                   		                                                   eepromConfig.velocityI[YAXIS],
                   		                                                   eepromConfig.velocityD[YAXIS],
                   		                                                   eepromConfig.velocityLimit[YAXIS]);

                cliPortPrintF(  "hDot PID:  %8.4f, %8.4f, %8.4f, %8.4f\n", eepromConfig.velocityP[ZAXIS],
                   		                                                   eepromConfig.velocityI[ZAXIS],
                   		                                                   eepromConfig.velocityD[ZAXIS],
                   		                                                   eepromConfig.velocityLimit[ZAXIS]);
                cliQuery = 'x';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'd': // Position PIDs
                cliPortPrintF("\nN PID:  %8.4f, %8.4f, %8.4f, %8.4f\n", eepromConfig.positionP[XAXIS],
                   		                                                eepromConfig.positionI[XAXIS],
                   		                                                eepromConfig.positionD[XAXIS],
                   		                                                eepromConfig.positionLimit[XAXIS]);

                cliPortPrintF(  "E PID:  %8.4f, %8.4f, %8.4f, %8.4f\n", eepromConfig.positionP[YAXIS],
                   		                                                eepromConfig.positionI[YAXIS],
                   		                                                eepromConfig.positionD[YAXIS],
                   		                                                eepromConfig.positionLimit[YAXIS]);

                cliPortPrintF(  "h PID:  %8.4f, %8.4f, %8.4f, %8.4f\n", eepromConfig.positionP[ZAXIS],
                   		                                                eepromConfig.positionI[ZAXIS],
                   		                                                eepromConfig.positionD[ZAXIS],
                   		                                                eepromConfig.positionLimit[ZAXIS]);
                cliQuery = 'x';
                validCliCommand = false;
              	break;

            ///////////////////////////////

            case 'e': // Loop Delta Times
               	cliPortPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\n", deltaTime1000Hz,
                   		                                                    deltaTime500Hz,
                   		                                                    deltaTime100Hz,
                   		                                                    deltaTime50Hz,
                   		                                                    deltaTime10Hz,
                   		                                                    deltaTime5Hz,
                   		                                                    deltaTime1Hz);
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'f': // Loop Execution Times
               	cliPortPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\n", executionTime1000Hz,
               	        			                                        executionTime500Hz,
               	        			                                        executionTime100Hz,
               	        			                                        executionTime50Hz,
               	        			                                        executionTime10Hz,
               	        			                                        executionTime5Hz,
               	        			                                        executionTime1Hz);
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'g': // 100 Hz Accels
            	if (accelValid)
            		cliPortPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", sensors.accel100Hz[XAXIS],
            		            			                      sensors.accel100Hz[YAXIS],
            		            			                      sensors.accel100Hz[ZAXIS],
            		            			                      mpu6000Temperature);
            	else
            		cliPortPrint("Accel Invalid....\n");

            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'h': // 100 hz Earth Axis Accels
            	if (accelValid)
            		cliPortPrintF("%9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f\n", tasks100Hz_Y.earthAxisAccels[XAXIS],
            		    	                                                    tasks100Hz_Y.earthAxisAccels[YAXIS],
            			                                                        tasks100Hz_Y.earthAxisAccels[ZAXIS],
            			                                                        tasks100Hz_Y.earthAxisAccelsNF[XAXIS],
            			                                                        tasks100Hz_Y.earthAxisAccelsNF[YAXIS],
            			                                                        tasks100Hz_Y.earthAxisAccelsNF[ZAXIS]);
            	else
            		cliPortPrint("Accel Invalid....\n");

            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'i': // 500 hz Gyros
            	if (gyroValid)
            	    cliPortPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", sensors.gyro500Hz[ROLL ] * R2D,
            		    	                                      sensors.gyro500Hz[PITCH] * R2D,
            			    		                              sensors.gyro500Hz[YAW  ] * R2D,
            				    	                              mpu6000Temperature);
            	else
            		cliPortPrint("Gyro Invalid....\n");

            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'j': // 10 Hz Mag Data
            	if (magValid)
            		cliPortPrintF("%9.4f, %9.4f, %9.4f\n", sensors.mag10Hz[XAXIS],
            			                                   sensors.mag10Hz[YAXIS],
            			                                   sensors.mag10Hz[ZAXIS]);
            	else
            		cliPortPrint("Mag Invalid....\n");

            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'k': // Vertical Axis Variables
            	if (pressureValid)
            		cliPortPrintF("%9.4f, %9.4f, %9.4f, %9.4f, %4ld, %9.4f\n", tasks100Hz_Y.earthAxisAccels[ZAXIS],
            			                                                       sensors.pressureAlt50Hz,
            					                                               tasks100Hz_Y.bodyVelocityEstimates[ZAXIS],
            					                                               tasks100Hz_Y.bodyPositionEstimates[ZAXIS],
            					                                               ms5611Temperature,
            					                                               aglRead());
            	else
            		cliPortPrint("Pressure Alt Invalid....\n");

            	validCliCommand = false;
        	    break;

            ///////////////////////////////

            case 'l': // Attitudes
            	if (tasks500Hz_Y.attValid)
            		cliPortPrintF("%9.4f, %9.4f, %9.4f\n", sensors.attitude500Hz[ROLL ] * R2D,
            			                                   sensors.attitude500Hz[PITCH] * R2D,
            			                                   sensors.attitude500Hz[YAW  ] * R2D);
            	else
            		cliPortPrint("Attitude Invalid....\n");

            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'm': // Rate PIDs
            	cliPortPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", tasks500Hz_Y.axisCmds[ROLL    ],
            			                                      tasks500Hz_Y.axisCmds[PITCH   ],
            			                                      tasks500Hz_Y.axisCmds[YAW     ],
            			                                      tasks500Hz_Y.axisCmds[VERTICAL]);
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'n': // GPS Data
               	switch (gpsDataType)
               	{
               	    ///////////////////////

               	    case 0:
               	        cliPortPrintF("%12ld, %12ld, %12ld, %12ld, %12ld, %12ld, %4d, %4d\n", sensors.gps.latitude,
               	        		                                                              sensors.gps.longitude,
               			                                                                      sensors.gps.hMSL,
               			                                                                      sensors.gps.velN,
               			                                                                      sensors.gps.velE,
               			                                                                      sensors.gps.velD,
               			                                                                      sensors.gps.valid,
               			                                                                      sensors.gps.numSats);
               	        break;

               	    ///////////////////////

               	    case 1:
               	    	cliPortPrintF("%3d: ", sensors.gps.numCh);

               	    	for (index = 0; index < sensors.gps.numCh; index++)
               	    	    cliPortPrintF("%3d  ", sensors.gps.chn[index]);

               	    	cliPortPrint("\n");

               	    	break;

               	    ///////////////////////

               	    case 2:
               	    	cliPortPrintF("%3d: ", sensors.gps.numCh);

               	    	for (index = 0; index < sensors.gps.numCh; index++)
               	    		cliPortPrintF("%3d  ", sensors.gps.svid[index]);

               	    	cliPortPrint("\n");

               	    	break;

               	    ///////////////////////

               	    case 3:
               	    	cliPortPrintF("%3d: ", sensors.gps.numCh);

               	    	for (index = 0; index < sensors.gps.numCh; index++)
               	    		cliPortPrintF("%3d  ", sensors.gps.cno[index]);

               	    	cliPortPrint("\n");

               	    	break;

               	    ///////////////////////

               	    case 4:
               	    	cliPortPrintF("%12ld    ",  sensors.gps.iTOW);
               	    	cliPortPrintF("%2d    ",   (sensors.gps.fix == FIX_3D));
               	    	cliPortPrintF("%2d    ",   (sensors.gps.statusFlags & GPS_FIX_OK));
               	    	cliPortPrintF("%5.2f\n",   (float)sensors.gps.hDop / 100.0f);

               	    	break;
               	}

               	validCliCommand = false;
                break;

            ///////////////////////////////

            case 'o':
                cliPortPrintF("%9.4f\n", batteryVoltage);

                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'p': // Primary Spektrum Raw Data
            	cliPortPrintF("%04X, %04X, %04X, %04X, %04X, %04X, %04X, %04X, %04X, %04X\n", primarySpektrumState.lostFrameCnt,
            			                                                                      primarySpektrumState.rcAvailable,
            			                                                                      primarySpektrumState.values[0],
            			                                                                      primarySpektrumState.values[1],
             			                                                                      primarySpektrumState.values[2],
            			                                                                      primarySpektrumState.values[3],
            			                                                                      primarySpektrumState.values[4],
            			                                                                      primarySpektrumState.values[5],
            			                                                                      primarySpektrumState.values[6],
            			                                                                      primarySpektrumState.values[7]);
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            #if defined(STM32F40XX)
                case 'q': // Slave Spektrum Raw Data
                	cliPortPrintF("%04X, %04X, %04X, %04X, %04X, %04X, %04X, %04X, %04X, %04X\n", slaveSpektrumState.lostFrameCnt,
                	            		                                                          slaveSpektrumState.rcAvailable,
                			                                                                      slaveSpektrumState.values[0],
                			                                                                      slaveSpektrumState.values[1],
                			                                                                      slaveSpektrumState.values[2],
                			                                                                      slaveSpektrumState.values[3],
                			                                                                      slaveSpektrumState.values[4],
                 			                                                                      slaveSpektrumState.values[5],
                			                                                                      slaveSpektrumState.values[6],
                			                                                                      slaveSpektrumState.values[7]);
                	validCliCommand = false;
                	break;
            #endif

            ///////////////////////////////

            case 'r':
            	if (flightMode == RATE)
            		cliPortPrint("Flight Mode = RATE      ");
            	else if (flightMode == ATTITUDE)
            		cliPortPrint("Flight Mode = ATTITUDE  ");
            	else if (flightMode == GPS)
            		cliPortPrint("Flight Mode = GPS       ");

            	if (headingHoldEngaged == true)
        	        cliPortPrint("Heading Hold = ENGAGED     ");
        	    else
        	        cliPortPrint("Heading Hold = DISENGAGED  ");

        	    cliPortPrint("Alt Hold = ");

                switch (verticalModeState)
        	    {
        	    	case ALT_DISENGAGED_THROTTLE_ACTIVE:
		                cliPortPrint("Alt Disenaged Throttle Active\n");

        	    	    break;

        	    	case ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT:
		                cliPortPrint("Alt Hold Fixed at Engagement Alt\n");

        	    	    break;

        	    	case ALT_HOLD_AT_REFERENCE_ALTITUDE:
		                cliPortPrint("Alt Hold at Reference Alt\n");

        	    	    break;

        	    	case VERTICAL_VELOCITY_HOLD_AT_REFERENCE_VELOCITY:
		                cliPortPrint("V Velocity Hold at Reference Vel\n");

        	    	    break;

        	    	case ALT_DISENGAGED_THROTTLE_INACTIVE:
        	    	    cliPortPrint("Alt Disengaged Throttle Inactive\n");

        	    	    break;
                }

        	    validCliCommand = false;
        	    break;

            ///////////////////////////////

            case 's': // Raw Receiver Commands
                if ((eepromConfig.receiverType == SPEKTRUM) && (maxChannelNum > 0))
                {
		    		for (index = 0; index < maxChannelNum - 1; index++)
                         cliPortPrintF("%4ld, ", spektrumBuf[index]);

                    cliPortPrintF("%4ld\n", spektrumBuf[maxChannelNum - 1]);
                }
                else if ((eepromConfig.receiverType == SPEKTRUM) && (maxChannelNum == 0))
                    cliPortPrint("Invalid Number of Spektrum Channels....\n");
		        else
		        {
		    		for (index = 0; index < 7; index++)
                        cliPortPrintF("%4i, ", ppmRxRead(index));

                    cliPortPrintF("%4i\n", ppmRxRead(7));
                }

            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 't': // Processed Receiver Commands
                for (index = 0; index < 7; index++)
                    cliPortPrintF("%8.2f, ", rxCommand[index]);

                cliPortPrintF("%8.2f\n", rxCommand[7]);

                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'u': // Command in Detent Discretes
            	cliPortPrintF("%s, ", commandInDetent[ROLL    ] ? " true" : "false");
            	cliPortPrintF("%s, ", commandInDetent[PITCH   ] ? " true" : "false");
            	cliPortPrintF("%s, ", commandInDetent[YAW     ] ? " true" : "false");
                cliPortPrintF("%s\n", commandInDetent[VERTICAL] ? " true" : "false");

                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'v': // ESC PWM Outputs
                #if defined(STM32F30X)
            	    cliPortPrintF("%4ld, %4ld, %4ld, %4ld, %4ld, %4ld\n", TIM2->CCR1,
            	                                                          TIM2->CCR2,
                                                                          TIM15->CCR1,
            	                                                          TIM15->CCR2,
            	                                                          TIM3->CCR1,
            	                                                          TIM3->CCR2);
                #endif

                #if defined(STM32F40XX)
            	    cliPortPrintF("%4ld, %4ld, %4ld, %4ld, %4ld, %4ld, %4ld, %4ld\n", TIM8->CCR4,
            	                                                                      TIM8->CCR3,
            	                                                                      TIM2->CCR1,
            	                                                                      TIM2->CCR2,
            	                                                                      TIM3->CCR1,
            	                                                                      TIM3->CCR2,
            	                                                                      TIM5->CCR1,
            	                                                                      TIM5->CCR3);
                #endif

                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'w': // Servo PWM Outputs
                #if defined(STM32F40XX)
            	    cliPortPrintF("%4ld, %4ld, ", TIM4->CCR1, TIM4->CCR2);
                #endif

            	cliPortPrintF("%4ld, %4ld\n", TIM4->CCR3, TIM4->CCR4);

                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'x':
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'y': // ESC Calibration
            	escCalibration();

            	cliQuery = 'x';
            	break;

            ///////////////////////////////

            case 'z':
                #if defined(STM32F30X)
            	    cliPortPrintF("%9.4f\n", voltageMonitor());
                #endif

                #if defined(STM32F40XX)
            	    cliPortPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", mxr9150XAxis(),
                                                                  mxr9150YAxis(),
                                                                  mxr9150ZAxis(),
                                                                  voltageMonitor());
                #endif

                break;

            ///////////////////////////////

            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////

            ///////////////////////////////

            case 'A': // Read Roll Rate PID Values
            	eepromConfig.rateP[ROLL]     = readFloatCLI();
            	eepromConfig.rateI[ROLL]     = readFloatCLI();
            	eepromConfig.rateD[ROLL]     = readFloatCLI();
            	eepromConfig.rateLimit[ROLL] = readFloatCLI();

                cliPortPrint( "\nRoll Rate PID Received....\n" );

            	cliQuery = 'a';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'B': // Read Pitch Rate PID Values
            	eepromConfig.rateP[PITCH]     = readFloatCLI();
            	eepromConfig.rateI[PITCH]     = readFloatCLI();
            	eepromConfig.rateD[PITCH]     = readFloatCLI();
            	eepromConfig.rateLimit[PITCH] = readFloatCLI();

                cliPortPrint( "\nPitch Rate PID Received....\n" );

            	cliQuery = 'a';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'C': // Read Yaw Rate PID Values
            	eepromConfig.rateP[YAW]     = readFloatCLI();
            	eepromConfig.rateI[YAW]     = readFloatCLI();
            	eepromConfig.rateD[YAW]     = readFloatCLI();
            	eepromConfig.rateLimit[YAW] = readFloatCLI();

                cliPortPrint( "\nYaw Rate PID Received....\n" );

            	cliQuery = 'a';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'D': // Read Roll Attitude PID Values
            	eepromConfig.attitudeP[ROLL]     = readFloatCLI();
            	eepromConfig.attitudeI[ROLL]     = readFloatCLI();
            	eepromConfig.attitudeD[ROLL]     = readFloatCLI();
            	eepromConfig.attitudeLimit[ROLL] = readFloatCLI();

                cliPortPrint( "\nRoll Attitude PID Received....\n" );

            	cliQuery = 'b';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'E': // Read Pitch Attitude PID Values
            	eepromConfig.attitudeP[PITCH]     = readFloatCLI();
            	eepromConfig.attitudeI[PITCH]     = readFloatCLI();
            	eepromConfig.attitudeD[PITCH]     = readFloatCLI();
            	eepromConfig.attitudeLimit[PITCH] = readFloatCLI();

                cliPortPrint( "\nPitch Attitude PID Received....\n" );

            	cliQuery = 'b';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'F': // Read Heading Hold PID Values
            	eepromConfig.attitudeP[YAW]     = readFloatCLI();
            	eepromConfig.attitudeI[YAW]     = readFloatCLI();
            	eepromConfig.attitudeD[YAW]     = readFloatCLI();
            	eepromConfig.attitudeLimit[YAW] = readFloatCLI();

                cliPortPrint( "\nHeading PID Received....\n" );

            	cliQuery = 'b';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'G': // Read nDot PID Values
            	eepromConfig.velocityP[XAXIS]     = readFloatCLI();
            	eepromConfig.velocityI[XAXIS]     = readFloatCLI();
            	eepromConfig.velocityD[XAXIS]     = readFloatCLI();
            	eepromConfig.velocityLimit[XAXIS] = readFloatCLI();

                cliPortPrint( "\nnDot PID Received....\n" );

            	cliQuery = 'c';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'H': // Read eDot PID Values
            	eepromConfig.velocityP[YAXIS]     = readFloatCLI();
            	eepromConfig.velocityI[YAXIS]     = readFloatCLI();
            	eepromConfig.velocityD[YAXIS]     = readFloatCLI();
            	eepromConfig.velocityLimit[YAXIS] = readFloatCLI();

                cliPortPrint( "\neDot PID Received....\n" );

                cliQuery = 'c';
             	validCliCommand = false;
              	break;

            ///////////////////////////////

            case 'I': // Read hDot PID Values
            	eepromConfig.velocityP[ZAXIS]     = readFloatCLI();
            	eepromConfig.velocityI[ZAXIS]     = readFloatCLI();
            	eepromConfig.velocityD[ZAXIS]     = readFloatCLI();
            	eepromConfig.velocityLimit[ZAXIS] = readFloatCLI();

                cliPortPrint( "\nhDot PID Received....\n" );

              	cliQuery = 'c';
              	validCliCommand = false;
              	break;

       	    ///////////////////////////////

            case 'J': // Read n PID Values
            	eepromConfig.positionP[XAXIS]     = readFloatCLI();
            	eepromConfig.positionI[XAXIS]     = readFloatCLI();
            	eepromConfig.positionD[XAXIS]     = readFloatCLI();
            	eepromConfig.positionLimit[XAXIS] = readFloatCLI();

                cliPortPrint( "\nn PID Received....\n" );

                cliQuery = 'd';
                validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'K': // Read e PID Values
            	eepromConfig.positionP[YAXIS]     = readFloatCLI();
            	eepromConfig.positionI[YAXIS]     = readFloatCLI();
            	eepromConfig.positionD[YAXIS]     = readFloatCLI();
            	eepromConfig.positionLimit[YAXIS] = readFloatCLI();

                cliPortPrint( "\ne PID Received....\n" );

                cliQuery = 'd';
                validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'L': // Read h PID Values
            	eepromConfig.positionP[ZAXIS]     = readFloatCLI();
            	eepromConfig.positionI[ZAXIS]     = readFloatCLI();
            	eepromConfig.positionD[ZAXIS]     = readFloatCLI();
            	eepromConfig.positionLimit[ZAXIS] = readFloatCLI();

                cliPortPrint( "\nh PID Received....\n" );

                cliQuery = 'd';
            	validCliCommand = false;
            	break;

            ///////////////////////////

            case 'M': // Attitude Calibration
            	attCalibrationCount = 500;

            	cliQuery = 'x';
            	validCliCommand = false;
            	break;

            ///////////////////////////////

            case 'N': // Mixer CLI
                mixerCLI();

                cliQuery = 'x';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'O': // Receiver CLI
                receiverCLI();

                cliQuery = 'x';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'P': // Sensor CLI
               	sensorCLI();

               	cliQuery = 'x';
               	validCliCommand = false;
               	break;

            ///////////////////////////////

            case 'Q': // GPS Data Type
            	gpsDataType = (uint8_t)readFloatCLI();

            	cliPortPrint("\n");

                cliQuery = 'n';
                validCliCommand = false;
                break;

            ///////////////////////////////

            case 'R': // Reset to Bootloader
                cliPortPrint("\nEntering Bootloader....\n\n");
            	delay(100);
            	systemReset(true);
            	break;

            ///////////////////////////////

            case 'S': // Reset System
            	cliPortPrint("\nSystem Reseting....\n\n");
            	delay(100);
            	systemReset(false);
            	break;

            ///////////////////////////////

            case 'T': // Telemetry CLI
                telemetryCLI();

                cliQuery = 'x';
             	validCliCommand = false;
             	break;

            ///////////////////////////////

            case 'U': // EEPROM CLI
                eepromCLI();

                cliQuery = 'x';
             	validCliCommand = false;
             	break;

            ///////////////////////////////

            case 'V': // Reset EEPROM Parameters
                cliPortPrint( "\nEEPROM Parameters Reset....\n" );
                checkFirstTime(true);
                cliPortPrint("\nSystem Resetting....\n\n");
                delay(100);
                systemReset(false);
                break;

            ///////////////////////////////

            case 'W': // Write EEPROM Parameters
                cliPortPrint("\nWriting EEPROM Parameters....\n\n");

                cliBusy = true;

                writeEEPROM();

                cliBusy = false;

                cliQuery = 'x';
             	validCliCommand = false;
             	break;

            ///////////////////////////////

            case 'X': // Debug
                cliPortPrintF("Home Lat:    %12.4f\n", (float)homeData.latitude  * 1e-7);
	        	cliPortPrintF("Home Long:   %12.4f\n", (float)homeData.longitude * 1e-7);
	        	cliPortPrintF("Home Alt:    %12.4f\n", homeData.altitude);
	        	cliPortPrintF("Home Hdg:    %12.4f\n", homeData.magHeading * R2D);
	        	cliPortPrintF("Home X:      %12.4f\n", homeData.geoMagX);
	        	cliPortPrintF("Home Y:      %12.4f\n", homeData.geoMagY);
	        	cliPortPrintF("Home Z:      %12.4f\n", homeData.geoMagZ);
	        	cliPortPrintF("Home H:      %12.4f\n", homeData.geoMagH);
	        	cliPortPrintF("Home F:      %12.4f\n", homeData.geoMagF);
	        	cliPortPrintF("Home INCL:   %12.4f\n", homeData.geoMagIncl);
	        	cliPortPrintF("Home DECL:   %12.4f\n", homeData.geoMagDecl);
	        	cliPortPrintF("Home hDop:   %12.4f\n", (float)homeData.hDop / 100.0f); // TEST
	        	cliPortPrint ("\n");

            	cliQuery = 'x';
            	validCliCommand = false;
                break;

            ///////////////////////////////

            case 'Y': // Accel Calibration
                accelCalibrationState = 0;
                accelCalibrating      = true;

	        	cliQuery = 'x';
                break;

            ///////////////////////////////

            case 'Z': // Mag Calibration
                magCalibrationState = 0;
                magCalibrating      = true;

                cliQuery = 'x';
                break;

            ///////////////////////////////

            case '?': // Command Summary
            	cliBusy = true;

            	cliPortPrint("\n");
   		        cliPortPrint("'a' Rate PIDs                              'A' Set Roll Rate PID Data   AP;I;D;N\n");
   		        cliPortPrint("'b' Attitude PIDs                          'B' Set Pitch Rate PID Data  BP;I;D;N\n");
   		        cliPortPrint("'c' Velocity PIDs                          'C' Set Yaw Rate PID Data    CP;I;D;N\n");
   		        cliPortPrint("'d' Position PIDs                          'D' Set Roll Att PID Data    DP;I;D;N\n");
   		        cliPortPrint("'e' Loop Delta Times                       'E' Set Pitch Att PID Data   EP;I;D;N\n");
   		        cliPortPrint("'f' Loop Execution Times                   'F' Set Hdg Hold PID Data    FP;I;D;N\n");
   		        cliPortPrint("'g' 500 Hz Accels                          'G' Set nDot PID Data        GP;I;D;N\n");
   		        cliPortPrint("'h' 100 Hz Earth Axis Accels               'H' Set eDot PID Data        HP;I;D;N\n");
   		        cliPortPrint("'i' 500 Hz Gyros                           'I' Set hDot PID Data        IP;I;D;N\n");
   		        cliPortPrint("'j' 10 Hz Mag Data                         'J' Set n PID Data           JP;I;D;N\n");
   		        cliPortPrint("'k' Vertical Axis Variable                 'K' Set e PID Data           KP;I;D;N\n");
   		        cliPortPrint("'l' Attitudes                              'L' Set h PID Data           LP;I;D;N\n");
   		        cliPortPrint("\n");

   		        cliPortPrint("Press space bar for more, or enter a command....\n");

   		        while (cliPortAvailable() == false);

   		        cliQuery = cliPortRead();

   		        if (cliQuery != ' ')
   		        {
   		            validCliCommand = true;
   		            cliBusy = false;
   		        	return;
   		        }

   		        cliPortPrint("\n");
   		        cliPortPrint("'m' Axis PIDs                              'M' Attitude Calibration\n");
   		        cliPortPrint("'n' GPS Data                               'N' Mixer CLI\n");
   		        cliPortPrint("'o' Battery Voltage                        'O' Receiver CLI\n");
   		        cliPortPrint("'p' Primary Spektrum Raw Data              'P' Sensor CLI\n");

   		        #if defined(STM32F30X)
   		            cliPortPrint("                                           'Q' GPS Data Type\n");
   		        #endif

   		        #if defined(STM32F$0XX)
   		            cliPortPrint("'q' Slave Spektrum Raw Data                'Q' GPS Data Type\n");
   		        #endif

   		        cliPortPrint("'r' Mode States                            'R' Reset and Enter Bootloader\n");
   		        cliPortPrint("'s' Raw Receiver Commands                  'S' Reset\n");
   		        cliPortPrint("'t' Processed Receiver Commands            'T' Telemetry CLI\n");
   		        cliPortPrint("'u' Command In Detent Discretes            'U' EEPROM CLI\n");
   		        cliPortPrint("'v' Motor PWM Outputs                      'V' Reset EEPROM Parameters\n");
   		        cliPortPrint("'w' Servo PWM Outputs                      'W' Write EEPROM Parameters\n");
   		        cliPortPrint("'x' Terminate Serial Communication         'X' Debug\n");
   		        cliPortPrint("\n");

   		        cliPortPrint("Press space bar for more, or enter a command....\n");

   		        while (cliPortAvailable() == false);

   		        cliQuery = cliPortRead();

   		        if (cliQuery != ' ')
   		        {
   		        	validCliCommand = true;
   		        	cliBusy = false;
   		        	return;
   		        }

   		        cliPortPrint("\n");
   		        cliPortPrint("'y' ESC Calibration                        'Y' Accel Calibration\n");
   		        cliPortPrint("'z' ADC Values                             'Z' Mag Calibration\n");
   		        cliPortPrint("                                           '?' Command Summary\n");
   		        cliPortPrint("\n");

  		        cliQuery = 'x';
  		        cliBusy = false;
   		        break;

                ///////////////////////////////
	    }
    }
}

///////////////////////////////////////////////////////////////////////////////
