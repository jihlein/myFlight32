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
// Sensor CLI
///////////////////////////////////////////////////////////////////////////////

void sensorCLI()
{
    uint8_t  sensorQuery = 'x';
    uint8_t  tempInt;
    uint8_t  validQuery  = false;

    cliBusy = true;

    cliPortPrint("\nEntering Sensor CLI....\n\n");

    while(true)
    {
        cliPortPrint("Sensor CLI -> ");

		while ((cliPortAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    sensorQuery = cliPortRead();

		cliPortPrint("\n");

		switch(sensorQuery)
		{
            ///////////////////////////

            case 'a': // Sensor Data
            	cliPortPrintF("\n");
            	cliPortPrintF("External HMC5883 in use:   %s\n", eepromConfig.externalHMC5883 ? "Yes" : "No");

            	#if defined(STM32F40XX)
            	    cliPortPrintF("External MS5611  in use:   %s\n", eepromConfig.externalMS5611  ? "Yes" : "No");
            	    cliPortPrintF("MXR9150 Accel in use:      %s\n", eepromConfig.useMXR9150      ? "Yes" : "No");
            	#endif

            	cliPortPrintF("\n");

                if (eepromConfig.useMXR9150 == 0)
                {
					cliPortPrintF("MPU Accel Bias:            %9.3f, %9.3f, %9.3f\n", eepromConfig.accelBias[XAXIS],
				                                                    		          eepromConfig.accelBias[YAXIS],
				                                                    		          eepromConfig.accelBias[ZAXIS]);
				    cliPortPrintF("MPU Accel Scale Factor:    %9.7f, %9.7f, %9.7f\n", eepromConfig.accelScaleFactor[XAXIS],
								                                                      eepromConfig.accelScaleFactor[YAXIS],
				                                                		              eepromConfig.accelScaleFactor[ZAXIS]);
			    }
                else
                {
					cliPortPrintF("MXR Accel Bias:            %9.3f, %9.3f, %9.3f\n", eepromConfig.accelBias[XAXIS + 3],
				                                                		              eepromConfig.accelBias[YAXIS + 3],
				                                                		              eepromConfig.accelBias[ZAXIS + 3]);
				    cliPortPrintF("MXR Accel Scale Factor:    %9.7f, %9.7f, %9.7f\n", eepromConfig.accelScaleFactor[XAXIS + 3],
								                                                      eepromConfig.accelScaleFactor[YAXIS + 3],
				                                                		              eepromConfig.accelScaleFactor[ZAXIS + 3]);
                }

                cliPortPrintF("Accel Temp Comp Slope:     %9.4f, %9.4f, %9.4f\n", eepromConfig.accelTCBiasSlope[XAXIS],
                                                		                          eepromConfig.accelTCBiasSlope[YAXIS],
                                                		                          eepromConfig.accelTCBiasSlope[ZAXIS]);
                cliPortPrintF("Accel Temp Comp Bias:      %9.4f, %9.4f, %9.4f\n", eepromConfig.accelTCBiasIntercept[XAXIS],
                                                		                          eepromConfig.accelTCBiasIntercept[YAXIS],
                                                		                          eepromConfig.accelTCBiasIntercept[ZAXIS]);
                cliPortPrintF("Gyro Temp Comp Slope:      %9.4f, %9.4f, %9.4f\n", eepromConfig.gyroTCBiasSlope[ROLL ],
                                                                		          eepromConfig.gyroTCBiasSlope[PITCH],
                                                                		          eepromConfig.gyroTCBiasSlope[YAW  ]);
                cliPortPrintF("Gyro Temp Comp Intercept:  %9.4f, %9.4f, %9.4f\n", eepromConfig.gyroTCBiasIntercept[ROLL ],
                                                                   		          eepromConfig.gyroTCBiasIntercept[PITCH],
                                                                   		          eepromConfig.gyroTCBiasIntercept[YAW  ]);
                if (eepromConfig.externalHMC5883 == 0)
                {
					cliPortPrintF("Internal Mag Bias:         %9.4f, %9.4f, %9.4f\n", eepromConfig.magBias[XAXIS],
                                                   		                              eepromConfig.magBias[YAXIS],
                                                   		                              eepromConfig.magBias[ZAXIS]);
                }
                else
                {
					cliPortPrintF("External Mag Bias:         %9.4f, %9.4f, %9.4f\n", eepromConfig.magBias[XAXIS + 3],
                                                   		                              eepromConfig.magBias[YAXIS + 3],
                                                   		                              eepromConfig.magBias[ZAXIS + 3]);
                }

                cliPortPrintF("Accel One G:               %9.4f\n", accelOneG);
                cliPortPrintF("Accel Cutoff:              %9.4f\n", eepromConfig.accelCutoff);
                cliPortPrintF("kpAcc (MARG):              %9.4f\n", eepromConfig.kpAcc);
                cliPortPrintF("kpMag (MARG):              %9.4f\n", eepromConfig.kpMag);
                cliPortPrintF("Roll Trim:                 %9.4f\n", eepromConfig.attTrim[ROLL ] * R2D);
                cliPortPrintF("Pitch Trim:                %9.4f\n", eepromConfig.attTrim[PITCH] * R2D);
                cliPortPrintF("Earth Accel HP Tau:        %9.4f\n", eepromConfig.earthAccel100HzHPtau);
                cliPortPrintF("posEstA:                   %9.4f\n", eepromConfig.posEstA);
                cliPortPrintF("posEstB:                   %9.4f\n", eepromConfig.posEstB);
                cliPortPrintF("hEstA:                     %9.4f\n", eepromConfig.hEstA);
                cliPortPrintF("hEstB:                     %9.4f\n", eepromConfig.hEstB);

                cliPortPrint("MPU6000 DLPF:                 ");
                switch(eepromConfig.dlpfSetting)
                {
                    case DLPF_256HZ:
                        cliPortPrint("256 Hz\n");
                        break;
                    case DLPF_188HZ:
                        cliPortPrint("188 Hz\n");
                        break;
                    case DLPF_98HZ:
                        cliPortPrint("98 Hz\n");
                        break;
                    case DLPF_42HZ:
                        cliPortPrint("42 Hz\n");
                        break;
                }

                cliPortPrint("Sensor Orientation:           ");
                switch(eepromConfig.sensorOrientation)
                {
                    case 1:
                        cliPortPrint("Normal\n");
                        break;
                    case 2:
                        cliPortPrint("Rotated 90 Degrees CW\n");
                        break;
                    case 3:
                        cliPortPrint("Rotated 180 Degrees\n");
                        break;
                    case 4:
                        cliPortPrint("Rotated 90 Degrees CCW\n");
                        break;
                    default:
                        cliPortPrint("Normal\n");
				}

                if (eepromConfig.useGPS)
                	cliPortPrint("\nGPS Installed\n\n");
                else
                	cliPortPrint("\nGPS Not Installed\n\n");

                if (eepromConfig.verticalVelocityHoldOnly)
                	cliPortPrint("Vertical Velocity Hold Only\n\n");
                else
                	cliPortPrint("Vertical Velocity and Altitude Hold\n\n");

                cliPortPrintF("Voltage Monitor Scale:     %9.4f\n",    eepromConfig.voltageMonitorScale);
                cliPortPrintF("Voltage Monitor Bias:      %9.4f\n",    eepromConfig.voltageMonitorBias);
                cliPortPrintF("Number of Battery Cells:      %1d\n\n", eepromConfig.batteryCells);

                cliPortPrintF("Battery Low Setpoint:      %4.2f volts\n",   eepromConfig.batteryLow);
                cliPortPrintF("Battery Very Low Setpoint: %4.2f volts\n",   eepromConfig.batteryVeryLow);
                cliPortPrintF("Battery Max Low Setpoint:  %4.2f volts\n\n", eepromConfig.batteryMaxLow);

                validQuery = false;
                break;

            ///////////////////////////

            case 'b': // MPU6000 Calibration
                mpu6000Calibration();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'e': // Toggle External HMC5883 Use
                if (eepromConfig.externalHMC5883 == 0)
                {
                	eepromConfig.externalHMC5883 = 3;

                	initMag5883();
                }
                else
                {
               	    eepromConfig.externalHMC5883 = 0;

               	    #if defined(STM32F30X)
               	        initMag5983();
               	    #endif

               	    #if defined(STM32F40XX)
               	        initMag5883();
               	    #endif
                }

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            #if defined(STM32F40XX)
                case 'f': // Toggle External MS5611 Use
                    if (eepromConfig.externalMS5611)
                	    eepromConfig.externalMS5611 = false;
                    else
               	        eepromConfig.externalMS5611 = true;

                    initPressure();

                    sensorQuery = 'a';
                    validQuery = true;
                    break;
            #endif

            ///////////////////////////

            #if defined(STM32F40XX)
                case 'g': // Toggle MXR9150 Use
                    if (eepromConfig.useMXR9150 == 0)
                   	    eepromConfig.useMXR9150 = 3;
                    else
               	        eepromConfig.useMXR9150 = 0;

                    sensorQuery = 'a';
                    validQuery = true;
                    break;
            #endif

            ///////////////////////////

            case 'h': // Toggle GPS Installed
                if (eepromConfig.useGPS == false)
                   	eepromConfig.useGPS = true;
                else
               	    eepromConfig.useGPS = false;

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'v': // Toggle Vertical Velocity Hold Only
                if (eepromConfig.verticalVelocityHoldOnly)
                	eepromConfig.verticalVelocityHoldOnly = false;
                else
               	    eepromConfig.verticalVelocityHoldOnly = true;

                sensorQuery = 'a';
                validQuery = true;
                break;

			///////////////////////////

        	case 'x':
			    cliPortPrint("\nExiting Sensor CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Set MPU6000 Digital Low Pass Filter
                tempInt = (uint8_t)readFloatCLI();

                switch(tempInt)
                {
                    case DLPF_256HZ:
                        eepromConfig.dlpfSetting = BITS_DLPF_CFG_256HZ;
                        break;

                    case DLPF_188HZ:
                    	eepromConfig.dlpfSetting = BITS_DLPF_CFG_188HZ;
                    	break;

                    case DLPF_98HZ:
                    	eepromConfig.dlpfSetting = BITS_DLPF_CFG_98HZ;
                    	break;

                    case DLPF_42HZ:
                    	eepromConfig.dlpfSetting = BITS_DLPF_CFG_42HZ;
                     	break;
                }

                setSPIdivisor(MPU6000_SPI, 64);  // 0.65625 MHz SPI clock (within 20 +/- 10%)

                GPIO_ResetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN);
			    spiTransfer(MPU6000_SPI, MPU6000_CONFIG);
			    spiTransfer(MPU6000_SPI, eepromConfig.dlpfSetting);
			    GPIO_SetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN);

                setSPIdivisor(MPU6000_SPI, 2);  // 21 MHz SPI clock (within 20 +/- 10%)

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'B': // Accel Cutoff
                eepromConfig.accelCutoff = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // kpAcc
                eepromConfig.kpAcc = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // kpMag
                eepromConfig.kpMag = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // Roll/Pitch Trim
                eepromConfig.attTrim[ROLL ] = readFloatCLI() * D2R;
                eepromConfig.attTrim[PITCH] = readFloatCLI() * D2R;

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'F': // posEstA/posEstB
                eepromConfig.posEstA = readFloatCLI();
                eepromConfig.posEstB = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'G': // hEstA/hEstb
                eepromConfig.hEstA = readFloatCLI();
                eepromConfig.hEstB = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

           ///////////////////////////

            case 'H': // Sensor Orientation
                eepromConfig.sensorOrientation = (uint8_t)readFloatCLI();

                orientSensors();

                sensorQuery = 'a';
                validQuery = true;
                break;

           ///////////////////////////

           case 'I': // Earth Accel HP Tau
               eepromConfig.earthAccel100HzHPtau = readFloatCLI();

               initMatlabFilters();

               sensorQuery = 'a';
               validQuery = true;
               break;

            ///////////////////////////

            case 'N': // Set Voltage Monitor Trip Points
                eepromConfig.batteryLow     = readFloatCLI();
                eepromConfig.batteryVeryLow = readFloatCLI();
                eepromConfig.batteryMaxLow  = readFloatCLI();

                thresholds[BATTERY_LOW].value      = eepromConfig.batteryLow;
                thresholds[BATTERY_VERY_LOW].value = eepromConfig.batteryVeryLow;
                thresholds[BATTRY_MAX_LOW].value   = eepromConfig.batteryMaxLow;

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'V': // Set Voltage Monitor Parameters
                eepromConfig.voltageMonitorScale = readFloatCLI();
                eepromConfig.voltageMonitorBias  = readFloatCLI();
                eepromConfig.batteryCells        = (uint8_t)readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'W': // Write EEPROM Parameters
                cliPortPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();

                validQuery = false;
                break;

			///////////////////////////

			case '?':
			   	cliPortPrint("\n");
			   	cliPortPrint("'a' Display Sensor Data                    'A' Set MPU6000 DLPF                     A0 thru 3, see aq32Plus.h\n");
			   	cliPortPrint("'b' MPU6000 Temp Calibration               'B' Set Accel Cutoff                     BAccelCutoff\n");
			   	cliPortPrint("                                           'C' Set kpAcc                            CkpAcc\n");
			   	cliPortPrint("                                           'D' Set kpMag                            DkpMag\n");
			   	cliPortPrint("'e' Toggle External HMC5883 State          'E' Set Roll/Pitch Trim                  ERoll;Pitch\n");

			   	#if defined(STM32F30X)
			   	    cliPortPrint("                                           'F' Set posEstA/posEstB                  FA;B\n");
			   	    cliPortPrint("                                           'G' Set hEstA/hEstB                      GA;B\n");
			   	#endif

			   	#if defined(STM32F40XX)
			   	    cliPortPrint("'f' Toggle External MS5611 State           'F' Set posEstA/posEstB                  FA;B\n");
			   	    cliPortPrint("'g' Toggle MXR9150 Use                     'G' Set hEstA/hEstB                      GA;B\n");
			   	#endif

			   	cliPortPrint("'h' Toggle GPS Installed                   'H' Set Sensor Orientation               H1 thru 4\n");
			    cliPortPrint("                                           'I' Set Earth Accel HP Tau               Itau\n");
			   	cliPortPrint("                                           'N' Set Voltage Monitor Trip Points      Nlow;veryLow;maxLow\n");
			   	cliPortPrint("'v' Toggle Vertical Velocity Hold Only     'V' Set Voltage Monitor Parameters       Vscale;bias;cells\n");
			   	cliPortPrint("                                           'W' Write EEPROM Parameters\n");
			   	cliPortPrint("'x' Exit Sensor CLI                        '?' Command Summary\n");
	    	    break;

	    	///////////////////////////
	    }
	}
}

///////////////////////////////////////////////////////////////////////////////
