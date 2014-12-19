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
            	#endif

            	cliPortPrintF("\n");

                cliPortPrintF("MPU Min/Max Temperature:   %11.6f, %11.6f\n", eepromConfig.mpuTempMin,
				                                                             eepromConfig.mpuTempMax);

                cliPortPrintF("X Accel Bias Polynomial:   %11.6f, %11.6f, %11.6f, %11.6f\n", eepromConfig.accelBiasPolynomial[XAXIS * 4 + 0],
				                                                    		                 eepromConfig.accelBiasPolynomial[XAXIS * 4 + 1],
				                                                    		                 eepromConfig.accelBiasPolynomial[XAXIS * 4 + 2],
				                                                    		                 eepromConfig.accelBiasPolynomial[XAXIS * 4 + 3]);

                cliPortPrintF("Y Accel Bias Polynomial:   %11.6f, %11.6f, %11.6f, %11.6f\n", eepromConfig.accelBiasPolynomial[YAXIS * 4 + 0],
				                                                    		                 eepromConfig.accelBiasPolynomial[YAXIS * 4 + 1],
				                                                    		                 eepromConfig.accelBiasPolynomial[YAXIS * 4 + 2],
				                                                    		                 eepromConfig.accelBiasPolynomial[YAXIS * 4 + 3]);

                cliPortPrintF("Z Accel Bias Polynomial:   %11.6f, %11.6f, %11.6f, %11.6f\n", eepromConfig.accelBiasPolynomial[ZAXIS * 4 + 0],
				                                                    		                 eepromConfig.accelBiasPolynomial[ZAXIS * 4 + 1],
				                                                    		                 eepromConfig.accelBiasPolynomial[ZAXIS * 4 + 2],
				                                                    		                 eepromConfig.accelBiasPolynomial[ZAXIS * 4 + 3]);

				cliPortPrintF("X Accel SF Polynomial:     %11.6f, %11.6f, %11.6f, %11.6f\n", eepromConfig.accelScaleFactorPolynomial[XAXIS * 4 + 0],
				                                                                             eepromConfig.accelScaleFactorPolynomial[XAXIS * 4 + 1],
				                                                                             eepromConfig.accelScaleFactorPolynomial[XAXIS * 4 + 2],
				                                                                             eepromConfig.accelScaleFactorPolynomial[XAXIS * 4 + 3]);

				cliPortPrintF("Y Accel SF Polynomial:     %11.6f, %11.6f, %11.6f, %11.6f\n", eepromConfig.accelScaleFactorPolynomial[YAXIS * 4 + 0],
				                                                                             eepromConfig.accelScaleFactorPolynomial[YAXIS * 4 + 1],
				                                                                             eepromConfig.accelScaleFactorPolynomial[YAXIS * 4 + 2],
				                                                                             eepromConfig.accelScaleFactorPolynomial[YAXIS * 4 + 3]);

				cliPortPrintF("Z Accel SF Polynomial:     %11.6f, %11.6f, %11.6f, %11.6f\n", eepromConfig.accelScaleFactorPolynomial[ZAXIS * 4 + 0],
				                                                                             eepromConfig.accelScaleFactorPolynomial[ZAXIS * 4 + 1],
				                                                                             eepromConfig.accelScaleFactorPolynomial[ZAXIS * 4 + 2],
				                                                                             eepromConfig.accelScaleFactorPolynomial[ZAXIS * 4 + 3]);

                cliPortPrintF("X Gyro Bias Polynomial:    %11.6f, %11.6f, %11.6f, %11.6f\n", eepromConfig.gyroBiasPolynomial[ROLL  * 4 + 0],
                                                                		                     eepromConfig.gyroBiasPolynomial[ROLL  * 4 + 1],
                                                                		                     eepromConfig.gyroBiasPolynomial[ROLL  * 4 + 2],
                                                                		                     eepromConfig.gyroBiasPolynomial[ROLL  * 4 + 3]);

                cliPortPrintF("Y Gyro Bias Polynomial:    %11.6f, %11.6f, %11.6f, %11.6f\n", eepromConfig.gyroBiasPolynomial[PITCH * 4 + 0],
                                                                		                     eepromConfig.gyroBiasPolynomial[PITCH * 4 + 1],
                                                                		                     eepromConfig.gyroBiasPolynomial[PITCH * 4 + 2],
                                                                		                     eepromConfig.gyroBiasPolynomial[PITCH * 4 + 3]);

                cliPortPrintF("Z Gyro Bias Polynomial:    %11.6f, %11.6f, %11.6f, %11.6f\n", eepromConfig.gyroBiasPolynomial[YAW   * 4 + 0],
                                                                		                     eepromConfig.gyroBiasPolynomial[YAW   * 4 + 1],
                                                                		                     eepromConfig.gyroBiasPolynomial[YAW   * 4 + 2],
                                                                		                     eepromConfig.gyroBiasPolynomial[YAW   * 4 + 3]);
                if (eepromConfig.externalHMC5883 == 0)
                {
					cliPortPrintF("Internal Mag Bias:         %9.4f,   %9.4f,   %9.4f\n", eepromConfig.magBias[XAXIS],
                                                   		                                  eepromConfig.magBias[YAXIS],
                                                   		                                  eepromConfig.magBias[ZAXIS]);
                }
                else
                {
					cliPortPrintF("External Mag Bias:         %9.4f,   %9.4f,   %9.4f\n", eepromConfig.magBias[XAXIS + 3],
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

            case 'b': // MPU Temperature Limits
                eepromConfig.mpuTempMin = readFloatCLI();
                eepromConfig.mpuTempMax = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'c': // X Accel Bias Polynomial
                eepromConfig.accelBiasPolynomial[XAXIS * 4 + 0] = readFloatCLI();
                eepromConfig.accelBiasPolynomial[XAXIS * 4 + 1] = readFloatCLI();
                eepromConfig.accelBiasPolynomial[XAXIS * 4 + 2] = readFloatCLI();
                eepromConfig.accelBiasPolynomial[XAXIS * 4 + 3] = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'd': // Y Accel Bias Polynomial
                eepromConfig.accelBiasPolynomial[YAXIS * 4 + 0] = readFloatCLI();
                eepromConfig.accelBiasPolynomial[YAXIS * 4 + 1] = readFloatCLI();
                eepromConfig.accelBiasPolynomial[YAXIS * 4 + 2] = readFloatCLI();
                eepromConfig.accelBiasPolynomial[YAXIS * 4 + 3] = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'e': // Z Accel Bias Polynomial
                eepromConfig.accelBiasPolynomial[ZAXIS * 4 + 0] = readFloatCLI();
                eepromConfig.accelBiasPolynomial[ZAXIS * 4 + 1] = readFloatCLI();
                eepromConfig.accelBiasPolynomial[ZAXIS * 4 + 2] = readFloatCLI();
                eepromConfig.accelBiasPolynomial[ZAXIS * 4 + 3] = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'f': // X Accel Scale Factor Polynomial
                eepromConfig.accelScaleFactorPolynomial[XAXIS * 4 + 0] = readFloatCLI();
                eepromConfig.accelScaleFactorPolynomial[XAXIS * 4 + 1] = readFloatCLI();
                eepromConfig.accelScaleFactorPolynomial[XAXIS * 4 + 2] = readFloatCLI();
                eepromConfig.accelScaleFactorPolynomial[XAXIS * 4 + 3] = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'g': // Y Accel Scale Factor Polynomial
                eepromConfig.accelScaleFactorPolynomial[YAXIS * 4 + 0] = readFloatCLI();
                eepromConfig.accelScaleFactorPolynomial[YAXIS * 4 + 1] = readFloatCLI();
                eepromConfig.accelScaleFactorPolynomial[YAXIS * 4 + 2] = readFloatCLI();
                eepromConfig.accelScaleFactorPolynomial[YAXIS * 4 + 3] = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'h': // Z Accel Scale Factor Polynomial
                eepromConfig.accelScaleFactorPolynomial[ZAXIS * 4 + 0] = readFloatCLI();
                eepromConfig.accelScaleFactorPolynomial[ZAXIS * 4 + 1] = readFloatCLI();
                eepromConfig.accelScaleFactorPolynomial[ZAXIS * 4 + 2] = readFloatCLI();
                eepromConfig.accelScaleFactorPolynomial[ZAXIS * 4 + 3] = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'i': // X Gyro Bias Polynomial
                eepromConfig.gyroBiasPolynomial[ROLL  * 4 + 0] = readFloatCLI();
                eepromConfig.gyroBiasPolynomial[ROLL  * 4 + 1] = readFloatCLI();
                eepromConfig.gyroBiasPolynomial[ROLL  * 4 + 2] = readFloatCLI();
                eepromConfig.gyroBiasPolynomial[ROLL  * 4 + 3] = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'j': // Y Gyro Bias Polynomial
                eepromConfig.gyroBiasPolynomial[PITCH * 4 + 0] = readFloatCLI();
                eepromConfig.gyroBiasPolynomial[PITCH * 4 + 1] = readFloatCLI();
                eepromConfig.gyroBiasPolynomial[PITCH * 4 + 2] = readFloatCLI();
                eepromConfig.gyroBiasPolynomial[PITCH * 4 + 3] = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'k': // Z Gyro Bias Polynomial
                eepromConfig.gyroBiasPolynomial[YAW   * 4 + 0] = readFloatCLI();
                eepromConfig.gyroBiasPolynomial[YAW   * 4 + 1] = readFloatCLI();
                eepromConfig.gyroBiasPolynomial[YAW   * 4 + 2] = readFloatCLI();
                eepromConfig.gyroBiasPolynomial[YAW   * 4 + 3] = readFloatCLI();

                sensorQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'n': // Toggle External HMC5883 Use
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
                case 'o': // Toggle External MS5611 Use
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

            case 'p': // Toggle GPS Installed
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
			   	cliPortPrint("'b' Set MPU Temperature Limits             'B' Set Accel Cutoff                     BAccelCutoff\n");
			   	cliPortPrint("'c' Set X Accel Bias Polynomial            'C' Set kpAcc                            CkpAcc\n");
			   	cliPortPrint("'d' Set Y Accel Bias Polynomial            'D' Set kpMag                            DkpMag\n");
                cliPortPrint("'e' Set Z Accel Bias Polynomial            'E' Set Roll/Pitch Trim                  ERoll;Pitch\n");
                cliPortPrint("'f' Set X Accel Scale Factor Polynomial    'F' Set posEstA/posEstB                  FA;B\n");
                cliPortPrint("'g' Set Y Accel Scale Factor Polynomial    'G' Set hEstA/hEstB                      GA;B\n");
                cliPortPrint("'h' Set Z Accel Scale Factor Polynomial    'H' Set Sensor Orientation               H1 thru 4\n");
                cliPortPrint("'i' Set X Gyro Bias Polynomial             'I' Set Earth Accel HP Tau               Itau\n");
                cliPortPrint("'j' Set Y Gyro Bias Polynomial             'J' Not Used\n");
                cliPortPrint("'k' Set Z Gyro Bias Polynomial             'K' Not Used\n");
                cliPortPrint("'l' Not Used                               'L' Not Used\n");
                cliPortPrint("'m' Not Used                               'M' Not Used\n");
			   	cliPortPrint("'n' Toggle External HMC5883 State          'N' Set Voltage Monitor Trip Points      Nlow;veryLow;maxLow\n");

			    #if defined(STM32F40XX)
	                cliPortPrint("'o' Toggle External MS5611 State           'O' Not Used\n");
                #else
	                cliPortPrint("'o' Not Used                               'O' Not Used\n");
	            #endif

	            cliPortPrint("'p' Toggle GPS Installed                   'P' Not Used\n");
	            cliPortPrint("'v' Toggle Vertical Velocity Hold Only     'V' Set Voltage Monitor Parameters       Vscale;bias;cells\n");
			   	cliPortPrint("'w' Not Used                               'W' Write EEPROM Parameters\n");
			   	cliPortPrint("'x' Exit Sensor CLI                        '?' Command Summary\n");
	    	    break;

	    	///////////////////////////
	    }
	}
}

///////////////////////////////////////////////////////////////////////////////
