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
// Receiver CLI
///////////////////////////////////////////////////////////////////////////////

void receiverCLI()
{
    char     rcOrderString[9];
    float    tempFloat;
    uint8_t  index;
    uint8_t  receiverQuery = 'x';
    uint8_t  validQuery    = false;

    cliBusy = true;

    cliPortPrint("\nEntering Receiver CLI....\n\n");

    while(true)
    {
        cliPortPrint("Receiver CLI -> ");

		while ((cliPortAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    receiverQuery = cliPortRead();

		cliPortPrint("\n");

		switch(receiverQuery)
		{
            ///////////////////////////

            case 'a': // Receiver Configuration
                cliPortPrint("\nReceiver Type:                  ");
                switch(eepromConfig.receiverType)
                {
                    case PPM:
                        cliPortPrint("PPM\n");
                        break;
                    case SPEKTRUM:
                        cliPortPrint("Spektrum\n");
                        break;
		        }

                cliPortPrint("Current RC Channel Assignment:  ");
                for (index = 0; index < 8; index++)
                    rcOrderString[eepromConfig.rcMap[index]] = rcChannelLetters[index];

                rcOrderString[index] = '\0';

                cliPortPrint(rcOrderString);  cliPortPrint("\n");

                #if defined(STM32F40XX)
                    cliPortPrintF("Secondary Spektrum:             ");

                    if (eepromConfig.slaveSpektrum == true)
                        cliPortPrintF("Installed\n");
                    else
                        cliPortPrintF("Uninstalled\n");
                #endif

                cliPortPrintF("Mid Command:                    %4ld\n",   (uint16_t)eepromConfig.midCommand);
				cliPortPrintF("Min Check:                      %4ld\n",   (uint16_t)eepromConfig.minCheck);
				cliPortPrintF("Max Check:                      %4ld\n",   (uint16_t)eepromConfig.maxCheck);
				cliPortPrintF("Min Throttle:                   %4ld\n",   (uint16_t)eepromConfig.minThrottle);
				cliPortPrintF("Max Thottle:                    %4ld\n\n", (uint16_t)eepromConfig.maxThrottle);

				tempFloat = eepromConfig.rollAndPitchRateScaling * 180000.0 / PI;
				cliPortPrintF("Max Roll and Pitch Rate Cmd:    %6.2f DPS\n", tempFloat);

				tempFloat = eepromConfig.yawRateScaling * 180000.0 / PI;
				cliPortPrintF("Max Yaw Rate Cmd:               %6.2f DPS\n\n", tempFloat);

				tempFloat = eepromConfig.attitudeScaling * 180000.0 / PI;
                cliPortPrintF("Max Attitude Cmd:               %6.2f Degrees\n\n", tempFloat);

				cliPortPrintF("Arm Delay Count:                %3d Frames\n",   eepromConfig.armCount);
				cliPortPrintF("Disarm Delay Count:             %3d Frames\n\n", eepromConfig.disarmCount);

				validQuery = false;
				break;

            ///////////////////////////

			case 'x':
			    cliPortPrint("\nExiting Receiver CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Toggle PPM/Spektrum Satellite Receiver
            	if (eepromConfig.receiverType == PPM)
                {
                    #if defined(STM32F30X)
                        TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
                    #endif

                    #if defined(STM32F40XX)
                        TIM_ITConfig(TIM1, TIM_IT_CC4, DISABLE);
                    #endif

                	eepromConfig.receiverType = SPEKTRUM;

                    spektrumInit();
                }
                else
                {
                	#if defined(STM32F30X)
                	    TIM_ITConfig(TIM17, TIM_IT_Update, DISABLE);
                    #endif

                	#if defined(STM32F40XX)
                	    TIM_ITConfig(TIM6, TIM_IT_Update, DISABLE);
                    #endif

                  	eepromConfig.receiverType = PPM;

                    #if defined(STM32F30X)
                  	    ppmRxInit();
                    #endif

                    #if defined(STM32F40XX)
                  	    agl_ppmRxInit();
                    #endif
                }

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'B': // Read RC Control Order
                readStringCLI( rcOrderString, 8 );
                parseRcChannels( rcOrderString );

          	    receiverQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            #if defined(STM32F40XX)
        	    case 'C': // Toggle Slave Spektrum State
                    if (eepromConfig.slaveSpektrum == true)
                	    eepromConfig.slaveSpektrum = false;
                    else
                	    eepromConfig.slaveSpektrum = true;

                    receiverQuery = 'a';
                    validQuery = true;
                    break;
            #endif

            ///////////////////////////

            case 'D': // Read RC Control Points
                eepromConfig.midCommand   = readFloatCLI();
    	        eepromConfig.minCheck     = readFloatCLI();
    		    eepromConfig.maxCheck     = readFloatCLI();
    		    eepromConfig.minThrottle  = readFloatCLI();
    		    eepromConfig.maxThrottle  = readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // Read Arm/Disarm Counts
                eepromConfig.armCount    = (uint8_t)readFloatCLI();
        	    eepromConfig.disarmCount = (uint8_t)readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'F': // Read Max Rate Value
                eepromConfig.rollAndPitchRateScaling = readFloatCLI() / 180000.0f * PI;
                eepromConfig.yawRateScaling          = readFloatCLI() / 180000.0f * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'G': // Read Max Attitude Value
                eepromConfig.attitudeScaling = readFloatCLI() / 180000.0f * PI;

                receiverQuery = 'a';
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
			   	cliPortPrint("'a' Receiver Configuration Data            'A' Toggle PPM/Spektrum Receiver\n");
   		        cliPortPrint("                                           'B' Set RC Control Order                 BTAER1234\n");

                #if defined(STM32F40XX)
   		            cliPortPrint("                                           'C' Toggle Slave Spektrum State\n");
                #endif

			   	cliPortPrint("                                           'D' Set RC Control Points                DmidCmd;minChk;maxChk;minThrot;maxThrot\n");
			   	cliPortPrint("                                           'E' Set Arm/Disarm Counts                EarmCount;disarmCount\n");
			   	cliPortPrint("                                           'F' Set Maximum Rate Commands            FRP;Y RP = Roll/Pitch, Y = Yaw\n");
			   	cliPortPrint("                                           'G' Set Maximum Attitude Command\n");
			   	cliPortPrint("                                           'W' Write EEPROM Parameters\n");
			   	cliPortPrint("'x' Exit Receiver CLI                      '?' Command Summary\n");
			   	cliPortPrint("\n");
	    	    break;

	    	///////////////////////////
	    }
	}

}

///////////////////////////////////////////////////////////////////////////////
