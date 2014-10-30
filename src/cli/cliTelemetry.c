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
// Telemetry CLI
///////////////////////////////////////////////////////////////////////////////

void telemetryCLI()
{
    uint8_t  telemetryQuery = 'x';
    uint8_t  validQuery = false;

    cliBusy = true;

    cliPortPrint("\nEntering Telemetry CLI....\n\n");

    while(true)
    {
        cliPortPrint("Telemetry CLI -> ");

            while ((cliPortAvailable() == false) && (validQuery == false));

	    if (validQuery == false)
		telemetryQuery = cliPortRead();

	    cliPortPrint("\n");

	    switch(telemetryQuery)

	    {
            ///////////////////////////

            case 'a': // Telemetry Configuration
                cliPortPrint("\nTelemetry Configuration:\n");

                cliPortPrint("    Telemetry Set 1: ");
                cliPortPrintF("%s\n", eepromConfig.activeTelemetry == 1 ?   "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 2: ");
                cliPortPrintF("%s\n", eepromConfig.activeTelemetry == 2 ?   "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 3: ");
                cliPortPrintF("%s\n", eepromConfig.activeTelemetry == 4 ?   "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 4: ");
                cliPortPrintF("%s\n", eepromConfig.activeTelemetry == 8 ?   "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 5: ");
                cliPortPrintF("%s\n", eepromConfig.activeTelemetry == 16 ?  "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 6: ");
                cliPortPrintF("%s\n", eepromConfig.activeTelemetry == 32 ?  "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 7: ");
                cliPortPrintF("%s\n", eepromConfig.activeTelemetry == 64 ?  "  Active" : "Inactive");

                cliPortPrint("    Telemetry Set 8: ");
                cliPortPrintF("%s\n", eepromConfig.activeTelemetry == 128 ? "  Active" : "Inactive");

                validQuery = false;
                break;

            ///////////////////////////

            case '0': // Turn all Telemetry Off
                eepromConfig.activeTelemetry = 0;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case '1': // Toggle Telemetry Set 1 State
                eepromConfig.activeTelemetry = 1;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case '2': // Toggle Telemetry Set 2 State
                eepromConfig.activeTelemetry = 2;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case '3': // Toggle Telemetry Set 3 State
                eepromConfig.activeTelemetry = 4;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case '4': // Toggle Telemetry Set 4 State
                eepromConfig.activeTelemetry = 8;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case '5': // Toggle Telemetry Set 5 State
                eepromConfig.activeTelemetry = 16;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case '6': // Toggle Telemetry Set 6 State
                eepromConfig.activeTelemetry = 32;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case '7': // Toggle Telemetry Set 7 State
                eepromConfig.activeTelemetry = 64;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

            case '8': // Toggle Telemetry Set 8 State
                eepromConfig.activeTelemetry = 128;

                telemetryQuery = 'a';
                validQuery     = true;
                break;

            ///////////////////////////

			case 'x':
			    cliPortPrint("\nExiting Telemetry CLI....\n\n");
			    cliBusy = false;
			    return;
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
			   	cliPortPrint("'a' Telemetry Configuration Data\n");
   		        cliPortPrint("'0' Turn all Telemetry Off\n");
			   	cliPortPrint("'1' Toggle Telemetry Set 1 State\n");
			   	cliPortPrint("'2' Toggle Telemetry Set 2 State\n");
			   	cliPortPrint("'3' Toggle Telemetry Set 3 State\n");
			   	cliPortPrint("'4' Toggle Telemetry Set 4 State\n");
   		        cliPortPrint("'5' Toggle Telemetry Set 5 State\n");
   		        cliPortPrint("'6' Toggle Telemetry Set 6 State\n");
   		        cliPortPrint("'7' Toggle Telemetry Set 7 State\n");
   		        cliPortPrint("'8' Toggle Telemetry Set 8 State\n");
   		        cliPortPrint("                                           'W' Write EEPROM Parameters\n");
   		        cliPortPrint("'x' Exit Telemetry CLI                     '?' Command Summary\n");
   		        cliPortPrint("\n");
	    	    break;

	    	///////////////////////////
	    }
	}
}

///////////////////////////////////////////////////////////////////////////////
