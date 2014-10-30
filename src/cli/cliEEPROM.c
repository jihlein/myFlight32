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
// EEPROM CLI
///////////////////////////////////////////////////////////////////////////////

#define LINE_LENGTH 32

#define TIMEOUT 100

///////////////////////////////////////

int min(int a, int b)
{
    return a < b ? a : b;
}

///////////////////////////////////////

int8_t parse_hex(char c)
{
    if ('0' <= c && c <= '9')
        return c - '0';
    if ('a' <= c && c <= 'f')
        return c - 'a' + 0x0A;
    if ('A' <= c && c <= 'F')
        return c - 'A' + 0x0A;
    return -1;
}

///////////////////////////////////////

void cliPrintEEPROM(void)
{
    uint32_t old_crc = eepromConfig.CRCAtEnd;

    uint8_t *by = (uint8_t*)&eepromConfig;

    int i, j;

    eepromConfig.CRCAtEnd = crc32B((uint32_t*)(&eepromConfig),
                                   (uint32_t*)(&eepromConfig.CRCAtEnd));

    if (eepromConfig.CRCFlags & CRC_HistoryBad)
      evrPush(EVR_ConfigBadHistory, 0);

    for (i = 0; i < ceil((float)sizeof(eepromConfig) / LINE_LENGTH); i++)
    {
        for (j = 0; j < min(LINE_LENGTH, sizeof(eepromConfig) - LINE_LENGTH * i); j++)
            cliPortPrintF("%02X", by[i * LINE_LENGTH + j]);

        cliPortPrint("\n");
    }

    eepromConfig.CRCAtEnd = old_crc;
}

///////////////////////////////////////

void eepromCLI()
{
	char c;

	eepromConfig_t eepromRam;

	uint8_t  eepromQuery = 'x';

	uint8_t  *p;

	uint8_t  *end;

	uint8_t  secondNibble;

	uint8_t  validQuery = false;

	uint16_t i;

	uint32_t c1, c2;

    uint32_t size;

	uint32_t time;

	uint32_t charsEncountered;

	///////////////////////////////////////////////////////////////////////////////

    cliBusy = true;

    cliPortPrint("\nEntering EEPROM CLI....\n\n");

    while(true)
    {
        cliPortPrint("EEPROM CLI -> ");

        while ((cliPortAvailable() == false) && (validQuery == false));

        if (validQuery == false)
            eepromQuery = cliPortRead();

        cliPortPrint("\n");

        switch(eepromQuery)
        {
            // 'a' is the standard "print all the information" character
            case 'a': // config struct data
                c1 = eepromConfig.CRCAtEnd;

                c2 = crc32B((uint32_t*)(&eepromConfig),                  // CRC32B[eepromConfig]
                            (uint32_t*)(&eepromConfig.CRCAtEnd));

                cliPortPrintF("Config structure information:\n");
                cliPortPrintF("Version          : %d\n", eepromConfig.version );
                cliPortPrintF("Size             : %d\n", sizeof(eepromConfig) );
                cliPortPrintF("CRC on last read : %08X\n", c1 );
                cliPortPrintF("Current CRC      : %08X\n", c2 );

                if ( c1 != c2 )
                    cliPortPrintF("  CRCs differ. Current Config has not yet been saved.\n");

                cliPortPrintF("CRC Flags :\n");
                cliPortPrintF("  History Bad    : %s\n", eepromConfig.CRCFlags & CRC_HistoryBad ? "true" : "false" );

                validQuery = false;
                break;

            ///////////////////////////

            case 'c': // Write out to Console in Hex.  (RAM -> console)
                cliPortPrintF("\n");

                cliPrintEEPROM();

                cliPortPrintF("\n");

                if (crcCheckVal != crc32B((uint32_t*)(&eepromConfig),       // CRC32B[eepromConfig CRC32B[eepromConfig]]
                                          (uint32_t*)(&eepromConfig + 1)))
                {
                    cliPortPrint("NOTE: in-memory config CRC invalid; there have probably been changes to\n");
                    cliPortPrint("      eepromConfig since the last write to flash/eeprom.\n");
                }

                validQuery = false;
                break;

            ///////////////////////////

            case 'H': // clear bad history flag
                cliPortPrintF("Clearing Bad History flag.\n");
                eepromConfig.CRCFlags &= ~CRC_HistoryBad;

                validQuery = false;
                break;

            ///////////////////////////

            case 'C': // Read in from Console in hex.  Console -> RAM
                charsEncountered = 0;

                secondNibble = 0;

                size = sizeof(eepromConfig);

                time = millis();

                p = (uint8_t*)&eepromRam;

                end = (uint8_t*)(&eepromRam + 1);

                ///////////////////////

                cliPortPrintF("Ready to read in config. Expecting %d (0x%03X) bytes as %d\n", size, size, size * 2);

                cliPortPrintF("hexadecimal characters, optionally separated by [ \\n\\r_].\n");

                cliPortPrintF("Times out if no character is received for %dms\n", TIMEOUT);

                memset(p, 0, end - p);

                while (p < end)
                {
                    while (!cliPortAvailable() && millis() - time < TIMEOUT) {}
                    time = millis();

                    c = cliPortAvailable() ? cliPortRead() : '\0';

                    int8_t hex = parse_hex(c);

                    int ignore = c == ' ' || c == '\n' || c == '\r' || c == '_' ? true : false;

                    if (c != '\0') // assume the person isn't sending null chars
                        charsEncountered++;

                    if (ignore)
                        continue;

                    if (hex == -1)
                        break;

                    *p |= secondNibble ? hex : hex << 4;

                    p += secondNibble;

                    secondNibble ^= 1;
                }

                if (c == 0)
                {
                    cliPortPrintF("Did not receive enough hex chars! (got %d, expected %d)\n",
                        (p - (uint8_t*)&eepromRam) * 2 + secondNibble, size * 2);
                }
                else if (p < end || secondNibble)
                {
                    cliPortPrintF("Invalid character found at position %d: '%c' (0x%02x)",
                        charsEncountered, c, c);
                }
                // HJI else if (crcCheckVal != crc32B((uint32_t*)(&eepromConfig),       // CRC32B[sensorConfig CRC32B[sensorConfig]]
                // HJI                                (uint32_t*)(&eepromConfig + 1)))
                // HJI {
                // HJI     cliPortPrintF("CRC mismatch! Not writing to in-memory config.\n");
                // HJI     cliPortPrintF("Here's what was received:\n\n");
                // HJI     cliPrintSensorEEPROM();
                // HJI }
                else
                {
                    // check to see if the newly received eeprom config
                    // actually differs from what's in-memory

                    for (i = 0; i < size; i++)
                        if (((uint8_t*)&eepromRam)[i] != ((uint8_t*)&eepromConfig)[i])
                            break;

                    if (i == size)
                    {
                        cliPortPrintF("NOTE: uploaded config was identical to in-memory config.\n");
                    }
                    else
                    {
                        eepromConfig = eepromRam;
                        cliPortPrintF("In-memory config updated!\n");
                        cliPortPrintF("NOTE: config not written to EEPROM; use 'W' to do so.\n");
                    }

                }

                // eat the next 100ms (or whatever Timeout is) of characters,
                // in case the person pasted too much by mistake or something

                time = millis();

                while (millis() - time < TIMEOUT)
                    if (cliPortAvailable())
                        cliPortRead();

                validQuery = false;
                break;

            ///////////////////////////

            case 'h': // Read in from EEPROM.  (EEPROM -> RAM)
                cliPortPrint("Re-reading EEPROM.\n");
                readEEPROM();
                validQuery = false;
                break;

            ///////////////////////////

            case 'x': // exit EEPROM CLI
                cliPortPrint("\nExiting EEPROM CLI....\n\n");
                cliBusy = false;
                return;
                break;

            ///////////////////////////

            case 'W':
            case 'e': // Write out to EEPROM. (RAM -> EEPROM)
                cliPortPrint("\nWriting EEPROM Parameters....\n\n");

                validQuery = false;
                writeEEPROM();
                break;

            ///////////////////////////

            case 'f': // Write out to sdCard FILE. (RAM -> FILE)
                validQuery = false;
                break;

            ///////////////////////////

            case 'F': // Read in from sdCard FILE. (FILE -> RAM)
                validQuery = false;
                break;

            ///////////////////////////

            case 'V': // Reset EEPROM Parameters
                cliPortPrint( "\nEEPROM Parameters Reset....(not rebooting)\n" );
                checkFirstTime(true);
                validQuery = false;
            break;


            ///////////////////////////

            case '?':
            //                0         1         2         3         4         5         6         7
            //                01234567890123456789012345678901234567890123456789012345678901234567890123456789
                cliPortPrintF("\n");
                cliPortPrintF("'a' Display in-RAM config information\n");
                cliPortPrintF("'c' Write in-RAM -> Console (as Hex)      'C' Read Console (as Hex) -> in-RAM\n");
                cliPortPrintF("'e' Write in-RAM -> EEPROM                'E' Read EEPROM -> in-RAM\n");
                cliPortPrintF("'f' Write in-RAM -> sd FILE (Not yet imp) 'F' Read sd FILE -> in-RAM (Not imp)\n");
                cliPortPrintF("                                          'H' Clear CRC Bad History flag\n");
                cliPortPrintF("                                          'V' Reset in-RAM config to default.\n");
                cliPortPrintF("'x' Exit EEPROM CLI                       '?' Command Summary\n");
                cliPortPrintF("\n");
                cliPortPrintF("For compatability:                        'W' Write in-RAM -> EEPROM\n");
                cliPortPrintF("\n");
                break;

            ///////////////////////////
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
