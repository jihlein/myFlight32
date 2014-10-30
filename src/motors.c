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

////////////////////////////////////////////////////////////////////////////////

uint8_t numberMotor;

///////////////////////////////////////////////////////////////////////////////
// Set Motor Count
///////////////////////////////////////////////////////////////////////////////

void setMotorCount(void)
{
    switch (eepromConfig.mixerConfiguration)
    {
        case MIXERTYPE_TRI:
            numberMotor = 3;
            break;

       case MIXERTYPE_QUADX:
            numberMotor = 4;
            break;

        case MIXERTYPE_HEX6X:
        case MIXERTYPE_Y6:
            numberMotor = 6;
            break;

        case MIXERTYPE_FREE:
		    numberMotor = eepromConfig.freeMixMotors;
        	break;
    }
}///////////////////////////////////////////////////////////////////////////////
// Write to Motors
///////////////////////////////////////////////////////////////////////////////

void writeMotors(void)
{
    uint8_t i;

    for (i = 0; i < numberMotor; i++)
        pwmEscWrite(i, (uint16_t)tasks500Hz_Y.motorCmds[i]);

    if (eepromConfig.mixerConfiguration == MIXERTYPE_TRI)
        pwmEscWrite(LAST_MOTOR - 1, (uint16_t)tasks500Hz_Y.triCopterServoCmd);
  }

///////////////////////////////////////////////////////////////////////////////
// Write to All Motors
///////////////////////////////////////////////////////////////////////////////

void writeAllMotors(float mc)
{
    uint8_t i;

    // Sends commands to all motors
    for (i = 0; i < numberMotor; i++)
        tasks500Hz_Y.motorCmds[i] = mc;
    writeMotors();
}

///////////////////////////////////////////////////////////////////////////////
// Pulse Motors
///////////////////////////////////////////////////////////////////////////////

void pulseMotors(uint8_t quantity)
{
    uint8_t i;

    for ( i = 0; i < quantity; i++ )
    {
        writeAllMotors( eepromConfig.minThrottle );
        delay(250);
        writeAllMotors( (float)MINCOMMAND );
        delay(250);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Limit Motor Cmds
///////////////////////////////////////////////////////////////////////////////

void limitMotorCmds(void)
{
    int16_t maxMotor;
    uint8_t i;

    ///////////////////////////////////

    // this is a way to still have good gyro corrections if any motor reaches its max.

    maxMotor = tasks500Hz_Y.motorCmds[0];

    for (i = 1; i < numberMotor; i++)
        if (tasks500Hz_Y.motorCmds[i] > maxMotor)
            maxMotor = tasks500Hz_Y.motorCmds[i];

    for (i = 0; i < numberMotor; i++)
    {
        if (maxMotor > eepromConfig.maxThrottle)
            tasks500Hz_Y.motorCmds[i] -= maxMotor - eepromConfig.maxThrottle;

        tasks500Hz_Y.motorCmds[i] = constrain(tasks500Hz_Y.motorCmds[i], eepromConfig.minThrottle, eepromConfig.maxThrottle);

        if ((rxCommand[THROTTLE] < eepromConfig.minCheck) && (verticalModeState == ALT_DISENGAGED_THROTTLE_ACTIVE))
            tasks500Hz_Y.motorCmds[i] = eepromConfig.minThrottle;

        if ( armed == false )
            tasks500Hz_Y.motorCmds[i] = (float)MINCOMMAND;
    }
}

///////////////////////////////////////////////////////////////////////////////
