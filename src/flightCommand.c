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
// Process Pilot Commands Defines and Variables
///////////////////////////////////////////////////////////////////////////////

float    rxCommand[9] = { 0.0f, 0.0f, 0.0f, 0.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f };

uint8_t  commandInDetent[4]         = { true, true, true, true };
uint8_t  previousCommandInDetent[4] = { true, true, true, true };

///////////////////////////////////////////////////////////////////////////////
// Flight Mode Defines and Variables
///////////////////////////////////////////////////////////////////////////////

uint8_t flightMode = RATE;

uint8_t headingHoldEngaged = false;

///////////////////////////////////////////////////////////////////////////////
// Arm State Variables
///////////////////////////////////////////////////////////////////////////////

semaphore_t armed          = false;
uint8_t     armingTimer    = 0;
uint8_t     disarmingTimer = 0;

///////////////////////////////////////////////////////////////////////////////
// Vertical Mode State Variables
///////////////////////////////////////////////////////////////////////////////

uint8_t  verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

uint16_t previousAUX2State = MINCOMMAND;
uint16_t previousAUX4State = MINCOMMAND;

///////////////////////////////////////////////////////////////////////////////
// Read Flight Commands
///////////////////////////////////////////////////////////////////////////////

void processFlightCommands(void)
{
    uint8_t channel, subChannel;

    float hdgDelta, simpleX, simpleY;

    if (rcActive == true)
    {
		// Read receiver commands
        for (channel = 0; channel < 8; channel++)
        {
			subChannel = channel;

			if (channel >= VERTICAL)
			    subChannel++;

			if (eepromConfig.receiverType == SPEKTRUM)
			    rxCommand[subChannel] = (float)spektrumRead(eepromConfig.rcMap[channel]);
			else
			    rxCommand[subChannel] = (float)ppmRxRead(eepromConfig.rcMap[channel]);
        }

        rxCommand[ROLL]    -= eepromConfig.midCommand;                       // Roll Range     -1000:1000
        rxCommand[PITCH]   -= eepromConfig.midCommand;                        // Pitch Range    -1000:1000
        rxCommand[YAW]     -= eepromConfig.midCommand;                        // Yaw Range      -1000:1000
        rxCommand[VERTICAL] = rxCommand[THROTTLE] - eepromConfig.midCommand;  // Vertical Range -1000:1000

        rxCommand[THROTTLE] -= eepromConfig.midCommand - MIDCOMMAND;  // Throttle Range 2000:4000
        rxCommand[AUX1]     -= eepromConfig.midCommand - MIDCOMMAND;  // Aux1 Range     2000:4000
        rxCommand[AUX2]     -= eepromConfig.midCommand - MIDCOMMAND;  // Aux2 Range     2000:4000
        rxCommand[AUX3]     -= eepromConfig.midCommand - MIDCOMMAND;  // Aux3 Range     2000:4000
        rxCommand[AUX4]     -= eepromConfig.midCommand - MIDCOMMAND;  // Aux4 Range     2000:4000
    }

    // Set past command in detent values
    for (channel = 0; channel < 4; channel++)
    	previousCommandInDetent[channel] = commandInDetent[channel];

    // Apply deadbands and set detent discretes'
    for (channel = 0; channel < 4; channel++)
    {
    	if ((rxCommand[channel] <= DEADBAND) && (rxCommand[channel] >= -DEADBAND))
        {
            rxCommand[channel] = 0;
  	        commandInDetent[channel] = true;
  	    }
        else
  	    {
  	        commandInDetent[channel] = false;
  	        if (rxCommand[channel] > 0)
  	        {
  		        rxCommand[channel] = (rxCommand[channel] - DEADBAND) * DEADBAND_SLOPE;
  	        }
  	        else
  	        {
  	            rxCommand[channel] = (rxCommand[channel] + DEADBAND) * DEADBAND_SLOPE;
  	        }
        }
    }

    ///////////////////////////////////

    // Check for low throttle
    if ( rxCommand[THROTTLE] < eepromConfig.minCheck )
    {
		// Check for disarm command ( low throttle, left yaw )
		if (((rxCommand[YAW] < (eepromConfig.minCheck - MIDCOMMAND)) && (armed == true)) && (verticalModeState == ALT_DISENGAGED_THROTTLE_ACTIVE))
		{
			disarmingTimer++;

			if (disarmingTimer > eepromConfig.disarmCount)
			{
				armed = false;
			    disarmingTimer = 0;
			}
		}
		else
		{
			disarmingTimer = 0;
		}

		// Check for gyro bias command ( low throttle, left yaw, aft pitch, right roll )
		if ( (rxCommand[YAW  ] < (eepromConfig.minCheck - MIDCOMMAND)) &&
		     (rxCommand[ROLL ] > (eepromConfig.maxCheck - MIDCOMMAND)) &&
		     (rxCommand[PITCH] < (eepromConfig.minCheck - MIDCOMMAND)) )
		{
	        gyroRTBias[ROLL ] = 0.0f;
	        gyroRTBias[PITCH] = 0.0f;
	        gyroRTBias[YAW  ] = 0.0f;

	        gyroRunTimeCalCount = 2520;

	        pulseMotors(3);
		}

		// Check for arm command ( low throttle, right yaw)
		if ((rxCommand[YAW] > (eepromConfig.maxCheck - MIDCOMMAND) ) && (armed == false) && (systemOperational == true) && (gyroRunTimeCalCount == 0))
		{
			armingTimer++;

			if (armingTimer > eepromConfig.armCount)
			{
				armed = true;
				armingTimer = 0;
			}
		}
		else
		{
			armingTimer = 0;
		}
	}

	///////////////////////////////////

	// Check for armed true and throttle command > minThrottle

    if ((armed == true) && (rxCommand[THROTTLE] > eepromConfig.minThrottle))
    	tasks500Hz_U.resetPIDs = false;
    else
    	tasks500Hz_U.resetPIDs = true;

	///////////////////////////////////

	// Simple Mode Command Processing

	if (rxCommand[AUX3] > MIDCOMMAND)
	{
        hdgDelta = tasks500Hz_Y.attitudes[YAW] - homeData.magHeading;

        hdgDelta = standardRadianFormat(hdgDelta);

        simpleX = cosf(hdgDelta) * rxCommand[PITCH] + sinf(hdgDelta) * rxCommand[ROLL ];

        simpleY = cosf(hdgDelta) * rxCommand[ROLL ] - sinf(hdgDelta) * rxCommand[PITCH];

        rxCommand[ROLL ] = simpleY;

        rxCommand[PITCH] = simpleX;
	}

    ///////////////////////////////////

    // Check AUX1 for rate, attitude, or GPS mode (3 Position Switch) NOT COMPLETE YET....

	if (rxCommand[AUX1] > MIDCOMMAND)
	{
		flightMode = ATTITUDE;

		tasks500Hz_U.rateModes[ROLL ] = OUTER_LOOP_INPUT;
		tasks500Hz_U.rateModes[PITCH] = OUTER_LOOP_INPUT;

		tasks500Hz_U.attModes[ROLL ] = COMMAND_INPUT;
		tasks500Hz_U.attModes[PITCH] = COMMAND_INPUT;

		tasks500Hz_U.velModes[XAXIS] = STATE_INPUT;
		tasks500Hz_U.velModes[YAXIS] = STATE_INPUT;

		tasks500Hz_U.posModes[XAXIS] = STATE_INPUT;
		tasks500Hz_U.posModes[YAXIS] = STATE_INPUT;

		tasks500Hz_U.attCmds[ROLL ] = rxCommand[ROLL ] * eepromConfig.attitudeScaling;
		tasks500Hz_U.attCmds[PITCH] = rxCommand[PITCH] * eepromConfig.attitudeScaling;
	}
	else if (rxCommand[AUX1] <= MIDCOMMAND)
	{
		flightMode = RATE;

		tasks500Hz_U.rateModes[ROLL ] = COMMAND_INPUT;
		tasks500Hz_U.rateModes[PITCH] = COMMAND_INPUT;

		tasks500Hz_U.attModes[ROLL ] = STATE_INPUT;
		tasks500Hz_U.attModes[PITCH] = STATE_INPUT;

		tasks500Hz_U.velModes[XAXIS] = STATE_INPUT;
		tasks500Hz_U.velModes[YAXIS] = STATE_INPUT;

		tasks500Hz_U.posModes[XAXIS] = STATE_INPUT;
		tasks500Hz_U.posModes[YAXIS] = STATE_INPUT;

		tasks500Hz_U.rateCmds[ROLL ] = rxCommand[ROLL ] * eepromConfig.rollAndPitchRateScaling;
		tasks500Hz_U.rateCmds[PITCH] = rxCommand[PITCH] * eepromConfig.rollAndPitchRateScaling;
	}

	///////////////////////////////////

	// Check yaw in detent and flight mode to determine hdg hold engaged state

	if ((commandInDetent[YAW] == true) &&
		(flightMode == ATTITUDE)       &&
		(headingHoldEngaged == false)  &&
		(abs(tasks500Hz_U.gyro[YAW]) < (5.0f * D2R)))
	{
		headingHoldEngaged = true;

		tasks500Hz_U.rateModes[YAW] = OUTER_LOOP_INPUT;
		tasks500Hz_U.attModes[YAW]  = COMMAND_INPUT;

		tasks500Hz_U.attCmds[YAW] = tasks500Hz_Y.attitudes[YAW];
	}

	if ((commandInDetent[YAW] == false) || (flightMode != ATTITUDE))
	{
	    headingHoldEngaged = false;

	    tasks500Hz_U.rateModes[YAW] = COMMAND_INPUT;
	    tasks500Hz_U.attModes[YAW]  = STATE_INPUT;

	    tasks500Hz_U.rateCmds[YAW] = rxCommand[YAW] * eepromConfig.yawRateScaling;
	}

	///////////////////////////////////

    // Vertical Mode State Machine

    switch (verticalModeState)
	{
		case ALT_DISENGAGED_THROTTLE_ACTIVE:
		    tasks500Hz_U.velModes[ZAXIS] = STATE_INPUT;
		    tasks500Hz_U.posModes[ZAXIS] = STATE_INPUT;

		    tasks500Hz_U.velCmds[ZAXIS] = rxCommand[THROTTLE];

			if ((rxCommand[AUX2] > MIDCOMMAND) && (previousAUX2State <= MIDCOMMAND))  // AUX2 Rising edge detection
		    {
				verticalModeState = ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT;

				tasks500Hz_U.posCmds[ZAXIS] = tasks500Hz_U.positions[ZAXIS];
		    }

		    break;

		///////////////////////////////

		case ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT:
			tasks500Hz_U.velModes[ZAXIS] = OUTER_LOOP_INPUT;
			tasks500Hz_U.posModes[ZAXIS] = COMMAND_INPUT;

		    if ((commandInDetent[VERTICAL] == true) || eepromConfig.verticalVelocityHoldOnly)
		        verticalModeState = ALT_HOLD_AT_REFERENCE_ALTITUDE;

		    if ((rxCommand[AUX2] <= MIDCOMMAND) && (previousAUX2State > MIDCOMMAND))  // AUX2 Falling edge detection
		        verticalModeState = ALT_DISENGAGED_THROTTLE_INACTIVE;

		    if ((rxCommand[AUX4] > MIDCOMMAND) && (previousAUX4State <= MIDCOMMAND))  // AUX4 Rising edge detection
		    	verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

		    break;

		///////////////////////////////

		case ALT_HOLD_AT_REFERENCE_ALTITUDE:
			tasks500Hz_U.velModes[ZAXIS] = OUTER_LOOP_INPUT;
			tasks500Hz_U.posModes[ZAXIS] = COMMAND_INPUT;

		    if ((commandInDetent[VERTICAL] == false) || eepromConfig.verticalVelocityHoldOnly)
		        verticalModeState = VERTICAL_VELOCITY_HOLD_AT_REFERENCE_VELOCITY;

		    if ((rxCommand[AUX2] <= MIDCOMMAND) && (previousAUX2State > MIDCOMMAND))  // AUX2 Falling edge detection
		        verticalModeState = ALT_DISENGAGED_THROTTLE_INACTIVE;

		    if ((rxCommand[AUX4] > MIDCOMMAND) && (previousAUX4State <= MIDCOMMAND))  // AUX4 Rising edge detection
		    	verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

		    break;

		///////////////////////////////

		case VERTICAL_VELOCITY_HOLD_AT_REFERENCE_VELOCITY:
			tasks500Hz_U.velModes[ZAXIS] = COMMAND_INPUT;
			tasks500Hz_U.posModes[ZAXIS] = STATE_INPUT;

			tasks500Hz_U.velCmds[ZAXIS] = rxCommand[VERTICAL] * eepromConfig.hDotScaling;

		    if ((commandInDetent[VERTICAL] == true) && !eepromConfig.verticalVelocityHoldOnly)
		    {
				verticalModeState = ALT_HOLD_AT_REFERENCE_ALTITUDE;

				tasks500Hz_U.posCmds[ZAXIS] = tasks500Hz_U.positions[ZAXIS];
			}

		    if ((rxCommand[AUX2] <= MIDCOMMAND) && (previousAUX2State > MIDCOMMAND))  // AUX2 Falling edge detection
		    {
				verticalModeState = ALT_DISENGAGED_THROTTLE_INACTIVE;

				tasks500Hz_U.posCmds[ZAXIS] = tasks500Hz_U.positions[ZAXIS];
			}

		    if ((rxCommand[AUX4] > MIDCOMMAND) && (previousAUX4State <= MIDCOMMAND))  // AUX4 Rising edge detection
		    	verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

		    break;

		///////////////////////////////

		case ALT_DISENGAGED_THROTTLE_INACTIVE:
			if (((rxCommand[THROTTLE] < tasks500Hz_Y.axisCmds[VERTICAL] + THROTTLE_WINDOW) && (rxCommand[THROTTLE] > tasks500Hz_Y.axisCmds[VERTICAL] - THROTTLE_WINDOW)) ||
			    eepromConfig.verticalVelocityHoldOnly)
			    verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

			if ((rxCommand[AUX2] > MIDCOMMAND) && (previousAUX2State <= MIDCOMMAND))  // AUX2 Rising edge detection
		        verticalModeState = ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT;

			if ((rxCommand[AUX4] > MIDCOMMAND) && (previousAUX4State <= MIDCOMMAND))  // AUX4 Rising edge detection
			    verticalModeState = ALT_DISENGAGED_THROTTLE_ACTIVE;

		    break;
    }

	previousAUX2State = rxCommand[AUX2];
	previousAUX4State = rxCommand[AUX4];

	///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////




