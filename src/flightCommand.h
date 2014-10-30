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

#pragma once

///////////////////////////////////////////////////////////////////////////////
// Process Pilot Commands Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define DEADBAND       48
#define DEADBAND_SLOPE (1000/(1000-DEADBAND))

#define ALT_DEADBAND       200
#define ALT_DEADBAND_SLOPE (1000/(1000-ALT_DEADBAND))

#define THROTTLE_WINDOW    48

extern float rxCommand[9];

extern uint8_t commandInDetent[4];
extern uint8_t previousCommandInDetent[4];

extern uint8_t channelOrder[8];

///////////////////////////////////////////////////////////////////////////////
// Flight Mode Defines and Variables
///////////////////////////////////////////////////////////////////////////////

extern uint8_t flightMode;

extern uint8_t headingHoldEngaged;

///////////////////////////////////////////////////////////////////////////////
// Arm State Variables
///////////////////////////////////////////////////////////////////////////////

extern semaphore_t armed;
extern uint8_t     armingTimer;
extern uint8_t     disarmingTimer;

///////////////////////////////////////////////////////////////////////////////
// Verical Mode State Variables
///////////////////////////////////////////////////////////////////////////////

extern uint8_t  verticalModeState;

extern uint8_t  vertRefCmdInDetent;

extern float    verticalReferenceCommand;

///////////////////////////////////////////////////////////////////////////////
// Process Flight Commands
///////////////////////////////////////////////////////////////////////////////

void processFlightCommands(void);

///////////////////////////////////////////////////////////////////////////////
