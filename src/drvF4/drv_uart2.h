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
// UART2 Init
///////////////////////////////////////////////////////////////////////////////

void uart2Init(void);

///////////////////////////////////////////////////////////////////////////////
// UART2 Available
///////////////////////////////////////////////////////////////////////////////

uint32_t uart2Available(void);

///////////////////////////////////////////////////////////////////////////////
// UART2 Clear Buffer
///////////////////////////////////////////////////////////////////////////////

void uart2ClearBuffer(void);

///////////////////////////////////////////////////////////////////////////////
// UART2 Number of Characters Available
///////////////////////////////////////////////////////////////////////////////

uint16_t uart2NumCharsAvailable(void);

///////////////////////////////////////////////////////////////////////////////
// UART2 Read
///////////////////////////////////////////////////////////////////////////////

uint8_t uart2Read(void);

///////////////////////////////////////////////////////////////////////////////
// UART2 Read Poll
///////////////////////////////////////////////////////////////////////////////

uint8_t uart2ReadPoll(void);

///////////////////////////////////////////////////////////////////////////////
// UART2 Write
///////////////////////////////////////////////////////////////////////////////

void uart2Write(uint8_t ch);

///////////////////////////////////////////////////////////////////////////////
// UART2 Print
///////////////////////////////////////////////////////////////////////////////

void uart2Print(char *str);

///////////////////////////////////////////////////////////////////////////////
// UART2 Print Formatted - Print formatted string to UART2
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void uart2PrintF(const char * fmt, ...);

///////////////////////////////////////////////////////////////////////////////
// UART2 Print Binary
///////////////////////////////////////////////////////////////////////////////

void uart2PrintBinary(uint8_t *buf, uint16_t length);

///////////////////////////////////////////////////////////////////////////////
