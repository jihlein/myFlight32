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

static const float wmmCoefficients[90][4] = {
    // 2010.0            WMM-2010        11/20/2009
    { -29496.6f,       0.0f,       11.6f,        0.0f },  // Row 0
    {  -1586.3f,    4944.4f,       16.5f,      -25.9f },
    {  -2396.6f,       0.0f,      -12.1f,        0.0f },
    {   3026.1f,   -2707.7f,       -4.4f,      -22.5f },
    {   1668.6f,    -576.1f,        1.9f,      -11.8f },
    {   1340.1f,       0.0f,        0.4f,        0.0f },
    {  -2326.2f,    -160.2f,       -4.1f,        7.3f },
    {   1231.9f,     251.9f,       -2.9f,       -3.9f },
    {    634.0f,    -536.6f,       -7.7f,       -2.6f },
    {    912.6f,       0.0f,       -1.8f,        0.0f },
    {    808.9f,     286.4f,        2.3f,        1.1f },  // Row 10
    {    166.7f,    -211.2f,       -8.7f,        2.7f },
    {   -357.1f,     164.3f,        4.6f,        3.9f },
    {     89.4f,    -309.1f,       -2.1f,       -0.8f },
    {   -230.9f,       0.0f,       -1.0f,        0.0f },
    {    357.2f,      44.6f,        0.6f,        0.4f },
    {    200.3f,     188.9f,       -1.8f,        1.8f },
    {   -141.1f,    -118.2f,       -1.0f,        1.2f },
    {   -163.0f,       0.0f,        0.9f,        4.0f },
    {     -7.8f,     100.9f,        1.0f,       -0.6f },
    {     72.8f,       0.0f,       -0.2f,        0.0f },  // Row 20
    {     68.6f,     -20.8f,       -0.2f,       -0.2f },
    {     76.0f,      44.1f,       -0.1f,       -2.1f },
    {   -141.4f,      61.5f,        2.0f,       -0.4f },
    {    -22.8f,     -66.3f,       -1.7f,       -0.6f },
    {     13.2f,       3.1f,       -0.3f,        0.5f },
    {    -77.9f,      55.0f,        1.7f,        0.9f },
    {     80.5f,       0.0f,        0.1f,        0.0f },
    {    -75.1f,     -57.9f,       -0.1f,        0.7f },
    {     -4.7f,     -21.1f,       -0.6f,        0.3f },
    {     45.3f,       6.5f,        1.3f,       -0.1f },  // Row 30
    {     13.9f,      24.9f,        0.4f,       -0.1f },
    {     10.4f,       7.0f,        0.3f,       -0.8f },
    {      1.7f,     -27.7f,       -0.7f,       -0.3f },
    {      4.9f,      -3.3f,        0.6f,        0.3f },
    {     24.4f,       0.0f,       -0.1f,        0.0f },
    {      8.1f,      11.0f,        0.1f,       -0.1f },
    {    -14.5f,     -20.0f,       -0.6f,        0.2f },
    {     -5.6f,      11.9f,        0.2f,        0.4f },
    {    -19.3f,     -17.4f,       -0.2f,        0.4f },
    {     11.5f,      16.7f,        0.3f,        0.1f },  // Row 40
    {     10.9f,       7.0f,        0.3f,       -0.1f },
    {    -14.1f,     -10.8f,       -0.6f,        0.4f },
    {     -3.7f,       1.7f,        0.2f,        0.3f },
    {      5.4f,       0.0f,        0.0f,        0.0f },
    {      9.4f,     -20.5f,       -0.1f,        0.0f },
    {      3.4f,      11.5f,        0.0f,       -0.2f },
    {     -5.2f,      12.8f,        0.3f,        0.0f },
    {      3.1f,      -7.2f,       -0.4f,       -0.1f },
    {    -12.4f,      -7.4f,       -0.3f,        0.1f },
    {     -0.7f,       8.0f,        0.1f,        0.0f },  // Row 50
    {      8.4f,       2.1f,       -0.1f,       -0.2f },
    {     -8.5f,      -6.1f,       -0.4f,        0.3f },
    {    -10.1f,       7.0f,       -0.2f,        0.2f },
    {     -2.0f,       0.0f,        0.0f,        0.0f },
    {     -6.3f,       2.8f,        0.0f,        0.1f },
    {      0.9f,      -0.1f,       -0.1f,       -0.1f },
    {     -1.1f,       4.7f,        0.2f,        0.0f },
    {     -0.2f,       4.4f,        0.0f,       -0.1f },
    {      2.5f,      -7.2f,       -0.1f,       -0.1f },
    {     -0.3f,      -1.0f,       -0.2f,        0.0f },  // Row 60
    {      2.2f,      -3.9f,        0.0f,       -0.1f },
    {      3.1f,      -2.0f,       -0.1f,       -0.2f },
    {     -1.0f,      -2.0f,       -0.2f,        0.0f },
    {     -2.8f,      -8.3f,       -0.2f,       -0.1f },
    {      3.0f,       0.0f,        0.0f,        0.0f },
    {     -1.5f,       0.2f,        0.0f,        0.0f },
    {     -2.1f,       1.7f,        0.0f,        0.1f },
    {      1.7f,      -0.6f,        0.1f,        0.0f },
    {     -0.5f,      -1.8f,        0.0f,        0.1f },
    {      0.5f,       0.9f,        0.0f,        0.0f },  // Row 70
    {     -0.8f,      -0.4f,        0.0f,        0.1f },
    {      0.4f,      -2.5f,        0.0f,        0.0f },
    {      1.8f,      -1.3f,        0.0f,       -0.1f },
    {      0.1f,      -2.1f,        0.0f,       -0.1f },
    {      0.7f,      -1.9f,       -0.1f,        0.0f },
    {      3.8f,      -1.8f,        0.0f,       -0.1f },
    {     -2.2f,       0.0f,        0.0f,        0.0f },
    {     -0.2f,      -0.9f,        0.0f,        0.0f },
    {      0.3f,       0.3f,        0.1f,        0.0f },
    {      1.0f,       2.1f,        0.1f,        0.0f },  // Row 80
    {     -0.6f,      -2.5f,       -0.1f,        0.0f },
    {      0.9f,       0.5f,        0.0f,        0.0f },
    {     -0.1f,       0.6f,        0.0f,        0.1f },
    {      0.5f,       0.0f,        0.0f,        0.0f },
    {     -0.4f,       0.1f,        0.0f,        0.0f },
    {     -0.4f,       0.3f,        0.0f,        0.0f },
    {      0.2f,      -0.9f,        0.0f,        0.0f },
    {     -0.8f,      -0.2f,       -0.1f,        0.0f },
    {      0.0f,       0.9f,        0.1f,        0.0f }   // Row 89
};

///////////////////////////////////////////////////////////////////////////////

