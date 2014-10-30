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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

///////////////////////////////////////////////////////////////////////////////

#if defined(STM32F30X)
    #include "stm32f30x.h"
    #include "stm32f30x_conf.h"

    #include "hw_config.h"
    #include "usb_lib.h"
    #include "usb_desc.h"
    #include "usb_pwr.h"
#endif

#if defined(STM32F40XX)
    #include "stm32f4xx.h"

    #include "usbd_cdc_core.h"
    #include "usbd_cdc.h"
    #include "usbd_usr.h"
    #include "usbd_desc.h"
#endif

#include "arm_math.h"

/////////////////////////////////////////////////////////////////////////////

#include "mavlink.h"

/////////////////////////////////////////////////////////////////////////////

#include "rtwtypes.h"

#include "tasks100Hz.h"
#include "tasks500Hz.h"

///////////////////////////////////////////////////////////////////////////////

#include "myFlight32.h"

#if defined(STM32F30X)
    #include "drv_adc.h"
    #include "drv_agl.h"
    #include "drv_crc.h"
    #include "drv_gpio.h"
    #include "drv_i2c.h"
    #include "drv_ppmRx.h"
    #include "drv_pwmEsc.h"
    #include "drv_pwmServo.h"
    #include "drv_spektrum.h"
    #include "drv_spi.h"
    #include "drv_system.h"
    #include "drv_timingFunctions.h"
    #include "drv_usb.h"
    #include "drv_uart1.h"
    #include "drv_uart2.h"

    #include "hmcCommon.h"
    #include "mpu6000.h"
    #include "ms5611_SPI.h"
    #include "orientation.h"
    #include "ublox.h"
#endif

#if defined(STM32F40XX)
    #include "drv_adc.h"
    #include "drv_agl_ppmRx.h"
    #include "drv_crc.h"
    #include "drv_gpio.h"
    #include "drv_i2c.h"
    #include "drv_pwmEsc.h"
    #include "drv_pwmServo.h"
    #include "drv_sdCard.h"
    #include "drv_spektrum.h"
    #include "drv_spi.h"
    #include "drv_system.h"
    #include "drv_timingFunctions.h"
    #include "drv_uart1.h"
    #include "drv_uart2.h"
    #include "drv_uart3.h"
    #include "drv_uart6.h"
    #include "drv_usb.h"

    #include "hmc5883.h"
    #include "mpu6000.h"
    #include "ms5611_I2C.h"
    #include "orientation.h"
    #include "uBlox.h"
#endif

#include "accelCalibration.h"
#include "attitudeCalibration.h"
#include "batMon.h"
#include "cli.h"
#include "eeprom.h"
#include "escCalibration.h"
#include "evr.h"
#include "firstOrderFilter.h"
#include "flightCommand.h"
#include "geoMagElements.h"
#include "GeomagnetismHeader.h"
#include "gps.h"
#include "gyroCalibration.h"
#include "magCalibration.h"
#include "mavlinkStrings.h"
#include "motors.h"
#include "mpu6000Calibration.h"
#include "utilities.h"
#include "watchdogs.h"

///////////////////////////////////////////////////////////////////////////////
