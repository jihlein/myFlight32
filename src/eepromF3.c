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
// EEPROM Defines
///////////////////////////////////////////////////////////////////////////////

#define WRITE_ENABLE                    0x06
#define WRITE_DISABLE                   0x04
#define READ_STATUS_REGISTER            0x05
#define WRITE_STATUS_REGISTER           0x01
#define READ_DATA                       0x03
#define FAST_READ                       0x0B
#define PAGE_PROGRAM_256_BYTES          0x02
#define SECTOR_ERASE_64KB               0xD8
#define CHIP_ERASE                      0xC7

///////////////////////////////////////////////////////////////////////////////

// EEPROM data stored in 1st 64KB Sector, 0x000000

#define EEPROM_ADDR  0x000000

///////////////////////////////////////////////////////////////////////////////
// EEPROM Variables
///////////////////////////////////////////////////////////////////////////////

static uint8_t checkNewEEPROMConf = 5;

eepromConfig_t eepromConfig;

const char rcChannelLetters[] = "AERT1234";

uint32andUint8_t eepromConfigAddr;

///////////////////////////////////////////////////////////////////////////////
// Parse RC Channels
///////////////////////////////////////////////////////////////////////////////

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++)
    {
        s = strchr(rcChannelLetters, *c);
        if (s)
            eepromConfig.rcMap[s - rcChannelLetters] = c - input;
    }
}

///////////////////////////////////////////////////////////////////////////////
// EEPROM Busy
///////////////////////////////////////////////////////////////////////////////

void eepromBusy(void)
{
	uint8_t busy = 0x01;

	ENABLE_EEPROM;

    spiTransfer(EEPROM_SPI, READ_STATUS_REGISTER);

    while (busy == 0x01)
        busy = spiTransfer(EEPROM_SPI, 0x00) & 0x01;

    DISABLE_EEPROM;

    delayMicroseconds(2);
}

///////////////////////////////////////////////////////////////////////////////
// Write Enable
///////////////////////////////////////////////////////////////////////////////

void writeEnable(void)
{
	ENABLE_EEPROM;

    spiTransfer(EEPROM_SPI, WRITE_ENABLE);

    DISABLE_EEPROM;

    delayMicroseconds(2);
}

///////////////////////////////////////////////////////////////////////////////
// Read EEPROM
///////////////////////////////////////////////////////////////////////////////

void readEEPROM(void)
{
	uint8_t* start = (uint8_t*)(&eepromConfig);
	uint8_t* end   = (uint8_t*)(&eepromConfig + 1);

	///////////////////////////////////

	setSPIdivisor(EEPROM_SPI, 2);  // 18 MHz SPI clock

	eepromConfigAddr.value = EEPROM_ADDR;

	///////////////////////////////////

	ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, eepromConfigAddr.bytes[2]);
	spiTransfer(EEPROM_SPI, eepromConfigAddr.bytes[1]);
	spiTransfer(EEPROM_SPI, eepromConfigAddr.bytes[0]);

	while (start < end)
		*start++ = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

	///////////////////////////////////

    if ( crcCheckVal != crc32B((uint32_t*)(&eepromConfig),       // CRC32B[eepromConfig CRC32B[eepromConfig]]
                               (uint32_t*)(&eepromConfig + 1)))
    {
        evrPush(EVR_FlashCRCFail,0);
        eepromConfig.CRCFlags |= CRC_HistoryBad;
    }
    else if ( eepromConfig.CRCFlags & CRC_HistoryBad )
    {
        evrPush(EVR_ConfigBadHistory,0);
    }

    ///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
// Write EEPROM
///////////////////////////////////////////////////////////////////////////////

uint8_t writeEEPROM(void)
{
    uint16_t byteCount;
    uint8_t  pageIndex;

    uint8_t* start = (uint8_t*)(&eepromConfig);
	uint8_t* end   = (uint8_t*)(&eepromConfig + 1);

    ///////////////////////////////////

	if (eepromConfig.CRCFlags & CRC_HistoryBad)
        evrPush(EVR_ConfigBadHistory,0);

    eepromConfig.CRCAtEnd = crc32B((uint32_t*)(&eepromConfig),                  // CRC32B[eepromConfig]
                                   (uint32_t*)(&eepromConfig.CRCAtEnd));

    ///////////////////////////////////

    setSPIdivisor(EEPROM_SPI, 2);  // 18 MHz SPI clock

    eepromConfigAddr.value = EEPROM_ADDR;

    ///////////////////////////////////

    // Sector Erase

    writeEnable();

    ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, SECTOR_ERASE_64KB);

	spiTransfer(EEPROM_SPI, eepromConfigAddr.bytes[2]);
	spiTransfer(EEPROM_SPI, eepromConfigAddr.bytes[1]);
	spiTransfer(EEPROM_SPI, eepromConfigAddr.bytes[0]);

	DISABLE_EEPROM;

	delayMicroseconds(2);

	eepromBusy();

    ///////////////////////////////////

    // Program Page(s)

	for (pageIndex = 0; pageIndex < ((sizeof(eepromConfig) / 256) + 1); pageIndex++)
    {
        writeEnable();

        ENABLE_EEPROM;

        spiTransfer(EEPROM_SPI, PAGE_PROGRAM_256_BYTES);

		spiTransfer(EEPROM_SPI, eepromConfigAddr.bytes[2]);
		spiTransfer(EEPROM_SPI, eepromConfigAddr.bytes[1]);
		spiTransfer(EEPROM_SPI, eepromConfigAddr.bytes[0]);

	    for (byteCount = 0; byteCount < 256; byteCount++)
	    {
			spiTransfer(EEPROM_SPI, *start++);

			if (start >= end)
			    break;
		}

		DISABLE_EEPROM;

		delayMicroseconds(2);

		eepromBusy();

		eepromConfigAddr.value += 0x0100;
    }

	readEEPROM();

	return 8;  // FLASH_COMPLETE....
}

///////////////////////////////////////////////////////////////////////////////
// Check First Time
///////////////////////////////////////////////////////////////////////////////

void checkFirstTime(bool eepromReset)
{
    uint8_t version;

    eepromConfigAddr.value = EEPROM_ADDR;

    ENABLE_EEPROM;

	spiTransfer(EEPROM_SPI, READ_DATA);

	spiTransfer(EEPROM_SPI, eepromConfigAddr.bytes[2]);
	spiTransfer(EEPROM_SPI, eepromConfigAddr.bytes[1]);
	spiTransfer(EEPROM_SPI, eepromConfigAddr.bytes[0]);

	version = spiTransfer(EEPROM_SPI, 0x00);

	DISABLE_EEPROM;

	delayMicroseconds(2);

    ///////////////////////////////////

    if (eepromReset || version != checkNewEEPROMConf)
    {
		// Default settings
        eepromConfig.version = checkNewEEPROMConf;

	    ///////////////////////////////

        eepromConfig.mpuTempMin = -50.0f;
        eepromConfig.mpuTempMax =  95.0f;

        ///////////////////////////////

        eepromConfig.accelBiasPolynomial[XAXIS * 4 + 0] = 0.0f;
        eepromConfig.accelBiasPolynomial[XAXIS * 4 + 1] = 0.0f;
        eepromConfig.accelBiasPolynomial[XAXIS * 4 + 2] = 0.0f;
        eepromConfig.accelBiasPolynomial[XAXIS * 4 + 3] = 0.0f;

        eepromConfig.accelBiasPolynomial[YAXIS * 4 + 0] = 0.0f;
        eepromConfig.accelBiasPolynomial[YAXIS * 4 + 1] = 0.0f;
        eepromConfig.accelBiasPolynomial[YAXIS * 4 + 2] = 0.0f;
        eepromConfig.accelBiasPolynomial[YAXIS * 4 + 3] = 0.0f;

        eepromConfig.accelBiasPolynomial[ZAXIS * 4 + 0] = 0.0f;
        eepromConfig.accelBiasPolynomial[ZAXIS * 4 + 1] = 0.0f;
        eepromConfig.accelBiasPolynomial[ZAXIS * 4 + 2] = 0.0f;
        eepromConfig.accelBiasPolynomial[ZAXIS * 4 + 3] = 0.0f;

        ///////////////////////////////

        eepromConfig.accelScaleFactorPolynomial[XAXIS * 4 + 0] = 0.0f;
        eepromConfig.accelScaleFactorPolynomial[XAXIS * 4 + 1] = 0.0f;
        eepromConfig.accelScaleFactorPolynomial[XAXIS * 4 + 2] = 0.0f;
        eepromConfig.accelScaleFactorPolynomial[XAXIS * 4 + 3] = ACCEL_SCALE_FACTOR;

        eepromConfig.accelScaleFactorPolynomial[YAXIS * 4 + 0] = 0.0f;
        eepromConfig.accelScaleFactorPolynomial[YAXIS * 4 + 1] = 0.0f;
        eepromConfig.accelScaleFactorPolynomial[YAXIS * 4 + 2] = 0.0f;
        eepromConfig.accelScaleFactorPolynomial[YAXIS * 4 + 3] = ACCEL_SCALE_FACTOR;

        eepromConfig.accelScaleFactorPolynomial[ZAXIS * 4 + 0] = 0.0f;
        eepromConfig.accelScaleFactorPolynomial[ZAXIS * 4 + 1] = 0.0f;
        eepromConfig.accelScaleFactorPolynomial[ZAXIS * 4 + 2] = 0.0f;
        eepromConfig.accelScaleFactorPolynomial[ZAXIS * 4 + 3] = ACCEL_SCALE_FACTOR;

        ///////////////////////////////

        eepromConfig.gyroBiasPolynomial[ROLL  * 4 + 0] = 0.0f;
        eepromConfig.gyroBiasPolynomial[ROLL  * 4 + 1] = 0.0f;
        eepromConfig.gyroBiasPolynomial[ROLL  * 4 + 2] = 0.0f;
        eepromConfig.gyroBiasPolynomial[ROLL  * 4 + 3] = 0.0f;

        eepromConfig.gyroBiasPolynomial[PITCH * 4 + 0] = 0.0f;
        eepromConfig.gyroBiasPolynomial[PITCH * 4 + 1] = 0.0f;
        eepromConfig.gyroBiasPolynomial[PITCH * 4 + 2] = 0.0f;
        eepromConfig.gyroBiasPolynomial[PITCH * 4 + 3] = 0.0f;

        eepromConfig.gyroBiasPolynomial[YAW   * 4 + 0] = 0.0f;
        eepromConfig.gyroBiasPolynomial[YAW   * 4 + 1] = 0.0f;
        eepromConfig.gyroBiasPolynomial[YAW   * 4 + 2] = 0.0f;
        eepromConfig.gyroBiasPolynomial[YAW   * 4 + 3] = 0.0f;

	    ///////////////////////////////

        eepromConfig.magBias[XAXIS] = 0.0f;
	    eepromConfig.magBias[YAXIS] = 0.0f;
	    eepromConfig.magBias[ZAXIS] = 0.0f;

		eepromConfig.magBias[XAXIS + 3] = 0.0f;
	    eepromConfig.magBias[YAXIS + 3] = 0.0f;
	    eepromConfig.magBias[ZAXIS + 3] = 0.0f;

		///////////////////////////////

		eepromConfig.accelCutoff = 0.25f;

		///////////////////////////////

	    eepromConfig.kpAcc = 1.0f;    // proportional gain governs rate of convergence to accelerometer
	    eepromConfig.kpMag = 5.0f;    // proportional gain governs rate of convergence to magnetometer

	    ///////////////////////////////////

	    eepromConfig.attTrim[ROLL ] = 0.0f;
	    eepromConfig.attTrim[PITCH] = 0.0f;

	    ///////////////////////////////////

	    eepromConfig.earthAccel100HzHPtau = 4.0f;

	    ///////////////////////////////////

	    eepromConfig.posEstA = 2.0f;
	    eepromConfig.posEstB = 1.0f;

	    eepromConfig.hEstA = 2.0f;
	    eepromConfig.hEstB = 1.0f;

	    ///////////////////////////////////

	    eepromConfig.dlpfSetting = BITS_DLPF_CFG_98HZ;

		///////////////////////////////////

	    eepromConfig.sensorOrientation        = 1;  // No rotation

	    ///////////////////////////////////

	    eepromConfig.rollAndPitchRateScaling = 100.0f / 180000.0f * PI;  // Stick to rate scaling for 100 DPS
	    eepromConfig.yawRateScaling          = 100.0f / 180000.0f * PI;  // Stick to rate scaling for 100 DPS

        eepromConfig.attitudeScaling         = 60.0f  / 180000.0f * PI;  // Stick to att scaling for 60 degrees

        eepromConfig.nDotEdotScaling         = 0.009f;                   // Stick to nDot/eDot scaling (9 mps)/(1000 RX PWM Steps) = 0.009

        eepromConfig.hDotScaling             = 0.003f;                   // Stick to hDot scaling (3 mps)/(1000 RX PWM Steps) = 0.003

        ///////////////////////////////

	    eepromConfig.receiverType  = SPEKTRUM;

	    eepromConfig.slaveSpektrum = false;

	    parseRcChannels("TAER2134");

	    eepromConfig.escPwmRate   = 450;
        eepromConfig.servoPwmRate = 50;

        eepromConfig.mixerConfiguration = MIXERTYPE_TRI;
        eepromConfig.yawDirection = 1.0f;

        eepromConfig.triYawServoPwmRate = 50;
        eepromConfig.triYawServoMin     = 2000.0f;
        eepromConfig.triYawServoMid     = 3000.0f;
        eepromConfig.triYawServoMax     = 4000.0f;
        eepromConfig.triCopterYawCmd500HzLowPassTau = 0.05f;

        // Free Mix Defaults to Quad X
		eepromConfig.freeMixMotors = 4;

		eepromConfig.freeMix[ 0] =  1.0f;
		eepromConfig.freeMix[ 8] = -1.0f;
		eepromConfig.freeMix[16] = -1.0f;
		eepromConfig.freeMix[24] =  1.0f;

		eepromConfig.freeMix[ 1] = -1.0f;
		eepromConfig.freeMix[ 9] = -1.0f;
		eepromConfig.freeMix[17] =  1.0f;
		eepromConfig.freeMix[25] =  1.0f;

		eepromConfig.freeMix[ 2] = -1.0f;
		eepromConfig.freeMix[10] =  1.0f;
		eepromConfig.freeMix[18] = -1.0f;
		eepromConfig.freeMix[26] =  1.0f;

		eepromConfig.freeMix[ 3] =  1.0f;
		eepromConfig.freeMix[11] =  1.0f;
		eepromConfig.freeMix[19] =  1.0f;
		eepromConfig.freeMix[27] =  1.0f;

		eepromConfig.freeMix[ 4] =  0.0f;
		eepromConfig.freeMix[12] =  0.0f;
		eepromConfig.freeMix[20] =  0.0f;
		eepromConfig.freeMix[28] =  0.0f;

		eepromConfig.freeMix[ 5] =  0.0f;
		eepromConfig.freeMix[13] =  0.0f;
        eepromConfig.freeMix[21] =  0.0f;
        eepromConfig.freeMix[29] =  0.0f;

        eepromConfig.freeMix[ 6] =  0.0f;
        eepromConfig.freeMix[14] =  0.0f;
        eepromConfig.freeMix[22] =  0.0f;
        eepromConfig.freeMix[30] =  0.0f;

        eepromConfig.freeMix[ 7] =  0.0f;
        eepromConfig.freeMix[15] =  0.0f;
        eepromConfig.freeMix[23] =  0.0f;
        eepromConfig.freeMix[31] =  0.0f;

        eepromConfig.rollAttAltCompensationGain   =  1.0f;
        eepromConfig.rollAttAltCompensationLimit  =  0.0f * D2R;

        eepromConfig.pitchAttAltCompensationGain  =  1.0f;
        eepromConfig.pitchAttAltCompensationLimit =  0.0f * D2R;

        eepromConfig.midCommand   = 3000.0f;
        eepromConfig.minCheck     = (float)(MINCOMMAND + 200);
        eepromConfig.maxCheck     = (float)(MAXCOMMAND - 200);
        eepromConfig.minThrottle  = (float)(MINCOMMAND + 200);
        eepromConfig.maxThrottle  = (float)(MAXCOMMAND);

        ///////////////////////////////

        eepromConfig.rateP[ROLL]      =  250.0f;
        eepromConfig.rateI[ROLL]      =  100.0f;
        eepromConfig.rateD[ROLL]      =    0.0f;
        eepromConfig.rateLimit[ROLL]  = 1000.0f * eepromConfig.rollAndPitchRateScaling * eepromConfig.rateP[ROLL];

        eepromConfig.rateP[PITCH]     =  250.0f;
        eepromConfig.rateI[PITCH]     =  100.0f;
        eepromConfig.rateD[PITCH]     =    0.0f;
        eepromConfig.rateLimit[PITCH] = 1000.0f * eepromConfig.rollAndPitchRateScaling * eepromConfig.rateP[PITCH];

        eepromConfig.rateP[YAW]       =  350.0f;
        eepromConfig.rateI[YAW]       =  100.0f;
        eepromConfig.rateD[YAW]       =    0.0f;
        eepromConfig.rateLimit[YAW]   = 1000.0f * eepromConfig.yawRateScaling * eepromConfig.rateP[YAW];

        ///////////////////////////////

        eepromConfig.attitudeP[ROLL]      =    3.0f;
        eepromConfig.attitudeI[ROLL]      =    0.0f;
        eepromConfig.attitudeD[ROLL]      =    0.0f;
        eepromConfig.attitudeLimit[ROLL]  = 1000.0f * eepromConfig.attitudeScaling * eepromConfig.attitudeP[ROLL];

        eepromConfig.attitudeP[PITCH]     =    3.0f;
        eepromConfig.attitudeI[PITCH]     =    0.0f;
        eepromConfig.attitudeD[PITCH]     =    0.0f;
        eepromConfig.attitudeLimit[PITCH] = 1000.0f * eepromConfig.attitudeScaling * eepromConfig.attitudeP[PITCH];

        eepromConfig.attitudeP[YAW]       =    3.0f;
        eepromConfig.attitudeI[YAW]       =    0.0f;
        eepromConfig.attitudeD[YAW]       =    0.0f;
        eepromConfig.attitudeLimit[YAW]   =   90.0f * D2R;

        ///////////////////////////////

        eepromConfig.velocityP[XAXIS]     =    1.0f;
        eepromConfig.velocityI[XAXIS]     =    0.0f;
        eepromConfig.velocityD[XAXIS]     =    0.0f;
        eepromConfig.velocityLimit[XAXIS] = 1000.0f * eepromConfig.nDotEdotScaling * eepromConfig.velocityP[XAXIS];

        eepromConfig.velocityP[YAXIS]     =    1.0f;
        eepromConfig.velocityI[YAXIS]     =    0.0f;
        eepromConfig.velocityD[YAXIS]     =    0.0f;
        eepromConfig.velocityLimit[YAXIS] = 1000.0f * eepromConfig.nDotEdotScaling * eepromConfig.velocityP[YAXIS];

        eepromConfig.velocityP[ZAXIS]     =    1.0f;
        eepromConfig.velocityI[ZAXIS]     =    0.0f;
        eepromConfig.velocityD[ZAXIS]     =    0.0f;
        eepromConfig.velocityLimit[ZAXIS] = 1000.0f * eepromConfig.hDotScaling * eepromConfig.velocityP[ZAXIS];

        ///////////////////////////////

        eepromConfig.positionP[XAXIS]     =    1.0f;
        eepromConfig.positionI[XAXIS]     =    0.0f;
        eepromConfig.positionD[XAXIS]     =    0.0f;
        eepromConfig.positionLimit[XAXIS] =   10.0f;

        eepromConfig.positionP[YAXIS]     =    1.0f;
        eepromConfig.positionI[YAXIS]     =    0.0f;
        eepromConfig.positionD[YAXIS]     =    0.0f;
        eepromConfig.positionLimit[YAXIS] =   10.0f;

        eepromConfig.positionP[ZAXIS]     =    1.0f;
        eepromConfig.positionI[ZAXIS]     =    0.0f;
        eepromConfig.positionD[ZAXIS]     =    0.0f;
        eepromConfig.positionLimit[ZAXIS] =   10.0f;

	    ///////////////////////////////////

        eepromConfig.batteryCells             = 3;
		eepromConfig.voltageMonitorScale      = 11.0f / 1.0f;
		eepromConfig.voltageMonitorBias       = 0.0f;

		eepromConfig.batteryLow               = 3.30f;
        eepromConfig.batteryVeryLow           = 3.20f;
        eepromConfig.batteryMaxLow            = 3.10f;

        eepromConfig.armCount                 = 50;
		eepromConfig.disarmCount              =  0;

		eepromConfig.activeTelemetry          =  0;
		eepromConfig.mavlinkEnabled           =  false;

		eepromConfig.useGPS                   =  false;

		eepromConfig.verticalVelocityHoldOnly =  true;

		eepromConfig.externalHMC5883          =  0;
		eepromConfig.externalMS5611           =  false;

		writeEEPROM();
	}
}

///////////////////////////////////////////////////////////////////////////////
// Init Matlab Filters
///////////////////////////////////////////////////////////////////////////////

void initMatlabFilters(void)
{
    // TAU = Filter Time Constant
    // T   = Filter Sample Time

    // A   = 2 * TAU / T

    // Low Pass:
    // GX1 = 1 / (1 + A)
    // GX2 = 1 / (1 + A)
    // GX3 = (1 - A) / (1 + A)

    // High Pass:
    // GX1 =  A / (1 + A)
    // GX2 = -A / (1 + A)
    // GX3 = (1 - A) / (1 + A)

    ///////////////////////////////////

    float a;

    a = 2.0f * eepromConfig.earthAccel100HzHPtau * 100.0f;

    earthAccelNumHP[0] =  a / (1.0f + a);
	earthAccelNumHP[1] = -a / (1.0f + a);
	earthAccelDenHP[0] = 1.0f;
	earthAccelDenHP[1] = (1.0f - a) / (1.0f + a);

	///////////////////////////////////

	a = 2.0f * eepromConfig.triCopterYawCmd500HzLowPassTau * 500.0f;

	triYawRateNumLP[0] = 1.0f / (1.0f + a);
	triYawRateNumLP[1] = 1.0f / (1.0f + a);
	triYawRateDenLP[0] = 1.0f;
	triYawRateDenLP[1] = (1.0f - a) / (1.0f + a);
}

///////////////////////////////////////////////////////////////////////////////
