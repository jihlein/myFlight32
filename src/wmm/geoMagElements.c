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
#include "wmm.h"

///////////////////////////////////////////////////////////////////////////////

MAGtype_CoordGeodetic coordGeodetic;

MAGtype_CoordSpherical coordSpherical;

MAGtype_Date userDate;

MAGtype_Ellipsoid ellipsoid;

MAGtype_GeoMagneticElements geoMagneticElements;

MAGtype_MagneticModel *timedMagneticModel, *magneticModel;

///////////////////////////////////////////////////////////////////////////////

void setupGeoMagWorkspace(void)
{
    uint8_t  index;
    uint16_t nMax = 12;
    uint16_t numTerms;

    ///////////////////////////////////

    numTerms = CALCULATE_NUMTERMS(nMax);

    magneticModel = MAG_AllocateModelMemory(numTerms);

    ///////////////////////////////////

    magneticModel->nMax       = nMax;
	magneticModel->nMaxSecVar = nMax;
    magneticModel->epoch      = 2010.0f;

    strncpy(magneticModel->ModelName, "WMM-2010        11/20/2009\0", sizeof(magneticModel->ModelName));

    magneticModel->Main_Field_Coeff_G[0]  = 0.0f;
    magneticModel->Main_Field_Coeff_H[0]  = 0.0f;
    magneticModel->Secular_Var_Coeff_G[0] = 0.0f;
    magneticModel->Secular_Var_Coeff_H[0] = 0.0f;

    for ( index = 1; index < 91; index++)
    {
		magneticModel->Main_Field_Coeff_G[index]  = wmmCoefficients[index - 1][0];  // gnm
		magneticModel->Main_Field_Coeff_H[index]  = wmmCoefficients[index - 1][1];  // hnm
        magneticModel->Secular_Var_Coeff_G[index] = wmmCoefficients[index - 1][2];  // dgnm
		magneticModel->Secular_Var_Coeff_H[index] = wmmCoefficients[index - 1][3];  // dhnm
	}

	magneticModel->CoefficientFileEndDate = magneticModel->epoch + 5.0f;

	///////////////////////////////////

	numTerms = ((nMax + 1) * (nMax + 2) / 2);

    timedMagneticModel = MAG_AllocateModelMemory(numTerms);

    ///////////////////////////////////

    //  Set WGS-84 parameters
	ellipsoid.a     = 6378.137f;                                                               // semi-major axis of the ellipsoid in km
	ellipsoid.b     = 6356.7523142f;                                                           // semi-minor axis of the ellipsoid in km
	ellipsoid.fla   = 1.0f / 298.257223563f;                                                   // flattening
	ellipsoid.eps   = sqrt(1.0f - (ellipsoid.b * ellipsoid.b) / (ellipsoid.a * ellipsoid.a));  // first eccentricity
	ellipsoid.epssq = (ellipsoid.eps * ellipsoid.eps);                                         // first eccentricity squared
    ellipsoid.re    = 6371.2f;                                                                 // earth's radius in km

    ///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////

void clearGeoMagWorkspace(void)
{
	MAG_FreeMagneticModelMemory(magneticModel);
	MAG_FreeMagneticModelMemory(timedMagneticModel);
}

///////////////////////////////////////////////////////////////////////////////

void computeGeoMagElements(void)
{
	setupGeoMagWorkspace();

	coordGeodetic.lambda               = (float)sensors.gps.longitude * 1E-7f;  // Convert to decimal degrees
	coordGeodetic.phi                  = (float)sensors.gps.latitude  * 1E-7f;  // Convert to decimal degrees
	coordGeodetic.HeightAboveEllipsoid = (float)sensors.gps.height    * 1E-6f;  // Convert from mm to km
	coordGeodetic.UseGeoid             = 0;

	userDate.Year  = sensors.gps.year;
	userDate.Month = sensors.gps.month;
	userDate.Day   = sensors.gps.day;

	//cliPortPrint ("\n");
	//cliPortPrintF("GPS Lat: %7.3f\n", coordGeodetic.phi);
	//cliPortPrintF("GPS Lon: %7.3f\n", coordGeodetic.lambda);
    //cliPortPrintF("GPS Alt: %7.3f\n", coordGeodetic.HeightAboveEllipsoid);
    //cliPortPrint ("\n");

    MAG_DateToYear(&userDate);

	//cliPortPrintF("GPS Mon:     %4ld\n",  userDate.Month);
    //cliPortPrintF("GPS Day:     %4ld\n",  userDate.Day);
    //cliPortPrintF("GPS Yr:      %4ld\n",  userDate.Year);
    //cliPortPrintF("GPS Dec Yr:  %7.3f\n", userDate.DecimalYear);
    //cliPortPrint ("\n");

	MAG_GeodeticToSpherical(ellipsoid, coordGeodetic, &coordSpherical);

	MAG_TimelyModifyMagneticModel(userDate, magneticModel, timedMagneticModel);

	MAG_Geomag(ellipsoid, coordSpherical, coordGeodetic, timedMagneticModel, &geoMagneticElements);

	//cliPortPrintF("X:      %8.1f\n", geoMagneticElements.X);
	//cliPortPrintF("Y:      %8.1f\n", geoMagneticElements.Y);
	//cliPortPrintF("Z:      %8.1f\n", geoMagneticElements.Z);
	//cliPortPrintF("H:      %8.1f\n", geoMagneticElements.H);
	//cliPortPrintF("F:      %8.1f\n", geoMagneticElements.F);
	//cliPortPrintF("INCL:   %8.1f\n", geoMagneticElements.Incl);
	//cliPortPrintF("DECL:   %8.1f\n", geoMagneticElements.Decl);
	//cliPortPrint ("\n");

	clearGeoMagWorkspace();
}

///////////////////////////////////////////////////////////////////////////////
