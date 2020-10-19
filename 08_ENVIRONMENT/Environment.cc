// ----------------------------------------------------------------------------
// OpenSDSE - HLA Compliant Distributed Aircraft Simulation
// Copyright (C) 2017  ISAE
//
// This program is free software ; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation ; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY ; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program ; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
// Note: A lot of concepts and code in this file has been borrowed from:
// - JsBSim: http://jsbsim.sourceforge.net/
// - Flightgear: http://www.flightgear.org/
// ----------------------------------------------------------------------------
//

#include "Environment.hh"
#include "Common.hh"
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>


// ----------------------------------------------------------------------------
// Probability of Excedance Table
const double Environment::POEtable[8][12]= 
{
	{ 500.0 , 1750.0 , 3750.0 , 7500.0 , 15000.0 , 25000.0 , 35000.0 , 45000.0 , 55000.0 , 65000.0 , 75000.0 , 80000.0 },
	{   3.2 ,    2.2 ,    1.5 ,    0.0 ,     0.0 ,     0.0 ,     0.0 ,     0.0 ,     0.0 ,     0.0 ,     0.0 ,     0.0 },
	{   4.2 ,    3.6 ,    3.3 ,    1.6 ,     0.0 ,     0.0 ,     0.0 ,     0.0 ,     0.0 ,     0.0 ,     0.0 ,     0.0 },
	{   6.6 ,    6.9 ,    7.4 ,    6.7 ,     4.6 ,     2.7 ,     0.4 ,     0.0 ,     0.0 ,     0.0 ,     0.0 ,     0.0 },
	{   8.6 ,    9.6 ,   10.6 ,   10.1 ,     8.0 ,     6.6 ,     5.0 ,     4.2 ,     2.7 ,     0.0 ,     0.0 ,     0.0 },
	{  11.8 ,   13.0 ,   16.0 ,   15.1 ,    11.6 ,     9.7 ,     8.1 ,     8.2 ,     7.9 ,     4.9 ,     3.2 ,     2.1 },
	{  15.6 ,   17.6 ,   23.0 ,   23.6 ,    22.1 ,    20.0 ,    16.0 ,    15.1 ,    12.1 ,     7.9 ,     6.2 ,     5.1 },
	{  18.7 ,   21.5 ,   28.4 ,   30.2 ,    30.7 ,    31.0 ,    25.2 ,    23.1 ,    17.5 ,    10.7 ,     8.4 ,     7.2 }
}; 

// ----------------------------------------------------------------------------
// Constructor 
Environment::Environment() 
{
  srand(time(0));

// Set up access path for xml attributes
  XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);
  vector<string> attribute_path(3);
  attribute_path[0] = "ENVIRONMENT";
  attribute_path[1] = "Wind";

// Atmosphere 
  lastIndex = 0;
  h = 0.0;
  htab[0] = 0.0;
  htab[1] = 11000.0;
  htab[2] = 20000.0;
  htab[3] = 32000.0;
  htab[4] = 47000.0;
  htab[5] = 51000.0;
  htab[6] = 71000.0;
  htab[7] = 84852.0; // m.
  computeStdAtmosphere(h);  
  SLsoundspeed  = sqrt(SHRatio*Reng*SLtemperature); // m/s

//  Turbulence
  
  b_w = 111.3; // wingspan of the aircraft in ft
  dt  = 0.1;   // period of the turbulence
  trueAirSpeed = 250.0; // in m/s

  mNEDtoBody.InitMatrix();
  mWindtoNED.InitMatrix();

  attribute_path[2] = "WindSpeed";
  windSpeed = getDoubleValue(params, attribute_path);// in m/s
  
  attribute_path[2] = "WindPsi";
  windPsi = getDoubleValue(params, attribute_path);// in degres
  windPsi = windPsi*degtorad;// in radians

  attribute_path[2] = "TurbulenceType";
  turbType = static_cast<tType>(getIntValue(params, attribute_path));

  iterations = 10;
  delta_T = dt/iterations; // Turbulence Integration Step !

// VonKarman Turbulence Filter State Initialization

  ssVonKarmanAtAltitudeft.H_u[0] = ssVonKarmanAtAltitudeft.H_u[1] = 0.0;
  ssVonKarmanAtAltitudeft.H_v[0] = ssVonKarmanAtAltitudeft.H_v[1] = ssVonKarmanAtAltitudeft.H_v[2] = 0.0;
  ssVonKarmanAtAltitudeft.H_w[0] = ssVonKarmanAtAltitudeft.H_w[1] = ssVonKarmanAtAltitudeft.H_w[2] = 0.0; 
  ssVonKarmanAtAltitudeft.H_p[0] = 0.0;
  ssVonKarmanAtAltitudeft.H_q[0] = 0.0;
  ssVonKarmanAtAltitudeft.H_r[0] = 0.0;
  for (int i=0;i<6;i++) {ssVonKarmanAtAltitudeft.H_Output[i] = 0.0;}

  ssVonKarmanAt1000ft = ssVonKarmanAtAltitudeft;
  ssVonKarmanAt2000ft = ssVonKarmanAtAltitudeft;
  ssVonKarmanActual   = &ssVonKarmanAt1000ft; 

  // Dryden Turbulence Filter State

  ssDrydenAtAltitudeft.H_u[0] = 0.0;
  ssDrydenAtAltitudeft.H_v[0] = ssDrydenAtAltitudeft.H_v[1] = 0.0;
  ssDrydenAtAltitudeft.H_w[0] = ssDrydenAtAltitudeft.H_w[1] = 0.0; 
  ssDrydenAtAltitudeft.H_p[0] = 0.0;
  ssDrydenAtAltitudeft.H_q[0] = 0.0;
  ssDrydenAtAltitudeft.H_r[0] = 0.0;
  for (int i=0;i<6;i++) {ssDrydenAtAltitudeft.H_Output[i] = 0.0;}

  ssDrydenAt1000ft = ssDrydenAtAltitudeft;
  ssDrydenAt2000ft = ssDrydenAtAltitudeft;
  ssDrydenActual   = &ssDrydenAt1000ft; 

// Milspec turbulence model

  attribute_path[2] = "W20";
  W20 = getDoubleValue(params , attribute_path);// in m/s
  W20ft = W20*mtoft; // in ft/s

// Milspec probability

  attribute_path[2] = "POEindex";
  POEindex = getIntValue(params , attribute_path);

// cleanup XMLDocument
  delete params;

// Failure mode
    _FailureTemperature=0;
    _FailureDensity=0;
    _FailurePressure=0;
    _FailureSoudSpeed=0;
    _FailureTWB=0;
    _FailureTWRB=0;

}

// ----------------------------------------------------------------------------
// Destructor 
Environment::~Environment()
{

}

// ----------------------------------------------------------------------------
// Calculate the atmosphere for the given altitude, including effects of temperature deviation.
// The result of calculations is stored in the intTemperature, intDensity, 
// intPressure and intSoundSpeed variables, these variables are not available
// via get functions (intended behavior).
void Environment::calculate(double altitude)
{
  double slope, reftemp, refpress;
  int i = lastIndex;

  if (altitude < htab[lastIndex]) {
    if (altitude <= 0) {
      i = 0;
      altitude=0.0;
    } else {
       i = lastIndex-1;
       while (htab[i] > altitude) i--;
    }
  } else if (altitude > htab[lastIndex+1]) {
    if (altitude >= htab[7]) {
      i = 7;
      altitude = htab[7];
    } else {
      i = lastIndex+1;
      while (htab[i+1] < altitude) i++;
    }
  }

  switch(i) {
  case 0:     // Sea level
    slope     = -0.0065;  // kelvin/m
    reftemp   = 288.15;   // kelvin or 518.67 degrees Rankine
    refpress  = 101325.0; // Pa
    //refdens   = 1.225;    // kg/m^3
    break;
  case 1:     // 11 km or 36089 ft 
    slope     = 0.0;      // kelvin/m
    reftemp   = 216.65;   // kelvin or 389.97 degrees Rankine
    refpress  = 22632.1;  // Pa
    //refdens   = 0.36391;  // kg/m^3
    break;
  case 2:     // 20 km or 65616 ft
    slope     = 0.001;    // kelvin/m
    reftemp   = 216.65;   // kelvin or 389.97 degrees Rankine
    refpress  = 5474.89;  // Pa
    //refdens   = 8.8033E-2;  // kg/m^3
    break;
  case 3:     // 32 km or 104986 ft
    slope     = 0.0028;   // kelvin/m
    reftemp   = 228.65;   // kelvin or 411.57 degrees Rankine
    refpress  = 868.019;  // Pa
    //refdens   = 1.3225E-2;  // kg/m^3
    break;
  case 4:     // 47 km or 154199 ft 
    slope     = 0.0;      // kelvin/m
    reftemp   = 270.65;   // kelvin or 487.17 degrees Rankine
    refpress  = 110.906;  // Pa
    //refdens   = 1.4275E-3;  // kg/m^3
    break;
  case 5:     // 51 km or 167322 ft
    slope     = -0.0028;  // kelvin/m
    reftemp   = 270.65;   // kelvin or 487.17 degrees Rankine
    refpress  = 66.9389;  // Pa
    //refdens   = 8.6159E-4;  // kg/m^3
    break;
  case 6:     // 71 km or 232940 ft 
    slope     = -0.002;   // kelvin/m
    reftemp   = 214.65;   // kelvin or 386.368 degrees Rankine
    refpress  = 3.95642;  // Pa
    //refdens   = 6.421E-5;  // kg/m^3
    break;
  case 7:     // 84.852 km or 278385 ft 
    slope     = 0.0;      // kelvin/m
    reftemp   = 186.94;   // kelvin or 336.5 degrees  Rankine,  
    refpress  = 0.3734;   // Pa
    //refdens   = 6.9589E-6;  // kg/m^3
    break;
 default:
    break;
  }

  if (slope == 0.0) {
    intTemperature = reftemp;
    intPressure    = refpress*exp(-SLgravity/(reftemp*Reng)*(altitude-htab[i]));
    intDensity     = intPressure/(Reng*intTemperature);
    intSoundSpeed  = sqrt(SHRatio*Reng*intTemperature);

  } else {
    intTemperature = reftemp+slope*(altitude-htab[i]);
    intPressure    = refpress*pow(intTemperature/reftemp,-SLgravity/(slope*Reng));
    intDensity     = intPressure/(Reng*intTemperature);
    intSoundSpeed  = sqrt(SHRatio*Reng*intTemperature); 
  }
  lastIndex=i;
}

// ----------------------------------------------------------------------------
// Get the standard atmospheric properties at a specified altitude
void Environment::calculate_output() 
{
	calculate(mAltitude);
	computeWind(trueAirSpeed, mAltitude, vEulerAngles);

	if (_FailureTemperature==0)   {atmosphere.Temperature = intTemperature;}
	if (_FailurePressure==0)      {atmosphere.Pressure    = intPressure;}
	if (_FailureDensity==0)       {atmosphere.Density     = intDensity;}
	if (_FailureSoudSpeed==0)     {atmosphere.SoundSpeed  = intSoundSpeed;}
}

// ----------------------------------------------------------------------------
// Compute T, P and rho for a standard atmosphere at the given altitude.
// This function is called in T, P and rho get functions.
void Environment::computeStdAtmosphere(double altitude)
{
	calculate(altitude);

	atmosphere.Temperature = intTemperature;
	atmosphere.Pressure    = intPressure;
	atmosphere.Density     = intDensity;
	atmosphere.SoundSpeed  = intSoundSpeed;
}

// ----------------------------------------------------------------------------
// Get the standard pressure at a specified altitude
double Environment::getPressure(double altitude) 
{
	if (h != altitude) 
	{
		h = altitude;
		computeStdAtmosphere(h);
	}
	return atmosphere.Pressure;
}

// ----------------------------------------------------------------------------
// Get the standard temperature at a specified altitude
double Environment::getTemperature(double altitude) 
{
	if (h != altitude) 
	{
		h = altitude;
		computeStdAtmosphere(h);
	}
	return atmosphere.Temperature;
}

// ----------------------------------------------------------------------------
// Get the standard density at a specified altitude
double Environment::getDensity(double altitude)
{
	if (h != altitude) 
	{
		h = altitude;
		computeStdAtmosphere(h);
	}
	return atmosphere.Density;
}

// ----------------------------------------------------------------------------
// Get the sound speed at a specified altitude
double Environment::getSoundSpeed(double altitude) 
{
	if (h != altitude) 
	{
		h = altitude;
		computeStdAtmosphere(h);
	}
	return atmosphere.SoundSpeed;
}

// ----------------------------------------------------------------------------
// Get the wind components
void Environment::computeWind(double trueAirSpeed, double altitude, FGColumnVector3 eulerAngles) 
{
	h = altitude;
	V = trueAirSpeed * mtoft; // in ft/s 

	double phi   = eulerAngles(ePhi);
	double theta = eulerAngles(eTheta);
	double psi   = eulerAngles(ePsi);

	double cphi   = cos(phi);
	double sphi   = sin(phi);
	double ctheta = cos(theta);
	double stheta = sin(theta);
	double cpsi   = cos(psi);
	double spsi   = sin(psi);

	mNEDtoBody(1,1) =  cpsi*ctheta;
	mNEDtoBody(1,2) =  spsi*ctheta;
	mNEDtoBody(1,3) = -stheta;
	mNEDtoBody(2,1) =  cpsi*stheta*sphi - spsi*cphi;
	mNEDtoBody(2,2) =  spsi*stheta*sphi + cpsi*cphi;
	mNEDtoBody(2,3) =  ctheta*sphi;
	mNEDtoBody(3,1) =  cpsi*stheta*cphi + spsi*sphi;
	mNEDtoBody(3,2) =  spsi*stheta*cphi - cpsi*sphi;
	mNEDtoBody(3,3) =  ctheta*cphi;

	double cwindPsi = cos(windPsi);
	double swindPsi = sin(windPsi);

	mWindtoNED(1,1) =  cwindPsi;
	mWindtoNED(1,2) =  swindPsi;
	mWindtoNED(2,1) = -swindPsi;
	mWindtoNED(2,2) =  cwindPsi;
	mWindtoNED(3,3) =  1.0; 

	horizontalWind();
	//windGust();
	windShear();
	turbulence();

	cout << "vHorizontalWindBody : " << vHorizontalWindBody << endl;
	cout << "vWindGustBody : " << vWindGustBody << endl;
	cout << "vWindShearBody : " << vWindShearBody << endl;
	cout << "vWindTurbulenceBody : " << vWindTurbulenceBody << endl;
	cout << "vTotalWindRateBody : " << vTotalWindRateBody << endl;

	for (int i=0;i<3;i++) 
	{
		vTotalWindBody(i+1) = vHorizontalWindBody(i+1)
						  + vWindGustBody(i+1)
						  + vWindShearBody(i+1)
						  + vWindTurbulenceBody(i+1);
		vTotalWindRateBody(i+1) = vWindTurbulenceRateBody(i+1);   
	}
}

// ----------------------------------------------------------------------------
// Calculate the horizontal wind components
void Environment::horizontalWind(void) 
{
	FGColumnVector3 vHorizontalWindNED; 
	vHorizontalWindNED(eNorth) = -windSpeed * cos(windPsi);
	vHorizontalWindNED(eEast)  = -windSpeed * sin(windPsi);
	vHorizontalWindNED(eDown)  = 0.0;
	vHorizontalWindBody = mNEDtoBody*vHorizontalWindNED;  
}

// ----------------------------------------------------------------------------
// Calculate a discrete wind gust

void Environment::windGust(void) 
{
	//FGColumnVector3 vWindGustNED; 
	//    vWindGustNED(eNorth) = 0;
	//   vWindGustNED(eEast)  = 0;
	//   vWindGustNED(eDown)  = 0;
	//   vWindGustBody = mNEDtoBody*vWindGustNED; 
}

// ----------------------------------------------------------------------------
// Calculate a wind shear
void Environment::windShear(void) 
{
	double altitudeft = h*mtoft;
	double windShearSpeed; 
	double z0 = 0.15;
	FGColumnVector3 vWindShearNED; 

	if (altitudeft > 1000)   {altitudeft = 1000;}
	else if (altitudeft < 3) {altitudeft = 3;}

	windShearSpeed = W20*log(altitudeft/z0)/log(20.0/z0); // W20 in m/s
	vWindShearNED(eNorth) = -windShearSpeed * cos(windPsi);
	vWindShearNED(eEast)  = -windShearSpeed * sin(windPsi);
	vWindShearNED(eDown)  = 0.0;
	vWindShearBody = mNEDtoBody*vWindShearNED;  
}

// ----------------------------------------------------------------------------
//
void Environment::turbulence(void)
{

	FGColumnVector3 vWindTurbulenceAt1000ftWind;
	FGColumnVector3 vWindTurbulenceRateAt1000ftWind;
	FGColumnVector3 vWindTurbulenceAt2000ftBody;
	FGColumnVector3 vWindTurbulenceRateAt2000ftBody;
	FGColumnVector3 vWindTurbulence;
	FGColumnVector3 vWindTurbulenceRate;

	nu_u = sqrt(M_PI/dt)*GaussianRandomNumber();
	nu_v = sqrt(M_PI/dt)*GaussianRandomNumber();
	nu_w = sqrt(M_PI/dt)*GaussianRandomNumber();
	nu_p = sqrt(M_PI/dt)*GaussianRandomNumber();

 switch (turbType) {

    case ttDryden:{

      // As in MIL-F-8785C (MIL-HDBK-1797 is actually similar, except for the definitions of some constants
      // But it boils down to the same thing)

      // PROBLEME SUR L ALTITUDE A CONSIDERER

      double altitudeft;

      // sqrt(M_PI)/sqrt(0.1)
      // Calculus at 1000 ft and 2000 ft for interpolation in case of 1000 < altitudeft < 2000
      // Actually needed as inspired by concerned Simulink Aerospace Block

      //%%%%%% Calculus at 1000 ft %%%%%%
      ssDrydenActual = &ssDrydenAt1000ft; // Pointing now on Von Karman State-Space at 1000 ft
      altitudeft = 1000.0;
      L_u = L_v = altitudeft/pow(0.177 + 0.000823*altitudeft,1.2);
      L_w = altitudeft;
      sigma_u = sigma_v = 0.1*W20ft/pow(0.177 + 0.000823*altitudeft,0.4);	
      sigma_w = 0.1*W20ft;  
      
      for (int i=0;i<iterations;i++){
        DrydenFilters();
      }

      //%%%%%% Calculus at 2000 ft %%%%%%%

      ssDrydenActual = &ssDrydenAt2000ft; // Pointing now on Von Karman State-Space at 2000 ft
      L_u = L_v = L_w = 1750.0;
      sigma_u = sigma_v = sigma_w = getPOEtableValue(2000.0);
 
      for (int i=0;i<iterations;i++){
        DrydenFilters();
      }

      //%%%%%% Clip Altitude at 10 ft %%%%%%

      altitudeft = h*mtoft;
   
      if (altitudeft <= 10.0) altitudeft = 10.0;

      // Definitions of L_u, L_v, L_w, sigma_u, sigma_v, sigma_w depending on the altitude	

      if (altitudeft <= 1000.0)
       {
        ssDrydenActual = &ssDrydenAtAltitudeft; // Pointing now on Von Karman State-Space at altitudeft
       	L_u = L_v = altitudeft/pow(0.177+0.000823*altitudeft,1.2);
       	L_w = altitudeft;
       	sigma_u = sigma_v = 0.1*W20ft/pow(0.177 + 0.000823*altitudeft,0.4);	
       	sigma_w = 0.1*W20ft;

        for (int i=0;i<iterations;i++){
          DrydenFilters();
        } 

        for (int i=0;i<3;i++) {
          vWindTurbulence(i+1)     = ssDrydenAtAltitudeft.H_Output[i];
          vWindTurbulenceRate(i+1) = ssDrydenAtAltitudeft.H_Output[i+3];
        }
        
        vWindTurbulenceBody     = mNEDtoBody * mWindtoNED * vWindTurbulence;
        vWindTurbulenceRateBody = mNEDtoBody * mWindtoNED * vWindTurbulenceRate;

       }
      else if (altitudeft >= 2000.0)
       {
        ssDrydenActual = &ssDrydenAtAltitudeft; // Pointing now on Von Karman State-Space at altitudeft
       	L_u = L_v = L_w = 1750.0;
        sigma_u = sigma_v = sigma_w = getPOEtableValue(altitudeft);

        for (int i=0;i<iterations;i++){
          DrydenFilters();
        }

        for (int i=0;i<3;i++) {
          vWindTurbulence(i+1)     = ssDrydenAtAltitudeft.H_Output[i];
          vWindTurbulenceRate(i+1) = ssDrydenAtAltitudeft.H_Output[i+3];
        }
        
        vWindTurbulenceBody     = vWindTurbulence;
        vWindTurbulenceRateBody = vWindTurbulenceRate;
       
       }       
      else
       {

        for (int i=0;i<3;i++) {
          vWindTurbulenceAt1000ftWind(i+1)     = ssDrydenAt1000ft.H_Output[i];
          vWindTurbulenceRateAt1000ftWind(i+1) = ssDrydenAt1000ft.H_Output[i+3];
          vWindTurbulenceAt2000ftBody(i+1)     = ssDrydenAt2000ft.H_Output[i];;
          vWindTurbulenceRateAt2000ftBody(i+1) = ssDrydenAt2000ft.H_Output[i+3];;
        }

        // Interpolation lineaire des modeles à 1000 ft et à 2000 ft apres changement de base du modele a 1000 ft.
        double lambda = (altitudeft-1000.0)/1000.0;

        vWindTurbulenceBody     = (1 - lambda) * (mNEDtoBody * mWindtoNED * vWindTurbulenceAt1000ftWind) + lambda * vWindTurbulenceAt2000ftBody;
        vWindTurbulenceRateBody = (1 - lambda) * (mNEDtoBody * vWindTurbulenceRateAt1000ftWind) + lambda * vWindTurbulenceRateAt2000ftBody;
       
       } 
     break;
    }

    case ttVonKarman:{

  // As in MIL-F-8785C (MIL-HDBK-1797 is actually similar, except for the definitions of some constants
  // But it boils down to the same thing)

  // PROBLEME SUR L ALTITUDE A CONSIDERER

   double altitudeft;

  // sqrt(M_PI)/sqrt(0.1)

  // Calculus at 1000 ft and 2000 ft for interpolation in case of 1000 < altitudeft < 2000
  // Actually needed as inspired by concerned Simulink Aerospace Block

  //%%%%%% Calculus at 1000 ft %%%%%%
        
      ssVonKarmanActual = &ssVonKarmanAt1000ft; // Pointing now on Von Karman State-Space at 1000 ft
      altitudeft = 1000.0;
      L_u = L_v = altitudeft/pow(0.177+0.000823*altitudeft,1.2);
      L_w = altitudeft;
      sigma_u = sigma_v = 0.1*W20ft/pow(0.177 + 0.000823*altitudeft,0.4);	
      sigma_w = 0.1*W20ft;  
      
      for (int i=0;i<iterations;i++){
        VonKarmanFilters();
      }

  //%%%%%% Calculus at 2000 ft %%%%%%%
        
      ssVonKarmanActual = &ssVonKarmanAt2000ft; // Pointing now on Von Karman State-Space at 2000 ft
      L_u = L_v = L_w = 2500.0;
      sigma_u = sigma_v = sigma_w = getPOEtableValue(2000.0);
 
      for (int i=0;i<iterations;i++){
        VonKarmanFilters();
      }

  //%%%%%% Clip Altitude at 10 ft %%%%%%

      altitudeft = h*mtoft;
      
      if (altitudeft <= 10.0) altitudeft = 10.0;

  // Definitions of L_u, L_v, L_w, sigma_u, sigma_v, sigma_w depending on the altitude	

      if (altitudeft <= 1000.0)
       {
        ssVonKarmanActual = &ssVonKarmanAtAltitudeft; // Pointing now on Von Karman State-Space at altitudeft
       	L_u = L_v = altitudeft/pow(0.177+0.000823*altitudeft,1.2);
       	L_w = altitudeft;
       	sigma_u = sigma_v = 0.1*W20ft/pow(0.177 + 0.000823*altitudeft,0.4);	
       	sigma_w = 0.1*W20ft;

        for (int i=0;i<iterations;i++){
          VonKarmanFilters();
        } 

        for (int i=0;i<3;i++) {
          vWindTurbulence(i+1)     = ssVonKarmanAtAltitudeft.H_Output[i];
          vWindTurbulenceRate(i+1) = ssVonKarmanAtAltitudeft.H_Output[i+3];
        }
        
        vWindTurbulenceBody     = mNEDtoBody * mWindtoNED * vWindTurbulence;
        vWindTurbulenceRateBody = mNEDtoBody * mWindtoNED * vWindTurbulenceRate;

       }
      else if (altitudeft >= 2000.0)
       {

        ssVonKarmanActual = &ssVonKarmanAtAltitudeft; // Pointing now on Von Karman State-Space at altitudeft
       	L_u = L_v = L_w = 2500.0;
        sigma_u = sigma_v = sigma_w = getPOEtableValue(altitudeft);

        for (int i=0;i<iterations;i++){
          VonKarmanFilters();
        }

        for (int i=0;i<3;i++) {
          vWindTurbulence(i+1)     = ssVonKarmanAtAltitudeft.H_Output[i];
          vWindTurbulenceRate(i+1) = ssVonKarmanAtAltitudeft.H_Output[i+3];
        }
        
        vWindTurbulenceBody     = vWindTurbulence;
        vWindTurbulenceRateBody = vWindTurbulenceRate;

       }
      else
       {
 
       for (int i=0;i<3;i++) {
          vWindTurbulenceAt1000ftWind(i+1)     = ssVonKarmanAt1000ft.H_Output[i];
          vWindTurbulenceRateAt1000ftWind(i+1) = ssVonKarmanAt1000ft.H_Output[i+3];
          vWindTurbulenceAt2000ftBody(i+1)     = ssVonKarmanAt2000ft.H_Output[i];;
          vWindTurbulenceRateAt2000ftBody(i+1) = ssVonKarmanAt2000ft.H_Output[i+3];;
        }

        // Interpolation lineaire des modeles à 1000 ft et à 2000 ft apres changement de base du modele a 1000 ft.
        double lambda = (altitudeft-1000.0)/1000.0;

        vWindTurbulenceBody     = (1 - lambda) * (mNEDtoBody * mWindtoNED * vWindTurbulenceAt1000ftWind)     + lambda * vWindTurbulenceAt2000ftBody;
        vWindTurbulenceRateBody = (1 - lambda) * (mNEDtoBody * mWindtoNED * vWindTurbulenceRateAt1000ftWind) + lambda * vWindTurbulenceRateAt2000ftBody;    
      } 
    break;
  }
  default:
    break;
  }
}

// ----------------------------------------------------------------------------
// Dryden Filters
void Environment::DrydenFilters(void) 
{
        
  double tempdot[3];                     // Turbulence Temporary Derivative States
  double tempoutput[2];                  // Temporary variable for output of H_v and H_w
  ssDryden* ssD = ssDrydenActual;        // Temporary Pointer to Von Karman State-Space Description (shorter name, that's all!)

	// ---------------------------
	// H_uDryden(s)

	// State Derivatives 		
       	tempdot[0] = (-ssD->H_u[0] + nu_u)/(L_u/V);

	// Next Iterate
       	ssD->H_u[0] += tempdot[0]*delta_T;

	// Output
       	ssD->H_Output[0] = sigma_u*sqrt(2.0*L_u/M_PI/V)*(ssD->H_u[0]);

	// ---------------------------
	// H_vDryden(s)

	// State Derivatives
       	tempdot[0] = ssD->H_v[1];
       	tempdot[1] = (-ssD->H_v[0] - 2.0*L_v/V*ssD->H_v[1] + nu_v)/(pow(L_v/V,2.0));

	// Next Iterate
	for (int i=0;i<2;i++) {ssD->H_v[i] += tempdot[i]*delta_T;}

	// Output
	tempoutput[0] = ssD->H_Output[1]; // We store the former output of H_v
       	ssD->H_Output[1] = sigma_v*sqrt(L_v/M_PI/V)*(ssD->H_v[0]+sqrt(3.0)*L_v/V*ssD->H_v[1]);

	// ---------------------------
	// H_wDryden(s)

	// State Derivatives
	tempdot[0] = ssD->H_w[1];
	tempdot[1] = (-ssD->H_w[0] - 2.0*L_w/V*ssD->H_w[1] + nu_w)/(pow(L_w/V,2.0));

       	// Next Iterate
	for (int i=0;i<2;i++) {ssD->H_w[i] += tempdot[i]*delta_T;}

	// Output
        
	tempoutput[1] = ssD->H_Output[2]; // We store the former output of H_w
       
       	ssD->H_Output[2] = sigma_w*sqrt(L_w/M_PI/V)*(ssD->H_w[0]+sqrt(3.0)*L_w/V*ssD->H_w[1]);
	
	// ---------------------------
	// H_pDryden(s)
      
	// State Derivatives
	tempdot[0] = (-ssD->H_p[0] +nu_p)/(4.0*b_w/M_PI/V);

	// Next Iterate
       	ssD->H_p[0] += tempdot[0]*delta_T;

	// Output
       	ssD->H_Output[3] = sigma_w*sqrt(0.8/V)*pow(M_PI/4.0/b_w,1.0/6.0)/pow(L_w,1.0/3.0)*ssD->H_p[0];

	// ---------------------------
	// H_qDryden(s)
     
	// State Derivatives
       	tempdot[0] =  (-ssD->H_q[0] + tempoutput[1])/(4.0*b_w/M_PI/V);

	// Next Iterate
	ssD->H_q[0] += tempdot[0]*delta_T;

	// Output
	ssD->H_Output[4] = 1.0/(4.0*b_w/M_PI)*(-ssD->H_q[0] + tempoutput[1]);	

	// ---------------------------
	// H_rDryden(s)
      
	// State Derivatives
       	tempdot[0] =  (-ssD->H_r[0] + tempoutput[0])/(3.0*b_w/M_PI/V);

	// Next Iterate
	ssD->H_r[0] += tempdot[0]*delta_T;

	// Output
	ssD->H_Output[5] = 1.0/(3.0*b_w/M_PI)*(-ssD->H_r[0] + tempoutput[0]);

}

// ----------------------------------------------------------------------------
// Von Karman Filters
void Environment::VonKarmanFilters(void) 
{
        
 double tempdot[3];                     // Turbulence Temporary Derivative States
 double tempoutput[2];                  // Temporary variable for output of H_v and H_w
 ssVonKarman* ssVK = ssVonKarmanActual; // Temporary Pointer to Von Karman State-Space Description (shorter name, that's all!)

	// ---------------------------
	// H_uVonKarman(s)

	// State Derivatives
       	tempdot[0] = ssVK->H_u[1]; 		
       	tempdot[1] = (-ssVK->H_u[0] - 1.357*L_u/V*ssVK->H_u[1]+nu_u)/(0.1987*pow(L_u/V,2.0));

	// Next Iterate
       	for (int i=0;i<2;i++) {ssVK->H_u[i] += tempdot[i]*delta_T;}

	// Output
       	ssVK->H_Output[0] = sigma_u*sqrt(2.0*L_u/M_PI/V)*(ssVK->H_u[0]+0.25*L_u/V*ssVK->H_u[1]);

	// ---------------------------
	// H_vVonKarman(s)

	// State Derivatives
       	tempdot[0] = ssVK->H_v[1];
	tempdot[1] = ssVK->H_v[2];
       	tempdot[2] = (-ssVK->H_v[0] - 2.9958*L_v/V*ssVK->H_v[1]-1.9754*pow(L_v/V,2.0)*ssVK->H_v[2]+nu_v)/(0.1539*pow(L_v/V,3.0));

	// Next Iterate
	for (int i=0;i<3;i++) {ssVK->H_v[i] += tempdot[i]*delta_T;}

	// Output
	tempoutput[0] = ssVK->H_Output[1]; // We store the former output of H_v
       	ssVK->H_Output[1] = sigma_v*sqrt(L_v/M_PI/V)*(ssVK->H_v[0]+2.7478*L_v/V*ssVK->H_v[1]+0.3398*pow(L_v/V,2.0)*ssVK->H_v[2]);

	// ---------------------------
	// H_wVonKarman(s)

	// State Derivatives
	tempdot[0] = ssVK->H_w[1];
	tempdot[1] = ssVK->H_w[2];
	tempdot[2] = (-ssVK->H_w[0] - 2.9958*L_w/V*ssVK->H_w[1]-1.9754*pow(L_w/V,2.0)*ssVK->H_w[2]+nu_w)/(0.1539*pow(L_w/V,3.0));

       	// Next Iterate
	for (int i=0;i<3;i++) {ssVK->H_w[i] += tempdot[i]*delta_T;}

	// Output
	tempoutput[1] = ssVK->H_Output[2]; // We store the former output of H_w
       	ssVK->H_Output[2] = sigma_w*sqrt(L_w/M_PI/V)*(ssVK->H_w[0]+2.7478*L_w/V*ssVK->H_w[1]+0.3398*pow(L_w/V,2.0)*ssVK->H_w[2]);
	
	// ---------------------------
	// H_pVonKarman(s)
      
	// State Derivatives
	tempdot[0] = (-ssVK->H_p[0] +nu_p)/(4.0*b_w/M_PI/V);

	// Next Iterate
       	ssVK->H_p[0] += tempdot[0]*delta_T;

	// Output
       	ssVK->H_Output[3] = sigma_w*sqrt(0.8/V)*pow(M_PI/4.0/b_w,1.0/6.0)/pow(L_w,1.0/3.0)*ssVK->H_p[0];

	// ---------------------------
	// H_qVonKarman(s)
     
	// State Derivatives
       	tempdot[0] =  (-ssVK->H_q[0] + tempoutput[1])/(4.0*b_w/M_PI/V);

	// Next Iterate
	ssVK->H_q[0] += tempdot[0]*delta_T;

	// Output
	ssVK->H_Output[4] = 1.0/(4.0*b_w/M_PI)*(-ssVK->H_q[0] + tempoutput[1]);	

	// ---------------------------
	// H_rVonKarman(s)
      
	// State Derivatives
       	tempdot[0] =  (-ssVK->H_r[0] + tempoutput[0])/(3.0*b_w/M_PI/V);

	// Next Iterate
	ssVK->H_r[0] += tempdot[0]*delta_T;

	// Output
	ssVK->H_Output[5] = 1.0/(3.0*b_w/M_PI)*(-ssVK->H_r[0] + tempoutput[0]);

}

// ----------------------------------------------------------------------------
// Get POE Table Value
double Environment::getPOEtableValue(double altitudeft) 
{
	double lambda, value;
	unsigned int c = 0;

	while(c < 12 && POEtable[0][c] < altitudeft) { c++; }

	if (c == 0) 
	{
		value = POEtable[POEindex][c];
	}
	else if (c == 12) 
	{
		value = POEtable[POEindex][c];
	}
	else 
	{
		lambda = (altitudeft - POEtable[0][c-1])/(POEtable[0][c] - POEtable[0][c-1]);
		value  = POEtable[POEindex][c-1] + lambda*(POEtable[POEindex][c] - POEtable[POEindex][c-1]);
	}
	return value;
}
