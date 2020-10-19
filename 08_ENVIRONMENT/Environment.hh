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

#ifndef __ENVIRONMENT_HH_DEF__
#define __ENVIRONMENT_HH_DEF__

#include <cmath>
#include "UnitConversion.hh"
#include "FGColumnVector3.hh"
#include "FGMatrix33.hh"

// ----------------------------------------------------------------------------
// Handle all the environment calculations involved in the simulator.
// Provides pressure, temperature, density and sound speed at provided altitude
// Allow to simulate wind :
// - Horizontal wind
// - Wind shear
// - Turbulences with two different modelizations:
// - Dryden
// - Von Karman
// TO DO: Implement the wind gust.
// TO DO: Get rid of dead code.
// ----------------------------------------------------------------------------
class Environment : public UnitConversion 
{
	public:

		// Constructor
		Environment();
		// Destructor
		~Environment();

		// Specify the type of the turbulence : None (0), Dryden (1), Von Karman (2)
		enum tType {ttNone, ttDryden, ttVonKarman} turbType;

		void setAltitude(double val) {mAltitude = val;}
		double getAltitude() {return mAltitude;}

		// Returns the standard pressure at a specified altitude
		double getPressure(double altitude);
		double getPressure() {return atmosphere.Pressure;}
		void setPressure(double input) {atmosphere.Pressure=input;}

		// Returns the standard temperature at a specified altitude
		double getTemperature(double altitude);
		double getTemperature() {return atmosphere.Temperature;}
		void setTemperature(double input) {atmosphere.Temperature=input;}

		// Returns the standard density at a specified altitude
		double getDensity(double altitude);
		double getDensity() {return atmosphere.Density;}
		void setDensity(double input) {atmosphere.Density=input;}

		// Returns the speed of sound in ft/sec.
		double getSoundSpeed(double altitude);
		double getSoundSpeed() {return atmosphere.SoundSpeed;}
		void setSoundSpeed(double input) {atmosphere.SoundSpeed=input;}

		// Access the provided component of the total wind in the Body frame
		double getTotalWindBody(int idx)     const {return vTotalWindBody(idx);}
		// Access the provided component of the total wind rate in the Body frame
		double getTotalWindRateBody(int idx) const {return vTotalWindRateBody(idx);}

		void setTotalWindBody(int idx,double input)     {vTotalWindBody(idx)=input;}
		void setTotalWindRateBody(int idx,double input) {vTotalWindRateBody(idx)=input;}

		void   setWindPsiDeg(double dir) {windPsi = dir*degtorad;}        
		void   setWindPsiRad(double dir) {windPsi = dir;}
		double getWindPsiDeg(void) const {return windPsi*radtodeg;}
		double getWindPsiRad(void) const {return windPsi;}
		// Set the wind speed to the provided value in m/s
		void   setWindSpeed(double speed) {windSpeed = speed;} // in m/s
		// Provide the wind speed in m/s
		double getWindSpeed(void) const {return windSpeed;}

		// Set the True Airspeed
		void setTrueAirSpeed(double TAS) {trueAirSpeed = TAS;}


		// Turbulence models available: ttNone, ttDryden, ttVonKarman 
		void   setTurbType(tType tt) {turbType = tt;}
		tType  getTurbType(void) const {return turbType;}

		// Set the Aircraft Euler Angles
		void setPhi(double val)    {vEulerAngles(ePhi) = val;}    
		void setTheta(double val)  {vEulerAngles(eTheta) = val;}    
		void setPsi(double val)    {vEulerAngles(ePsi) = val;}
		void computeWind(double trueAirSpeed, double altitude, FGColumnVector3 eulerAngles);

		// Standard funcitons
		void calculate_state() {} // Empty : doesn't need continuous calculation
		void calculate_output(void);

		void setFailureTemperature(int input)               {_FailureTemperature=input;}
		void setFailureDensity(int input)                   {_FailureDensity=input;}
		void setFailurePressure(int input)                  {_FailurePressure=input;}
		void setFailureSoundSpeed(int input)                {_FailureSoudSpeed=input;}
		void setFailureTotalWindBody(int i,int input)       {_FailureTWB=input;}
		void setFailureTotalWindRateBody(int i,int input)   {_FailureTWRB=input;}

		int _FailureTemperature;
		int _FailureDensity;
		int _FailurePressure;
		int _FailureSoudSpeed;
		int _FailureTWB;
		int _FailureTWRB;

	protected:

		// Contains informations about the atmosphere
		struct atmType 
		{
			double Temperature; 
			double Pressure; 
			double Density; 
			double SoundSpeed;
		};
		int lastIndex;
		double h, mAltitude;
		double htab[8];
		double SLsoundspeed;
		double soundspeed;
		double intTemperature, intDensity, intPressure, intSoundSpeed;
		double SutherlandConstant, Beta, intViscosity, intKinematicViscosity;

		atmType atmosphere;
		double spike, target_time, strength;

		// W20 = Windspeed at 6 meters (20 ft) in m/s 
		// -> 7.72 m/s (25.31 ft/s) or 15 knots for light turbulence 
		// -> 15.43 m/s (50.63 ft/s) or 30 knots for moderate turbulence 
		// -> 23.15 m/s (75.95 ft/s) or 45 knots for severe turbulence 
		double W20, W20ft;

		// Turbulence Parameters

		int iterations;
		double delta_T;
		double L_u, L_v, L_w;
		double sigma_u, sigma_v, sigma_w;
		static const double POEtable[8][12];

		// Strengh of the turbulence :
		// probability of exceedence index 
		// Index 1 : 2E-1            
		// Index 2 : 1E-1            
		// Index 3 : 1E-2 (Light)    
		// Index 4 : 1E-3 (Moderate) 
		// Index 5 : 1E-4            
		// Index 6 : 1E-5 (Severe)   
		// Index 7 : 1E-6 
		unsigned int POEindex;

		double nu_u, nu_v, nu_w, nu_p;

		// VonKarman Turbulence Filter State
		struct ssVonKarman 
		{
			double H_u[2]; 
			double H_v[3]; 
			double H_w[3];
			double H_p[1];
			double H_q[1];
			double H_r[1];
			double H_Output[6];// U,V,W,P,Q,R
		};
		ssVonKarman  ssVonKarmanAt1000ft;
		ssVonKarman  ssVonKarmanAt2000ft;
		ssVonKarman  ssVonKarmanAtAltitudeft;
		ssVonKarman* ssVonKarmanActual;

		// Dryden Turbulence Filter State
		struct ssDryden 
		{
			double H_u[1]; 
			double H_v[2]; 
			double H_w[2];
			double H_p[1];
			double H_q[1];
			double H_r[1];
			double H_Output[6];// U,V,W,P,Q,R
		};
		ssDryden  ssDrydenAt1000ft;
		ssDryden  ssDrydenAt2000ft;
		ssDryden  ssDrydenAtAltitudeft;
		ssDryden* ssDrydenActual;

		double b_w;  // wingspan in ft
		double dt;   // turbulence period
		double V;    // true airspeed in ft/s


		// Angle of the Horizontal wind relatively to the North in degrees
		double windPsi;   // in degrees
		// Speed of the wind in m/s 
		double windSpeed; // in m/s 
		double trueAirSpeed; // in m/s

		FGColumnVector3 vHorizontalWindBody;     // Initialized to zero vector
		FGColumnVector3 vWindGustBody;
		FGColumnVector3 vWindShearBody;
		FGColumnVector3 vWindTurbulenceBody;
		FGColumnVector3 vWindTurbulenceRateBody;
		FGColumnVector3 vTotalWindBody;
		FGColumnVector3 vTotalWindRateBody;

		FGMatrix33 mNEDtoBody;
		FGMatrix33 mWindtoNED;

		FGColumnVector3 vEulerAngles; // Aircraft Euler Angles


		void calculate(double altitude);
		void computeStdAtmosphere(double altitude);

		void turbulence(void);
		void horizontalWind(void);
		void windGust(void);
		void windShear(void);

		void turbEssai();
		void VonKarmanFilters();
		void DrydenFilters();
		double getPOEtableValue(double);
 
};

#endif // __ENVIRONMENT_HH_DEF__
