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

#ifndef __ENGINE_HH_DEF__
#define __ENGINE_HH_DEF__

#include <cmath>
#include "UnitConversion.hh"


// Handle the thrust calculations
// Calculate the thrust of the engine depending on the external conditions.
class Engine : public UnitConversion 
{
	public:
		enum eIntegrateType {eNone = 0, eRectEuler, eTrapezoidal, eAdamsBashforth2, eAdamsBashforth3};

		// Constructor
		Engine();
		// Destructor
		~Engine();
		// Initialization
		void initialization(void);

		// set/get functions
		void setPressure(double pressure) {mPressure = pressure;}
		double getPressure() {return mPressure;} 

		void setTemperature(double temperature) {mTemperature = temperature;}
		double getTemperature() {return mTemperature;} 

		void setMach(double mach) {mMach = mach;}
		double getMach() {return mMach;} 

		void setAltitude(double input) {mAltitude = input;}
		double getAltitude(void) {return mAltitude;}

		void setDt(double dt) {mDt = dt;}
		void setIterations(int iterations) {mIterations = iterations;}

		double getDt(void) {return mDt;}
		int    getIterations(void) {return mIterations;}

		void setDeltaThrottle(double deltathrottle) {mDeltaThrottle = deltathrottle;}
		void setThrustActual(double thrustactual)   {mThrustActual = thrustactual;}
		void setInitialState(double initialstate)   {mState = initialstate / mUninstallInstallRatio;}
		double getDeltaThrottle(void) {return mDeltaThrottle;}
		double getThrustActual(void) {return mThrustActual;}  
		unsigned long ID;
		void setInputs(double mach, double temperature, double pressure, double deltathrottle);
		void calculate_state(void);
		void calculate_output(void);
		void setOutput(double output) {mState=output/ mUninstallInstallRatio;}
		void setFailure(int input) {_Failure=input;}

	private:
		double calculateNonDimTau(double x);

		double mTau;
		double mNonDimTau;
		double mTauNominal;
		double mGain;

		double mDt;
		int    mIterations;
		int    mIntegrationMethod;

		double mBypassRatio;
		double mPressureRatio;
		double mTemperatureRatio;
		double mMach;
		double mTemperature;
		double mPressure;
		double mDeltaThrottle;
		double mThrust;
		double mThrustActual;
		double mMaxThrust;
		double mUninstallInstallRatio; 
		double mAltitude;

		double mState;
		double mTempdot;
		double mLastTempdot;
		double mLast2Tempdot;

		double mNonDimThrust;

		int _Failure;
};

#endif
