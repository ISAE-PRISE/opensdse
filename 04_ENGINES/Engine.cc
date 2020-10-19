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

#include "Engine.hh"
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>

using namespace std;

// ----------------------------------------------------------------------------
// Constructor 
Engine::Engine() 
{
	mTauNominal = 1.0;
	mTau = 0.0;
	mNonDimTau = 0.0;

	mDt = 0.01;
	mIterations = 1;
	mIntegrationMethod = 2;

	mBypassRatio = 5.7;
	mDeltaThrottle = 0;
	mMaxThrust = 120000;
	mThrust = 0;
	mNonDimThrust = 0;
	mUninstallInstallRatio = 1.0;

	mState = 0.0;
	mTempdot = 0.0;
	mLastTempdot = 0.0;
	mLast2Tempdot = 0.0;

	_Failure=0;
}

// ----------------------------------------------------------------------------
// Destructor 
Engine::~Engine()
{
	// Nothing to be done
}

// ----------------------------------------------------------------------------
//
void Engine::setInputs(double mach, double temperature, double pressure, double deltathrottle)
{
	mMach = mach;
	mTemperature = temperature;
	mPressure = pressure;
	mDeltaThrottle = deltathrottle;
}

// ----------------------------------------------------------------------------
//
void Engine::initialization(void)
{
	double deltathrottlemin = 0.0;
	double deltathrottlemax = 1.0;
	double fal = 1;
	double tol = 1e-8;
	double thrusteq = mThrustActual;

	while (fabs(fal)>tol) 
	{ 
		mDeltaThrottle = (deltathrottlemin + deltathrottlemax)/2.0;
		fal = mThrustActual - thrusteq ;

		if (fal>0.0) 
		{
			deltathrottlemax = mDeltaThrottle;
		} 
		else 
		{
			deltathrottlemin = mDeltaThrottle;
		}
	}
	mDeltaThrottle = (deltathrottlemin + deltathrottlemax)/2.0;
}

// ----------------------------------------------------------------------------
//
void Engine::calculate_state(void)
{
    // Next State
    double dt=mDt/((double)mIterations);
    // Non-Dimensional Thrust

    if (mMach<0.5) 
    {
		mNonDimThrust = (1.0 - 0.8 * mMach) * mDeltaThrottle;
    }
    else 
    {
		mNonDimThrust = (0.7 - 0.2 * mMach) * mDeltaThrottle;
    } 
 
    // Calculus of Temperature and Pressure Ratios
    double temp;
    temp = 1.0 + (SHRatio - 1.0) * 0.5 *(mMach * mMach) ; 
    mTemperatureRatio = mTemperature * temp / SLtemperature;
    mPressureRatio    = mPressure * pow(temp,SHRatio/(SHRatio - 1.0)) / SLpressure;

    // Thrust Demand
    mThrust = mNonDimThrust * mMaxThrust * mPressureRatio;
    if (mThrust > mMaxThrust) {mThrust = mMaxThrust;}

    for (int i=0;i<mIterations;i++)
    {
        temp = mState / mMaxThrust / mPressureRatio;
        mNonDimTau =  calculateNonDimTau(temp);
        mTau = mNonDimTau * mTauNominal * mPressureRatio / sqrt(mTemperatureRatio);
        mTempdot = (mThrust - mState) / mTau;

        if (_Failure==0) 
        {
			// Update State Variables
			switch(mIntegrationMethod) 
			{
				case eRectEuler:
					mState += dt*mTempdot;
					break;

				case eTrapezoidal:
					mState += 0.5*dt*(mTempdot + mLastTempdot);   
					break;

				case eAdamsBashforth2:
					mState += dt*(1.5*mTempdot - 0.5*mLastTempdot);    
					break;

				case eAdamsBashforth3: 
					mState += (1/12.0)*dt*(23.0*mTempdot - 16.0*mLastTempdot + 5.0*mLast2Tempdot);         
					break;
		 
				case eNone:
					break;
			}
        }
        // Set past values    
        mLast2Tempdot = mLastTempdot;
        mLastTempdot = mTempdot;  
    }    
}

// ----------------------------------------------------------------------------
//
void Engine::calculate_output(void)
{
    // Output
    mThrustActual = mState * mUninstallInstallRatio;
    double mMachRatio;
    if (mMach<0.5) 
    {
      mMachRatio = (1.0 - 0.8 * mMach);
    }
    else 
    {
      mMachRatio = (0.7 - 0.2 * mMach);
    }
    #ifdef DEBUG_ENGINE
    cout << "Engine : Thrust = "<< mThrustActual  << " ( " << 100*mThrustActual/(mMachRatio*mPressureRatio*mMaxThrust)  << " % of Thrust max)" << endl;
    #endif
}

// ----------------------------------------------------------------------------
//
double Engine::calculateNonDimTau(double x)
{
	double output;
	output = (2.612*x*x-1.243*x+0.4133)/(x*x*x+0.5303*x*x+0.1922*x+0.07973);
	return output;
}
