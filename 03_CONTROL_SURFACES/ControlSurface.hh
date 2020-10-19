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


#ifndef __CONTROL_SURFACES_HH_DEF__
#define __CONTROL_SURFACES_HH_DEF__

#include <cmath>
#include <vector>
#include "UnitConversion.hh"

using namespace std;

// ----------------------------------------------------------------------------
// Handle all the acuator calculations involved in the simulator.
// Simulate the behavior of a control surface actuator. 
// ----------------------------------------------------------------------------
class ControlSurface : public UnitConversion 
{
	private:
	
		double mMinPos;
		double mMaxPos;
		double mMaxRate;
		double mBias;
		double mNaturalFrequency;
		double mDamping;

		double mDt;
		int    mIterations;
		int    mIntegrationMethod;

		double mDelay; 
		double mHysteresis;

		double mDeltaDemand;
		double mDeltaActual;

		double mState[2]; 
		double mTempdot[2];
		double mLastTempdot[2];
		double mLast2Tempdot[2];

		int _Failure; 

	public:

		enum eIntegrateType {eNone = 0, eRectEuler, eTrapezoidal, eAdamsBashforth2, eAdamsBashforth3};
		ControlSurface();
		~ControlSurface();

		// set/get functions
		void setMinPosDeg(double minpos) {mMinPos = minpos*degtorad;}
		void setMaxPosDeg(double maxpos) {mMaxPos = maxpos*degtorad;}
		void setMaxRateDeg(double maxrate) {mMaxRate = maxrate*degtorad;}
		void setBias(double bias) {mBias = bias;}
		void setNaturalFrequency(double naturalfrequency) {mNaturalFrequency = naturalfrequency;}
		void setDamping(double damping) {mDamping = damping;}
		void setDt(double dt) {mDt = dt;}
		void setIterations(int iterations) {mIterations = iterations;}
		void setDelay(double delay) {mDelay = delay;}
		void setHysteresis(double hysteresis) {mHysteresis = hysteresis;}

		double getMinPosDeg(void) {return mMinPos*radtodeg;}
		double getMaxPosDeg(void) {return mMaxPos*radtodeg;}  
		double getMaxRateDeg(void) {return mMaxRate*radtodeg;}
		double getBias(void) {return mBias;}
		double getNaturalFrequency(void) {return mNaturalFrequency;}
		double getDamping(void) {return mDamping;}
		double getDt(void) {return mDt;}
		int    getIterations(void) {return mIterations;}
		double getDelay(void) {return mDelay;}
		double getHysteresis(void) {return mHysteresis;}  

		void setDeltaDemandDeg(double deltademand) {mDeltaDemand = deltademand*degtorad;}
		void setDeltaDemandRad(double deltademand) {mDeltaDemand = deltademand;}
		void setDeltaActualDeg(double deltaactual) {mDeltaActual = deltaactual*degtorad;}
		void setDeltaActualRad(double deltaactual) {mDeltaActual = deltaactual;}
		void setInitialStateDeg(double initialstate) {mState[0] = initialstate*degtorad;}
		void setInitialStateRad(double initialstate) {mState[0] = initialstate;}

		double getDeltaDemandDeg(void) {return mDeltaDemand*radtodeg;}
		double getDeltaDemandRad(void) {return mDeltaDemand;}
		double getDeltaActualDeg(void) {return mDeltaActual*radtodeg;}
		double getDeltaActualRad(void) {return mDeltaActual;}

		void initialize( double damping
		               , double naturalfrequency
		               , double minpos
		               , double maxpos
		               , double maxrate
		               , double Delta_t_Init
		               , int Iteration_Steps
		               , int Integration_Method
		               );

		void calculate_state(void);
		void calculate_stateNL(void);   
		void calculate_output(void);

		void setOutput(double output) {mState[0] = output;}
		int getFailure()              {return _Failure;}
		void setFailure(int input)    {_Failure = input;}

};

#endif
