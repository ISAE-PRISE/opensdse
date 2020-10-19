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

#ifndef __SENSOR_HH__
#define __SENSOR_HH__

#include <cmath>
#include "UnitConversion.hh"

using namespace std;

// ----------------------------------------------------------------------------
// Simulate the behavior of a sensor.
// The Sensor process the data from the simulation (input) to provide an
// output simulating what would be obtained with a sensor fusion unit, for 
// with some effects like limited bandwith, noise, etc.
// ----------------------------------------------------------------------------
class Sensor : public UnitConversion 
{

	public:

		enum eIntegrateType {eNone = 0, eRectEuler, eTrapezoidal, eAdamsBashforth2, eAdamsBashforth3};

		// Constructor
		Sensor();
		// Destructor
		~Sensor();

		// set/get functions
		void setGranularity(double granularity) {mGranularity = granularity;}
		void setBias(double bias) {mBias = bias;}
		void setDt(double dt) {mDt = dt;}
		void setIterations(int iterations) {mIterations = iterations;}
		void setDelay(double delay) {mDelay = delay;}
		void setFilterOrder(int filterOrder) {mFilterOrder = filterOrder;}
		void setFilterCutOffFrequency(double filterCutOffFrequency) {mFilterCutOffFrequency = filterCutOffFrequency;}

		double getGranularity(void) {return mGranularity;}
		double getBias(void) {return mBias;}
		double getDt(void) {return mDt;}
		int    getIterations(void) {return mIterations;}
		double getDelay(void) {return mDelay;}

		void setInputSignal(double inputsignal) {mInputSignal = inputsignal;}
		void setOutputSignal(double outputsignal) {mOutputSignal = outputsignal;}
		void setInitialState(double initialstate) {mState[0] = initialstate;}

		double getInputSignal(void) {return mInputSignal;}
		double getOutputSignal(void) {return mOutputSignal;} 

		double* getFilter(void) {return mFilter;}

		unsigned long ID;

		// Set up the order and the cutoff frequency of the lowpass filter used.
		// the filter cutoff frequency is specified in Hz
		void setFilter(int filterOrder, double filterCutOffFrequency);
		void setButterworth(void);
		void calculateState(void);
		void calculateOutput(void);

		void setFailure(int input) {_Failure=input;}

	private:

		void Noise(void);
		void Bias(void);
		void Drift(void);
		void Quantize(void);
		void Delay(void);

		double mNoise;
		enum eNoiseType {ePercent=0, eAbsolute} mNoiseType;
		enum eDistributionType {eUniform=0, eGaussian} mDistributionType;

		double mDriftRate;
		double mDrift;
		double mMin;
		double mMax;
		double mGranularity;
		double mBias;

		double mDt;
		int    mIterations;
		int    mIntegrationMethod;

		double mDelay;
		int    mFilterOrder;
		double mFilterCutOffFrequency;
		double* mFilter;

		double* mState;
		double* mTempdot;
		double* mLastTempdot;
		double* mLast2Tempdot;

		double mInputSignal;
		double mOutputSignal;

		int _Failure;

};

#endif //  __SENSOR_HH__
