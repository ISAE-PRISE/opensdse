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

#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>

#include "Sensor.hh"
#include "Common.hh"


// ----------------------------------------------------------------------------
// Constructor 
Sensor::Sensor() 
{
	srand(time(0));
  	// Get values from xml file
	XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);
	vector<string> attribute_path(3);
	attribute_path[0] = "SENSORS";

	attribute_path[1] = "simulation_loop";
	attribute_path[2] = "Delta_t_Init";
	mDt = getDoubleValue(params, attribute_path);
	attribute_path[2] = "Iteration_Steps";
	mIterations = getIntValue(params, attribute_path);
	attribute_path[2] = "Integration_Method";
	mIntegrationMethod = getIntValue(params, attribute_path);

	// Debug
	cout<<"mDt : "<<mDt<<endl;
	cout<<"mIterations    : "<<mIterations<<endl;
	cout<<"mIntegrationMethod: "<<mIntegrationMethod<<endl;
	delete params;

	// Static Definitions
	mNoise = 0.0;
	mNoiseType = ePercent;
	mDistributionType = eGaussian;
	mDriftRate = 0.0;
	mDrift = 0.0;
	mMin = -100000.0;
	mMax =  100000.0;
	mGranularity = 0.0;
	mBias = 0.0;
	mDelay = 0.0; 

	mFilterOrder = 2;
	mFilterCutOffFrequency = 3.0;
	mFilter = NULL;
	mState = NULL;
	mTempdot = NULL;
	mLastTempdot = NULL;
	mLast2Tempdot = NULL;

	mInputSignal = 0.0;
	mOutputSignal = 0.0;
	_Failure=0;
}

// ----------------------------------------------------------------------------
//
Sensor::~Sensor()
{

 delete [] mFilter;
 delete [] mState;
 delete [] mTempdot;
 delete [] mLastTempdot;
 delete [] mLast2Tempdot;

}

// ----------------------------------------------------------------------------
//
void Sensor::setFilter(int filterOrder, double filterCutOffFrequency)
{
    mFilterOrder = filterOrder;
    mFilterCutOffFrequency = filterCutOffFrequency;
    setButterworth();
}

// ----------------------------------------------------------------------------
//
void Sensor::setButterworth(void)
{
    // Calculate Butterworth filter coefficients

    double temp = 0.0;
    double* tempFilter = NULL;
    mFilter       = new double[mFilterOrder+1];
    mState        = new double[mFilterOrder];
    mTempdot      = new double[mFilterOrder];
    mLastTempdot  = new double[mFilterOrder];
    mLast2Tempdot = new double[mFilterOrder];
    tempFilter    = new double[mFilterOrder-1];

    for (int i=0;i<mFilterOrder+1;i++) 
    {
        mFilter[i]=0.0;
    }
    for (int i=0;i<mFilterOrder;i++) 
    {
        mState[i]        = 0.0;
        mTempdot[i]      = 0.0;
        mLastTempdot[i]  = 0.0;
        mLast2Tempdot[i] = 0.0;
    }
    
    if (mFilterOrder % 2 == 0) 
    {
        
        mFilter[0] = 1.0;
        mFilter[1] = -2.0 * cos((mFilterOrder+1.0)/(2.0*mFilterOrder)*M_PI);
        mFilter[2] = 1.0; 
            
        for (int i=0;i<mFilterOrder/2-1;i++) 
        {
            
            for (int j=0;j<2*i+3;j++) {tempFilter[j] = mFilter[j];}
            temp = -2.0 * cos((2.0*i+mFilterOrder+3.0)/(2.0*mFilterOrder)*M_PI);
            for (int j=0;j<2*i+3;j++) 
            {
                mFilter[j+1] += temp * tempFilter[j];
                mFilter[j+2] += tempFilter[j];          
            }         
        }   
    }
    else 
    {
        mFilter[0] = 1.0;
        mFilter[1] = 1.0; 
        
        for (int i=0;i<(mFilterOrder-1)/2;i++) 
        {
            for (int j=0;j<2*i+2;j++) {tempFilter[j] = mFilter[j];}
            temp = -2.0 * cos((2.0*i+mFilterOrder+1.0)/(2.0*mFilterOrder)*M_PI);
            for (int j=0;j<2*i+2;j++) 
            {
                mFilter[j+1] += temp * tempFilter[j];
                mFilter[j+2] += tempFilter[j]; 
            }       
        }   
    }
    
    temp = 2*M_PI*mFilterCutOffFrequency;
    for (int i=0;i<mFilterOrder;i++) 
    {
		mFilter[i] = mFilter[i]*pow(temp,mFilterOrder-i);
    }
   
    delete [] tempFilter;
}

// ----------------------------------------------------------------------------
//
void Sensor::calculateState(void)
{
    // Next State
    double dt=mDt/((double)mIterations);

    for (int i=0;i<mIterations;i++) 
    {

        for (int j=0;j<mFilterOrder-1;j++) 
        {
            mTempdot[j] = mState[j+1];
        }

        mTempdot[mFilterOrder-1] = 0.0;
        for (int j=0;j<mFilterOrder;j++) 
        {
            mTempdot[mFilterOrder-1] -= mState[j] * mFilter[j];
        }

        mTempdot[mFilterOrder-1] += mFilter[0] * mInputSignal;
        for (int j=0;j<mFilterOrder;j++) 
        {
            mState[j] += mTempdot[j] * dt;
        } 
      
        // Update State Variables
        switch(mIntegrationMethod) 
        {
			case eRectEuler:
				for (int j=0;j<mFilterOrder;j++) 
				{
					mState[j] += dt*mTempdot[j];
				}
				break;

			case eTrapezoidal:
				for (int j=0;j<mFilterOrder;j++) 
				{
					mState[j] += 0.5*dt*(mTempdot[j] + mLastTempdot[j]);   
				}
				break;

			case eAdamsBashforth2:
				for (int j=0;j<mFilterOrder;j++) 
				{
					mState[j] += dt*(1.5*mTempdot[j] - 0.5*mLastTempdot[j]);    
				}
				break;

			case eAdamsBashforth3: 
				for (int j=0;j<mFilterOrder;j++) 
				{
					mState[j] += (1/12.0)*dt*(23.0*mTempdot[j] - 16.0*mLastTempdot[j] + 5.0*mLast2Tempdot[j]);         
				}
				break;
	 
			case eNone:
				break;
        }
  
        // Set past values
        for (int j=0;j<mFilterOrder;j++) 
        {    
            mLast2Tempdot[j] = mLastTempdot[j];
            mLastTempdot[j] = mTempdot[j]; 
        } 
    }
}

// ----------------------------------------------------------------------------
//
void Sensor::calculateOutput(void)
{
    // Output
    if (_Failure==0) {mOutputSignal = mState[0];} 

    if (mNoise != 0.0)     Noise();     // models noise
    if (mDriftRate != 0.0) Drift();     // models drift over time
    if (mBias != 0.0)      Bias();      // models a finite bias
    if (mGranularity != 0) Quantize();  // models quantization
    if (mDelay != 0.0)     Delay();     // models system signal transport latencies
}

// ----------------------------------------------------------------------------
//
void Sensor::Noise(void)
{
	double randomValue = 0.0;
	if (mDistributionType == eUniform) 
	{
		randomValue = ((double)rand()/(double)RAND_MAX) - 0.5;
	} else 
	{
		randomValue = GaussianRandomNumber();
	}

	switch( mNoiseType ) 
	{
	  case ePercent:
		mOutputSignal *= (1.0 + mNoise*randomValue);
		break;

	  case eAbsolute:
		mOutputSignal += mNoise*randomValue;
		break;
	}
}

// ----------------------------------------------------------------------------
//
void Sensor::Bias(void)
{
	mOutputSignal += mBias;
}

// ----------------------------------------------------------------------------
//
void Sensor::Drift(void)
{
	mDrift += mDriftRate * mDt;
	mOutputSignal += mDrift;
}

// ----------------------------------------------------------------------------
//
void Sensor::Quantize(void)
{
	if (mOutputSignal < mMin) mOutputSignal = mMin;
	if (mOutputSignal > mMax) mOutputSignal = mMax;
	double portion = mOutputSignal - mMin;
	int quantized = (int)(portion/mGranularity);
	mOutputSignal = quantized*mGranularity + mMin;
}

// ----------------------------------------------------------------------------
//
void Sensor::Delay(void)
{
	// Not implemented
}
