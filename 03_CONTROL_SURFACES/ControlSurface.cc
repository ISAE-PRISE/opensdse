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


#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>

#include "ControlSurface.hh"
#include "Common.hh"


// ----------------------------------------------------------------------------
// ControlSurface Constructor
ControlSurface::ControlSurface() 
{	
	mMinPos = -30.0*degtorad;
	mMaxPos = 17.0*degtorad;
	mMaxRate = 40.0*degtorad;
	mBias = 0.0;
	mNaturalFrequency = 25.0;
	mDamping = 0.7;
	mDelay = 0.0;
	mDeltaDemand = 0;
	mDeltaActual = 0;
	mState[0] = mState[1] = 0.0;
	mTempdot[0] = mTempdot[1] = 0.0;
	mLastTempdot[0] = mLastTempdot[1] = 0.0;
	mLast2Tempdot[0] = mLast2Tempdot[1] = 0.0;
	_Failure=0;
}

// ----------------------------------------------------------------------------
// ControlSurface Destructor 
ControlSurface::~ControlSurface()
{
	// Nothing to be done
}

// ----------------------------------------------------------------------------
// ControlSurface initialization
void ControlSurface::initialize( double damping, double naturalfrequency, double minpos, double maxpos, double maxrate, double Delta_t_Init, int Iteration_Steps, int Integration_Method)
{
	mDamping           = damping;
	mNaturalFrequency  = naturalfrequency;
	mMinPos            = minpos * degtorad;
	mMaxPos            = maxpos * degtorad;
	mMaxRate           = maxrate * degtorad;
	mDt                = Delta_t_Init;
	mIterations        = Iteration_Steps;
	mIntegrationMethod = Integration_Method;
}

// ----------------------------------------------------------------------------
// Calculate state values
void ControlSurface::calculate_state(void)
{
    // Next State
    double dt=mDt/((double)mIterations);

    for (int i=0;i<mIterations;i++) 
    {   
        
        mTempdot[0] = mState[1];
        mTempdot[1] = - mNaturalFrequency * mNaturalFrequency * mState[0] - 2.0 * mNaturalFrequency * mDamping * mState[1] + mNaturalFrequency * mNaturalFrequency * mDeltaDemand;
                
        // Update State Variables
        switch(mIntegrationMethod) 
        {
        case eRectEuler:
            for (int j=0;j<2;j++) 
            {
                mState[j] += dt*mTempdot[j];
            }
            break;

        case eTrapezoidal:
            for (int j=0;j<2;j++) 
            {
                mState[j] += 0.5*dt*(mTempdot[j] + mLastTempdot[j]);
            }   
            break;

        case eAdamsBashforth2:
            for (int j=0;j<2;j++) 
            {
               mState[j] += dt*(1.5*mTempdot[j] - 0.5*mLastTempdot[j]);
            }    
            break;

        case eAdamsBashforth3: 
            for (int j=0;j<2;j++) 
            {
                mState[j] += (1/12.0)*dt*(23.0*mTempdot[j] - 16.0*mLastTempdot[j] + 5.0*mLast2Tempdot[j]);
            }            
            break;
 
        case eNone:
            break;
        }
  
        // Set past values    
        for (int j=0;j<2;j++) 
        {
            mLast2Tempdot[j] = mLastTempdot[j];
            mLastTempdot[j] = mTempdot[j];
        }
    }    
}

// ----------------------------------------------------------------------------
// Calculate Non Linear state values
void ControlSurface::calculate_stateNL(void)
{
    // Next State
    double dt=mDt/((double)mIterations);

    // Position Demand Saturation
    if (mDeltaDemand > mMaxPos) 
    {
        mDeltaDemand = mMaxPos;
    } 
    else if (mDeltaDemand < mMinPos) 
    {
        mDeltaDemand = mMinPos;
    }

    for (int i=0;i<mIterations;i++) 
    {
        
        mTempdot[0] = mState[1];
        mTempdot[1] = - mNaturalFrequency * mNaturalFrequency * mState[0] - 2.0 * mNaturalFrequency * mDamping * mState[1] + mNaturalFrequency * mNaturalFrequency * mDeltaDemand;
                
        // Update State Variables
        switch(mIntegrationMethod) 
        {
        case eRectEuler:
            for (int j=0;j<2;j++) 
            {
                mState[j] += dt*mTempdot[j];
            }
            break;

        case eTrapezoidal:
            for (int j=0;j<2;j++) 
            {
                mState[j] += 0.5*dt*(mTempdot[j] + mLastTempdot[j]);
            }   
            break;

        case eAdamsBashforth2:
            for (int j=0;j<2;j++) 
            {
               mState[j] += dt*(1.5*mTempdot[j] - 0.5*mLastTempdot[j]);
            }    
            break;

        case eAdamsBashforth3: 
            for (int j=0;j<2;j++) 
            {
                mState[j] += (1/12.0)*dt*(23.0*mTempdot[j] - 16.0*mLastTempdot[j] + 5.0*mLast2Tempdot[j]);
            }            
            break;
 
        case eNone:
            break;
        }
  
        // Set past values    
        for (int j=0;j<2;j++) 
        {
            mLast2Tempdot[j] = mLastTempdot[j];
            mLastTempdot[j] = mTempdot[j];
        }

        // Rate Saturation
        if (mState[1] > mMaxRate) 
        {
            mState[1] = mMaxRate;
        }
        else if (mState[1] < -mMaxRate) 
        {
            mState[1] = -mMaxRate;
        }
       
        // Position Saturation
        if (mState[0] > mMaxPos) 
        {
            mState[0] = mMaxPos;
            mState[1] = 0.0;
        }
        else if (mState[0] < mMinPos)
        {
            mState[0] = mMinPos;
            mState[1] = 0.0;
        }
    }   
}

// ----------------------------------------------------------------------------
// Calculate output values
void ControlSurface::calculate_output(void)
{
    // Output
    if (_Failure==0) {mDeltaActual = mState[0];}
}
