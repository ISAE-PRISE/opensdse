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

#include "Efcs.hh"
#include "Common.hh"

using namespace std;

// ----------------------------------------------------------------------------
// Constructor 
Efcs::Efcs() 
{
	XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);
	vector<string> attribute_path(3);
	attribute_path[0] = "EFCS";
	attribute_path[1] = "simulation_loop";
	attribute_path[2] = "Sample_Time";
	mSampleTime = getDoubleValue(params, attribute_path);
	attribute_path[2] = "Discretization_Type";
	mDiscretizationType  = (eDiscretizationType) getIntValue(params, attribute_path); //unused
	// Debug
	#ifdef DEBUG_EFCS
	cout<<"mSampleTime : "<<mSampleTime<<endl;
	cout<<"mDiscretizationType    : "<<mDiscretizationType<<endl;
	#endif
	delete params;

	// Initialisation of variables linked to subscriptions
	mLongitude = mLatitude = mAltitude = 0.0;
	mPhi = mTheta = mPsi = 0.0;
	mU = mV = mW = 0.0;
	mP = mQ = mR = 0.0;
	mAx = mAy = mAz = 0.0;
	mIAS = mEAS = mCAS = mTAS = mGS = mVS = mMach = 0.0;
	mAlpha = mBeta = mQbar = 0.0;

	// Initialisation of trim parameters
	mTrimElevator = 0.0; // 0.01575;
	mTrimThrottle = 0.0; // 0.824906603433192; 

	// Initialisation of consignes and Eq values
	mAltitudeEq = 0.0;
	mVaEq = 0.0;
	mVzConsigne = 0.0;
	mPsiConsigne = 0.0;
	_PhiConsigne = 0.0;

	// Initialisation of Joystick outputs
	mAileron_Joystick = mElevator_Joystick = mRudder_Joystick = 0.0;
	mThrottle_Left_Joystick = 0.0;
	mThrottle_Right_Joystick = 0.0;

	// Initialisation of Cockpit outputs
	mAutopilot_AP_Active = 0 ;
	mAutopilot_athr_Active = 0 ;
	mAutopilot_spd = 0 ;
	mAutopilot_hdg = 0 ;
	mAutopilot_alt = 0 ;
	mAutopilot_vs = 0 ;

	// Initialisation of delta outputs
	mDeltaElevator = 0.0;
	mDeltaAileron = 0.0;
	mDeltaRudder = 0.0;
	mDeltaThrottle = 0.0;
	mThrottleEngines = 0.0;
	mThrottleEngine_Left = 0.0;
	mThrottleEngine_Right = 0.0;

	// Initialisation of outputs
	mRightElevator = mLeftElevator = 0.0;
	mRightAileron  = mLeftAileron  = 0.0;
	mRudder = 0.0;
	mRightEngine = mLeftEngine = 0.0;

	// Autopilot longitudinal mode
	mAP_case = 0;
	mSwitch_DeltaAlt = 200.0;

	// Constants for Altitude Hold
	mState_alt = 0;
	mKpalt     = 0.0972;
	mKialt     = 0.0038848;

	// Constants for Va Speed Control
	mState_Va =  0;
	mK1_intVa =  0;// Disabled temporarily. David's value = 0.0421459;
	mK1_Va    = -0.3045668;
	mK1_Vz    = -0.0440490;
	mK1_q     =  5.2927935;

	// Constants for Vz Speed Control
	mState_Vz =  0;
	mK2_intVz =  0.0002267;
	mK2_Vz    = -0.0012292;
	mK2_q     =  0.1786434;
	mK2_az    =  -0.0019416;

	// Variables for Lateral Law
	mPsiCorrection = 0.0;
	mPhiCorrection = 0.0;

	// Constants for Lateral Law (deg)
	mdeltaPsiMax_linear = 5.0;
	mPhiMax_linear = 20.0;
	mKphi = 0.1;
	
	// Altitude Hold
	_KpAlt    = 0.0005;
	_KdAlt    = 0.0004;
	_KiAlt    = 0.0;
	
	// Kp, Ts, Ti, Td, Alpha, Beta, Gamma, Min, Max
	_RollHoldControl.setParameters(0.25,0.05,1.5,0.25,0.1,0.1,0.0125,-1.0,1.0);
	_PitchHoldControl.setParameters(-0.1,0.05,2.515,0.5,0.1,0.001,0.0125,-1.0,1.0);
	_PitchStabilizeControl.setParameters(-0.15,0.05,10,0.000001,0.1,1.0,0.0,-1.0,1.0);

}

// ----------------------------------------------------------------------------
// Destructor 

Efcs::~Efcs()
{
	// Nothing to clean or done for destructor
}

// ----------------------------------------------------------------------------
//
void Efcs::calculate_state(void)
{ 
	if (mAutopilot_AP_Active) // Full Autopilot mode
	{ 			
		calculateAutopilotCase();
		calculateAutopilotConsigne();
		
		mState_alt += mSampleTime * (mAltitudeEq - mAltitude);
		mState_Va  += mSampleTime * (mVaEq - mIAS*mtoft*fpstokts);
		mState_Vz  += mSampleTime * (mVzConsigne - mVS);

	}
	else  if (mAutopilot_athr_Active) // Manual mode with ATHR
	{ 	
		mVaEq=mAutopilot_spd;
		mState_alt =0;
		mState_Va  += mSampleTime * (mVaEq - mIAS*mtoft*fpstokts);
		mState_Vz  =0;
	}
	else // Full Manual mode
	{			
		mState_alt =0;
		mState_Va  =0;
		mState_Vz  =0;
	}
}

// ----------------------------------------------------------------------------
//
void Efcs::calculate_output(void)
{ 

	if (mAutopilot_AP_Active)  // Full Autopilot mode
	{					
		#ifdef DEBUG_EFCS
		cout << "AUTOPILOT CONTROL"  << endl;
		cout << "mAltitudeEq    : " << mAltitudeEq    << endl;
		cout << "mAltitude      : " << mAltitude      << endl;
		cout << "mVaEq          : " << mVaEq          << endl;
		cout << "mVzConsigne    : " << mVzConsigne    << endl;
		cout << "mPsiConsigne   : " << mPsiConsigne   << endl;
		cout << "_PhiConsigne   : " << _PhiConsigne   << endl;
		cout << "mAutopilot_alt : " << mAutopilot_alt << endl;
		cout << "mAutopilot_vs  : " << mAutopilot_vs  << endl;
		cout << "mState_alt     : " << mState_alt     << endl;
		cout << "mState_Va      : " << mState_Va      << endl;
		cout << "mState_Vz      : " << mState_Vz      << endl;
		cout << "mVS            : " << mVS            << endl;
		#endif
		calculateAutopilotLaw();
	}
	else  if (mAutopilot_athr_Active) // Manual mode with ATHR
	{	
		#ifdef DEBUG_EFCS
		cout << "MANUAL CONTROL WITH ATHR"  << endl;
		#endif
		calculateManualLaw();

		// ATHR :
		mDeltaThrottle  =  mK1_intVa * mState_Va + mK1_Va * (mIAS*mtoft*fpstokts - mVaEq) + mK1_Vz * (mVS) + mK1_q * mQ;
		mThrottleEngines = mDeltaThrottle;
		if ( mThrottleEngines > 1 ) {mThrottleEngines = 1;}
		else if ( mThrottleEngines < 0 ) {mThrottleEngines = 0;}
		mThrottleEngine_Left = mThrottleEngines;
		mThrottleEngine_Right = mThrottleEngines;
	}
	else // Full Manual mode
	{									
		#ifdef DEBUG_EFCS
		cout << "MANUAL CONTROL"  << endl;
		#endif
		calculateManualLaw();
	}
  
	// CALCULATE OUTPUT VALUES, INCLUDING FAILURES
	mLeftElevator  =  mDeltaElevator;
	mRightElevator =  mDeltaElevator;
	mLeftAileron   =  mDeltaAileron;
	mRightAileron  =  -mDeltaAileron;
	mRudder        =  mDeltaRudder;
	mLeftEngine    =  mThrottleEngine_Left;
	mRightEngine   =  mThrottleEngine_Right;
	#ifdef DEBUG_EFCS
	cout << "Left Engine   : "  << mLeftEngine << endl;
	cout << "Right Engine  : "  << mRightEngine << endl;
	cout << "Elevator      : "  << mLeftElevator << endl;
	cout << "Ailerons      : "  << mLeftAileron << endl;
	cout << "Rudder        : "  << mRudder << endl;
	#endif
	
}

// ----------------------------------------------------------------------------
//
void Efcs::calculateAutopilotCase(void)
{
	int AP_case_old=mAP_case;
	switch (mAP_case) 
	{
		case 0 : 
		  if ( mAltitude < mAltitudeEq - mSwitch_DeltaAlt) mAP_case = 1;
		  else if ( mAltitude > mAltitudeEq + mSwitch_DeltaAlt) mAP_case = -1;
		  break;
		case 1 :
		  if ( mAltitude > mAltitudeEq ) mAP_case = 0;
		  break;
		case -1 :
		  if ( mAltitude < mAltitudeEq ) mAP_case = 0;
		  break;
		default:
		  mAP_case = 0;
		  break;
	}
	if (mAP_case!=AP_case_old) // Reset derivatives to prevent weird behaviors...
	{ 
		mState_alt = 0.0;
		mState_Va  = 0.0;
		mState_Vz  = 0.0;
	}
}

// ----------------------------------------------------------------------------
//
void Efcs::calculateAutopilotConsigne(void)
{
	// Airspeed consigne
	mVaEq = mAutopilot_spd;
	// Altitude consigne
	mAltitudeEq = mAutopilot_alt*fttom;
	// VZ consigne
    switch(mAP_case) {
    case 0: // ALT
	    mVzConsigne = mKpalt * (mAltitudeEq - mAltitude) - mKialt * mState_alt;
	    break;
	case 1: // CLB
		if(mAutopilot_vs <= 0 ) mVzConsigne = -10.0; 
		else mVzConsigne = - mAutopilot_vs*fttom/60; //mVzCons en m/s, mAP en ft/min
	    break;
	case -1: // DES
		if(mAutopilot_vs >= 0 ) mVzConsigne = 10.0; 
		else mVzConsigne = - mAutopilot_vs*fttom/60; //mVzCons en m/s, mAP en ft/min
	    break;
	default:
		break;
	}
	// Psi consigne
	if (fabs(mAutopilot_hdg - 360.0) < EPS_EFCS) mPsiConsigne = 0.0; else mPsiConsigne = mAutopilot_hdg*degtorad;
	// Phi consigne
	mPsiCorrection = mPsiConsigne - mPsi;
	
	while (mPsiCorrection >= M_PI) mPsiCorrection-=2*M_PI;
	if (mPsiCorrection <= -M_PI) mPsiCorrection+=2*M_PI;
	if (mPsiCorrection > mdeltaPsiMax_linear * degtorad) _PhiConsigne = mPhiMax_linear * degtorad;
	else if (mPsiCorrection < -mdeltaPsiMax_linear*degtorad) _PhiConsigne = -mPhiMax_linear * degtorad;
	else _PhiConsigne = mPsiCorrection * mPhiMax_linear / mdeltaPsiMax_linear;
	
	// Theta consigne
    _ThetaConsigne = _KpAlt*(mAltitudeEq-mAltitude) - _KdAlt*mVS;
    if (_ThetaConsigne < -asin(15/mTAS))
    {
		_ThetaConsigne = -asin(15/mTAS);
	}
    else if(_ThetaConsigne >  asin(15/mTAS))
    {
		_ThetaConsigne =  asin(15/mTAS);
	}
    _ThetaConsigne += mAlpha;
}

// ----------------------------------------------------------------------------
// Calculation of the autopilot law
void Efcs::calculateAutopilotLaw(void)
{
    // Throttle
    mDeltaThrottle  =  mK1_intVa * mState_Va + mK1_Va * (mIAS*mtoft*fpstokts - mVaEq) + mK1_Vz * (mVS) + mK1_q * mQ;
	mThrottleEngines = mDeltaThrottle;
	
	if ( mThrottleEngines > 1 ) 
	{
		mThrottleEngines = 1.0;
	}
	else if ( mThrottleEngines < 0 ) 
	{
		mThrottleEngines = 0.0;
	}
	
	mThrottleEngine_Left = mThrottleEngines;
	mThrottleEngine_Right = mThrottleEngines;
	
	// Aileron
	if     (_PhiConsigne > 30*degtorad) 
	{
		_PhiConsigne = 30*degtorad;
	}
    else if(_PhiConsigne <-30*degtorad) 
    {
		_PhiConsigne =-30*degtorad;
	}
    double PhiEq = _PhiConsigne -mPhi;
    if     (PhiEq <-10*degtorad) 
    {
		_PhiConsigne =-10*degtorad + mPhi;
	}
    else if(PhiEq > 10*degtorad) 
    {
		_PhiConsigne = 10*degtorad + mPhi;
	}
	mDeltaAileron = _RollHoldControl.calculatePID(mPhi, _PhiConsigne);

	// Elevator
	if     (_ThetaConsigne > 30*degtorad)
	{
		 _ThetaConsigne = 30*degtorad;
	}
    else if(_ThetaConsigne <-15*degtorad) 
    {
		_ThetaConsigne =-15*degtorad;
	}
    double ThetaEq = _ThetaConsigne -mTheta;
    if     (ThetaEq <-5*degtorad)
    {
		_ThetaConsigne =-5*degtorad + mTheta;
	}
    else if(ThetaEq > 5*degtorad) 
    {
		_ThetaConsigne = 5*degtorad + mTheta;
	}
	mDeltaElevator = _PitchHoldControl.calculatePID(mTheta, _ThetaConsigne);
	
	//Rudder
	mDeltaRudder=0.0;
	
}

// ----------------------------------------------------------------------------
// Calculation for Manual low
void Efcs::calculateManualLaw(void)
{
	_StickPitch = -(mElevator_Joystick<0)*el_minpos*degtorad*mElevator_Joystick + (mElevator_Joystick>=0)*el_maxpos*degtorad*mElevator_Joystick;
	_StickRoll = -(mAileron_Joystick<0)*ra_minpos*degtorad*mAileron_Joystick + (mAileron_Joystick>=0)*ra_maxpos*degtorad*mAileron_Joystick;
	
	// Aileron
	if     (_StickRoll > 30*degtorad)
	{
		_StickRoll = 30*degtorad;
	}
    else if(_StickRoll <-30*degtorad)
    {
		_StickRoll =-30*degtorad;
	}
    double PhiEq = _StickRoll -mPhi;
    if     (PhiEq <-15*degtorad) 
    {
		_StickRoll =-15*degtorad + mPhi;
	}
    else if(PhiEq > 15*degtorad) 
    {
		_StickRoll = 15*degtorad + mPhi;
	}
	mDeltaAileron = _RollHoldControl.calculatePID(mPhi, _StickRoll);

	// Elevator
	if     (_StickPitch > 30*degtorad) 
	{
		_StickPitch = 30*degtorad;
	}
    else if(_StickPitch <-15*degtorad) 
    {
		_StickPitch =-15*degtorad;
	}
    
    double ThetaEq = _StickPitch -mTheta;
    if     (ThetaEq <-5*degtorad) 
    {
		_StickPitch =-5*degtorad + mTheta;
	}
    else if(ThetaEq > 5*degtorad) 
    {
		_StickPitch = 5*degtorad + mTheta;
	}
	mDeltaElevator = _PitchHoldControl.calculatePID(mTheta, _StickPitch);
	
	//Rudder - Disabled to avoid problem when not using yoke / pedals system i.e. control the rudder
	//mDeltaRudder = (mRudder_Joystick>0)*ru_minpos*degtorad*mRudder_Joystick - (mRudder_Joystick<=0)*ru_maxpos*degtorad*mRudder_Joystick;
	mDeltaRudder=0.0;
	
	mThrottleEngine_Left = mThrottle_Left_Joystick;
	mThrottleEngine_Right = mThrottle_Right_Joystick;
	mDeltaFlaps = (mFlaps_Joystick>=0)*fl_maxpos*degtorad*mFlaps_Joystick;
	mDeltaSpoilers = (mSpoilers_Joystick>=0)*sp_maxpos*degtorad*mSpoilers_Joystick;
	mDeltaGears = (mGears_Joystick>=0)*ge_maxpos*degtorad*mGears_Joystick;
	mDeltaBrakes = mBrakes_Joystick;
}
