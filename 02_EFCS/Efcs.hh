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

#ifndef __EFCS_HH_DEF__
#define __EFCS_HH_DEF__

#include <cmath>
#include "UnitConversion.hh"
#include "A320.hh"
#include "ControllerPID.hh"

// This is an epsilon to compare floating point values
#define EPS_EFCS 1e-10

// ----------------------------------------------------------------------------
// Handle all the flight control calculations involved in the simulator.
// The aircraf can be controlled by an autopilot (closed loop) or be controlled
// using the Joystick.
// TO DO: Revamp the command laws which have to be much evolved and accurate
// ----------------------------------------------------------------------------
class Efcs : public UnitConversion, public A320 
{
	public:

		// Constructor
		Efcs();
		// Destructor
		~Efcs();

		// set/get functions

		void setLongitude(double longitude) {mLongitude = longitude;}
		void setLatitude(double latitude)   {mLatitude  = latitude;}
		void setAltitude(double altitude)   {mAltitude  = altitude;}

		double getLongitude(void) {return mLongitude;}
		double getLatude(void)    {return mLatitude;}
		double getAltitude(void)  {return mAltitude;}

		void setPhi(double phi)     {mPhi   = phi;}
		void setTheta(double theta) {mTheta = theta;}
		void setPsi(double psi)     {mPsi   = psi;}

		double getPhi(void)   {return mPhi;}
		double getTheta(void) {return mTheta;}
		double getPsi(void)   {return mPsi;}

		void setU(double u) {mU = u;}
		void setV(double v) {mV = v;}
		void setW(double w) {mW = w;}

		double getU(void) {return mU;}
		double getV(void) {return mV;}
		double getW(void) {return mW;}

		void setP(double p) {mP = p;}
		void setQ(double q) {mQ = q;}
		void setR(double r) {mR = r;}

		double getP(void) {return mP;}
		double getQ(void) {return mQ;}
		double getR(void) {return mR;}

		void setAx(double ax) {mAx = ax;}
		void setAy(double ay) {mAy = ay;}
		void setAz(double az) {mAz = az;}

		double getAx(void) {return mAx;}
		double getAy(void) {return mAy;}
		double getAz(void) {return mAz;}

		void setIAS(double val)  {mIAS = val;}
		void setEAS(double val)  {mEAS = val;}
		void setCAS(double val)  {mCAS = val;}
		void setTAS(double val)  {mTAS = val;}
		void setGS(double val)   {mGS = val;}
		void setVS(double val)   {mVS = val;}
		void setMach(double val) {mMach = val;}

		double getIAS(void)  {return mIAS;}
		double getEAS(void)  {return mEAS;}
		double getCAS(void)  {return mCAS;}
		double getTAS(void)  {return mTAS;}
		double getGS(void)   {return mGS;}
		double getVS(void)   {return mVS;}
		double getMach(void) {return mMach;}

		void setAlpha(double val) {mAlpha = val;}
		void setBeta(double val)  {mBeta  = val;}
		void setQbar(double val)  {mQbar  = val;}
		void setTankFilling(double val) {mTankFilling = val;}

		double getAlpha(void) {return mAlpha;}
		double getBeta(void)  {return mBeta;}
		double getQbar(void)  {return mQbar;}
		double getTankFilling(void) {return mTankFilling;}

		void setVaEq(double val)       {mVaEq = val;}
		void setAltitudeEq(double val) {mAltitudeEq = val;}

		double getVaEq(void)       {return mVaEq;}
		double getAltitudeEq(void) {return mAltitudeEq;}  

		void setAileron_Joystick(double input)     {mAileron_Joystick  = input;}
		void setElevator_Joystick(double input)    {mElevator_Joystick = input;}
		void setRudder_Joystick(double input)      {mRudder_Joystick   = input;}
		void setFlaps_Joystick(double input)       {mFlaps_Joystick    = input;}
		void setSpoilers_Joystick(double input)    {mSpoilers_Joystick = input;}
		void setGears_Joystick(double input)       {mGears_Joystick    = input;}
		void setBrakes_Joystick(double input)      {mBrakes_Joystick   = input;}

		double getAileron_Joystick(void)  {return mAileron_Joystick;}
		double getElevator_Joystick(void) {return mElevator_Joystick;}
		double getRudder_Joystick(void)   {return mRudder_Joystick;}
		double getFlaps_Joystick(void)    {return mFlaps_Joystick;}
		double getSpoilers_Joystick(void) {return mSpoilers_Joystick;}
		double getGears_Joystick(void)    {return mGears_Joystick;}
		double getBrakes_Joystick(void)   {return mBrakes_Joystick;}

		void setThrottle_Left_Joystick(double throttle) {mThrottle_Left_Joystick = throttle;}
		void setThrottle_Right_Joystick(double throttle) {mThrottle_Right_Joystick = throttle;}

		double getThrottle_Left_Joystick(void)   {return mThrottle_Left_Joystick;}
		double getThrottle_Right_Joystick(void)   {return mThrottle_Right_Joystick;}

		void setAutopilot_AP_Active(double AP_active)   {mAutopilot_AP_Active = AP_active;}
		void setAutopilot_athr_Active(double athr_active)   {mAutopilot_athr_Active = athr_active;}
		void setAutopilot_spd(double spd)   {mAutopilot_spd = spd;}
		void setAutopilot_hdg(double hdg)   {mAutopilot_hdg = hdg;}
		void setAutopilot_alt(double alt)   {mAutopilot_alt = alt;}
		void setAutopilot_vs(double vs)    {mAutopilot_vs = vs;}

		double getAutopilot_AP_Active(void)   {return mAutopilot_AP_Active;}
		double getAutopilot_athr_Active(void)   {return mAutopilot_athr_Active;}
		double getAutopilot_spd(void)   {return mAutopilot_spd;}
		double getAutopilot_hdg(void)   {return mAutopilot_hdg;}
		double getAutopilot_alt(void)   {return mAutopilot_alt;}
		double getAutopilot_vs(void)    {return mAutopilot_vs;}

		void setTrimElevator(double trimElevator) {mTrimElevator = trimElevator;}
		void setTrimStabilizer(double trimStabilizer) {mTrimStabilizer = trimStabilizer;}
		void setTrimThrottle(double trimThrottle) {mTrimThrottle = trimThrottle;}

		double getTrimElevator(void) {return mTrimElevator;}
		double getTrimStabilizer(void) {return mTrimStabilizer;}
		double getTrimThrottle(void) {return mTrimThrottle;}

		void setSampleTime(double sampletime) {mSampleTime = sampletime;}
		double getSampleTime(void) {return mSampleTime;}

		double getLeftElevator(void)  {return mLeftElevator;}
		double getRightElevator(void) {return mRightElevator;}
		double getLeftAileron(void)   {return mLeftAileron;}
		double getRightAileron(void)  {return mRightAileron;}
		double getRudder(void)        {return mRudder;}
		double getLeftEngine(void)    {return mLeftEngine;}
		double getRightEngine(void)   {return mRightEngine;}
		double getFlaps(void)         {return mFlaps;}
		double getSpoilers(void)      {return mSpoilers;}
		double getStabilizer(void)    {return mStabilizer;}
		double getGears(void)         {return mGears;}
		double getBrakes(void)        {return mBrakes;}

		void setLeftElevator(double input)  { mLeftElevator=input;}
		void setRightElevator(double input) { mRightElevator=input;}
		void setLeftAileron(double input)   { mLeftAileron=input;}
		void setRightAileron(double input)  { mRightAileron=input;}
		void setRudder(double input)        { mRudder=input;}
		void setLeftEngine(double input)    { mLeftEngine=input;}
		void setRightEngine(double input)   { mRightEngine=input;}
		void setFlaps(double input)         { mFlaps=input;}
		void setSpoilers(double input)      { mSpoilers=input;}
		void setStabilizer(double input)    { mStabilizer=input;}
		void setGears(double input)         { mGears=input;}
		void setBrakes(double input)        { mBrakes=input;}

		void calculate_state(void);
		void calculate_output(void);

		void calculateAutopilotCase(void);
		void calculateAutopilotConsigne(void);
		void calculateAutopilotLaw(void);
		void calculateManualLaw(void);

	private:

		enum eDiscretizationType {eTustin, eBilinear} mDiscretizationType; //unused

		double mLongitude, mLatitude, mAltitude;
		double mPhi, mTheta, mPsi;
		double mU, mV, mW;
		double mP, mQ, mR;
		double mAx, mAy, mAz;
		double mIAS, mEAS, mCAS, mTAS, mGS, mVS, mMach;
		double mAlpha, mBeta, mBetalast, mQbar;
		double mTankFilling;

		double mAileron_Joystick, mElevator_Joystick, mRudder_Joystick;
		double mThrottle_Left_Joystick;
		double mThrottle_Right_Joystick;
		double mFlaps_Joystick, mSpoilers_Joystick, mGears_Joystick, mBrakes_Joystick;

		double mAutopilot_AP_Active; // Man (0) & Autom. (1)
		double mAutopilot_athr_Active; // Man (0) & Autom. (1)
		double mAutopilot_spd;
		double mAutopilot_hdg;
		double mAutopilot_alt;
		double mAutopilot_vs;

		double mRightElevator, mLeftElevator;
		double mRightAileron, mLeftAileron;
		double mRudder;
		double mThrottleEngines;
		double mThrottleEngine_Left;
		double mThrottleEngine_Right;
		double mRightEngine, mLeftEngine;
		double mFlaps;
		double mSpoilers;
		double mStabilizer;
		double mGears;
		double mBrakes;

		double mDeltaElevator;
		double mDeltaAileron;
		double mDeltaRudder;
		double mDeltaThrottle;
		double mDeltaHS;
		double mDeltaFlaps;
		double mDeltaSpoilers;
		double mDeltaGears;
		double mDeltaBrakes;

		double mTrimThrottle;
		double mTrimStabilizer;
		double mTrimElevator;

		double mSampleTime;

		double mThetaEq;
		double mAltitudeEq;
		double mVaEq;
		double mVzConsigne;
		double mPsiConsigne;
		double _PhiConsigne;
		double _ThetaConsigne;

		// Autopilot longitudinal mode
		int mAP_case; //-1=DES 0=ALT 1=CLB
		double mSwitch_DeltaAlt;

		// Altitude Hold
		double mState_alt;
		double mKpalt;
		double mKialt;

		// Va Speed Control
		double mState_Va;
		double mK1_intVa;
		double mK1_Va;
		double mK1_Vz;
		double mK1_q;

		// Vz Speed Control
		double mState_Vz;
		double mK2_intVz;
		double mK2_Vz;
		double mK2_q;
		double mK2_az;

		// LATERAL LAW
		double mPsiCorrection;
		double mPhiCorrection;

		double mdeltaPsiMax_linear;
		double mPhiMax_linear;
		double mKphi;

		// Stick commands
		double _StickPitch, _StickRoll;
		
		// Altitude Hold
		double _KpAlt;
		double _KdAlt;
		double _KiAlt;
		
		ControllerPID _RollHoldControl;
		ControllerPID _PitchHoldControl;
		ControllerPID _PitchStabilizeControl;
 
};
#endif // __EFCS_HH_DEF__
