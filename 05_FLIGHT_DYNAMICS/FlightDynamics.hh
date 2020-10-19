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

#ifndef __FLIGHT_DYNAMICS_HH_DEF__
#define __FLIGHT_DYNAMICS_HH_DEF__

#include <cmath>
#include <vector>
#include "UnitConversion.hh"
#include "A320.hh"
#include "FGColumnVector3.hh"
#include "FGMatrix33.hh"
#include "EarthModel.hh"

using namespace std;


// Handle all the flight dynamics calculations involved in the simulator.
// See the VehicleState struct to see how the information regarding the aircraft
// is stored. The earth model used is handled by the EarthModel class (elipsoidal).
class FlightDynamics : public UnitConversion, public A320 
{

	public:

		// Three frames are used to characterise the aircraft location, velocity, acceleration and attitude :
		// 1 - The Earth Centered Earth Fixed frame, with geodetic coordinates : Longitude, Latitude and Height.
		// 2 - The Navigation frame, its origin is at the surface of earth, directly under the aircraft, and it uses the North, East, Down axes.
		// 3 - The Body frame, its origin is at the CG of the aircraft, and it uses the body (forward), the right wing and down as axes.
		struct VehicleState 
		{
		FGColumnVector3 vGeodLocation; // Location in Geod coordinates {eLong,eLat,eH} (ECEF frame) (rad,m)
		FGColumnVector3 vEulerAngles;  // Orientation (Euler Angles) Body to NED {ePhi,eTheta,ePsi} (rad)
		FGColumnVector3 vNavVelocity;  // Velocity expressed in the navigation frame (NED) {eNorth,eEast,eDown} (m/s)
		FGColumnVector3 vUVW;          // Velocity expressed in Body relative to ECEF {eU,eV,eW} (m/s)
		FGColumnVector3 vPQR;          // Angular Velocity expressed in Body relative to ECEF {eP,eQ,eR} (rad/s)
		FGColumnVector3 vAcceleration; // Linear Acceleration expressed in Body relative to ECEF (m/s2)
		double vQuaternions[4];        // Quaternions representing the atitude of the aircraft
		};

		enum eIntegrateType {eNone = 0, eRectEuler, eTrapezoidal, eAdamsBashforth2, eAdamsBashforth3};

		// Constructor
		FlightDynamics();
		// Destructor
		~FlightDynamics();

		// Initialization
		void initialization(void);

		// Calculation functions


		void calculate_state(void);  
		void calculate_output(void);
		void setBodytoNav(void);  
		void setWindtoBody(void);  
		void calculateFlightParameters(void);
		void calculateGravity(void);
		void calculateAerodynamics(void);
		void calculatePropulsion(void);
		void updateMassAndInertia(double dt);

		void calculateGeodLocationdot(void);
		void calculateQuaternionsdot(void);
		void calculateNavVelocitydot(void);
		void calculatePQRdot(void);

		// set/get functions
		void setDt(double dt) {mDt = dt;}
		void setIterations(int iterations) {mIterations = iterations;}
		void setIntegrationMethod(int val)  {mIntegrationMethod  = val;}

		double getDt(void) {return mDt;}
		int    getIterations(void) {return mIterations;}

		void setThrustLeftEngine(double thrustleftengine) {mThrustLeftEngine = thrustleftengine;}
		void setThrustRightEngine(double thrustrightengine) {mThrustRightEngine = thrustrightengine;}
		void setDeltaLeftAileronRad(double deltaleftaileron) {mDeltaLeftAileronRad = deltaleftaileron;}
		void setDeltaRightAileronRad(double deltarightaileron) {mDeltaRightAileronRad = deltarightaileron;}
		void setDeltaLeftElevatorRad(double deltaleftelevator) {mDeltaLeftElevatorRad = deltaleftelevator;}
		void setDeltaRightElevatorRad(double deltarightelevator) {mDeltaRightElevatorRad = deltarightelevator;}
		void setDeltaRudderRad(double deltarudder) {mDeltaRudderRad = deltarudder;}
		void setDeltaFlapRad(double input) {mDeltaFlapRad = input;}
		void setDeltaSpoilersRad(double input) {mDeltaSpoilersRad = input;}
		void setDeltaHSRad(double input) {mDeltaHSRad = input;}
		void setDeltaGear(double input) {mDeltaGears = input;}
		void setBrakes(double input) {mBrakes = input;}

		double getThrustLeftEngine() {return mThrustLeftEngine;}
		double getThrustRightEngine() {return mThrustRightEngine;}
		double getDeltaLeftAileronRad() {return mDeltaLeftAileronRad;}
		double getDeltaRightAileronRad() {return mDeltaRightAileronRad;}
		double getDeltaLeftElevatorRad() {return mDeltaLeftElevatorRad;}
		double getDeltaRightElevatorRad() {return mDeltaRightElevatorRad;}
		double getDeltaLeftElevatorDeg() {return mDeltaLeftElevatorRad * radtodeg;}
		double getDeltaRightElevatorDeg() {return mDeltaRightElevatorRad * radtodeg;}
		double getDeltaRudderRad() {return mDeltaRudderRad;}
		double getDeltaFlapRad() {return mDeltaFlapRad;}
		double getDeltaHSRad() {return mDeltaHSRad;}
		double getDeltaSpoilersRad() {return mDeltaSpoilersRad;}
		double getDeltaGear() {return mDeltaGears;}
		double getBrakes() {return mBrakes;}

		// From Env
		void setDensity(double density) {mDensity = density;}
		void setSoundSpeed(double soundspeed) {mSoundSpeed = soundspeed;}
		void setPressure(double pressure) {mPressure = pressure;}
		void setTemperature(double temperature) {mTemperature = temperature;}

		double getDensity() {return mDensity;}
		double getSoundSpeed() {return mSoundSpeed;}
		double getPressure() {return mPressure;}
		double getTemperature() {return mTemperature;}

		// Wind from Env
		void setUwind(double u) {vTotalWindBody(eU) = u;}
		void setVwind(double v) {vTotalWindBody(eV) = v;}
		void setWwind(double w) {vTotalWindBody(eW) = w;}
		void setPwind(double p) {vTotalWindRateBody(eP) = p;}
		void setQwind(double q) {vTotalWindRateBody(eQ) = q;}
		void setRwind(double r) {vTotalWindRateBody(eR) = r;}

		// Aircraft location
		// Returns the Longitude of the aircraft in degrees.
		double getLongitude(void) {return  radtodeg*vOutput.vGeodLocation(eLong);}
		// Returns the Latitude of the aircraft in degrees.
		double getLatitude(void)  {return  radtodeg*vOutput.vGeodLocation(eLat);}
		// Returns the Altitude of the aircraft in meters.
		double getAltitude(void)  {return  vOutput.vGeodLocation(eH);}

		// Set the Longitude of the aircraft in degrees.
		void setLongitude(double val) {vOutput.vGeodLocation(eLong) = degtorad*val; vState.vGeodLocation(eLong) = degtorad*val;}  
		// Set the Latitude of the aircraft in degrees.
		void setLatitude(double val)  {vOutput.vGeodLocation(eLat) = degtorad*val; vState.vGeodLocation(eLat) = degtorad*val;}  
		// Set the Altitude of the aircraft in meters.
		void setAltitude(double val)  {
		vOutput.vGeodLocation(eH) = val; vState.vGeodLocation(eH) = val;
		vGeodLocationdot(eLong)=0;last_vGeodLocationdot(eLat)=0;last2_vGeodLocationdot(eH)=0;
		}  

		// Aircraft attitude 
		double getPhi(void)   {return vOutput.vEulerAngles(ePhi);}
		double getTheta(void) {return vOutput.vEulerAngles(eTheta);}
		double getPsi(void)   {return vOutput.vEulerAngles(ePsi);}

		void setPhi(double val)    {vOutput.vEulerAngles(ePhi) = val; vState.vEulerAngles(ePhi) = val;}    
		void setTheta(double val)  {vOutput.vEulerAngles(eTheta) = val; vState.vEulerAngles(eTheta) = val;}    
		void setPsi(double val)    {vOutput.vEulerAngles(ePsi) = val; vState.vEulerAngles(ePsi) = val;}    

		// Angular rate 
		double getP(void) {return vOutput.vPQR(eP);} 
		double getQ(void) {return vOutput.vPQR(eQ);} 
		double getR(void) {return vOutput.vPQR(eR);} 

		void setP(double val) {vOutput.vPQR(eP) = val; vState.vPQR(eP) = val;}    
		void setQ(double val) {vOutput.vPQR(eQ) = val; vState.vPQR(eQ) = val;}    
		void setR(double val) {vOutput.vPQR(eR) = val; vState.vPQR(eR) = val;}    

		// Speed
		double getU(void) {return vOutput.vUVW(eU);} 
		double getV(void) {return vOutput.vUVW(eV);} 
		double getW(void) {return vOutput.vUVW(eW);} 

		void setU(double val) {vOutput.vUVW(eU) = val; vState.vUVW(eU) = val;}    
		void setV(double val) {vOutput.vUVW(eV) = val; vState.vUVW(eV) = val;}    
		void setW(double val) {vOutput.vUVW(eW) = val; vState.vUVW(eW) = val;}    

		// Acceleration
		void setAx(double ax) {vOutput.vAcceleration(eX) = ax; vState.vAcceleration(eX) = ax;}
		void setAy(double ay) {vOutput.vAcceleration(eY) = ay; vState.vAcceleration(eY) = ay;}
		void setAz(double az) {vOutput.vAcceleration(eZ) = az; vState.vAcceleration(eZ) = az; }

		double getAx(void) {return vOutput.vAcceleration(eX);}
		double getAy(void) {return vOutput.vAcceleration(eY);}
		double getAz(void) {return vOutput.vAcceleration(eZ);}

		// Others
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
		void setTank(double val)  {mTankFilling = val;}

		double getAlpha(void) {return mAlpha;}
		double getBeta(void)  {return mBeta;}
		double getQbar(void)  {return mQbar;}
		double getTank(void)  {return mTankFilling;}

		double getCoeff1D(vector< vector<double> >& coeff, double var1);
		double getCoeff2D(vector< vector<double> >& coeff, double var1, double var2);

		unsigned long ID;

		// Failure mode
		void setFailure(int input)  {  _Failure= input;}

	private:

		vector< vector<double> > CDalphaTable;
		vector< vector<double> > CYbTable;
		vector< vector<double> > CLalphaTable;
		vector< vector<double> > Cm0Table;
		vector< vector<double> > ClbTable;

		VehicleState vState;
		VehicleState vOutput;

		EarthModel gEarth; // Contains the Earth Geodetic modelization

		// Rotation difference between Earth rotating frame and Earth inertial frame (fixed)
		FGColumnVector3 vOmegaEarth;

		// Rotation difference between Navigation frame and Earth rotating frame
		FGColumnVector3 vOmegaNavRelative;

		// Rotation difference between Navigation frame and Earth inertial frame (fixed)
		FGColumnVector3 vOmegaNav;

		FGColumnVector3 vTotalWindBody;
		FGColumnVector3 vTotalWindRateBody;

		FGColumnVector3 vAeroUVW;
		FGColumnVector3 vAeroPQR;

		FGColumnVector3 vGravityForces;
		FGColumnVector3 vAerodynamicForces;
		FGColumnVector3 vPropulsionForces;

		FGColumnVector3 vAerodynamicMoments;
		FGColumnVector3 vPropulsionMoments;

		FGColumnVector3 vForces; // Forces in the navigation frame (NED)
		FGColumnVector3 vMoments;

		double mMass;

		double mDt;
		int    mIterations;
		int    mIntegrationMethod;

		// Coming from Engines and Actuators
		double mThrustLeftEngine;
		double mThrustRightEngine;
		double mDeltaLeftAileronRad;
		double mDeltaRightAileronRad;
		double mDeltaLeftElevatorRad;
		double mDeltaRightElevatorRad;
		double mDeltaRudderRad;
		double mDeltaHSRad;
		double mDeltaFlapRad;
		double mDeltaSpoilersRad;
		double mDeltaGears;
		double mBrakes;

		// Coming from Environment
		double mDensity;
		double mSoundSpeed;
		double mPressure;
		double mTemperature;

		// Flight Parameters
		double mIAS;  // in m/s
		double mEAS;  // in m/s
		double mCAS;  // in m/s
		double mTAS;  // in m/s
		double mGS;   // in m/s
		double mVS;   // in m/s

		double mMach;
		double mQbar;     // Dynamic Pressure  
		double mAlpha;    // Incidence Angle
		double mBeta;     // Sideslip Angle
		double mAlphadot;
		double mBetadot;
		double mNz;

		double mAlphaMin;
		double mAlphaMax;

		// Initial Conditions
		double mAltitudeInit;
		double mVitesseInit;
		double mLatitudeInit;
		double mLongitudeInit;
		double mHeadingInit;
		double mTankFilling;

		FGColumnVector3 vNavVelocitydot, last_vNavVelocitydot, last2_vNavVelocitydot;
		FGColumnVector3 vGeodLocationdot, last_vGeodLocationdot, last2_vGeodLocationdot;
		FGColumnVector3 vPQRdot, last_vPQRdot, last2_vPQRdot;
		double vQuaternionsdot[4], last_vQuaternionsdot[4], last2_vQuaternionsdot[4];

		FGColumnVector3 vUVWdot;

		FGMatrix33 mECItoNED;
		FGMatrix33 mNEDtoECI;

		FGMatrix33 mNavtoBody;
		FGMatrix33 mBodytoNav;

		FGMatrix33 mBodytoWind;
		FGMatrix33 mWindtoBody;

		FGMatrix33 mInertia;
		FGMatrix33 mEmptyInertia;
		FGMatrix33 mFullInertia;
		FGMatrix33 mInvInertia;

		FGColumnVector3 LeftEnginePositiontoCG;
		FGColumnVector3 RightEnginePositiontoCG;

		// Failure States
		int _Failure;

};

#endif
