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

#include "FlightDynamics.hh"
#include "Common.hh"
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>


// ----------------------------------------------------------------------------
// Constructor 
FlightDynamics::FlightDynamics()
{
	XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);

	// Get values from xml file
	vector<string> attribute_path(3);
	attribute_path[0] = "FLIGHT_DYNAMICS";
	attribute_path[1] = "init";
	attribute_path[2] = "AltitudeInit";
	mAltitudeInit = getDoubleValue(params, attribute_path);
	attribute_path[2] = "VitesseInit";
	mVitesseInit  = getDoubleValue(params, attribute_path);
	attribute_path[2] = "LongitudeInit";
	mLongitudeInit = getDoubleValue(params, attribute_path)/radtodeg;
	attribute_path[2] = "LatitudeInit";
	mLatitudeInit = getDoubleValue(params, attribute_path)/radtodeg;
	attribute_path[2] = "HeadingInit";
	mHeadingInit = getIntValue(params, attribute_path);
	attribute_path[2] = "TankFilling";
	mTankFilling = getDoubleValue(params, attribute_path);
	
	// Integration Method
	attribute_path[1] = "simulation_loop";
	attribute_path[2] = "Delta_t_Init";
	mDt = getDoubleValue(params, attribute_path);
	attribute_path[2] = "Iteration_Steps";
	mIterations = getIntValue(params, attribute_path);
	attribute_path[2] = "Integration_Method";
	mIntegrationMethod = getIntValue(params, attribute_path);
	delete params;
  
	// Debug
	#ifdef DEBUG_FLIGH_DYNAMICS
	cout<<"mDt : "<<mDt<<endl;
	cout<<"mIterations    : "<<mIterations<<endl;
	cout<<"mIntegrationMethod: "<<mIntegrationMethod<<endl;
	#endif
	
	vState.vGeodLocation = FGColumnVector3(mLongitudeInit,mLatitudeInit,mAltitudeInit);
	vOutput.vGeodLocation = FGColumnVector3(mLongitudeInit,mLatitudeInit,mAltitudeInit);
	
	// State Vector Initialization
	vNavVelocitydot.InitMatrix(); 
	last_vNavVelocitydot.InitMatrix(); 
	last2_vNavVelocitydot.InitMatrix();

	vGeodLocationdot.InitMatrix();
	last_vGeodLocationdot.InitMatrix();
	last2_vGeodLocationdot.InitMatrix();

	vPQRdot.InitMatrix(); 
	last_vPQRdot.InitMatrix(); 
	last2_vPQRdot.InitMatrix();

	for (int i=0;i<4;i++) 
	{
		vQuaternionsdot[i]=0.0; 
		last_vQuaternionsdot[i]=0.0;
		last2_vQuaternionsdot[i]=0.0;
	}

	// Input Initialization
	mThrustLeftEngine = 0.0;
	mThrustRightEngine = 0.0;
	mDeltaLeftAileronRad = 0.0;
	mDeltaRightAileronRad = 0.0;
	mDeltaLeftElevatorRad = 0.0;
	mDeltaRightElevatorRad = 0.0;
	mDeltaRudderRad = 0.0;
	mMass = EmptyMass;

	// Transformation Matrix Initialization
	mBodytoNav.InitMatrix();
	mNavtoBody.InitMatrix();

	mBodytoWind.InitMatrix();
	mWindtoBody.InitMatrix();

	mDensity = 0.412705;
	mSoundSpeed = 340.0;
	mPressure = SLpressure;
	mTemperature = SLtemperature;

	// Approximation of Mach
	mMach = mVitesseInit/mSoundSpeed;

	mAlphaMin = -0.0900;
	mAlphaMax =  0.3600;

	mEmptyInertia = FGMatrix33(EmptyIxx,-EmptyIxy,-EmptyIxz,
	-EmptyIxy,EmptyIyy,-EmptyIyz,-EmptyIxz,-EmptyIyz,EmptyIzz);
	mFullInertia = FGMatrix33(FullIxx,-FullIxy,-FullIxz,
	-FullIxy,FullIyy,-FullIyz,-FullIxz,-FullIyz,FullIzz);

	LeftEnginePositiontoCG  = LeftEngineLocation  - CG;
	RightEnginePositiontoCG = RightEngineLocation - CG;

	// Aero Coeff Tables

	CDalphaTable.resize(27,vector<double>(5,0));
	for (int i=0;i<27;i++) 
	{
		for (int j=0;j<5;j++) 
		{
			CDalphaTable[i][j] = CDalpha[i][j];  
		}
	}

	CYbTable.resize(3,vector<double>(2,0));
	for (int i=0;i<3;i++) 
	{
		for (int j=0;j<2;j++) 
		{
			CYbTable[i][j] = CYb[i][j];  
		}
	}

	CLalphaTable.resize(18,vector<double>(6,0));
	for (int i=0;i<18;i++) 
	{
		for (int j=0;j<6;j++) 
		{
			CLalphaTable[i][j] = CLalpha[i][j];  
		}
	}

	ClbTable.resize(3,vector<double>(2,0));
	for (int i=0;i<3;i++) 
	{
		for (int j=0;j<2;j++) 
		{
			ClbTable[i][j] = Clb[i][j];  
		}
	}

	Cm0Table.resize(2,vector<double>(2,0));
	for (int i=0;i<2;i++) 
	{
		for (int j=0;j<2;j++) 
		{
			Cm0Table[i][j] = Cm0[i][j];  
		}
	}
}

// ----------------------------------------------------------------------------
// Destructor 
FlightDynamics::~FlightDynamics()
{ 

}

// ----------------------------------------------------------------------------
// Initialization
void FlightDynamics::initialization(void)
{
	// mVitesseInit = mMach * mSoundSpeed; 

	// calculate Mass of the aircraft
	double fuelMass = mTankFilling * (FullMass - EmptyMass);
	mMass = EmptyMass + fuelMass;

	// calculate Inertia of the Aircraft
	mInertia = mTankFilling * mFullInertia + (1 - mTankFilling) * mEmptyInertia;
	mInvInertia = mInertia.Inverse();
	mQbar = 0.5 * mDensity * mVitesseInit * mVitesseInit;

	double QS = mQbar * wingarea;
	double alphaInf = mAlphaMin;
	double alphaSup = mAlphaMax;
	double fal = 1;
	double tol = 1e-8;
	double ca, sa;
	double thrust, lift, drag, deltae;
	double localG = gEarth.getG(mLatitudeInit,mAltitudeInit);

	// Calculate the equilibrium (Alpha)
	while (fabs(fal)>tol) 
	{ 
		mAlpha = (alphaInf + alphaSup)/2.0;
		ca     = cos(mAlpha);
		sa     = sin(mAlpha);

		// Moment equation
		deltae = -(getCoeff1D(Cm0Table,0.0) + Cmalpha * mAlpha)/Cmde;

		// Drag/Thrust equation
		lift = QS * getCoeff2D(CLalphaTable,mAlpha,0.0);
		lift += QS * CLde * deltae;
		drag = QS * CD0;
		drag += QS * getCoeff2D(CDalphaTable,mAlpha,0.0);
		drag += QS * CDde * deltae;
		thrust = mMass * localG * sa - lift * sa + drag * ca;

		// Lift/Weight equation
		fal = mMass * localG * ca -lift * ca - drag * sa;

		if (fal<0.0) 
		{
			alphaSup = mAlpha;
			mAlpha = (mAlpha+alphaInf)/2;
		} 
		else 
		{
			alphaInf = mAlpha;
			mAlpha = (mAlpha+alphaSup)/2;
		}
	}

	// Initialization of concerned variables according to the calculated equilibrium 
	vState.vGeodLocation  = FGColumnVector3(mLongitudeInit,mLatitudeInit,mAltitudeInit+100.0);
	vOutput.vGeodLocation = FGColumnVector3(mLongitudeInit,mLatitudeInit,mAltitudeInit+100.0);
	vState.vEulerAngles   = FGColumnVector3(0.0,mAlpha,0.0);
	vOutput.vEulerAngles  = FGColumnVector3(0.0,mAlpha,0.0);
	vState.vNavVelocity   = FGColumnVector3(mVitesseInit,0.0,0.0);
	vOutput.vNavVelocity  = FGColumnVector3(mVitesseInit,0.0,0.0);
	vState.vUVW           = FGColumnVector3(mVitesseInit * cos(mAlpha),0.0,mVitesseInit * sin(mAlpha));
	vOutput.vUVW          = FGColumnVector3(mVitesseInit * cos(mAlpha),0.0,mVitesseInit * sin(mAlpha));
	vState.vPQR           = FGColumnVector3(0.0,0.0,0.0);
	vOutput.vPQR          = FGColumnVector3(0.0,0.0,0.0);
	vState.vAcceleration  = FGColumnVector3(0.0,0.0,0.0);
	vOutput.vAcceleration = FGColumnVector3(0.0,0.0,0.0);

	vState.vQuaternions[0] = cos(mAlpha/2); // Psi=Phi=0
	vState.vQuaternions[1] = 0.0;           // Psi=Phi=0
	vState.vQuaternions[2] = sin(mAlpha/2); // Psi=Phi=0
	vState.vQuaternions[3] = 0.0;           // Psi=Phi=0
	
	for (int i=0;i<4;i++)
	{
		vOutput.vQuaternions[i] = vState.vQuaternions[i];
	}
	setBodytoNav();

	mDeltaLeftElevatorRad  = deltae;
	mDeltaRightElevatorRad = deltae;

	mThrustLeftEngine  = thrust/2.0;
	mThrustRightEngine = thrust/2.0;

	mBeta     = 0.0;
	mAlphadot = 0.0;

	mCAS = sqrt(SHRatio * Reng * SLtemperature) * sqrt(5 * ( pow(mQbar/SLpressure + 1.0,2.0/7.0) - 1.0 ) );
	mIAS = mCAS;
	mEAS = sqrt(2 * mQbar / SLdensity); 
	mTAS = vState.vUVW.Magnitude();
	mGS  = vState.vUVW.Magnitude();
	mVS  = -sin(vOutput.vEulerAngles(eTheta)) * vState.vUVW(eU) + cos(vOutput.vEulerAngles(eTheta)) * vState.vUVW(eW);

	// Failure mode
	_Failure=0;
}

// ----------------------------------------------------------------------------
// 
void FlightDynamics::calculate_state(void)
{

	if (_Failure==0)
	{
		double dt=mDt/((double)mIterations);
		for (int j=0;j<mIterations;j++) 
		{

			setBodytoNav();
			calculateFlightParameters();
			setWindtoBody();

			// Calculate Forces and Moments
			calculateGravity();
			calculateAerodynamics();
			calculatePropulsion();

			// Update Mass and Inertia
			updateMassAndInertia(dt);

			//Calculate State Derivatives
			calculateNavVelocitydot();
			calculateGeodLocationdot();
			calculatePQRdot();
			calculateQuaternionsdot(); 

			// Update State Variables
			switch(mIntegrationMethod) 
			{
				case eRectEuler:
				  vState.vNavVelocity  += dt*vNavVelocitydot;
				  vState.vGeodLocation += dt*vGeodLocationdot;
				  vState.vPQR          += dt*vPQRdot;
				  for (int i=0;i<4;i++) 
				  {
					vState.vQuaternions[i] += dt*vQuaternionsdot[i];
				  }
				break;

				case eTrapezoidal:
				  vState.vNavVelocity  += 0.5*dt*(vNavVelocitydot + last_vNavVelocitydot);
				  vState.vGeodLocation += 0.5*dt*(vGeodLocationdot + last_vGeodLocationdot);
				  vState.vPQR          += 0.5*dt*(vPQRdot + last_vPQRdot);
				  for (int i=0;i<4;i++) 
				  {
					vState.vQuaternions[i] += 0.5*dt*(vQuaternionsdot[i] + last_vQuaternionsdot[i]);
				  }  
				break;

				case eAdamsBashforth2: 
				  vState.vNavVelocity  += dt*(1.5*vNavVelocitydot - 0.5*last_vNavVelocitydot);
				  vState.vGeodLocation += dt*(1.5*vGeodLocationdot - 0.5*last_vGeodLocationdot);
				  vState.vPQR          += dt*(1.5*vPQRdot - 0.5*last_vPQRdot);
				  for (int i=0;i<4;i++)
				  {
					vState.vQuaternions[i] += dt*(1.5*vQuaternionsdot[i] - 0.5*last_vQuaternionsdot[i]);
				  }
				break;

				case eAdamsBashforth3: 
				   vState.vNavVelocity  += (1/12.0)*dt*(23.0*vNavVelocitydot - 16.0*last_vNavVelocitydot + 5.0*last2_vNavVelocitydot);
				   vState.vGeodLocation += (1/12.0)*dt*(23.0*vGeodLocationdot - 16.0*last_vGeodLocationdot + 5.0*last2_vGeodLocationdot);
				   vState.vPQR          += (1/12.0)*dt*(23.0*vPQRdot - 16.0*last_vPQRdot + 5.0*last2_vPQRdot);
				   for (int i=0;i<4;i++) 
				   {
					 vState.vQuaternions[i] += (1/12.0)*dt*(23.0*vQuaternionsdot[i] - 16.0*last_vQuaternionsdot[i] + 5.0*last2_vQuaternionsdot[i]);
				   }   
				break;

				case eNone:

				break;
				}
		} //end for (int j=0;j<mIterations;j++)

		// Handle Geodesic coordinates periodicity
		// -Pi < Longitude < Pi
		if(vState.vGeodLocation(eLong)< -M_PI) {vState.vGeodLocation(eLong) += 2*M_PI;}
		if(vState.vGeodLocation(eLong)> M_PI) {vState.vGeodLocation(eLong) -= 2*M_PI;}

		// Update p, q, r with earth rotation rate
		vOmegaNav = vOmegaEarth + vOmegaNavRelative;
		vState.vPQR = vState.vPQR + mNavtoBody*vOmegaNav;

		// Update the Euler Angles values(using the DCM)
		vState.vEulerAngles(eTheta) = asin(-mBodytoNav(3,1));
		vState.vEulerAngles(ePhi) = atan(mBodytoNav(3,2)/mBodytoNav(3,3));
		// quaternions only give values between -PI/2 and PI/2 so we need to handle the offset for psi
		double psi_correction = atan(mBodytoNav(2,1)/mBodytoNav(1,1)) - vState.vEulerAngles(ePsi);
		while  ( psi_correction > M_PI/3) psi_correction -= M_PI/2;
		while  ( psi_correction <-M_PI/3) psi_correction += M_PI/2;
		vState.vEulerAngles(ePsi) += psi_correction;

		// then handle periodicity
		if     (vState.vEulerAngles(ePsi) >= 2*M_PI) vState.vEulerAngles(ePsi) -= 2*M_PI;
		else if(vState.vEulerAngles(ePsi) <  0     ) vState.vEulerAngles(ePsi) += 2*M_PI;


		// Update u, v, w
		vState.vUVW = mNavtoBody * vState.vNavVelocity;

		// Set past values  
		last2_vNavVelocitydot = last_vNavVelocitydot;
		last_vNavVelocitydot = vNavVelocitydot;
		last2_vGeodLocationdot = last_vGeodLocationdot;
		last_vGeodLocationdot = vGeodLocationdot;
		last2_vPQRdot = last_vPQRdot;
		last_vPQRdot = vPQRdot;

		for (int i=0;i<4;i++) 
		{
			last2_vQuaternionsdot[i] = last_vQuaternionsdot[i];
			last_vQuaternionsdot[i] = vQuaternionsdot[i];
		}
	} 
}

// ----------------------------------------------------------------------------
//
void FlightDynamics::calculate_output(void)
{
	if (_Failure==0)
	{
		vOutput.vGeodLocation = vState.vGeodLocation;
		vOutput.vEulerAngles  = vState.vEulerAngles;
		vOutput.vNavVelocity  = vState.vNavVelocity;
		vOutput.vUVW          = vState.vUVW;
		vOutput.vPQR          = vState.vPQR;

		calculateFlightParameters();

		mCAS = sqrt(SHRatio * Reng * SLtemperature) * sqrt(5 * ( pow(mQbar/SLpressure + 1.0,2.0/7.0) - 1.0 ) );
		mIAS = mCAS; // Use measured mQbar
		mEAS = sqrt(2 * mQbar / SLdensity); // Use measured mQbar
		mTAS = vAeroUVW.Magnitude();
		mGS  = vState.vNavVelocity.Magnitude(eNorth,eEast);
		mVS  = vState.vNavVelocity(eDown); // negative when Aircraft climbs

		vOutput.vAcceleration = vUVWdot + vState.vPQR * vState.vUVW;
		
		#ifdef DEBUG_FLIGH_DYNAMICS
		cout << "Flight Dynamics : GEOD POSITION (deg and m)" << endl;
		cout << "Longitude : " <<  vOutput.vGeodLocation(eLong)*radtodeg << endl;
		cout << "Latitude :  " <<  vOutput.vGeodLocation(eLat)*radtodeg << endl;
		cout << "Height :    " <<  vOutput.vGeodLocation(eH) << endl;  
		cout << "Flight Dynamics : ORIENTATION (deg)" << endl;
		cout << "Phi :       " <<  vOutput.vEulerAngles(ePhi)*180/M_PI << endl;
		cout << "Theta :     " <<  vOutput.vEulerAngles(eTheta)*180/M_PI << endl;
		cout << "Psi :       " <<  vOutput.vEulerAngles(ePsi)*180/M_PI << endl;
		cout << "Flight Dynamics : NAVIGATION SPEEDS (m/s)" << endl;
		cout << "Vnorth :    " <<  vOutput.vNavVelocity(eNorth) << endl;
		cout << "Veast :     " <<  vOutput.vNavVelocity(eEast) << endl;
		cout << "Vdown :     " <<  vOutput.vNavVelocity(eDown) << endl;
		cout << "Flight Dynamics : Speeds (m/s)" << endl;
		cout << "IAS :       " <<  mIAS << endl;
		cout << "TAS :       " <<  mTAS << endl;
		#endif
	}
}

// ----------------------------------------------------------------------------
// Set the DCM (Direction Cosine Matrix)
void FlightDynamics::setBodytoNav(void)
{
	double e0, e1, e2, e3;

	e0 = vState.vQuaternions[0];
	e1 = vState.vQuaternions[1];
	e2 = vState.vQuaternions[2];
	e3 = vState.vQuaternions[3];

	mBodytoNav(1,1) = pow(e0,2) + pow(e1,2) - pow(e2,2) - pow(e3,2);
	mBodytoNav(1,2) = 2*(e1*e2 - e0*e3);
	mBodytoNav(1,3) = 2*(e0*e2 + e1*e3);
	mBodytoNav(2,1) = 2*(e1*e2 + e0*e3);
	mBodytoNav(2,2) = pow(e0,2) - pow(e1,2) + pow(e2,2) - pow(e3,2);
	mBodytoNav(2,3) = 2*(e2*e3 - e0*e1);
	mBodytoNav(3,1) = 2*(e1*e3 - e0*e2);
	mBodytoNav(3,2) = 2*(e2*e3 + e0*e1);
	mBodytoNav(3,3) = pow(e0,2) - pow(e1,2) - pow(e2,2) + pow(e3,2);

	mNavtoBody = mBodytoNav.Transposed();
}

// ----------------------------------------------------------------------------
// Set the matrix from Wind to Body frame
void FlightDynamics::setWindtoBody(void)
{
	double ca, cb, sa, sb;

	ca = cos(mAlpha);
	sa = sin(mAlpha);
	cb = cos(mBeta);
	sb = sin(mBeta);

	mWindtoBody(1,1) =  ca*cb;
	mWindtoBody(1,2) = -ca*sb;
	mWindtoBody(1,3) = -sa;
	mWindtoBody(2,1) =  sb;
	mWindtoBody(2,2) =  cb;
	mWindtoBody(2,3) =  0.0;
	mWindtoBody(3,1) =  sa*cb;
	mWindtoBody(3,2) = -sa*sb;
	mWindtoBody(3,3) =  ca;

	mBodytoWind = mWindtoBody.Transposed();  
}  

// ----------------------------------------------------------------------------
// Calculate the speed of the aircraft in the navigation frame (NED)
void FlightDynamics::calculateGeodLocationdot(void)
{
	double latitude = vState.vGeodLocation(eLat);
	double flight_radius = gEarth.getRadius(latitude) + vState.vGeodLocation(eH);
	vGeodLocationdot(eLat)  = vState.vNavVelocity(eNorth)/flight_radius;
	vGeodLocationdot(eLong) = vState.vNavVelocity(eEast)/(cos(latitude)*flight_radius);
	vGeodLocationdot(eH)    = -vState.vNavVelocity(eDown);
}

// ----------------------------------------------------------------------------
//
void FlightDynamics::calculateQuaternionsdot(void)
{
	double e0, e1, e2, e3, p, q, r, lambda;

	e0 = vState.vQuaternions[0];
	e1 = vState.vQuaternions[1];
	e2 = vState.vQuaternions[2];
	e3 = vState.vQuaternions[3];
	p  = vState.vPQR(eP);
	q  = vState.vPQR(eQ);
	r  = vState.vPQR(eR);

	// Correction factor
	lambda = 1 -(pow(e0,2) + pow(e1,2) + pow(e2,2) + pow(e3,2));

	// Computation of quaternionsdot
	vQuaternionsdot[0] = -(e1*p + e2*q + e3*r)/2 + lambda * e0;
	vQuaternionsdot[1] = (e0*p + e2*r - e3*q)/2 + lambda * e1;
	vQuaternionsdot[2] = (e0*q + e3*p - e1*r)/2 + lambda * e2;
	vQuaternionsdot[3] = (e0*r + e1*q - e2*p)/2 + lambda * e3;
}

// ----------------------------------------------------------------------------
//
void FlightDynamics::calculateNavVelocitydot(void)
{
	double latitude = vState.vGeodLocation(eLat);
	double flight_radius = gEarth.getRadius(latitude) + vState.vGeodLocation(eH);

	vForces = mBodytoNav * (vAerodynamicForces + vPropulsionForces);

	vOmegaEarth(eNorth) = gEarth.getOmega() * cos(latitude);
	// vOmegaEarth(eEast) = 0; // Already set
	vOmegaEarth(eDown)  = -gEarth.getOmega() * sin(latitude);

	vOmegaNavRelative(eNorth) = vState.vNavVelocity(eEast)/flight_radius;
	vOmegaNavRelative(eEast)  = -vState.vNavVelocity(eNorth)/flight_radius;
	vOmegaNavRelative(eDown)  = -vState.vNavVelocity(eEast)*tan(latitude)/flight_radius;

	vNavVelocitydot = vForces/mMass + vGravityForces -(2*vOmegaEarth + vOmegaNavRelative) * vState.vNavVelocity ;
	vUVWdot = mNavtoBody * vNavVelocitydot;
}

// ----------------------------------------------------------------------------
//
void FlightDynamics::calculatePQRdot(void)
{
	vMoments = vAerodynamicMoments + vPropulsionMoments;
	// Compute body frame rotational accelerations based on the current body
	// moments and the total inertial angular velocity expressed in the body
	// frame.
	vPQRdot = mInvInertia * (vMoments - vState.vPQR * (mInertia * vState.vPQR));
}
  
// ----------------------------------------------------------------------------
// Calculate main flight parameters : Mach, Alpha, Beta...
void FlightDynamics::calculateFlightParameters(void)
{
	vAeroUVW = vState.vUVW - vTotalWindBody;
	vAeroPQR = vState.vPQR + vTotalWindRateBody;

	mQbar     = 0.5* mDensity * pow(vAeroUVW.Magnitude(),2.0);
	mMach     = vAeroUVW.Magnitude()/mSoundSpeed;
	mAlpha    = atan2(vAeroUVW(eW),vAeroUVW(eU));
	mBeta     = asin(vAeroUVW(eV)/vAeroUVW.Magnitude());
	mAlphadot = (vAeroUVW(eU)*vUVWdot(eW) - vAeroUVW(eW)*vUVWdot(eU))
			  /(vAeroUVW(eU)*vAeroUVW(eU) + vAeroUVW(eW)*vAeroUVW(eW));
}

// ----------------------------------------------------------------------------
// Calculate the gravity forces exerced on the aircraft taking into account
// the position of the aircraft (Earth Geodetic model) and express them in 
// the Navigation frame.
void FlightDynamics::calculateGravity(void)
{
	// Local value of g
	double localG = gEarth.getG(vState.vGeodLocation(eLat),vState.vGeodLocation(eH));
	// vGravityForces(eNorth) = 0; // Already set
	// vGravityForces(eEast) = 0; // Already set
	vGravityForces(eDown) = localG;
}

// ----------------------------------------------------------------------------
// Calculate the Aerodynamic forces and moments exerced on the aircraft in 
// the Body frame.
void FlightDynamics::calculateAerodynamics(void)
{
	double vAeroModule = vAeroUVW.Magnitude();
	double QS  = mQbar * wingarea;
	double QSb = mQbar * wingarea * wingspan;    
	double QSc = mQbar * wingarea * chord;  

	// Calculate Lift, Side and Drag in Stability axes
	// LIFT
	double lift = 0.0;
	lift += QS * getCoeff2D(CLalphaTable,mAlpha,0.0);
	lift += QS * CLde * (mDeltaLeftElevatorRad + mDeltaRightElevatorRad)/2.0;

	// SIDE
	double side = 0.0;
	side += QS * getCoeff1D(CYbTable,mBeta);

	// DRAG
	double drag = 0.0;
	drag += QS * CD0;
	drag += QS * getCoeff2D(CDalphaTable,mAlpha,0.0);
	drag += QS * CDde * (mDeltaLeftElevatorRad + mDeltaRightElevatorRad)/2.0;
	drag += QS * CDbeta * mBeta;
	//drag += CDgear...;
	//drag += CDspeedbrake...;

	// Set Aerodynamic Forces in body frame (rotation Alpha from stability frame)
	vAerodynamicForces(eX) = lift*sin(mAlpha) - drag*cos(mAlpha);
	vAerodynamicForces(eY) = side;
	vAerodynamicForces(eZ) = -lift*cos(mAlpha) - drag*sin(mAlpha);

	// Calculate Roll, Pitch and Yaw in Stability axes

	// ROLL
	double roll = 0.0;
	roll += QSb * getCoeff1D(ClbTable,mBeta); 
	roll += QSb * wingspan /(2 * vAeroModule) * Clp * vAeroPQR(eP);
	roll += QSb * wingspan /(2 * vAeroModule) * Clr * vAeroPQR(eR);
	roll += QSb * Clda * (mDeltaLeftAileronRad - mDeltaRightAileronRad)/2.0;
	roll += QSb * Cldr * mDeltaRudderRad;

	// cout << "Roll beta :    " << QSb * getCoeff1D(ClbTable,mBeta) << endl;
	// cout << "Roll P :       " << QSb * wingspan /(2 * vAeroModule) * Clp * vAeroPQR(eP) << endl;
	// cout << "Roll R :       " << QSb * wingspan /(2 * vAeroModule) * Clr * vAeroPQR(eR) << endl;
	// cout << "Roll Rudder :  " << QSb * Cldr * mDeltaRudderRad << endl;
	// cout << "TOTAL MOMENT : " << roll << endl;

	// PITCH
	double pitch = 0.0;
	pitch += QSc * getCoeff1D(Cm0Table,0.0);
	pitch += QSc * Cmalpha * mAlpha;
	pitch += QSc * Cmde * (mDeltaLeftElevatorRad + mDeltaRightElevatorRad)/2.0;
	pitch += QSc * chord /(2 * vAeroModule) * Cmq * vAeroPQR(eQ);
	pitch += QSc * chord /(2 * vAeroModule) * Cmalphadot * mAlphadot;

	// YAW
	double yaw = 0.0;
	yaw += QSb * wingspan /(2 * vAeroModule) * Cnr * vAeroPQR(eR);
	yaw += QSb * Cnb * mBeta;
	yaw += QSb * Cndr * mDeltaRudderRad;


	// cout << "Yaw R :        " << QSb * wingspan /(2 * vAeroModule) * Cnr * vAeroPQR(eR) << endl;
	// cout << "Yaw beta :     " << QSb * Cnb * mBeta << endl;
	// cout << "Yaw Rudder :   " << QSb * Cndr * mDeltaRudderRad << endl;
	// cout << "TOTAL MOMENT : " << yaw << endl;


	// Set Aerodynamic Moments in body frame (rotation Alpha from stability frame)
	vAerodynamicMoments(eX) = roll*cos(mAlpha) - yaw*sin(mAlpha);
	vAerodynamicMoments(eY) = pitch; //+lift*CGtoP*cos(mAlpha) + drag*CGtoP*sin(mAlpha)
	vAerodynamicMoments(eZ) = yaw*cos(mAlpha) + roll*sin(mAlpha); //-side*CGtoP

}

// ----------------------------------------------------------------------------
// Calculate the Propulsion forces and moments exerced on the aircraft in 
// the Body frame.
void FlightDynamics::calculatePropulsion(void)
{
	FGColumnVector3 thrustLeftEngine(mThrustLeftEngine,0.0,0.0);
	FGColumnVector3 thrustRightEngine(mThrustRightEngine,0.0,0.0);
	vPropulsionForces  = thrustLeftEngine + thrustRightEngine;
	vPropulsionMoments = LeftEnginePositiontoCG * thrustLeftEngine + RightEnginePositiontoCG * thrustRightEngine;
}

// ----------------------------------------------------------------------------
// Update the Mass and the Inertia of the Aircraft depending on the fuel consumption.
// dt : Time elapsed since the last update
void FlightDynamics::updateMassAndInertia(double dt)
{
	mMass -= SFC * dt * (mThrustRightEngine + mThrustLeftEngine);
	mTankFilling = (mMass - EmptyMass) / (FullMass - EmptyMass);
	mInertia = mInertia = mTankFilling * mFullInertia + (1 - mTankFilling) * mEmptyInertia;
	mInvInertia = mInertia.Inverse();

	// Optional check
	if (mMass <= EmptyMass) 
	{
		// No fuel left
		mThrustRightEngine = 0.0;
		mThrustLeftEngine = 0.0;
	}
}

// ----------------------------------------------------------------------------
//
double FlightDynamics::getCoeff1D(vector< vector<double> > &coeff, double var1) 
{
	double lambda, value;
	int rows = coeff.size();
	int r = 0;

	while(r < rows && coeff[r][0] < var1) {r++;}

	if (r == 0) {r = 1;}
	if (r == rows) {r--;}

	lambda = (var1 - coeff[r-1][0])/(coeff[r][0] - coeff[r-1][0]);
	value  = coeff[r-1][1] + lambda*(coeff[r][1] - coeff[r-1][1]);

	return value;
}


// ----------------------------------------------------------------------------
//
double FlightDynamics::getCoeff2D(vector< vector<double> > &coeff, double var1, double var2) 
{
	double lambda1, lambda2, value;
	int rows = coeff.size();
	int columns = coeff[0].size();
	int r = 1;
	int c = 1;

	while(r < rows && coeff[r][0] < var1) {r++;}
	while(c < columns && coeff[0][c] < var2) {c++;}

	if (r == 1) {r = 2;}
	if (r == rows) {r--;}
	if (c == 1) {c = 2;}
	if (c == columns) {c--;}

	lambda1 = (var1 - coeff[r-1][0])/(coeff[r][0] - coeff[r-1][0]);
	lambda2 = (var2 - coeff[0][c-1])/(coeff[0][c] - coeff[0][c-1]);

	value  = (1.0 - lambda1) * (1.0 - lambda2) * coeff[r-1][c-1];
	value +=       (lambda1) * (1.0 - lambda2) * coeff[r][c-1]; 
	value += (1.0 - lambda1) *       (lambda2) * coeff[r-1][c]; 
	value +=       (lambda1) *       (lambda2) * coeff[r][c];

	return value;
}
