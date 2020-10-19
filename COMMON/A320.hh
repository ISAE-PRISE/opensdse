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
// Note: A lot of coefficients in this file has been borrowed from:
// - JsBSim: http://jsbsim.sourceforge.net/
// - Flightgear: http://www.flightgear.org/
// ----------------------------------------------------------------------------
//

#ifndef __A320_HH_DEF__
#define __A320_HH_DEF__

#include <float.h>
#include <queue>
#include <string>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include "FGColumnVector3.hh"

using std::fabs;
using std::string;
using namespace std;

#ifndef M_PI
#  define M_PI 3.1415926535897932384626433832795028841971 
#endif

#if !defined(WIN32) || defined(__GNUC__) || (defined(_MSC_VER) && (_MSC_VER >= 1300))
  using std::max;
#endif

// ----------------------------------------------------------------------------
// Provides constants of the modelization of an A320 aircraft.
// Those parameters are used for the mechanical model 
// as well as the aerodynamic model.
// ----------------------------------------------------------------------------
class A320 
{
    
	public:
		/// Constructor
		A320() {};

		/// Destructor
		~A320() {};


	protected:

		// Dimensions
		static const double wingarea; 
		static const double wingspan;
		static const double chord;    
		static const double htailarea;
		static const double htailarm; 
		static const double vtailarea; 
		static const double vtailarm;  

		// Mass and Inertia
		static const double EmptyMass;
		static const double FullMass;
		static const double EmptyIxx;
		static const double EmptyIyy;
		static const double EmptyIzz;
		static const double EmptyIxy;
		static const double EmptyIxz;
		static const double EmptyIyz;
		static const double FullIxx;
		static const double FullIyy;
		static const double FullIzz;
		static const double FullIxy;
		static const double FullIxz;
		static const double FullIyz;

		// CG Location
		static const FGColumnVector3 CG;

		// Engine location
		static const FGColumnVector3 LeftEngineLocation; 
		static const FGColumnVector3 RightEngineLocation;

		// Engine consumption
		static const double SFC; ///< specific fuel consumption

		// Aerodynamic coefficients
		// Drag 
		static const double CD0;
		static const double CDalpha[27][5]; 
		static const double CDde; 
		static const double CDbeta;
		static const double CDgear;
		static const double CDspeedbrake;

		// Side
		static const double CYb[3][2];

		// Lift
		static const double CLalpha[18][6];
		static const double CLde;

		// Roll
		static const double Clb[3][2];
		static const double Clp; 
		static const double Clr;
		static const double Clda;
		static const double Cldr;

		// Pitch
		static const double Cm0[2][2];
		static const double Cmalpha;
		static const double Cmde;
		static const double Cmq;
		static const double Cmalphadot;

		// Yaw
		static const double Cnr;
		static const double Cnb;
		static const double Cndr;

		// Control Surfaces 
		static const double ra_damping;
		static const double ra_frequency;
		static const double ra_minpos;
		static const double ra_maxpos;
		static const double ra_maxrate;

		static const double la_damping;
		static const double la_frequency;
		static const double la_minpos;
		static const double la_maxpos;
		static const double la_maxrate;

		static const double el_damping;
		static const double el_frequency;
		static const double el_minpos;
		static const double el_maxpos;
		static const double el_maxrate;

		static const double ru_damping;
		static const double ru_frequency;
		static const double ru_minpos;
		static const double ru_maxpos;
		static const double ru_maxrate;

		static const double fl_damping;
		static const double fl_frequency;
		static const double fl_minpos;
		static const double fl_maxpos;
		static const double fl_maxrate;

		static const double sp_damping;
		static const double sp_frequency;
		static const double sp_minpos;
		static const double sp_maxpos;
		static const double sp_maxrate;

		static const double st_damping;
		static const double st_frequency;
		static const double st_minpos;
		static const double st_maxpos;
		static const double st_maxrate;

		static const double ge_damping;
		static const double ge_frequency;
		static const double ge_minpos;
		static const double ge_maxpos;
		static const double ge_maxrate;

};

#endif
