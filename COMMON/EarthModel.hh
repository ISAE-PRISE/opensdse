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

#ifndef __EARTH_MODEL_HH_DEF__
#define __EARTH_MODEL_HH_DEF__

// ----------------------------------------------------------------------------
// Earth model in the World Geodetic System 1984 (WGS84).
// Provides a common acces point for any parameter dealing with
// Earth modelization.
// ----------------------------------------------------------------------------
class EarthModel 
{

	public:

		// Constructor
		EarthModel();
		// Destructor
		~EarthModel();

		double getRadius(double Latitude);
		double getG(double Latitude, double H);
		double getOmega();

	protected:

		double a;     // Semi-major axis (m)
		double b;     // Semi-minor axis (m)
		// double f;     // Flattening factor
		// double e;     // The first eccentricity
		double Omega; // Mean angular speed (rad/s)

};

#endif
