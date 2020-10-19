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

#include <cmath>
#include "EarthModel.hh"

// ----------------------------------------------------------------------------
// Constructor
EarthModel::EarthModel() 
{
	a = 6378137;           // Semi-major axis (m)
	// f = 1/298.257223563;   // Flattening factor
	b = 6356752;           // Semi-minor axis (m)
	// e = 0.08181919;        // The first eccentricity
	Omega = 0.00007292115; // Mean Angular speed (rad/s)
}

// ----------------------------------------------------------------------------
// Destructor
EarthModel::~EarthModel() 
{
	
}

// ----------------------------------------------------------------------------
// Return the radius of the earth at the provided Latitude
// -> Latitude : Latitude in radians
double EarthModel::getRadius(double Latitude) 
{
	return sqrt( pow(a*cos(Latitude),2) + pow(b*sin(Latitude),2) );
}

// ----------------------------------------------------------------------------
//Return the value of g (scalar) at the provided Latitude and Height
// -> Latitude : Latitude in radians
// -> H : Height in meters

double EarthModel::getG(double Latitude, double H) 
{
	return 9.780327*(1 
			+5.302E-3*pow(sin(Latitude),2) 
			-5.8E-6*pow(sin(2*Latitude),2) 
			-3.086E-7*H);
}

// ----------------------------------------------------------------------------
// Return the angular speed of Earth
double EarthModel::getOmega() 
{
	return Omega;
}
