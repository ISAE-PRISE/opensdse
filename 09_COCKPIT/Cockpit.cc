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

#include "Cockpit.hh"
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>


// ----------------------------------------------------------------------------
// Constructor 
Cockpit::Cockpit() 
{

	 COCKPIT_AUTOPILOT_AP_ACTIVE=false;
	 COCKPIT_AUTOPILOT_ATHR_ACTIVE=false;
	 COCKPIT_AUTOPILOT_SPD=0;
	 COCKPIT_AUTOPILOT_HDG=0;
	 COCKPIT_AUTOPILOT_ALT=0;
	 COCKPIT_AUTOPILOT_VS=0;
	 
	 AIRCRAFT_POSITION_LONGITUDE =0.0 ;
	 AIRCRAFT_POSITION_LATITUDE =0.0 ;
	 AIRCRAFT_POSITION_ALTITUDE =0.0 ;
	 
	 AIRCRAFT_ORIENTATION_THETA = 0.0 ;
	 AIRCRAFT_ORIENTATION_PHI = 0.0 ;
	 AIRCRAFT_ORIENTATION_PSI = 0.0 ;
	
	 AIRCRAFT_SPEED_IAS = 0.0 ;
	 AIRCRAFT_SPEED_MACH = 0.0 ;
	
	 ACTUATORS_RIGHT_ENGINE_THRUST = 0.0 ;
	 ACTUATORS_LEFT_ENGINE_THRUST = 0.0 ;
	
	 ENVIRONMENT_VARIABLES_PRESSURE = 0.0 ;

     AIRCRAFT_ADDITIONAL_ALPHA = 0.0 ;
     AIRCRAFT_ADDITIONAL_BETA = 0.0 ;
     AIRCRAFT_ADDITIONAL_DYNAMIC_PRESSURE = 0.0 ;
	 
}

// ----------------------------------------------------------------------------
// Destructor 
Cockpit::~Cockpit()
{
	// Nothing to do
}
