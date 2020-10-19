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

#ifndef __COCKPIT_HH_DEF__
#define __COCKPIT_HH_DEF__

#include "UnitConversion.hh"
#include <cmath>
#include <iostream>

using namespace std;
using std::cout;

// ----------------------------------------------------------------------------
// Contain the state of several variables regarding the cockpit
// ----------------------------------------------------------------------------
class Cockpit : public UnitConversion
{
  
private:

  bool COCKPIT_AUTOPILOT_AP_ACTIVE;
  bool COCKPIT_AUTOPILOT_ATHR_ACTIVE;
  double COCKPIT_AUTOPILOT_SPD;
  double COCKPIT_AUTOPILOT_HDG;
  double COCKPIT_AUTOPILOT_ALT;
  double COCKPIT_AUTOPILOT_VS;
  
  double AIRCRAFT_POSITION_LONGITUDE;
  double AIRCRAFT_POSITION_LATITUDE;
  double AIRCRAFT_POSITION_ALTITUDE;
  
  double AIRCRAFT_ORIENTATION_THETA;
  double AIRCRAFT_ORIENTATION_PHI;
  double AIRCRAFT_ORIENTATION_PSI;
  
  double AIRCRAFT_SPEED_IAS;
  double AIRCRAFT_SPEED_MACH;
  
  double ACTUATORS_RIGHT_ENGINE_THRUST;
  double ACTUATORS_LEFT_ENGINE_THRUST;
  
  double ENVIRONMENT_VARIABLES_PRESSURE;

  double AIRCRAFT_ADDITIONAL_ALPHA;
  double AIRCRAFT_ADDITIONAL_BETA;
  double AIRCRAFT_ADDITIONAL_DYNAMIC_PRESSURE;
  
  

public:

  // Constructor
  Cockpit();
  // Destructor
  ~Cockpit();

  // Get functions

  bool GET_COCKPIT_AUTOPILOT_AP_ACTIVE()        {return  COCKPIT_AUTOPILOT_AP_ACTIVE;};
  bool GET_COCKPIT_AUTOPILOT_ATHR_ACTIVE()      {return  COCKPIT_AUTOPILOT_ATHR_ACTIVE;};
  double GET_COCKPIT_AUTOPILOT_SPD()          {return  COCKPIT_AUTOPILOT_SPD;};
  double GET_COCKPIT_AUTOPILOT_HDG()          {return  COCKPIT_AUTOPILOT_HDG;};
  double GET_COCKPIT_AUTOPILOT_ALT()          {return  COCKPIT_AUTOPILOT_ALT;};
  double GET_COCKPIT_AUTOPILOT_VS()          {return  COCKPIT_AUTOPILOT_VS;};
  
  double GET_AIRCRAFT_POSITION_LONGITUDE()        {return  AIRCRAFT_POSITION_LONGITUDE;};
  double GET_AIRCRAFT_POSITION_LATITUDE()          {return  AIRCRAFT_POSITION_LATITUDE;};
  double GET_AIRCRAFT_POSITION_ALTITUDE()          {return  AIRCRAFT_POSITION_ALTITUDE;};
  
  double GET_AIRCRAFT_ORIENTATION_THETA()        {return  AIRCRAFT_ORIENTATION_THETA;};
  double GET_AIRCRAFT_ORIENTATION_PHI()        {return  AIRCRAFT_ORIENTATION_PHI;};
  double GET_AIRCRAFT_ORIENTATION_PSI()        {return  AIRCRAFT_ORIENTATION_PSI;};
  
  double GET_AIRCRAFT_SPEED_IAS()            {return  AIRCRAFT_SPEED_IAS;};
  double GET_AIRCRAFT_SPEED_MACH()          {return  AIRCRAFT_SPEED_MACH;};
  
  double GET_ACTUATORS_RIGHT_ENGINE_THRUST()        {return  ACTUATORS_RIGHT_ENGINE_THRUST;};
  double GET_ACTUATORS_LEFT_ENGINE_THRUST()        {return  ACTUATORS_LEFT_ENGINE_THRUST;};
  
  double GET_ENVIRONMENT_VARIABLES_PRESSURE()        {return  ENVIRONMENT_VARIABLES_PRESSURE;};

  double GET_AIRCRAFT_ADDITIONAL_ALPHA()         {return AIRCRAFT_ADDITIONAL_ALPHA;} ;
  double GET_AIRCRAFT_ADDITIONAL_BETA()         {return AIRCRAFT_ADDITIONAL_BETA;} ;
  double GET_AIRCRAFT_ADDITIONAL_DYNAMIC_PRESSURE()        {return AIRCRAFT_ADDITIONAL_DYNAMIC_PRESSURE;} ;


  // Set functions

  void SET_COCKPIT_AUTOPILOT_AP_ACTIVE(bool input)        {COCKPIT_AUTOPILOT_AP_ACTIVE = input;};
  void SET_COCKPIT_AUTOPILOT_ATHR_ACTIVE(bool input)        {COCKPIT_AUTOPILOT_ATHR_ACTIVE = input;};
  void SET_COCKPIT_AUTOPILOT_SPD(double input)          {COCKPIT_AUTOPILOT_SPD = input;};
  void SET_COCKPIT_AUTOPILOT_HDG(double input)          {COCKPIT_AUTOPILOT_HDG = input;};
  void SET_COCKPIT_AUTOPILOT_ALT(double input)          {COCKPIT_AUTOPILOT_ALT = input;};
  void SET_COCKPIT_AUTOPILOT_VS(double input)            {COCKPIT_AUTOPILOT_VS = input;};
  
  void SET_AIRCRAFT_POSITION_LONGITUDE(double input)         { AIRCRAFT_POSITION_LONGITUDE= input; };
  void SET_AIRCRAFT_POSITION_LATITUDE(double input)         { AIRCRAFT_POSITION_LATITUDE= input; };
  void SET_AIRCRAFT_POSITION_ALTITUDE(double input)         { AIRCRAFT_POSITION_ALTITUDE= input; };
  
  void SET_AIRCRAFT_ORIENTATION_THETA(double input)         { AIRCRAFT_ORIENTATION_THETA= input; };
  void SET_AIRCRAFT_ORIENTATION_PHI(double input)         { AIRCRAFT_ORIENTATION_PHI= input; };
  void SET_AIRCRAFT_ORIENTATION_PSI(double input)         { AIRCRAFT_ORIENTATION_PSI= input; };
  
  void SET_AIRCRAFT_SPEED_IAS(double input)             { AIRCRAFT_SPEED_IAS= input; };
  void SET_AIRCRAFT_SPEED_MACH(double input)               { AIRCRAFT_SPEED_MACH= input; };
  
  void SET_ACTUATORS_RIGHT_ENGINE_THRUST(double input)         { ACTUATORS_RIGHT_ENGINE_THRUST= input; };
  void SET_ACTUATORS_LEFT_ENGINE_THRUST(double input)         { ACTUATORS_LEFT_ENGINE_THRUST= input; };
  
  void SET_ENVIRONMENT_VARIABLES_PRESSURE(double input)         { ENVIRONMENT_VARIABLES_PRESSURE = input; };

  void SET_AIRCRAFT_ADDITIONAL_ALPHA(double input)         {AIRCRAFT_ADDITIONAL_ALPHA = input;} ;
  void SET_AIRCRAFT_ADDITIONAL_BETA(double input)         {AIRCRAFT_ADDITIONAL_BETA = input;} ;
  void SET_AIRCRAFT_ADDITIONAL_DYNAMIC_PRESSURE(double input)         {AIRCRAFT_ADDITIONAL_DYNAMIC_PRESSURE = input;} ;

};

#endif // __COCKPIT_HH_DEF__
