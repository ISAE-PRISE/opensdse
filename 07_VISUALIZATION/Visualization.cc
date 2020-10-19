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

#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>

#include "Visualization.hh"
#include "Common.hh"

// ----------------------------------------------------------------------------
// Constructor 
Visualization::Visualization() 
{	
	// Flighgear IP address and UDP port used
	// TO DO: The name might change when using interface to Xplane
	char * FGIPAddr = NULL;
	int FGUdpPort;
	
	// Get values from xml file
	XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);
	// Get values from xml file
	vector<string> attribute_path(3);
	attribute_path[0] = "VISUALIZATION";
	attribute_path[1] = "Flightgear_Net_Config";
	attribute_path[2] = "UDP_port";
	FGUdpPort = getIntValue(params, attribute_path);
	attribute_path[2] = "IP_Address";
	string FGIPAddrStr = getTextValue(params, attribute_path);
	FGIPAddr = (char * ) FGIPAddrStr.c_str();
	// Debug
	// cout<<"flightgear_udp_port : "<<flightgear_udp_port<<endl;
	// cout<<"flightgear_ip_address    : "<<flightgear_ip_address<<endl;
	delete params;
	
	memset(&_FlightGearTxAddr,0,sizeof(_FlightGearTxAddr));
	_FlightGearTxAddr.sin_family = AF_INET;
	_FlightGearTxAddr.sin_port = htons(FGUdpPort);
	_FlightGearTxAddr.sin_addr.s_addr = inet_addr(FGIPAddr);


	_FlightGearTxSocket = socket(AF_INET,SOCK_DGRAM,0);
	if (_FlightGearTxSocket == -1)
	{
		cout << "Visualization.cc: FlighGear Socket creation ERROR." << endl;
	}
	
	// Init FG model 
    // TO DO: We might need to change these hardcoded values
	memset(&_FlighGearFDM,0,sizeof(_FlighGearFDM));
	_FlighGearFDM.version = htonl(FG_NET_FDM_VERSION);
	_FlighGearFDM.latitude = htond((37.2) * DEG2RAD);
	_FlighGearFDM.longitude = htond((-122.385) * DEG2RAD);
	_FlighGearFDM.altitude = htond(10000.0);
	_FlighGearFDM.num_engines = htonl(4);
	_FlighGearFDM.eng_state[0] = htonl(2);
	_FlighGearFDM.eng_state[1] = htonl(2);
	_FlighGearFDM.eng_state[2] = htonl(2);
	_FlighGearFDM.eng_state[3] = htonl(2);
	_FlighGearFDM.num_tanks = htonl(1);
	_FlighGearFDM.fuel_quantity[0] = htonf(100.0);
	_FlighGearFDM.num_wheels = htonl(3);
	_FlighGearFDM.cur_time = htonl(time(0));
	_FlighGearFDM.warp = htonl(1);
	_FlighGearFDM.visibility = htonf(5000.0);
}

// ----------------------------------------------------------------------------
// Destructor
Visualization::~Visualization() 
{	
	// Nothing to do
}

// ----------------------------------------------------------------------------
// Send on dedicated Udp socket the native FDM structure 
void Visualization::FlighGearSocketSend() 
{
	int return_val;
	return_val = sendto( _FlightGearTxSocket
		   				,(char *)&_FlighGearFDM
		  				 ,sizeof(_FlighGearFDM)
		  				 ,0
		  				 ,(struct sockaddr *)&_FlightGearTxAddr
		  				 ,sizeof(_FlightGearTxAddr)
		  				);
	if (return_val == -1)
	{
		cout << "Visualization.cc : FlighGear_Socket_Send function ==> ERROR in sendto." << endl;
	}
	else
	{
		#ifdef DEBUG_VISUALIZATION
		cout << "Visualization_Fed.cc : FlighGear_Socket_Send function ==> sendto is OK." << endl;
		cout << "_FlighGear_FDM is : " << endl;
		cout << "Longitude (rad) = " << _FlighGearFDM.longitude << endl;
		cout << "Latitude (rad) = "  << _FlighGearFDM.latitude << endl;
		cout << "Altitude (meters) = "  << _FlighGearFDM.altitude << endl;
		cout << "" << endl;
		cout << "Phi (rad) = " << _FlighGearFDM.phi << endl;
		cout << "Theta (rad) = "  << _FlighGearFDM.theta << endl;
		cout << "Psi (rad) = "  << _FlighGearFDM.psi << endl;
		cout << "" << endl;
		cout << "v_body_u (fps) = " << _FlighGearFDM.v_body_u << endl;
		cout << "v_body_v (fps) = "  << _FlighGearFDM.v_body_v << endl;
		cout << "v_body_w (fps) = "  << _FlighGearFDM.v_body_w << endl;
		#endif
	}
}

// ----------------------------------------------------------------------------
//
void Visualization::setLongitude(double val)
{
	_FlighGearFDM.longitude = htond(DEG2RAD * val);
}

// ----------------------------------------------------------------------------
//
void Visualization::setLatitude(double val)
{
	_FlighGearFDM.latitude = htond(DEG2RAD * val); 
}

// ----------------------------------------------------------------------------
//
void Visualization::setAltitude(double val)
{
	_FlighGearFDM.altitude = htond(val);
}

// ----------------------------------------------------------------------------
//
void Visualization::setPhi(double val)
{
	_FlighGearFDM.phi = htonf((float) (val)); 
}

// ----------------------------------------------------------------------------
//
void Visualization::setTheta(double val)
{
	_FlighGearFDM.theta = htonf((float) (val)); 
}

// ----------------------------------------------------------------------------
//
void Visualization::setPsi(double val)
{
	_FlighGearFDM.psi = htonf((float) (val)); 
}

// ----------------------------------------------------------------------------
//
void Visualization::setUspeed(double val)
{
	_FlighGearFDM.v_body_u = htonf((float) (METER2FEET * val)); 
}

// ----------------------------------------------------------------------------
//
void Visualization::setVspeed(double val)
{
	_FlighGearFDM.v_body_v = htonf((float) (METER2FEET * val)); 
}

// ----------------------------------------------------------------------------
//
void Visualization::setWspeed(double val)
{
	_FlighGearFDM.v_body_w = htonf((float) (METER2FEET * val)); 
}

// ----------------------------------------------------------------------------
//
void Visualization::setXacc(double val)
{
	_FlighGearFDM.A_X_pilot = htonf((float) (METER2FEET * val)); 
}

// ----------------------------------------------------------------------------
//
void Visualization::setYacc(double val)
{
	_FlighGearFDM.A_Y_pilot = htonf((float) (METER2FEET * val));  
}

// ----------------------------------------------------------------------------
//
void Visualization::setZacc(double val)
{
	_FlighGearFDM.A_Z_pilot = htonf((float) (METER2FEET * val)); 
}

// ----------------------------------------------------------------------------
//
void Visualization::setVcas(double val)
{
	_FlighGearFDM.vcas = htonf((float) (val)); 
}

// ----------------------------------------------------------------------------
//
void Visualization::setAlpha(double val)
{
	_FlighGearFDM.alpha = htonf((float) (DEG2RAD * val)); 
}

// ----------------------------------------------------------------------------
//
void Visualization::setBeta(double val)
{
	_FlighGearFDM.beta = htonf((float) (DEG2RAD * val));
}

// ----------------------------------------------------------------------------
//
void Visualization::setRightAileron(double val)
{
	double normalized_value = -(val<=0)*((val - ra_minpos) / (ra_maxpos - ra_minpos)) 
							+ (val>0)*((val - ra_minpos) / (ra_maxpos - ra_minpos));
	_FlighGearFDM.right_aileron = htonf((float) normalized_value);
}

// ----------------------------------------------------------------------------
//
void Visualization::setLeftAileron(double val)
{
	double normalized_value = -(val<=0)*((val - la_minpos) / (la_maxpos - la_minpos) )
							  + (val>0)*((val - la_minpos) / (la_maxpos - la_minpos));
	_FlighGearFDM.left_aileron = htonf((float) normalized_value);  
}

// ----------------------------------------------------------------------------
//
void Visualization::setElevator(double val)
{
	_FlighGearFDM.elevator = htonf((float) (-(val<=0)*val/(el_minpos*degtorad) + (val>0)*val/(el_maxpos*degtorad)));
}

// ----------------------------------------------------------------------------
//
void Visualization::setRudder(double val)
{
	_FlighGearFDM.rudder = htonf((float) ((val<=0)*val/(ru_minpos*degtorad) - (val>0)*val/(ru_maxpos*degtorad))); 
}

// ----------------------------------------------------------------------------
//
void Visualization::setFlaps(double val)
{
	_FlighGearFDM.left_flap  = htonf((float) (val/(fl_maxpos*degtorad)));
	_FlighGearFDM.right_flap = _FlighGearFDM.left_flap;
}

// ----------------------------------------------------------------------------
//
void Visualization::setSpoilers(double val)
{
	_FlighGearFDM.spoilers = htonf((float) (val/(sp_maxpos*degtorad)));
}

// ----------------------------------------------------------------------------
void Visualization::setGears(double val)
{
	_FlighGearFDM.gear_pos[0] = htonf((float) (val/(ge_maxpos*degtorad)));
	_FlighGearFDM.gear_pos[1] = _FlighGearFDM.gear_pos[0];
	_FlighGearFDM.gear_pos[2] = _FlighGearFDM.gear_pos[0];
}

// ----------------------------------------------------------------------------
//
double htond (double x)	
{
    int * p = (int*)&x;
    int tmp = p[0];
    p[0] = htonl(p[1]);
    p[1] = htonl(tmp);

    return x;
}

// ----------------------------------------------------------------------------
//
float htonf (float x)	
{
    int * p = (int *)&x;
    *p = htonl(*p);
    return x;
}
