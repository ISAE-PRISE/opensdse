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
// This file has been copied from FlighGear 2016.1.1 version
// Original file (which has been slightly modified here) is:
// 
// net_fdm.hxx -- defines a common net I/O interface to the flight
//                dynamics model
//
// Written by Curtis Olson - http://www.flightgear.org/~curt
// Started September 2001.
//
// Modified per Jb Chaudron for Open SDSE project: March 2017
// ----------------------------------------------------------------------------

#ifndef __VISUALIZATION_HH_DEF__
#define __VISUALIZATION_HH_DEF__

#include <iostream>
#include <memory>
#include <string>
#include <string.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "A320.hh"
#include "Common.hh" 

// For Flightgear Net FDM
#define DEG2RAD (3.14159 / 180.0)
#define FEET2METER 0.3048
#define METER2FEET 3.2808399
#define MS2KTS 1,94384
const uint32_t FG_NET_FDM_VERSION = 24;

// For Flightgear Net ctrl
#define RESERVED_SPACE 25
const uint32_t FG_NET_CTRLS_VERSION = 27;

// Useful typedef for socket definition
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;

// ----------------------------------------------------------------------------
// Define a structure containing the top level flight dynamics 
// model parameters for FlighGear
// ----------------------------------------------------------------------------
class FGNetFDM {

public:

    enum {
        FG_MAX_ENGINES = 4,
        FG_MAX_WHEELS = 3,
        FG_MAX_TANKS = 4
    };

    uint32_t version;		// increment when data values change
    uint32_t padding;		// padding

    // Positions
    double longitude;		// geodetic (radians)
    double latitude;		// geodetic (radians)
    double altitude;		// above sea level (meters)
    float agl;			// above ground level (meters)
    float phi;			// roll (radians)
    float theta;		// pitch (radians)
    float psi;			// yaw or true heading (radians)
    float alpha;                // angle of attack (radians)
    float beta;                 // side slip angle (radians)

    // Velocities
    float phidot;		// roll rate (radians/sec)
    float thetadot;		// pitch rate (radians/sec)
    float psidot;		// yaw rate (radians/sec)
    float vcas;		        // calibrated airspeed
    float climb_rate;		// feet per second
    float v_north;              // north velocity in local/body frame, fps
    float v_east;               // east velocity in local/body frame, fps
    float v_down;               // down/vertical velocity in local/body frame, fps
    float v_body_u;    // ECEF velocity in body frame
    float v_body_v;    // ECEF velocity in body frame 
    float v_body_w;    // ECEF velocity in body frame
    
    // Accelerations
    float A_X_pilot;		// X accel in body frame ft/sec^2
    float A_Y_pilot;		// Y accel in body frame ft/sec^2
    float A_Z_pilot;		// Z accel in body frame ft/sec^2

    // Stall
    float stall_warning;        // 0.0 - 1.0 indicating the amount of stall
    float slip_deg;		// slip ball deflection

    // Pressure
    
    // Engine status
    uint32_t num_engines;	     // Number of valid engines
    uint32_t eng_state[FG_MAX_ENGINES];// Engine state (off, cranking, running)
    float rpm[FG_MAX_ENGINES];	     // Engine RPM rev/min
    float fuel_flow[FG_MAX_ENGINES]; // Fuel flow gallons/hr
    float fuel_px[FG_MAX_ENGINES];   // Fuel pressure psi
    float egt[FG_MAX_ENGINES];	     // Exhuast gas temp deg F
    float cht[FG_MAX_ENGINES];	     // Cylinder head temp deg F
    float mp_osi[FG_MAX_ENGINES];    // Manifold pressure
    float tit[FG_MAX_ENGINES];	     // Turbine Inlet Temperature
    float oil_temp[FG_MAX_ENGINES];  // Oil temp deg F
    float oil_px[FG_MAX_ENGINES];    // Oil pressure psi

    // Consumables
    uint32_t num_tanks;		// Max number of fuel tanks
    float fuel_quantity[FG_MAX_TANKS];

    // Gear status
    uint32_t num_wheels;
    uint32_t wow[FG_MAX_WHEELS];
    float gear_pos[FG_MAX_WHEELS];
    float gear_steer[FG_MAX_WHEELS];
    float gear_compression[FG_MAX_WHEELS];

    // Environment
    uint32_t cur_time;           // current unix time
                                 // FIXME: make this uint64_t before 2038
    int32_t warp;                // offset in seconds to unix time
    float visibility;            // visibility in meters (for env. effects)

    // Control surface positions (normalized values)
    float elevator;
    float elevator_trim_tab;
    float left_flap;
    float right_flap;
    float left_aileron;
    float right_aileron;
    float rudder;
    float nose_wheel;
    float speedbrake;
    float spoilers;
};

class FGNetCtrls {

public:

    enum {
        FG_MAX_ENGINES = 4,
        FG_MAX_WHEELS = 16,
        FG_MAX_TANKS = 8
    };

    uint32_t version;		         // increment when data values change

    // Aero controls
    double aileron;		         // -1 ... 1
    double elevator;		         // -1 ... 1
    double rudder;		         // -1 ... 1
    double aileron_trim;	         // -1 ... 1
    double elevator_trim;	         // -1 ... 1
    double rudder_trim;		         // -1 ... 1
    double flaps;		         //  0 ... 1
    double spoilers;
    double speedbrake;

    // Aero control faults
    uint32_t flaps_power;                 // true = power available
    uint32_t flap_motor_ok;

    // Engine controls
    uint32_t num_engines;		 // number of valid engines
    uint32_t master_bat[FG_MAX_ENGINES];
    uint32_t master_alt[FG_MAX_ENGINES];
    uint32_t magnetos[FG_MAX_ENGINES];
    uint32_t starter_power[FG_MAX_ENGINES];// true = starter power
    double throttle[FG_MAX_ENGINES];     //  0 ... 1
    double mixture[FG_MAX_ENGINES];      //  0 ... 1
    double condition[FG_MAX_ENGINES];    //  0 ... 1
    uint32_t fuel_pump_power[FG_MAX_ENGINES];// true = on
    double prop_advance[FG_MAX_ENGINES]; //  0 ... 1
    uint32_t feed_tank_to[4];
    uint32_t reverse[4];


    // Engine faults
    uint32_t engine_ok[FG_MAX_ENGINES];
    uint32_t mag_left_ok[FG_MAX_ENGINES];
    uint32_t mag_right_ok[FG_MAX_ENGINES];
    uint32_t spark_plugs_ok[FG_MAX_ENGINES];  // false = fouled plugs
    uint32_t oil_press_status[FG_MAX_ENGINES];// 0 = normal, 1 = low, 2 = full fail
    uint32_t fuel_pump_ok[FG_MAX_ENGINES];

    // Fuel management
    uint32_t num_tanks;                      // number of valid tanks
    uint32_t fuel_selector[FG_MAX_TANKS];    // false = off, true = on
    uint32_t xfer_pump[5];                   // specifies transfer from array
                                             // value tank to tank specified by
                                             // int value
    uint32_t cross_feed;                     // false = off, true = on

    // Brake controls
    double brake_left;
    double brake_right;
    double copilot_brake_left;
    double copilot_brake_right;
    double brake_parking;
    
    // Landing Gear
    uint32_t gear_handle; // true=gear handle down; false= gear handle up

    // Switches
    uint32_t master_avionics;
    
        // nav and Comm
    double	comm_1;
    double	comm_2;
    double	nav_1;
    double	nav_2;

    // wind and turbulance
    double wind_speed_kt;
    double wind_dir_deg;
    double turbulence_norm;

    // temp and pressure
    double temp_c;
    double press_inhg;

    // other information about environment
    double hground;		         // ground elevation (meters)
    double magvar;		         // local magnetic variation in degs.

    // hazards
    uint32_t icing;                      // icing status could me much
                                         // more complex but I'm
                                         // starting simple here.

    // simulation control
    uint32_t speedup;		         // integer speedup multiplier
    uint32_t freeze;		         // 0=normal
				         // 0x01=master
				         // 0x02=position
				         // 0x04=fuel

    // --- New since FlightGear 0.9.10 (FG_NET_CTRLS_VERSION = 27)

    // --- Add new variables just before this line.

    uint32_t reserved[RESERVED_SPACE];	 // 100 bytes reserved for future use.
};

class Visualization: public A320, public UnitConversion
{
	private:
	
		FGNetFDM _FlighGearFDM;
		SOCKET _FlightGearTxSocket;
		SOCKADDR_IN _FlightGearTxAddr;
		
		FGNetCtrls _FlighGearCTRL;
		SOCKET _FlightGearTxCtrlSocket;
		SOCKADDR_IN _FlightGearTxCtrlAddr;
		
	public:
	
		Visualization();
		~Visualization();	
		void FlighGearSocketSend();
		void setLongitude(double val);
		void setLatitude(double val);
		void setAltitude(double val);
		void setPhi(double val);
		void setTheta(double val);
		void setPsi(double val);
		void setUspeed(double val);
		void setVspeed(double val);
		void setWspeed(double val);
		void setXacc(double val);
		void setYacc(double val);
		void setZacc(double val);
		void setVcas(double val);
		void setAlpha(double val);
		void setBeta(double val);
		void setRightAileron(double val);
		void setLeftAileron(double val);
		void setElevator(double val);
		void setRudder(double val);
		void setFlaps(double val);
		void setSpoilers(double val);
		void setGears(double val);
};

// Network to Host and Host to Network conversion
// for double and float values
double htond (double x) ;
float htonf (float x);

#endif // VISUALIZATION_HH_DEF
