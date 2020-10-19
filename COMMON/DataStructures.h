#ifndef __DATA_STRUCTURES_H__
#define __DATA_STRUCTURES_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" 
{
#endif

// ---------------------------------------------------------------------------------
// Actually it has been taken and adapted from Flightgear FDM (needs to be updated)
typedef struct meas_aircraft_fdm 
{ 
    // Positions
    double longitude;		// geodetic (radians)
    double latitude;		// geodetic (radians)
    double altitude_sl;		// above sea level (meters)
    double altitude_gl;		// above ground level (meters)
    
    double phi;				// roll (radians)
    double theta;			// pitch (radians)
    double psi;				// yaw or true heading (radians)
    
    double alpha;           // angle of attack (radians)
    double beta;            // side slip angle (radians)

    // Velocities
    double phidot;			// roll rate (radians/sec)
    double thetadot;		// pitch rate (radians/sec)
    double psidot;			// yaw rate (radians/sec)
    
	double v_body_u;    	// ECEF velocity in body frame
    double v_body_v;    	// ECEF velocity in body frame 
    double v_body_w;    	// ECEF velocity in body frame
    
    double vcas;		    // calibrated airspeed
    double vtas;		    // true airspeed
    double vias;		    // indicated airspeed
    double veas;		    // equivalent airspeed
    double climb_rate;		// feet per second
    double ground_speed;	// feet per second
    double vmach;
    double v_north;         // north velocity in local/body frame, fps
    double v_east;          // east velocity in local/body frame, fps
    double v_down;          // down/vertical velocity in local/body frame, fps
    double dyn_pressure;

    // Accelerations
    double A_X_pilot;		// X accel in body frame ft/sec^2
    double A_Y_pilot;		// Y accel in body frame ft/sec^2
    double A_Z_pilot;		// Z accel in body frame ft/sec^2
} 
meas_aircraft_fdm_t;

// ---------------------------------------------------------------------------------
typedef struct efcs_data
{ 
    double cmd_left_elevator;
	double cmd_right_elevator;
    double cmd_left_flap;
    double cmd_right_flap;
    double cmd_left_aileron;
    double cmd_right_aileron;
    double cmd_rudder;
    double cmd_stabilizer;
    double cmd_nose_wheel;
    double cmd_speedbrake;
    double cmd_spoilers;
    double cmd_gears;
    // TO DO: Move this to FADEC
    double cmd_left_engine_throttle;
    double cmd_right_engine_throttle;
} 
efcs_data_t;

// ---------------------------------------------------------------------------------
// To do: What about trimming values...
typedef struct trimming_values
{ 
    double left_engine_throttle;
	double right_engine_throttle;
	double elevator;
	double stabilizer;
	double rudder;
} 
trimming_values_t;

// ---------------------------------------------------------------------------------
// TO DO: Set all the units here
typedef struct fcu_data
{ 
    bool ap1_active;		// AP1 button is ON
    bool ap2_active;		// AP2 button is ON
	bool athr_active;		// ATHR button is ON
	bool fd1_active;		// FD1 button is ON
	bool fd2_active;		// FD1 button is ON
	bool heading_active;	// HDG V/S is ON otherwise its TRACK FPA
    double speed_mach;		// Required Mach speed 
    double speed_std;		// Required speed (unit is mach)
    double vs_speed;		// Required Vertical speed 
    double heading;			// Required Heading 
	double altitude;		// Required Altitude 
} 
fcu_data_t;

// ---------------------------------------------------------------------------------
// TO DO: Set all the units here
typedef struct joystick_data
{ 
    double ss_left_elevator;
	double ss_right_elevator;
    double ss_left_flap;
    double ss_right_flap;
    double ss_left_aileron;
    double ss_right_aileron;
    double ss_rudder;
    double ss_nose_wheel;
    double ss_speedbrake;
    double ss_spoilers;
    double ss_gears;
    // TO DO: to be moved somewhere else
    double ss_left_engine_throttle;
    double ss_right_engine_throttle;
} 
joystick_data_t;

// ---------------------------------------------------------------------------------
typedef struct bridge_data
{ 
	joystick_data_t joystick_output;
	fcu_data_t fcu_output;
	meas_aircraft_fdm_t flight_dynamics_output;
} 
bridge_data_t;

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif //__AIRCRAFT_TYPES_H__
