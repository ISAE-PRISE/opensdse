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

#include "EfcsFederateHla13.hh"

using namespace std;

// ----------------------------------------------------------------------------
// EfcsFederateHla13 Constructor
EfcsFederateHla13::EfcsFederateHla13( std::wstring FederationName
									, std::wstring FederateName
									, std::wstring FomFileName
									) 
									: NullFederateAmbassador()
									, _RtiAmb() 
{
	#ifdef DEBUG_EFCS_FED
	std::cout << "EfcsFederateHla13.cc -> Constructor(): Start" << std::endl;
	#endif
	_FederationName = FederationName;
	_FederateName = FederateName;
	_FomFileName = FomFileName;
	_TimeStep = 0.0;
	_Lookahead = 0.0;
	_RestOfTimeStep = 0.0;
	_LocalTime = 0.0;
	_SimulationEndTime = 0000.0;
	_IsTimeReg = _IsTimeConst = _IsTimeAdvanceGrant = false ;
	// Note: FD is creator for the SDSE federation
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = false ;
	_IsOutMesTimespamped = true;
	
	_Discov_TRIM_DATA_ENGINES               = false;
	_Discov_TRIM_DATA_FLIGHT_DYNAMICS       = false;
	_Discov_JOYSTICK                        = false;
	_Discov_COCKPIT                         = false;
	_Discov_MEAS_AIRCRAFT_POSITION          = false;
	_Discov_MEAS_AIRCRAFT_ORIENTATION       = false;
	_Discov_MEAS_AIRCRAFT_UVW_SPEED         = false;
	_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ACCELERATION      = false;
	_Discov_MEAS_AIRCRAFT_SPEED             = false;
	_Discov_MEAS_AIRCRAFT_ADDITIONAL        = false;
	_New_TRIM_DATA_ENGINES                  = false;
	_New_TRIM_DATA_FLIGHT_DYNAMICS          = false;
	_New_JOYSTICK                           = false;
	_New_COCKPIT                            = false;
	_New_MEAS_AIRCRAFT_POSITION             = false;
	_New_MEAS_AIRCRAFT_ORIENTATION          = false;
	_New_MEAS_AIRCRAFT_UVW_SPEED            = false;
	_New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED    = false;
	_New_MEAS_AIRCRAFT_ACCELERATION         = false;
	_New_MEAS_AIRCRAFT_SPEED                = false;
	_New_MEAS_AIRCRAFT_ADDITIONAL           = false;

	#ifdef DEBUG_EFCS_FED
	std::cout << "EfcsFederateHla13.cc -> Constructor(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// EfcsFederateHla13 Destructor
EfcsFederateHla13::~EfcsFederateHla13()
	                     throw(RTI::FederateInternalError)
{

}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void EfcsFederateHla13::createFederationExecution() 
{
	#ifdef DEBUG_EFCS_FED
	std::cout << "EfcsFederateHla13.cc -> createFederationExecution(): Start" << std::endl;
	#endif
    try 
    {
        _RtiAmb.createFederationExecution( getString(_FederationName).c_str()
										 , getString(_FomFileName).c_str()
										 );
    } 
    catch ( RTI::FederationExecutionAlreadyExists ) 
    {
		std::cout << "CFE: Federation \"" << getString(_FederationName).c_str() << "\" already created by another federate." << std::endl;
	} 
	catch ( RTI::Exception &e ) 
	{
		std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
	} 
	catch ( ... ) 
	{
		std::cerr << "Error: unknown non-RTI exception." << std::endl;
	}
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> createFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void EfcsFederateHla13::destroyFederationExecution() 
{
	#ifdef DEBUG_EFCS_FED
	std::cout << "EfcsFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
	if (_IsCreator)
	{
		try 
		{
			_RtiAmb.destroyFederationExecution(getString(_FederationName).c_str());
		}
		catch (RTI::Exception& e) 
		{
			std::cout << "DFE: caught " << e._name << " reason " << e._reason <<std::endl;
		}
	}
	#ifdef DEBUG_EFCS_FED
	std::cout << "EfcsFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EfcsFederateHla13::joinFederationExecution() 
{
	#ifdef DEBUG_EFCS_FED
	std::cout << "EfcsFederateHla13.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb.joinFederationExecution( getString(_FederateName).c_str()
                                                         , getString(_FederationName).c_str()
                                                         ,this
                                                         );
    }
    catch (RTI::Exception& e) 
    {
        std::cout << "JFE: caught " << e._name << " reason " << e._reason <<std::endl;
    }
    #ifdef DEBUG_EFCS_FED
	std::cout << "EfcsFederateHla13.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EfcsFederateHla13::resignFederationExecution() 
{
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb.resignFederationExecution(RTI::DELETE_OBJECTS_AND_RELEASE_ATTRIBUTES);
	}
	catch (RTI::Exception& e) 
	{
		std::cout << "RFE: caught " << e._name << " reason " << e._reason <<std::endl;
	}
	#ifdef DEBUG_EFCS_FED
	std::cout << "EfcsFederateHla13.cc -> resignFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
RTI::FederateHandle EfcsFederateHla13::getFederateHandle() const
{
    return _FederateHandle;
}

// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void EfcsFederateHla13::setHlaTimeManagementSettings( RTIfedTime TimeStep
                                                    , RTIfedTime Lookahead
                                                    , RTIfedTime LocalTime
                                                    , RTIfedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void EfcsFederateHla13::getAllHandles()
{
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed
		_TRIM_DATA_ClassHandle = _RtiAmb.getObjectClassHandle("TRIM_DATA");     
		_JOYSTICK_ClassHandle = _RtiAmb.getObjectClassHandle("JOYSTICK");
		_COCKPIT_ClassHandle = _RtiAmb.getObjectClassHandle("COCKPIT");
		_MEAS_AIRCRAFT_POSITION_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_POSITION");
		_MEAS_AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_ORIENTATION");
		_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_UVW_SPEED");
		_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_PQR_ANGULAR_SPEED");
		_MEAS_AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_ACCELERATION");
		_MEAS_AIRCRAFT_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_SPEED");
		_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_ADDITIONAL");
		_RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_THROTTLE_EQ",_TRIM_DATA_ClassHandle);
        _LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ENGINE_THROTTLE_EQ",_TRIM_DATA_ClassHandle);
        _ELEVATOR_DEFLECTION_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("ELEVATOR_DEFLECTION_EQ",_TRIM_DATA_ClassHandle);
        _STABILIZER_DEFLECTION_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("STABILIZER_DEFLECTION_EQ",_TRIM_DATA_ClassHandle);    
        _AILERON_ATTRIBUTE = _RtiAmb.getAttributeHandle("AILERON",_JOYSTICK_ClassHandle);
        _ELEVATOR_ATTRIBUTE = _RtiAmb.getAttributeHandle("ELEVATOR",_JOYSTICK_ClassHandle);
        _RUDDER_ATTRIBUTE = _RtiAmb.getAttributeHandle("RUDDER",_JOYSTICK_ClassHandle);
        _THROTTLE_LEFT_ATTRIBUTE = _RtiAmb.getAttributeHandle("THROTTLE_LEFT",_JOYSTICK_ClassHandle);
        _THROTTLE_RIGHT_ATTRIBUTE = _RtiAmb.getAttributeHandle("THROTTLE_RIGHT",_JOYSTICK_ClassHandle);
        _FLAPS_ATTRIBUTE = _RtiAmb.getAttributeHandle("FLAPS",_JOYSTICK_ClassHandle);
        _SPOILERS_ATTRIBUTE = _RtiAmb.getAttributeHandle("SPOILERS",_JOYSTICK_ClassHandle);
        _GEARS_ATTRIBUTE = _RtiAmb.getAttributeHandle("GEARS",_JOYSTICK_ClassHandle);
        _BRAKES_ATTRIBUTE = _RtiAmb.getAttributeHandle("BRAKES",_JOYSTICK_ClassHandle);
        _AUTOPILOT_AP_ACTIVE_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_AP_ACTIVE", _COCKPIT_ClassHandle);
        _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_ATHR_ACTIVE", _COCKPIT_ClassHandle);
        _AUTOPILOT_SPD_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_SPD", _COCKPIT_ClassHandle);
        _AUTOPILOT_HDG_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_HDG", _COCKPIT_ClassHandle);
        _AUTOPILOT_ALT_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_ALT", _COCKPIT_ClassHandle);
        _AUTOPILOT_VS_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_VS",_COCKPIT_ClassHandle);
        _MEAS_LONGITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_LONGITUDE",_MEAS_AIRCRAFT_POSITION_ClassHandle);
        _MEAS_LATITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_LATITUDE", _MEAS_AIRCRAFT_POSITION_ClassHandle);
        _MEAS_ALTITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_ALTITUDE",_MEAS_AIRCRAFT_POSITION_ClassHandle);     
        _MEAS_PHI_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_PHI", _MEAS_AIRCRAFT_ORIENTATION_ClassHandle);
        _MEAS_THETA_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_THETA",_MEAS_AIRCRAFT_ORIENTATION_ClassHandle);
        _MEAS_PSI_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_PSI", _MEAS_AIRCRAFT_ORIENTATION_ClassHandle);     
        _MEAS_U_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_U_SPEED",_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _MEAS_V_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_V_SPEED",_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _MEAS_W_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_W_SPEED",_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _MEAS_P_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_P_ANG_SPEED",_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _MEAS_Q_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_Q_ANG_SPEED",_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _MEAS_R_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_R_ANG_SPEED",_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);    
        _MEAS_X_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_X_ACC",_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);
        _MEAS_Y_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_Y_ACC",_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);
        _MEAS_Z_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_Z_ACC",_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);      
        _MEAS_INDICATED_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_INDICATED_AIRSPEED",_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_EQUIVALENT_AIRSPEED",_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_CALIBRATED_AIRSPEED",_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _MEAS_TRUE_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_TRUE_AIRSPEED", _MEAS_AIRCRAFT_SPEED_ClassHandle);
        _MEAS_GROUND_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_GROUND_SPEED", _MEAS_AIRCRAFT_SPEED_ClassHandle);
        _MEAS_VERTICAL_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_VERTICAL_SPEED", _MEAS_AIRCRAFT_SPEED_ClassHandle);
        _MEAS_MACH_NUMBER_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_MACH_NUMBER", _MEAS_AIRCRAFT_SPEED_ClassHandle);  
        _MEAS_ALPHA_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_ALPHA", _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
        _MEAS_BETA_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_BETA", _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
        _MEAS_DYNAMIC_PRESSURE_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_DYNAMIC_PRESSURE",_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle); 
        _MEAS_TANK_FILLING_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_TANK_FILLING",    _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
        // Published
		_FLIGHT_CONTROLS_ClassHandle = _RtiAmb.getObjectClassHandle("FLIGHT_CONTROLS"); 
        _RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_AILERON_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_AILERON_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ELEVATOR_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ELEVATOR_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RUDDER_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_COMMANDED_THROTTLE",_FLIGHT_CONTROLS_ClassHandle);
        _LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ENGINE_COMMANDED_THROTTLE",_FLIGHT_CONTROLS_ClassHandle);
        _FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("FLAPS_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("SPOILERS_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("STABILIZER_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _GEARS_COMMANDED_POSITION_ATTRIBUTE = _RtiAmb.getAttributeHandle("GEARS_COMMANDED_POSITION",_FLIGHT_CONTROLS_ClassHandle); 
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void EfcsFederateHla13::publishAndSubscribe()
{
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		attr_TRIM_DATA.reset(RTI::AttributeHandleSetFactory::create(4));
		attr_TRIM_DATA->add(_RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE);
		attr_TRIM_DATA->add(_LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE);
		attr_TRIM_DATA->add(_ELEVATOR_DEFLECTION_EQ_ATTRIBUTE);
		attr_TRIM_DATA->add(_STABILIZER_DEFLECTION_EQ_ATTRIBUTE);
	  
		attr_JOYSTICK.reset(RTI::AttributeHandleSetFactory::create(9));
		attr_JOYSTICK->add(_AILERON_ATTRIBUTE);
		attr_JOYSTICK->add(_ELEVATOR_ATTRIBUTE);
		attr_JOYSTICK->add(_RUDDER_ATTRIBUTE);
		attr_JOYSTICK->add(_THROTTLE_LEFT_ATTRIBUTE);
		attr_JOYSTICK->add(_THROTTLE_RIGHT_ATTRIBUTE);
		attr_JOYSTICK->add(_FLAPS_ATTRIBUTE);
		attr_JOYSTICK->add(_SPOILERS_ATTRIBUTE);
		attr_JOYSTICK->add(_GEARS_ATTRIBUTE);
		attr_JOYSTICK->add(_BRAKES_ATTRIBUTE);
		
		attr_COCKPIT.reset(RTI::AttributeHandleSetFactory::create(6)) ;    
		attr_COCKPIT->add(_AUTOPILOT_AP_ACTIVE_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_SPD_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_HDG_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_ALT_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_VS_ATTRIBUTE);

		attr_MEAS_AIRCRAFT_POSITION.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_ORIENTATION.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_UVW_SPEED.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_ACCELERATION.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_SPEED.reset(RTI::AttributeHandleSetFactory::create(7)) ;
		attr_MEAS_AIRCRAFT_ADDITIONAL.reset(RTI::AttributeHandleSetFactory::create(4)) ;
		attr_MEAS_AIRCRAFT_POSITION->add(_MEAS_LONGITUDE_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_POSITION->add(_MEAS_LATITUDE_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_POSITION->add(_MEAS_ALTITUDE_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ORIENTATION->add(_MEAS_PHI_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ORIENTATION->add(_MEAS_THETA_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ORIENTATION->add(_MEAS_PSI_ATTRIBUTE);   
		attr_MEAS_AIRCRAFT_UVW_SPEED->add(_MEAS_U_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_UVW_SPEED->add(_MEAS_V_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_UVW_SPEED->add(_MEAS_W_SPEED_ATTRIBUTE);  
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED->add(_MEAS_P_ANG_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED->add(_MEAS_Q_ANG_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED->add(_MEAS_R_ANG_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ACCELERATION->add(_MEAS_X_ACC_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ACCELERATION->add(_MEAS_Y_ACC_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ACCELERATION->add(_MEAS_Z_ACC_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_INDICATED_AIRSPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_TRUE_AIRSPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_GROUND_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_VERTICAL_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_MACH_NUMBER_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ADDITIONAL->add(_MEAS_ALPHA_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ADDITIONAL->add(_MEAS_BETA_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ADDITIONAL->add(_MEAS_DYNAMIC_PRESSURE_ATTRIBUTE);  
		attr_MEAS_AIRCRAFT_ADDITIONAL->add(_MEAS_TANK_FILLING_ATTRIBUTE);
		
		_RtiAmb.subscribeObjectClassAttributes( _TRIM_DATA_ClassHandle,*attr_TRIM_DATA); 
        _RtiAmb.subscribeObjectClassAttributes( _JOYSTICK_ClassHandle,*attr_JOYSTICK);
        _RtiAmb.subscribeObjectClassAttributes(_COCKPIT_ClassHandle,*attr_COCKPIT);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_POSITION_ClassHandle, *attr_MEAS_AIRCRAFT_POSITION);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle, *attr_MEAS_AIRCRAFT_ORIENTATION);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle, *attr_MEAS_AIRCRAFT_UVW_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,*attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle, *attr_MEAS_AIRCRAFT_ACCELERATION);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_SPEED_ClassHandle, *attr_MEAS_AIRCRAFT_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle, *attr_MEAS_AIRCRAFT_ADDITIONAL); 

		// For Class/Attributes published
		attr_FLIGHT_CONTROLS.reset(RTI::AttributeHandleSetFactory::create(11)) ;   
		attr_FLIGHT_CONTROLS->add(_RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_GEARS_COMMANDED_POSITION_ATTRIBUTE);
		
        _RtiAmb.publishObjectClass(_FLIGHT_CONTROLS_ClassHandle,*attr_FLIGHT_CONTROLS);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }   
    #ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EfcsFederateHla13::registerObjectInstances()
{
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
        _FLIGHT_CONTROLS_ObjectHandle = _RtiAmb.registerObjectInstance(_FLIGHT_CONTROLS_ClassHandle,"Flight_Controls");
        ahvps_FLIGHT_CONTROLS.reset(RTI::AttributeSetFactory::create(11));
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void EfcsFederateHla13::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb.unsubscribeObjectClass(_TRIM_DATA_ClassHandle);  
        _RtiAmb.unsubscribeObjectClass(_JOYSTICK_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_COCKPIT_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
       
    try 
    {
        _RtiAmb.unpublishObjectClass(_FLIGHT_CONTROLS_ClassHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
    #ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EfcsFederateHla13::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_Discov_TRIM_DATA_ENGINES               ||
			!_Discov_TRIM_DATA_FLIGHT_DYNAMICS       ||
			!_Discov_JOYSTICK                        ||
			!_Discov_COCKPIT                         ||
			!_Discov_MEAS_AIRCRAFT_POSITION          ||
			!_Discov_MEAS_AIRCRAFT_ORIENTATION       ||
			!_Discov_MEAS_AIRCRAFT_UVW_SPEED         ||
			!_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED ||
			!_Discov_MEAS_AIRCRAFT_ACCELERATION      ||
			!_Discov_MEAS_AIRCRAFT_SPEED             ||
			!_Discov_MEAS_AIRCRAFT_ADDITIONAL) 
	{
		try 
		{
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		} 
		catch ( RTI::Exception &e ) 
		{
			cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
		} 
		catch ( ... ) 
		{
			cerr << "Error: unknown non-RTI exception." << endl;
		}
	}
		
	_Discov_TRIM_DATA_ENGINES = false;
	_Discov_TRIM_DATA_FLIGHT_DYNAMICS = false;
	_Discov_JOYSTICK = false;
	_Discov_COCKPIT = false;
	_Discov_MEAS_AIRCRAFT_POSITION = false;
	_Discov_MEAS_AIRCRAFT_ORIENTATION = false;
	_Discov_MEAS_AIRCRAFT_UVW_SPEED = false;
	_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ACCELERATION = false;
	_Discov_MEAS_AIRCRAFT_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ADDITIONAL = false;
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EfcsFederateHla13::waitForAllAttributesReceived()
{
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_TRIM_DATA_ENGINES               ||
           !_New_TRIM_DATA_FLIGHT_DYNAMICS       ||
           !_New_JOYSTICK                        ||
           !_New_COCKPIT                         ||
           !_New_MEAS_AIRCRAFT_POSITION          ||
           !_New_MEAS_AIRCRAFT_ORIENTATION       ||
           !_New_MEAS_AIRCRAFT_UVW_SPEED         ||
           !_New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED ||
           !_New_MEAS_AIRCRAFT_ACCELERATION      ||
           !_New_MEAS_AIRCRAFT_SPEED             ||
           !_New_MEAS_AIRCRAFT_ADDITIONAL) 
	{
		try 
		{
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
			
		} 
		catch ( RTI::Exception &e ) 
		{
			cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
		} 
		catch ( ... ) 
		{
			cerr << "Error: unknown non-RTI exception." << endl;
		}
	} 
    
    _New_TRIM_DATA_ENGINES = false;
    _New_TRIM_DATA_FLIGHT_DYNAMICS = false;
    _New_JOYSTICK = false;
    _New_COCKPIT = false;
    _New_MEAS_AIRCRAFT_POSITION = false;
    _New_MEAS_AIRCRAFT_ORIENTATION = false;
    _New_MEAS_AIRCRAFT_UVW_SPEED = false;
    _New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
    _New_MEAS_AIRCRAFT_ACCELERATION = false;
    _New_MEAS_AIRCRAFT_SPEED = false;
    _New_MEAS_AIRCRAFT_ADDITIONAL = false;
    #ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void EfcsFederateHla13::discoverObjectInstance( RTI::ObjectHandle theObject
                                                   , RTI::ObjectClassHandle theObjectClass
                                                   , const char *theObjectName
                                                   )
                                             throw ( RTI::CouldNotDiscover
                                                   , RTI::ObjectClassNotKnown
                                                   , RTI::FederateInternalError
                                                   )
{
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    if ( (theObjectClass == _TRIM_DATA_ClassHandle) && (!strcmp(theObjectName,"Trim_Data_Engines")) ) 
    {
        _Discov_TRIM_DATA_ENGINES = true;
        _ObjInstance_TRIM_DATA_ENGINES_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        cout << "< Trim_Data_Engines > Object Instance has been discovered with handle "
             << _ObjInstance_TRIM_DATA_ENGINES_ObjectHandle
             << "."
             << endl;
		#endif
    }
    else if ( (theObjectClass == _TRIM_DATA_ClassHandle) && (!strcmp(theObjectName,"Trim_Data_Flight_Dynamics")) ) 
    {
        _Discov_TRIM_DATA_FLIGHT_DYNAMICS = true;
        _ObjInstance_TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        cout << "< Trim_Data_Flight_Dynamics > Object Instance has been discovered with handle "
             << _ObjInstance_TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle
             << "."
             << endl;
        #endif
    }
    else if ( (theObjectClass == _JOYSTICK_ClassHandle) && (!strcmp(theObjectName,"Joystick")) ) 
    {
        _Discov_JOYSTICK = true;
        _ObjInstance_JOYSTICK_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        cout << "< Joystick > Object Instance has been discovered with handle "
             << _ObjInstance_JOYSTICK_ObjectHandle
             << "."
             << endl;
        #endif
    }
    else if ( (theObjectClass == _COCKPIT_ClassHandle) && (!strcmp(theObjectName,"Cockpit")) ) 
    {
        _Discov_COCKPIT = true;
        _ObjInstance_COCKPIT_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        cout << "< Cockpit > Object Instance has been discovered with handle "
             << _ObjInstance_COCKPIT_ObjectHandle
             << "."
             << endl;
        #endif
    }
    else if ( (theObjectClass == _MEAS_AIRCRAFT_POSITION_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Position")) ) 
    {
        _Discov_MEAS_AIRCRAFT_POSITION = true;
        _ObjInstance_MEAS_AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        cout << "< Meas_Aircraft_Position > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_POSITION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ORIENTATION_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Orientation")) ) 
    {
        _Discov_MEAS_AIRCRAFT_ORIENTATION = true;
        _ObjInstance_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        cout << "< Meas_Aircraft_Orientation > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_UVW_SPEED_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_UVW_Speed")) ) 
    {
        _Discov_MEAS_AIRCRAFT_UVW_SPEED = true;
        _ObjInstance_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        cout << "< Meas_Aircraft_UVW_Speed > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_PQR_Angular_Speed")) ) 
    {
        _Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = true;
        _ObjInstance_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        cout << "< Meas_Aircraft_PQR_Angular_Speed > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ACCELERATION_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Acceleration")) ) 
    {
        _Discov_MEAS_AIRCRAFT_ACCELERATION = true;
        _ObjInstance_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        cout << "< Meas_Aircraft_Acceleration > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_SPEED_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Speed")) ) 
    {
        _Discov_MEAS_AIRCRAFT_SPEED = true;
        _ObjInstance_MEAS_AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        cout << "< Meas_Aircraft_Speed > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Additional")) ) 
    {
        _Discov_MEAS_AIRCRAFT_ADDITIONAL = true;
        _ObjInstance_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        cout << "< Meas_Aircraft_Additional > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle
             << "."
             << endl;
        #endif
    }   
    #ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void EfcsFederateHla13::pauseInitTrim()
{
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> pause_init_trim(): Start" << std::endl;
    #endif
    if (_IsCreator) 
    {
		// Trimming synchronization starts automatically (Creator)
        try 
        {
            _RtiAmb.registerFederationSynchronizationPoint("Trimming", "");
        }
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
		while (!_SyncRegSuccess && !_SyncRegFailed) 
		{
			try 
			{
				_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
			} 
			catch ( RTI::Exception &e ) 
			{
				std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
			} 
			catch ( ... ) 
			{
				std::cerr << "Error: unknown non-RTI exception." << std::endl;
			}
			std::cout << ">> Waiting for success or failure of synchronisation point Trimming. " << std::endl;
		} 
		if(_SyncRegFailed)
		{
			std::cout << ">> Error on Trimming Synchronization" << std::endl;
		} 
    }
	while (!_InPause) 
	{
		std::cout << ">> Waiting for synchronisation point Trimming announcement." << std::endl;
		try 
		{
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		} 
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	try 
	{
		_RtiAmb.synchronizationPointAchieved("Trimming");
	} 
	catch ( RTI::Exception &e ) 
	{
		std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
	} 
	catch ( ... ) 
	{
		std::cerr << "Error: unknown non-RTI exception." << std::endl;
	}
	std::cout << ">> Synchronisation point Trimming satisfied." << std::endl;     

	while (_InPause) 
	{
		std::cout << ">> Waiting for initialization phase." << std::endl ;
		try 
		{
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		} 
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Simulation Sync
void EfcsFederateHla13::pauseInitSim()
{
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> pause_init_sim(): Start" << std::endl;
    #endif
    if (_IsCreator) 
    {
		std::cout << ">> PRESS ENTER TO START SIMULATING " << endl;      
        std::cin.get();
		// Simulating synchronization starts with an action of the user (on Creator interface)
        //std::cout << "Pause requested per Creator " << std::endl;
        try 
        {
            _RtiAmb.registerFederationSynchronizationPoint("Simulating", "");
        }
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
		while (!_SyncRegSuccess && !_SyncRegFailed) 
		{
			try 
			{
				_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
			} 
			catch ( RTI::Exception &e ) 
			{
				std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
			} 
			catch ( ... ) 
			{
				std::cerr << "Error: unknown non-RTI exception." << std::endl;
			}
			std::cout << ">> Waiting for success or failure of synchronisation point Simulating. " << std::endl;
		} 
		if(_SyncRegFailed)
		{
			std::cout << ">> Error on initial Synchronization" << std::endl;
		} 
    }
	while (!_InPause) 
	{
		std::cout << ">> Waiting for synchronisation point Simulating announcement." << std::endl;
		try 
		{
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		} 
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	try 
	{
		_RtiAmb.synchronizationPointAchieved("Simulating");
	} 
	catch ( RTI::Exception &e ) 
	{
		std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
	} 
	catch ( ... ) 
	{
		std::cerr << "Error: unknown non-RTI exception." << std::endl;
	}
	cout << ">> Synchronisation point Simulating satisfied." << endl;      

	while (_InPause) 
	{
		std::cout << ">> Waiting for simulation phase." << std::endl ;
		try 
		{
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		} 
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	#ifdef DEBUG_EFCS_FED
    std::cout << "EfcsFederateHla13.cc -> pause_init_sim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void EfcsFederateHla13::initialization() 
{
	waitForAllObjectDiscovered();
	waitForAllAttributesReceived();
	//_EFCS.initialization();
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void EfcsFederateHla13::calculateState() 
{
	// Model State calculation
	_EFCS.calculate_state(); 
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void EfcsFederateHla13::calculateOutput() 
{
	// Model output calculation
	_EFCS.calculate_output();
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void EfcsFederateHla13::sendUpdateAttributes(const RTI::FedTime& UpdateTime)
{   
	//std::cout << "EfcsFederateHla13::sendUpdateAttributes: Start" << std::endl;
	ahvps_FLIGHT_CONTROLS->empty();
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_EFCS.getRightAileron());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_FLIGHT_CONTROLS -> add( _RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE
	                            , static_cast<char*>(_OutputMessagebuffer(0))
	                            , _OutputMessagebuffer.size()
	                            ); 
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_EFCS.getLeftAileron());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_FLIGHT_CONTROLS -> add( _LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE
	                            , static_cast<char*>(_OutputMessagebuffer(0))
	                            , _OutputMessagebuffer.size()
	                            ); 
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_EFCS.getRightElevator());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_FLIGHT_CONTROLS -> add( _RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE
	                            , static_cast<char*>(_OutputMessagebuffer(0))
	                            , _OutputMessagebuffer.size()
	                            ); 
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_EFCS.getLeftElevator());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_FLIGHT_CONTROLS -> add( _LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE
	                            , static_cast<char*>(_OutputMessagebuffer(0))
	                            , _OutputMessagebuffer.size()
	                            ); 
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_EFCS.getRudder());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_FLIGHT_CONTROLS -> add(_RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE
	                            , static_cast<char*>(_OutputMessagebuffer(0))
	                            , _OutputMessagebuffer.size()
	                            ); 
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_EFCS.getRightEngine());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_FLIGHT_CONTROLS -> add( _RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE
	                            , static_cast<char*>(_OutputMessagebuffer(0))
	                            , _OutputMessagebuffer.size()
	                            ); 
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_EFCS.getLeftEngine());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_FLIGHT_CONTROLS -> add( _LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE
	                            , static_cast<char*>(_OutputMessagebuffer(0))
	                            , _OutputMessagebuffer.size()
	                            );
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_EFCS.getFlaps());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_FLIGHT_CONTROLS -> add(_FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE
	                            , static_cast<char*>(_OutputMessagebuffer(0))
	                            , _OutputMessagebuffer.size()
	                            );
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_EFCS.getSpoilers());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_FLIGHT_CONTROLS -> add( _SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE
	                            , static_cast<char*>(_OutputMessagebuffer(0))
	                            , _OutputMessagebuffer.size()
	                            );
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_EFCS.getStabilizer());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_FLIGHT_CONTROLS -> add( _STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE
	                            , static_cast<char*>(_OutputMessagebuffer(0))
	                            , _OutputMessagebuffer.size()
	                            );
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_EFCS.getGears());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_FLIGHT_CONTROLS -> add( _GEARS_COMMANDED_POSITION_ATTRIBUTE
	                            , static_cast<char*>(_OutputMessagebuffer(0))
	                            , _OutputMessagebuffer.size()
	                            ); 
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb.updateAttributeValues(_FLIGHT_CONTROLS_ObjectHandle, *ahvps_FLIGHT_CONTROLS, "");
        }
        else
        {
            _RtiAmb.updateAttributeValues(_FLIGHT_CONTROLS_ObjectHandle, *ahvps_FLIGHT_CONTROLS, UpdateTime, "");
        }
    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
	//std::cout << "EfcsFederateHla13::sendUpdateR: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void EfcsFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
                                                   , const RTI::AttributeHandleValuePairSet& theAttributes
                                                   , const char *theTag
                                                   )
                                             throw ( RTI::ObjectNotKnown
							                       , RTI::AttributeNotKnown
							                       , RTI::FederateOwnsAttributes
							                       , RTI::FederateInternalError
							                       ) 
{
  	RTI::ULong valueLength ;
	RTI::AttributeHandle parmHandle ;
	char *attrValue ;
	MessageBuffer buffer;
	double tmp_value;
    
    if (theObject == _ObjInstance_TRIM_DATA_ENGINES_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();

            if      (parmHandle == _RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE) 
            { 
				_EFCS.setTrimThrottle(buffer.read_double()); 
				#ifdef DEBUG_EFCS_FED
				std::cout << "Trim for Right Engine Ok <<" << std::endl;
				#endif
			}
            else if (parmHandle == _LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE)  
            { 
				_EFCS.setTrimThrottle(buffer.read_double()); 
				#ifdef DEBUG_EFCS_FED
				std::cout << "Trim for Left Engine Ok <<" << std::endl;
				#endif
			}
            else
            { 
				cout << "EfcsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_TRIM_DATA_ENGINES= true;
    }  
    else if (theObject == _ObjInstance_TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();

            if      (parmHandle == _ELEVATOR_DEFLECTION_EQ_ATTRIBUTE) 
            { 
				_EFCS.setTrimElevator(buffer.read_double()); 
				#ifdef DEBUG_EFCS_FED
				std::cout << "Trim for Elevator Ok <<" << std::endl;
				#endif
			}
            else if (parmHandle == _STABILIZER_DEFLECTION_EQ_ATTRIBUTE)  
            { 
				_EFCS.setTrimStabilizer(buffer.read_double()); 
				#ifdef DEBUG_EFCS_FED
				std::cout << "Trim for Stabilizer Ok <<" << std::endl;
				#endif
			}
            else
            { 
				cout << "EfcsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_TRIM_DATA_FLIGHT_DYNAMICS= true;
    } 
    else if (theObject == _ObjInstance_JOYSTICK_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();

            if      (parmHandle == _AILERON_ATTRIBUTE)   
            { 
				double tmp_value = buffer.read_double();
				_EFCS.setAileron_Joystick(tmp_value);
				//_EFCS.setStickRoll(tmp_value);
			}
            else if (parmHandle == _ELEVATOR_ATTRIBUTE)  
            { 
				double tmp_value = buffer.read_double();
				_EFCS.setElevator_Joystick(tmp_value);
				//_EFCS.setStickPitch(tmp_value); 
			}
            else if (parmHandle == _RUDDER_ATTRIBUTE)    
            { 
				_EFCS.setRudder_Joystick(buffer.read_double());
			}
            else if (parmHandle == _THROTTLE_LEFT_ATTRIBUTE)  	
            { 
				_EFCS.setThrottle_Left_Joystick(buffer.read_double()); 
			}
            else if (parmHandle == _THROTTLE_RIGHT_ATTRIBUTE)  	
            { 
				_EFCS.setThrottle_Right_Joystick(buffer.read_double()); 
			}
            else if (parmHandle == _FLAPS_ATTRIBUTE)     
            { 
				_EFCS.setFlaps_Joystick(buffer.read_double());
			}
            else if (parmHandle == _SPOILERS_ATTRIBUTE)  
            { 
				_EFCS.setSpoilers_Joystick(buffer.read_double());
			}
            else if (parmHandle == _GEARS_ATTRIBUTE)     
            { 
				_EFCS.setGears_Joystick(buffer.read_double());
			}
            else if (parmHandle == _BRAKES_ATTRIBUTE)    
            { 
				_EFCS.setBrakes_Joystick(buffer.read_double());
			}
            else
            { 
				cout << "EfcsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_JOYSTICK = true;
    }   
    else if (theObject == _ObjInstance_COCKPIT_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();

            if      (parmHandle == _AUTOPILOT_AP_ACTIVE_ATTRIBUTE)   	
            { 
				_EFCS.setAutopilot_AP_Active(buffer.read_double());
			}
            else if (parmHandle == _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE)   	
            { 
				_EFCS.setAutopilot_athr_Active(buffer.read_double());
			}
            else if (parmHandle == _AUTOPILOT_SPD_ATTRIBUTE)   			
            { 
				_EFCS.setAutopilot_spd(buffer.read_double());
			}
            else if (parmHandle == _AUTOPILOT_HDG_ATTRIBUTE)   			
            { 
				_EFCS.setAutopilot_hdg(buffer.read_double());
			}
            else if (parmHandle == _AUTOPILOT_ALT_ATTRIBUTE)   			
            { 
				_EFCS.setAutopilot_alt(buffer.read_double());
			}
            else if (parmHandle == _AUTOPILOT_VS_ATTRIBUTE)   			
            {
				_EFCS.setAutopilot_vs(buffer.read_double());
			 }
            else
            { 
				cout << "EfcsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_COCKPIT = true;
    }
    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_POSITION_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();

            if      (parmHandle == _MEAS_LONGITUDE_ATTRIBUTE) 
            { 
				_EFCS.setLongitude(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_LATITUDE_ATTRIBUTE)  
            { 
				_EFCS.setLatitude(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_ALTITUDE_ATTRIBUTE)  
            { 
				_EFCS.setAltitude(buffer.read_double()); 
			}
            else
            { 
				cout << "EfcsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_MEAS_AIRCRAFT_POSITION = true;
    } 

    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();

            if      (parmHandle == _MEAS_PHI_ATTRIBUTE)   
            { 
				double tmp_value = buffer.read_double();
				_EFCS.setPhi(tmp_value); 
				//_EFCS.setBankDeg(tmp_value*radtodeg); 
			}
            else if (parmHandle == _MEAS_THETA_ATTRIBUTE) 
            {
				double tmp_value = buffer.read_double(); 
				_EFCS.setTheta(tmp_value); 
				//_EFCS.setPitchDeg(tmp_value*radtodeg); 
			}
            else if (parmHandle == _MEAS_PSI_ATTRIBUTE)   
            { 
				_EFCS.setPsi(buffer.read_double()); 
			}
            else
            { 
				cout << "EfcsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_ORIENTATION = true;
    } 
    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();

            if      (parmHandle == _MEAS_U_SPEED_ATTRIBUTE) 
            { 
				_EFCS.setU(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_V_SPEED_ATTRIBUTE) 
            { 
				_EFCS.setV(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_W_SPEED_ATTRIBUTE) 
            { 
				_EFCS.setW(buffer.read_double()); 
			}
            else
            { 
				cout << "EfcsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_UVW_SPEED = true;
    } 
    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();

            if      (parmHandle == _MEAS_P_ANG_SPEED_ATTRIBUTE) 
            { 
				double tmp_value = buffer.read_double();
				_EFCS.setP(tmp_value); 
				//_EFCS.setRollRateDegs(tmp_value*radtodeg);
			}
            else if (parmHandle == _MEAS_Q_ANG_SPEED_ATTRIBUTE) 
            { 
				double tmp_value = buffer.read_double();
				_EFCS.setQ(tmp_value); 
				//_EFCS.setPitchRateDegs(tmp_value*radtodeg);
			}
            else if (parmHandle == _MEAS_R_ANG_SPEED_ATTRIBUTE) 
            { 
				_EFCS.setR(buffer.read_double()); 
			}
            else
            { 
				cout << "EfcsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = true;
    } 
    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();

            if      (parmHandle == _MEAS_X_ACC_ATTRIBUTE) 
            { 
				_EFCS.setAx(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_Y_ACC_ATTRIBUTE) 
            { 
				_EFCS.setAy(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_Z_ACC_ATTRIBUTE) 
            { 
				_EFCS.setAz(buffer.read_double()); 
			}
            else
            { 
				cout << "EfcsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_ACCELERATION = true;
    } 
    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_SPEED_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();
            
            if      (parmHandle == _MEAS_INDICATED_AIRSPEED_ATTRIBUTE)  
            { 
				_EFCS.setIAS(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE) 
            { 
				_EFCS.setEAS(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE) 
            { 
				_EFCS.setCAS(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_TRUE_AIRSPEED_ATTRIBUTE)       
            { 
				_EFCS.setTAS(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_GROUND_SPEED_ATTRIBUTE)        
            { 
				_EFCS.setGS(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_VERTICAL_SPEED_ATTRIBUTE)      
            { 
				_EFCS.setVS(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_MACH_NUMBER_ATTRIBUTE)        
            { 
				_EFCS.setMach(buffer.read_double()); 
			}
            else
            { 
				cout << "EfcsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_SPEED = true;
    } 
    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();

            if      (parmHandle == _MEAS_ALPHA_ATTRIBUTE)            
            { 
				_EFCS.setAlpha(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_BETA_ATTRIBUTE)             
            { 
				_EFCS.setBeta(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_DYNAMIC_PRESSURE_ATTRIBUTE) 
            { 
				_EFCS.setQbar(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_TANK_FILLING_ATTRIBUTE)     
            { 
				_EFCS.setTankFilling(buffer.read_double()); 
			}
            else
            { 
				cout << "EfcsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_ADDITIONAL = true;
    } 
	else
	{ 
		cout << "EfcsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << endl; 
	} 
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void EfcsFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
                                                   , const RTI::AttributeHandleValuePairSet& theAttributes
                                                   , const RTI::FedTime& theTime
                                                   , const char *theTag
                                                   , RTI::EventRetractionHandle theHandle
                                                   )
											 throw ( RTI::ObjectNotKnown
											       , RTI::AttributeNotKnown
											       , RTI::FederateOwnsAttributes
											       , RTI::InvalidFederationTime 
											       , RTI::FederateInternalError
											       ) 
{
	reflectAttributeValues(theObject, theAttributes, theTag); 
	#ifdef DEBUG_EFCS_FED
	std::cout << "reflectAttributeValues received with ts " <<  ((RTIfedTime&)theTime).getTime() << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void EfcsFederateHla13::timeRegulationEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeRegulationWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeReg = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void EfcsFederateHla13::timeConstrainedEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeConstrainedWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeConst = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeAdvanceGrant
void EfcsFederateHla13::timeAdvanceGrant(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::TimeAdvanceWasNotInProgress,
            RTI::FederateInternalError) 
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_EFCS_FED
	std::cout << " >> TAG RECU == LocalTime = " <<  _LocalTime << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void EfcsFederateHla13::enableTimeRegulation()
{
	_RtiAmb.enableTimeRegulation(_LocalTime, _Lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void EfcsFederateHla13::disableTimeRegulation()
{
	_RtiAmb.disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void EfcsFederateHla13::enableTimeConstrained()
{
	_RtiAmb.enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void EfcsFederateHla13::disableTimeConstrained()
{
	_RtiAmb.disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void EfcsFederateHla13::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb.enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void EfcsFederateHla13::disableAsynchronousDelivery()
{
	_RtiAmb.disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void EfcsFederateHla13::timeAdvanceRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_EFCS_FED
	std::cout << "timeAdvanceRequest requested to " <<  ((RTIfedTime&)NextLogicalTime).getTime() << " begin" << std::endl;
	#endif
	_RtiAmb.timeAdvanceRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_EFCS_FED
	std::cout << "timeAdvanceRequest requested to " <<  ((RTIfedTime&)NextLogicalTime).getTime() << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EfcsFederateHla13::timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime)
{
	_RtiAmb.timeAdvanceRequestAvailable(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
void EfcsFederateHla13::nextEventRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_EFCS_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.nextEventRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_EFCS_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// function : nextEventAvailable
void EfcsFederateHla13::nextEventAvailable(RTIfedTime NextLogicalTime)
{
	_RtiAmb.nextEventRequestAvailable(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationSucceeded
void EfcsFederateHla13::synchronizationPointRegistrationSucceeded(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegSuccess = true ;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void EfcsFederateHla13::synchronizationPointRegistrationFailed(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegFailed = true ;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void EfcsFederateHla13::announceSynchronizationPoint(const char *label, const char *tag)
    throw ( RTI::FederateInternalError) 
{
       _InPause = true ;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void EfcsFederateHla13::federationSynchronized(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _InPause = false ;
} 

// ----------------------------------------------------------------------------
// function : run
void EfcsFederateHla13::run()
{
	while (_LocalTime < _SimulationEndTime)
	{
		// Model calculations
		calculateState(); 
		calculateOutput();
		sendUpdateAttributes(_LocalTime + _Lookahead);
		timeAdvanceRequest(_TimeStep);
	}
} 
