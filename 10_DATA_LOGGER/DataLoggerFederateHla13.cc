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

#include "DataLoggerFederateHla13.hh"

// ----------------------------------------------------------------------------
// DataLoggerFederateHla13 Constructor
DataLoggerFederateHla13::DataLoggerFederateHla13( std::wstring FederationName
									, std::wstring FederateName
									, std::wstring FomFileName
									) 
									: NullFederateAmbassador()
									, _RtiAmb() 
{
	#ifdef DEBUG_DATA_LOGGER_FED
	std::cout << "DataLoggerFederateHla13.cc -> Constructor(): Start" << std::endl;
	#endif
	_FederationName = FederationName;
	_FederateName = FederateName;
	_FomFileName = FomFileName;
	_TimeStep = 0.0;
	_Lookahead = 0.0;
	_RestOfTimeStep = 0.0;
	_LocalTime = 0.0;
	_SimulationEndTime = 0000.0;
	_TotalRealTimeMs = 0.0;
	_IsTimeReg = _IsTimeConst = _IsTimeAdvanceGrant = false ;
	// Note: FD is creator for the SDSE federation
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = false ;
	_IsOutMesTimespamped = true;
	
	XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);
	// Get values from xml file
	vector<string> attribute_path(2);
	attribute_path[0] = "DATA_LOGGER";
	attribute_path[1] = "LogFileName";
	_FileName = getTextValue(params, attribute_path);
	attribute_path[1] = "Granularity";
	_Granularity = getIntValue(params, attribute_path);
	delete params;
	

	_Discov_TRIM_DATA_FLIGHT_DYNAMICS		= false;
	_Discov_TRIM_DATA_ENGINES				= false;
	_Discov_JOYSTICK						= false;
	_Discov_FLIGHT_CONTROLS					= false;
	_Discov_ACTUATORS_CONTROL_SURFACES		= false;
	_Discov_ACTUATORS_ENGINES				= false;
	_Discov_AIRCRAFT_POSITION				= false;
	_Discov_AIRCRAFT_ORIENTATION			= false;
	_Discov_AIRCRAFT_UVW_SPEED				= false;
	_Discov_AIRCRAFT_PQR_ANGULAR_SPEED		= false;
	_Discov_AIRCRAFT_ACCELERATION			= false;
	_Discov_AIRCRAFT_SPEED					= false;
	_Discov_AIRCRAFT_ADDITIONAL				= false;
	_Discov_MEAS_AIRCRAFT_POSITION			= false;
	_Discov_MEAS_AIRCRAFT_ORIENTATION		= false;
	_Discov_MEAS_AIRCRAFT_UVW_SPEED			= false;
	_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED	= false;
	_Discov_MEAS_AIRCRAFT_ACCELERATION		= false;
	_Discov_MEAS_AIRCRAFT_SPEED				= false;
	_Discov_MEAS_AIRCRAFT_ADDITIONAL		= false;
	_Discov_ENVIRONMENT_VARIABLES			= false;
	_Discov_WIND_COMPONENTS					= false;
	_Discov_COCKPIT                         = false;

	_New_TRIM_DATA_ENGINES					= false;
	_New_TRIM_DATA_FLIGHT_DYNAMICS			= false;
	_New_JOYSTICK							= false;
	_New_FLIGHT_CONTROLS					= false;
	_New_ACTUATORS_CONTROL_SURFACES			= false;
	_New_ACTUATORS_ENGINES					= false;
	_New_AIRCRAFT_POSITION					= false;
	_New_AIRCRAFT_ORIENTATION				= false;
	_New_AIRCRAFT_UVW_SPEED					= false;
	_New_AIRCRAFT_PQR_ANGULAR_SPEED			= false;
	_New_AIRCRAFT_ACCELERATION				= false;
	_New_AIRCRAFT_SPEED						= false;
	_New_AIRCRAFT_ADDITIONAL				= false;
	_New_MEAS_AIRCRAFT_POSITION				= false;
	_New_MEAS_AIRCRAFT_ORIENTATION			= false;
	_New_MEAS_AIRCRAFT_UVW_SPEED			= false;
	_New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED	= false;
	_New_MEAS_AIRCRAFT_ACCELERATION			= false;
	_New_MEAS_AIRCRAFT_SPEED				= false;
	_New_MEAS_AIRCRAFT_ADDITIONAL			= false;
	_New_ENVIRONMENT_VARIABLES				= false;
	_New_WIND_COMPONENTS					= false;
	_New_COCKPIT                            = false;
	
	memset(&_TimeStamp, 0, sizeof(timespec));
	memset(&_TimeStamp_old, 0, sizeof(timespec));
	memset(&_ExecutionTime, 0, sizeof(timespec));
	
	#ifdef DEBUG_DATA_LOGGER_FED
	std::cout << "DataLoggerFederateHla13.cc -> Constructor(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// DataLoggerFederateHla13 Destructor
DataLoggerFederateHla13::~DataLoggerFederateHla13()
	                     throw(RTI::FederateInternalError)
{

}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void DataLoggerFederateHla13::createFederationExecution() 
{
	#ifdef DEBUG_DATA_LOGGER_FED
	std::cout << "DataLoggerFederateHla13.cc -> createFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> createFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void DataLoggerFederateHla13::destroyFederationExecution() 
{
	#ifdef DEBUG_DATA_LOGGER_FED
	std::cout << "DataLoggerFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_DATA_LOGGER_FED
	std::cout << "DataLoggerFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla13::joinFederationExecution() 
{
	#ifdef DEBUG_DATA_LOGGER_FED
	std::cout << "DataLoggerFederateHla13.cc -> joinFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_DATA_LOGGER_FED
	std::cout << "DataLoggerFederateHla13.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla13::resignFederationExecution() 
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb.resignFederationExecution(RTI::DELETE_OBJECTS_AND_RELEASE_ATTRIBUTES);
	}
	catch (RTI::Exception& e) 
	{
		std::cout << "RFE: caught " << e._name << " reason " << e._reason <<std::endl;
	}
	#ifdef DEBUG_DATA_LOGGER_FED
	std::cout << "DataLoggerFederateHla13.cc -> resignFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
RTI::FederateHandle DataLoggerFederateHla13::getFederateHandle() const
{
    return _FederateHandle;
}

// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void DataLoggerFederateHla13::setHlaTimeManagementSettings( RTIfedTime TimeStep
                                                    , RTIfedTime Lookahead
                                                    , RTIfedTime LocalTime
                                                    , RTIfedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void DataLoggerFederateHla13::getAllHandles()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed
		_ACTUATORS_ClassHandle = _RtiAmb.getObjectClassHandle("ACTUATORS");
		_JOYSTICK_ClassHandle = _RtiAmb.getObjectClassHandle("JOYSTICK");
		_FLIGHT_CONTROLS_ClassHandle = _RtiAmb.getObjectClassHandle("FLIGHT_CONTROLS"); 
		_TRIM_DATA_ClassHandle = _RtiAmb.getObjectClassHandle("TRIM_DATA");
		_ENVIRONMENT_VARIABLES_ClassHandle = _RtiAmb.getObjectClassHandle("ENVIRONMENT_VARIABLES");
        _WIND_COMPONENTS_ClassHandle = _RtiAmb.getObjectClassHandle("WIND_COMPONENTS");  
        _COCKPIT_ClassHandle = _RtiAmb.getObjectClassHandle("COCKPIT"); 
		_AIRCRAFT_POSITION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_POSITION");
		_AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ORIENTATION");
		_AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_UVW_SPEED");
		_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_PQR_ANGULAR_SPEED");
		_AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ACCELERATION");
		_AIRCRAFT_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_SPEED");
		_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ADDITIONAL");
		
		_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_THRUST_EQ",_TRIM_DATA_ClassHandle);
		_LEFT_ENGINE_THRUST_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ENGINE_THRUST_EQ",_TRIM_DATA_ClassHandle);
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
        _RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_AILERON_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_AILERON_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ELEVATOR_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ELEVATOR_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RUDDER_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("FLAPS_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("SPOILERS_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("STABILIZER_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _GEARS_COMMANDED_POSITION_ATTRIBUTE = _RtiAmb.getAttributeHandle("GEARS_COMMANDED_POSITION",_FLIGHT_CONTROLS_ClassHandle);
		_RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_COMMANDED_THROTTLE",_FLIGHT_CONTROLS_ClassHandle);
		_LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ENGINE_COMMANDED_THROTTLE", _FLIGHT_CONTROLS_ClassHandle);
        _TEMPERATURE_ATTRIBUTE = _RtiAmb.getAttributeHandle("TEMPERATURE",_ENVIRONMENT_VARIABLES_ClassHandle);
		_DENSITY_OF_AIR_ATTRIBUTE = _RtiAmb.getAttributeHandle("DENSITY_OF_AIR",_ENVIRONMENT_VARIABLES_ClassHandle);
		_PRESSURE_ATTRIBUTE = _RtiAmb.getAttributeHandle("PRESSURE",_ENVIRONMENT_VARIABLES_ClassHandle);
		_SPEED_OF_SOUND_ATTRIBUTE = _RtiAmb.getAttributeHandle("SPEED_OF_SOUND",_ENVIRONMENT_VARIABLES_ClassHandle); 
		_U_WIND_ATTRIBUTE = _RtiAmb.getAttributeHandle("U_WIND",_WIND_COMPONENTS_ClassHandle);
		_V_WIND_ATTRIBUTE = _RtiAmb.getAttributeHandle("V_WIND",_WIND_COMPONENTS_ClassHandle);
		_W_WIND_ATTRIBUTE = _RtiAmb.getAttributeHandle("W_WIND",_WIND_COMPONENTS_ClassHandle);
		_P_WIND_ATTRIBUTE = _RtiAmb.getAttributeHandle("P_WIND",_WIND_COMPONENTS_ClassHandle);   
		_Q_WIND_ATTRIBUTE = _RtiAmb.getAttributeHandle("Q_WIND",_WIND_COMPONENTS_ClassHandle);
		_R_WIND_ATTRIBUTE = _RtiAmb.getAttributeHandle("R_WIND",_WIND_COMPONENTS_ClassHandle);  
        _LONGITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("LONGITUDE",_AIRCRAFT_POSITION_ClassHandle);
        _LATITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("LATITUDE", _AIRCRAFT_POSITION_ClassHandle);
        _ALTITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("ALTITUDE",_AIRCRAFT_POSITION_ClassHandle);     
        _PHI_ATTRIBUTE = _RtiAmb.getAttributeHandle("PHI", _AIRCRAFT_ORIENTATION_ClassHandle);
        _THETA_ATTRIBUTE = _RtiAmb.getAttributeHandle("THETA",_AIRCRAFT_ORIENTATION_ClassHandle);
        _PSI_ATTRIBUTE = _RtiAmb.getAttributeHandle("PSI", _AIRCRAFT_ORIENTATION_ClassHandle);     
        _U_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("U_SPEED",_AIRCRAFT_UVW_SPEED_ClassHandle);
        _V_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("V_SPEED",_AIRCRAFT_UVW_SPEED_ClassHandle);
        _W_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("W_SPEED",_AIRCRAFT_UVW_SPEED_ClassHandle);
        _P_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("P_ANG_SPEED",_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _Q_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("Q_ANG_SPEED",_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _R_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("R_ANG_SPEED",_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);    
        _X_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("X_ACC",_AIRCRAFT_ACCELERATION_ClassHandle);
        _Y_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("Y_ACC",_AIRCRAFT_ACCELERATION_ClassHandle);
        _Z_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("Z_ACC",_AIRCRAFT_ACCELERATION_ClassHandle);      
        _INDICATED_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("INDICATED_AIRSPEED",_AIRCRAFT_SPEED_ClassHandle);
        _EQUIVALENT_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("EQUIVALENT_AIRSPEED",_AIRCRAFT_SPEED_ClassHandle);
        _CALIBRATED_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("CALIBRATED_AIRSPEED",_AIRCRAFT_SPEED_ClassHandle);
        _TRUE_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("TRUE_AIRSPEED", _AIRCRAFT_SPEED_ClassHandle);
        _GROUND_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("GROUND_SPEED", _AIRCRAFT_SPEED_ClassHandle);
        _VERTICAL_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("VERTICAL_SPEED", _AIRCRAFT_SPEED_ClassHandle);
        _MACH_NUMBER_ATTRIBUTE = _RtiAmb.getAttributeHandle("MACH_NUMBER", _AIRCRAFT_SPEED_ClassHandle);  
        _ALPHA_ATTRIBUTE = _RtiAmb.getAttributeHandle("ALPHA", _AIRCRAFT_ADDITIONAL_ClassHandle);
        _BETA_ATTRIBUTE = _RtiAmb.getAttributeHandle("BETA", _AIRCRAFT_ADDITIONAL_ClassHandle);
        _DYNAMIC_PRESSURE_ATTRIBUTE = _RtiAmb.getAttributeHandle("DYNAMIC_PRESSURE",_AIRCRAFT_ADDITIONAL_ClassHandle); 
        _TANK_FILLING_ATTRIBUTE = _RtiAmb.getAttributeHandle("TANK_FILLING",    _AIRCRAFT_ADDITIONAL_ClassHandle);
		_MEAS_AIRCRAFT_POSITION_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_POSITION");
		_MEAS_AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_ORIENTATION");
		_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_UVW_SPEED");
		_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_PQR_ANGULAR_SPEED");
		_MEAS_AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_ACCELERATION");
		_MEAS_AIRCRAFT_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_SPEED");
		_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_ADDITIONAL");
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
        _MEAS_TANK_FILLING_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_TANK_FILLING",_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
        _RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_AILERON_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
		_LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_AILERON_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
		_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
		_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ELEVATOR_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
		_RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RUDDER_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);            
		_RIGHT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_THRUST",_ACTUATORS_ClassHandle);
		_LEFT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ENGINE_THRUST",_ACTUATORS_ClassHandle);
		_FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("FLAPS_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
		_SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("SPOILERS_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
		_STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("STABILIZER_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
		_GEARS_POSITION_ATTRIBUTE = _RtiAmb.getAttributeHandle("GEARS_POSITION",_ACTUATORS_ClassHandle);
		_AUTOPILOT_AP_ACTIVE_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_AP_ACTIVE", _COCKPIT_ClassHandle);
        _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_ATHR_ACTIVE", _COCKPIT_ClassHandle);
        _AUTOPILOT_SPD_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_SPD", _COCKPIT_ClassHandle);
        _AUTOPILOT_HDG_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_HDG", _COCKPIT_ClassHandle);
        _AUTOPILOT_ALT_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_ALT", _COCKPIT_ClassHandle);
        _AUTOPILOT_VS_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_VS",_COCKPIT_ClassHandle);
        // Published 
        // None
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void DataLoggerFederateHla13::publishAndSubscribe()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		attr_FLIGHT_CONTROLS.reset(RTI::AttributeHandleSetFactory::create(11));  
		attr_TRIM_DATA.reset(RTI::AttributeHandleSetFactory::create(6));
		attr_JOYSTICK.reset(RTI::AttributeHandleSetFactory::create(9));
		attr_AIRCRAFT_POSITION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_ORIENTATION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_UVW_SPEED.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_PQR_ANGULAR_SPEED.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_ACCELERATION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_SPEED.reset(RTI::AttributeHandleSetFactory::create(7));
		attr_AIRCRAFT_ADDITIONAL.reset(RTI::AttributeHandleSetFactory::create(4));
		attr_MEAS_AIRCRAFT_POSITION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_MEAS_AIRCRAFT_ORIENTATION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_MEAS_AIRCRAFT_UVW_SPEED.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_MEAS_AIRCRAFT_ACCELERATION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_MEAS_AIRCRAFT_SPEED.reset(RTI::AttributeHandleSetFactory::create(7));
		attr_MEAS_AIRCRAFT_ADDITIONAL.reset(RTI::AttributeHandleSetFactory::create(4));
		attr_ACTUATORS.reset(RTI::AttributeHandleSetFactory::create(11));
		attr_COCKPIT.reset(RTI::AttributeHandleSetFactory::create(6)); 	
		attr_ENVIRONMENT_VARIABLES.reset(RTI::AttributeHandleSetFactory::create(4)) ;
	    attr_WIND_COMPONENTS.reset(RTI::AttributeHandleSetFactory::create(6)) ;
	
		attr_TRIM_DATA->add(_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE);
	    attr_TRIM_DATA->add(_LEFT_ENGINE_THRUST_EQ_ATTRIBUTE);	
		attr_TRIM_DATA->add(_RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE);
		attr_TRIM_DATA->add(_LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE);
		attr_TRIM_DATA->add(_ELEVATOR_DEFLECTION_EQ_ATTRIBUTE);
		attr_TRIM_DATA->add(_STABILIZER_DEFLECTION_EQ_ATTRIBUTE);
		attr_JOYSTICK->add(_AILERON_ATTRIBUTE);
		attr_JOYSTICK->add(_ELEVATOR_ATTRIBUTE);
		attr_JOYSTICK->add(_RUDDER_ATTRIBUTE);
		attr_JOYSTICK->add(_THROTTLE_LEFT_ATTRIBUTE);
		attr_JOYSTICK->add(_THROTTLE_RIGHT_ATTRIBUTE);
		attr_JOYSTICK->add(_FLAPS_ATTRIBUTE);
		attr_JOYSTICK->add(_SPOILERS_ATTRIBUTE);
		attr_JOYSTICK->add(_GEARS_ATTRIBUTE);
		attr_JOYSTICK->add(_BRAKES_ATTRIBUTE);
		attr_AIRCRAFT_POSITION->add(_LONGITUDE_ATTRIBUTE);
		attr_AIRCRAFT_POSITION->add(_LATITUDE_ATTRIBUTE);
		attr_AIRCRAFT_POSITION->add(_ALTITUDE_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION->add(_PHI_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION->add(_THETA_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION->add(_PSI_ATTRIBUTE);   
		attr_AIRCRAFT_UVW_SPEED->add(_U_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_UVW_SPEED->add(_V_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_UVW_SPEED->add(_W_SPEED_ATTRIBUTE);  
		attr_AIRCRAFT_PQR_ANGULAR_SPEED->add(_P_ANG_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_PQR_ANGULAR_SPEED->add(_Q_ANG_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_PQR_ANGULAR_SPEED->add(_R_ANG_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_ACCELERATION->add(_X_ACC_ATTRIBUTE);
		attr_AIRCRAFT_ACCELERATION->add(_Y_ACC_ATTRIBUTE);
		attr_AIRCRAFT_ACCELERATION->add(_Z_ACC_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_INDICATED_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_EQUIVALENT_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_CALIBRATED_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_TRUE_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_GROUND_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_VERTICAL_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_MACH_NUMBER_ATTRIBUTE);
		attr_AIRCRAFT_ADDITIONAL->add(_ALPHA_ATTRIBUTE);
		attr_AIRCRAFT_ADDITIONAL->add(_BETA_ATTRIBUTE);
		attr_AIRCRAFT_ADDITIONAL->add(_DYNAMIC_PRESSURE_ATTRIBUTE);  
		attr_AIRCRAFT_ADDITIONAL->add(_TANK_FILLING_ATTRIBUTE);
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
		attr_ACTUATORS->add(_RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_RIGHT_ENGINE_THRUST_ATTRIBUTE);
		attr_ACTUATORS->add(_LEFT_ENGINE_THRUST_ATTRIBUTE);
		attr_ACTUATORS->add(_FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_GEARS_POSITION_ATTRIBUTE);	
		attr_COCKPIT->add(_AUTOPILOT_AP_ACTIVE_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_SPD_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_HDG_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_ALT_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_VS_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES->add(_TEMPERATURE_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES->add(_DENSITY_OF_AIR_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES->add(_PRESSURE_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES->add(_SPEED_OF_SOUND_ATTRIBUTE);
		attr_WIND_COMPONENTS->add(_U_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS->add(_V_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS->add(_W_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS->add(_P_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS->add(_Q_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS->add(_R_WIND_ATTRIBUTE); 
		
		_RtiAmb.subscribeObjectClassAttributes(_COCKPIT_ClassHandle,*attr_COCKPIT);
		_RtiAmb.subscribeObjectClassAttributes(_TRIM_DATA_ClassHandle,*attr_TRIM_DATA); 
        _RtiAmb.subscribeObjectClassAttributes(_JOYSTICK_ClassHandle,*attr_JOYSTICK);		
		_RtiAmb.subscribeObjectClassAttributes(_FLIGHT_CONTROLS_ClassHandle,*attr_FLIGHT_CONTROLS);
		_RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_POSITION_ClassHandle, *attr_AIRCRAFT_POSITION);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_ORIENTATION_ClassHandle, *attr_AIRCRAFT_ORIENTATION);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_UVW_SPEED_ClassHandle, *attr_AIRCRAFT_UVW_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,*attr_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_ACCELERATION_ClassHandle, *attr_AIRCRAFT_ACCELERATION);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_SPEED_ClassHandle, *attr_AIRCRAFT_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_ADDITIONAL_ClassHandle, *attr_AIRCRAFT_ADDITIONAL); 
		_RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_POSITION_ClassHandle, *attr_MEAS_AIRCRAFT_POSITION);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle, *attr_MEAS_AIRCRAFT_ORIENTATION);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle, *attr_MEAS_AIRCRAFT_UVW_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,*attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle, *attr_MEAS_AIRCRAFT_ACCELERATION);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_SPEED_ClassHandle, *attr_MEAS_AIRCRAFT_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle, *attr_MEAS_AIRCRAFT_ADDITIONAL);
        _RtiAmb.subscribeObjectClassAttributes(_ACTUATORS_ClassHandle,*attr_ACTUATORS);
        _RtiAmb.subscribeObjectClassAttributes(_ENVIRONMENT_VARIABLES_ClassHandle,*attr_ENVIRONMENT_VARIABLES);
        _RtiAmb.subscribeObjectClassAttributes(_WIND_COMPONENTS_ClassHandle,*attr_WIND_COMPONENTS);

		// For Class/Attributes published
		// None
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }   
    #ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void DataLoggerFederateHla13::registerObjectInstances()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		// Not used
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void DataLoggerFederateHla13::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb.unsubscribeObjectClass(_COCKPIT_ClassHandle);
		_RtiAmb.unsubscribeObjectClass(_TRIM_DATA_ClassHandle);  
        _RtiAmb.unsubscribeObjectClass(_JOYSTICK_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_FLIGHT_CONTROLS_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_ACTUATORS_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_ADDITIONAL_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_ENVIRONMENT_VARIABLES_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_WIND_COMPONENTS_ClassHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
    #ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void DataLoggerFederateHla13::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while (!_Discov_JOYSTICK       			||
		   !_Discov_FLIGHT_CONTROLS       		||
		   !_Discov_ACTUATORS_CONTROL_SURFACES     	||
		   !_Discov_ACTUATORS_ENGINES     		||
		   !_Discov_AIRCRAFT_POSITION       		||
		   !_Discov_AIRCRAFT_ORIENTATION       		||
		   !_Discov_AIRCRAFT_UVW_SPEED         		||
		   !_Discov_AIRCRAFT_PQR_ANGULAR_SPEED 		||
		   !_Discov_AIRCRAFT_ACCELERATION      		||
		   !_Discov_AIRCRAFT_SPEED             		||
		   !_Discov_AIRCRAFT_ADDITIONAL       		||
		   !_Discov_MEAS_AIRCRAFT_POSITION       		||
		   !_Discov_MEAS_AIRCRAFT_ORIENTATION       	||
		   !_Discov_MEAS_AIRCRAFT_UVW_SPEED         	||
		   !_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED 	||
		   !_Discov_MEAS_AIRCRAFT_ACCELERATION      	||
		   !_Discov_MEAS_AIRCRAFT_SPEED             	||
		   !_Discov_MEAS_AIRCRAFT_ADDITIONAL      	||
		   !_Discov_ENVIRONMENT_VARIABLES             	||
		   !_Discov_WIND_COMPONENTS) 
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
	_Discov_JOYSTICK = false;
	_Discov_ACTUATORS_CONTROL_SURFACES = false;
    _Discov_ACTUATORS_ENGINES = false;
    _Discov_FLIGHT_CONTROLS = false;
    _Discov_ENVIRONMENT_VARIABLES = false;
    _Discov_WIND_COMPONENTS = false;
    _Discov_MEAS_AIRCRAFT_POSITION = false;
	_Discov_MEAS_AIRCRAFT_ORIENTATION = false;
	_Discov_MEAS_AIRCRAFT_UVW_SPEED = false;
	_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ACCELERATION = false;
	_Discov_MEAS_AIRCRAFT_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ADDITIONAL = false;		
	_Discov_AIRCRAFT_POSITION = false;
	_Discov_AIRCRAFT_ORIENTATION = false;
	_Discov_AIRCRAFT_UVW_SPEED = false;
	_Discov_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_AIRCRAFT_ACCELERATION = false;
	_Discov_AIRCRAFT_SPEED = false;
	_Discov_AIRCRAFT_ADDITIONAL = false;
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void DataLoggerFederateHla13::waitForAllAttributesReceived()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_MEAS_AIRCRAFT_POSITION          ||
           !_New_MEAS_AIRCRAFT_ORIENTATION       ||
           !_New_MEAS_AIRCRAFT_UVW_SPEED         ||
           !_New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED ||
           !_New_MEAS_AIRCRAFT_ACCELERATION      ||
           !_New_MEAS_AIRCRAFT_SPEED             ||
           !_New_MEAS_AIRCRAFT_ADDITIONAL        ||
           !_New_AIRCRAFT_POSITION          ||
           !_New_AIRCRAFT_ORIENTATION       ||
           !_New_AIRCRAFT_UVW_SPEED         ||
           !_New_AIRCRAFT_PQR_ANGULAR_SPEED ||
           !_New_AIRCRAFT_ACCELERATION      ||
           !_New_AIRCRAFT_SPEED             ||
           !_New_AIRCRAFT_ADDITIONAL) 
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
	_New_MEAS_AIRCRAFT_POSITION = false;
    _New_MEAS_AIRCRAFT_ORIENTATION = false;
    _New_MEAS_AIRCRAFT_UVW_SPEED = false;
    _New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
    _New_MEAS_AIRCRAFT_ACCELERATION = false;
    _New_MEAS_AIRCRAFT_SPEED = false;
    _New_MEAS_AIRCRAFT_ADDITIONAL = false;
    _New_AIRCRAFT_POSITION = false;
    _New_AIRCRAFT_ORIENTATION = false;
    _New_AIRCRAFT_UVW_SPEED = false;
    _New_AIRCRAFT_PQR_ANGULAR_SPEED = false;
    _New_AIRCRAFT_ACCELERATION = false;
    _New_AIRCRAFT_SPEED = false;
    _New_AIRCRAFT_ADDITIONAL = false;
    #ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void DataLoggerFederateHla13::discoverObjectInstance( RTI::ObjectHandle theObject
                                                 , RTI::ObjectClassHandle theObjectClass
                                                 , const char *theObjectName
                                                 )
                                           throw ( RTI::CouldNotDiscover
                                                 , RTI::ObjectClassNotKnown
                                                 , RTI::FederateInternalError
                                                 )
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    if ( (theObjectClass == _TRIM_DATA_ClassHandle) && (!strcmp(theObjectName,"Trim_Data_Engines"))) 
    {
        _Discov_TRIM_DATA_ENGINES = true;
        _TRIM_DATA_ENGINES_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Trim_Data_Engines > Object Instance has been discovered with handle "
             << _TRIM_DATA_ENGINES_ObjectHandle
             << "."
             << endl;
        #endif

    }
    else if ( (theObjectClass == _TRIM_DATA_ClassHandle) && (!strcmp(theObjectName,"Trim_Data_Flight_Dynamics"))) 
    {
        _Discov_TRIM_DATA_FLIGHT_DYNAMICS = true;
        _TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Trim_Data_Flight_Dynamics > Object Instance has been discovered with handle "
             << _TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle
             << "."
             << endl;
        #endif
    }
    else if ( (theObjectClass == _JOYSTICK_ClassHandle) && (!strcmp(theObjectName,"Joystick"))) 
    {
        _Discov_JOYSTICK = true;
        _JOYSTICK_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Joystick > Object Instance has been discovered with handle "
             << _JOYSTICK_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _COCKPIT_ClassHandle) && (!strcmp(theObjectName,"Cockpit")) ) 
    {
        _Discov_COCKPIT = true;
        _COCKPIT_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Cockpit > Object Instance has been discovered with handle "
             << _COCKPIT_ObjectHandle
             << "."
             << endl;
        #endif
    }
    else if ( (theObjectClass == _FLIGHT_CONTROLS_ClassHandle) && (!strcmp(theObjectName,"Flight_Controls"))) 
    {
        _Discov_FLIGHT_CONTROLS = true;
        _FLIGHT_CONTROLS_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Flight_Controls > Object Instance has been discovered with handle "
             << _FLIGHT_CONTROLS_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    if ( (theObjectClass == _ACTUATORS_ClassHandle) && (!strcmp(theObjectName,"Actuators_Control_Surfaces"))) 
    {
        _Discov_ACTUATORS_CONTROL_SURFACES = true;
        _ACTUATORS_CONTROL_SURFACES_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Actuators_Control_Surfaces > Object Instance has been discovered with handle "
             << _ACTUATORS_CONTROL_SURFACES_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _ACTUATORS_ClassHandle) && (!strcmp(theObjectName,"Actuators_Engines"))) 
    {
        _Discov_ACTUATORS_ENGINES = true;
        _ACTUATORS_ENGINES_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Actuators_Engines > Object Instance has been discovered with handle "
             << _ACTUATORS_ENGINES_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_POSITION_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Position"))) 
    {
        _Discov_AIRCRAFT_POSITION = true;
        _AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Aircraft_Position > Object Instance has been discovered with handle "
             << _AIRCRAFT_POSITION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_ORIENTATION_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Orientation"))) 
    {
        _Discov_AIRCRAFT_ORIENTATION = true;
        _AIRCRAFT_ORIENTATION_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Aircraft_Orientation > Object Instance has been discovered with handle "
             << _AIRCRAFT_ORIENTATION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_UVW_SPEED_ClassHandle) && (!strcmp(theObjectName,"Aircraft_UVW_Speed"))) 
    {
        _Discov_AIRCRAFT_UVW_SPEED = true;
        _AIRCRAFT_UVW_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Aircraft_UVW_Speed > Object Instance has been discovered with handle "
             << _AIRCRAFT_UVW_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle) && (!strcmp(theObjectName,"Aircraft_PQR_Angular_Speed")))
    {
        _Discov_AIRCRAFT_PQR_ANGULAR_SPEED = true;
        _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Aircraft_PQR_Angular_Speed > Object Instance has been discovered with handle "
             << _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_ACCELERATION_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Acceleration"))) 
    {
        _Discov_AIRCRAFT_ACCELERATION = true;
        _AIRCRAFT_ACCELERATION_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Aircraft_Acceleration > Object Instance has been discovered with handle "
             << _AIRCRAFT_ACCELERATION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_SPEED_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Speed"))) 
    {
        _Discov_AIRCRAFT_SPEED = true;
        _AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Aircraft_Speed > Object Instance has been discovered with handle "
             << _AIRCRAFT_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_ADDITIONAL_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Additional"))) 
    {
        _Discov_AIRCRAFT_ADDITIONAL = true;
        _AIRCRAFT_ADDITIONAL_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Aircraft_Additional > Object Instance has been discovered with handle "
             << _AIRCRAFT_ADDITIONAL_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_POSITION_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Position"))) 
    {
        _Discov_MEAS_AIRCRAFT_POSITION = true;
        _MEAS_AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Meas_Aircraft_Position > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_POSITION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ORIENTATION_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Orientation"))) 
    {
        _Discov_MEAS_AIRCRAFT_ORIENTATION = true;
        _MEAS_AIRCRAFT_ORIENTATION_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Meas_Aircraft_Orientation > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_ORIENTATION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_UVW_SPEED_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_UVW_Speed"))) 
    {
        _Discov_MEAS_AIRCRAFT_UVW_SPEED = true;
        _MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Meas_Aircraft_UVW_Speed > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_PQR_Angular_Speed"))) 
    {
        _Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = true;
        _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Meas_Aircraft_PQR_Angular_Speed > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ACCELERATION_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Acceleration"))) 
    {
        _Discov_MEAS_AIRCRAFT_ACCELERATION = true;
        _MEAS_AIRCRAFT_ACCELERATION_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Meas_Aircraft_Acceleration > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_ACCELERATION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_SPEED_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Speed"))) 
    {
        _Discov_MEAS_AIRCRAFT_SPEED = true;
        _MEAS_AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Meas_Aircraft_Speed > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Additional"))) 
    {
        _Discov_MEAS_AIRCRAFT_ADDITIONAL = true;
        _MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Meas_Aircraft_Additional > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _ENVIRONMENT_VARIABLES_ClassHandle) && (!strcmp(theObjectName,"Environment_Variables"))) 
    {
        _Discov_ENVIRONMENT_VARIABLES = true;
        _ENVIRONMENT_VARIABLES_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Environment_Variables > Object Instance has been discovered with handle "
             << _ENVIRONMENT_VARIABLES_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _WIND_COMPONENTS_ClassHandle) && (!strcmp(theObjectName,"Wind_Components"))) 
    {
        _Discov_WIND_COMPONENTS = true;
        _WIND_COMPONENTS_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        cout << "< Wind_Components > Object Instance has been discovered with handle "
             << _WIND_COMPONENTS_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    #ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void DataLoggerFederateHla13::pauseInitTrim()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void DataLoggerFederateHla13::pauseInitSim()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> pause_init_sim(): Start" << std::endl;
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
	#ifdef DEBUG_DATA_LOGGER_FED
    std::cout << "DataLoggerFederateHla13.cc -> pause_init_sim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void DataLoggerFederateHla13::initialization() 
{
	waitForAllObjectDiscovered();
	//waitForAllAttributesReceived();
	string path = "../logs/" + _FileName;	
	_SdseDataLog.open (path);
	_SdseDataLog.clear();
}

// ----------------------------------------------------------------------------
// Calculate State values
void DataLoggerFederateHla13::calculateState() 
{
	// Models State calculation
	_SdseDataLog 	<< _LocalTime // 1
	<< "," << _DataLogger.GET_JOYSTICK_AILERON() // 2
	<< "," << _DataLogger.GET_JOYSTICK_ELEVATOR() // 3
	<< "," << _DataLogger.GET_JOYSTICK_RUDDER() // 4
	<< "," << _DataLogger.GET_JOYSTICK_THROTTLE_LEFT() // 5
	<< "," << _DataLogger.GET_JOYSTICK_THROTTLE_RIGHT() // 6
    << "," << _DataLogger.GET_JOYSTICK_FLAPS() // 7
    << "," << _DataLogger.GET_JOYSTICK_SPOILERS() // 8
    << "," << _DataLogger.GET_JOYSTICK_GEARS() // 9
    << "," << _DataLogger.GET_JOYSTICK_BRAKES() // 10
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_RIGHT_AILERON_COMMANDED_DEFLECTION() // 11
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_LEFT_AILERON_COMMANDED_DEFLECTION() // 12
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_RIGHT_ELEVATOR_COMMANDED_DEFLECTION() // 13
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_LEFT_ELEVATOR_COMMANDED_DEFLECTION() // 14
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_RUDDER_COMMANDED_DEFLECTION() // 15
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_RIGHT_ENGINE_COMMANDED_THROTTLE() // 16
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_LEFT_ENGINE_COMMANDED_THROTTLE() // 17
    << "," << _DataLogger.GET_FLIGHT_CONTROLS_FLAPS_COMMANDED_DEFLECTION() // 18
    << "," << _DataLogger.GET_FLIGHT_CONTROLS_SPOILERS_COMMANDED_DEFLECTION() // 19
    << "," << _DataLogger.GET_FLIGHT_CONTROLS_STABILIZER_COMMANDED_DEFLECTION() // 20
    << "," << _DataLogger.GET_FLIGHT_CONTROLS_GEARS_COMMANDED_POSITION() // 21
	<< "," << _DataLogger.GET_ACTUATORS_RIGHT_AILERON_EFFECTIVE_DEFLECTION() // 22
	<< "," << _DataLogger.GET_ACTUATORS_LEFT_AILERON_EFFECTIVE_DEFLECTION() // 23
	<< "," << _DataLogger.GET_ACTUATORS_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION() // 24
	<< "," << _DataLogger.GET_ACTUATORS_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION() // 25
	<< "," << _DataLogger.GET_ACTUATORS_RUDDER_EFFECTIVE_DEFLECTION() // 26
	<< "," << _DataLogger.GET_ACTUATORS_RIGHT_ENGINE_THRUST() // 27
	<< "," << _DataLogger.GET_ACTUATORS_LEFT_ENGINE_THRUST() // 28
    << "," << _DataLogger.GET_ACTUATORS_FLAPS_EFFECTIVE_DEFLECTION() // 29
    << "," << _DataLogger.GET_ACTUATORS_SPOILERS_EFFECTIVE_DEFLECTION() // 30
    << "," << _DataLogger.GET_ACTUATORS_STABILIZER_EFFECTIVE_DEFLECTION() // 31
    << "," << _DataLogger.GET_ACTUATORS_GEARS_POSITION() // 32
	<< "," << _DataLogger.GET_AIRCRAFT_POSITION_LONGITUDE() // 33
	<< "," << _DataLogger.GET_AIRCRAFT_POSITION_LATITUDE() // 34
	<< "," << _DataLogger.GET_AIRCRAFT_POSITION_ALTITUDE() // 35
	<< "," << _DataLogger.GET_AIRCRAFT_ORIENTATION_PHI() // 36
	<< "," << _DataLogger.GET_AIRCRAFT_ORIENTATION_THETA() // 37
	<< "," << _DataLogger.GET_AIRCRAFT_ORIENTATION_PSI() // 38
	<< "," << _DataLogger.GET_AIRCRAFT_UVW_SPEED_U_SPEED() // 39
	<< "," << _DataLogger.GET_AIRCRAFT_UVW_SPEED_V_SPEED() // 40
	<< "," << _DataLogger.GET_AIRCRAFT_UVW_SPEED_W_SPEED() // 41
	<< "," << _DataLogger.GET_AIRCRAFT_PQR_ANGULAR_SPEED_P_ANG_SPEED() // 42
	<< "," << _DataLogger.GET_AIRCRAFT_PQR_ANGULAR_SPEED_Q_ANG_SPEED() // 43
	<< "," << _DataLogger.GET_AIRCRAFT_PQR_ANGULAR_SPEED_R_ANG_SPEED() // 44
	<< "," << _DataLogger.GET_AIRCRAFT_ACCELERATION_X_ACC() // 45
	<< "," << _DataLogger.GET_AIRCRAFT_ACCELERATION_Y_ACC() // 46
	<< "," << _DataLogger.GET_AIRCRAFT_ACCELERATION_Z_ACC() // 47
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_INDICATED_AIRSPEED() // 48
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_EQUIVALENT_AIRSPEED() // 49
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_CALIBRATED_AIRSPEED() // 50
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_TRUE_AIRSPEED() // 51
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_GROUND_SPEED() // 52
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_VERTICAL_SPEED() // 53
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_MACH_NUMBER() // 54
	<< "," << _DataLogger.GET_AIRCRAFT_ADDITIONAL_ALPHA() // 55
	<< "," << _DataLogger.GET_AIRCRAFT_ADDITIONAL_BETA() // 56
	<< "," << _DataLogger.GET_AIRCRAFT_ADDITIONAL_DYNAMIC_PRESSURE() // 57
    << "," << _DataLogger.GET_AIRCRAFT_ADDITIONAL_TANK_FILLING() // 58
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_POSITION_MEAS_LONGITUDE() // 59
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_POSITION_MEAS_LATITUDE() // 60
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_POSITION_MEAS_ALTITUDE() // 61
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ORIENTATION_MEAS_PHI() // 62
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ORIENTATION_MEAS_THETA() // 63
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ORIENTATION_MEAS_PSI() // 64
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_UVW_SPEED_MEAS_U_SPEED() // 65
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_UVW_SPEED_MEAS_V_SPEED() // 66
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_UVW_SPEED_MEAS_W_SPEED() // 67
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_MEAS_P_ANG_SPEED() // 68
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_MEAS_Q_ANG_SPEED() // 69
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_MEAS_R_ANG_SPEED() // 70
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ACCELERATION_MEAS_X_ACC() // 71
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ACCELERATION_MEAS_Y_ACC() // 72
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ACCELERATION_MEAS_Z_ACC() // 73
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_INDICATED_AIRSPEED() // 74
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_EQUIVALENT_AIRSPEED() // 75
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_CALIBRATED_AIRSPEED() // 76
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_TRUE_AIRSPEED() // 77
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_GROUND_SPEED() // 78
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_VERTICAL_SPEED() // 79
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_MACH_NUMBER() // 80
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_ALPHA() // 81
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_BETA() // 82
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_DYNAMIC_PRESSURE() // 83
    << "," << _DataLogger.GET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_TANK_FILLING() // 84
	<< "," << _DataLogger.GET_ENVIRONMENT_VARIABLES_TEMPERATURE() // 85
	<< "," << _DataLogger.GET_ENVIRONMENT_VARIABLES_DENSITY_OF_AIR() // 86
	<< "," << _DataLogger.GET_ENVIRONMENT_VARIABLES_PRESSURE() // 87
	<< "," << _DataLogger.GET_ENVIRONMENT_VARIABLES_SPEED_OF_SOUND() // 88
	<< "," << _DataLogger.GET_WIND_COMPONENTS_U_WIND() // 89
	<< "," << _DataLogger.GET_WIND_COMPONENTS_V_WIND()	// 90
	<< "," << _DataLogger.GET_WIND_COMPONENTS_W_WIND() // 91
	<< "," << _DataLogger.GET_WIND_COMPONENTS_P_WIND() // 92
	<< "," << _DataLogger.GET_WIND_COMPONENTS_Q_WIND() // 93
	<< "," << _DataLogger.GET_WIND_COMPONENTS_R_WIND()//94
	<< "," << _TotalRealTimeMs//95
	<< "," << _TimeStep//96
	<< "," << ((_ExecutionTime.tv_nsec) / 1000000)//97
	<< '\n';
}

// ----------------------------------------------------------------------------
// Calculate Ouput values
void DataLoggerFederateHla13::calculateOutput() 
{
	// Models output calculation
	// Not used
}

// ----------------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void DataLoggerFederateHla13::sendInitialAllAttributes()
{
	// Not used
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void DataLoggerFederateHla13::sendUpdateAttributes(const RTI::FedTime& UpdateTime)
{   
	// Not used
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void DataLoggerFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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

      if (theObject == _TRIM_DATA_ENGINES_ObjectHandle)
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
            
            if (parmHandle == _RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE) 
            { 
				_DataLogger.SET_TRIM_DATA_RIGHT_ENGINE_THROTTLE_EQ(buffer.read_double());
			}
            else if (parmHandle == _LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE)  
            {
				_DataLogger.SET_TRIM_DATA_LEFT_ENGINE_THROTTLE_EQ(buffer.read_double());
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}

        }
        _New_TRIM_DATA_ENGINES= true;
    } 
    else if (theObject == _TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle){
        for (unsigned int j=0 ; j<theAttributes.size(); j++) {

            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();
            
            if      (parmHandle == _RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE)
            { 
				_DataLogger.SET_TRIM_DATA_RIGHT_ENGINE_THRUST_EQ(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_ENGINE_THRUST_EQ_ATTRIBUTE)
            { 
				_DataLogger.SET_TRIM_DATA_LEFT_ENGINE_THRUST_EQ(buffer.read_double()); 
			}
            else if (parmHandle == _ELEVATOR_DEFLECTION_EQ_ATTRIBUTE)
            { 
				_DataLogger.SET_TRIM_DATA_ELEVATOR_DEFLECTION_EQ(buffer.read_double()); 
			}
            else if (parmHandle == _STABILIZER_DEFLECTION_EQ_ATTRIBUTE) 
            { 
				_DataLogger.SET_TRIM_DATA_STABILIZER_DEFLECTION_EQ(buffer.read_double());
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_TRIM_DATA_FLIGHT_DYNAMICS= true;
    } 

    else if (theObject == _JOYSTICK_ObjectHandle)
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
				_DataLogger.SET_JOYSTICK_AILERON(buffer.read_double()); 
			}
            else if (parmHandle == _ELEVATOR_ATTRIBUTE) 
            { 
				_DataLogger.SET_JOYSTICK_ELEVATOR(buffer.read_double());
			}
            else if (parmHandle == _RUDDER_ATTRIBUTE)  		
            { 
				_DataLogger.SET_JOYSTICK_RUDDER(buffer.read_double()); 
			}
            else if (parmHandle == _THROTTLE_LEFT_ATTRIBUTE)  	
            { 
				_DataLogger.SET_JOYSTICK_THROTTLE_LEFT(buffer.read_double()); 
			}
            else if (parmHandle == _THROTTLE_RIGHT_ATTRIBUTE) 
            { 
				_DataLogger.SET_JOYSTICK_THROTTLE_RIGHT(buffer.read_double());  
			}
            else if (parmHandle == _FLAPS_ATTRIBUTE)  	
            { 
				_DataLogger.SET_JOYSTICK_FLAPS(buffer.read_double());
			}
            else if (parmHandle == _SPOILERS_ATTRIBUTE)  	
            { 
				_DataLogger.SET_JOYSTICK_SPOILERS(buffer.read_double()); 
			}
            else if (parmHandle == _GEARS_ATTRIBUTE)  	
            { 
				_DataLogger.SET_JOYSTICK_GEARS(buffer.read_double());
			}
            else if (parmHandle == _BRAKES_ATTRIBUTE)
            { 
				_DataLogger.SET_JOYSTICK_BRAKES(buffer.read_double());
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_JOYSTICK = true;
    }
	else if (theObject == _COCKPIT_ObjectHandle)
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
				//buffer.read_double();
			}
            else if (parmHandle == _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE)   	
            { 
				//buffer.read_double();
			}
            else if (parmHandle == _AUTOPILOT_SPD_ATTRIBUTE)   			
            { 
				//buffer.read_double();
			}
            else if (parmHandle == _AUTOPILOT_HDG_ATTRIBUTE)   			
            { 
				//buffer.read_double();
			}
            else if (parmHandle == _AUTOPILOT_ALT_ATTRIBUTE)   			
            { 
				//buffer.read_double();
			}
            else if (parmHandle == _AUTOPILOT_VS_ATTRIBUTE)   			
            {
				//buffer.read_double();
			 }
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_COCKPIT = true;
    } 
    else if (theObject == _FLIGHT_CONTROLS_ObjectHandle)
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
            if      (parmHandle == _RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE) 		
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_RIGHT_AILERON_COMMANDED_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE) 
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_LEFT_AILERON_COMMANDED_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_RIGHT_ELEVATOR_COMMANDED_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_LEFT_ELEVATOR_COMMANDED_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_RUDDER_COMMANDED_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_RIGHT_ENGINE_COMMANDED_THROTTLE(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_LEFT_ENGINE_COMMANDED_THROTTLE(buffer.read_double()); 
			}
            else if (parmHandle == _FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_FLAPS_COMMANDED_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_SPOILERS_COMMANDED_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE) 
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_STABILIZER_COMMANDED_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _GEARS_COMMANDED_POSITION_ATTRIBUTE) 
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_GEARS_COMMANDED_POSITION(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}

        }    
        _New_FLIGHT_CONTROLS = true;
    }
    else     if (theObject == _ACTUATORS_CONTROL_SURFACES_ObjectHandle)
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
            if      (parmHandle == _RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE) 
            { 
				_DataLogger.SET_ACTUATORS_RIGHT_AILERON_EFFECTIVE_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ACTUATORS_LEFT_AILERON_EFFECTIVE_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ACTUATORS_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE) 
            { 
				_DataLogger.SET_ACTUATORS_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE)  			
            { 
				_DataLogger.SET_ACTUATORS_RUDDER_EFFECTIVE_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE) 
            { 
				_DataLogger.SET_ACTUATORS_FLAPS_EFFECTIVE_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE)  	
            { 
				_DataLogger.SET_ACTUATORS_SPOILERS_EFFECTIVE_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ACTUATORS_STABILIZER_EFFECTIVE_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _GEARS_POSITION_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ACTUATORS_GEARS_POSITION(buffer.read_double());
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_ACTUATORS_CONTROL_SURFACES = true;

    } 
    else if (theObject == _ACTUATORS_ENGINES_ObjectHandle)
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
            if      (parmHandle == _RIGHT_ENGINE_THRUST_ATTRIBUTE) 		
            { 
				_DataLogger.SET_ACTUATORS_RIGHT_ENGINE_THRUST(buffer.read_double());
			}
            else if (parmHandle == _LEFT_ENGINE_THRUST_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ACTUATORS_LEFT_ENGINE_THRUST(buffer.read_double());
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
		}
        _New_ACTUATORS_ENGINES = true;
    } 
    else if (theObject == _AIRCRAFT_POSITION_ObjectHandle)
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
            if      (parmHandle == _LONGITUDE_ATTRIBUTE) 		
            { 
				_DataLogger.SET_AIRCRAFT_POSITION_LONGITUDE(buffer.read_double());
			}
            else if (parmHandle == _LATITUDE_ATTRIBUTE)
            { 
				_DataLogger.SET_AIRCRAFT_POSITION_LATITUDE(buffer.read_double()); 
			}
            else if (parmHandle == _ALTITUDE_ATTRIBUTE)
            { 
				_DataLogger.SET_AIRCRAFT_POSITION_ALTITUDE(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_AIRCRAFT_POSITION = true;
    } 
    else if (theObject == _AIRCRAFT_ORIENTATION_ObjectHandle)
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
            if      (parmHandle == _PHI_ATTRIBUTE) 		
            { 
				_DataLogger.SET_AIRCRAFT_ORIENTATION_PHI(buffer.read_double()); 
			}
            else if (parmHandle == _THETA_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_ORIENTATION_THETA(buffer.read_double());
			}
            else if (parmHandle == _PSI_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_ORIENTATION_PSI(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_AIRCRAFT_ORIENTATION = true;
    } 
    else if (theObject == _AIRCRAFT_UVW_SPEED_ObjectHandle)
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
            if      (parmHandle == _U_SPEED_ATTRIBUTE) 		
            { 
				_DataLogger.SET_AIRCRAFT_UVW_SPEED_U_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _V_SPEED_ATTRIBUTE)  	
            { 
				_DataLogger.SET_AIRCRAFT_UVW_SPEED_V_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _W_SPEED_ATTRIBUTE)  	
            { 
				_DataLogger.SET_AIRCRAFT_UVW_SPEED_W_SPEED(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}

        }    
        _New_AIRCRAFT_UVW_SPEED = true;
    } 
    else if (theObject == _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle)
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
            if      (parmHandle == _P_ANG_SPEED_ATTRIBUTE) 		
            { 
				_DataLogger.SET_AIRCRAFT_PQR_ANGULAR_SPEED_P_ANG_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _Q_ANG_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_PQR_ANGULAR_SPEED_Q_ANG_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _R_ANG_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_PQR_ANGULAR_SPEED_R_ANG_SPEED(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_AIRCRAFT_PQR_ANGULAR_SPEED = true;
    } 
    else if (theObject == _AIRCRAFT_ACCELERATION_ObjectHandle)
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
            if      (parmHandle == _X_ACC_ATTRIBUTE) 		
            { 
				_DataLogger.SET_AIRCRAFT_ACCELERATION_X_ACC(buffer.read_double()); 
			}
            else if (parmHandle == _Y_ACC_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_ACCELERATION_Y_ACC(buffer.read_double()); 
			}
            else if (parmHandle == _Z_ACC_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_ACCELERATION_Z_ACC(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}

        }    
        _New_AIRCRAFT_ACCELERATION = true;
    } 
    else if (theObject == _AIRCRAFT_SPEED_ObjectHandle)
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
            if      (parmHandle == _INDICATED_AIRSPEED_ATTRIBUTE) 		
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_INDICATED_AIRSPEED(buffer.read_double());
			}
            else if (parmHandle == _EQUIVALENT_AIRSPEED_ATTRIBUTE) 
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_EQUIVALENT_AIRSPEED(buffer.read_double());
			}
            else if (parmHandle == _CALIBRATED_AIRSPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_CALIBRATED_AIRSPEED(buffer.read_double());
			}
            else if (parmHandle == _TRUE_AIRSPEED_ATTRIBUTE)  			
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_TRUE_AIRSPEED(buffer.read_double());
			}
            else if (parmHandle == _GROUND_SPEED_ATTRIBUTE)  			
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_GROUND_SPEED(buffer.read_double());
			}
            else if (parmHandle == _VERTICAL_SPEED_ATTRIBUTE)  			
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_VERTICAL_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _MACH_NUMBER_ATTRIBUTE)  			
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_MACH_NUMBER(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
            
        }    
        _New_AIRCRAFT_SPEED = true;
    } 
    else if (theObject == _AIRCRAFT_ADDITIONAL_ObjectHandle)
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
            if      (parmHandle == _ALPHA_ATTRIBUTE) 				
            { 
				_DataLogger.SET_AIRCRAFT_ADDITIONAL_ALPHA(buffer.read_double());  
			}
            else if (parmHandle == _BETA_ATTRIBUTE)  				
            { 
				_DataLogger.SET_AIRCRAFT_ADDITIONAL_BETA(buffer.read_double()); 
			}
            else if (parmHandle == _DYNAMIC_PRESSURE_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_ADDITIONAL_DYNAMIC_PRESSURE(buffer.read_double()); 
			}
            else if (parmHandle == _TANK_FILLING_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_ADDITIONAL_TANK_FILLING(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_AIRCRAFT_ADDITIONAL = true;
    } 
   else if (theObject == _MEAS_AIRCRAFT_POSITION_ObjectHandle){
        for (unsigned int j=0 ; j<theAttributes.size(); j++) {

            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();
            if      (parmHandle == _MEAS_LONGITUDE_ATTRIBUTE) 		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_POSITION_MEAS_LONGITUDE(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_LATITUDE_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_POSITION_MEAS_LATITUDE(buffer.read_double());
			}
            else if (parmHandle == _MEAS_ALTITUDE_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_POSITION_MEAS_ALTITUDE(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_MEAS_AIRCRAFT_POSITION = true;
    } 
    else if (theObject == _MEAS_AIRCRAFT_ORIENTATION_ObjectHandle)
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
				_DataLogger.SET_MEAS_AIRCRAFT_ORIENTATION_MEAS_PHI(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_THETA_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ORIENTATION_MEAS_THETA(buffer.read_double());  
			}
            else if (parmHandle == _MEAS_PSI_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ORIENTATION_MEAS_PSI(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}

        }    
        _New_MEAS_AIRCRAFT_ORIENTATION = true;
    } 
    else if (theObject == _MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle)
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
				_DataLogger.SET_MEAS_AIRCRAFT_UVW_SPEED_MEAS_U_SPEED(buffer.read_double());
			}
            else if (parmHandle == _MEAS_V_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_UVW_SPEED_MEAS_V_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_W_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_UVW_SPEED_MEAS_W_SPEED(buffer.read_double());  
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_UVW_SPEED = true;
    } 
    else if (theObject == _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle)
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
				_DataLogger.SET_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_MEAS_P_ANG_SPEED(buffer.read_double());  
			}
            else if (parmHandle == _MEAS_R_ANG_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_MEAS_R_ANG_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_Q_ANG_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_MEAS_Q_ANG_SPEED(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = true;
    } 
    else if (theObject == _MEAS_AIRCRAFT_ACCELERATION_ObjectHandle)
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
				_DataLogger.SET_MEAS_AIRCRAFT_ACCELERATION_MEAS_X_ACC(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_Y_ACC_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ACCELERATION_MEAS_Y_ACC(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_Z_ACC_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ACCELERATION_MEAS_Z_ACC(buffer.read_double());
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_ACCELERATION = true;
    }  
    else if (theObject == _MEAS_AIRCRAFT_SPEED_ObjectHandle)
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
				_DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_INDICATED_AIRSPEED(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE)  	
            {
				 _DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_EQUIVALENT_AIRSPEED(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE)  	
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_CALIBRATED_AIRSPEED(buffer.read_double());
			}
            else if (parmHandle == _MEAS_TRUE_AIRSPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_TRUE_AIRSPEED(buffer.read_double());  
			}
            else if (parmHandle == _MEAS_GROUND_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_GROUND_SPEED(buffer.read_double());
			}
            else if (parmHandle == _MEAS_VERTICAL_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_VERTICAL_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_MACH_NUMBER_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_MACH_NUMBER(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}    
        }    
        _New_MEAS_AIRCRAFT_SPEED = true;
    } 
    else if (theObject == _MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle)
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
				_DataLogger.SET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_ALPHA(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_BETA_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_BETA(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_DYNAMIC_PRESSURE_ATTRIBUTE)  	
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_DYNAMIC_PRESSURE(buffer.read_double());
			}
            else if (parmHandle == _MEAS_TANK_FILLING_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_TANK_FILLING(buffer.read_double());
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_ADDITIONAL = true;
    }    
    else if (theObject == _ENVIRONMENT_VARIABLES_ObjectHandle)
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
            if      (parmHandle == _TEMPERATURE_ATTRIBUTE) 		
            { 
				_DataLogger.SET_ENVIRONMENT_VARIABLES_TEMPERATURE(buffer.read_double()); 
			}
            else if (parmHandle == _DENSITY_OF_AIR_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ENVIRONMENT_VARIABLES_DENSITY_OF_AIR(buffer.read_double()); 
			}
            else if (parmHandle == _PRESSURE_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ENVIRONMENT_VARIABLES_PRESSURE(buffer.read_double()); 
			}
            else if (parmHandle == _SPEED_OF_SOUND_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ENVIRONMENT_VARIABLES_SPEED_OF_SOUND(buffer.read_double());
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}      
        }    
        _New_ENVIRONMENT_VARIABLES = true;
    } 
    else if (theObject == _WIND_COMPONENTS_ObjectHandle)
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
            if      (parmHandle == _U_WIND_ATTRIBUTE) 		
            { 
				_DataLogger.SET_WIND_COMPONENTS_U_WIND(buffer.read_double()); 
			}
            else if (parmHandle == _V_WIND_ATTRIBUTE)  		
            { 
				_DataLogger.SET_WIND_COMPONENTS_V_WIND(buffer.read_double());
			}
            else if (parmHandle == _W_WIND_ATTRIBUTE)  		
            { 
				_DataLogger.SET_WIND_COMPONENTS_W_WIND(buffer.read_double());
			}
            else if (parmHandle == _P_WIND_ATTRIBUTE)  		
            { 
				_DataLogger.SET_WIND_COMPONENTS_P_WIND(buffer.read_double()); 
			}
            else if (parmHandle == _Q_WIND_ATTRIBUTE)  		
            { 
				_DataLogger.SET_WIND_COMPONENTS_Q_WIND(buffer.read_double()); 
			}
            else if (parmHandle == _R_WIND_ATTRIBUTE)  		
            { 
				_DataLogger.SET_WIND_COMPONENTS_R_WIND(buffer.read_double()); 
			}
            else
            { 
				cout << "DataLoggerFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_WIND_COMPONENTS = true;
    } 
}


// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void DataLoggerFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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
	#ifdef DEBUG_DATA_LOGGER_FED
	std::cout << "reflectAttributeValues received with ts " <<  ((RTIfedTime&)theTime).getTime() << std::endl;
	#endif 
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void DataLoggerFederateHla13::timeRegulationEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeRegulationWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeReg = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void DataLoggerFederateHla13::timeConstrainedEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeConstrainedWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeConst = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeAdvanceGrant
void DataLoggerFederateHla13::timeAdvanceGrant(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::TimeAdvanceWasNotInProgress,
            RTI::FederateInternalError) 
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_DATA_LOGGER_FED
	std::cout << " >> TAG RECU == LocalTime = " <<  _LocalTime << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void DataLoggerFederateHla13::enableTimeRegulation()
{
	_RtiAmb.enableTimeRegulation(_LocalTime, _Lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla13::disableTimeRegulation()
{
	_RtiAmb.disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla13::enableTimeConstrained()
{
	_RtiAmb.enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla13::disableTimeConstrained()
{
	_RtiAmb.disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla13::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb.enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla13::disableAsynchronousDelivery()
{
	_RtiAmb.disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla13::timeAdvanceRequest(RTIfedTime NextLogicalTime)
{
	_RtiAmb.timeAdvanceRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla13::timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime)
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
void DataLoggerFederateHla13::nextEventRequest(RTIfedTime NextLogicalTime)
{
	_RtiAmb.nextEventRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
// function : nextEventAvailable
void DataLoggerFederateHla13::nextEventAvailable(RTIfedTime NextLogicalTime)
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
void DataLoggerFederateHla13::synchronizationPointRegistrationSucceeded(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegSuccess = true ;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void DataLoggerFederateHla13::synchronizationPointRegistrationFailed(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegFailed = true ;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void DataLoggerFederateHla13::announceSynchronizationPoint(const char *label, const char *tag)
    throw ( RTI::FederateInternalError) 
{
       _InPause = true ;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void DataLoggerFederateHla13::federationSynchronized(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _InPause = false ;
} 

// ----------------------------------------------------------------------------
// function : run
void DataLoggerFederateHla13::run()
{
	while (_LocalTime < _SimulationEndTime)
	{
		clock_gettime(CLOCK_MONOTONIC, &_TimeStamp_old);
		// Model calculations
		calculateState(); 
		calculateOutput();
		timeAdvanceRequest(_TimeStep);
		clock_gettime(CLOCK_MONOTONIC, &_TimeStamp);
		if ((_TimeStamp.tv_nsec-_TimeStamp_old.tv_nsec)<0) 
		{  
			_ExecutionTime.tv_sec = _TimeStamp.tv_sec-_TimeStamp_old.tv_sec-1;  
			_ExecutionTime.tv_nsec = 1000000000+_TimeStamp.tv_nsec-_TimeStamp_old.tv_nsec;  
		} 
		else 
		{  
			_ExecutionTime.tv_sec = _TimeStamp.tv_sec-_TimeStamp_old.tv_sec;  
			_ExecutionTime.tv_nsec = _TimeStamp.tv_nsec-_TimeStamp_old.tv_nsec;  
		} 
		_TotalRealTimeMs = _TotalRealTimeMs + ((_ExecutionTime.tv_nsec) / 1000000);
	}
	_SdseDataLog.close();	
} 
