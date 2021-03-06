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

#include "FlightDynamicsFederateHla13.hh"

// ----------------------------------------------------------------------------
// FlightDynamicsFederateHla13 Constructor
FlightDynamicsFederateHla13::FlightDynamicsFederateHla13( std::wstring FederationName
									, std::wstring FederateName
									, std::wstring FomFileName
									) 
									: NullFederateAmbassador()
									, _RtiAmb() 
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "FlightDynamicsFederateHla13.cc -> Constructor(): Start" << std::endl;
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
	_IsCreator = true;
	_SyncRegSuccess = _SyncRegFailed = _InPause = false ;
	_IsOutMesTimespamped = true;
	
    _Discov_ACTUATORS_CONTROL_SURFACES = false;
    _Discov_ACTUATORS_ENGINES          = false;
    _Discov_ENVIRONMENT_VARIABLES      = false;
    _Discov_WIND_COMPONENTS            = false;
    _New_ACTUATORS_CONTROL_SURFACES = false;
    _New_ACTUATORS_ENGINES          = false;
    _New_ENVIRONMENT_VARIABLES      = false;
    _New_WIND_COMPONENTS            = false;
	
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "FlightDynamicsFederateHla13.cc -> Constructor(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// FlightDynamicsFederateHla13 Destructor
FlightDynamicsFederateHla13::~FlightDynamicsFederateHla13()
	                     throw(RTI::FederateInternalError)
{

}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void FlightDynamicsFederateHla13::createFederationExecution() 
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "FlightDynamicsFederateHla13.cc -> createFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> createFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void FlightDynamicsFederateHla13::destroyFederationExecution() 
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "FlightDynamicsFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "FlightDynamicsFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla13::joinFederationExecution() 
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "FlightDynamicsFederateHla13.cc -> joinFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "FlightDynamicsFederateHla13.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla13::resignFederationExecution() 
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb.resignFederationExecution(RTI::DELETE_OBJECTS_AND_RELEASE_ATTRIBUTES);
	}
	catch (RTI::Exception& e) 
	{
		std::cout << "RFE: caught " << e._name << " reason " << e._reason <<std::endl;
	}
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "FlightDynamicsFederateHla13.cc -> resignFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
RTI::FederateHandle FlightDynamicsFederateHla13::getFederateHandle() const
{
    return _FederateHandle;
}

// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void FlightDynamicsFederateHla13::setHlaTimeManagementSettings( RTIfedTime TimeStep
                                                    , RTIfedTime Lookahead
                                                    , RTIfedTime LocalTime
                                                    , RTIfedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void FlightDynamicsFederateHla13::getAllHandles()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed
		_ACTUATORS_ClassHandle = _RtiAmb.getObjectClassHandle("ACTUATORS");
        _ENVIRONMENT_VARIABLES_ClassHandle = _RtiAmb.getObjectClassHandle("ENVIRONMENT_VARIABLES");
        _WIND_COMPONENTS_ClassHandle = _RtiAmb.getObjectClassHandle("WIND_COMPONENTS");
		_RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_AILERON_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
		_LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_AILERON_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);
		_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
		_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ELEVATOR_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);
		_RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RUDDER_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);            
		_RIGHT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_THRUST",_ACTUATORS_ClassHandle);
		_LEFT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ENGINE_THRUST", _ACTUATORS_ClassHandle);
		_FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("FLAPS_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);
		_SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("SPOILERS_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);
		_STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("STABILIZER_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
		_GEARS_POSITION_ATTRIBUTE = _RtiAmb.getAttributeHandle("GEARS_POSITION", _ACTUATORS_ClassHandle);
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
        // Published
        _TRIM_DATA_ClassHandle = _RtiAmb.getObjectClassHandle("TRIM_DATA");
        _AIRCRAFT_POSITION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_POSITION");
        _AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ORIENTATION");
        _AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_UVW_SPEED");
        _AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_PQR_ANGULAR_SPEED");
        _AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ACCELERATION");
        _AIRCRAFT_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_SPEED");
		_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ADDITIONAL"); 
		_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_THRUST_EQ",      _TRIM_DATA_ClassHandle); 
        _LEFT_ENGINE_THRUST_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ENGINE_THRUST_EQ",       _TRIM_DATA_ClassHandle);
        _ELEVATOR_DEFLECTION_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("ELEVATOR_DEFLECTION_EQ",_TRIM_DATA_ClassHandle);
        _STABILIZER_DEFLECTION_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("STABILIZER_DEFLECTION_EQ", _TRIM_DATA_ClassHandle);
  		_LONGITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("LONGITUDE",_AIRCRAFT_POSITION_ClassHandle);
        _LATITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("LATITUDE", _AIRCRAFT_POSITION_ClassHandle);
        _ALTITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("ALTITUDE", _AIRCRAFT_POSITION_ClassHandle);  
  		_PHI_ATTRIBUTE = _RtiAmb.getAttributeHandle("PHI",  _AIRCRAFT_ORIENTATION_ClassHandle);
        _THETA_ATTRIBUTE = _RtiAmb.getAttributeHandle("THETA",_AIRCRAFT_ORIENTATION_ClassHandle);
        _PSI_ATTRIBUTE = _RtiAmb.getAttributeHandle("PSI",  _AIRCRAFT_ORIENTATION_ClassHandle);    
        _U_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("U_SPEED",_AIRCRAFT_UVW_SPEED_ClassHandle);
        _V_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("V_SPEED",_AIRCRAFT_UVW_SPEED_ClassHandle);
        _W_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("W_SPEED",_AIRCRAFT_UVW_SPEED_ClassHandle);  
        _P_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("P_ANG_SPEED",_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _Q_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("Q_ANG_SPEED",_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _R_ANG_SPEED_ATTRIBUTE =_RtiAmb.getAttributeHandle("R_ANG_SPEED",_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle); 
        _X_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("X_ACC",_AIRCRAFT_ACCELERATION_ClassHandle);
        _Y_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("Y_ACC",_AIRCRAFT_ACCELERATION_ClassHandle);
        _Z_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("Z_ACC",_AIRCRAFT_ACCELERATION_ClassHandle);
        _INDICATED_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("INDICATED_AIRSPEED", _AIRCRAFT_SPEED_ClassHandle);
        _EQUIVALENT_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("EQUIVALENT_AIRSPEED",_AIRCRAFT_SPEED_ClassHandle);
        _CALIBRATED_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("CALIBRATED_AIRSPEED",_AIRCRAFT_SPEED_ClassHandle);
        _TRUE_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("TRUE_AIRSPEED",_AIRCRAFT_SPEED_ClassHandle);
        _GROUND_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("GROUND_SPEED",_AIRCRAFT_SPEED_ClassHandle);
        _VERTICAL_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("VERTICAL_SPEED",_AIRCRAFT_SPEED_ClassHandle);
        _MACH_NUMBER_ATTRIBUTE = _RtiAmb.getAttributeHandle("MACH_NUMBER",_AIRCRAFT_SPEED_ClassHandle);
        _ALPHA_ATTRIBUTE = _RtiAmb.getAttributeHandle("ALPHA",_AIRCRAFT_ADDITIONAL_ClassHandle);
        _BETA_ATTRIBUTE = _RtiAmb.getAttributeHandle("BETA", _AIRCRAFT_ADDITIONAL_ClassHandle);
        _DYNAMIC_PRESSURE_ATTRIBUTE = _RtiAmb.getAttributeHandle("DYNAMIC_PRESSURE",_AIRCRAFT_ADDITIONAL_ClassHandle);
        _TANK_FILLING_ATTRIBUTE = _RtiAmb.getAttributeHandle("TANK_FILLING",_AIRCRAFT_ADDITIONAL_ClassHandle);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void FlightDynamicsFederateHla13::publishAndSubscribe()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		attr_ACTUATORS.reset(RTI::AttributeHandleSetFactory::create(11));
		attr_ENVIRONMENT_VARIABLES.reset(RTI::AttributeHandleSetFactory::create(4));
		attr_WIND_COMPONENTS.reset(RTI::AttributeHandleSetFactory::create(6));
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
		
		_RtiAmb.subscribeObjectClassAttributes(_ACTUATORS_ClassHandle,*attr_ACTUATORS);
        _RtiAmb.subscribeObjectClassAttributes(_ENVIRONMENT_VARIABLES_ClassHandle,*attr_ENVIRONMENT_VARIABLES);
        _RtiAmb.subscribeObjectClassAttributes(_WIND_COMPONENTS_ClassHandle,*attr_WIND_COMPONENTS);

		// For Class/Attributes published
		attr_TRIM_DATA.reset(RTI::AttributeHandleSetFactory::create(4));
		attr_AIRCRAFT_POSITION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_ORIENTATION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_UVW_SPEED.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_PQR_ANGULAR_SPEED.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_ACCELERATION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_SPEED.reset(RTI::AttributeHandleSetFactory::create(7));
		attr_AIRCRAFT_ADDITIONAL.reset(RTI::AttributeHandleSetFactory::create(4));
		attr_TRIM_DATA->add(_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE);
		attr_TRIM_DATA->add(_LEFT_ENGINE_THRUST_EQ_ATTRIBUTE);
		attr_TRIM_DATA->add(_ELEVATOR_DEFLECTION_EQ_ATTRIBUTE);
		attr_TRIM_DATA->add(_STABILIZER_DEFLECTION_EQ_ATTRIBUTE);
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
		
        _RtiAmb.publishObjectClass(_TRIM_DATA_ClassHandle,*attr_TRIM_DATA);
        _RtiAmb.publishObjectClass(_AIRCRAFT_POSITION_ClassHandle,*attr_AIRCRAFT_POSITION);
        _RtiAmb.publishObjectClass(_AIRCRAFT_ORIENTATION_ClassHandle,*attr_AIRCRAFT_ORIENTATION);
        _RtiAmb.publishObjectClass(_AIRCRAFT_UVW_SPEED_ClassHandle,*attr_AIRCRAFT_UVW_SPEED);
        _RtiAmb.publishObjectClass(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,*attr_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb.publishObjectClass(_AIRCRAFT_ACCELERATION_ClassHandle,*attr_AIRCRAFT_ACCELERATION);
        _RtiAmb.publishObjectClass(_AIRCRAFT_SPEED_ClassHandle,*attr_AIRCRAFT_SPEED);
        _RtiAmb.publishObjectClass(_AIRCRAFT_ADDITIONAL_ClassHandle,*attr_AIRCRAFT_ADDITIONAL);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }   
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void FlightDynamicsFederateHla13::registerObjectInstances()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
        _TRIM_DATA_ObjectHandle = _RtiAmb.registerObjectInstance(_TRIM_DATA_ClassHandle,"Trim_Data_Flight_Dynamics");
        _AIRCRAFT_POSITION_ObjectHandle = _RtiAmb.registerObjectInstance(_AIRCRAFT_POSITION_ClassHandle,"Aircraft_Position");
        _AIRCRAFT_ORIENTATION_ObjectHandle = _RtiAmb.registerObjectInstance(_AIRCRAFT_ORIENTATION_ClassHandle,"Aircraft_Orientation");
        _AIRCRAFT_UVW_SPEED_ObjectHandle = _RtiAmb.registerObjectInstance(_AIRCRAFT_UVW_SPEED_ClassHandle,"Aircraft_UVW_Speed");
        _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle = _RtiAmb.registerObjectInstance(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,"Aircraft_PQR_Angular_Speed");
        _AIRCRAFT_ACCELERATION_ObjectHandle = _RtiAmb.registerObjectInstance(_AIRCRAFT_ACCELERATION_ClassHandle,"Aircraft_Acceleration");
        _AIRCRAFT_SPEED_ObjectHandle = _RtiAmb.registerObjectInstance(_AIRCRAFT_SPEED_ClassHandle,"Aircraft_Speed");
        _AIRCRAFT_ADDITIONAL_ObjectHandle = _RtiAmb.registerObjectInstance(_AIRCRAFT_ADDITIONAL_ClassHandle,"Aircraft_Additional");
        ahvps_TRIM_DATA.reset(RTI::AttributeSetFactory::create(4));
		ahvps_AIRCRAFT_POSITION.reset(RTI::AttributeSetFactory::create(3));
		ahvps_AIRCRAFT_ORIENTATION.reset(RTI::AttributeSetFactory::create(3));
		ahvps_AIRCRAFT_UVW_SPEED.reset(RTI::AttributeSetFactory::create(3));
		ahvps_AIRCRAFT_PQR_ANGULAR_SPEED.reset(RTI::AttributeSetFactory::create(3));
		ahvps_AIRCRAFT_ACCELERATION.reset(RTI::AttributeSetFactory::create(3));
		ahvps_AIRCRAFT_SPEED.reset(RTI::AttributeSetFactory::create(7));
		ahvps_AIRCRAFT_ADDITIONAL.reset(RTI::AttributeSetFactory::create(4));
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void FlightDynamicsFederateHla13::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb.unsubscribeObjectClass(_ACTUATORS_ClassHandle);
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
       
    try 
    {
        _RtiAmb.unpublishObjectClass(_TRIM_DATA_ClassHandle);
        _RtiAmb.unpublishObjectClass(_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb.unpublishObjectClass(_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb.unpublishObjectClass(_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb.unpublishObjectClass(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb.unpublishObjectClass(_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb.unpublishObjectClass(_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb.unpublishObjectClass(_AIRCRAFT_ADDITIONAL_ClassHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void FlightDynamicsFederateHla13::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_Discov_ACTUATORS_CONTROL_SURFACES ||
            !_Discov_ACTUATORS_ENGINES          ||
            !_Discov_ENVIRONMENT_VARIABLES      ||
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
    _Discov_ACTUATORS_CONTROL_SURFACES = false;
    _Discov_ACTUATORS_ENGINES = false;
    _Discov_ENVIRONMENT_VARIABLES = false;
    _Discov_WIND_COMPONENTS = false;
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void FlightDynamicsFederateHla13::waitForAllAttributesReceived()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_ENVIRONMENT_VARIABLES) 
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
    
    _New_ENVIRONMENT_VARIABLES = false;
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void FlightDynamicsFederateHla13::discoverObjectInstance( RTI::ObjectHandle theObject
                                                   , RTI::ObjectClassHandle theObjectClass
                                                   , const char *theObjectName
                                                   )
                                             throw ( RTI::CouldNotDiscover
                                                   , RTI::ObjectClassNotKnown
                                                   , RTI::FederateInternalError
                                                   )
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
	#endif
	if ( (theObjectClass == _ACTUATORS_ClassHandle) && (!strcmp(theObjectName,"Actuators_Control_Surfaces")) ) 
	{
		_Discov_ACTUATORS_CONTROL_SURFACES = true;
		_ACTUATORS_CONTROL_SURFACES_ObjectHandle = theObject;
		#ifdef DEBUG_FLIGH_DYNAMICS_FED
		cout << "< Actuators_Control_Surfaces > Object Instance has been discovered" << endl;
		#endif
	} 
	else if ( (theObjectClass == _ACTUATORS_ClassHandle) && (!strcmp(theObjectName,"Actuators_Engines")) ) 
	{
		_Discov_ACTUATORS_ENGINES = true;
		_ACTUATORS_ENGINES_ObjectHandle = theObject;
		#ifdef DEBUG_FLIGH_DYNAMICS_FED
		cout << "< Actuators_Engines > Object Instance has been discovered" << endl;
		#endif
	} 
	else if ( (theObjectClass == _ENVIRONMENT_VARIABLES_ClassHandle) && (!strcmp(theObjectName,"Environment_Variables")) ) 
	{
		_Discov_ENVIRONMENT_VARIABLES = true;
		_ENVIRONMENT_VARIABLES_ObjectHandle = theObject;
		#ifdef DEBUG_FLIGH_DYNAMICS_FED
		cout << "< Environment_Variables > Object Instance has been discovered" << endl;
		#endif
	} 
	else if ( (theObjectClass == _WIND_COMPONENTS_ClassHandle) && (!strcmp(theObjectName,"Wind_Components")) ) 
	{
		_Discov_WIND_COMPONENTS = true;
		_WIND_COMPONENTS_ObjectHandle = theObject;
		#ifdef DEBUG_FLIGH_DYNAMICS_FED
		cout << "< Wind_Components > Object Instance has been discovered" << endl;
		#endif
	} 
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void FlightDynamicsFederateHla13::pauseInitTrim()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> pause_init_trim(): Start" << std::endl;
    #endif
	std::cout << ">> PRESS ENTER TO START TRIMMING " << endl;      
	std::cin.get();
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void FlightDynamicsFederateHla13::pauseInitSim()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> pause_init_sim(): Start" << std::endl;
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::cout << "FlightDynamicsFederateHla13.cc -> pause_init_sim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void FlightDynamicsFederateHla13::initialization() 
{
	waitForAllObjectDiscovered();
	sendInitialAttributesPos();
	waitForAllAttributesReceived();
	_Aircraft.initialization(); 
	sendInitialAttributesTrim();
	sendInitialAllAttributes();
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void FlightDynamicsFederateHla13::calculateState() 
{
	// Model State calculation
	 _Aircraft.calculate_state(); 
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void FlightDynamicsFederateHla13::calculateOutput() 
{
	// Model output calculation
	_Aircraft.calculate_output(); 
}

// ----------------------------------------------------------------------------
// Send Updates for init attributes position
void FlightDynamicsFederateHla13::sendInitialAttributesPos()
{
	// update attribute for initial AIRCRAFT_POSITION 
	//std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_AIRCRAFT_POSITION(RTI::AttributeSetFactory::create(3));
	ahvps_AIRCRAFT_POSITION->empty();
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getLongitude());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_POSITION -> add(_LONGITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getLatitude());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_POSITION -> add(_LATITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAltitude());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_POSITION -> add(_ALTITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  

    try 
    {
		_RtiAmb.updateAttributeValues(_AIRCRAFT_POSITION_ObjectHandle, *ahvps_AIRCRAFT_POSITION, "Aircraft_Position");

    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Send Updates for init attributes position
void FlightDynamicsFederateHla13::sendInitialAttributesTrim()
{
	// update attribute for initial TRIM_DATA 
	ahvps_TRIM_DATA->empty();
	std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_TRIM_DATA(RTI::AttributeSetFactory::create(4));
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getThrustRightEngine());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_TRIM_DATA -> add(_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getThrustLeftEngine());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_TRIM_DATA -> add(_LEFT_ENGINE_THRUST_EQ_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getDeltaRightElevatorRad());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_TRIM_DATA -> add(_ELEVATOR_DEFLECTION_EQ_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getDeltaHSRad());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_TRIM_DATA -> add(_STABILIZER_DEFLECTION_EQ_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

    try 
    {
		_RtiAmb.updateAttributeValues(_TRIM_DATA_ObjectHandle, *ahvps_TRIM_DATA, "Trim_Data");

    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void FlightDynamicsFederateHla13::sendInitialAllAttributes()
{
	//std::cout << "FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;
	ahvps_AIRCRAFT_POSITION          -> empty();
    ahvps_AIRCRAFT_ORIENTATION       -> empty();
    ahvps_AIRCRAFT_UVW_SPEED         -> empty();
    ahvps_AIRCRAFT_PQR_ANGULAR_SPEED -> empty();
    ahvps_AIRCRAFT_ACCELERATION      -> empty();
    ahvps_AIRCRAFT_SPEED             -> empty();
    ahvps_AIRCRAFT_ADDITIONAL        -> empty();  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getLongitude());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_POSITION -> add(_LONGITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getLatitude());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_POSITION -> add(_LATITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAltitude());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_POSITION -> add(_ALTITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getPhi());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ORIENTATION -> add(_PHI_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getTheta());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ORIENTATION -> add(_THETA_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getPsi());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ORIENTATION -> add(_PSI_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getU());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_UVW_SPEED -> add(_U_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getV());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_UVW_SPEED -> add(_V_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getW());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_UVW_SPEED -> add(_W_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getP());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_PQR_ANGULAR_SPEED -> add(_P_ANG_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getQ());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_PQR_ANGULAR_SPEED -> add(_Q_ANG_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getR());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_PQR_ANGULAR_SPEED -> add(_R_ANG_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAx());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ACCELERATION -> add(_X_ACC_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAy());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ACCELERATION -> add(_Y_ACC_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());   
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAz());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ACCELERATION -> add(_Z_ACC_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getIAS());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_INDICATED_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getEAS());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_EQUIVALENT_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getCAS());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_CALIBRATED_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getTAS());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_TRUE_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getGS());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_GROUND_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getVS());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_VERTICAL_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getMach());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_MACH_NUMBER_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAlpha());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ADDITIONAL -> add(_ALPHA_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getBeta());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ADDITIONAL -> add(_BETA_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());   
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getQbar());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ADDITIONAL -> add(_DYNAMIC_PRESSURE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getTank());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ADDITIONAL -> add(_TANK_FILLING_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

    try 
    {
		_RtiAmb.updateAttributeValues(_AIRCRAFT_POSITION_ObjectHandle, *ahvps_AIRCRAFT_POSITION, "Aircraft_Position");
		_RtiAmb.updateAttributeValues(_AIRCRAFT_ORIENTATION_ObjectHandle, *ahvps_AIRCRAFT_ORIENTATION, "Aircraft_Orientation");
		_RtiAmb.updateAttributeValues(_AIRCRAFT_UVW_SPEED_ObjectHandle, *ahvps_AIRCRAFT_UVW_SPEED, "Aircraft_UVW_Speed");
		_RtiAmb.updateAttributeValues(_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle, *ahvps_AIRCRAFT_PQR_ANGULAR_SPEED, "Aircraft_PQR_Angular_Speed");
		_RtiAmb.updateAttributeValues(_AIRCRAFT_ACCELERATION_ObjectHandle, *ahvps_AIRCRAFT_ACCELERATION, "Aircraft_UVW_Speed");
		_RtiAmb.updateAttributeValues(_AIRCRAFT_SPEED_ObjectHandle, *ahvps_AIRCRAFT_SPEED, "Aircraft_Speed");
		_RtiAmb.updateAttributeValues(_AIRCRAFT_ADDITIONAL_ObjectHandle, *ahvps_AIRCRAFT_ADDITIONAL, "Aircraft_Additional");
    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes Phase 1
void FlightDynamicsFederateHla13::sendUpdateAttributes1(const RTI::FedTime& UpdateTime)
{ 
	//std::cout << "FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;
	ahvps_AIRCRAFT_POSITION          -> empty();
    ahvps_AIRCRAFT_ORIENTATION       -> empty();
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getLongitude());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_POSITION -> add(_LONGITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getLatitude());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_POSITION -> add(_LATITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAltitude());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_POSITION -> add(_ALTITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getPhi());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ORIENTATION -> add(_PHI_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getTheta());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ORIENTATION -> add(_THETA_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getPsi());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ORIENTATION -> add(_PSI_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb.updateAttributeValues(_AIRCRAFT_POSITION_ObjectHandle, *ahvps_AIRCRAFT_POSITION, "Aircraft_Position");
            _RtiAmb.updateAttributeValues(_AIRCRAFT_ORIENTATION_ObjectHandle, *ahvps_AIRCRAFT_ORIENTATION, "Aircraft_Orientation");
        }
        else
        {
            _RtiAmb.updateAttributeValues(_AIRCRAFT_POSITION_ObjectHandle, *ahvps_AIRCRAFT_POSITION, UpdateTime, "Aircraft_Position");
            _RtiAmb.updateAttributeValues(_AIRCRAFT_ORIENTATION_ObjectHandle, *ahvps_AIRCRAFT_ORIENTATION, UpdateTime, "Aircraft_Orientation");
        }
    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
	//std::cout << "FlightDynamicsFederateHla13::sendUpdateR: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void FlightDynamicsFederateHla13::sendUpdateAttributes2(const RTI::FedTime& UpdateTime)
{ 
	//std::cout << "FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;
    ahvps_AIRCRAFT_UVW_SPEED         -> empty ();
    ahvps_AIRCRAFT_PQR_ANGULAR_SPEED -> empty ();
    ahvps_AIRCRAFT_ACCELERATION      -> empty ();
    ahvps_AIRCRAFT_SPEED             -> empty ();
    ahvps_AIRCRAFT_ADDITIONAL        -> empty ();  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getU());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_UVW_SPEED -> add(_U_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getV());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_UVW_SPEED -> add(_V_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getW());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_UVW_SPEED -> add(_W_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getP());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_PQR_ANGULAR_SPEED -> add(_P_ANG_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getQ());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_PQR_ANGULAR_SPEED -> add(_Q_ANG_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getR());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_PQR_ANGULAR_SPEED -> add(_R_ANG_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAx());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ACCELERATION -> add(_X_ACC_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAy());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ACCELERATION -> add(_Y_ACC_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());   
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAz());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ACCELERATION -> add(_Z_ACC_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getIAS());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_INDICATED_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getEAS());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_EQUIVALENT_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getCAS());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_CALIBRATED_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getTAS());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_TRUE_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getGS());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_GROUND_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getVS());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_VERTICAL_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getMach());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_SPEED -> add(_MACH_NUMBER_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAlpha());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ADDITIONAL -> add(_ALPHA_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getBeta());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ADDITIONAL -> add(_BETA_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());   
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getQbar());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ADDITIONAL -> add(_DYNAMIC_PRESSURE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getTank());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_AIRCRAFT_ADDITIONAL -> add(_TANK_FILLING_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb.updateAttributeValues(_AIRCRAFT_UVW_SPEED_ObjectHandle, *ahvps_AIRCRAFT_UVW_SPEED, "Aircraft_UVW_Speed");
            _RtiAmb.updateAttributeValues(_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle, *ahvps_AIRCRAFT_PQR_ANGULAR_SPEED, "Aircraft_PQR_Angular_Speed");
            _RtiAmb.updateAttributeValues(_AIRCRAFT_ACCELERATION_ObjectHandle, *ahvps_AIRCRAFT_ACCELERATION, "Aircraft_UVW_Speed");
            _RtiAmb.updateAttributeValues(_AIRCRAFT_SPEED_ObjectHandle, *ahvps_AIRCRAFT_SPEED, "Aircraft_Speed");
            _RtiAmb.updateAttributeValues(_AIRCRAFT_ADDITIONAL_ObjectHandle, *ahvps_AIRCRAFT_ADDITIONAL, "Aircraft_Additional");
        }
        else
        {
            _RtiAmb.updateAttributeValues(_AIRCRAFT_UVW_SPEED_ObjectHandle, *ahvps_AIRCRAFT_UVW_SPEED, UpdateTime, "Aircraft_UVW_Speed");
            _RtiAmb.updateAttributeValues(_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle, *ahvps_AIRCRAFT_PQR_ANGULAR_SPEED, UpdateTime, "Aircraft_PQR_Angular_Speed");
            _RtiAmb.updateAttributeValues(_AIRCRAFT_ACCELERATION_ObjectHandle, *ahvps_AIRCRAFT_ACCELERATION, UpdateTime, "Aircraft_UVW_Speed");
            _RtiAmb.updateAttributeValues(_AIRCRAFT_SPEED_ObjectHandle, *ahvps_AIRCRAFT_SPEED, UpdateTime, "Aircraft_Speed");
            _RtiAmb.updateAttributeValues(_AIRCRAFT_ADDITIONAL_ObjectHandle, *ahvps_AIRCRAFT_ADDITIONAL, UpdateTime, "Aircraft_Additional");
        }
    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
	//std::cout << "FlightDynamicsFederateHla13::sendUpdateR: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void FlightDynamicsFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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

    if (theObject == _ACTUATORS_CONTROL_SURFACES_ObjectHandle)
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
				_Aircraft.setDeltaRightAileronRad(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE)   
            { 
				_Aircraft.setDeltaLeftAileronRad(buffer.read_double()); 
			}
            else if (parmHandle == _RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE) 
            { 
				_Aircraft.setDeltaRightElevatorRad(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE)  
            { 
				_Aircraft.setDeltaLeftElevatorRad(buffer.read_double()); 
			}
            else if (parmHandle == _RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE)
            { 
				_Aircraft.setDeltaRudderRad(buffer.read_double()); 
			}
            else if (parmHandle == _FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE)  
            { 
				_Aircraft.setDeltaFlapRad(buffer.read_double()); 
			}
            else if (parmHandle == _SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE)  
            { 
				_Aircraft.setDeltaSpoilersRad(buffer.read_double()); 
			}
            else if (parmHandle == _STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE)  
            { 
				_Aircraft.setDeltaHSRad(buffer.read_double()); 
			}
            else if (parmHandle == _GEARS_POSITION_ATTRIBUTE)  
            { 
				_Aircraft.setDeltaGear(buffer.read_double()); 
			}
            else
            { 
				cout << "FlightDynamicsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Aircraft.setThrustRightEngine(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_ENGINE_THRUST_ATTRIBUTE)  
            { 
				_Aircraft.setThrustLeftEngine(buffer.read_double()); 
			}
            else
            { 
				cout << "FlightDynamicsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_ACTUATORS_ENGINES = true;
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
				_Aircraft.setTemperature(buffer.read_double()); 
			}
            else if (parmHandle == _DENSITY_OF_AIR_ATTRIBUTE) 
            { 
				_Aircraft.setDensity(buffer.read_double()); 
			}
            else if (parmHandle == _PRESSURE_ATTRIBUTE)       
            { 
				_Aircraft.setPressure(buffer.read_double()); 
			}
            else if (parmHandle == _SPEED_OF_SOUND_ATTRIBUTE) 
            { 
				_Aircraft.setSoundSpeed(buffer.read_double()); 
			}
            else
            { 
				cout << "FlightDynamicsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Aircraft.setUwind(buffer.read_double()); 
			}
            else if (parmHandle == _V_WIND_ATTRIBUTE) 
            { 
				_Aircraft.setVwind(buffer.read_double()); 
			}
            else if (parmHandle == _W_WIND_ATTRIBUTE) 
            { 
				_Aircraft.setWwind(buffer.read_double()); 
			}
            else if (parmHandle == _P_WIND_ATTRIBUTE) 
            { 
				_Aircraft.setPwind(buffer.read_double()); 
			} 
            else if (parmHandle == _Q_WIND_ATTRIBUTE) 
            { 
				_Aircraft.setQwind(buffer.read_double()); 
			}
            else if (parmHandle == _R_WIND_ATTRIBUTE) 
            { 
				_Aircraft.setRwind(buffer.read_double()); 
			}
            else
            { 
				cout << "FlightDynamicsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_WIND_COMPONENTS = true;
    }
	else
	{ 
		cout << "FlightDynamicsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << endl; 
	}
}


// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void FlightDynamicsFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "reflectAttributeValues received with ts " <<  ((RTIfedTime&)theTime).getTime() << std::endl;
	#endif 
}


// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void FlightDynamicsFederateHla13::timeRegulationEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeRegulationWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeReg = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void FlightDynamicsFederateHla13::timeConstrainedEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeConstrainedWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeConst = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeAdvanceGrant
void FlightDynamicsFederateHla13::timeAdvanceGrant(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::TimeAdvanceWasNotInProgress,
            RTI::FederateInternalError) 
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << " >> TAG RECU == LocalTime = " <<  _LocalTime << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla13::enableTimeRegulation()
{
	_RtiAmb.enableTimeRegulation(_LocalTime, _Lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla13::disableTimeRegulation()
{
	_RtiAmb.disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla13::enableTimeConstrained()
{
	_RtiAmb.enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla13::disableTimeConstrained()
{
	_RtiAmb.disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla13::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb.enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla13::disableAsynchronousDelivery()
{
	_RtiAmb.disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla13::timeAdvanceRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.timeAdvanceRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla13::timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime)
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
void FlightDynamicsFederateHla13::nextEventRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.nextEventRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// function : nextEventAvailable
void FlightDynamicsFederateHla13::nextEventAvailable(RTIfedTime NextLogicalTime)
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
void FlightDynamicsFederateHla13::synchronizationPointRegistrationSucceeded(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegSuccess = true ;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void FlightDynamicsFederateHla13::synchronizationPointRegistrationFailed(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegFailed = true ;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void FlightDynamicsFederateHla13::announceSynchronizationPoint(const char *label, const char *tag)
    throw ( RTI::FederateInternalError) 
{
       _InPause = true ;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void FlightDynamicsFederateHla13::federationSynchronized(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _InPause = false ;
} 

// ----------------------------------------------------------------------------
// function : run
void FlightDynamicsFederateHla13::run()
{
	while (_LocalTime < _SimulationEndTime)
	{
		// Model calculations
		calculateState();
		sendUpdateAttributes1(_LocalTime + _Lookahead); 
		calculateOutput();
		nextEventRequest(_TimeStep);
		sendUpdateAttributes2(_LocalTime + _Lookahead);
		timeAdvanceRequest(_TimeStep);
	}
} 

// ----------------------------------------------------------------------------
// function : run
void FlightDynamicsFederateHla13::runOneStep()
{
	// Model calculations
	calculateState();
	sendUpdateAttributes1(_LocalTime + _Lookahead); 
	calculateOutput();
	nextEventRequest(_TimeStep);
	sendUpdateAttributes2(_LocalTime + _Lookahead);
	timeAdvanceRequest(_TimeStep);
} 

bool FlightDynamicsFederateHla13::getEndOfSimulation()
{
	return (_LocalTime < _SimulationEndTime);
}
