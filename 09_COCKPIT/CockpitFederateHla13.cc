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

#include "CockpitFederateHla13.hh"
#include "CockpitFederateThreadHla13.hh"
#include "Cockpit.hh"

// ----------------------------------------------------------------------------
// CockpitFederateHla13 Constructor
CockpitFederateHla13::CockpitFederateHla13( std::wstring FederationName
									, std::wstring FederateName
									, std::wstring FomFileName
									) 
									: NullFederateAmbassador()
									, _RtiAmb() 
{
	#ifdef DEBUG_COCKPIT_FED
	std::cout << "CockpitFederateHla13.cc -> Constructor(): Start" << std::endl;
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
	
	_Discov_MEAS_AIRCRAFT_POSITION          = false;
	_Discov_MEAS_AIRCRAFT_ORIENTATION       = false;
	_Discov_MEAS_AIRCRAFT_UVW_SPEED         = false;
	_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ACCELERATION      = false;
	_Discov_MEAS_AIRCRAFT_SPEED             = false;
	_Discov_MEAS_AIRCRAFT_ADDITIONAL        = false;
	_Discov_ACTUATORS_ENGINES          		= false;
	_Discov_ENVIRONMENT_VARIABLES      		= false;
	_New_MEAS_AIRCRAFT_POSITION             = false;
	_New_MEAS_AIRCRAFT_ORIENTATION          = false;
	_New_MEAS_AIRCRAFT_UVW_SPEED            = false;
	_New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED    = false;
	_New_MEAS_AIRCRAFT_ACCELERATION         = false;
	_New_MEAS_AIRCRAFT_SPEED                = false;
	_New_MEAS_AIRCRAFT_ADDITIONAL           = false;
	_New_ACTUATORS_ENGINES          		= false;
    _New_ENVIRONMENT_VARIABLES      		= false;
	
	#ifdef DEBUG_COCKPIT_FED
	std::cout << "CockpitFederateHla13.cc -> Constructor(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// CockpitFederateHla13 Destructor
CockpitFederateHla13::~CockpitFederateHla13()
	                     throw(RTI::FederateInternalError)
{

}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void CockpitFederateHla13::createFederationExecution() 
{
	#ifdef DEBUG_COCKPIT_FED
	std::cout << "CockpitFederateHla13.cc -> createFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> createFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void CockpitFederateHla13::destroyFederationExecution() 
{
	#ifdef DEBUG_COCKPIT_FED
	std::cout << "CockpitFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_COCKPIT_FED
	std::cout << "CockpitFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void CockpitFederateHla13::joinFederationExecution() 
{
	#ifdef DEBUG_COCKPIT_FED
	std::cout << "CockpitFederateHla13.cc -> joinFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_COCKPIT_FED
	std::cout << "CockpitFederateHla13.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void CockpitFederateHla13::resignFederationExecution() 
{
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb.resignFederationExecution(RTI::DELETE_OBJECTS_AND_RELEASE_ATTRIBUTES);
	}
	catch (RTI::Exception& e) 
	{
		std::cout << "RFE: caught " << e._name << " reason " << e._reason <<std::endl;
	}
	#ifdef DEBUG_COCKPIT_FED
	std::cout << "CockpitFederateHla13.cc -> resignFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
RTI::FederateHandle CockpitFederateHla13::getFederateHandle() const
{
    return _FederateHandle;
}

// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void CockpitFederateHla13::setHlaTimeManagementSettings( RTIfedTime TimeStep
                                                    , RTIfedTime Lookahead
                                                    , RTIfedTime LocalTime
                                                    , RTIfedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void CockpitFederateHla13::getAllHandles()
{
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed
		_MEAS_AIRCRAFT_POSITION_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_POSITION");
		_MEAS_AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_ORIENTATION");
		_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_UVW_SPEED");
		_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_PQR_ANGULAR_SPEED");
		_MEAS_AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_ACCELERATION");
		_MEAS_AIRCRAFT_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_SPEED");
		_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_ADDITIONAL");
		_ACTUATORS_ClassHandle = _RtiAmb.getObjectClassHandle("ACTUATORS");
		_ENVIRONMENT_VARIABLES_ClassHandle = _RtiAmb.getObjectClassHandle("ENVIRONMENT_VARIABLES");
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
        _RIGHT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_THRUST",_ACTUATORS_ClassHandle);
		_LEFT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ENGINE_THRUST", _ACTUATORS_ClassHandle);
		_PRESSURE_ATTRIBUTE = _RtiAmb.getAttributeHandle("PRESSURE",      _ENVIRONMENT_VARIABLES_ClassHandle);
		
        // Published 
        _COCKPIT_ClassHandle = _RtiAmb.getObjectClassHandle("COCKPIT");
        _AUTOPILOT_AP_ACTIVE_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_AP_ACTIVE", _COCKPIT_ClassHandle);
        _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_ATHR_ACTIVE", _COCKPIT_ClassHandle);
        _AUTOPILOT_SPD_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_SPD", _COCKPIT_ClassHandle);
        _AUTOPILOT_HDG_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_HDG", _COCKPIT_ClassHandle);
        _AUTOPILOT_ALT_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_ALT", _COCKPIT_ClassHandle);
        _AUTOPILOT_VS_ATTRIBUTE = _RtiAmb.getAttributeHandle("AUTOPILOT_VS",_COCKPIT_ClassHandle);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void CockpitFederateHla13::publishAndSubscribe()
{
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		attr_MEAS_AIRCRAFT_POSITION.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_ORIENTATION.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_UVW_SPEED.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_ACCELERATION.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_SPEED.reset(RTI::AttributeHandleSetFactory::create(7)) ;
		attr_MEAS_AIRCRAFT_ADDITIONAL.reset(RTI::AttributeHandleSetFactory::create(4)) ;
		attr_ACTUATORS_ENGINES.reset(RTI::AttributeHandleSetFactory::create(2));
		attr_ENVIRONMENT_VARIABLES.reset(RTI::AttributeHandleSetFactory::create(1));
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
		attr_ACTUATORS_ENGINES->add(_RIGHT_ENGINE_THRUST_ATTRIBUTE);
		attr_ACTUATORS_ENGINES->add(_LEFT_ENGINE_THRUST_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES->add(_PRESSURE_ATTRIBUTE);

        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_POSITION_ClassHandle, *attr_MEAS_AIRCRAFT_POSITION);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle, *attr_MEAS_AIRCRAFT_ORIENTATION);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle, *attr_MEAS_AIRCRAFT_UVW_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,*attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle, *attr_MEAS_AIRCRAFT_ACCELERATION);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_SPEED_ClassHandle, *attr_MEAS_AIRCRAFT_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle, *attr_MEAS_AIRCRAFT_ADDITIONAL);
        _RtiAmb.subscribeObjectClassAttributes(_ACTUATORS_ClassHandle, *attr_ACTUATORS_ENGINES);
        _RtiAmb.subscribeObjectClassAttributes(_ENVIRONMENT_VARIABLES_ClassHandle, *attr_ENVIRONMENT_VARIABLES); 

		// For Class/Attributes published
		attr_COCKPIT.reset(RTI::AttributeHandleSetFactory::create(6)) ;    
		attr_COCKPIT->add(_AUTOPILOT_AP_ACTIVE_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_SPD_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_HDG_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_ALT_ATTRIBUTE);
		attr_COCKPIT->add(_AUTOPILOT_VS_ATTRIBUTE);
		
        _RtiAmb.publishObjectClass(_COCKPIT_ClassHandle,*attr_COCKPIT);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }   
    #ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void CockpitFederateHla13::registerObjectInstances()
{
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
        _COCKPIT_ObjectHandle = _RtiAmb.registerObjectInstance(_COCKPIT_ClassHandle,"Cockpit");
        ahvps_COCKPIT.reset(RTI::AttributeSetFactory::create(6));
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void CockpitFederateHla13::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_ACTUATORS_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_ENVIRONMENT_VARIABLES_ClassHandle);
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
        _RtiAmb.unpublishObjectClass(_COCKPIT_ClassHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
    #ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void CockpitFederateHla13::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_Discov_MEAS_AIRCRAFT_POSITION          ||
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
	_Discov_MEAS_AIRCRAFT_POSITION = false;
	_Discov_MEAS_AIRCRAFT_ORIENTATION = false;
	_Discov_MEAS_AIRCRAFT_UVW_SPEED = false;
	_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ACCELERATION = false;
	_Discov_MEAS_AIRCRAFT_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ADDITIONAL = false;
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void CockpitFederateHla13::waitForAllAttributesReceived()
{
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_MEAS_AIRCRAFT_POSITION          ||
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
    _New_MEAS_AIRCRAFT_POSITION = false;
    _New_MEAS_AIRCRAFT_ORIENTATION = false;
    _New_MEAS_AIRCRAFT_UVW_SPEED = false;
    _New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
    _New_MEAS_AIRCRAFT_ACCELERATION = false;
    _New_MEAS_AIRCRAFT_SPEED = false;
    _New_MEAS_AIRCRAFT_ADDITIONAL = false;
    #ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void CockpitFederateHla13::discoverObjectInstance( RTI::ObjectHandle theObject
                                                   , RTI::ObjectClassHandle theObjectClass
                                                   , const char *theObjectName
                                                   )
                                             throw ( RTI::CouldNotDiscover
                                                   , RTI::ObjectClassNotKnown
                                                   , RTI::FederateInternalError
                                                   )
{
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
	if ( (theObjectClass == _MEAS_AIRCRAFT_POSITION_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Position")) ) 
    {
        _Discov_MEAS_AIRCRAFT_POSITION = true;
        _MEAS_AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_COCKPIT_FED
        cout << "< Meas_Aircraft_Position > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_POSITION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ORIENTATION_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Orientation")) ) 
    {
        _Discov_MEAS_AIRCRAFT_ORIENTATION = true;
        _MEAS_AIRCRAFT_ORIENTATION_ObjectHandle = theObject;
        #ifdef DEBUG_COCKPIT_FED
        cout << "< Meas_Aircraft_Orientation > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_ORIENTATION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_UVW_SPEED_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_UVW_Speed")) ) 
    {
        _Discov_MEAS_AIRCRAFT_UVW_SPEED = true;
        _MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_COCKPIT_FED
        cout << "< Meas_Aircraft_UVW_Speed > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_PQR_Angular_Speed")) ) 
    {
        _Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = true;
        _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_COCKPIT_FED
        cout << "< Meas_Aircraft_PQR_Angular_Speed > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ACCELERATION_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Acceleration")) ) 
    {
        _Discov_MEAS_AIRCRAFT_ACCELERATION = true;
        _MEAS_AIRCRAFT_ACCELERATION_ObjectHandle = theObject;
        #ifdef DEBUG_COCKPIT_FED
        cout << "< Meas_Aircraft_Acceleration > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_ACCELERATION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_SPEED_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Speed")) ) 
    {
        _Discov_MEAS_AIRCRAFT_SPEED = true;
        _MEAS_AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_COCKPIT_FED
        cout << "< Meas_Aircraft_Speed > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle) && (!strcmp(theObjectName,"Meas_Aircraft_Additional")) ) 
    {
        _Discov_MEAS_AIRCRAFT_ADDITIONAL = true;
        _MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle = theObject;
        #ifdef DEBUG_COCKPIT_FED
        cout << "< Meas_Aircraft_Additional > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle
             << "."
             << endl;
        #endif
    }   
    else if ( (theObjectClass == _ACTUATORS_ClassHandle) && (!strcmp(theObjectName,"Actuators_Engines")) ) 
    {
        _Discov_ACTUATORS_ENGINES = true;
        _ACTUATORS_ENGINES_ObjectHandle = theObject;
        #ifdef DEBUG_COCKPIT_FED
        cout << "< Actuators_Engines > Object Instance has been discovered with handle "
             << _ACTUATORS_ENGINES_ObjectHandle
             << "."
             << endl;
        #endif
    }
    else if ( (theObjectClass == _ENVIRONMENT_VARIABLES_ClassHandle) && (!strcmp(theObjectName,"Environment_Variables")) ) 
    {
        _Discov_ENVIRONMENT_VARIABLES = true;
        _ENVIRONMENT_VARIABLES_ObjectHandle = theObject;
        #ifdef DEBUG_COCKPIT_FED
        cout << "< Environment_Variables > Object Instance has been discovered with handle "
             << _ENVIRONMENT_VARIABLES_ObjectHandle
             << "."
             << endl;
        #endif
    }
    #ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void CockpitFederateHla13::pauseInitTrim()
{
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void CockpitFederateHla13::pauseInitSim()
{
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> pause_init_sim(): Start" << std::endl;
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
	#ifdef DEBUG_COCKPIT_FED
    std::cout << "CockpitFederateHla13.cc -> pause_init_sim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase 
void CockpitFederateHla13::initialization() 
{
	waitForAllObjectDiscovered();
	waitForAllAttributesReceived();
	sendInitialAllAttributes();
}

// ----------------------------------------------------------------------------
// Calculate State values
void CockpitFederateHla13::calculateState() 
{
	// Model State calculation
	// Not used
}

// ----------------------------------------------------------------------------
// Calculate Ouput values 
void CockpitFederateHla13::calculateOutput() 
{
	// Model output calculation
	// Not used 
}

// ----------------------------------------------------------------------------
// Send Updates for initial attributes
void CockpitFederateHla13::sendInitialAllAttributes()
{   
	//std::cout << "CockpitFederateHla13::sendUpdateAttributes: Start" << std::endl;
	ahvps_COCKPIT->empty();
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla13::window->Get_Autopilot_AP());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_COCKPIT->add(_AUTOPILOT_AP_ACTIVE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla13::window->Get_Autopilot_athr());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_COCKPIT -> add(_AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla13::window->Get_Autopilot_spd());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_COCKPIT -> add(_AUTOPILOT_SPD_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla13::window->Get_Autopilot_hdg());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_COCKPIT -> add(_AUTOPILOT_HDG_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla13::window->Get_Autopilot_alt());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_COCKPIT -> add(_AUTOPILOT_ALT_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla13::window->Get_Autopilot_vs());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_COCKPIT -> add(_AUTOPILOT_VS_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	
    try 
    {
		_RtiAmb.updateAttributeValues(_COCKPIT_ObjectHandle, *ahvps_COCKPIT, "Cockpit");

    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
	//std::cout << "CockpitFederateHla13::sendUpdateR: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void CockpitFederateHla13::sendUpdateAttributes(const RTI::FedTime& UpdateTime)
{   
	//std::cout << "CockpitFederateHla13::sendUpdateAttributes: Start" << std::endl;
	ahvps_COCKPIT->empty();
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla13::window->Get_Autopilot_AP());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_COCKPIT->add(_AUTOPILOT_AP_ACTIVE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla13::window->Get_Autopilot_athr());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_COCKPIT -> add(_AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla13::window->Get_Autopilot_spd());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_COCKPIT -> add(_AUTOPILOT_SPD_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla13::window->Get_Autopilot_hdg());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_COCKPIT -> add(_AUTOPILOT_HDG_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla13::window->Get_Autopilot_alt());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_COCKPIT -> add(_AUTOPILOT_ALT_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla13::window->Get_Autopilot_vs());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_COCKPIT -> add(_AUTOPILOT_VS_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb.updateAttributeValues(_COCKPIT_ObjectHandle, *ahvps_COCKPIT, "");
        }
        else
        {
            _RtiAmb.updateAttributeValues(_COCKPIT_ObjectHandle, *ahvps_COCKPIT, UpdateTime, "");
        }
    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
	//std::cout << "CockpitFederateHla13::sendUpdateR: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void CockpitFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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
    
	if (theObject == _MEAS_AIRCRAFT_POSITION_ObjectHandle)
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
				_Cockpit.SET_AIRCRAFT_POSITION_LONGITUDE(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_LATITUDE_ATTRIBUTE)  
            {  
				_Cockpit.SET_AIRCRAFT_POSITION_LATITUDE(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_ALTITUDE_ATTRIBUTE)  
            { 
				_Cockpit.SET_AIRCRAFT_POSITION_ALTITUDE(buffer.read_double()/0.3048); // why 0.3048?? kind of magic number ...
 				CockpitFederateThreadHla13::window->set_PFD_altitude(_Cockpit.GET_AIRCRAFT_POSITION_ALTITUDE()); 
			}
            else
            { 
				cout << "CockpitFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Cockpit.SET_AIRCRAFT_ORIENTATION_PHI(buffer.read_double()*180/M_PI); 
 				CockpitFederateThreadHla13::window->set_PFD_roll(_Cockpit.GET_AIRCRAFT_ORIENTATION_PHI()); 
			}
            else if (parmHandle == _MEAS_THETA_ATTRIBUTE) 
            { 
				_Cockpit.SET_AIRCRAFT_ORIENTATION_THETA(buffer.read_double()*180/M_PI); 
 				CockpitFederateThreadHla13::window->set_PFD_pitch(_Cockpit.GET_AIRCRAFT_ORIENTATION_THETA());
			}
            else if (parmHandle == _MEAS_PSI_ATTRIBUTE)   
            { 
				_Cockpit.SET_AIRCRAFT_ORIENTATION_PSI(buffer.read_double()*180/M_PI); 
				_Cockpit.SET_COCKPIT_AUTOPILOT_HDG(CockpitFederateThreadHla13::window->Get_Autopilot_hdg());
 				CockpitFederateThreadHla13::window->set_PFD_heading(_Cockpit.GET_AIRCRAFT_ORIENTATION_PSI()); 
 				CockpitFederateThreadHla13::window->set_ND_heading(_Cockpit.GET_AIRCRAFT_ORIENTATION_PSI()); 
 				CockpitFederateThreadHla13::window->set_ND_headingBug(_Cockpit.GET_COCKPIT_AUTOPILOT_HDG());
			}
            else
            { 
				cout << "CockpitFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
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
				// Not used
			}
            else if (parmHandle == _MEAS_V_SPEED_ATTRIBUTE) 
            { 
				// Not used 
			}
            else if (parmHandle == _MEAS_W_SPEED_ATTRIBUTE) 
            { 
				// Not used
			}
            else
            { 
				cout << "CockpitFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
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
				// Not used 
			}
            else if (parmHandle == _MEAS_Q_ANG_SPEED_ATTRIBUTE) 
            { 
				// Not used 
			}
            else if (parmHandle == _MEAS_R_ANG_SPEED_ATTRIBUTE) 
            { 
				// Not used 
			}
            else
            { 
				cout << "CockpitFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
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
				// Not used  
			}
            else if (parmHandle == _MEAS_Y_ACC_ATTRIBUTE) 
            { 
				// Not used 
			}
            else if (parmHandle == _MEAS_Z_ACC_ATTRIBUTE) 
            { 
				// Not used 
			}
            else
            { 
				cout << "CockpitFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Cockpit.SET_AIRCRAFT_SPEED_IAS(buffer.read_double()*1.94); // why 1.94?? kind of magic number ...
 				CockpitFederateThreadHla13::window->set_PFD_airspeed(_Cockpit.GET_AIRCRAFT_SPEED_IAS()); 
			}
            else if (parmHandle == _MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE) 
            { 
				// Not used  
			}
            else if (parmHandle == _MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE) 
            { 
				// Not used  
			}
            else if (parmHandle == _MEAS_TRUE_AIRSPEED_ATTRIBUTE)       
            { 
				// Not used  
			}
            else if (parmHandle == _MEAS_GROUND_SPEED_ATTRIBUTE)        
            { 
				// Not used  
			}
            else if (parmHandle == _MEAS_VERTICAL_SPEED_ATTRIBUTE)      
            { 
				// Not used 
			}
            else if (parmHandle == _MEAS_MACH_NUMBER_ATTRIBUTE)        
            { 
				_Cockpit.SET_AIRCRAFT_SPEED_MACH(buffer.read_double()); 
 				CockpitFederateThreadHla13::window->set_ECAM_mach(_Cockpit.GET_AIRCRAFT_SPEED_MACH());
 				CockpitFederateThreadHla13::window->set_PFD_mach(_Cockpit.GET_AIRCRAFT_SPEED_MACH());
			}
            else
            { 
				cout << "CockpitFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Cockpit.SET_AIRCRAFT_ADDITIONAL_ALPHA(buffer.read_double());
				// Conversion from Radians to Degrees ==> *180/M_PI 
				CockpitFederateThreadHla13::window->set_PFD_FlightPath( _Cockpit.GET_AIRCRAFT_ADDITIONAL_ALPHA()*180/M_PI
				                                                 , _Cockpit.GET_AIRCRAFT_ADDITIONAL_BETA()*180/M_PI
				                                                 );
				// Not used  
			}
            else if (parmHandle == _MEAS_BETA_ATTRIBUTE)             
            { 
				_Cockpit.SET_AIRCRAFT_ADDITIONAL_BETA(buffer.read_double()); 
				// Conversion from Radians to Degrees ==> *180/M_PI
				CockpitFederateThreadHla13::window->set_PFD_FlightPath( _Cockpit.GET_AIRCRAFT_ADDITIONAL_ALPHA()*180/M_PI
				                                                 , _Cockpit.GET_AIRCRAFT_ADDITIONAL_BETA()*180/M_PI
				                                                 );
				// Not used 
			}
            else if (parmHandle == _MEAS_DYNAMIC_PRESSURE_ATTRIBUTE) 
            { 
				// Not used 
			}
            else if (parmHandle == _MEAS_TANK_FILLING_ATTRIBUTE)     
            { 
				// Not used  
			}
            else
            { 
				cout << "CockpitFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_ADDITIONAL = true;
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
				_Cockpit.SET_ACTUATORS_RIGHT_ENGINE_THRUST(buffer.read_double()); 
 				CockpitFederateThreadHla13::window->set_ECAM_right_engine_thr(_Cockpit.GET_ACTUATORS_RIGHT_ENGINE_THRUST()); 
			}
            else if (parmHandle == _LEFT_ENGINE_THRUST_ATTRIBUTE)  
            { 
				_Cockpit.SET_ACTUATORS_LEFT_ENGINE_THRUST(buffer.read_double()); 
 				CockpitFederateThreadHla13::window->set_ECAM_left_engine_thr(_Cockpit.GET_ACTUATORS_LEFT_ENGINE_THRUST());  
			}
            else
            { 
				cout << "CockpitFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
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
            if      (parmHandle == _PRESSURE_ATTRIBUTE)
            { 
				_Cockpit.SET_ENVIRONMENT_VARIABLES_PRESSURE(buffer.read_double()); 
 				CockpitFederateThreadHla13::window->set_ECAM_pressure(_Cockpit.GET_ENVIRONMENT_VARIABLES_PRESSURE()); 
			}
			else
            { 
				cout << "CockpitFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
            
        }    
        _New_ENVIRONMENT_VARIABLES = true;
    }
}


// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void CockpitFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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
	#ifdef DEBUG_COCKPIT_FED
	std::cout << "reflectAttributeValues received with ts " <<  ((RTIfedTime&)theTime).getTime() << std::endl;
	#endif 
}


// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void CockpitFederateHla13::timeRegulationEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeRegulationWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeReg = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void CockpitFederateHla13::timeConstrainedEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeConstrainedWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeConst = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeAdvanceGrant
void CockpitFederateHla13::timeAdvanceGrant(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::TimeAdvanceWasNotInProgress,
            RTI::FederateInternalError) 
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_COCKPIT_FED
	std::cout << " >> TAG RECU == LocalTime = " <<  _LocalTime << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void CockpitFederateHla13::enableTimeRegulation()
{
	_RtiAmb.enableTimeRegulation(_LocalTime, _Lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void CockpitFederateHla13::disableTimeRegulation()
{
	_RtiAmb.disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void CockpitFederateHla13::enableTimeConstrained()
{
	_RtiAmb.enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void CockpitFederateHla13::disableTimeConstrained()
{
	_RtiAmb.disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void CockpitFederateHla13::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb.enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void CockpitFederateHla13::disableAsynchronousDelivery()
{
	_RtiAmb.disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void CockpitFederateHla13::timeAdvanceRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_COCKPIT_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.timeAdvanceRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_COCKPIT_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void CockpitFederateHla13::timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime)
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
void CockpitFederateHla13::nextEventRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_COCKPIT_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.nextEventRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_COCKPIT_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// function : nextEventAvailable
void CockpitFederateHla13::nextEventAvailable(RTIfedTime NextLogicalTime)
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
void CockpitFederateHla13::synchronizationPointRegistrationSucceeded(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegSuccess = true ;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void CockpitFederateHla13::synchronizationPointRegistrationFailed(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegFailed = true ;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void CockpitFederateHla13::announceSynchronizationPoint(const char *label, const char *tag)
    throw ( RTI::FederateInternalError) 
{
       _InPause = true ;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void CockpitFederateHla13::federationSynchronized(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _InPause = false ;
} 

// ----------------------------------------------------------------------------
// function : run
void CockpitFederateHla13::run()
{
	while (_LocalTime < _SimulationEndTime)
	{
		// Model calculations
		calculateState();
		sendUpdateAttributes(_LocalTime + _Lookahead);
		timeAdvanceRequest(_TimeStep);
	}
} 

// ----------------------------------------------------------------------------
// function : run
void CockpitFederateHla13::runOneStep()
{
	// Model calculations
	sendUpdateAttributes(_LocalTime + _Lookahead);
	timeAdvanceRequest(_TimeStep);
} 

bool CockpitFederateHla13::getEndOfSimulation()
{
	return (_LocalTime < _SimulationEndTime);
}
