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

#include "EnginesFederateHla13.hh"

using namespace std;

// ----------------------------------------------------------------------------
// EnginesFederateHla13 Constructor
EnginesFederateHla13::EnginesFederateHla13( std::wstring FederationName
									, std::wstring FederateName
									, std::wstring FomFileName
									) 
									: NullFederateAmbassador()
									, _RtiAmb() 
{
	#ifdef DEBUG_ENGINES_FED
	std::cout << "EnginesFederateHla13.cc -> Constructor(): Start" << std::endl;
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
	
	// TO DO: Is it the right way for initialisation?
    _Right_Engine.setDeltaThrottle(0.5); 
    _Left_Engine.setDeltaThrottle(0.5);  
    _Right_Engine.setMach(0.88); 
    _Left_Engine.setMach(0.88);  
    _Right_Engine.setTemperature(300.0); 
    _Left_Engine.setTemperature(300.0);  
    _Right_Engine.setPressure(101325.0); 
    _Left_Engine.setPressure(101325.0);  
    _Right_Engine.setAltitude(0.0);
    _Left_Engine.setAltitude(0.0);
    
	_Discov_TRIM_DATA_FLIGHT_DYNAMICS = false;
	_Discov_FLIGHT_CONTROLS           = false;
	_Discov_AIRCRAFT_POSITION         = false;
	_Discov_AIRCRAFT_SPEED            = false;
	_Discov_ENVIRONMENT_VARIABLES     = false;
	_New_TRIM_DATA_FLIGHT_DYNAMICS    = false;    
	_New_FLIGHT_CONTROLS              = false;
	_New_AIRCRAFT_POSITION            = false;
	_New_AIRCRAFT_SPEED               = false;
	_New_ENVIRONMENT_VARIABLES        = false;

	#ifdef DEBUG_ENGINES_FED
	std::cout << "EnginesFederateHla13.cc -> Constructor(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// EnginesFederateHla13 Destructor
EnginesFederateHla13::~EnginesFederateHla13()
	                     throw(RTI::FederateInternalError)
{

}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void EnginesFederateHla13::createFederationExecution() 
{
	#ifdef DEBUG_ENGINES_FED
	std::cout << "EnginesFederateHla13.cc -> createFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> createFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void EnginesFederateHla13::destroyFederationExecution() 
{
	#ifdef DEBUG_ENGINES_FED
	std::cout << "EnginesFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_ENGINES_FED
	std::cout << "EnginesFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EnginesFederateHla13::joinFederationExecution() 
{
	#ifdef DEBUG_ENGINES_FED
	std::cout << "EnginesFederateHla13.cc -> joinFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_ENGINES_FED
	std::cout << "EnginesFederateHla13.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EnginesFederateHla13::resignFederationExecution() 
{
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb.resignFederationExecution(RTI::DELETE_OBJECTS_AND_RELEASE_ATTRIBUTES);
	}
	catch (RTI::Exception& e) 
	{
		std::cout << "RFE: caught " << e._name << " reason " << e._reason <<std::endl;
	}
	#ifdef DEBUG_ENGINES_FED
	std::cout << "EnginesFederateHla13.cc -> resignFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
RTI::FederateHandle EnginesFederateHla13::getFederateHandle() const
{
    return _FederateHandle;
}

// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void EnginesFederateHla13::setHlaTimeManagementSettings( RTIfedTime TimeStep
                                                    , RTIfedTime Lookahead
                                                    , RTIfedTime LocalTime
                                                    , RTIfedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void EnginesFederateHla13::getAllHandles()
{
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed
        _FLIGHT_CONTROLS_ClassHandle = _RtiAmb.getObjectClassHandle("FLIGHT_CONTROLS");
        _AIRCRAFT_POSITION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_POSITION");
        _AIRCRAFT_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_SPEED");
        _ENVIRONMENT_VARIABLES_ClassHandle = _RtiAmb.getObjectClassHandle("ENVIRONMENT_VARIABLES"); 
		_RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_COMMANDED_THROTTLE",_FLIGHT_CONTROLS_ClassHandle);
		_LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ENGINE_COMMANDED_THROTTLE", _FLIGHT_CONTROLS_ClassHandle);
		_ALTITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("ALTITUDE",_AIRCRAFT_POSITION_ClassHandle);
		_MACH_NUMBER_ATTRIBUTE = _RtiAmb.getAttributeHandle("MACH_NUMBER",_AIRCRAFT_SPEED_ClassHandle);  
		_TEMPERATURE_ATTRIBUTE = _RtiAmb.getAttributeHandle("TEMPERATURE",  _ENVIRONMENT_VARIABLES_ClassHandle);
		_DENSITY_OF_AIR_ATTRIBUTE = _RtiAmb.getAttributeHandle("DENSITY_OF_AIR",_ENVIRONMENT_VARIABLES_ClassHandle);
		_PRESSURE_ATTRIBUTE = _RtiAmb.getAttributeHandle("PRESSURE",_ENVIRONMENT_VARIABLES_ClassHandle);   
        // Published 
		_ACTUATORS_ClassHandle = _RtiAmb.getObjectClassHandle("ACTUATORS");
		_TRIM_DATA_ClassHandle = _RtiAmb.getObjectClassHandle("TRIM_DATA");
		_RIGHT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_THRUST",_ACTUATORS_ClassHandle);
		_LEFT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ENGINE_THRUST", _ACTUATORS_ClassHandle); 
		_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_THRUST_EQ", _TRIM_DATA_ClassHandle);
        _LEFT_ENGINE_THRUST_EQ_ATTRIBUTE =  _RtiAmb.getAttributeHandle("LEFT_ENGINE_THRUST_EQ", _TRIM_DATA_ClassHandle);
        _RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_THROTTLE_EQ",_TRIM_DATA_ClassHandle);
        _LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ENGINE_THROTTLE_EQ", _TRIM_DATA_ClassHandle);  
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void EnginesFederateHla13::publishAndSubscribe()
{
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		attr_FLIGHT_CONTROLS.reset(RTI::AttributeHandleSetFactory::create(2));
		attr_AIRCRAFT_POSITION.reset(RTI::AttributeHandleSetFactory::create(1));
		attr_AIRCRAFT_SPEED.reset(RTI::AttributeHandleSetFactory::create(1));
		attr_ENVIRONMENT_VARIABLES.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_TRIM_DATA_FLIGHT_DYNAMICS.reset(RTI::AttributeHandleSetFactory::create(2));
		attr_FLIGHT_CONTROLS->add(_RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE);
		attr_AIRCRAFT_POSITION->add(_ALTITUDE_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_MACH_NUMBER_ATTRIBUTE); 
		attr_ENVIRONMENT_VARIABLES->add(_TEMPERATURE_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES->add(_DENSITY_OF_AIR_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES->add(_PRESSURE_ATTRIBUTE);
		attr_TRIM_DATA_FLIGHT_DYNAMICS->add(_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE);
		attr_TRIM_DATA_FLIGHT_DYNAMICS->add(_LEFT_ENGINE_THRUST_EQ_ATTRIBUTE);  
        _RtiAmb.subscribeObjectClassAttributes(_TRIM_DATA_ClassHandle, *attr_TRIM_DATA_FLIGHT_DYNAMICS);
        _RtiAmb.subscribeObjectClassAttributes(_FLIGHT_CONTROLS_ClassHandle, *attr_FLIGHT_CONTROLS);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_POSITION_ClassHandle, *attr_AIRCRAFT_POSITION);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_SPEED_ClassHandle, *attr_AIRCRAFT_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_ENVIRONMENT_VARIABLES_ClassHandle,*attr_ENVIRONMENT_VARIABLES);

		// For Class/Attributes published
		attr_ACTUATORS.reset(RTI::AttributeHandleSetFactory::create(2));  
		attr_TRIM_DATA_ENGINES.reset(RTI::AttributeHandleSetFactory::create(2));  
		attr_ACTUATORS->add(_RIGHT_ENGINE_THRUST_ATTRIBUTE);
		attr_ACTUATORS->add(_LEFT_ENGINE_THRUST_ATTRIBUTE);
		attr_TRIM_DATA_ENGINES->add(_RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE);
		attr_TRIM_DATA_ENGINES->add(_LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE); 
        _RtiAmb.publishObjectClass(_ACTUATORS_ClassHandle,*attr_ACTUATORS);
        _RtiAmb.publishObjectClass(_TRIM_DATA_ClassHandle,*attr_TRIM_DATA_ENGINES);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }   
    #ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnginesFederateHla13::registerObjectInstances()
{
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
        _ACTUATORS_ObjectHandle = _RtiAmb.registerObjectInstance(_ACTUATORS_ClassHandle,"Actuators_Engines");
        _TRIM_DATA_ENGINES_ObjectHandle = _RtiAmb.registerObjectInstance(_TRIM_DATA_ClassHandle,"Trim_Data_Engines");
        ahvps_ACTUATORS.reset(RTI::AttributeSetFactory::create(2));
        ahvps_TRIM_DATA_ENGINES.reset(RTI::AttributeSetFactory::create(2));
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void EnginesFederateHla13::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb.unsubscribeObjectClass(_TRIM_DATA_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_FLIGHT_CONTROLS_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_SPEED_ClassHandle);
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
        _RtiAmb.unpublishObjectClass(_TRIM_DATA_ClassHandle);
        _RtiAmb.unpublishObjectClass(_ACTUATORS_ClassHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
    #ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnginesFederateHla13::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_Discov_TRIM_DATA_FLIGHT_DYNAMICS ||
            !_Discov_FLIGHT_CONTROLS           ||
            !_Discov_AIRCRAFT_POSITION         ||
            !_Discov_AIRCRAFT_SPEED            ||
            !_Discov_ENVIRONMENT_VARIABLES)
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
    _Discov_TRIM_DATA_FLIGHT_DYNAMICS = false;
    _Discov_FLIGHT_CONTROLS = false;
    _Discov_AIRCRAFT_POSITION = false;
    _Discov_AIRCRAFT_SPEED = false;
    _Discov_ENVIRONMENT_VARIABLES = false;
    
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnginesFederateHla13::waitForAllAttributesReceived()
{
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_TRIM_DATA_FLIGHT_DYNAMICS ||
           !_New_AIRCRAFT_POSITION         ||
           !_New_AIRCRAFT_SPEED            ||
           !_New_ENVIRONMENT_VARIABLES) 
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
    _New_TRIM_DATA_FLIGHT_DYNAMICS = false;
    _New_AIRCRAFT_POSITION = false;
    _New_AIRCRAFT_SPEED = false;
    _New_ENVIRONMENT_VARIABLES = false;
    #ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void EnginesFederateHla13::discoverObjectInstance( RTI::ObjectHandle theObject
                                                   , RTI::ObjectClassHandle theObjectClass
                                                   , const char *theObjectName
                                                   )
                                             throw ( RTI::CouldNotDiscover
                                                   , RTI::ObjectClassNotKnown
                                                   , RTI::FederateInternalError
                                                   )
{
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    if ( (theObjectClass == _FLIGHT_CONTROLS_ClassHandle) && (!strcmp(theObjectName,"Flight_Controls")) ) 
    {
        _Discov_FLIGHT_CONTROLS = true;
        _FLIGHT_CONTROLS_ObjectHandle = theObject;
        #ifdef DEBUG_ENGINES_FED
        cout << "< Flight_Controls > Object Instance has been discovered" << endl;
        #endif
    }  
    else if ( (theObjectClass == _AIRCRAFT_POSITION_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Position")) ) 
    {
        _Discov_AIRCRAFT_POSITION = true;
        _AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_ENGINES_FED
        cout << "< Aircraft_Position > Object Instance has been discovered" << endl;
        #endif
    }
    else if ( (theObjectClass == _AIRCRAFT_SPEED_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Speed")) ) 
    {
        _Discov_AIRCRAFT_SPEED = true;
        _AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_ENGINES_FED
        cout << "< Aircraft_Speed > Object Instance has been discovered" << endl;
        #endif
    } 
    else if ( (theObjectClass == _ENVIRONMENT_VARIABLES_ClassHandle) && (!strcmp(theObjectName,"Environment_Variables")) ) 
    {
        _Discov_ENVIRONMENT_VARIABLES = true;
        _ENVIRONMENT_VARIABLES_ObjectHandle = theObject;
        #ifdef DEBUG_ENGINES_FED
        cout << "< Environment_Variables > Object Instance has been discovered" << endl;
        #endif
    }
    else if ( (theObjectClass == _TRIM_DATA_ClassHandle) && (!strcmp(theObjectName,"Trim_Data_Flight_Dynamics")) ) 
    {
        _Discov_TRIM_DATA_FLIGHT_DYNAMICS = true;
        _TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle = theObject;
        #ifdef DEBUG_ENGINES_FED
        cout << "< Trim_Data_Flight_Dynamics > Object Instance has been discovered" << endl;
        #endif
    } 
    #ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void EnginesFederateHla13::pauseInitTrim()
{
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void EnginesFederateHla13::pauseInitSim()
{
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> pause_init_sim(): Start" << std::endl;
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
	#ifdef DEBUG_ENGINES_FED
    std::cout << "EnginesFederateHla13.cc -> pause_init_sim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void EnginesFederateHla13::initialization() 
{
	waitForAllAttributesReceived();
    _Right_Engine.initialization();
    _Left_Engine.initialization();
    sendInitialAttributes();
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void EnginesFederateHla13::calculateState() 
{
	// Model State calculation
    _Right_Engine.calculate_state();
    _Left_Engine.calculate_state();
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void EnginesFederateHla13::calculateOutput() 
{
    _Right_Engine.calculate_output();
    _Left_Engine.calculate_output();
}

// ----------------------------------------------------------------------------
// Send Updates for init attributes
void EnginesFederateHla13::sendInitialAttributes()
{   
	ahvps_TRIM_DATA_ENGINES->empty();
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Right_Engine.getDeltaThrottle());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_TRIM_DATA_ENGINES -> add(_RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
   
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Left_Engine.getDeltaThrottle());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_TRIM_DATA_ENGINES -> add(_LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

    try 
    {
		_RtiAmb.updateAttributeValues(_TRIM_DATA_ENGINES_ObjectHandle, *ahvps_TRIM_DATA_ENGINES, "Trim_Data_Engines");

    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void EnginesFederateHla13::sendUpdateAttributes(const RTI::FedTime& UpdateTime)
{   
	ahvps_ACTUATORS->empty();
    _OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Right_Engine.getThrustActual());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_ACTUATORS -> add(_RIGHT_ENGINE_THRUST_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Left_Engine.getThrustActual());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_ACTUATORS -> add(_LEFT_ENGINE_THRUST_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb.updateAttributeValues(_ACTUATORS_ObjectHandle, *ahvps_ACTUATORS, "Actuators");
        }
        else
        {
            _RtiAmb.updateAttributeValues(_ACTUATORS_ObjectHandle, *ahvps_ACTUATORS, UpdateTime, "Actuators");
        }
    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
	//std::cout << "EnginesFederateHla13::sendUpdateR: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void EnginesFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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

    if (theObject == _FLIGHT_CONTROLS_ObjectHandle){
        for (unsigned int j=0 ; j<theAttributes.size(); j++) {

            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
            buffer.assumeSizeFromReservedBytes();

            if      (parmHandle == _RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE) 
            { 
				_Right_Engine.setDeltaThrottle(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE)  
            { 
				_Left_Engine.setDeltaThrottle(buffer.read_double()); 
			}
            else
            { 
				cout << "EnginesFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_FLIGHT_CONTROLS = true;
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
            double tmp;
            if   (parmHandle == _ALTITUDE_ATTRIBUTE) 
            { 
				tmp =  buffer.read_double();
				_Right_Engine.setAltitude(tmp);
				_Left_Engine.setAltitude(tmp);
			}
            else
            { 
				cout << "EnginesFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_AIRCRAFT_POSITION = true;
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
            double tmp;                 
            if   (parmHandle == _MACH_NUMBER_ATTRIBUTE) 
            { 
				tmp =  buffer.read_double();
                _Right_Engine.setMach(tmp);
                _Left_Engine.setMach(tmp); 
			}
            else
            { 
				cout << "EnginesFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_AIRCRAFT_SPEED = true;
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
            double tmp;
            if      (parmHandle == _TEMPERATURE_ATTRIBUTE) 
            { 
				tmp =  buffer.read_double(); 
                _Right_Engine.setTemperature(tmp);
                _Left_Engine.setTemperature(tmp); 
			}
            else if (parmHandle == _DENSITY_OF_AIR_ATTRIBUTE) 
			{ 
				tmp =  buffer.read_double(); 
			 }
            else if (parmHandle == _PRESSURE_ATTRIBUTE) 
            { 
				tmp =  buffer.read_double(); 
				_Right_Engine.setPressure(tmp);
				_Left_Engine.setPressure(tmp); 
			}
            else
            { 
				cout << "EnginesFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_ENVIRONMENT_VARIABLES = true;
    } 
    else if (theObject == _TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle)
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
            double tmp;
            if      (parmHandle == _RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE) 
            { 
				_Right_Engine.setThrustActual(buffer.read_double()); 
				cout << "_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE OK " << endl;
			}
            else if (parmHandle == _LEFT_ENGINE_THRUST_EQ_ATTRIBUTE)  
            { 
				_Left_Engine.setThrustActual(buffer.read_double()); 
				cout << "_LEFT_ENGINE_THRUST_EQ_ATTRIBUTE OK " << endl;
			}
            else
            { 
				cout << "EnginesFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_TRIM_DATA_FLIGHT_DYNAMICS = true;
    } 
	else
	{ 
		cout << "EnginesFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << endl; 
	} 
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void EnginesFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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
	#ifdef DEBUG_ENGINES_FED
	std::cout << "reflectAttributeValues received with ts " <<  ((RTIfedTime&)theTime).getTime() << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void EnginesFederateHla13::timeRegulationEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeRegulationWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeReg = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void EnginesFederateHla13::timeConstrainedEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeConstrainedWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeConst = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeAdvanceGrant
void EnginesFederateHla13::timeAdvanceGrant(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::TimeAdvanceWasNotInProgress,
            RTI::FederateInternalError) 
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_ENGINES_FED
	std::cout << " >> TAG RECU == LocalTime = " <<  _LocalTime << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void EnginesFederateHla13::enableTimeRegulation()
{
	_RtiAmb.enableTimeRegulation(_LocalTime, _Lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void EnginesFederateHla13::disableTimeRegulation()
{
	_RtiAmb.disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void EnginesFederateHla13::enableTimeConstrained()
{
	_RtiAmb.enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void EnginesFederateHla13::disableTimeConstrained()
{
	_RtiAmb.disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void EnginesFederateHla13::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb.enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void EnginesFederateHla13::disableAsynchronousDelivery()
{
	_RtiAmb.disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void EnginesFederateHla13::timeAdvanceRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_ENGINES_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.timeAdvanceRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_ENGINES_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EnginesFederateHla13::timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime)
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
void EnginesFederateHla13::nextEventRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_ENGINES_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.nextEventRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_ENGINES_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// function : nextEventAvailable
void EnginesFederateHla13::nextEventAvailable(RTIfedTime NextLogicalTime)
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
void EnginesFederateHla13::synchronizationPointRegistrationSucceeded(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegSuccess = true ;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void EnginesFederateHla13::synchronizationPointRegistrationFailed(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegFailed = true ;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void EnginesFederateHla13::announceSynchronizationPoint(const char *label, const char *tag)
    throw ( RTI::FederateInternalError) 
{
       _InPause = true ;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void EnginesFederateHla13::federationSynchronized(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _InPause = false ;
} 

// ----------------------------------------------------------------------------
// function : run
void EnginesFederateHla13::run()
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
