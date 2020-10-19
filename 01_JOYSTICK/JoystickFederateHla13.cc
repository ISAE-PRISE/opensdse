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

#include "JoystickFederateHla13.hh"

// ----------------------------------------------------------------------------
// JoystickFederateHla13 Constructor
JoystickFederateHla13::JoystickFederateHla13( std::wstring FederationName
									, std::wstring FederateName
									, std::wstring FomFileName
									) 
									: NullFederateAmbassador()
									, _RtiAmb() 
{
	#ifdef DEBUG_JOYSTICK_FED
	std::cout << "JoystickFederateHla13.cc -> Constructor(): Start" << std::endl;
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
	_IsRtTimerEnabled = false;
	_IsTimeReg = _IsTimeConst = _IsTimeAdvanceGrant = false ;
	// Note: FD is creator for the SDSE federation
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = false ;
	_IsOutMesTimespamped = true;
	
	// Init Local values to 0.0
	_Aileron = 0.0;
	_Elevator= 0.0; 
	_Rudder= 0.0;
	_Throttle_Left= 0.0; 
	_Throttle_Right= 0.0; 
	_Flaps= 0.0; 
	_Spoilers= 0.0; 
	_Gears= 0.0;
	_Brakes= 0.0;
	
	memset(&_TimeStamp, 0, sizeof(timespec));
	memset(&_TimeStamp_old, 0, sizeof(timespec));
	memset(&_ExecutionTime, 0, sizeof(timespec));

	#ifdef DEBUG_JOYSTICK_FED
	std::cout << "JoystickFederateHla13.cc -> Constructor(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// JoystickFederateHla13 Destructor
JoystickFederateHla13::~JoystickFederateHla13()
	                     throw(RTI::FederateInternalError)
{

}

// ----------------------------------------------------------------------------
// Create a federation 
void JoystickFederateHla13::createFederationExecution() 
{
	#ifdef DEBUG_JOYSTICK_FED
	std::cout << "JoystickFederateHla13.cc -> createFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> createFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Destroy a federation
void JoystickFederateHla13::destroyFederationExecution() 
{
	#ifdef DEBUG_JOYSTICK_FED
	std::cout << "JoystickFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_JOYSTICK_FED
	std::cout << "JoystickFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Join the Feferation federation from Federation Name
void JoystickFederateHla13::joinFederationExecution() 
{
	#ifdef DEBUG_JOYSTICK_FED
	std::cout << "JoystickFederateHla13.cc -> joinFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_JOYSTICK_FED
	std::cout << "JoystickFederateHla13.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
//
void JoystickFederateHla13::resignFederationExecution() 
{
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb.resignFederationExecution(RTI::DELETE_OBJECTS_AND_RELEASE_ATTRIBUTES);
	}
	catch (RTI::Exception& e) 
	{
		std::cout << "RFE: caught " << e._name << " reason " << e._reason <<std::endl;
	}
	#ifdef DEBUG_JOYSTICK_FED
	std::cout << "JoystickFederateHla13.cc -> resignFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
RTI::FederateHandle JoystickFederateHla13::getFederateHandle() const
{
    return _FederateHandle;
}

// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void JoystickFederateHla13::setHlaTimeManagementSettings( RTIfedTime TimeStep
                                                    , RTIfedTime Lookahead
                                                    , RTIfedTime LocalTime
                                                    , RTIfedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void JoystickFederateHla13::getAllHandles()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
        // Published
		_JOYSTICK_ClassHandle = _RtiAmb.getObjectClassHandle("JOYSTICK");
		_AILERON_ATTRIBUTE = _RtiAmb.getAttributeHandle("AILERON",_JOYSTICK_ClassHandle);
		_ELEVATOR_ATTRIBUTE = _RtiAmb.getAttributeHandle("ELEVATOR",_JOYSTICK_ClassHandle); 
		_RUDDER_ATTRIBUTE = _RtiAmb.getAttributeHandle("RUDDER",_JOYSTICK_ClassHandle); 
		_THROTTLE_LEFT_ATTRIBUTE = _RtiAmb.getAttributeHandle("THROTTLE_RIGHT",_JOYSTICK_ClassHandle); 
		_THROTTLE_RIGHT_ATTRIBUTE = _RtiAmb.getAttributeHandle("THROTTLE_LEFT",_JOYSTICK_ClassHandle);
		_FLAPS_ATTRIBUTE = _RtiAmb.getAttributeHandle("FLAPS",_JOYSTICK_ClassHandle);
		_SPOILERS_ATTRIBUTE = _RtiAmb.getAttributeHandle("SPOILERS",_JOYSTICK_ClassHandle);
		_GEARS_ATTRIBUTE = _RtiAmb.getAttributeHandle("GEARS",_JOYSTICK_ClassHandle);
		_BRAKES_ATTRIBUTE  = _RtiAmb.getAttributeHandle("BRAKES",_JOYSTICK_ClassHandle);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void JoystickFederateHla13::publishAndSubscribe()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes published
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
		
        _RtiAmb.publishObjectClass(_JOYSTICK_ClassHandle,*attr_JOYSTICK);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }   
    #ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void JoystickFederateHla13::registerObjectInstances()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
        _JOYSTICK_ObjectHandle = _RtiAmb.registerObjectInstance(_JOYSTICK_ClassHandle,"Joystick");
        ahvps_JOYSTICK.reset(RTI::AttributeSetFactory::create(9));
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void JoystickFederateHla13::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif  
    try 
    {
        _RtiAmb.unpublishObjectClass(_JOYSTICK_ClassHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
    #ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void JoystickFederateHla13::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	// No object to discover
	//_Discov_USER_DATA = false;
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void JoystickFederateHla13::waitForAllAttributesReceived()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    //_New_USER_DATA = false;
    #ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void JoystickFederateHla13::discoverObjectInstance( RTI::ObjectHandle theObject
                                                   , RTI::ObjectClassHandle theObjectClass
                                                   , const char *theObjectName
                                                   )
                                             throw ( RTI::CouldNotDiscover
                                                   , RTI::ObjectClassNotKnown
                                                   , RTI::FederateInternalError
                                                   )
{
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
	// The name of the object Classe 
	// Nothing to discover
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> discoverObjectInstance(): End" << std::endl;
    #endif 
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void JoystickFederateHla13::pauseInitTrim()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync (Simulation go !)
void JoystickFederateHla13::pauseInitSim()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> pause_init_sim(): Start" << std::endl;
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
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> pause_init_sim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Joystick
void JoystickFederateHla13::initialization() 
{
	#ifdef DEBUG_JOYSTICK_FED
    std::cout << "JoystickFederateHla13.cc -> Initialization(): Start" << std::endl;
    #endif
    
	// Parse the init XML file 
	XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);

	vector<string> input_vector(3);
	input_vector[0] = "JOYSTICK";

	input_vector[1] = "input";
	input_vector[2] = "Aileron";
	_Aileron_port   = getIntValue(params, input_vector);
	input_vector[2] = "Elevator";
	_Elevator_port  = getIntValue(params, input_vector); 
	input_vector[2] = "Rudder";
	_Rudder_port    = getIntValue(params, input_vector);
	input_vector[2] = "Throttle_Left";
	_Throttle_Left_port  = getIntValue(params, input_vector);
	input_vector[2] = "Throttle_Right";
	_Throttle_Right_port  = getIntValue(params, input_vector);
	input_vector[2] = "Flaps_Up";
	_Flaps_Up_port  = getIntValue(params, input_vector) + MAX_AXIS;
	input_vector[2] = "Flaps_Down";
	_Flaps_Down_port= getIntValue(params, input_vector) + MAX_AXIS;
	input_vector[2] = "Spoilers";
	_Spoilers_port  = getIntValue(params, input_vector) + MAX_AXIS;
	input_vector[2] = "Gears";
	_Gears_port  = getIntValue(params, input_vector) + MAX_AXIS;
	input_vector[2] = "Brakes";
	_Brakes_port = getIntValue(params, input_vector) + MAX_AXIS;

	input_vector[1] = "port";
	input_vector[2] = "Aileron";
	_Aileron_jnb = getIntValue(params, input_vector);
	input_vector[2] = "Elevator";
	_Elevator_jnb  = getIntValue(params, input_vector);
	input_vector[2] = "Rudder";
	_Rudder_jnb    = getIntValue(params, input_vector);
	input_vector[2] = "Throttle_Left";
	_Throttle_Left_jnb  = getIntValue(params, input_vector);
	input_vector[2] = "Throttle_Right";
	_Throttle_Right_jnb  = getIntValue(params, input_vector);
	input_vector[2] = "Flaps_Up";
	_Flaps_Up_jnb    = getIntValue(params, input_vector);
	input_vector[2] = "Flaps_Down";
	_Flaps_Down_jnb    = getIntValue(params, input_vector);
	input_vector[2] = "Spoilers";
	_Spoilers_jnb    = getIntValue(params, input_vector);
	input_vector[2] = "Gears";
	_Gears_jnb    = getIntValue(params, input_vector);
	input_vector[2] = "Brakes";
	_Brakes_jnb    = getIntValue(params, input_vector);

	input_vector[1] = "sensitivity";
	input_vector[2] = "Aileron";
	_Aileron_sens   = getDoubleValue(params, input_vector);
	input_vector[2] = "Elevator";
	_Elevator_sens  = getDoubleValue(params, input_vector);
	input_vector[2] = "Rudder";
	_Rudder_sens    = getDoubleValue(params, input_vector);
	input_vector[2] = "Throttle_Left";
	_Throttle_Left_sens  = getIntValue(params, input_vector);
	input_vector[2] = "Throttle_Right";
	_Throttle_Right_sens = getIntValue(params, input_vector);
	
	input_vector[1] = "rt_timer";
	input_vector[2] = "Enabled";
	_IsRtTimerEnabled  = static_cast<bool>(getIntValue(params, input_vector));

	delete params;
	
	waitForAllObjectDiscovered();
    calculateOutput();
    sendInitialAttributes();
	
	#ifdef DEBUG_JOYSTICK_FED
	cout << "Configuration Joystick:  " << _Elevator << endl;
    cout << "" << endl;
    cout << "_Aileron_port :  " << _Aileron_port << endl;
    cout << "_Aileron_jnb  :  " << _Aileron_jnb << endl;
    cout << "_Aileron_sens :  " << _Aileron_sens << endl;
    cout << "" << endl;
    cout << "_Elevator_port :  " << _Elevator_port << endl;
    cout << "_Elevator_jnb  :  " << _Elevator_jnb << endl;
    cout << "_Elevator_sens :  " << _Elevator_sens << endl;
    cout << "" << endl;
    cout << "_Rudder_port :  " << _Rudder_port << endl;
    cout << "_Rudder_jnb  :  " << _Rudder_jnb << endl;
    cout << "_Rudder_sens :  " << _Rudder_sens << endl;
    cout << "" << endl;
    cout << "_Throttle_Left_port :  " << _Throttle_Left_port << endl;
    cout << "_Throttle_Left_jnb  :  " << _Throttle_Left_jnb << endl;
    cout << "_Throttle_Left_sens :  " << _Throttle_Left_sens << endl;
    cout << "" << endl;
    cout << "_Throttle_Right_port :  " << _Throttle_Right_port << endl;
    cout << "_Throttle_Right_jnb  :  " << _Throttle_Right_jnb << endl;
    cout << "_Throttle_Right_sens :  " << _Throttle_Right_sens << endl;
    cout << "" << endl;
    cout << "_Flaps_Up_port :  " << _Flaps_Up_port << endl;
    cout << "_Flaps_Up_jnb  :  " << _Flaps_Up_jnb << endl;
    cout << "" << endl;
    cout << "_Flaps_Down_port :  " << _Flaps_Down_port << endl;
    cout << "_Flaps_Down_jnb  :  " << _Flaps_Down_jnb << endl;
    cout << "" << endl;   
    cout << "_Spoilers_port :  " << _Spoilers_port << endl;
    cout << "_Spoilers_jnb  :  " << _Spoilers_jnb << endl;
    cout << "" << endl;     
    cout << "_Gears_port :  " << _Gears_port << endl;
    cout << "_Gears_jnb  :  " << _Gears_jnb << endl;
    cout << "" << endl;    
    cout << "_Brakes_port :  " << _Brakes_port << endl;
    cout << "_Brakes_jnb  :  " << _Brakes_jnb << endl;
    cout << "" << endl;
    std::cout << "JoystickFederateHla13.cc -> Initialization(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Calculate State values for Joystick
void JoystickFederateHla13::calculateState() 
{
	// No implementation here
}
// ----------------------------------------------------------------------------
// Calculate Ouput values for Joystick
void JoystickFederateHla13::calculateOutput() 
{
	double* Inputs;
	Inputs = _Joystick.getInputs();

	// Get axis inputs
	_Aileron          = Inputs[_Aileron_jnb       *MAX_INPUTS + _Aileron_port];
	_Elevator         = Inputs[_Elevator_jnb      *MAX_INPUTS + _Elevator_port];
	_Rudder           = Inputs[_Rudder_jnb        *MAX_INPUTS + _Rudder_port];
	_Throttle_Left    = Inputs[_Throttle_Left_jnb *MAX_INPUTS + _Throttle_Left_port];
	_Throttle_Right   = Inputs[_Throttle_Right_jnb*MAX_INPUTS + _Throttle_Right_port];

	if ( Inputs[_Flaps_Up_jnb  *MAX_INPUTS + _Flaps_Up_port] &&  _Flaps_Up_released)
	{
		// Go to next flaps position
		_Flaps += 0.34; 
		if(_Flaps >1) _Flaps =1;
		_Flaps_Up_released = false;
	}
	else if (!Inputs[_Flaps_Up_jnb  *MAX_INPUTS + _Flaps_Up_port] && !_Flaps_Up_released)
	{
		_Flaps_Up_released = true;
	}
	
	if ( Inputs[_Flaps_Down_jnb*MAX_INPUTS + _Flaps_Down_port] &&  _Flaps_Down_released)
	{
		// Go to previous flaps position
		_Flaps -= 0.34; 
		if(_Flaps <0) _Flaps =0;
		_Flaps_Down_released = false;
	}
	else if (!Inputs[_Flaps_Down_jnb*MAX_INPUTS + _Flaps_Down_port] && !_Flaps_Down_released)
	{
		_Flaps_Down_released = true;
	}
	
	if ( Inputs[_Spoilers_jnb  *MAX_INPUTS + _Spoilers_port] &&  _Spoilers_released)
	{
		// Switch spoilers position
		_Spoilers = 1 - _Spoilers; 
		_Spoilers_released = false;
	}
	else if (!Inputs[_Spoilers_jnb  *MAX_INPUTS + _Spoilers_port] && !_Spoilers_released)
	{
		_Spoilers_released = true;
	}
	
	if ( Inputs[_Gears_jnb  *MAX_INPUTS + _Gears_port] &&  _Gears_released)
	{
		// Switch Gears position
		_Gears = 1 - _Gears; 
		_Gears_released = false;
	}
	else if (!Inputs[_Gears_jnb  *MAX_INPUTS + _Gears_port] && !_Gears_released)
	{
		_Gears_released = true;
	}
	
	if ( Inputs[_Brakes_jnb  *MAX_INPUTS + _Brakes_port] &&  _Brakes_released)
	{
		// Switch Brakes position
		_Brakes = 1 - _Brakes; 
		_Brakes_released = false;
	}
	else if (!Inputs[_Brakes_jnb  *MAX_INPUTS + _Brakes_port] && !_Brakes_released)
	{
		_Brakes_released = true;
	}

	// Adjust sensitivity
	_Aileron        = _Aileron*_Aileron_sens;
	_Elevator       = _Elevator*_Elevator_sens;
	_Rudder         = _Rudder*_Rudder_sens;
	_Throttle_Left  = _Throttle_Left*_Throttle_Left_sens;
	_Throttle_Right = _Throttle_Right*_Throttle_Right_sens;

	// Throttles need to be in [0,1]
	_Throttle_Left  = (1 + _Throttle_Left )/2;
	_Throttle_Right = (1 + _Throttle_Right)/2;

	#ifdef DEBUG_JOYSTICK_FED
	cout << "Joystick (Elevator) :  " << _Elevator << endl;
	cout << "Joystick (Aileron) :   " << _Aileron << endl;
	cout << "Joystick (Rudder) :    " << _Rudder << endl;
	cout << "Joystick (Throttle Left) :  " << _Throttle_Left << endl;
	cout << "Joystick (Throttle Right) :  " << _Throttle_Right << endl;
	cout << "Joystick (Flaps) :     " << _Flaps <<endl;
	cout << "Joystick (Spoilers) :  " << _Spoilers <<endl;
	cout << "Joystick (Gears) :     " << _Gears <<endl;
	cout << "Joystick (Brakes) :    " << _Brakes <<endl;
	#endif
}

// ----------------------------------------------------------------------------
// Send Updates for init attributes
void JoystickFederateHla13::sendInitialAttributes()
{   
	ahvps_JOYSTICK->empty();
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aileron);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_AILERON_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Elevator);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_ELEVATOR_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Rudder);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_RUDDER_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Throttle_Left);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_THROTTLE_LEFT_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Throttle_Right);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_THROTTLE_RIGHT_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Flaps);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_FLAPS_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Spoilers);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_SPOILERS_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Gears);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_GEARS_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Brakes);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_BRAKES_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
    
    try 
    {
		_RtiAmb.updateAttributeValues(_JOYSTICK_ObjectHandle, *ahvps_JOYSTICK, "Joystick");

    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void JoystickFederateHla13::sendUpdateAttributes(const RTI::FedTime& UpdateTime)
{   
	//std::cout << "JoystickFederateHla13::sendUpdateAttributes: Start" << std::endl;
	std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_JOYSTICK(RTI::AttributeSetFactory::create(5));
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aileron);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_AILERON_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Elevator);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_ELEVATOR_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Rudder);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_RUDDER_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Throttle_Left);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_THROTTLE_LEFT_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Throttle_Right);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_THROTTLE_RIGHT_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Flaps);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_FLAPS_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Spoilers);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_SPOILERS_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Gears);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_GEARS_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Brakes);
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_JOYSTICK -> add(_BRAKES_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
    
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb.updateAttributeValues(_JOYSTICK_ObjectHandle, *ahvps_JOYSTICK, "");
        }
        else
        {
            _RtiAmb.updateAttributeValues(_JOYSTICK_ObjectHandle, *ahvps_JOYSTICK, UpdateTime, "");
        }
    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
	//std::cout << "JoystickFederateHla13::sendUpdateR: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void JoystickFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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
   // Joystick federate is not receiving from any federate for now
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void JoystickFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
                                                   , const RTI::AttributeHandleValuePairSet& theAttributes
                                                   , const char *theTag
                                                   )
                                             throw ( RTI::ObjectNotKnown
							                       , RTI::AttributeNotKnown
							                       , RTI::FederateOwnsAttributes
							                       , RTI::FederateInternalError
							                       ) 
{
    // Joystick federate is not receiving from any federate for now
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void JoystickFederateHla13::timeRegulationEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeRegulationWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeReg = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void JoystickFederateHla13::timeConstrainedEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeConstrainedWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeConst = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeAdvanceGrant
void JoystickFederateHla13::timeAdvanceGrant(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::TimeAdvanceWasNotInProgress,
            RTI::FederateInternalError) 
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_JOYSTICK_FED
	std::cout << " >> Joystick Federate: TAG RECU == LocalTime = " <<  _LocalTime << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void JoystickFederateHla13::enableTimeRegulation()
{
	_RtiAmb.enableTimeRegulation(_LocalTime, _Lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void JoystickFederateHla13::disableTimeRegulation()
{
	_RtiAmb.disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void JoystickFederateHla13::enableTimeConstrained()
{
	_RtiAmb.enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void JoystickFederateHla13::disableTimeConstrained()
{
	_RtiAmb.disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void JoystickFederateHla13::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb.enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void JoystickFederateHla13::disableAsynchronousDelivery()
{
	_RtiAmb.disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void JoystickFederateHla13::timeAdvanceRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_JOYSTICK_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.timeAdvanceRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_JOYSTICK_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void JoystickFederateHla13::timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime)
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
void JoystickFederateHla13::nextEventRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_JOYSTICK_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.nextEventRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_JOYSTICK_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// function : nextEventAvailable
void JoystickFederateHla13::nextEventAvailable(RTIfedTime NextLogicalTime)
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
void JoystickFederateHla13::synchronizationPointRegistrationSucceeded(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegSuccess = true ;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void JoystickFederateHla13::synchronizationPointRegistrationFailed(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegFailed = true ;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void JoystickFederateHla13::announceSynchronizationPoint(const char *label, const char *tag)
    throw ( RTI::FederateInternalError) 
{
       _InPause = true ;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void JoystickFederateHla13::federationSynchronized(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _InPause = false ;
} 

// ----------------------------------------------------------------------------
// function : run
void JoystickFederateHla13::run()
{
	while (_LocalTime < _SimulationEndTime)
	{
		if (_IsRtTimerEnabled)
		{
			clock_gettime(CLOCK_MONOTONIC, &_TimeStamp_old);
		}
		// Model calculations
		calculateOutput();
		sendUpdateAttributes(_LocalTime + _Lookahead);
		timeAdvanceRequest(_TimeStep);
		if (_IsRtTimerEnabled)
		{
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
			_TotalRealTimeMs = ((_ExecutionTime.tv_nsec) / 1000000);
			if (_TotalRealTimeMs < 20)
			{
				usleep((20-_TotalRealTimeMs)*1000);
			}
		}
	}
} 
