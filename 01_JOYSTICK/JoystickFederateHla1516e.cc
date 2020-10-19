#include "JoystickFederateHla1516e.hh"

// ----------------------------------------------------------------------------
// TemplateFederateHla13 Constructor
JoystickFederateHla1516e::JoystickFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickFederateHla1516e.cc -> Constructor(): Start" << std::endl;
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
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = _IsSyncAnnonced = false ;
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
	std::wcout << L"JoystickFederateHla1516e.cc -> Constructor(): End" << std::endl;
	#endif
	
	std::string testTag ("test tag");
	_MyTag.setData (testTag.c_str (), testTag.size () + 1);
	
	std::string testSyncTag ("");
	_MySyncTag.setData (testSyncTag.c_str (), testSyncTag.size () + 1);
	
	///
	/// 1. create the RTIambassador
	///
  std::wcout << L"=>create RTI Ambassador" << std::endl;
  
  bool test = true;  
  try
  {
	std::unique_ptr < rti1516e::RTIambassadorFactory > rtiAmbFact (new rti1516e::RTIambassadorFactory ());
    std::unique_ptr < rti1516e::RTIambassador > rtiAmbP (rtiAmbFact->createRTIambassador ());
    _RtiAmb = rtiAmbP.release ();
    std::wcout << L"* Ambassador created" << std::endl; 
  }
   
  catch (rti1516e::Exception & e)
  {
    test = false;
   std:: wcout << L"\t->createAmbassador" << std::endl;
    std::wcout << L"* Error creating ambassador" << e.what() << std::endl;  
  }

  if (test) {
      
      /* HLA Evolved requires to 'connect' to the RTI before using RTIAmb */
      try {
           _RtiAmb->connect((* this), rti1516e::HLA_EVOKED);
           std::wcout << L"* Ambassador connected" << std::endl;     
      } 
      catch (rti1516e::Exception& e) {
           std::wcout << L"RTIambassador connect caught Error " << e.what() <<std::endl;
      }
  }
}

// ----------------------------------------------------------------------------
// TemplateFederateHla13 Destructor
JoystickFederateHla1516e::~JoystickFederateHla1516e()
{
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void JoystickFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
	#endif
    try 
    {
        _RtiAmb->createFederationExecution( _FederationName
										 , _FomFileName
										 );
    } 
    catch ( rti1516e::FederationExecutionAlreadyExists ) 
    {
		std::wcout << L"CFE: Federation \"" << getString(_FederationName).c_str() << "\" already created by another federate." << std::endl;
	} 
	catch ( rti1516e::Exception &e ) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	} 
	catch ( ... ) 
	{
		std::cerr << "Error: unknown non-RTI exception." << std::endl;
	}
    #ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void JoystickFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
	if (_IsCreator)
	{
		try 
		{
			_RtiAmb->destroyFederationExecution(_FederationName);
		}
		catch (rti1516e::Exception& e) 
		{
			std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
		}
	}
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void JoystickFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
                                                         , L"joystick"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"JoystickFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void JoystickFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle JoystickFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void JoystickFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void JoystickFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring JOYSTICK (L"JOYSTICK");
		std::wstring AILERON (L"AILERON");
		std::wstring ELEVATOR (L"ELEVATOR");
		std::wstring RUDDER (L"RUDDER");
		std::wstring THROTTLE_LEFT (L"THROTTLE_LEFT");
		std::wstring THROTTLE_RIGHT (L"THROTTLE_RIGHT");
		std::wstring FLAPS (L"FLAPS");
		std::wstring SPOILERS (L"SPOILERS");
		std::wstring GEARS (L"GEARS");
		std::wstring BRAKES (L"BRAKES");
		
		// Published
		_JOYSTICK_ClassHandle = _RtiAmb->getObjectClassHandle(JOYSTICK);
        _AILERON_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, AILERON);
        _ELEVATOR_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, ELEVATOR);
        _RUDDER_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, RUDDER);
        _THROTTLE_LEFT_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, THROTTLE_LEFT);
        _THROTTLE_RIGHT_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, THROTTLE_RIGHT);
        _FLAPS_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, FLAPS);
        _SPOILERS_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, SPOILERS);
        _GEARS_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, GEARS);
        _BRAKES_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, BRAKES);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void JoystickFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes published
		attr_JOYSTICK.insert(_AILERON_ATTRIBUTE);
		attr_JOYSTICK.insert(_ELEVATOR_ATTRIBUTE);
		attr_JOYSTICK.insert(_RUDDER_ATTRIBUTE);
		attr_JOYSTICK.insert(_THROTTLE_LEFT_ATTRIBUTE);
		attr_JOYSTICK.insert(_THROTTLE_RIGHT_ATTRIBUTE);
		attr_JOYSTICK.insert(_FLAPS_ATTRIBUTE);
		attr_JOYSTICK.insert(_SPOILERS_ATTRIBUTE);
		attr_JOYSTICK.insert(_GEARS_ATTRIBUTE);
		attr_JOYSTICK.insert(_BRAKES_ATTRIBUTE);
		
        _RtiAmb->publishObjectClassAttributes(_JOYSTICK_ClassHandle,attr_JOYSTICK);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void JoystickFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring Joystick (L"Joystick");
        _JOYSTICK_ObjectHandle = _RtiAmb->registerObjectInstance(_JOYSTICK_ClassHandle,Joystick);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void JoystickFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb->unpublishObjectClass(_JOYSTICK_ClassHandle);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void JoystickFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void JoystickFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    #ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void JoystickFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    #ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void JoystickFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
    #endif
	std::wstring TrimString (L"Trimming");
    if (_IsCreator) 
    {
		std::wcout << L">> PRESS ENTER TO START TRIMMING " << std::endl;      
        std::cin.get();
		// Simulating synchronization starts with an action of the user (on Creator interface)
        std::wcout << L"Pause requested per Creator " << std::endl;
        try 
        {
            _RtiAmb->registerFederationSynchronizationPoint(TrimString, _MySyncTag);
        }
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::wcout  << "Error: unknown non-RTI exception." << std::endl;
		}
		while (_SyncRegSuccess && !_SyncRegFailed) 
		{
			try 
			{
				_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
			} 
			catch ( rti1516e::Exception &e ) 
			{
				std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
			} 
			catch ( ... ) 
			{
				std::wcout  << "Error: unknown non-RTI exception." << std::endl;
			}
			std::wcout << L">> Waiting for success or failure of synchronisation point init. " << std::endl;
		} 
		if(_SyncRegFailed)
		{
			std::wcout << L">> Error on initial Synchronization" << std::endl;
		} 
    }
	while (!_IsSyncAnnonced) 
	{
		std::wcout << L">> Waiting for synchronisation point Init announcement." << std::endl;
		try 
		{
			_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		} 
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::wcout  << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	try 
	{
		_RtiAmb->synchronizationPointAchieved(TrimString);
	} 
	catch ( rti1516e::Exception &e ) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	} 
	catch ( ... ) 
	{
		std::wcout  << "Error: unknown non-RTI exception." << std::endl;
	}
	std::wcout << L">> Init Synchronisation point satisfied." << std::endl;     

	while (_InPause) 
	{
		std::wcout << L">> Waiting for initialization phase." << std::endl ;
		try 
		{
			_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		} 
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::wcout  << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void JoystickFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
    #endif
	std::wstring SimString (L"Simulating");
    if (_IsCreator) 
    {
		std::wcout << L">> PRESS ENTER TO START SIMULATING " << std::endl;      
        std::cin.get();
		// Simulating synchronization starts with an action of the user (on Creator interface)
        std::wcout << L"Pause requested per Creator " << std::endl;
        try 
        {
            _RtiAmb->registerFederationSynchronizationPoint(SimString, _MySyncTag);
        }
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::wcout  << "Error: unknown non-RTI exception." << std::endl;
		}
		while (_SyncRegSuccess && !_SyncRegFailed) 
		{
			try 
			{
				_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
			} 
			catch ( rti1516e::Exception &e ) 
			{
				std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
			} 
			catch ( ... ) 
			{
				std::wcout  << "Error: unknown non-RTI exception." << std::endl;
			}
			std::wcout << L">> Waiting for success or failure of synchronisation point init. " << std::endl;
		} 
		if(_SyncRegFailed)
		{
			std::wcout << L">> Error on initial Synchronization" << std::endl;
		} 
    }
	while (!_InPause) 
	{
		std::wcout << L">> Waiting for synchronisation point Init announcement." << std::endl;
		try 
		{
			_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		} 
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::wcout  << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	try 
	{
		_RtiAmb->synchronizationPointAchieved(SimString);
	} 
	catch ( rti1516e::Exception &e ) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	} 
	catch ( ... ) 
	{
		std::wcout  << "Error: unknown non-RTI exception." << std::endl;
	}
	std::wcout << L">> Init Synchronisation point satisfied." << std::endl;     

	while (_InPause) 
	{
		std::wcout << L">> Waiting for initialization phase." << std::endl ;
		try 
		{
			_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		} 
		catch ( rti1516e::Exception &e ) 
		{
			std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
		} 
		catch ( ... ) 
		{
			std::wcout  << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void JoystickFederateHla1516e::initialization() 
{
	#ifdef DEBUG_JOYSTICK_FED
    std::wcout << L"JoystickFederateHla1516e.cc -> Initialization(): Start" << std::endl;
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
	
	//waitForAllObjectDiscovered();
    calculateOutput();
    sendInitialAttributes();
	
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L"Configuration Joystick:  " << _Elevator << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Aileron_port :  " << _Aileron_port << std::endl;
    std::wcout << L"_Aileron_jnb  :  " << _Aileron_jnb << std::endl;
    std::wcout << L"_Aileron_sens :  " << _Aileron_sens << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Elevator_port :  " << _Elevator_port << std::endl;
    std::wcout << L"_Elevator_jnb  :  " << _Elevator_jnb << std::endl;
    std::wcout << L"_Elevator_sens :  " << _Elevator_sens << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Rudder_port :  " << _Rudder_port << std::endl;
    std::wcout << L"_Rudder_jnb  :  " << _Rudder_jnb << std::endl;
    std::wcout << L"_Rudder_sens :  " << _Rudder_sens << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Throttle_Left_port :  " << _Throttle_Left_port << std::endl;
    std::wcout << L"_Throttle_Left_jnb  :  " << _Throttle_Left_jnb << std::endl;
    std::wcout << L"_Throttle_Left_sens :  " << _Throttle_Left_sens << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Throttle_Right_port :  " << _Throttle_Right_port << std::endl;
    std::wcout << L"_Throttle_Right_jnb  :  " << _Throttle_Right_jnb << std::endl;
    std::wcout << L"_Throttle_Right_sens :  " << _Throttle_Right_sens << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Flaps_Up_port :  " << _Flaps_Up_port << std::endl;
    std::wcout << L"_Flaps_Up_jnb  :  " << _Flaps_Up_jnb << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"_Flaps_Down_port :  " << _Flaps_Down_port << std::endl;
    std::wcout << L"_Flaps_Down_jnb  :  " << _Flaps_Down_jnb << std::endl;
    std::wcout << L"" << std::endl;   
    std::wcout << L"_Spoilers_port :  " << _Spoilers_port << std::endl;
    std::wcout << L"_Spoilers_jnb  :  " << _Spoilers_jnb << std::endl;
    std::wcout << L"" << std::endl;     
    std::wcout << L"_Gears_port :  " << _Gears_port << std::endl;
    std::wcout << L"_Gears_jnb  :  " << _Gears_jnb << std::endl;
    std::wcout << L"" << std::endl;    
    std::wcout << L"_Brakes_port :  " << _Brakes_port << std::endl;
    std::wcout << L"_Brakes_jnb  :  " << _Brakes_jnb << std::endl;
    std::wcout << L"" << std::endl;
    std::wcout << L"JoystickFederateHla1516e.cc -> Initialization(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void JoystickFederateHla1516e::calculateState() 
{
	// No implementation here
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void JoystickFederateHla1516e::calculateOutput() 
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
	std::wcout << L"Joystick (Elevator) :  " << _Elevator << std::endl;
	std::wcout << L"Joystick (Aileron) :   " << _Aileron << std::endl;
	std::wcout << L"Joystick (Rudder) :    " << _Rudder << std::endl;
	std::wcout << L"Joystick (Throttle Left) :  " << _Throttle_Left << std::endl;
	std::wcout << L"Joystick (Throttle Right) :  " << _Throttle_Right << std::endl;
	std::wcout << L"Joystick (Flaps) :     " << _Flaps << std::endl;
	std::wcout << L"Joystick (Spoilers) :  " << _Spoilers << std::endl;
	std::wcout << L"Joystick (Gears) :     " << _Gears << std::endl;
	std::wcout << L"Joystick (Brakes) :    " << _Brakes << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void JoystickFederateHla1516e::sendInitialAttributes()
{   
	rti1516e::AttributeHandleValueMap ahvps_JOYSTICK;
	
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aileron);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AILERON_ATTRIBUTE,attrValue1)); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Elevator);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_ELEVATOR_ATTRIBUTE,attrValue2)); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Rudder);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_RUDDER_ATTRIBUTE,attrValue3));

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Throttle_Left);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_THROTTLE_LEFT_ATTRIBUTE,attrValue4));

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Throttle_Right);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue5 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_THROTTLE_RIGHT_ATTRIBUTE,attrValue5));

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Flaps);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue6 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_FLAPS_ATTRIBUTE,attrValue6));

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Spoilers);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue7 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_SPOILERS_ATTRIBUTE,attrValue7));

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Gears);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue8 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_GEARS_ATTRIBUTE,attrValue8));

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Brakes);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue9 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_BRAKES_ATTRIBUTE,attrValue9));

    try 
    {
        _RtiAmb->updateAttributeValues(_JOYSTICK_ObjectHandle, ahvps_JOYSTICK, _MyTag);
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void JoystickFederateHla1516e::sendUpdateAttributes(RTI1516fedTime UpdateTime)
{   
	rti1516e::AttributeHandleValueMap ahvps_JOYSTICK;
	
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aileron);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_AILERON_ATTRIBUTE,attrValue1)); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Elevator);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_ELEVATOR_ATTRIBUTE,attrValue2)); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Rudder);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_RUDDER_ATTRIBUTE,attrValue3));

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Throttle_Left);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_THROTTLE_LEFT_ATTRIBUTE,attrValue4));

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Throttle_Right);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue5 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_THROTTLE_RIGHT_ATTRIBUTE,attrValue5));

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Flaps);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue6 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_FLAPS_ATTRIBUTE,attrValue6));

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Spoilers);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue7 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_SPOILERS_ATTRIBUTE,attrValue7));

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Gears);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue8 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_GEARS_ATTRIBUTE,attrValue8));

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Brakes);
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue9 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_JOYSTICK.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_BRAKES_ATTRIBUTE,attrValue9));

    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb->updateAttributeValues(_JOYSTICK_ObjectHandle, ahvps_JOYSTICK, _MyTag);
        }
        else
        {
            _RtiAmb->updateAttributeValues(_JOYSTICK_ObjectHandle, ahvps_JOYSTICK, _MyTag , UpdateTime);
        }
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void JoystickFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
														rti1516e::AttributeHandleValueMap const &theAttributes,
														rti1516e::VariableLengthData const &theUserSuppliedTag,
														rti1516e::OrderType sentOrdering,
														rti1516e::TransportationType theTransport,
														rti1516e::LogicalTime const &theTime,
														rti1516e::OrderType receivedOrdering,
														rti1516e::MessageRetractionHandle theHandle,
														rti1516e::SupplementalReflectInfo theReflectInfo
														)
											 throw ( rti1516e::FederateInternalError) 
{
	// Joystick federate is not receiving from any federate for now
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void JoystickFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
														rti1516e::AttributeHandleValueMap const &theAttributes,
														rti1516e::VariableLengthData const &theUserSuppliedTag,
														rti1516e::OrderType sentOrdering,
														rti1516e::TransportationType theTransport,
														rti1516e::SupplementalReflectInfo theReflectInfo
														)
                                             throw ( rti1516e::FederateInternalError) 
{
	// Joystick federate is not receiving from any federate for now
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void JoystickFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::NoRequestToEnableTimeRegulationWasPending,
            rti1516e::FederateInternalError) */
{
	_IsTimeReg = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void JoystickFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::NoRequestToEnableTimeConstrainedWasPending,
            rti1516e::FederateInternalError) */
{
	_IsTimeConst = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeAdvanceGrant
void JoystickFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_JOYSTICK_FED
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void JoystickFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void JoystickFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void JoystickFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void JoystickFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void JoystickFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void JoystickFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void JoystickFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
{
	RTI1516fedTime newTime = (_LocalTime.getFedTime() + NextLogicalTime.getFedTime());
	_RtiAmb->timeAdvanceRequest(newTime);
	while (!_IsTimeAdvanceGrant) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		sched_yield();
	}

	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
void JoystickFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
{
	RTI1516fedTime newTime = (_LocalTime.getFedTime() + NextLogicalTime.getFedTime());
	_RtiAmb->timeAdvanceRequestAvailable(newTime);
	while (!_IsTimeAdvanceGrant) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		sched_yield();
	}

	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
void JoystickFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
{
	RTI1516fedTime newTime = (_LocalTime.getFedTime() + NextLogicalTime.getFedTime());
	_RtiAmb->nextMessageRequest(newTime);
	while (!_IsTimeAdvanceGrant) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		sched_yield();
	}

	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
// function : nextEventAvailable
void JoystickFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
{
	RTI1516fedTime newTime = (_LocalTime.getFedTime() + NextLogicalTime.getFedTime());
	_RtiAmb->nextMessageRequestAvailable(newTime);
	while (!_IsTimeAdvanceGrant) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
		sched_yield();
	}

	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationSucceeded
void JoystickFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void JoystickFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void JoystickFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void JoystickFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// function : run
void JoystickFederateHla1516e::run()
{
	RTI1516fedTime updateTime(0.0);
	while (_LocalTime < _SimulationEndTime)
	{
		if (_IsRtTimerEnabled)
		{
			clock_gettime(CLOCK_MONOTONIC, &_TimeStamp_old);
		}
		// Model calculations
		calculateOutput();
		updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
		sendUpdateAttributes(updateTime);
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
