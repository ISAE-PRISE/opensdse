#include "EnginesFederateHla1516e.hh"

// ----------------------------------------------------------------------------
// TemplateFederateHla13 Constructor
EnginesFederateHla1516e::EnginesFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_ENGINES_FED
	std::wcout << L"EnginesFederateHla1516e.cc -> Constructor(): Start" << std::endl;
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
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = _IsSyncAnnonced = false ;
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
	std::wcout << L"EnginesFederateHla1516e.cc -> Constructor(): End" << std::endl;
	#endif
	
	std::string testTag ("test tag");
	_MyTag.setData (testTag.c_str (), testTag.size () + 1);
	
	std::string testTag2 ("Trim_Data_Engines");
	_TrimTag.setData (testTag2.c_str (), testTag2.size () + 1);
	
	std::string testTag3 ("Actuators");
	_ActuatorTag.setData (testTag2.c_str (), testTag2.size () + 1);
	
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
EnginesFederateHla1516e::~EnginesFederateHla1516e()
{
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void EnginesFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_ENGINES_FED
	std::wcout << L"EnginesFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_ENGINES_FED
	std::wcout << L"EnginesFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void EnginesFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_ENGINES_FED
	std::wcout << L"EnginesFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_ENGINES_FED
	std::wcout << L"EnginesFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EnginesFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_ENGINES_FED
	std::wcout << L"EnginesFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
														 , L"engines"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_ENGINES_FED
	std::wcout << L"EnginesFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EnginesFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle EnginesFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void EnginesFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void EnginesFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed
		std::wstring FLIGHT_CONTROLS (L"FLIGHT_CONTROLS");
		std::wstring RIGHT_ENGINE_COMMANDED_THROTTLE (L"RIGHT_ENGINE_COMMANDED_THROTTLE");
		std::wstring LEFT_ENGINE_COMMANDED_THROTTLE (L"LEFT_ENGINE_COMMANDED_THROTTLE");	
		std::wstring AIRCRAFT_POSITION (L"AIRCRAFT_POSITION");
		std::wstring AIRCRAFT_SPEED (L"AIRCRAFT_SPEED");
		std::wstring ALTITUDE (L"ALTITUDE");
		std::wstring MACH_NUMBER (L"MACH_NUMBER");	
		std::wstring ENVIRONMENT_VARIABLES (L"ENVIRONMENT_VARIABLES");
		std::wstring TEMPERATURE (L"TEMPERATURE");
		std::wstring DENSITY_OF_AIR (L"DENSITY_OF_AIR");
		std::wstring PRESSURE (L"PRESSURE");
		
		// published
		std::wstring TRIM_DATA (L"TRIM_DATA");
		std::wstring RIGHT_ENGINE_THROTTLE_EQ (L"RIGHT_ENGINE_THROTTLE_EQ");
		std::wstring LEFT_ENGINE_THROTTLE_EQ (L"LEFT_ENGINE_THROTTLE_EQ");
		// Warning -> This has to be RECEIVED...NOT SENT !!!
		std::wstring RIGHT_ENGINE_THRUST_EQ (L"RIGHT_ENGINE_THRUST_EQ");
		std::wstring LEFT_ENGINE_THRUST_EQ (L"LEFT_ENGINE_THRUST_EQ");
		std::wstring ACTUATORS (L"ACTUATORS");
		std::wstring RIGHT_ENGINE_THRUST (L"RIGHT_ENGINE_THRUST");
		std::wstring LEFT_ENGINE_THRUST (L"LEFT_ENGINE_THRUST");
		
		
		// Subscribed
        _FLIGHT_CONTROLS_ClassHandle = _RtiAmb->getObjectClassHandle(FLIGHT_CONTROLS);
        _AIRCRAFT_POSITION_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_POSITION);
        _AIRCRAFT_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_SPEED);
        _ENVIRONMENT_VARIABLES_ClassHandle = _RtiAmb->getObjectClassHandle(ENVIRONMENT_VARIABLES); 
		_RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, RIGHT_ENGINE_COMMANDED_THROTTLE);
		_LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, LEFT_ENGINE_COMMANDED_THROTTLE);
		_ALTITUDE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_POSITION_ClassHandle, ALTITUDE);
		_MACH_NUMBER_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, MACH_NUMBER);  
		_TEMPERATURE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ENVIRONMENT_VARIABLES_ClassHandle, TEMPERATURE);
		_DENSITY_OF_AIR_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ENVIRONMENT_VARIABLES_ClassHandle, DENSITY_OF_AIR);
		_PRESSURE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ENVIRONMENT_VARIABLES_ClassHandle, PRESSURE);   
        // Published 
		_ACTUATORS_ClassHandle = _RtiAmb->getObjectClassHandle(ACTUATORS);
		_TRIM_DATA_ClassHandle = _RtiAmb->getObjectClassHandle(TRIM_DATA);
		_RIGHT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, RIGHT_ENGINE_THRUST);
		_LEFT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, LEFT_ENGINE_THRUST); 
		_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, RIGHT_ENGINE_THRUST_EQ);
        _LEFT_ENGINE_THRUST_EQ_ATTRIBUTE =  _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, LEFT_ENGINE_THRUST_EQ);
        _RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, RIGHT_ENGINE_THROTTLE_EQ);
        _LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, LEFT_ENGINE_THROTTLE_EQ);  
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void EnginesFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		attr_FLIGHT_CONTROLS.insert(_RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE);
		attr_AIRCRAFT_POSITION.insert(_ALTITUDE_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_MACH_NUMBER_ATTRIBUTE); 
		attr_ENVIRONMENT_VARIABLES.insert(_TEMPERATURE_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES.insert(_DENSITY_OF_AIR_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES.insert(_PRESSURE_ATTRIBUTE);
		attr_TRIM_DATA_FLIGHT_DYNAMICS.insert(_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE);
		attr_TRIM_DATA_FLIGHT_DYNAMICS.insert(_LEFT_ENGINE_THRUST_EQ_ATTRIBUTE);  
        _RtiAmb->subscribeObjectClassAttributes(_TRIM_DATA_ClassHandle, attr_TRIM_DATA_FLIGHT_DYNAMICS);
        _RtiAmb->subscribeObjectClassAttributes(_FLIGHT_CONTROLS_ClassHandle, attr_FLIGHT_CONTROLS);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_POSITION_ClassHandle, attr_AIRCRAFT_POSITION);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_SPEED_ClassHandle, attr_AIRCRAFT_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_ENVIRONMENT_VARIABLES_ClassHandle,attr_ENVIRONMENT_VARIABLES);

		// For Class/Attributes published
		attr_ACTUATORS.insert(_RIGHT_ENGINE_THRUST_ATTRIBUTE);
		attr_ACTUATORS.insert(_LEFT_ENGINE_THRUST_ATTRIBUTE);
		attr_TRIM_DATA_ENGINES.insert(_RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE);
		attr_TRIM_DATA_ENGINES.insert(_LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE); 
        _RtiAmb->publishObjectClassAttributes(_ACTUATORS_ClassHandle,attr_ACTUATORS);
        _RtiAmb->publishObjectClassAttributes(_TRIM_DATA_ClassHandle,attr_TRIM_DATA_ENGINES);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnginesFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring Actuators_Engines (L"Actuators_Engines");
        _ACTUATORS_ObjectHandle = _RtiAmb->registerObjectInstance(_ACTUATORS_ClassHandle,Actuators_Engines);
        std::wstring Trim_Data_Engines (L"Trim_Data_Engines");
        _TRIM_DATA_ENGINES_ObjectHandle = _RtiAmb->registerObjectInstance(_TRIM_DATA_ClassHandle,Trim_Data_Engines);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void EnginesFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb->unsubscribeObjectClass(_TRIM_DATA_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_FLIGHT_CONTROLS_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_ENVIRONMENT_VARIABLES_ClassHandle);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
       
    try 
    {
        _RtiAmb->unpublishObjectClass(_FLIGHT_CONTROLS_ClassHandle);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnginesFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_Discov_TRIM_DATA_FLIGHT_DYNAMICS ||
            !_Discov_FLIGHT_CONTROLS           ||
            !_Discov_AIRCRAFT_POSITION         ||
            !_Discov_AIRCRAFT_SPEED            ||
            !_Discov_ENVIRONMENT_VARIABLES)
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
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	}
		
	_Discov_TRIM_DATA_FLIGHT_DYNAMICS = false;
    _Discov_FLIGHT_CONTROLS = false;
    _Discov_AIRCRAFT_POSITION = false;
    _Discov_AIRCRAFT_SPEED = false;
    _Discov_ENVIRONMENT_VARIABLES = false;
    
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnginesFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_TRIM_DATA_FLIGHT_DYNAMICS ||
           !_New_AIRCRAFT_POSITION         ||
           !_New_AIRCRAFT_SPEED            ||
           !_New_ENVIRONMENT_VARIABLES)  
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
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
    
    _New_TRIM_DATA_FLIGHT_DYNAMICS = false;
    _New_AIRCRAFT_POSITION = false;
    _New_AIRCRAFT_SPEED = false;
    _New_ENVIRONMENT_VARIABLES = false;
    
    #ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void EnginesFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    std::wstring Flight_Controls (L"Flight_Controls");
    std::wstring Aircraft_Position (L"Aircraft_Position");
    std::wstring Aircraft_Speed (L"Aircraft_Speed");
    std::wstring Environment_Variables (L"Environment_Variables");
	std::wstring Trim_Data_Flight_Dynamics (L"Trim_Data_Flight_Dynamics");
    
    if ( (theObjectClass == _FLIGHT_CONTROLS_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Flight_Controls.c_str())) ) 
    {
        _Discov_FLIGHT_CONTROLS = true;
        _FLIGHT_CONTROLS_ObjectHandle = theObject;
        #ifdef DEBUG_ENGINES_FED
        std::wcout << L"< Flight_Controls > Object Instance has been discovered" << std::endl;
        #endif
    }  
    else if ( (theObjectClass == _AIRCRAFT_POSITION_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Aircraft_Position.c_str())) ) 
    {
        _Discov_AIRCRAFT_POSITION = true;
        _AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_ENGINES_FED
        std::wcout << L"< Aircraft_Position > Object Instance has been discovered" << std::endl;
        #endif
    }
    else if ( (theObjectClass == _AIRCRAFT_SPEED_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Aircraft_Speed.c_str())) ) 
    {
        _Discov_AIRCRAFT_SPEED = true;
        _AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_ENGINES_FED
        std::wcout << L"< Aircraft_Speed > Object Instance has been discovered" << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _ENVIRONMENT_VARIABLES_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Environment_Variables.c_str())) ) 
    {
        _Discov_ENVIRONMENT_VARIABLES = true;
        _ENVIRONMENT_VARIABLES_ObjectHandle = theObject;
        #ifdef DEBUG_ENGINES_FED
        std::wcout << L"< Environment_Variables > Object Instance has been discovered" << std::endl;
        #endif
    }
    else if ( (theObjectClass == _TRIM_DATA_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Trim_Data_Flight_Dynamics.c_str())) ) 
    {
        _Discov_TRIM_DATA_FLIGHT_DYNAMICS = true;
        _TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle = theObject;
        #ifdef DEBUG_ENGINES_FED
        std::wcout << L"< Trim_Data_Flight_Dynamics > Object Instance has been discovered" << std::endl;
        #endif
    } 
    #ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void EnginesFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void EnginesFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
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
	#ifdef DEBUG_ENGINES_FED
    std::wcout << L"EnginesFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void EnginesFederateHla1516e::initialization() 
{
	waitForAllAttributesReceived();
    _Right_Engine.initialization();
    _Left_Engine.initialization();
    sendInitialAttributes();
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void EnginesFederateHla1516e::calculateState() 
{
	// Model State calculation
    _Right_Engine.calculate_state();
    _Left_Engine.calculate_state(); 
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void EnginesFederateHla1516e::calculateOutput() 
{
	_Right_Engine.calculate_output();
    _Left_Engine.calculate_output();
}

// ----------------------------------------------------------------------------
// Send Updates for init attributes
void EnginesFederateHla1516e::sendInitialAttributes()
{   
	rti1516e::AttributeHandleValueMap ahvps_TRIM_DATA_ENGINES;
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Right_Engine.getDeltaThrottle());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_TRIM_DATA_ENGINES.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE,attrValue1)); 
	
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Left_Engine.getDeltaThrottle());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_TRIM_DATA_ENGINES.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE,attrValue2)); 

    try 
    {
		_RtiAmb->updateAttributeValues(_TRIM_DATA_ENGINES_ObjectHandle, ahvps_TRIM_DATA_ENGINES, _TrimTag);

    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void EnginesFederateHla1516e::sendUpdateAttributes(RTI1516fedTime UpdateTime)
{  
	 rti1516e::AttributeHandleValueMap ahvps_ACTUATORS;
	 
    _OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Right_Engine.getThrustActual());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ACTUATORS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_RIGHT_ENGINE_THRUST_ATTRIBUTE,attrValue1)); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Left_Engine.getThrustActual());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ACTUATORS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LEFT_ENGINE_THRUST_ATTRIBUTE,attrValue2)); 
    
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb->updateAttributeValues(_ACTUATORS_ObjectHandle, ahvps_ACTUATORS, _ActuatorTag);
        }
        else
        {
            _RtiAmb->updateAttributeValues(_ACTUATORS_ObjectHandle, ahvps_ACTUATORS, _ActuatorTag, UpdateTime);
        }
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void EnginesFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
	// Same function as without Logical Time
	reflectAttributeValues(theObject, theAttributes, theUserSuppliedTag, sentOrdering, theTransport, theReflectInfo);
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void EnginesFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
														rti1516e::AttributeHandleValueMap const &theAttributes,
														rti1516e::VariableLengthData const &theUserSuppliedTag,
														rti1516e::OrderType sentOrdering,
														rti1516e::TransportationType theTransport,
														rti1516e::SupplementalReflectInfo theReflectInfo
														)
                                             throw ( rti1516e::FederateInternalError) 
{
	uint32_t valueLength ;
	rti1516e::AttributeHandle parmHandle ;
	MessageBuffer buffer;
	rti1516e::AttributeHandleValueMap::const_iterator it;
	double tmp_value;
    
    if (theObject == _FLIGHT_CONTROLS_ObjectHandle)
    {
        for (it = theAttributes.begin (); it != theAttributes.end (); ++it) 
		{
			parmHandle = it->first;
			valueLength = it->second.size();
			assert(valueLength>0);
			buffer.resize(valueLength);        
			buffer.reset(); 
			std::memcpy(static_cast<char*>(buffer(0)),(it->second).data (), valueLength);                
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
				std::wcout << L"EnginesFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_FLIGHT_CONTROLS= true;
    }  
    else if (theObject == _AIRCRAFT_POSITION_ObjectHandle)
    {
        for (it = theAttributes.begin (); it != theAttributes.end (); ++it) 
		{
			parmHandle = it->first;
			valueLength = it->second.size();
			assert(valueLength>0);
			buffer.resize(valueLength);        
			buffer.reset(); 
			std::memcpy(static_cast<char*>(buffer(0)),(it->second).data (), valueLength);                
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
				std::wcout << L"EnginesFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_AIRCRAFT_POSITION= true;
    } 
    else if (theObject == _AIRCRAFT_SPEED_ObjectHandle)
    {
        for (it = theAttributes.begin (); it != theAttributes.end (); ++it) 
		{
			parmHandle = it->first;
			valueLength = it->second.size();
			assert(valueLength>0);
			buffer.resize(valueLength);        
			buffer.reset(); 
			std::memcpy(static_cast<char*>(buffer(0)),(it->second).data (), valueLength);                
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
				std::wcout << L"EnginesFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_AIRCRAFT_SPEED = true;
    }   
    else if (theObject == _ENVIRONMENT_VARIABLES_ObjectHandle)
    {
        for (it = theAttributes.begin (); it != theAttributes.end (); ++it) 
		{
			parmHandle = it->first;
			valueLength = it->second.size();
			assert(valueLength>0);
			buffer.resize(valueLength);        
			buffer.reset(); 
			std::memcpy(static_cast<char*>(buffer(0)),(it->second).data (), valueLength);                
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
				std::wcout << L"EnginesFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_ENVIRONMENT_VARIABLES = true;
    }
    else if (theObject == _TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle)
    {
        for (it = theAttributes.begin (); it != theAttributes.end (); ++it) 
		{
			parmHandle = it->first;
			valueLength = it->second.size();
			assert(valueLength>0);
			buffer.resize(valueLength);        
			buffer.reset(); 
			std::memcpy(static_cast<char*>(buffer(0)),(it->second).data (), valueLength);                
			buffer.assumeSizeFromReservedBytes();
            double tmp;
            if      (parmHandle == _RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE) 
            { 
				_Right_Engine.setThrustActual(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_ENGINE_THRUST_EQ_ATTRIBUTE)  
            { 
				_Left_Engine.setThrustActual(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"EnginesFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_TRIM_DATA_FLIGHT_DYNAMICS = true;
    } 
	else
	{ 
		std::wcout << L"EnginesFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << std::endl; 
	} 
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void EnginesFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
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
void EnginesFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
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
void EnginesFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_ENGINES_FED
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void EnginesFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void EnginesFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void EnginesFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void EnginesFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void EnginesFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void EnginesFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void EnginesFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
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
void EnginesFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
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
void EnginesFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
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
void EnginesFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
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
void EnginesFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void EnginesFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void EnginesFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void EnginesFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// function : run
void EnginesFederateHla1516e::run()
{
	RTI1516fedTime updateTime(0.0);
	while (_LocalTime < _SimulationEndTime)
	{
		// Model calculations
		calculateState(); 
		calculateOutput();
		updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
		sendUpdateAttributes(updateTime);
		timeAdvanceRequest(_TimeStep);
	}
} 
