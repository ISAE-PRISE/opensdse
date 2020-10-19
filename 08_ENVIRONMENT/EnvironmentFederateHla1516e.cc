#include "EnvironmentFederateHla1516e.hh"

// ----------------------------------------------------------------------------
// TemplateFederateHla13 Constructor
EnvironmentFederateHla1516e::EnvironmentFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_ENVIRONMENT_FED
	std::wcout << L"EnvironmentFederateHla1516e.cc -> Constructor(): Start" << std::endl;
	#endif
	_FederationName = FederationName;
	_FederateName = FederateName;
	_FomFileName = FomFileName;
	_TimeStep = 0.0;
	_Lookahead = 0.0;
	_RestOfTimeStep = 0.0;
	_LocalTime = 0.0;
	_SimulationEndTime = 0000.0;
	_IsTimeReg = _IsTimeConst = _IsTimeAdvanceGrant = false;
	
	// Note: FD is creator for the SDSE federation
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = _IsSyncAnnonced = false;
	_IsOutMesTimespamped = true;
	
    _Discov_AIRCRAFT_POSITION          	= false;
    _Discov_AIRCRAFT_ORIENTATION       	= false;
    _Discov_AIRCRAFT_SPEED            	= false;
    _New_AIRCRAFT_POSITION             	= false;
    _New_AIRCRAFT_ORIENTATION          	= false;
    _New_AIRCRAFT_SPEED                	= false; 

	#ifdef DEBUG_ENVIRONMENT_FED
	std::wcout << L"EnvironmentFederateHla1516e.cc -> Constructor(): End" << std::endl;
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
EnvironmentFederateHla1516e::~EnvironmentFederateHla1516e()
{
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void EnvironmentFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_ENVIRONMENT_FED
	std::wcout << L"EnvironmentFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_ENVIRONMENT_FED
	std::wcout << L"EnvironmentFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void EnvironmentFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_ENVIRONMENT_FED
	std::wcout << L"EnvironmentFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_ENVIRONMENT_FED
	std::wcout << L"EnvironmentFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_ENVIRONMENT_FED
	std::wcout << L"EnvironmentFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
														 , L"environment"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_ENVIRONMENT_FED
	std::wcout << L"EnvironmentFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle EnvironmentFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void EnvironmentFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void EnvironmentFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed 
		std::wstring AIRCRAFT_POSITION (L"AIRCRAFT_POSITION");
		std::wstring AIRCRAFT_ORIENTATION (L"AIRCRAFT_ORIENTATION");
		std::wstring AIRCRAFT_SPEED (L"AIRCRAFT_SPEED");
		std::wstring LONGITUDE (L"LONGITUDE");
		std::wstring LATITUDE (L"LATITUDE");
		std::wstring ALTITUDE (L"ALTITUDE");
		std::wstring PHI (L"PHI");
		std::wstring THETA (L"THETA");
		std::wstring PSI (L"PSI");
		std::wstring INDICATED_AIRSPEED (L"INDICATED_AIRSPEED");
		std::wstring EQUIVALENT_AIRSPEED (L"EQUIVALENT_AIRSPEED");
		std::wstring CALIBRATED_AIRSPEED (L"CALIBRATED_AIRSPEED");
		std::wstring TRUE_AIRSPEED (L"TRUE_AIRSPEED");
		std::wstring GROUND_SPEED (L"GROUND_SPEED");
		std::wstring VERTICAL_SPEED (L"VERTICAL_SPEED");
		std::wstring MACH_NUMBER (L"MACH_NUMBER");
		
		_AIRCRAFT_POSITION_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_POSITION);
        _AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_ORIENTATION);
        _AIRCRAFT_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_SPEED);
        _LONGITUDE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_POSITION_ClassHandle, LONGITUDE);
        _LATITUDE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_POSITION_ClassHandle, LATITUDE);
        _ALTITUDE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_POSITION_ClassHandle, ALTITUDE);  
  		_PHI_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ORIENTATION_ClassHandle, PHI);
        _THETA_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ORIENTATION_ClassHandle, THETA);
        _PSI_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ORIENTATION_ClassHandle, PSI);  
        _INDICATED_AIRSPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, INDICATED_AIRSPEED);
        _EQUIVALENT_AIRSPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, EQUIVALENT_AIRSPEED);
        _CALIBRATED_AIRSPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, CALIBRATED_AIRSPEED);
        _TRUE_AIRSPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, TRUE_AIRSPEED);
        _GROUND_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, GROUND_SPEED);
        _VERTICAL_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, VERTICAL_SPEED);
        _MACH_NUMBER_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, MACH_NUMBER); 
		
		// Published
		std::wstring ENVIRONMENT_VARIABLES (L"ENVIRONMENT_VARIABLES");
		std::wstring TEMPERATURE (L"TEMPERATURE");
		std::wstring DENSITY_OF_AIR (L"DENSITY_OF_AIR");
		std::wstring PRESSURE (L"PRESSURE");
		std::wstring SPEED_OF_SOUND (L"SPEED_OF_SOUND");
		
		std::wstring WIND_COMPONENTS (L"WIND_COMPONENTS");
		std::wstring U_WIND (L"U_WIND");
		std::wstring V_WIND (L"W_WIND");
		std::wstring W_WIND (L"W_WIND");
		std::wstring P_WIND (L"P_WIND");
		std::wstring Q_WIND (L"Q_WIND");
		std::wstring R_WIND (L"R_WIND");
		
        _ENVIRONMENT_VARIABLES_ClassHandle = _RtiAmb->getObjectClassHandle(ENVIRONMENT_VARIABLES);
        _WIND_COMPONENTS_ClassHandle = _RtiAmb->getObjectClassHandle(WIND_COMPONENTS);
		_TEMPERATURE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ENVIRONMENT_VARIABLES_ClassHandle, TEMPERATURE);
		_DENSITY_OF_AIR_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ENVIRONMENT_VARIABLES_ClassHandle, DENSITY_OF_AIR);
		_PRESSURE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ENVIRONMENT_VARIABLES_ClassHandle, PRESSURE);
		_SPEED_OF_SOUND_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ENVIRONMENT_VARIABLES_ClassHandle, SPEED_OF_SOUND); 
		
		_U_WIND_ATTRIBUTE = _RtiAmb->getAttributeHandle(_WIND_COMPONENTS_ClassHandle, U_WIND);
		_V_WIND_ATTRIBUTE = _RtiAmb->getAttributeHandle(_WIND_COMPONENTS_ClassHandle, V_WIND);
		_W_WIND_ATTRIBUTE = _RtiAmb->getAttributeHandle(_WIND_COMPONENTS_ClassHandle, W_WIND);
		_P_WIND_ATTRIBUTE = _RtiAmb->getAttributeHandle(_WIND_COMPONENTS_ClassHandle, P_WIND);   
		_Q_WIND_ATTRIBUTE = _RtiAmb->getAttributeHandle(_WIND_COMPONENTS_ClassHandle, Q_WIND);
		_R_WIND_ATTRIBUTE = _RtiAmb->getAttributeHandle(_WIND_COMPONENTS_ClassHandle, R_WIND);  
		
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void EnvironmentFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		attr_AIRCRAFT_POSITION.insert(_LONGITUDE_ATTRIBUTE);
		attr_AIRCRAFT_POSITION.insert(_LATITUDE_ATTRIBUTE);
		attr_AIRCRAFT_POSITION.insert(_ALTITUDE_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION.insert(_PHI_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION.insert(_THETA_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION.insert(_PSI_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_INDICATED_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_EQUIVALENT_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_CALIBRATED_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_TRUE_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_GROUND_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_VERTICAL_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_MACH_NUMBER_ATTRIBUTE);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_POSITION_ClassHandle,attr_AIRCRAFT_POSITION);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_ORIENTATION_ClassHandle,attr_AIRCRAFT_ORIENTATION);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_SPEED_ClassHandle,attr_AIRCRAFT_SPEED);
        
        // For Class/Attributes published
        attr_ENVIRONMENT_VARIABLES.insert(_TEMPERATURE_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES.insert(_DENSITY_OF_AIR_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES.insert(_PRESSURE_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES.insert(_SPEED_OF_SOUND_ATTRIBUTE);
		attr_WIND_COMPONENTS.insert(_U_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS.insert(_V_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS.insert(_W_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS.insert(_P_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS.insert(_Q_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS.insert(_R_WIND_ATTRIBUTE);         
        _RtiAmb->publishObjectClassAttributes(_ENVIRONMENT_VARIABLES_ClassHandle,attr_ENVIRONMENT_VARIABLES);
        _RtiAmb->publishObjectClassAttributes(_WIND_COMPONENTS_ClassHandle,attr_WIND_COMPONENTS);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnvironmentFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring Environment_Variables (L"Environment_Variables");
		std::wstring Wind_Components (L"Wind_Components");
        _ENVIRONMENT_VARIABLES_ObjectHandle = _RtiAmb->registerObjectInstance(_ENVIRONMENT_VARIABLES_ClassHandle,Environment_Variables);
        _WIND_COMPONENTS_ObjectHandle = _RtiAmb->registerObjectInstance(_WIND_COMPONENTS_ClassHandle,Wind_Components);

    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void EnvironmentFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_SPEED_ClassHandle);
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
        _RtiAmb->unpublishObjectClass(_ENVIRONMENT_VARIABLES_ClassHandle);
        _RtiAmb->unpublishObjectClass(_WIND_COMPONENTS_ClassHandle);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnvironmentFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_Discov_AIRCRAFT_POSITION    ||
            !_Discov_AIRCRAFT_ORIENTATION ||
            !_Discov_AIRCRAFT_SPEED) 
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
	_Discov_AIRCRAFT_POSITION = false;
    _Discov_AIRCRAFT_ORIENTATION = false;
    _Discov_AIRCRAFT_SPEED = false;
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnvironmentFederateHla1516e::waitForAttributesPos()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> waitForAttributesPos(): Start" << std::endl;
    #endif
    while (!_New_AIRCRAFT_POSITION)
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
    _New_AIRCRAFT_POSITION = false;
    #ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> waitForAttributesPos(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnvironmentFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_AIRCRAFT_POSITION    ||
           !_New_AIRCRAFT_ORIENTATION ||
           !_New_AIRCRAFT_SPEED)
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
    _New_AIRCRAFT_POSITION = false;
    _New_AIRCRAFT_ORIENTATION = false;  
    _New_AIRCRAFT_SPEED = false;
    #ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void EnvironmentFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    std::wstring Aircraft_Position (L"Aircraft_Position");
	std::wstring Aircraft_Orientation (L"Aircraft_Orientation");
	std::wstring Aircraft_Speed (L"Aircraft_Speed");
	
   if ( (theObjectClass == _AIRCRAFT_POSITION_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Aircraft_Position.c_str())) ) 
    {
        _Discov_AIRCRAFT_POSITION = true;
        _AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_SENSORS_FED
        std::wcout << L"< Aircraft_Position > Object Instance has been discovered with handle "
             << _AIRCRAFT_POSITION_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_ORIENTATION_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Aircraft_Orientation.c_str())) ) 
    {
        _Discov_AIRCRAFT_ORIENTATION = true;
        _AIRCRAFT_ORIENTATION_ObjectHandle = theObject;
        #ifdef DEBUG_SENSORS_FED
        std::wcout << L"< Aircraft_Orientation > Object Instance has been discovered with handle "
             << _AIRCRAFT_ORIENTATION_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_SPEED_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Aircraft_Speed.c_str())) ) 
    {
        _Discov_AIRCRAFT_SPEED = true;
        _AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_SENSORS_FED
        std::wcout << L"< Aircraft_Speed > Object Instance has been discovered with handle "
             << _AIRCRAFT_SPEED_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    #ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void EnvironmentFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
    #endif
	std::wstring TrimString (L"Trimming");
    if (_IsCreator) 
    {
		std::wcout << L">> PRESS ENTER TO START TRIMMING " << endl;      
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
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void EnvironmentFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
    #endif
	std::wstring SimString (L"Simulating");
    if (_IsCreator) 
    {
		std::wcout << L">> PRESS ENTER TO START SIMULATING " << endl;      
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
	#ifdef DEBUG_ENVIRONMENT_FED
    std::wcout << L"EnvironmentFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void EnvironmentFederateHla1516e::initialization() 
{
	waitForAllObjectDiscovered();
	waitForAttributesPos();
	calculateState();
	sendInitialAttributesEnv();
	waitForAllAttributesReceived();
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void EnvironmentFederateHla1516e::calculateState() 
{
	// Model State calculation
	_Atmosphere.calculate_state();
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void EnvironmentFederateHla1516e::calculateOutput() 
{
	// Model output calculation
	_Atmosphere.calculate_output();
}

// ----------------------------------------------------------------------------
// Send Updates for init attributes position
void EnvironmentFederateHla1516e::sendInitialAttributesEnv()
{
	rti1516e::AttributeHandleValueMap ahvps_ENVIRONMENT_VARIABLES;
	rti1516e::VariableLengthData MyTag;
	std::string testTag ("Environment_Variables");
	MyTag.setData (testTag.c_str (), testTag.size () + 1);
	
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getTemperature());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ENVIRONMENT_VARIABLES.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_TEMPERATURE_ATTRIBUTE,attrValue1)); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getDensity());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ENVIRONMENT_VARIABLES.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_DENSITY_OF_AIR_ATTRIBUTE,attrValue2)); 
	
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getPressure());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ENVIRONMENT_VARIABLES.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_PRESSURE_ATTRIBUTE,attrValue3)); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getSoundSpeed());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ENVIRONMENT_VARIABLES.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_SPEED_OF_SOUND_ATTRIBUTE,attrValue4)); 


    try 
    {
		_RtiAmb->updateAttributeValues(_ENVIRONMENT_VARIABLES_ObjectHandle, ahvps_ENVIRONMENT_VARIABLES, MyTag);


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

// ----------------------------------------------------------------------------
// Send Updates for all attributes Phase 1
void EnvironmentFederateHla1516e::sendUpdateAttributes(RTI1516fedTime UpdateTime)
{ 
	
		// update attribute for initial AIRCRAFT_POSITION 
	//std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_AIRCRAFT_POSITION(RTI::AttributeSetFactory::create(3));
	rti1516e::AttributeHandleValueMap ahvps_ENVIRONMENT_VARIABLES;
	rti1516e::AttributeHandleValueMap  ahvps_WIND_COMPONENTS;
	rti1516e::VariableLengthData MyTag, MyTag2;
	std::string testTag ("Environment_Variables");
	std::string testTag2 ("Wind_Components");
	MyTag.setData (testTag.c_str (), testTag.size () + 1);
	MyTag2.setData (testTag2.c_str (), testTag2.size () + 1);
	
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getTemperature());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ENVIRONMENT_VARIABLES.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_TEMPERATURE_ATTRIBUTE,attrValue1)); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getDensity());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ENVIRONMENT_VARIABLES.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_DENSITY_OF_AIR_ATTRIBUTE,attrValue2)); 
	
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getPressure());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ENVIRONMENT_VARIABLES.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_PRESSURE_ATTRIBUTE,attrValue3)); 

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getSoundSpeed());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ENVIRONMENT_VARIABLES.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_SPEED_OF_SOUND_ATTRIBUTE,attrValue4)); 
	
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Atmosphere.getTotalWindBody(1));
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue5 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_WIND_COMPONENTS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_U_WIND_ATTRIBUTE,attrValue5)); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Atmosphere.getTotalWindBody(2));
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue6 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_WIND_COMPONENTS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_V_WIND_ATTRIBUTE,attrValue6)); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Atmosphere.getTotalWindBody(3));
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue7 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_WIND_COMPONENTS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_W_WIND_ATTRIBUTE,attrValue7)); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Atmosphere.getTotalWindRateBody(1));
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue8 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_WIND_COMPONENTS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_P_WIND_ATTRIBUTE,attrValue8));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Atmosphere.getTotalWindRateBody(2));
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue9 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_WIND_COMPONENTS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_Q_WIND_ATTRIBUTE,attrValue9));
	
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Atmosphere.getTotalWindRateBody(3));
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue10 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_WIND_COMPONENTS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_R_WIND_ATTRIBUTE,attrValue10));
    
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb->updateAttributeValues(_ENVIRONMENT_VARIABLES_ObjectHandle, ahvps_ENVIRONMENT_VARIABLES, MyTag);
            _RtiAmb->updateAttributeValues(_WIND_COMPONENTS_ObjectHandle, ahvps_WIND_COMPONENTS, MyTag2);
        }
        else
        {
            _RtiAmb->updateAttributeValues(_ENVIRONMENT_VARIABLES_ObjectHandle, ahvps_ENVIRONMENT_VARIABLES, MyTag, UpdateTime);
            _RtiAmb->updateAttributeValues(_WIND_COMPONENTS_ObjectHandle, ahvps_WIND_COMPONENTS, MyTag2, UpdateTime);
        }
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

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void EnvironmentFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
void EnvironmentFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
    
    if (theObject == _AIRCRAFT_POSITION_ObjectHandle)
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
            if      (parmHandle == _LONGITUDE_ATTRIBUTE) 
            { 
				// TO DO: it is not used for now, it will be used with 3D environement model
				//buffer.read_double(); 
			}
            else if (parmHandle == _LATITUDE_ATTRIBUTE)  
            { 
				// TO DO: it is not used for now, it will be used with 3D environement model
				buffer.read_double();  
			}
            else if (parmHandle == _ALTITUDE_ATTRIBUTE)  
            { 
				_Atmosphere.setAltitude(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"EnvironmentFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_AIRCRAFT_POSITION = true;
    } 
    else if (theObject == _AIRCRAFT_ORIENTATION_ObjectHandle)
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
            if      (parmHandle == _PHI_ATTRIBUTE)   
            { 
				_Atmosphere.setPhi(buffer.read_double()); 
			}
            else if (parmHandle == _THETA_ATTRIBUTE) 
            {
				_Atmosphere.setTheta(buffer.read_double()); 
			}
            else if (parmHandle == _PSI_ATTRIBUTE)   
            { 
				_Atmosphere.setPsi(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"EnvironmentFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_AIRCRAFT_ORIENTATION = true;
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
            if      (parmHandle == _INDICATED_AIRSPEED_ATTRIBUTE)  
            { 
				// Not used
			}
            else if (parmHandle == _EQUIVALENT_AIRSPEED_ATTRIBUTE) 
            { 
				// Not used
			}
            else if (parmHandle == _CALIBRATED_AIRSPEED_ATTRIBUTE) 
            { 
				// Not used
			}
            else if (parmHandle == _TRUE_AIRSPEED_ATTRIBUTE)       
            { 
				_Atmosphere.setTrueAirSpeed(buffer.read_double()); 
			}
            else if (parmHandle == _GROUND_SPEED_ATTRIBUTE)        
            { 
				// Not used
			}
            else if (parmHandle == _VERTICAL_SPEED_ATTRIBUTE)      
            { 
				// Not used
			}
            else if (parmHandle == _MACH_NUMBER_ATTRIBUTE)        
            { 
				// Not used
			}
            else
            { 
				std::wcout << L"EnvironmentFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_AIRCRAFT_SPEED = true;
    } 
    else
	{ 
		std::wcout << L"SensorsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << std::endl; 
	} 
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void EnvironmentFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
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
void EnvironmentFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
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
void EnvironmentFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_ENVIRONMENT_FED
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void EnvironmentFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
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
void EnvironmentFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
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
void EnvironmentFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
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
void EnvironmentFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
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
void EnvironmentFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void EnvironmentFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void EnvironmentFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void EnvironmentFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// function : run
void EnvironmentFederateHla1516e::run()
{
	RTI1516fedTime updateTime(0.0);
	while (_LocalTime < _SimulationEndTime)
	{
		// Model calculations
		calculateState();
		nextEventRequest(_TimeStep);
		calculateOutput();
		updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
		sendUpdateAttributes(updateTime);
		timeAdvanceRequest(_TimeStep);
	}
} 

