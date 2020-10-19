#include "ControlSurfacesFederateHla1516e.hh"

// ----------------------------------------------------------------------------
// TemplateFederateHla13 Constructor
ControlSurfacesFederateHla1516e::ControlSurfacesFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::wcout << L"ControlSurfacesFederateHla1516e.cc -> Constructor(): Start" << std::endl;
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
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = _IsSyncAnnonced= false ;
	_IsOutMesTimespamped = true;

    // Get values from xml file
    XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);
    vector<string> attribute_path(3);
    attribute_path[0] = "CONTROL_SURFACES";

    attribute_path[1] = "simulation_loop";
    attribute_path[2] = "Delta_t_Init";
    mDt = getDoubleValue(params, attribute_path);
    attribute_path[2] = "Iteration_Steps";
    mIterations = getIntValue(params, attribute_path);
    attribute_path[2] = "Integration_Method";
    mIntegrationMethod= getIntValue(params, attribute_path);
    attribute_path[2] = "Saturation";
    mSaturation       = getIntValue(params, attribute_path);

    // Debug
    #ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc Configuration of the Control surfaces" << std::endl;
    cout<<"mDt : "<<mDt<<endl;
    cout<<"mIterations    : "<<mIterations<<endl;
    cout<<"mIntegrationMethod: "<<mIntegrationMethod<<endl;
    cout<<"mSaturation: "<<mSaturation<<endl;
    #endif
    delete params;

    _Right_Aileron.initialize(  ra_damping,ra_frequency,ra_minpos,ra_maxpos,ra_maxrate,mDt,mIterations,mIntegrationMethod);
    _Left_Aileron.initialize(   la_damping,la_frequency,la_minpos,la_maxpos,la_maxrate,mDt,mIterations,mIntegrationMethod);
    _Right_Elevator.initialize( el_damping,el_frequency,el_minpos,el_maxpos,el_maxrate,mDt,mIterations,mIntegrationMethod);
    _Left_Elevator.initialize(  el_damping,el_frequency,el_minpos,el_maxpos,el_maxrate,mDt,mIterations,mIntegrationMethod);
    _Rudder.initialize(         ru_damping,ru_frequency,ru_minpos,ru_maxpos,ru_maxrate,mDt,mIterations,mIntegrationMethod);
    _Flaps.initialize(          fl_damping,fl_frequency,fl_minpos,fl_maxpos,fl_maxrate,mDt,mIterations,mIntegrationMethod);
    _Spoilers.initialize(       sp_damping,sp_frequency,sp_minpos,sp_maxpos,sp_maxrate,mDt,mIterations,mIntegrationMethod);
    _Stabilizer.initialize(     st_damping,st_frequency,st_minpos,st_maxpos,st_maxrate,mDt,mIterations,mIntegrationMethod);
    _Gears.initialize(          ge_damping,ge_frequency,ge_minpos,ge_maxpos,ge_maxrate,mDt,mIterations,mIntegrationMethod);

	_Discov_FLIGHT_CONTROLS           	= false;
	_Discov_TRIM_DATA_FLIGHT_DYNAMICS 	= false;
	_New_FLIGHT_CONTROLS           		= false;
	_New_TRIM_DATA_FLIGHT_DYNAMICS 		= false;

	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::wcout << L"ControlSurfacesFederateHla1516e.cc -> Constructor(): End" << std::endl;
	#endif
	
	std::string testTag ("test tag");
	_MyTag.setData (testTag.c_str (), testTag.size () + 1);
	
	std::string testSyncTag ("");
	_MySyncTag.setData (testSyncTag.c_str (), testSyncTag.size () + 1);
	
	std::string testTag2 ("Trim_Data_Engines");
	_TrimTag.setData (testTag2.c_str (), testTag2.size () + 1);
	
	std::string testTag3 ("Actuators");
	_ActuatorTag.setData (testTag2.c_str (), testTag2.size () + 1);
	
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
ControlSurfacesFederateHla1516e::~ControlSurfacesFederateHla1516e()
{
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void ControlSurfacesFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::wcout << L"ControlSurfacesFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_CONTROL_SURFACES_FED
	std::wcout << L"ControlSurfacesFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void ControlSurfacesFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::wcout << L"ControlSurfacesFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::wcout << L"ControlSurfacesFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::wcout << L"ControlSurfacesFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
														 , L"controlSurfaces"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::wcout << L"ControlSurfacesFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle ControlSurfacesFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void ControlSurfacesFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void ControlSurfacesFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring TRIM_DATA (L"TRIM_DATA");
		std::wstring RIGHT_ENGINE_THROTTLE_EQ (L"RIGHT_ENGINE_THROTTLE_EQ");
		std::wstring LEFT_ENGINE_THROTTLE_EQ (L"LEFT_ENGINE_THROTTLE_EQ");
		std::wstring ELEVATOR_DEFLECTION_EQ (L"ELEVATOR_DEFLECTION_EQ");
		std::wstring STABILIZER_DEFLECTION_EQ (L"STABILIZER_DEFLECTION_EQ");
		std::wstring FLIGHT_CONTROLS (L"FLIGHT_CONTROLS");
		std::wstring RIGHT_AILERON_COMMANDED_DEFLECTION (L"RIGHT_AILERON_COMMANDED_DEFLECTION");
		std::wstring LEFT_AILERON_COMMANDED_DEFLECTION (L"LEFT_AILERON_COMMANDED_DEFLECTION");
		std::wstring RIGHT_ELEVATOR_COMMANDED_DEFLECTION (L"RIGHT_ELEVATOR_COMMANDED_DEFLECTION");
		std::wstring LEFT_ELEVATOR_COMMANDED_DEFLECTION (L"LEFT_ELEVATOR_COMMANDED_DEFLECTION");
		std::wstring RUDDER_COMMANDED_DEFLECTION (L"RUDDER_COMMANDED_DEFLECTION");
		std::wstring FLAPS_COMMANDED_DEFLECTION (L"FLAPS_COMMANDED_DEFLECTION");
		std::wstring SPOILERS_COMMANDED_DEFLECTION (L"SPOILERS_COMMANDED_DEFLECTION");
		std::wstring STABILIZER_COMMANDED_DEFLECTION (L"STABILIZER_COMMANDED_DEFLECTION");
		std::wstring GEARS_COMMANDED_POSITION (L"GEARS_COMMANDED_POSITION");
		std::wstring ACTUATORS (L"ACTUATORS");
		std::wstring RIGHT_AILERON_EFFECTIVE_DEFLECTION (L"RIGHT_AILERON_EFFECTIVE_DEFLECTION");
		std::wstring LEFT_AILERON_EFFECTIVE_DEFLECTION (L"LEFT_AILERON_EFFECTIVE_DEFLECTION");
		std::wstring RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION (L"RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION");
		std::wstring LEFT_ELEVATOR_EFFECTIVE_DEFLECTION (L"LEFT_ELEVATOR_EFFECTIVE_DEFLECTION");
		std::wstring RUDDER_EFFECTIVE_DEFLECTION (L"RUDDER_EFFECTIVE_DEFLECTION");
		std::wstring FLAPS_EFFECTIVE_DEFLECTION (L"FLAPS_EFFECTIVE_DEFLECTION");
		std::wstring SPOILERS_EFFECTIVE_DEFLECTION (L"SPOILERS_EFFECTIVE_DEFLECTION");
		std::wstring STABILIZER_EFFECTIVE_DEFLECTION (L"STABILIZER_EFFECTIVE_DEFLECTION");
		std::wstring GEARS_POSITION (L"GEARS_POSITION");
		
		// Subscribed
		_TRIM_DATA_ClassHandle = _RtiAmb->getObjectClassHandle(TRIM_DATA);     
        _ELEVATOR_DEFLECTION_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, ELEVATOR_DEFLECTION_EQ);
        _STABILIZER_DEFLECTION_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, STABILIZER_DEFLECTION_EQ);    
        
		_FLIGHT_CONTROLS_ClassHandle = _RtiAmb->getObjectClassHandle(FLIGHT_CONTROLS); 
        _RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, RIGHT_AILERON_COMMANDED_DEFLECTION);
        _LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, LEFT_AILERON_COMMANDED_DEFLECTION);
        _RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, RIGHT_ELEVATOR_COMMANDED_DEFLECTION);
        _LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, LEFT_ELEVATOR_COMMANDED_DEFLECTION);
        _RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, RUDDER_COMMANDED_DEFLECTION);
        _FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, FLAPS_COMMANDED_DEFLECTION);
        _SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, SPOILERS_COMMANDED_DEFLECTION);
        _STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, STABILIZER_COMMANDED_DEFLECTION);
        _GEARS_COMMANDED_POSITION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, GEARS_COMMANDED_POSITION); 
        
        // Published 
        _ACTUATORS_ClassHandle = _RtiAmb->getObjectClassHandle(ACTUATORS);
        _RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, RIGHT_AILERON_EFFECTIVE_DEFLECTION);
        _LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, LEFT_AILERON_EFFECTIVE_DEFLECTION);
        _RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION);
        _LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, LEFT_ELEVATOR_EFFECTIVE_DEFLECTION);
        _RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, RUDDER_EFFECTIVE_DEFLECTION);  
        _FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, FLAPS_EFFECTIVE_DEFLECTION);
        _SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, SPOILERS_EFFECTIVE_DEFLECTION);
        _STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, STABILIZER_EFFECTIVE_DEFLECTION);
        _GEARS_POSITION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, GEARS_POSITION);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void ControlSurfacesFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		//attr_TRIM_DATA.reset(RTI::AttributeHandleSetFactory::create(4));
		attr_TRIM_DATA.insert(_ELEVATOR_DEFLECTION_EQ_ATTRIBUTE);
		attr_TRIM_DATA.insert(_STABILIZER_DEFLECTION_EQ_ATTRIBUTE);
		
		attr_FLIGHT_CONTROLS.insert(_RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_GEARS_COMMANDED_POSITION_ATTRIBUTE);
		
		_RtiAmb->subscribeObjectClassAttributes( _TRIM_DATA_ClassHandle,attr_TRIM_DATA); 
        _RtiAmb->subscribeObjectClassAttributes( _FLIGHT_CONTROLS_ClassHandle,attr_FLIGHT_CONTROLS);

		// For Class/Attributes published
		//attr_FLIGHT_CONTROLS.reset(RTI::AttributeHandleSetFactory::create(11)) ;   
		attr_ACTUATORS.insert(_RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_GEARS_POSITION_ATTRIBUTE);
		
        _RtiAmb->publishObjectClassAttributes(_ACTUATORS_ClassHandle,attr_ACTUATORS);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void ControlSurfacesFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring Actuators_Control_Surfaces (L"Actuators_Control_Surfaces");
        _ACTUATORS_ObjectHandle = _RtiAmb->registerObjectInstance(_ACTUATORS_ClassHandle,Actuators_Control_Surfaces);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void ControlSurfacesFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb->unsubscribeObjectClass(_TRIM_DATA_ClassHandle);  
        _RtiAmb->unsubscribeObjectClass(_FLIGHT_CONTROLS_ClassHandle);  
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
        _RtiAmb->unpublishObjectClass(_ACTUATORS_ClassHandle);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void ControlSurfacesFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_Discov_FLIGHT_CONTROLS            ||
            !_Discov_TRIM_DATA_FLIGHT_DYNAMICS)
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
		
	_Discov_FLIGHT_CONTROLS = false;
    _Discov_TRIM_DATA_FLIGHT_DYNAMICS = false;
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void ControlSurfacesFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while  (!_New_TRIM_DATA_FLIGHT_DYNAMICS)
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
    #ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void ControlSurfacesFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    std::wstring Flight_Controls (L"Flight_Controls");
    std::wstring Trim_Data_Flight_Dynamics (L"Trim_Data_Flight_Dynamics");
    
    
    if ( (theObjectClass == _FLIGHT_CONTROLS_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Flight_Controls.c_str())) ) 
    {
       _Discov_FLIGHT_CONTROLS = true;
        _FLIGHT_CONTROLS_ObjectHandle = theObject;
        #ifdef DEBUG_CONTROL_SURFACES_FED
        std::wcout << L"< Flight_Controls > Object Instance has been discovered" << std::endl;
        #endif
    }
    else if ( (theObjectClass == _TRIM_DATA_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Trim_Data_Flight_Dynamics.c_str())) ) 
    {
        _Discov_TRIM_DATA_FLIGHT_DYNAMICS = true;
        _ObjInstance_TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle = theObject;
        #ifdef DEBUG_CONTROL_SURFACES_FED
        std::wcout << L"< Trim_Data_Flight_Dynamics > Object Instance has been discovered with handle "
             << _ObjInstance_TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle
             << "."
             << std::endl;
        #endif
    }
    #ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> discoverObjectInstance(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void ControlSurfacesFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void ControlSurfacesFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
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
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::wcout << L"ControlSurfacesFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void ControlSurfacesFederateHla1516e::initialization() 
{
	waitForAllObjectDiscovered();
	waitForAllAttributesReceived();
	_Right_Elevator.setDeltaActualRad(_Right_Elevator.getDeltaActualRad());
	_Left_Elevator.setDeltaActualRad(_Left_Elevator.getDeltaActualRad());
	_Stabilizer.setDeltaActualRad(_Stabilizer.getDeltaActualRad());
	_Right_Elevator.setInitialStateRad(_Right_Elevator.getDeltaActualRad());
	_Left_Elevator.setInitialStateRad(_Left_Elevator.getDeltaActualRad());
	_Stabilizer.setInitialStateRad(_Stabilizer.getDeltaActualRad());
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void ControlSurfacesFederateHla1516e::calculateState() 
{
	// Model State calculation
	// Choose L or NL integration method (if saturation is used)
    if(mSaturation)
    {
        _Right_Aileron.calculate_stateNL();
        _Left_Aileron.calculate_stateNL();
        _Right_Elevator.calculate_stateNL();
        _Left_Elevator.calculate_stateNL();
        _Rudder.calculate_stateNL();
        _Flaps.calculate_stateNL();
        _Spoilers.calculate_stateNL();
        _Stabilizer.calculate_stateNL();
        _Gears.calculate_stateNL();
    }
    else
    {
        _Right_Aileron.calculate_state();
        _Left_Aileron.calculate_state();
        _Right_Elevator.calculate_state();
        _Left_Elevator.calculate_state();
        _Rudder.calculate_state();
        _Flaps.calculate_state();
        _Spoilers.calculate_state();
        _Stabilizer.calculate_state();
        _Gears.calculate_state();
    }
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void ControlSurfacesFederateHla1516e::calculateOutput() 
{
		_Right_Aileron.calculate_output();
	_Left_Aileron.calculate_output();  
	_Right_Elevator.calculate_output();
	_Left_Elevator.calculate_output();
	_Rudder.calculate_output(); 
	_Flaps.calculate_output();
	_Spoilers.calculate_output();
	_Stabilizer.calculate_output();
	_Gears.calculate_output();

	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::wcout << L" Actual and Required Control Surface position (degrees) : "																		  << std::endl;
	std::wcout << L" RIGHT AILERON  = " << _Right_Aileron.getDeltaActualDeg() 	<< " ( Demand = "  << _Right_Aileron.getDeltaDemandDeg() << " ) " << std::endl;
	std::wcout << L" LEFT AILERON   = " << _Left_Aileron.getDeltaActualDeg() 	<< " ( Demand = "  << _Left_Aileron.getDeltaDemandDeg()  << " ) " << std::endl;
	std::wcout << L" RIGHT ELEVATOR = " << _Right_Elevator.getDeltaActualDeg() << " ( Demand = "  << _Right_Elevator.getDeltaDemandDeg()<< " ) " << std::endl;
	std::wcout << L" LEFT ELEVATOR  = " << _Left_Elevator.getDeltaActualDeg() 	<< " ( Demand = "  << _Left_Elevator.getDeltaDemandDeg() << " ) " << std::endl;
	std::wcout << L" RUDDER         = " << _Rudder.getDeltaActualDeg()			<< " ( Demand = "  << _Rudder.getDeltaDemandDeg() 		 << " ) " << std::endl;
	std::wcout << L" FLAPS          = " << _Flaps.getDeltaActualDeg()			<< " ( Demand = "  << _Flaps.getDeltaDemandDeg() 		 << " ) " << std::endl;
	std::wcout << L" SPOILERS       = " << _Spoilers.getDeltaActualDeg()       << " ( Demand = "  << _Spoilers.getDeltaDemandDeg() 	 << " ) " << std::endl;
	std::wcout << L" STABILIZER     = " << _Stabilizer.getDeltaActualDeg()     << " ( Demand = "  << _Stabilizer.getDeltaDemandDeg() 	 << " ) " << std::endl;
	std::wcout << L" GEARS          = " << _Gears.getDeltaActualDeg()			<< " ( Demand = "  << _Gears.getDeltaDemandDeg() 		 << " ) " << std::endl;
	std::wcout << L" " << endl ;
	#endif
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void ControlSurfacesFederateHla1516e::sendUpdateAttributes(RTI1516fedTime UpdateTime)
{   
	rti1516e::AttributeHandleValueMap ahvps_ACTUATORS;
	
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Right_Aileron.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ACTUATORS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE,attrValue1));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Left_Aileron.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ACTUATORS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE,attrValue2));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Right_Elevator.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ACTUATORS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE,attrValue3));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Left_Elevator.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ACTUATORS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE,attrValue4));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Rudder.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue5 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ACTUATORS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE,attrValue5));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Flaps.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue6 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ACTUATORS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE,attrValue6));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Spoilers.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue7 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ACTUATORS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE,attrValue7));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Stabilizer.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue8 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ACTUATORS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE,attrValue8));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Gears.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue9 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_ACTUATORS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_GEARS_POSITION_ATTRIBUTE,attrValue9));

    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb->updateAttributeValues(_ACTUATORS_ObjectHandle, ahvps_ACTUATORS, _MyTag);
        }
        else
        {
            _RtiAmb->updateAttributeValues(_ACTUATORS_ObjectHandle, ahvps_ACTUATORS, _MyTag , UpdateTime);
        }
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void ControlSurfacesFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
void ControlSurfacesFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
			
            if      (parmHandle == _RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE)  
            { 
				_Right_Aileron.setDeltaDemandRad(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE)   
            { 
				_Left_Aileron.setDeltaDemandRad(buffer.read_double()); 
			}
            else if (parmHandle == _RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE) 
            { 
				_Right_Elevator.setDeltaDemandRad(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE)  
            { 
				_Left_Elevator.setDeltaDemandRad(buffer.read_double()); 
			}
            else if (parmHandle == _RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE)         
            { 
				_Rudder.setDeltaDemandRad(buffer.read_double()); 
			}
            else if (parmHandle == _FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE)          
            { 
				_Flaps.setDeltaDemandRad(buffer.read_double()); 
			}
            else if (parmHandle == _SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE)       
            { 
				_Spoilers.setDeltaDemandRad(buffer.read_double()); 
			}
            else if (parmHandle == _STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE)     
            { 
				_Stabilizer.setDeltaDemandRad(buffer.read_double()); 
			}
            else if (parmHandle == _GEARS_COMMANDED_POSITION_ATTRIBUTE)            
            { 
				_Gears.setDeltaDemandRad(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"ControlSurfacesFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_FLIGHT_CONTROLS = true;
    }  
    else if (theObject == _ObjInstance_TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle)
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
            if      (parmHandle == _ELEVATOR_DEFLECTION_EQ_ATTRIBUTE) 
            { 
				_Right_Elevator.setDeltaActualRad(buffer.read_double());
                _Left_Elevator.setDeltaActualRad(_Right_Elevator.getDeltaActualRad());
			}
            else if (parmHandle == _STABILIZER_DEFLECTION_EQ_ATTRIBUTE)  
            { 
				_Stabilizer.setDeltaActualRad(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"ControlSurfacesFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_TRIM_DATA_FLIGHT_DYNAMICS= true;
    } 
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void ControlSurfacesFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
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
void ControlSurfacesFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
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
void ControlSurfacesFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
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
void ControlSurfacesFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
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
void ControlSurfacesFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
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
void ControlSurfacesFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
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
void ControlSurfacesFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void ControlSurfacesFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void ControlSurfacesFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void ControlSurfacesFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// function : run
void ControlSurfacesFederateHla1516e::run()
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
