#include "VisualizationFederateHla1516e.hh"

// ----------------------------------------------------------------------------
// TemplateFederateHla13 Constructor
VisualizationFederateHla1516e::VisualizationFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_VISUALIZATION_FED
	std::wcout << L"VisualizationFederateHla1516e.cc -> Constructor(): Start" << std::endl;
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
	
    _Discov_AIRCRAFT_POSITION          = false;
    _Discov_AIRCRAFT_ORIENTATION     	= false;
	_Discov_AIRCRAFT_UVW_SPEED       	= false;
    _Discov_AIRCRAFT_PQR_ANGULAR_SPEED	= false;
	_Discov_AIRCRAFT_ACCELERATION		= false;
    _Discov_AIRCRAFT_SPEED				= false;
	_Discov_AIRCRAFT_ADDITIONAL			= false;
    _Discov_ACTUATORS_CONTROL_SURFACES = false;
    _Discov_ACTUATORS_ENGINES          = false;
    
    _New_AIRCRAFT_POSITION             = false;
    _New_AIRCRAFT_ORIENTATION          = false;
	_New_AIRCRAFT_UVW_SPEED       	   = false;
    _New_AIRCRAFT_PQR_ANGULAR_SPEED	   = false;
	_New_AIRCRAFT_ACCELERATION		   = false;
    _New_AIRCRAFT_SPEED				   = false;
	_New_AIRCRAFT_ADDITIONAL		   = false;
    _New_ACTUATORS_CONTROL_SURFACES    = false;
    _New_ACTUATORS_ENGINES             = false;

	#ifdef DEBUG_VISUALIZATION_FED
	std::wcout << L"VisualizationFederateHla1516e.cc -> Constructor(): End" << std::endl;
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
VisualizationFederateHla1516e::~VisualizationFederateHla1516e()
{
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void VisualizationFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_VISUALIZATION_FED
	std::wcout << L"VisualizationFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_VISUALIZATION_FED
	std::wcout << L"VisualizationFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void VisualizationFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_VISUALIZATION_FED
	std::wcout << L"VisualizationFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_VISUALIZATION_FED
	std::wcout << L"VisualizationFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_VISUALIZATION_FED
	std::wcout << L"VisualizationFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
														 , L"vizu"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_VISUALIZATION_FED
	std::wcout << L"VisualizationFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle VisualizationFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void VisualizationFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void VisualizationFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{

		// Subscribed
		std::wstring AIRCRAFT_POSITION (L"AIRCRAFT_POSITION");
		std::wstring AIRCRAFT_ORIENTATION (L"AIRCRAFT_ORIENTATION");
		std::wstring AIRCRAFT_UVW_SPEED (L"AIRCRAFT_UVW_SPEED");
		std::wstring AIRCRAFT_PQR_ANGULAR_SPEED (L"AIRCRAFT_PQR_ANGULAR_SPEED");
		std::wstring AIRCRAFT_ACCELERATION (L"AIRCRAFT_ACCELERATION");
		std::wstring AIRCRAFT_SPEED (L"AIRCRAFT_SPEED");
		std::wstring AIRCRAFT_ADDITIONAL (L"AIRCRAFT_ADDITIONAL");
		std::wstring LONGITUDE (L"LONGITUDE");
		std::wstring LATITUDE (L"LATITUDE");
		std::wstring ALTITUDE (L"ALTITUDE");
		std::wstring PHI (L"PHI");
		std::wstring THETA (L"THETA");
		std::wstring PSI (L"PSI");
		std::wstring U_SPEED (L"U_SPEED");
		std::wstring V_SPEED (L"V_SPEED");
		std::wstring W_SPEED (L"W_SPEED");
		std::wstring P_ANG_SPEED (L"P_ANG_SPEED");
		std::wstring Q_ANG_SPEED (L"Q_ANG_SPEED");
		std::wstring R_ANG_SPEED (L"R_ANG_SPEED");
		std::wstring X_ACC (L"X_ACC");
		std::wstring Y_ACC (L"Y_ACC");
		std::wstring Z_ACC (L"Z_ACC");
		std::wstring INDICATED_AIRSPEED (L"INDICATED_AIRSPEED");
		std::wstring EQUIVALENT_AIRSPEED (L"EQUIVALENT_AIRSPEED");
		std::wstring CALIBRATED_AIRSPEED (L"CALIBRATED_AIRSPEED");
		std::wstring TRUE_AIRSPEED (L"TRUE_AIRSPEED");
		std::wstring GROUND_SPEED (L"GROUND_SPEED");
		std::wstring VERTICAL_SPEED (L"VERTICAL_SPEED");
		std::wstring MACH_NUMBER (L"MACH_NUMBER");
		std::wstring ALPHA (L"ALPHA");
		std::wstring BETA (L"BETA");
		std::wstring DYNAMIC_PRESSURE (L"DYNAMIC_PRESSURE");
		std::wstring TANK_FILLING (L"TANK_FILLING");
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
		
		std::wstring RIGHT_ENGINE_THRUST (L"RIGHT_ENGINE_THRUST");
		std::wstring LEFT_ENGINE_THRUST (L"LEFT_ENGINE_THRUST");
		_ACTUATORS_ClassHandle = _RtiAmb->getObjectClassHandle(ACTUATORS);
		_AIRCRAFT_POSITION_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_POSITION);
		_AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_ORIENTATION);
		_AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_UVW_SPEED);
		_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_PQR_ANGULAR_SPEED);
		_AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_ACCELERATION);
		_AIRCRAFT_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_SPEED);
		_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_ADDITIONAL);
		_RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, RIGHT_AILERON_EFFECTIVE_DEFLECTION);
		_LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, LEFT_AILERON_EFFECTIVE_DEFLECTION);
		_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION);
		_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, LEFT_ELEVATOR_EFFECTIVE_DEFLECTION);
		_RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, RUDDER_EFFECTIVE_DEFLECTION);            
		_RIGHT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, RIGHT_ENGINE_THRUST);
		_LEFT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, LEFT_ENGINE_THRUST);
		_FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, FLAPS_EFFECTIVE_DEFLECTION);
		_SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, SPOILERS_EFFECTIVE_DEFLECTION);
		_STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, STABILIZER_EFFECTIVE_DEFLECTION);
		_LONGITUDE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_POSITION_ClassHandle, LONGITUDE);
        _LATITUDE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_POSITION_ClassHandle, LATITUDE);
        _ALTITUDE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_POSITION_ClassHandle, ALTITUDE);  
  		_PHI_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ORIENTATION_ClassHandle, PHI);
        _THETA_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ORIENTATION_ClassHandle, THETA);
        _PSI_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ORIENTATION_ClassHandle, PSI);    
        _U_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_UVW_SPEED_ClassHandle, U_SPEED);
        _V_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_UVW_SPEED_ClassHandle, V_SPEED);
        _W_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_UVW_SPEED_ClassHandle, W_SPEED);  
        _P_ANG_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle, P_ANG_SPEED);
        _Q_ANG_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle, Q_ANG_SPEED);
        _R_ANG_SPEED_ATTRIBUTE =_RtiAmb->getAttributeHandle(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle, R_ANG_SPEED); 
        _X_ACC_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ACCELERATION_ClassHandle, X_ACC);
        _Y_ACC_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ACCELERATION_ClassHandle, Y_ACC);
        _Z_ACC_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ACCELERATION_ClassHandle, Z_ACC);
        _INDICATED_AIRSPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, INDICATED_AIRSPEED);
        _EQUIVALENT_AIRSPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, EQUIVALENT_AIRSPEED);
        _CALIBRATED_AIRSPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, CALIBRATED_AIRSPEED);
        _TRUE_AIRSPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, TRUE_AIRSPEED);
        _GROUND_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, GROUND_SPEED);
        _VERTICAL_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, VERTICAL_SPEED);
        _MACH_NUMBER_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_SPEED_ClassHandle, MACH_NUMBER);
        _ALPHA_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ADDITIONAL_ClassHandle, ALPHA);
        _BETA_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ADDITIONAL_ClassHandle, BETA);
        _DYNAMIC_PRESSURE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ADDITIONAL_ClassHandle, DYNAMIC_PRESSURE);
        _TANK_FILLING_ATTRIBUTE = _RtiAmb->getAttributeHandle(_AIRCRAFT_ADDITIONAL_ClassHandle, TANK_FILLING);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void VisualizationFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		attr_ACTUATORS.insert(_RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_RIGHT_ENGINE_THRUST_ATTRIBUTE);
		attr_ACTUATORS.insert(_LEFT_ENGINE_THRUST_ATTRIBUTE);
		attr_ACTUATORS.insert(_FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS.insert(_GEARS_POSITION_ATTRIBUTE);	
		attr_AIRCRAFT_POSITION.insert(_LONGITUDE_ATTRIBUTE);
		attr_AIRCRAFT_POSITION.insert(_LATITUDE_ATTRIBUTE);
		attr_AIRCRAFT_POSITION.insert(_ALTITUDE_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION.insert(_PHI_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION.insert(_THETA_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION.insert(_PSI_ATTRIBUTE);
		attr_AIRCRAFT_UVW_SPEED.insert(_U_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_UVW_SPEED.insert(_V_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_UVW_SPEED.insert(_W_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_PQR_ANGULAR_SPEED.insert(_P_ANG_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_PQR_ANGULAR_SPEED.insert(_Q_ANG_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_PQR_ANGULAR_SPEED.insert(_R_ANG_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_ACCELERATION.insert(_X_ACC_ATTRIBUTE);
		attr_AIRCRAFT_ACCELERATION.insert(_Y_ACC_ATTRIBUTE);
		attr_AIRCRAFT_ACCELERATION.insert(_Z_ACC_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_INDICATED_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_EQUIVALENT_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_CALIBRATED_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_TRUE_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_GROUND_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_VERTICAL_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED.insert(_MACH_NUMBER_ATTRIBUTE);
		attr_AIRCRAFT_ADDITIONAL.insert(_ALPHA_ATTRIBUTE);
		attr_AIRCRAFT_ADDITIONAL.insert(_BETA_ATTRIBUTE);
		attr_AIRCRAFT_ADDITIONAL.insert(_DYNAMIC_PRESSURE_ATTRIBUTE);
		attr_AIRCRAFT_ADDITIONAL.insert(_TANK_FILLING_ATTRIBUTE);
		
		_RtiAmb->subscribeObjectClassAttributes(_ACTUATORS_ClassHandle,attr_ACTUATORS);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_POSITION_ClassHandle,attr_AIRCRAFT_POSITION);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_ORIENTATION_ClassHandle,attr_AIRCRAFT_ORIENTATION);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_UVW_SPEED_ClassHandle,attr_AIRCRAFT_UVW_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,attr_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_ACCELERATION_ClassHandle,attr_AIRCRAFT_ACCELERATION);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_SPEED_ClassHandle,attr_AIRCRAFT_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_ADDITIONAL_ClassHandle,attr_AIRCRAFT_ADDITIONAL);
        
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void VisualizationFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	//Not used
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void VisualizationFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->unsubscribeObjectClass(_ACTUATORS_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_ADDITIONAL_ClassHandle);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
       
    #ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void VisualizationFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_Discov_AIRCRAFT_POSITION          ||
			!_Discov_AIRCRAFT_ORIENTATION       ||
			!_Discov_AIRCRAFT_UVW_SPEED         ||
			!_Discov_AIRCRAFT_PQR_ANGULAR_SPEED ||
			!_Discov_AIRCRAFT_ACCELERATION      ||
			!_Discov_AIRCRAFT_SPEED             ||
			!_Discov_AIRCRAFT_ADDITIONAL) 
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
	_Discov_AIRCRAFT_UVW_SPEED = false;
	_Discov_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_AIRCRAFT_ACCELERATION = false;
	_Discov_AIRCRAFT_SPEED = false;
	_Discov_AIRCRAFT_ADDITIONAL = false;
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void VisualizationFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_AIRCRAFT_POSITION          ||
           !_New_AIRCRAFT_ORIENTATION       ||
           !_New_AIRCRAFT_UVW_SPEED         ||
           !_New_AIRCRAFT_PQR_ANGULAR_SPEED ||
           !_New_AIRCRAFT_ACCELERATION      ||
           !_New_AIRCRAFT_SPEED             ||
           !_New_AIRCRAFT_ADDITIONAL) 
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
    _New_AIRCRAFT_UVW_SPEED = false;
    _New_AIRCRAFT_PQR_ANGULAR_SPEED = false;
    _New_AIRCRAFT_ACCELERATION = false;
    _New_AIRCRAFT_SPEED = false;
    _New_AIRCRAFT_ADDITIONAL = false;
    #ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void VisualizationFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    std::wstring Actuators_Control_Surfaces (L"Actuators_Control_Surfaces");
    std::wstring Actuators_Engines (L"Actuators_Engines");
	std::wstring Aircraft_Position (L"Aircraft_Position");
	std::wstring Aircraft_Orientation (L"Aircraft_Orientation");
    std::wstring Aircraft_UVW_Speed (L"Aircraft_UVW_Speed");
    std::wstring Aircraft_PQR_Angular_Speed (L"Aircraft_PQR_Angular_Speed");
    std::wstring Aircraft_Acceleration (L"Aircraft_Acceleration");
	std::wstring Aircraft_Speed (L"Aircraft_Speed");
    std::wstring Aircraft_Additional (L"Aircraft_Additional");
    
    if ( (theObjectClass == _ACTUATORS_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Actuators_Control_Surfaces.c_str())) ) 
    {
        _Discov_ACTUATORS_CONTROL_SURFACES = true;
        _ACTUATORS_CONTROL_SURFACES_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        std::wcout << L"< Aircraft_Position > Object Instance has been discovered with handle "
             << _AIRCRAFT_POSITION_ObjectHandle
             << "."
             << std::endl;
        #endif
    }
    else if ( (theObjectClass == _ACTUATORS_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Actuators_Engines.c_str())) ) 
    {
         _Discov_ACTUATORS_ENGINES = true;
        _ACTUATORS_ENGINES_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        std::wcout << L"< Aircraft_Position > Object Instance has been discovered with handle "
             << _AIRCRAFT_POSITION_ObjectHandle
             << "."
             << std::endl;
        #endif
    }
    else if ( (theObjectClass == _AIRCRAFT_POSITION_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Aircraft_Position.c_str())) ) 
    {
        _Discov_AIRCRAFT_POSITION = true;
        _AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
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
        #ifdef DEBUG_VISUALIZATION_FED
        std::wcout << L"< Aircraft_Orientation > Object Instance has been discovered with handle "
             << _AIRCRAFT_ORIENTATION_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_UVW_SPEED_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Aircraft_UVW_Speed.c_str())) ) 
    {
        _Discov_AIRCRAFT_UVW_SPEED = true;
        _AIRCRAFT_UVW_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        std::wcout << L"< Aircraft_UVW_Speed > Object Instance has been discovered with handle "
             << _AIRCRAFT_UVW_SPEED_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Aircraft_PQR_Angular_Speed.c_str())) ) 
    {
        _Discov_AIRCRAFT_PQR_ANGULAR_SPEED = true;
        _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        std::wcout << L"< Aircraft_PQR_Angular_Speed > Object Instance has been discovered with handle "
             << _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_ACCELERATION_ClassHandle) &&  (!wcscmp(theObjectInstanceName.c_str(),Aircraft_Acceleration.c_str()))) 
    {
        _Discov_AIRCRAFT_ACCELERATION = true;
        _AIRCRAFT_ACCELERATION_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        std::wcout << L"< Aircraft_Acceleration > Object Instance has been discovered with handle "
             << _AIRCRAFT_ACCELERATION_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_SPEED_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Aircraft_Speed.c_str())) ) 
    {
        _Discov_AIRCRAFT_SPEED = true;
        _AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        std::wcout << L"< Aircraft_Speed > Object Instance has been discovered with handle "
             << _AIRCRAFT_SPEED_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_ADDITIONAL_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Aircraft_Additional.c_str())) ) 
    {
        _Discov_AIRCRAFT_ADDITIONAL = true;
        _AIRCRAFT_ADDITIONAL_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        std::wcout << L"< Aircraft_Additional > Object Instance has been discovered with handle "
             << _AIRCRAFT_ADDITIONAL_ObjectHandle
             << "."
             << std::endl;
        #endif
    }   
    #ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void VisualizationFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void VisualizationFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
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
	#ifdef DEBUG_VISUALIZATION_FED
    std::wcout << L"VisualizationFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void VisualizationFederateHla1516e::initialization() 
{
	// Nothing to do
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void VisualizationFederateHla1516e::calculateState() 
{
	// Nothing to do
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void VisualizationFederateHla1516e::calculateOutput() 
{
	// Model output calculation
	_Visualization.FlighGearSocketSend();
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void VisualizationFederateHla1516e::sendUpdateAttributes(RTI1516fedTime UpdateTime)
{
	// Nothing to do   
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void VisualizationFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
void VisualizationFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
				tmp_value = buffer.read_double(); 
				_Visualization.setLongitude(tmp_value); 
			}
            else if (parmHandle == _LATITUDE_ATTRIBUTE)  
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setLatitude(tmp_value);
			}
            else if (parmHandle == _ALTITUDE_ATTRIBUTE)  
            { 
				tmp_value = buffer.read_double();  
				_Visualization.setAltitude(tmp_value);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				tmp_value = buffer.read_double(); 
				_Visualization.setPhi(tmp_value);
			}
            else if (parmHandle == _THETA_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setTheta(tmp_value); 
			}
            else if (parmHandle == _PSI_ATTRIBUTE)   
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setPsi(tmp_value);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_AIRCRAFT_ORIENTATION = true;
    } 
    else if (theObject == _AIRCRAFT_UVW_SPEED_ObjectHandle)
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
            if      (parmHandle == _U_SPEED_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setUspeed(tmp_value);		
			}
            else if (parmHandle == _V_SPEED_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double();
				_Visualization.setVspeed(tmp_value); 
				
			}
            else if (parmHandle == _W_SPEED_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double();
				_Visualization.setWspeed(tmp_value);  	
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_AIRCRAFT_UVW_SPEED = true;
    } 
    else if (theObject == _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle)
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
            if      (parmHandle == _P_ANG_SPEED_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double();  
			}
            else if (parmHandle == _Q_ANG_SPEED_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double();  
			}
            else if (parmHandle == _R_ANG_SPEED_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double();  
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_AIRCRAFT_PQR_ANGULAR_SPEED = true;
    } 
    else if (theObject == _AIRCRAFT_ACCELERATION_ObjectHandle)
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
            if      (parmHandle == _X_ACC_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setXacc(tmp_value);
			}
            else if (parmHandle == _Y_ACC_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setYacc(tmp_value); 
			}
            else if (parmHandle == _Z_ACC_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setZacc(tmp_value);
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_AIRCRAFT_ACCELERATION = true;
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
				tmp_value = buffer.read_double();  
			}
            else if (parmHandle == _EQUIVALENT_AIRSPEED_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double();   
			}
            else if (parmHandle == _CALIBRATED_AIRSPEED_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setVcas(tmp_value);
			}
            else if (parmHandle == _TRUE_AIRSPEED_ATTRIBUTE)       
            { 
				tmp_value = buffer.read_double();  
			}
            else if (parmHandle == _GROUND_SPEED_ATTRIBUTE)        
            { 
				tmp_value = buffer.read_double();   
			}
            else if (parmHandle == _VERTICAL_SPEED_ATTRIBUTE)      
            { 
				tmp_value = buffer.read_double();    
			}
            else if (parmHandle == _MACH_NUMBER_ATTRIBUTE)        
            { 
				tmp_value = buffer.read_double();   
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_AIRCRAFT_SPEED = true;
    } 
    else if (theObject == _AIRCRAFT_ADDITIONAL_ObjectHandle)
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
            if      (parmHandle == _ALPHA_ATTRIBUTE)            
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setAlpha(tmp_value); 
			}
            else if (parmHandle == _BETA_ATTRIBUTE)             
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setBeta(tmp_value); 
			}
            else if (parmHandle == _DYNAMIC_PRESSURE_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double(); 
			}
            else if (parmHandle == _TANK_FILLING_ATTRIBUTE)     
            { 
				tmp_value = buffer.read_double(); 
			}
            else
            { 
				std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_AIRCRAFT_ADDITIONAL = true;
    } 
    else if (theObject == _ACTUATORS_CONTROL_SURFACES_ObjectHandle)
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

            if      (parmHandle == _RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE)  
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setRightAileron(tmp_value); 
			}
            else if (parmHandle == _LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE)   
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setLeftAileron(tmp_value); 
			}
            else if (parmHandle == _RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setElevator(tmp_value); 
			}
            else if (parmHandle == _LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE)  
            { 
				tmp_value = buffer.read_double();
				// Not used
				// _Visualization.setElevator(tmp_value);   
			}
            else if (parmHandle == _RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE)
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setRudder(tmp_value); 
			}
            else if (parmHandle == _FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE)
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setFlaps(tmp_value); 
			}
            else if (parmHandle == _SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE)
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setSpoilers(tmp_value);
			}
            else if (parmHandle == _STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE)
            { 
				tmp_value = buffer.read_double();  
			}
            else if (parmHandle == _GEARS_POSITION_ATTRIBUTE)
            { 
				tmp_value = buffer.read_double(); 
				_Visualization.setGears(tmp_value);
			}
			else
            { 
				std::wcout << L"VisualizationFederateHla13: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl;
			}
        }
        _New_ACTUATORS_CONTROL_SURFACES = true;
    } 
    else if (theObject == _ACTUATORS_ENGINES_ObjectHandle)
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

            if      (parmHandle == _RIGHT_ENGINE_THRUST_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double();  
			}
            else if (parmHandle == _LEFT_ENGINE_THRUST_ATTRIBUTE)  
            { 
				tmp_value = buffer.read_double();  
			}
			else
            { 
				std::wcout << L"VisualizationFederateHla13: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl;
			}
        }    
        _New_ACTUATORS_ENGINES = true;
    }    
	else
	{ 
		std::wcout << L"VisualizationFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << std::endl; 
	} 
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void VisualizationFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
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
void VisualizationFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
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
void VisualizationFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_VISUALIZATION_FED
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void VisualizationFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void VisualizationFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void VisualizationFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
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
void VisualizationFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
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
void VisualizationFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
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
void VisualizationFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
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
void VisualizationFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void VisualizationFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void VisualizationFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void VisualizationFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// function : run
void VisualizationFederateHla1516e::run()
{
	RTI1516fedTime updateTime(0.0);
	while (_LocalTime < _SimulationEndTime)
	{
		// Model calculations
		calculateOutput();
		timeAdvanceRequest(_TimeStep);
	}
} 
