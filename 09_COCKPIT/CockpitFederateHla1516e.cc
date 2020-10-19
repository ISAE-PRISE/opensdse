#include "CockpitFederateHla1516e.hh"
#include "CockpitFederateThreadHla1516e.hh"
#include "Cockpit.hh"

// ----------------------------------------------------------------------------
// TemplateFederateHla13 Constructor
CockpitFederateHla1516e::CockpitFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> Constructor(): Start" << std::endl;
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
	std::wcout << L"CockpitFederateHla1516e.cc -> Constructor(): End" << std::endl;
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
CockpitFederateHla1516e::~CockpitFederateHla1516e()
{
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void CockpitFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void CockpitFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void CockpitFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
														 , L"cockpit"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L"CockpitFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void CockpitFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle CockpitFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void CockpitFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void CockpitFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{

		// Subscribed
		std::wstring MEAS_AIRCRAFT_POSITION (L"MEAS_AIRCRAFT_POSITION");
		std::wstring MEAS_AIRCRAFT_ORIENTATION (L"MEAS_AIRCRAFT_ORIENTATION");
		std::wstring MEAS_AIRCRAFT_UVW_SPEED (L"MEAS_AIRCRAFT_UVW_SPEED");
		std::wstring MEAS_AIRCRAFT_PQR_ANGULAR_SPEED (L"MEAS_AIRCRAFT_PQR_ANGULAR_SPEED");
		std::wstring MEAS_AIRCRAFT_ACCELERATION (L"MEAS_AIRCRAFT_ACCELERATION");
		std::wstring MEAS_AIRCRAFT_SPEED (L"MEAS_AIRCRAFT_SPEED");
		std::wstring MEAS_AIRCRAFT_ADDITIONAL (L"MEAS_AIRCRAFT_ADDITIONAL");
		std::wstring MEAS_LONGITUDE (L"MEAS_LONGITUDE");
		std::wstring MEAS_LATITUDE (L"MEAS_LATITUDE");
		std::wstring MEAS_ALTITUDE (L"MEAS_ALTITUDE");
		std::wstring MEAS_PHI (L"MEAS_PHI");
		std::wstring MEAS_THETA (L"MEAS_THETA");
		std::wstring MEAS_PSI (L"MEAS_PSI");
		std::wstring MEAS_U_SPEED (L"MEAS_U_SPEED");
		std::wstring MEAS_V_SPEED (L"MEAS_V_SPEED");
		std::wstring MEAS_W_SPEED (L"MEAS_W_SPEED");
		std::wstring MEAS_P_ANG_SPEED (L"MEAS_P_ANG_SPEED");
		std::wstring MEAS_Q_ANG_SPEED (L"MEAS_Q_ANG_SPEED");
		std::wstring MEAS_R_ANG_SPEED (L"MEAS_R_ANG_SPEED");
		std::wstring MEAS_X_ACC (L"MEAS_X_ACC");
		std::wstring MEAS_Y_ACC (L"MEAS_Y_ACC");
		std::wstring MEAS_Z_ACC (L"MEAS_Z_ACC");
		std::wstring MEAS_INDICATED_AIRSPEED (L"MEAS_INDICATED_AIRSPEED");
		std::wstring MEAS_EQUIVALENT_AIRSPEED (L"MEAS_EQUIVALENT_AIRSPEED");
		std::wstring MEAS_CALIBRATED_AIRSPEED (L"MEAS_CALIBRATED_AIRSPEED");
		std::wstring MEAS_TRUE_AIRSPEED (L"MEAS_TRUE_AIRSPEED");
		std::wstring MEAS_GROUND_SPEED (L"MEAS_GROUND_SPEED");
		std::wstring MEAS_VERTICAL_SPEED (L"MEAS_VERTICAL_SPEED");
		std::wstring MEAS_MACH_NUMBER (L"MEAS_MACH_NUMBER");
		std::wstring MEAS_ALPHA (L"MEAS_ALPHA");
		std::wstring MEAS_BETA (L"MEAS_BETA");
		std::wstring MEAS_DYNAMIC_PRESSURE (L"MEAS_DYNAMIC_PRESSURE");
		std::wstring MEAS_TANK_FILLING (L"MEAS_TANK_FILLING");
		_MEAS_AIRCRAFT_POSITION_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_POSITION);
		_MEAS_AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_ORIENTATION);
		_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_UVW_SPEED);
		_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_PQR_ANGULAR_SPEED);
		_MEAS_AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_ACCELERATION);
		_MEAS_AIRCRAFT_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_SPEED);
		_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_ADDITIONAL);
		_MEAS_LONGITUDE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_POSITION_ClassHandle, MEAS_LONGITUDE);
        _MEAS_LATITUDE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_POSITION_ClassHandle, MEAS_LATITUDE);
        _MEAS_ALTITUDE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_POSITION_ClassHandle, MEAS_ALTITUDE);     
        _MEAS_PHI_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle, MEAS_PHI);
        _MEAS_THETA_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle, MEAS_THETA);
        _MEAS_PSI_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle, MEAS_PSI);     
        _MEAS_U_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle, MEAS_U_SPEED);
        _MEAS_V_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle, MEAS_V_SPEED);
        _MEAS_W_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle, MEAS_W_SPEED);
        _MEAS_P_ANG_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle, MEAS_P_ANG_SPEED);
        _MEAS_Q_ANG_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle, MEAS_Q_ANG_SPEED);
        _MEAS_R_ANG_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle, MEAS_R_ANG_SPEED);    
        _MEAS_X_ACC_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle, MEAS_X_ACC);
        _MEAS_Y_ACC_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle, MEAS_Y_ACC);
        _MEAS_Z_ACC_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle, MEAS_Z_ACC);      
        _MEAS_INDICATED_AIRSPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_SPEED_ClassHandle, MEAS_INDICATED_AIRSPEED);
        _MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_SPEED_ClassHandle, MEAS_EQUIVALENT_AIRSPEED);
        _MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_SPEED_ClassHandle, MEAS_CALIBRATED_AIRSPEED);
        _MEAS_TRUE_AIRSPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_SPEED_ClassHandle, MEAS_TRUE_AIRSPEED);
        _MEAS_GROUND_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_SPEED_ClassHandle, MEAS_GROUND_SPEED);
        _MEAS_VERTICAL_SPEED_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_SPEED_ClassHandle, MEAS_VERTICAL_SPEED);
        _MEAS_MACH_NUMBER_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_SPEED_ClassHandle, MEAS_MACH_NUMBER);  
        _MEAS_ALPHA_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle, MEAS_ALPHA);
        _MEAS_BETA_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle, MEAS_BETA);
        _MEAS_DYNAMIC_PRESSURE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle, MEAS_DYNAMIC_PRESSURE); 
        _MEAS_TANK_FILLING_ATTRIBUTE = _RtiAmb->getAttributeHandle(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle, MEAS_TANK_FILLING);
        
        std::wstring ACTUATORS (L"ACTUATORS");
        std::wstring RIGHT_ENGINE_THRUST (L"RIGHT_ENGINE_THRUST");
		std::wstring LEFT_ENGINE_THRUST (L"LEFT_ENGINE_THRUST");
		_ACTUATORS_ClassHandle = _RtiAmb->getObjectClassHandle(ACTUATORS);
		_RIGHT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, RIGHT_ENGINE_THRUST);
		_LEFT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, LEFT_ENGINE_THRUST);
		
		std::wstring ENVIRONMENT_VARIABLES (L"ENVIRONMENT_VARIABLES");
		std::wstring PRESSURE (L"PRESSURE");
		_ENVIRONMENT_VARIABLES_ClassHandle = _RtiAmb->getObjectClassHandle(ENVIRONMENT_VARIABLES);
		_PRESSURE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ENVIRONMENT_VARIABLES_ClassHandle, PRESSURE);
        
        // Published
        std::wstring COCKPIT (L"COCKPIT");
        std::wstring AUTOPILOT_AP_ACTIVE (L"AUTOPILOT_AP_ACTIVE");
		std::wstring AUTOPILOT_ATHR_ACTIVE (L"AUTOPILOT_ATHR_ACTIVE");
		std::wstring AUTOPILOT_SPD (L"AUTOPILOT_SPD");
		std::wstring AUTOPILOT_HDG (L"AUTOPILOT_HDG");
		std::wstring AUTOPILOT_ALT (L"AUTOPILOT_ALT");
		std::wstring AUTOPILOT_VS (L"AUTOPILOT_VS");
		
		_COCKPIT_ClassHandle = _RtiAmb->getObjectClassHandle(COCKPIT);
		_AUTOPILOT_AP_ACTIVE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_COCKPIT_ClassHandle, AUTOPILOT_AP_ACTIVE);
        _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_COCKPIT_ClassHandle, AUTOPILOT_ATHR_ACTIVE);
        _AUTOPILOT_SPD_ATTRIBUTE = _RtiAmb->getAttributeHandle(_COCKPIT_ClassHandle, AUTOPILOT_SPD);
        _AUTOPILOT_HDG_ATTRIBUTE = _RtiAmb->getAttributeHandle(_COCKPIT_ClassHandle, AUTOPILOT_HDG);
        _AUTOPILOT_ALT_ATTRIBUTE = _RtiAmb->getAttributeHandle(_COCKPIT_ClassHandle, AUTOPILOT_ALT);
        _AUTOPILOT_VS_ATTRIBUTE = _RtiAmb->getAttributeHandle(_COCKPIT_ClassHandle, AUTOPILOT_VS);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void CockpitFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		attr_ACTUATORS_ENGINES.insert(_RIGHT_ENGINE_THRUST_ATTRIBUTE);
		attr_ACTUATORS_ENGINES.insert(_LEFT_ENGINE_THRUST_ATTRIBUTE);
        
        attr_ENVIRONMENT_VARIABLES.insert(_PRESSURE_ATTRIBUTE);
		
		attr_MEAS_AIRCRAFT_POSITION.insert(_MEAS_LONGITUDE_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_POSITION.insert(_MEAS_LATITUDE_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_POSITION.insert(_MEAS_ALTITUDE_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ORIENTATION.insert(_MEAS_PHI_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ORIENTATION.insert(_MEAS_THETA_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ORIENTATION.insert(_MEAS_PSI_ATTRIBUTE);   
		attr_MEAS_AIRCRAFT_UVW_SPEED.insert(_MEAS_U_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_UVW_SPEED.insert(_MEAS_V_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_UVW_SPEED.insert(_MEAS_W_SPEED_ATTRIBUTE);  
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.insert(_MEAS_P_ANG_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.insert(_MEAS_Q_ANG_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.insert(_MEAS_R_ANG_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ACCELERATION.insert(_MEAS_X_ACC_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ACCELERATION.insert(_MEAS_Y_ACC_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ACCELERATION.insert(_MEAS_Z_ACC_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED.insert(_MEAS_INDICATED_AIRSPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED.insert(_MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED.insert(_MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED.insert(_MEAS_TRUE_AIRSPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED.insert(_MEAS_GROUND_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED.insert(_MEAS_VERTICAL_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED.insert(_MEAS_MACH_NUMBER_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ADDITIONAL.insert(_MEAS_ALPHA_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ADDITIONAL.insert(_MEAS_BETA_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ADDITIONAL.insert(_MEAS_DYNAMIC_PRESSURE_ATTRIBUTE);  
		attr_MEAS_AIRCRAFT_ADDITIONAL.insert(_MEAS_TANK_FILLING_ATTRIBUTE);
		
		_RtiAmb->subscribeObjectClassAttributes(_ACTUATORS_ClassHandle,attr_ACTUATORS_ENGINES);
        _RtiAmb->subscribeObjectClassAttributes(_ENVIRONMENT_VARIABLES_ClassHandle,attr_ENVIRONMENT_VARIABLES);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_POSITION_ClassHandle, attr_MEAS_AIRCRAFT_POSITION);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle, attr_MEAS_AIRCRAFT_ORIENTATION);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle, attr_MEAS_AIRCRAFT_UVW_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle, attr_MEAS_AIRCRAFT_ACCELERATION);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_SPEED_ClassHandle, attr_MEAS_AIRCRAFT_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle, attr_MEAS_AIRCRAFT_ADDITIONAL);
        
        // For Class/Attributes published
        attr_COCKPIT.insert(_AUTOPILOT_AP_ACTIVE_ATTRIBUTE);
		attr_COCKPIT.insert(_AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE);
		attr_COCKPIT.insert(_AUTOPILOT_SPD_ATTRIBUTE);
		attr_COCKPIT.insert(_AUTOPILOT_HDG_ATTRIBUTE);
		attr_COCKPIT.insert(_AUTOPILOT_ALT_ATTRIBUTE);
		attr_COCKPIT.insert(_AUTOPILOT_VS_ATTRIBUTE); 
		_RtiAmb->publishObjectClassAttributes(_COCKPIT_ClassHandle,attr_COCKPIT);

	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void CockpitFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring Cockpit (L"Cockpit");
		_COCKPIT_ObjectHandle = _RtiAmb->registerObjectInstance(_COCKPIT_ClassHandle,Cockpit);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void CockpitFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb->unsubscribeObjectClass(_ACTUATORS_ClassHandle);  
        _RtiAmb->unsubscribeObjectClass(_ENVIRONMENT_VARIABLES_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
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
        _RtiAmb->unpublishObjectClass(_COCKPIT_ClassHandle);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void CockpitFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
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
		
	_Discov_MEAS_AIRCRAFT_POSITION = false;
	_Discov_MEAS_AIRCRAFT_ORIENTATION = false;
	_Discov_MEAS_AIRCRAFT_UVW_SPEED = false;
	_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ACCELERATION = false;
	_Discov_MEAS_AIRCRAFT_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ADDITIONAL = false;
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void CockpitFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
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
    _New_MEAS_AIRCRAFT_POSITION = false;
    _New_MEAS_AIRCRAFT_ORIENTATION = false;
    _New_MEAS_AIRCRAFT_UVW_SPEED = false;
    _New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
    _New_MEAS_AIRCRAFT_ACCELERATION = false;
    _New_MEAS_AIRCRAFT_SPEED = false;
    _New_MEAS_AIRCRAFT_ADDITIONAL = false;
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void CockpitFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
	std::wstring Actuators_Engines (L"Actuators_Engines");
	std::wstring Environment_Variables (L"Environment_Variables");
    
    std::wstring Meas_Aircraft_Position (L"Meas_Aircraft_Position");
	std::wstring Meas_Aircraft_Orientation (L"Meas_Aircraft_Orientation");
    std::wstring Meas_Aircraft_UVW_Speed (L"Meas_Aircraft_UVW_Speed");
    std::wstring Meas_Aircraft_PQR_Angular_Speed (L"Meas_Aircraft_PQR_Angular_Speed");
    std::wstring Meas_Aircraft_Acceleration (L"Meas_Aircraft_Acceleration");
	std::wstring Meas_Aircraft_Speed (L"Meas_Aircraft_Speed");
    std::wstring Meas_Aircraft_Additional (L"Meas_Aircraft_Additional");
    
    if ( (theObjectClass == _ACTUATORS_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Actuators_Engines.c_str())) ) 
    {
        _Discov_ACTUATORS_ENGINES = true;
        _ACTUATORS_ENGINES_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Actuators_Engines > Object Instance has been discovered with handle "
             << _ACTUATORS_ENGINES_ObjectHandle
             << "."
             << std::endl;
		#endif
    }
    else if ( (theObjectClass == _ENVIRONMENT_VARIABLES_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Environment_Variables.c_str())) ) 
    {
       _Discov_ENVIRONMENT_VARIABLES = true;
        _ENVIRONMENT_VARIABLES_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Environment_Variables > Object Instance has been discovered with handle "
             << _ENVIRONMENT_VARIABLES_ObjectHandle
             << "."
             << std::endl;
        #endif
    }
    else if ( (theObjectClass == _MEAS_AIRCRAFT_POSITION_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_Position.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_POSITION = true;
        _MEAS_AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_Position > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_POSITION_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ORIENTATION_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_Orientation.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_ORIENTATION = true;
        _MEAS_AIRCRAFT_ORIENTATION_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_Orientation > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_ORIENTATION_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_UVW_SPEED_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_UVW_Speed.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_UVW_SPEED = true;
        _MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_UVW_Speed > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_PQR_Angular_Speed.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = true;
        _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_PQR_Angular_Speed > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ACCELERATION_ClassHandle) &&  (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_Acceleration.c_str()))) 
    {
        _Discov_MEAS_AIRCRAFT_ACCELERATION = true;
        _MEAS_AIRCRAFT_ACCELERATION_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_Acceleration > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_ACCELERATION_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_SPEED_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_Speed.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_SPEED = true;
        _MEAS_AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_Speed > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_SPEED_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_Additional.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_ADDITIONAL = true;
        _MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_Additional > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle
             << "."
             << std::endl;
        #endif
    }   
    #ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void CockpitFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void CockpitFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
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
	#ifdef DEBUG_COCKPIT_FED
    std::wcout << L"CockpitFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void CockpitFederateHla1516e::initialization() 
{
	waitForAllObjectDiscovered();
	waitForAllAttributesReceived();
	sendInitialAllAttributes();
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void CockpitFederateHla1516e::calculateState() 
{
	// Model State calculation
	// Not used
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void CockpitFederateHla1516e::calculateOutput() 
{
	// Model output calculation
	// Not used 
}

// ----------------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void CockpitFederateHla1516e::sendInitialAllAttributes()
{

	rti1516e::AttributeHandleValueMap ahvps_COCKPIT;

    
    rti1516e::VariableLengthData MyTag1;
	std::string testTag ("Cockpit");
	MyTag1.setData (testTag.c_str (), testTag.size () + 1);
	
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla1516e::window->Get_Autopilot_AP());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_COCKPIT.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _AUTOPILOT_AP_ACTIVE_ATTRIBUTE,attrValue1)); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla1516e::window->Get_Autopilot_athr());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_COCKPIT.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE,attrValue2));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla1516e::window->Get_Autopilot_spd());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_COCKPIT.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _AUTOPILOT_SPD_ATTRIBUTE,attrValue3));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla1516e::window->Get_Autopilot_hdg());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_COCKPIT.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _AUTOPILOT_HDG_ATTRIBUTE,attrValue4));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla1516e::window->Get_Autopilot_alt());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue5 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_COCKPIT.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _AUTOPILOT_ALT_ATTRIBUTE,attrValue5));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla1516e::window->Get_Autopilot_vs());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue6 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_COCKPIT.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _AUTOPILOT_VS_ATTRIBUTE,attrValue6));

    try 
    {
		_RtiAmb->updateAttributeValues(_COCKPIT_ObjectHandle, ahvps_COCKPIT, MyTag1);
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
// Send Updates for all attributes
void CockpitFederateHla1516e::sendUpdateAttributes(RTI1516fedTime UpdateTime)
{   
	rti1516e::AttributeHandleValueMap ahvps_COCKPIT;

    
    rti1516e::VariableLengthData MyTag1;
	std::string testTag ("Cockpit");
	MyTag1.setData (testTag.c_str (), testTag.size () + 1);
	
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla1516e::window->Get_Autopilot_AP());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_COCKPIT.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _AUTOPILOT_AP_ACTIVE_ATTRIBUTE,attrValue1)); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla1516e::window->Get_Autopilot_athr());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_COCKPIT.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE,attrValue2));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla1516e::window->Get_Autopilot_spd());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_COCKPIT.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _AUTOPILOT_SPD_ATTRIBUTE,attrValue3));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla1516e::window->Get_Autopilot_hdg());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_COCKPIT.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _AUTOPILOT_HDG_ATTRIBUTE,attrValue4));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla1516e::window->Get_Autopilot_alt());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue5 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_COCKPIT.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _AUTOPILOT_ALT_ATTRIBUTE,attrValue5));

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(CockpitFederateThreadHla1516e::window->Get_Autopilot_vs());
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue6 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_COCKPIT.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _AUTOPILOT_VS_ATTRIBUTE,attrValue6));

    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
			_RtiAmb->updateAttributeValues(_COCKPIT_ObjectHandle, ahvps_COCKPIT, MyTag1);
        }
        else
        {
			_RtiAmb->updateAttributeValues(_COCKPIT_ObjectHandle, ahvps_COCKPIT, MyTag1, UpdateTime);

        }
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void CockpitFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
void CockpitFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
	
	if (theObject == _MEAS_AIRCRAFT_POSITION_ObjectHandle)
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
 				CockpitFederateThreadHla1516e::window->set_PFD_altitude(_Cockpit.GET_AIRCRAFT_POSITION_ALTITUDE()); 
			}
            else
            { 
				std::wcout << L"EfcsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_MEAS_AIRCRAFT_POSITION = true;
    } 

    else if (theObject == _MEAS_AIRCRAFT_ORIENTATION_ObjectHandle)
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
            if      (parmHandle == _MEAS_PHI_ATTRIBUTE)   
            { 
				_Cockpit.SET_AIRCRAFT_ORIENTATION_PHI(buffer.read_double()*180/M_PI); 
 				CockpitFederateThreadHla1516e::window->set_PFD_roll(_Cockpit.GET_AIRCRAFT_ORIENTATION_PHI()); 
			}
            else if (parmHandle == _MEAS_THETA_ATTRIBUTE) 
            {
				_Cockpit.SET_AIRCRAFT_ORIENTATION_THETA(buffer.read_double()*180/M_PI); 
 				CockpitFederateThreadHla1516e::window->set_PFD_pitch(_Cockpit.GET_AIRCRAFT_ORIENTATION_THETA());
			}
            else if (parmHandle == _MEAS_PSI_ATTRIBUTE)   
            { 
				_Cockpit.SET_AIRCRAFT_ORIENTATION_PSI(buffer.read_double()*180/M_PI); 
				_Cockpit.SET_COCKPIT_AUTOPILOT_HDG(CockpitFederateThreadHla1516e::window->Get_Autopilot_hdg());
 				CockpitFederateThreadHla1516e::window->set_PFD_heading(_Cockpit.GET_AIRCRAFT_ORIENTATION_PSI()); 
 				CockpitFederateThreadHla1516e::window->set_ND_heading(_Cockpit.GET_AIRCRAFT_ORIENTATION_PSI()); 
 				CockpitFederateThreadHla1516e::window->set_ND_headingBug(_Cockpit.GET_COCKPIT_AUTOPILOT_HDG());
			}
            else
            { 
				std::wcout << L"EfcsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_ORIENTATION = true;
    } 
    else if (theObject == _MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle)
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
				std::wcout << L"EfcsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_UVW_SPEED = true;
    } 
    else if (theObject == _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle)
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
				std::wcout << L"EfcsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = true;
    } 
    else if (theObject == _MEAS_AIRCRAFT_ACCELERATION_ObjectHandle)
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
				std::wcout << L"EfcsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_ACCELERATION = true;
    } 
    else if (theObject == _MEAS_AIRCRAFT_SPEED_ObjectHandle)
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
            if      (parmHandle == _MEAS_INDICATED_AIRSPEED_ATTRIBUTE)  
            { 
				_Cockpit.SET_AIRCRAFT_SPEED_IAS(buffer.read_double()*1.94); // why 1.94?? kind of magic number ...
 				CockpitFederateThreadHla1516e::window->set_PFD_airspeed(_Cockpit.GET_AIRCRAFT_SPEED_IAS()); 
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
 				CockpitFederateThreadHla1516e::window->set_ECAM_mach(_Cockpit.GET_AIRCRAFT_SPEED_MACH());
 				CockpitFederateThreadHla1516e::window->set_PFD_mach(_Cockpit.GET_AIRCRAFT_SPEED_MACH());
			}
            else
            { 
				std::wcout << L"EfcsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_SPEED = true;
    } 
    else if (theObject == _MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle)
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
            if      (parmHandle == _MEAS_ALPHA_ATTRIBUTE)            
            { 
				_Cockpit.SET_AIRCRAFT_ADDITIONAL_ALPHA(buffer.read_double());
				// Conversion from Radians to Degrees ==> *180/M_PI 
				CockpitFederateThreadHla1516e::window->set_PFD_FlightPath( _Cockpit.GET_AIRCRAFT_ADDITIONAL_ALPHA()*180/M_PI
				                                                 , _Cockpit.GET_AIRCRAFT_ADDITIONAL_BETA()*180/M_PI
				                                                 ); 
			}
            else if (parmHandle == _MEAS_BETA_ATTRIBUTE)             
            { 
				_Cockpit.SET_AIRCRAFT_ADDITIONAL_BETA(buffer.read_double()); 
				// Conversion from Radians to Degrees ==> *180/M_PI
				CockpitFederateThreadHla1516e::window->set_PFD_FlightPath( _Cockpit.GET_AIRCRAFT_ADDITIONAL_ALPHA()*180/M_PI
				                                                 , _Cockpit.GET_AIRCRAFT_ADDITIONAL_BETA()*180/M_PI
				                                                 );
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
				std::wcout << L"EfcsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_ADDITIONAL = true;
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
				_Cockpit.SET_ACTUATORS_RIGHT_ENGINE_THRUST(buffer.read_double()); 
 				CockpitFederateThreadHla1516e::window->set_ECAM_right_engine_thr(_Cockpit.GET_ACTUATORS_RIGHT_ENGINE_THRUST()); 
			}
            else if (parmHandle == _LEFT_ENGINE_THRUST_ATTRIBUTE)  
            { 
				_Cockpit.SET_ACTUATORS_LEFT_ENGINE_THRUST(buffer.read_double()); 
 				CockpitFederateThreadHla1516e::window->set_ECAM_left_engine_thr(_Cockpit.GET_ACTUATORS_LEFT_ENGINE_THRUST());  
			}
            else
            { 
				std::wcout << L"CockpitFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_ACTUATORS_ENGINES = true;
    } 
    else if (theObject == _ENVIRONMENT_VARIABLES_ObjectHandle)
    {for (it = theAttributes.begin (); it != theAttributes.end (); ++it) 
		{
			parmHandle = it->first;
			valueLength = it->second.size();
			assert(valueLength>0);
			buffer.resize(valueLength);        
			buffer.reset(); 
			std::memcpy(static_cast<char*>(buffer(0)),(it->second).data (), valueLength);                
			buffer.assumeSizeFromReservedBytes();
            if      (parmHandle == _PRESSURE_ATTRIBUTE)
            { 
				_Cockpit.SET_ENVIRONMENT_VARIABLES_PRESSURE(buffer.read_double()); 
 				CockpitFederateThreadHla1516e::window->set_ECAM_pressure(_Cockpit.GET_ENVIRONMENT_VARIABLES_PRESSURE()); 
			}
			else
            { 
				std::wcout << L"CockpitFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
            
        }    
        _New_ENVIRONMENT_VARIABLES = true;
    }
	else
	{ 
		std::wcout << L"CockpitFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << std::endl; 
	} 
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void CockpitFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
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
void CockpitFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
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
void CockpitFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_COCKPIT_FED
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void CockpitFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void CockpitFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void CockpitFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void CockpitFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void CockpitFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void CockpitFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void CockpitFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
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
void CockpitFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
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
void CockpitFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
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
void CockpitFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
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
void CockpitFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void CockpitFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void CockpitFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void CockpitFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// function : run
void CockpitFederateHla1516e::run()
{
	RTI1516fedTime updateTime(0.0);
	while (_LocalTime < _SimulationEndTime)
	{
		// Model calculations
		calculateState();
		updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
		sendUpdateAttributes(updateTime);
		timeAdvanceRequest(_TimeStep);
	}
} 

// ----------------------------------------------------------------------------
// function : run
void CockpitFederateHla1516e::runOneStep()
{
	// Model calculations
	RTI1516fedTime updateTime(0.0);
	updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
	sendUpdateAttributes(updateTime);
	timeAdvanceRequest(_TimeStep);
} 

bool CockpitFederateHla1516e::getEndOfSimulation()
{
	return (_LocalTime < _SimulationEndTime);
}
