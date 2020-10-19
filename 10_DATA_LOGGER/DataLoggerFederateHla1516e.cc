#include "DataLoggerFederateHla1516e.hh"

// ----------------------------------------------------------------------------
// TemplateFederateHla13 Constructor
DataLoggerFederateHla1516e::DataLoggerFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_DATA_LOGGER_FED
	std::wcout << L"DataLoggerFederateHla1516e.cc -> Constructor(): Start" << std::endl;
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
	_IsTimeReg = _IsTimeConst = _IsTimeAdvanceGrant = false ;
	// Note: FD is creator for the SDSE federation
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = _IsSyncAnnonced = false;
	_IsOutMesTimespamped = true;
	
	XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);
	// Get values from xml file
	vector<string> attribute_path(2);
	attribute_path[0] = "DATA_LOGGER";
	attribute_path[1] = "LogFileName";
	_FileName = getTextValue(params, attribute_path);
	attribute_path[1] = "Granularity";
	_Granularity = getIntValue(params, attribute_path);
	delete params;
	

	_Discov_TRIM_DATA_FLIGHT_DYNAMICS		= false;
	_Discov_TRIM_DATA_ENGINES				= false;
	_Discov_JOYSTICK						= false;
	_Discov_FLIGHT_CONTROLS					= false;
	_Discov_ACTUATORS_CONTROL_SURFACES		= false;
	_Discov_ACTUATORS_ENGINES				= false;
	_Discov_AIRCRAFT_POSITION				= false;
	_Discov_AIRCRAFT_ORIENTATION			= false;
	_Discov_AIRCRAFT_UVW_SPEED				= false;
	_Discov_AIRCRAFT_PQR_ANGULAR_SPEED		= false;
	_Discov_AIRCRAFT_ACCELERATION			= false;
	_Discov_AIRCRAFT_SPEED					= false;
	_Discov_AIRCRAFT_ADDITIONAL				= false;
	_Discov_MEAS_AIRCRAFT_POSITION			= false;
	_Discov_MEAS_AIRCRAFT_ORIENTATION		= false;
	_Discov_MEAS_AIRCRAFT_UVW_SPEED			= false;
	_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED	= false;
	_Discov_MEAS_AIRCRAFT_ACCELERATION		= false;
	_Discov_MEAS_AIRCRAFT_SPEED				= false;
	_Discov_MEAS_AIRCRAFT_ADDITIONAL		= false;
	_Discov_ENVIRONMENT_VARIABLES			= false;
	_Discov_WIND_COMPONENTS					= false;
	_Discov_COCKPIT                         = false;

	_New_TRIM_DATA_ENGINES					= false;
	_New_TRIM_DATA_FLIGHT_DYNAMICS			= false;
	_New_JOYSTICK							= false;
	_New_FLIGHT_CONTROLS					= false;
	_New_ACTUATORS_CONTROL_SURFACES			= false;
	_New_ACTUATORS_ENGINES					= false;
	_New_AIRCRAFT_POSITION					= false;
	_New_AIRCRAFT_ORIENTATION				= false;
	_New_AIRCRAFT_UVW_SPEED					= false;
	_New_AIRCRAFT_PQR_ANGULAR_SPEED			= false;
	_New_AIRCRAFT_ACCELERATION				= false;
	_New_AIRCRAFT_SPEED						= false;
	_New_AIRCRAFT_ADDITIONAL				= false;
	_New_MEAS_AIRCRAFT_POSITION				= false;
	_New_MEAS_AIRCRAFT_ORIENTATION			= false;
	_New_MEAS_AIRCRAFT_UVW_SPEED			= false;
	_New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED	= false;
	_New_MEAS_AIRCRAFT_ACCELERATION			= false;
	_New_MEAS_AIRCRAFT_SPEED				= false;
	_New_MEAS_AIRCRAFT_ADDITIONAL			= false;
	_New_ENVIRONMENT_VARIABLES				= false;
	_New_WIND_COMPONENTS					= false;
	_New_COCKPIT                            = false;
	
	memset(&_TimeStamp, 0, sizeof(timespec));
	memset(&_TimeStamp_old, 0, sizeof(timespec));
	memset(&_ExecutionTime, 0, sizeof(timespec));

	#ifdef DEBUG_DATA_LOGGER_FED
	std::wcout << L"DataLoggerFederateHla1516e.cc -> Constructor(): End" << std::endl;
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
DataLoggerFederateHla1516e::~DataLoggerFederateHla1516e()
{
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void DataLoggerFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_DATA_LOGGER_FED
	std::wcout << L"DataLoggerFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_DATA_LOGGER_FED
	std::wcout << L"DataLoggerFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void DataLoggerFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_DATA_LOGGER_FED
	std::wcout << L"DataLoggerFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_DATA_LOGGER_FED
	std::wcout << L"DataLoggerFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_DATA_LOGGER_FED
	std::wcout << L"DataLoggerFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
														 , L"datalogger"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_DATA_LOGGER_FED
	std::wcout << L"DataLoggerFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle DataLoggerFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void DataLoggerFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void DataLoggerFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
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
        
        std::wstring TRIM_DATA (L"TRIM_DATA");
		std::wstring JOYSTICK (L"JOYSTICK");
		std::wstring COCKPIT (L"COCKPIT");
		std::wstring RIGHT_ENGINE_THROTTLE_EQ (L"RIGHT_ENGINE_THROTTLE_EQ");
		std::wstring LEFT_ENGINE_THROTTLE_EQ (L"LEFT_ENGINE_THROTTLE_EQ");
		std::wstring ELEVATOR_DEFLECTION_EQ (L"ELEVATOR_DEFLECTION_EQ");
		std::wstring STABILIZER_DEFLECTION_EQ (L"STABILIZER_DEFLECTION_EQ");
		std::wstring AILERON (L"AILERON");
		std::wstring ELEVATOR (L"ELEVATOR");
		std::wstring RUDDER (L"RUDDER");
		std::wstring THROTTLE_LEFT (L"THROTTLE_LEFT");
		std::wstring THROTTLE_RIGHT (L"THROTTLE_RIGHT");
		std::wstring FLAPS (L"FLAPS");
		std::wstring SPOILERS (L"SPOILERS");
		std::wstring GEARS (L"GEARS");
		std::wstring BRAKES (L"BRAKES");
		std::wstring AUTOPILOT_AP_ACTIVE (L"AUTOPILOT_AP_ACTIVE");
		std::wstring AUTOPILOT_ATHR_ACTIVE (L"AUTOPILOT_ATHR_ACTIVE");
		std::wstring AUTOPILOT_SPD (L"AUTOPILOT_SPD");
		std::wstring AUTOPILOT_HDG (L"AUTOPILOT_HDG");
		std::wstring AUTOPILOT_ALT (L"AUTOPILOT_ALT");
		std::wstring AUTOPILOT_VS (L"AUTOPILOT_VS");
		_TRIM_DATA_ClassHandle = _RtiAmb->getObjectClassHandle(TRIM_DATA);     
		_JOYSTICK_ClassHandle = _RtiAmb->getObjectClassHandle(JOYSTICK);
		_COCKPIT_ClassHandle = _RtiAmb->getObjectClassHandle(COCKPIT);
		_RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, RIGHT_ENGINE_THROTTLE_EQ);
        _LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, LEFT_ENGINE_THROTTLE_EQ);
        _ELEVATOR_DEFLECTION_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, ELEVATOR_DEFLECTION_EQ);
        _STABILIZER_DEFLECTION_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, STABILIZER_DEFLECTION_EQ);    
        _AILERON_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, AILERON);
        _ELEVATOR_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, ELEVATOR);
        _RUDDER_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, RUDDER);
        _THROTTLE_LEFT_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, THROTTLE_LEFT);
        _THROTTLE_RIGHT_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, THROTTLE_RIGHT);
        _FLAPS_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, FLAPS);
        _SPOILERS_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, SPOILERS);
        _GEARS_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, GEARS);   
        _BRAKES_ATTRIBUTE = _RtiAmb->getAttributeHandle(_JOYSTICK_ClassHandle, BRAKES);
        _AUTOPILOT_AP_ACTIVE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_COCKPIT_ClassHandle, AUTOPILOT_AP_ACTIVE);
        _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_COCKPIT_ClassHandle, AUTOPILOT_ATHR_ACTIVE);
        _AUTOPILOT_SPD_ATTRIBUTE = _RtiAmb->getAttributeHandle(_COCKPIT_ClassHandle, AUTOPILOT_SPD);
        _AUTOPILOT_HDG_ATTRIBUTE = _RtiAmb->getAttributeHandle(_COCKPIT_ClassHandle, AUTOPILOT_HDG);
        _AUTOPILOT_ALT_ATTRIBUTE = _RtiAmb->getAttributeHandle(_COCKPIT_ClassHandle, AUTOPILOT_ALT);
        _AUTOPILOT_VS_ATTRIBUTE = _RtiAmb->getAttributeHandle(_COCKPIT_ClassHandle, AUTOPILOT_VS);
        
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
        
        std::wstring FLIGHT_CONTROLS (L"FLIGHT_CONTROLS");
		std::wstring RIGHT_AILERON_COMMANDED_DEFLECTION (L"RIGHT_AILERON_COMMANDED_DEFLECTION");
		std::wstring LEFT_AILERON_COMMANDED_DEFLECTION (L"LEFT_AILERON_COMMANDED_DEFLECTION");
		std::wstring RIGHT_ELEVATOR_COMMANDED_DEFLECTION (L"RIGHT_ELEVATOR_COMMANDED_DEFLECTION");
		std::wstring LEFT_ELEVATOR_COMMANDED_DEFLECTION (L"LEFT_ELEVATOR_COMMANDED_DEFLECTION");
		std::wstring RUDDER_COMMANDED_DEFLECTION (L"RUDDER_COMMANDED_DEFLECTION");
		std::wstring RIGHT_ENGINE_COMMANDED_THROTTLE (L"RIGHT_ENGINE_COMMANDED_THROTTLE");
		std::wstring LEFT_ENGINE_COMMANDED_THROTTLE (L"LEFT_ENGINE_COMMANDED_THROTTLE");
		std::wstring FLAPS_COMMANDED_DEFLECTION (L"FLAPS_COMMANDED_DEFLECTION");
		std::wstring SPOILERS_COMMANDED_DEFLECTION (L"SPOILERS_COMMANDED_DEFLECTION");
		std::wstring STABILIZER_COMMANDED_DEFLECTION (L"STABILIZER_COMMANDED_DEFLECTION");
		std::wstring GEARS_COMMANDED_POSITION (L"GEARS_COMMANDED_POSITION");
		
		_FLIGHT_CONTROLS_ClassHandle = _RtiAmb->getObjectClassHandle(FLIGHT_CONTROLS); 
        _RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, RIGHT_AILERON_COMMANDED_DEFLECTION);
        _LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, LEFT_AILERON_COMMANDED_DEFLECTION);
        _RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, RIGHT_ELEVATOR_COMMANDED_DEFLECTION);
        _LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, LEFT_ELEVATOR_COMMANDED_DEFLECTION);
        _RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, RUDDER_COMMANDED_DEFLECTION);
        _RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, RIGHT_ENGINE_COMMANDED_THROTTLE);
        _LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, LEFT_ENGINE_COMMANDED_THROTTLE);
        _FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, FLAPS_COMMANDED_DEFLECTION);
        _SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, SPOILERS_COMMANDED_DEFLECTION);
        _STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, STABILIZER_COMMANDED_DEFLECTION);
        _GEARS_COMMANDED_POSITION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_FLIGHT_CONTROLS_ClassHandle, GEARS_COMMANDED_POSITION); 
        
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
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void DataLoggerFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		attr_FLIGHT_CONTROLS.insert(_RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_GEARS_COMMANDED_POSITION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE);

        _RtiAmb->subscribeObjectClassAttributes( _FLIGHT_CONTROLS_ClassHandle,attr_FLIGHT_CONTROLS);

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
        
        //attr_TRIM_DATA.reset(RTI::AttributeHandleSetFactory::create(4));
		attr_TRIM_DATA.insert(_RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE);
		attr_TRIM_DATA.insert(_LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE);
		attr_TRIM_DATA.insert(_ELEVATOR_DEFLECTION_EQ_ATTRIBUTE);
		attr_TRIM_DATA.insert(_STABILIZER_DEFLECTION_EQ_ATTRIBUTE);
	  
		//attr_JOYSTICK.reset(RTI::AttributeHandleSetFactory::create(9));
		attr_JOYSTICK.insert(_AILERON_ATTRIBUTE);
		attr_JOYSTICK.insert(_ELEVATOR_ATTRIBUTE);
		attr_JOYSTICK.insert(_RUDDER_ATTRIBUTE);
		attr_JOYSTICK.insert(_THROTTLE_LEFT_ATTRIBUTE);
		attr_JOYSTICK.insert(_THROTTLE_RIGHT_ATTRIBUTE);
		attr_JOYSTICK.insert(_FLAPS_ATTRIBUTE);
		attr_JOYSTICK.insert(_SPOILERS_ATTRIBUTE);
		attr_JOYSTICK.insert(_GEARS_ATTRIBUTE);
		attr_JOYSTICK.insert(_BRAKES_ATTRIBUTE);
		
		//attr_COCKPIT.reset(RTI::AttributeHandleSetFactory::create(6)) ;    
		attr_COCKPIT.insert(_AUTOPILOT_AP_ACTIVE_ATTRIBUTE);
		attr_COCKPIT.insert(_AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE);
		attr_COCKPIT.insert(_AUTOPILOT_SPD_ATTRIBUTE);
		attr_COCKPIT.insert(_AUTOPILOT_HDG_ATTRIBUTE);
		attr_COCKPIT.insert(_AUTOPILOT_ALT_ATTRIBUTE);
		attr_COCKPIT.insert(_AUTOPILOT_VS_ATTRIBUTE);

		//attr_MEAS_AIRCRAFT_POSITION.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		//attr_MEAS_AIRCRAFT_ORIENTATION.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		//attr_MEAS_AIRCRAFT_UVW_SPEED.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		//attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		//attr_MEAS_AIRCRAFT_ACCELERATION.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		//attr_MEAS_AIRCRAFT_SPEED.reset(RTI::AttributeHandleSetFactory::create(7)) ;
		//attr_MEAS_AIRCRAFT_ADDITIONAL.reset(RTI::AttributeHandleSetFactory::create(4)) ;
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
		
        _RtiAmb->subscribeObjectClassAttributes( _JOYSTICK_ClassHandle,attr_JOYSTICK);
        _RtiAmb->subscribeObjectClassAttributes(_COCKPIT_ClassHandle,attr_COCKPIT);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_POSITION_ClassHandle, attr_MEAS_AIRCRAFT_POSITION);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle, attr_MEAS_AIRCRAFT_ORIENTATION);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle, attr_MEAS_AIRCRAFT_UVW_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle, attr_MEAS_AIRCRAFT_ACCELERATION);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_SPEED_ClassHandle, attr_MEAS_AIRCRAFT_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle, attr_MEAS_AIRCRAFT_ADDITIONAL); 
        
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
		
        _RtiAmb->subscribeObjectClassAttributes(_ENVIRONMENT_VARIABLES_ClassHandle,attr_ENVIRONMENT_VARIABLES);
        _RtiAmb->subscribeObjectClassAttributes(_WIND_COMPONENTS_ClassHandle,attr_WIND_COMPONENTS);
        
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
		_RtiAmb->subscribeObjectClassAttributes(_ACTUATORS_ClassHandle,attr_ACTUATORS);	
        
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void DataLoggerFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	//Not used
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void DataLoggerFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->unsubscribeObjectClass(_COCKPIT_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_JOYSTICK_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_FLIGHT_CONTROLS_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_ACTUATORS_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_AIRCRAFT_ADDITIONAL_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_ENVIRONMENT_VARIABLES_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_WIND_COMPONENTS_ClassHandle);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
       
    #ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void DataLoggerFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while (//!_Discov_JOYSTICK       			||
		   !_Discov_FLIGHT_CONTROLS       		||
		   !_Discov_ACTUATORS_CONTROL_SURFACES     	||
		   !_Discov_ACTUATORS_ENGINES     		||
		   !_Discov_AIRCRAFT_POSITION       		||
		   !_Discov_AIRCRAFT_ORIENTATION       		||
		   !_Discov_AIRCRAFT_UVW_SPEED         		||
		   !_Discov_AIRCRAFT_PQR_ANGULAR_SPEED 		||
		   !_Discov_AIRCRAFT_ACCELERATION      		||
		   !_Discov_AIRCRAFT_SPEED             		||
		   !_Discov_AIRCRAFT_ADDITIONAL       		||
		   !_Discov_MEAS_AIRCRAFT_POSITION       		||
		   !_Discov_MEAS_AIRCRAFT_ORIENTATION       	||
		   !_Discov_MEAS_AIRCRAFT_UVW_SPEED         	||
		   !_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED 	||
		   !_Discov_MEAS_AIRCRAFT_ACCELERATION      	||
		   !_Discov_MEAS_AIRCRAFT_SPEED             	||
		   !_Discov_MEAS_AIRCRAFT_ADDITIONAL      	||
		   !_Discov_ENVIRONMENT_VARIABLES             	||
		   !_Discov_WIND_COMPONENTS)
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
		
	_Discov_JOYSTICK = false;
	_Discov_ACTUATORS_CONTROL_SURFACES = false;
    _Discov_ACTUATORS_ENGINES = false;
    _Discov_FLIGHT_CONTROLS = false;
    _Discov_ENVIRONMENT_VARIABLES = false;
    _Discov_WIND_COMPONENTS = false;
    _Discov_MEAS_AIRCRAFT_POSITION = false;
	_Discov_MEAS_AIRCRAFT_ORIENTATION = false;
	_Discov_MEAS_AIRCRAFT_UVW_SPEED = false;
	_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ACCELERATION = false;
	_Discov_MEAS_AIRCRAFT_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ADDITIONAL = false;		
	_Discov_AIRCRAFT_POSITION = false;
	_Discov_AIRCRAFT_ORIENTATION = false;
	_Discov_AIRCRAFT_UVW_SPEED = false;
	_Discov_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_AIRCRAFT_ACCELERATION = false;
	_Discov_AIRCRAFT_SPEED = false;
	_Discov_AIRCRAFT_ADDITIONAL = false;
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void DataLoggerFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_MEAS_AIRCRAFT_POSITION          ||
           !_New_MEAS_AIRCRAFT_ORIENTATION       ||
           !_New_MEAS_AIRCRAFT_UVW_SPEED         ||
           !_New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED ||
           !_New_MEAS_AIRCRAFT_ACCELERATION      ||
           !_New_MEAS_AIRCRAFT_SPEED             ||
           !_New_MEAS_AIRCRAFT_ADDITIONAL        ||
           !_New_AIRCRAFT_POSITION          ||
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
    
    _New_MEAS_AIRCRAFT_POSITION = false;
    _New_MEAS_AIRCRAFT_ORIENTATION = false;
    _New_MEAS_AIRCRAFT_UVW_SPEED = false;
    _New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
    _New_MEAS_AIRCRAFT_ACCELERATION = false;
    _New_MEAS_AIRCRAFT_SPEED = false;
    _New_MEAS_AIRCRAFT_ADDITIONAL = false;
    _New_AIRCRAFT_POSITION = false;
    _New_AIRCRAFT_ORIENTATION = false;
    _New_AIRCRAFT_UVW_SPEED = false;
    _New_AIRCRAFT_PQR_ANGULAR_SPEED = false;
    _New_AIRCRAFT_ACCELERATION = false;
    _New_AIRCRAFT_SPEED = false;
    _New_AIRCRAFT_ADDITIONAL = false;
    #ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void DataLoggerFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
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
    
    std::wstring Meas_Aircraft_Position (L"Meas_Aircraft_Position");
	std::wstring Meas_Aircraft_Orientation (L"Meas_Aircraft_Orientation");
    std::wstring Meas_Aircraft_UVW_Speed (L"Meas_Aircraft_UVW_Speed");
    std::wstring Meas_Aircraft_PQR_Angular_Speed (L"Meas_Aircraft_PQR_Angular_Speed");
    std::wstring Meas_Aircraft_Acceleration (L"Meas_Aircraft_Acceleration");
	std::wstring Meas_Aircraft_Speed (L"Meas_Aircraft_Speed");
    std::wstring Meas_Aircraft_Additional (L"Meas_Aircraft_Additional");
    
    std::wstring Flight_Controls (L"Flight_Controls");
    std::wstring Environment_Variables (L"Environment_Variables");
    std::wstring Wind_Components (L"Wind_Components");
    
	if ( (theObjectClass == _ACTUATORS_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Actuators_Control_Surfaces.c_str())) ) 
    {
        _Discov_ACTUATORS_CONTROL_SURFACES = true;
        _ACTUATORS_CONTROL_SURFACES_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        std::wcout << L"< Actuators_Control_Surfaces > Object Instance has been discovered with handle "
             << _ACTUATORS_CONTROL_SURFACES_ObjectHandle
             << "."
             << std::endl;
        #endif
    }
    else if ( (theObjectClass == _ACTUATORS_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Actuators_Engines.c_str())) ) 
    {
         _Discov_ACTUATORS_ENGINES = true;
        _ACTUATORS_ENGINES_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        std::wcout << L"< Actuators_Engines > Object Instance has been discovered with handle "
             << _ACTUATORS_ENGINES_ObjectHandle
             << "."
             << std::endl;
        #endif
    }
    else if ( (theObjectClass == _AIRCRAFT_POSITION_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Aircraft_Position.c_str())) ) 
    {
        _Discov_AIRCRAFT_POSITION = true;
        _AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
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
        #ifdef DEBUG_DATA_LOGGER_FED
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
        #ifdef DEBUG_DATA_LOGGER_FED
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
        #ifdef DEBUG_DATA_LOGGER_FED
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
        #ifdef DEBUG_DATA_LOGGER_FED
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
        #ifdef DEBUG_DATA_LOGGER_FED
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
        #ifdef DEBUG_DATA_LOGGER_FED
        std::wcout << L"< Aircraft_Additional > Object Instance has been discovered with handle "
             << _AIRCRAFT_ADDITIONAL_ObjectHandle
             << "."
             << std::endl;
        #endif
    }
    else if ( (theObjectClass == _MEAS_AIRCRAFT_POSITION_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_Position.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_POSITION = true;
        _MEAS_AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
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
        #ifdef DEBUG_DATA_LOGGER_FED
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
        #ifdef DEBUG_DATA_LOGGER_FED
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
        #ifdef DEBUG_DATA_LOGGER_FED
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
        #ifdef DEBUG_DATA_LOGGER_FED
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
        #ifdef DEBUG_DATA_LOGGER_FED
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
        #ifdef DEBUG_DATA_LOGGER_FED
        std::wcout << L"< Meas_Aircraft_Additional > Object Instance has been discovered with handle "
             << _MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle
             << "."
             << std::endl;
        #endif
    }    
    else if ( (theObjectClass == _FLIGHT_CONTROLS_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Flight_Controls.c_str())) ) 
    {
        _Discov_FLIGHT_CONTROLS = true;
        _FLIGHT_CONTROLS_ObjectHandle = theObject;
        #ifdef DEBUG_ENGINES_FED
        std::wcout << L"< Flight_Controls > Object Instance has been discovered" << std::endl;
        #endif
    } 
	else if ( (theObjectClass == _ENVIRONMENT_VARIABLES_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Environment_Variables.c_str())) ) 
    {
       _Discov_ENVIRONMENT_VARIABLES = true;
        _ENVIRONMENT_VARIABLES_ObjectHandle = theObject;
        #ifdef DEBUG_DATA_LOGGER_FED
        std::wcout << L"< Environment_Variables > Object Instance has been discovered with handle "
             << _ENVIRONMENT_VARIABLES_ObjectHandle
             << "."
             << std::endl;
        #endif
    }  
    else if ( (theObjectClass == _WIND_COMPONENTS_ClassHandle) &&  (!wcscmp(theObjectInstanceName.c_str(),Wind_Components.c_str())) ) 
	{
		_Discov_WIND_COMPONENTS = true;
		_WIND_COMPONENTS_ObjectHandle = theObject;
		#ifdef DEBUG_FLIGH_DYNAMICS_FED
		std::wcout << L"Wind_Components > Object Instance has been discovered" << std::endl;
		#endif
	}  
    #ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> discoverObjectInstance(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void DataLoggerFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
    #endif
	std::wstring TrimString (L"Trimming");
    if (_IsCreator) 
    {
		std::wcout << L">> PRESS ENTER TO START TRIMMING " << endl;      
        std::cin.get();
		// Simulating synchronization starts with an action of the user (on Creator interface)
        std::wcout << L"Pause requested per Creator " << std::endl;
		// Trimming synchronization starts automatically (Creator)
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
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void DataLoggerFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
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
	#ifdef DEBUG_DATA_LOGGER_FED
    std::wcout << L"DataLoggerFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void DataLoggerFederateHla1516e::initialization() 
{
	waitForAllObjectDiscovered();
	//waitForAllAttributesReceived();
	string path = "../logs/" + _FileName;	
	_SdseDataLog.open (path);
	_SdseDataLog.clear();
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void DataLoggerFederateHla1516e::calculateState() 
{
// Models State calculation
	_SdseDataLog 	<< _LocalTime.getFedTime()// 1
	<< "," << _DataLogger.GET_JOYSTICK_AILERON() // 2
	<< "," << _DataLogger.GET_JOYSTICK_ELEVATOR() // 3
	<< "," << _DataLogger.GET_JOYSTICK_RUDDER() // 4
	<< "," << _DataLogger.GET_JOYSTICK_THROTTLE_LEFT() // 5
	<< "," << _DataLogger.GET_JOYSTICK_THROTTLE_RIGHT() // 6
    << "," << _DataLogger.GET_JOYSTICK_FLAPS() // 7
    << "," << _DataLogger.GET_JOYSTICK_SPOILERS() // 8
    << "," << _DataLogger.GET_JOYSTICK_GEARS() // 9
    << "," << _DataLogger.GET_JOYSTICK_BRAKES() // 10
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_RIGHT_AILERON_COMMANDED_DEFLECTION() // 11
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_LEFT_AILERON_COMMANDED_DEFLECTION() // 12
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_RIGHT_ELEVATOR_COMMANDED_DEFLECTION() // 13
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_LEFT_ELEVATOR_COMMANDED_DEFLECTION() // 14
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_RUDDER_COMMANDED_DEFLECTION() // 15
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_RIGHT_ENGINE_COMMANDED_THROTTLE() // 16
	<< "," << _DataLogger.GET_FLIGHT_CONTROLS_LEFT_ENGINE_COMMANDED_THROTTLE() // 17
    << "," << _DataLogger.GET_FLIGHT_CONTROLS_FLAPS_COMMANDED_DEFLECTION() // 18
    << "," << _DataLogger.GET_FLIGHT_CONTROLS_SPOILERS_COMMANDED_DEFLECTION() // 19
    << "," << _DataLogger.GET_FLIGHT_CONTROLS_STABILIZER_COMMANDED_DEFLECTION() // 20
    << "," << _DataLogger.GET_FLIGHT_CONTROLS_GEARS_COMMANDED_POSITION() // 21
	<< "," << _DataLogger.GET_ACTUATORS_RIGHT_AILERON_EFFECTIVE_DEFLECTION() // 22
	<< "," << _DataLogger.GET_ACTUATORS_LEFT_AILERON_EFFECTIVE_DEFLECTION() // 23
	<< "," << _DataLogger.GET_ACTUATORS_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION() // 24
	<< "," << _DataLogger.GET_ACTUATORS_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION() // 25
	<< "," << _DataLogger.GET_ACTUATORS_RUDDER_EFFECTIVE_DEFLECTION() // 26
	<< "," << _DataLogger.GET_ACTUATORS_RIGHT_ENGINE_THRUST() // 27
	<< "," << _DataLogger.GET_ACTUATORS_LEFT_ENGINE_THRUST() // 28
    << "," << _DataLogger.GET_ACTUATORS_FLAPS_EFFECTIVE_DEFLECTION() // 29
    << "," << _DataLogger.GET_ACTUATORS_SPOILERS_EFFECTIVE_DEFLECTION() // 30
    << "," << _DataLogger.GET_ACTUATORS_STABILIZER_EFFECTIVE_DEFLECTION() // 31
    << "," << _DataLogger.GET_ACTUATORS_GEARS_POSITION() // 32
	<< "," << _DataLogger.GET_AIRCRAFT_POSITION_LONGITUDE() // 33
	<< "," << _DataLogger.GET_AIRCRAFT_POSITION_LATITUDE() // 34
	<< "," << _DataLogger.GET_AIRCRAFT_POSITION_ALTITUDE() // 35
	<< "," << _DataLogger.GET_AIRCRAFT_ORIENTATION_PHI() // 36
	<< "," << _DataLogger.GET_AIRCRAFT_ORIENTATION_THETA() // 37
	<< "," << _DataLogger.GET_AIRCRAFT_ORIENTATION_PSI() // 38
	<< "," << _DataLogger.GET_AIRCRAFT_UVW_SPEED_U_SPEED() // 39
	<< "," << _DataLogger.GET_AIRCRAFT_UVW_SPEED_V_SPEED() // 40
	<< "," << _DataLogger.GET_AIRCRAFT_UVW_SPEED_W_SPEED() // 41
	<< "," << _DataLogger.GET_AIRCRAFT_PQR_ANGULAR_SPEED_P_ANG_SPEED() // 42
	<< "," << _DataLogger.GET_AIRCRAFT_PQR_ANGULAR_SPEED_Q_ANG_SPEED() // 43
	<< "," << _DataLogger.GET_AIRCRAFT_PQR_ANGULAR_SPEED_R_ANG_SPEED() // 44
	<< "," << _DataLogger.GET_AIRCRAFT_ACCELERATION_X_ACC() // 45
	<< "," << _DataLogger.GET_AIRCRAFT_ACCELERATION_Y_ACC() // 46
	<< "," << _DataLogger.GET_AIRCRAFT_ACCELERATION_Z_ACC() // 47
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_INDICATED_AIRSPEED() // 48
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_EQUIVALENT_AIRSPEED() // 49
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_CALIBRATED_AIRSPEED() // 50
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_TRUE_AIRSPEED() // 51
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_GROUND_SPEED() // 52
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_VERTICAL_SPEED() // 53
	<< "," << _DataLogger.GET_AIRCRAFT_SPEED_MACH_NUMBER() // 54
	<< "," << _DataLogger.GET_AIRCRAFT_ADDITIONAL_ALPHA() // 55
	<< "," << _DataLogger.GET_AIRCRAFT_ADDITIONAL_BETA() // 56
	<< "," << _DataLogger.GET_AIRCRAFT_ADDITIONAL_DYNAMIC_PRESSURE() // 57
    << "," << _DataLogger.GET_AIRCRAFT_ADDITIONAL_TANK_FILLING() // 58
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_POSITION_MEAS_LONGITUDE() // 59
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_POSITION_MEAS_LATITUDE() // 60
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_POSITION_MEAS_ALTITUDE() // 61
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ORIENTATION_MEAS_PHI() // 62
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ORIENTATION_MEAS_THETA() // 63
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ORIENTATION_MEAS_PSI() // 64
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_UVW_SPEED_MEAS_U_SPEED() // 65
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_UVW_SPEED_MEAS_V_SPEED() // 66
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_UVW_SPEED_MEAS_W_SPEED() // 67
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_MEAS_P_ANG_SPEED() // 68
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_MEAS_Q_ANG_SPEED() // 69
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_MEAS_R_ANG_SPEED() // 70
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ACCELERATION_MEAS_X_ACC() // 71
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ACCELERATION_MEAS_Y_ACC() // 72
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ACCELERATION_MEAS_Z_ACC() // 73
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_INDICATED_AIRSPEED() // 74
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_EQUIVALENT_AIRSPEED() // 75
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_CALIBRATED_AIRSPEED() // 76
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_TRUE_AIRSPEED() // 77
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_GROUND_SPEED() // 78
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_VERTICAL_SPEED() // 79
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_SPEED_MEAS_MACH_NUMBER() // 80
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_ALPHA() // 81
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_BETA() // 82
	<< "," << _DataLogger.GET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_DYNAMIC_PRESSURE() // 83
    << "," << _DataLogger.GET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_TANK_FILLING() // 84
	<< "," << _DataLogger.GET_ENVIRONMENT_VARIABLES_TEMPERATURE() // 85
	<< "," << _DataLogger.GET_ENVIRONMENT_VARIABLES_DENSITY_OF_AIR() // 86
	<< "," << _DataLogger.GET_ENVIRONMENT_VARIABLES_PRESSURE() // 87
	<< "," << _DataLogger.GET_ENVIRONMENT_VARIABLES_SPEED_OF_SOUND() // 88
	<< "," << _DataLogger.GET_WIND_COMPONENTS_U_WIND() // 89
	<< "," << _DataLogger.GET_WIND_COMPONENTS_V_WIND()	// 90
	<< "," << _DataLogger.GET_WIND_COMPONENTS_W_WIND() // 91
	<< "," << _DataLogger.GET_WIND_COMPONENTS_P_WIND() // 92
	<< "," << _DataLogger.GET_WIND_COMPONENTS_Q_WIND() // 93
	<< "," << _DataLogger.GET_WIND_COMPONENTS_R_WIND()//94
	<< "," << _TotalRealTimeMs//95
	<< "," << _TimeStep.getFedTime()//96
	<< "," << ((_ExecutionTime.tv_nsec) / 1000000)//97
	<< '\n';
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void DataLoggerFederateHla1516e::calculateOutput() 
{
	// Not used
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void DataLoggerFederateHla1516e::sendUpdateAttributes(RTI1516fedTime UpdateTime)
{
	// Nothing to do   
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void DataLoggerFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
void DataLoggerFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
	
	if (theObject == _JOYSTICK_ObjectHandle)
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
            if      (parmHandle == _AILERON_ATTRIBUTE) 		
            { 
				_DataLogger.SET_JOYSTICK_AILERON(buffer.read_double()); 
			}
            else if (parmHandle == _ELEVATOR_ATTRIBUTE) 
            { 
				_DataLogger.SET_JOYSTICK_ELEVATOR(buffer.read_double());
			}
            else if (parmHandle == _RUDDER_ATTRIBUTE)  		
            { 
				_DataLogger.SET_JOYSTICK_RUDDER(buffer.read_double()); 
			}
            else if (parmHandle == _THROTTLE_LEFT_ATTRIBUTE)  	
            { 
				_DataLogger.SET_JOYSTICK_THROTTLE_LEFT(buffer.read_double()); 
			}
            else if (parmHandle == _THROTTLE_RIGHT_ATTRIBUTE) 
            { 
				_DataLogger.SET_JOYSTICK_THROTTLE_RIGHT(buffer.read_double());  
			}
            else if (parmHandle == _FLAPS_ATTRIBUTE)  	
            { 
				_DataLogger.SET_JOYSTICK_FLAPS(buffer.read_double());
			}
            else if (parmHandle == _SPOILERS_ATTRIBUTE)  	
            { 
				_DataLogger.SET_JOYSTICK_SPOILERS(buffer.read_double()); 
			}
            else if (parmHandle == _GEARS_ATTRIBUTE)  	
            { 
				_DataLogger.SET_JOYSTICK_GEARS(buffer.read_double());
			}
            else if (parmHandle == _BRAKES_ATTRIBUTE)
            { 
				_DataLogger.SET_JOYSTICK_BRAKES(buffer.read_double());
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_JOYSTICK = true;
    }
	else if (theObject == _COCKPIT_ObjectHandle)
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

            if      (parmHandle == _AUTOPILOT_AP_ACTIVE_ATTRIBUTE)   	
            { 
				//buffer.read_double();
			}
            else if (parmHandle == _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE)   	
            { 
				//buffer.read_double();
			}
            else if (parmHandle == _AUTOPILOT_SPD_ATTRIBUTE)   			
            { 
				//buffer.read_double();
			}
            else if (parmHandle == _AUTOPILOT_HDG_ATTRIBUTE)   			
            { 
				//buffer.read_double();
			}
            else if (parmHandle == _AUTOPILOT_ALT_ATTRIBUTE)   			
            { 
				//buffer.read_double();
			}
            else if (parmHandle == _AUTOPILOT_VS_ATTRIBUTE)   			
            {
				//buffer.read_double();
			 }
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_COCKPIT = true;
    } 
    else if (theObject == _FLIGHT_CONTROLS_ObjectHandle)
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
				_DataLogger.SET_FLIGHT_CONTROLS_RIGHT_AILERON_COMMANDED_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE) 
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_LEFT_AILERON_COMMANDED_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_RIGHT_ELEVATOR_COMMANDED_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_LEFT_ELEVATOR_COMMANDED_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_RUDDER_COMMANDED_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_RIGHT_ENGINE_COMMANDED_THROTTLE(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_LEFT_ENGINE_COMMANDED_THROTTLE(buffer.read_double()); 
			}
            else if (parmHandle == _FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_FLAPS_COMMANDED_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE)
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_SPOILERS_COMMANDED_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE) 
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_STABILIZER_COMMANDED_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _GEARS_COMMANDED_POSITION_ATTRIBUTE) 
            { 
				_DataLogger.SET_FLIGHT_CONTROLS_GEARS_COMMANDED_POSITION(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}

        }    
        _New_FLIGHT_CONTROLS = true;
    }
    else     if (theObject == _ACTUATORS_CONTROL_SURFACES_ObjectHandle)
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
				_DataLogger.SET_ACTUATORS_RIGHT_AILERON_EFFECTIVE_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ACTUATORS_LEFT_AILERON_EFFECTIVE_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ACTUATORS_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE) 
            { 
				_DataLogger.SET_ACTUATORS_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE)  			
            { 
				_DataLogger.SET_ACTUATORS_RUDDER_EFFECTIVE_DEFLECTION(buffer.read_double()); 
			}
            else if (parmHandle == _FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE) 
            { 
				_DataLogger.SET_ACTUATORS_FLAPS_EFFECTIVE_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE)  	
            { 
				_DataLogger.SET_ACTUATORS_SPOILERS_EFFECTIVE_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ACTUATORS_STABILIZER_EFFECTIVE_DEFLECTION(buffer.read_double());
			}
            else if (parmHandle == _GEARS_POSITION_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ACTUATORS_GEARS_POSITION(buffer.read_double());
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_ACTUATORS_RIGHT_ENGINE_THRUST(buffer.read_double());
			}
            else if (parmHandle == _LEFT_ENGINE_THRUST_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ACTUATORS_LEFT_ENGINE_THRUST(buffer.read_double());
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
		}
        _New_ACTUATORS_ENGINES = true;
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
            if      (parmHandle == _LONGITUDE_ATTRIBUTE) 		
            { 
				_DataLogger.SET_AIRCRAFT_POSITION_LONGITUDE(buffer.read_double());
			}
            else if (parmHandle == _LATITUDE_ATTRIBUTE)
            { 
				_DataLogger.SET_AIRCRAFT_POSITION_LATITUDE(buffer.read_double()); 
			}
            else if (parmHandle == _ALTITUDE_ATTRIBUTE)
            { 
				_DataLogger.SET_AIRCRAFT_POSITION_ALTITUDE(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_AIRCRAFT_ORIENTATION_PHI(buffer.read_double()); 
			}
            else if (parmHandle == _THETA_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_ORIENTATION_THETA(buffer.read_double());
			}
            else if (parmHandle == _PSI_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_ORIENTATION_PSI(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_AIRCRAFT_UVW_SPEED_U_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _V_SPEED_ATTRIBUTE)  	
            { 
				_DataLogger.SET_AIRCRAFT_UVW_SPEED_V_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _W_SPEED_ATTRIBUTE)  	
            { 
				_DataLogger.SET_AIRCRAFT_UVW_SPEED_W_SPEED(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_AIRCRAFT_PQR_ANGULAR_SPEED_P_ANG_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _Q_ANG_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_PQR_ANGULAR_SPEED_Q_ANG_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _R_ANG_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_PQR_ANGULAR_SPEED_R_ANG_SPEED(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_AIRCRAFT_ACCELERATION_X_ACC(buffer.read_double()); 
			}
            else if (parmHandle == _Y_ACC_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_ACCELERATION_Y_ACC(buffer.read_double()); 
			}
            else if (parmHandle == _Z_ACC_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_ACCELERATION_Z_ACC(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_AIRCRAFT_SPEED_INDICATED_AIRSPEED(buffer.read_double());
			}
            else if (parmHandle == _EQUIVALENT_AIRSPEED_ATTRIBUTE) 
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_EQUIVALENT_AIRSPEED(buffer.read_double());
			}
            else if (parmHandle == _CALIBRATED_AIRSPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_CALIBRATED_AIRSPEED(buffer.read_double());
			}
            else if (parmHandle == _TRUE_AIRSPEED_ATTRIBUTE)  			
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_TRUE_AIRSPEED(buffer.read_double());
			}
            else if (parmHandle == _GROUND_SPEED_ATTRIBUTE)  			
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_GROUND_SPEED(buffer.read_double());
			}
            else if (parmHandle == _VERTICAL_SPEED_ATTRIBUTE)  			
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_VERTICAL_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _MACH_NUMBER_ATTRIBUTE)  			
            { 
				_DataLogger.SET_AIRCRAFT_SPEED_MACH_NUMBER(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_AIRCRAFT_ADDITIONAL_ALPHA(buffer.read_double());  
			}
            else if (parmHandle == _BETA_ATTRIBUTE)  				
            { 
				_DataLogger.SET_AIRCRAFT_ADDITIONAL_BETA(buffer.read_double()); 
			}
            else if (parmHandle == _DYNAMIC_PRESSURE_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_ADDITIONAL_DYNAMIC_PRESSURE(buffer.read_double()); 
			}
            else if (parmHandle == _TANK_FILLING_ATTRIBUTE)  		
            { 
				_DataLogger.SET_AIRCRAFT_ADDITIONAL_TANK_FILLING(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_AIRCRAFT_ADDITIONAL = true;
    } 
   else if (theObject == _MEAS_AIRCRAFT_POSITION_ObjectHandle){
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
				_DataLogger.SET_MEAS_AIRCRAFT_POSITION_MEAS_LONGITUDE(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_LATITUDE_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_POSITION_MEAS_LATITUDE(buffer.read_double());
			}
            else if (parmHandle == _MEAS_ALTITUDE_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_POSITION_MEAS_ALTITUDE(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_MEAS_AIRCRAFT_ORIENTATION_MEAS_PHI(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_THETA_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ORIENTATION_MEAS_THETA(buffer.read_double());  
			}
            else if (parmHandle == _MEAS_PSI_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ORIENTATION_MEAS_PSI(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_MEAS_AIRCRAFT_UVW_SPEED_MEAS_U_SPEED(buffer.read_double());
			}
            else if (parmHandle == _MEAS_V_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_UVW_SPEED_MEAS_V_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_W_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_UVW_SPEED_MEAS_W_SPEED(buffer.read_double());  
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_MEAS_P_ANG_SPEED(buffer.read_double());  
			}
            else if (parmHandle == _MEAS_R_ANG_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_MEAS_R_ANG_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_Q_ANG_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_MEAS_Q_ANG_SPEED(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_MEAS_AIRCRAFT_ACCELERATION_MEAS_X_ACC(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_Y_ACC_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ACCELERATION_MEAS_Y_ACC(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_Z_ACC_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ACCELERATION_MEAS_Z_ACC(buffer.read_double());
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_INDICATED_AIRSPEED(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE)  	
            {
				 _DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_EQUIVALENT_AIRSPEED(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE)  	
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_CALIBRATED_AIRSPEED(buffer.read_double());
			}
            else if (parmHandle == _MEAS_TRUE_AIRSPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_TRUE_AIRSPEED(buffer.read_double());  
			}
            else if (parmHandle == _MEAS_GROUND_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_GROUND_SPEED(buffer.read_double());
			}
            else if (parmHandle == _MEAS_VERTICAL_SPEED_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_VERTICAL_SPEED(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_MACH_NUMBER_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_SPEED_MEAS_MACH_NUMBER(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_DataLogger.SET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_ALPHA(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_BETA_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_BETA(buffer.read_double()); 
			}
            else if (parmHandle == _MEAS_DYNAMIC_PRESSURE_ATTRIBUTE)  	
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_DYNAMIC_PRESSURE(buffer.read_double());
			}
            else if (parmHandle == _MEAS_TANK_FILLING_ATTRIBUTE)  		
            { 
				_DataLogger.SET_MEAS_AIRCRAFT_ADDITIONAL_MEAS_TANK_FILLING(buffer.read_double());
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_MEAS_AIRCRAFT_ADDITIONAL = true;
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
            if      (parmHandle == _TEMPERATURE_ATTRIBUTE) 		
            { 
				_DataLogger.SET_ENVIRONMENT_VARIABLES_TEMPERATURE(buffer.read_double()); 
			}
            else if (parmHandle == _DENSITY_OF_AIR_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ENVIRONMENT_VARIABLES_DENSITY_OF_AIR(buffer.read_double()); 
			}
            else if (parmHandle == _PRESSURE_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ENVIRONMENT_VARIABLES_PRESSURE(buffer.read_double()); 
			}
            else if (parmHandle == _SPEED_OF_SOUND_ATTRIBUTE)  		
            { 
				_DataLogger.SET_ENVIRONMENT_VARIABLES_SPEED_OF_SOUND(buffer.read_double());
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl;  
			}      
        }    
        _New_ENVIRONMENT_VARIABLES = true;
    } 
    else if (theObject == _WIND_COMPONENTS_ObjectHandle)
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
            if      (parmHandle == _U_WIND_ATTRIBUTE) 		
            { 
				_DataLogger.SET_WIND_COMPONENTS_U_WIND(buffer.read_double()); 
			}
            else if (parmHandle == _V_WIND_ATTRIBUTE)  		
            { 
				_DataLogger.SET_WIND_COMPONENTS_V_WIND(buffer.read_double());
			}
            else if (parmHandle == _W_WIND_ATTRIBUTE)  		
            { 
				_DataLogger.SET_WIND_COMPONENTS_W_WIND(buffer.read_double());
			}
            else if (parmHandle == _P_WIND_ATTRIBUTE)  		
            { 
				_DataLogger.SET_WIND_COMPONENTS_P_WIND(buffer.read_double()); 
			}
            else if (parmHandle == _Q_WIND_ATTRIBUTE)  		
            { 
				_DataLogger.SET_WIND_COMPONENTS_Q_WIND(buffer.read_double()); 
			}
            else if (parmHandle == _R_WIND_ATTRIBUTE)  		
            { 
				_DataLogger.SET_WIND_COMPONENTS_R_WIND(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_WIND_COMPONENTS = true;
    } 
	else
	{ 
		std::wcout << L"DataLoggerFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << std::endl; 
	} 
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void DataLoggerFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
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
void DataLoggerFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
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
void DataLoggerFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_DATA_LOGGER_FED
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void DataLoggerFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void DataLoggerFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
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
void DataLoggerFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
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
void DataLoggerFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
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
void DataLoggerFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
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
void DataLoggerFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void DataLoggerFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void DataLoggerFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void DataLoggerFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// function : run
void DataLoggerFederateHla1516e::run()
{
	RTI1516fedTime updateTime(0.0);
	while (_LocalTime < _SimulationEndTime)
	{
		clock_gettime(CLOCK_MONOTONIC, &_TimeStamp_old);
		// Model calculations
		calculateState();
		calculateOutput();
		timeAdvanceRequest(_TimeStep);
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
		_TotalRealTimeMs = _TotalRealTimeMs + ((_ExecutionTime.tv_nsec) / 1000000);
	}
	_SdseDataLog.close();
} 
