#include "EfcsFederateHla1516e.hh"

// ----------------------------------------------------------------------------
// TemplateFederateHla13 Constructor
EfcsFederateHla1516e::EfcsFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_EFCS_FED
	std::wcout << L"EfcsFederateHla1516e.cc -> Constructor(): Start" << std::endl;
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
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator =  _IsSyncAnnonced = false ;
	_IsOutMesTimespamped = true;
	
	_Discov_TRIM_DATA_ENGINES               = false;
	_Discov_TRIM_DATA_FLIGHT_DYNAMICS       = false;
	_Discov_JOYSTICK                        = false;
	_Discov_COCKPIT                         = false;
	_Discov_MEAS_AIRCRAFT_POSITION          = false;
	_Discov_MEAS_AIRCRAFT_ORIENTATION       = false;
	_Discov_MEAS_AIRCRAFT_UVW_SPEED         = false;
	_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ACCELERATION      = false;
	_Discov_MEAS_AIRCRAFT_SPEED             = false;
	_Discov_MEAS_AIRCRAFT_ADDITIONAL        = false;
	_New_TRIM_DATA_ENGINES                  = false;
	_New_TRIM_DATA_FLIGHT_DYNAMICS          = false;
	_New_JOYSTICK                           = false;
	_New_COCKPIT                            = false;
	_New_MEAS_AIRCRAFT_POSITION             = false;
	_New_MEAS_AIRCRAFT_ORIENTATION          = false;
	_New_MEAS_AIRCRAFT_UVW_SPEED            = false;
	_New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED    = false;
	_New_MEAS_AIRCRAFT_ACCELERATION         = false;
	_New_MEAS_AIRCRAFT_SPEED                = false;
	_New_MEAS_AIRCRAFT_ADDITIONAL           = false;
	
	// Set data structures to 0
	memset(&_BridgeData, 0, sizeof(bridge_data_t));
	memset(&_EfcsOutput, 0, sizeof(efcs_data_t));
	memset(&_TrimmingData, 0, sizeof(trimming_values_t));

	#ifdef DEBUG_EFCS_FED
	std::wcout << L"EfcsFederateHla1516e.cc -> Constructor(): End" << std::endl;
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
EfcsFederateHla1516e::~EfcsFederateHla1516e()
{
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void EfcsFederateHla1516e::handleExternalBridgeSetup() 
{
	#ifdef DEBUG_EFCS_FED
	std::wcout << L"EfcsFederateHla1516e.cc -> handleExternalBridgeSetup(): Start" << std::endl;
	#endif
	// Get values from xml file
	XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);
	// Get values from xml file
	vector<string> attribute_path(3);
	attribute_path[0] = "EFCS";
	attribute_path[1] = "Bridge_Status";
	attribute_path[2] = "Enabled";
	_IsExternalBridgeEnabled = (bool) getIntValue(params, attribute_path);
	attribute_path[1] = "Bridge_Net_Config";
	attribute_path[2] = "UDP_port";
	_BridgeUdpPort = getIntValue(params, attribute_path);
	attribute_path[2] = "Trimming_UDP_port";
	_BridgeTrimmingUdpPort = getIntValue(params, attribute_path);
	attribute_path[2] = "IP_Address";
	_BridgeIpAddress = getTextValue(params, attribute_path);
	attribute_path[1] = "Efcs_Net_Config";
	attribute_path[2] = "UDP_port";
	_ExternalEfcsUdpPort = getIntValue(params, attribute_path);
	attribute_path[2] = "IP_Address";
	_ExternalEfcsIpAdress = getTextValue(params, attribute_path);
	delete params;
	if (_IsExternalBridgeEnabled)
	{
		cout<<"External Bridge Enabled ! "<<endl;
		//set max size to max UDP frame PAYLOAD frame
		_InBuffer.resize(1472);
		_OutBuffer.resize(1472);
		// Output sockets
		_OuputLinkToExternalEfcs.constructSocket(_BridgeIpAddress, _BridgeUdpPort, _ExternalEfcsIpAdress, _ExternalEfcsUdpPort);
		_OuputLinkToExternalEfcsTrimming.constructSocket(_BridgeIpAddress, _BridgeTrimmingUdpPort, _ExternalEfcsIpAdress, _ExternalEfcsUdpPort);
		// Input socket
		_IncomingLinkFromExternalEfcs.constructSocket(_BridgeIpAddress, _BridgeUdpPort);
	}
    #ifdef DEBUG_EFCS_FED
	std::wcout << L"EfcsFederateHla1516e.cc -> handleExternalBridgeSetup(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void EfcsFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_EFCS_FED
	std::wcout << L"EfcsFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_EFCS_FED
	std::wcout << L"EfcsFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void EfcsFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_EFCS_FED
	std::wcout << L"EfcsFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_EFCS_FED
	std::wcout << L"EfcsFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EfcsFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_EFCS_FED
	std::wcout << L"EfcsFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
														 , L"efcs"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_EFCS_FED
	std::wcout << L"EfcsFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EfcsFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle EfcsFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void EfcsFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void EfcsFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring TRIM_DATA (L"TRIM_DATA");
		std::wstring JOYSTICK (L"JOYSTICK");
		std::wstring COCKPIT (L"COCKPIT");
		std::wstring MEAS_AIRCRAFT_POSITION (L"MEAS_AIRCRAFT_POSITION");
		std::wstring MEAS_AIRCRAFT_ORIENTATION (L"MEAS_AIRCRAFT_ORIENTATION");
		std::wstring MEAS_AIRCRAFT_UVW_SPEED (L"MEAS_AIRCRAFT_UVW_SPEED");
		std::wstring MEAS_AIRCRAFT_PQR_ANGULAR_SPEED (L"MEAS_AIRCRAFT_PQR_ANGULAR_SPEED");
		std::wstring MEAS_AIRCRAFT_ACCELERATION (L"MEAS_AIRCRAFT_ACCELERATION");
		std::wstring MEAS_AIRCRAFT_SPEED (L"MEAS_AIRCRAFT_SPEED");
		std::wstring MEAS_AIRCRAFT_ADDITIONAL (L"MEAS_AIRCRAFT_ADDITIONAL");
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
		
	
		// Subscribed
		_TRIM_DATA_ClassHandle = _RtiAmb->getObjectClassHandle(TRIM_DATA);     
		_JOYSTICK_ClassHandle = _RtiAmb->getObjectClassHandle(JOYSTICK);
		_COCKPIT_ClassHandle = _RtiAmb->getObjectClassHandle(COCKPIT);
		_MEAS_AIRCRAFT_POSITION_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_POSITION);
		_MEAS_AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_ORIENTATION);
		_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_UVW_SPEED);
		_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_PQR_ANGULAR_SPEED);
		_MEAS_AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_ACCELERATION);
		_MEAS_AIRCRAFT_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_SPEED);
		_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb->getObjectClassHandle(MEAS_AIRCRAFT_ADDITIONAL);
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
        // Published
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
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void EfcsFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
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
		
		_RtiAmb->subscribeObjectClassAttributes( _TRIM_DATA_ClassHandle,attr_TRIM_DATA); 
        _RtiAmb->subscribeObjectClassAttributes( _JOYSTICK_ClassHandle,attr_JOYSTICK);
        _RtiAmb->subscribeObjectClassAttributes(_COCKPIT_ClassHandle,attr_COCKPIT);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_POSITION_ClassHandle, attr_MEAS_AIRCRAFT_POSITION);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle, attr_MEAS_AIRCRAFT_ORIENTATION);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle, attr_MEAS_AIRCRAFT_UVW_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle, attr_MEAS_AIRCRAFT_ACCELERATION);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_SPEED_ClassHandle, attr_MEAS_AIRCRAFT_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle, attr_MEAS_AIRCRAFT_ADDITIONAL); 

		// For Class/Attributes published
		//attr_FLIGHT_CONTROLS.reset(RTI::AttributeHandleSetFactory::create(11)) ;   
		attr_FLIGHT_CONTROLS.insert(_RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS.insert(_GEARS_COMMANDED_POSITION_ATTRIBUTE);
		
        _RtiAmb->publishObjectClassAttributes(_FLIGHT_CONTROLS_ClassHandle,attr_FLIGHT_CONTROLS);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EfcsFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring Flight_Controls (L"Flight_Controls");
        _FLIGHT_CONTROLS_ObjectHandle = _RtiAmb->registerObjectInstance(_FLIGHT_CONTROLS_ClassHandle,Flight_Controls);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void EfcsFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb->unsubscribeObjectClass(_TRIM_DATA_ClassHandle);  
        _RtiAmb->unsubscribeObjectClass(_JOYSTICK_ClassHandle);
        _RtiAmb->unsubscribeObjectClass(_COCKPIT_ClassHandle);
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
    #ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EfcsFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_Discov_TRIM_DATA_ENGINES               ||
			!_Discov_TRIM_DATA_FLIGHT_DYNAMICS       ||
			!_Discov_JOYSTICK                        ||
			!_Discov_COCKPIT                         ||
			!_Discov_MEAS_AIRCRAFT_POSITION          ||
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
		
	_Discov_TRIM_DATA_ENGINES = false;
	_Discov_TRIM_DATA_FLIGHT_DYNAMICS = false;
	_Discov_JOYSTICK = false;
	_Discov_COCKPIT = false;
	_Discov_MEAS_AIRCRAFT_POSITION = false;
	_Discov_MEAS_AIRCRAFT_ORIENTATION = false;
	_Discov_MEAS_AIRCRAFT_UVW_SPEED = false;
	_Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ACCELERATION = false;
	_Discov_MEAS_AIRCRAFT_SPEED = false;
	_Discov_MEAS_AIRCRAFT_ADDITIONAL = false;
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EfcsFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_TRIM_DATA_ENGINES               ||
           !_New_TRIM_DATA_FLIGHT_DYNAMICS       ||
           !_New_JOYSTICK                        ||
           !_New_COCKPIT                         ||
           !_New_MEAS_AIRCRAFT_POSITION          ||
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
    
    _New_TRIM_DATA_ENGINES = false;
    _New_TRIM_DATA_FLIGHT_DYNAMICS = false;
    _New_JOYSTICK = false;
    _New_COCKPIT = false;
    _New_MEAS_AIRCRAFT_POSITION = false;
    _New_MEAS_AIRCRAFT_ORIENTATION = false;
    _New_MEAS_AIRCRAFT_UVW_SPEED = false;
    _New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = false;
    _New_MEAS_AIRCRAFT_ACCELERATION = false;
    _New_MEAS_AIRCRAFT_SPEED = false;
    _New_MEAS_AIRCRAFT_ADDITIONAL = false;
    #ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void EfcsFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    std::wstring Trim_Data_Engines (L"Trim_Data_Engines");
    std::wstring Trim_Data_Flight_Dynamics (L"Trim_Data_Flight_Dynamics");
    std::wstring Joystick (L"Joystick");
    std::wstring Cockpit (L"Cockpit");
	std::wstring Meas_Aircraft_Position (L"Meas_Aircraft_Position");
	std::wstring Meas_Aircraft_Orientation (L"Meas_Aircraft_Orientation");
    std::wstring Meas_Aircraft_UVW_Speed (L"Meas_Aircraft_UVW_Speed");
    std::wstring Meas_Aircraft_PQR_Angular_Speed (L"Meas_Aircraft_PQR_Angular_Speed");
    std::wstring Meas_Aircraft_Acceleration (L"Meas_Aircraft_Acceleration");
	std::wstring Meas_Aircraft_Speed (L"Meas_Aircraft_Speed");
    std::wstring Meas_Aircraft_Additional (L"Meas_Aircraft_Additional");
    
    if ( (theObjectClass == _TRIM_DATA_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Trim_Data_Engines.c_str())) ) 
    {
        _Discov_TRIM_DATA_ENGINES = true;
        _ObjInstance_TRIM_DATA_ENGINES_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Trim_Data_Engines > Object Instance has been discovered with handle "
             << _ObjInstance_TRIM_DATA_ENGINES_ObjectHandle
             << "."
             << std::endl;
		#endif
    }
    else if ( (theObjectClass == _TRIM_DATA_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Trim_Data_Flight_Dynamics.c_str())) ) 
    {
        _Discov_TRIM_DATA_FLIGHT_DYNAMICS = true;
        _ObjInstance_TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Trim_Data_Flight_Dynamics > Object Instance has been discovered with handle "
             << _ObjInstance_TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle
             << "."
             << std::endl;
        #endif
    }
    else if ( (theObjectClass == _JOYSTICK_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Joystick.c_str())) ) 
    {
        _Discov_JOYSTICK = true;
        _ObjInstance_JOYSTICK_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Joystick > Object Instance has been discovered with handle "
             << _ObjInstance_JOYSTICK_ObjectHandle
             << "."
             << std::endl;
        #endif
    }
    else if ( (theObjectClass == _COCKPIT_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Cockpit.c_str())) ) 
    {
        _Discov_COCKPIT = true;
        _ObjInstance_COCKPIT_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Cockpit > Object Instance has been discovered with handle "
             << _ObjInstance_COCKPIT_ObjectHandle
             << "."
             << std::endl;
        #endif
    }
    else if ( (theObjectClass == _MEAS_AIRCRAFT_POSITION_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_Position.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_POSITION = true;
        _ObjInstance_MEAS_AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_Position > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_POSITION_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ORIENTATION_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_Orientation.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_ORIENTATION = true;
        _ObjInstance_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_Orientation > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_UVW_SPEED_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_UVW_Speed.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_UVW_SPEED = true;
        _ObjInstance_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_UVW_Speed > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_PQR_Angular_Speed.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED = true;
        _ObjInstance_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_PQR_Angular_Speed > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ACCELERATION_ClassHandle) &&  (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_Acceleration.c_str()))) 
    {
        _Discov_MEAS_AIRCRAFT_ACCELERATION = true;
        _ObjInstance_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_Acceleration > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_SPEED_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_Speed.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_SPEED = true;
        _ObjInstance_MEAS_AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_Speed > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_SPEED_ObjectHandle
             << "."
             << std::endl;
        #endif
    } 
    else if ( (theObjectClass == _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Meas_Aircraft_Additional.c_str())) ) 
    {
        _Discov_MEAS_AIRCRAFT_ADDITIONAL = true;
        _ObjInstance_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle = theObject;
        #ifdef DEBUG_EFCS_FED
        std::wcout << L"< Meas_Aircraft_Additional > Object Instance has been discovered with handle "
             << _ObjInstance_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle
             << "."
             << std::endl;
        #endif
    }   
    #ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void EfcsFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void EfcsFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
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
	#ifdef DEBUG_EFCS_FED
    std::wcout << L"EfcsFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void EfcsFederateHla1516e::initialization() 
{
	waitForAllObjectDiscovered();
	waitForAllAttributesReceived();
	if (_IsExternalBridgeEnabled)
	{
		_TrimmingData.left_engine_throttle = _EFCS.getTrimThrottle(); 
		_TrimmingData.elevator = _EFCS.getTrimElevator(); 
		_TrimmingData.stabilizer = _EFCS.getTrimStabilizer(); 
		// Send
		_OutBuffer.reset();
		_OutBuffer.write_bytes((char*)&_TrimmingData, sizeof(trimming_values_t)) ;
		_OutBuffer.updateReservedBytes();
		_OuputLinkToExternalEfcsTrimming.send(static_cast<char*>(_OutBuffer(0)), _OutBuffer.size());
		
		/*
		_OutBuffer.reset();
		_OutBuffer.write_bytes((char*)&_TrimmingData, sizeof(trimming_values_t)) ;
		_OutBuffer.updateReservedBytes();
		_OuputLinkToExternalEfcsTrimming.send(static_cast<char*>(_OutBuffer(0)), _OutBuffer.size());
		* */
		
	}
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void EfcsFederateHla1516e::calculateState() 
{
	if (_IsExternalBridgeEnabled)
	{
		_OutBuffer.reset();
		_OutBuffer.write_bytes((char*)&_BridgeData, sizeof(bridge_data_t)) ;
		_OutBuffer.updateReservedBytes();
		_OuputLinkToExternalEfcs.send(static_cast<char*>(_OutBuffer(0)), _OutBuffer.size());
		_InBuffer.reset();        
	}
	else
	{
		// Model State calculation
		_EFCS.calculate_state();
	} 
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void EfcsFederateHla1516e::calculateOutput() 
{
	int ret;
	if (_IsExternalBridgeEnabled)
	{
		ret = _IncomingLinkFromExternalEfcs.receive(static_cast<char*>(_InBuffer(0)), _InBuffer.maxSize());
		if (ret > 0)
		{
			_InBuffer.assumeSizeFromReservedBytes();
			_RemotePort =  _IncomingLinkFromExternalEfcs.getLastRegisteredRemoteUdpPort();
			if ( (_RemotePort == _ExternalEfcsUdpPort))
			{
				_InBuffer.assumeSizeFromReservedBytes();
				_InBuffer.read_bytes((char*)&_EfcsOutput, sizeof(efcs_data_t)) ;
			}
		}

	}
	else
	{
		// Model output calculation
		_EFCS.calculate_output();
	}
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void EfcsFederateHla1516e::sendUpdateAttributes(RTI1516fedTime UpdateTime)
{   
	rti1516e::AttributeHandleValueMap ahvps_FLIGHT_CONTROLS;
	
	_OutputMessagebuffer.reset() ;
	if (_IsExternalBridgeEnabled)
	{
		_OutputMessagebuffer.write_double(_EfcsOutput.cmd_right_aileron);
	}
	else
	{
		_OutputMessagebuffer.write_double(_EFCS.getRightAileron());
	}
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_FLIGHT_CONTROLS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE,attrValue1)); 
	
	_OutputMessagebuffer.reset() ;
	if (_IsExternalBridgeEnabled)
	{
		_OutputMessagebuffer.write_double(_EfcsOutput.cmd_left_aileron);
	}
	else
	{
		_OutputMessagebuffer.write_double(_EFCS.getLeftAileron());
	}
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_FLIGHT_CONTROLS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE,attrValue2)); 
	
	_OutputMessagebuffer.reset() ;
	if (_IsExternalBridgeEnabled)
	{
		_OutputMessagebuffer.write_double(_EfcsOutput.cmd_right_elevator);
	}
	else
	{
		_OutputMessagebuffer.write_double(_EFCS.getRightElevator());
	}
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_FLIGHT_CONTROLS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE,attrValue3));

	_OutputMessagebuffer.reset() ;
	if (_IsExternalBridgeEnabled)
	{
		_OutputMessagebuffer.write_double(_EfcsOutput.cmd_left_elevator);
	}
	else
	{
		_OutputMessagebuffer.write_double(_EFCS.getLeftElevator());
	}
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_FLIGHT_CONTROLS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE,attrValue4));

	_OutputMessagebuffer.reset() ;
	if (_IsExternalBridgeEnabled)
	{
		_OutputMessagebuffer.write_double(_EfcsOutput.cmd_rudder);
	}
	else
	{
		_OutputMessagebuffer.write_double(_EFCS.getRudder());
	}
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue5 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_FLIGHT_CONTROLS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE,attrValue5));
	
	_OutputMessagebuffer.reset() ;
	if (_IsExternalBridgeEnabled)
	{
		_OutputMessagebuffer.write_double(_EfcsOutput.cmd_right_engine_throttle);
	}
	else
	{
		_OutputMessagebuffer.write_double(_EFCS.getRightEngine());
	}
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue6 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_FLIGHT_CONTROLS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE,attrValue6));

	_OutputMessagebuffer.reset() ;
	if (_IsExternalBridgeEnabled)
	{
		_OutputMessagebuffer.write_double(_EfcsOutput.cmd_left_engine_throttle);
	}
	else
	{
		_OutputMessagebuffer.write_double(_EFCS.getLeftEngine());
	}
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue7 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_FLIGHT_CONTROLS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE,attrValue7));

	_OutputMessagebuffer.reset() ;
	if (_IsExternalBridgeEnabled)
	{
		_OutputMessagebuffer.write_double(_EfcsOutput.cmd_left_flap);
	}
	else
	{
		_OutputMessagebuffer.write_double(_EFCS.getFlaps());
	}
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue8 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_FLIGHT_CONTROLS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE,attrValue8));

	_OutputMessagebuffer.reset() ;
	if (_IsExternalBridgeEnabled)
	{
		_OutputMessagebuffer.write_double(_EfcsOutput.cmd_spoilers);
	}
	else
	{
		_OutputMessagebuffer.write_double(_EFCS.getSpoilers());
	}
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue9 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_FLIGHT_CONTROLS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE,attrValue9));
	
	_OutputMessagebuffer.reset() ;
	if (_IsExternalBridgeEnabled)
	{
		_OutputMessagebuffer.write_double(_EfcsOutput.cmd_stabilizer);
	}
	else
	{
		_OutputMessagebuffer.write_double(_EFCS.getStabilizer());
	}
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue10 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_FLIGHT_CONTROLS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE,attrValue10));

	_OutputMessagebuffer.reset() ;
	if (_IsExternalBridgeEnabled)
	{
		_OutputMessagebuffer.write_double(_EfcsOutput.cmd_gears);
	}
	else
	{
		_OutputMessagebuffer.write_double(_EFCS.getGears());
	}
	_OutputMessagebuffer.updateReservedBytes();
	rti1516e::VariableLengthData attrValue11 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_FLIGHT_CONTROLS.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_GEARS_COMMANDED_POSITION_ATTRIBUTE,attrValue11));

    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb->updateAttributeValues(_FLIGHT_CONTROLS_ObjectHandle, ahvps_FLIGHT_CONTROLS, _MyTag);
        }
        else
        {
            _RtiAmb->updateAttributeValues(_FLIGHT_CONTROLS_ObjectHandle, ahvps_FLIGHT_CONTROLS, _MyTag , UpdateTime);
        }
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void EfcsFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
void EfcsFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
    
    if (theObject == _ObjInstance_TRIM_DATA_ENGINES_ObjectHandle)
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
            if      (parmHandle == _RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE) 
            { 
				_EFCS.setTrimThrottle(buffer.read_double()); 
				#ifdef DEBUG_EFCS_FED
				std::wcout << L"Trim for Right Engine Ok <<" << std::endl;
				#endif
			}
            else if (parmHandle == _LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE)  
            { 
				_EFCS.setTrimThrottle(buffer.read_double()); 
				#ifdef DEBUG_EFCS_FED
				std::wcout << L"Trim for Left Engine Ok <<" << std::endl;
				#endif
			}
            else
            { 
				std::wcout << L"EfcsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_TRIM_DATA_ENGINES= true;
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
				_EFCS.setTrimElevator(buffer.read_double()); 
				#ifdef DEBUG_EFCS_FED
				std::wcout << L"Trim for Elevator Ok <<" << std::endl;
				#endif
			}
            else if (parmHandle == _STABILIZER_DEFLECTION_EQ_ATTRIBUTE)  
            { 
				_EFCS.setTrimStabilizer(buffer.read_double()); 
				#ifdef DEBUG_EFCS_FED
				std::wcout << L"Trim for Stabilizer Ok <<" << std::endl;
				#endif
			}
            else
            { 
				std::wcout << L"EfcsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_TRIM_DATA_FLIGHT_DYNAMICS= true;
    } 
    else if (theObject == _ObjInstance_JOYSTICK_ObjectHandle)
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
				tmp_value = buffer.read_double();
				_EFCS.setAileron_Joystick(tmp_value);
				_BridgeData.joystick_output.ss_left_aileron = tmp_value;
				_BridgeData.joystick_output.ss_right_aileron = tmp_value;
			}
            else if (parmHandle == _ELEVATOR_ATTRIBUTE)  
            { 
				tmp_value = buffer.read_double();
				_EFCS.setElevator_Joystick(tmp_value);
				_BridgeData.joystick_output.ss_left_elevator = tmp_value;
				_BridgeData.joystick_output.ss_right_elevator = tmp_value;
			}
            else if (parmHandle == _RUDDER_ATTRIBUTE)    
            { 
				tmp_value = buffer.read_double();
				_EFCS.setRudder_Joystick(tmp_value);
				_BridgeData.joystick_output.ss_rudder = tmp_value;
			}
            else if (parmHandle == _THROTTLE_LEFT_ATTRIBUTE)  	
            { 
				tmp_value = buffer.read_double();
				_EFCS.setThrottle_Left_Joystick(tmp_value); 
				_BridgeData.joystick_output.ss_left_engine_throttle = tmp_value;
			}
            else if (parmHandle == _THROTTLE_RIGHT_ATTRIBUTE)  	
            {
				tmp_value = buffer.read_double(); 
				_EFCS.setThrottle_Right_Joystick(tmp_value); 
				_BridgeData.joystick_output.ss_right_engine_throttle = tmp_value;
			}
            else if (parmHandle == _FLAPS_ATTRIBUTE)     
            { 
				tmp_value = buffer.read_double();
				_EFCS.setFlaps_Joystick(tmp_value);
				_BridgeData.joystick_output.ss_right_flap = tmp_value;
				_BridgeData.joystick_output.ss_left_flap = tmp_value;
			}
            else if (parmHandle == _SPOILERS_ATTRIBUTE)  
            { 
				tmp_value = buffer.read_double();
				_EFCS.setSpoilers_Joystick(tmp_value);
				_BridgeData.joystick_output.ss_spoilers = tmp_value;
			}
            else if (parmHandle == _GEARS_ATTRIBUTE)     
            {
				tmp_value = buffer.read_double(); 
				_EFCS.setGears_Joystick(tmp_value);
				_BridgeData.joystick_output.ss_gears = tmp_value;
			}
            else if (parmHandle == _BRAKES_ATTRIBUTE)    
            { 
				tmp_value = buffer.read_double(); 
				_EFCS.setBrakes_Joystick(tmp_value);
				_BridgeData.joystick_output.ss_speedbrake = tmp_value;
			}
            else
            { 
				std::wcout << L"EfcsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_JOYSTICK = true;
    }   
    else if (theObject == _ObjInstance_COCKPIT_ObjectHandle)
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
				tmp_value = buffer.read_double();
				_EFCS.setAutopilot_AP_Active(tmp_value);
				if (fabs(tmp_value -1.0) < 0.00001) _BridgeData.fcu_output.ap1_active = true;
				else _BridgeData.fcu_output.ap1_active = false;
				
			}
            else if (parmHandle == _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE)   	
            { 
				tmp_value = buffer.read_double();
				_EFCS.setAutopilot_athr_Active(tmp_value);
				if (fabs(tmp_value -1.0) < 0.00001) _BridgeData.fcu_output.athr_active = true;
				else _BridgeData.fcu_output.athr_active = false;
			}
            else if (parmHandle == _AUTOPILOT_SPD_ATTRIBUTE)   			
            { 
				tmp_value = buffer.read_double();
				_EFCS.setAutopilot_spd(tmp_value);
				_BridgeData.fcu_output.speed_std = tmp_value;
			}
            else if (parmHandle == _AUTOPILOT_HDG_ATTRIBUTE)   			
            { 
				tmp_value = buffer.read_double();
				_EFCS.setAutopilot_hdg(tmp_value);
				_BridgeData.fcu_output.heading = tmp_value;
			}
            else if (parmHandle == _AUTOPILOT_ALT_ATTRIBUTE)   			
            {
				tmp_value = buffer.read_double(); 
				_EFCS.setAutopilot_alt(tmp_value);
				_BridgeData.fcu_output.altitude = tmp_value;
			}
            else if (parmHandle == _AUTOPILOT_VS_ATTRIBUTE)   			
            {
				tmp_value = buffer.read_double();
				_EFCS.setAutopilot_vs(tmp_value);
				_BridgeData.fcu_output.vs_speed = tmp_value;
			 }
            else
            { 
				std::wcout << L"EfcsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_COCKPIT = true;
    }
    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_POSITION_ObjectHandle)
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
				tmp_value = buffer.read_double();
				_EFCS.setLongitude(tmp_value); 
				_BridgeData.flight_dynamics_output.longitude = tmp_value;
			}
            else if (parmHandle == _MEAS_LATITUDE_ATTRIBUTE)  
            { 
				tmp_value = buffer.read_double();
				_EFCS.setLatitude(tmp_value);
				_BridgeData.flight_dynamics_output.latitude = tmp_value; 
			}
            else if (parmHandle == _MEAS_ALTITUDE_ATTRIBUTE)  
            { 
				tmp_value = buffer.read_double();
				_EFCS.setAltitude(tmp_value); 
				_BridgeData.flight_dynamics_output.altitude_sl = tmp_value; 
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

    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle)
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
				tmp_value = buffer.read_double();
				_EFCS.setPhi(tmp_value); 
				_BridgeData.flight_dynamics_output.phi = tmp_value; 
			}
            else if (parmHandle == _MEAS_THETA_ATTRIBUTE) 
            {
				tmp_value = buffer.read_double(); 
				_EFCS.setTheta(tmp_value); 
				_BridgeData.flight_dynamics_output.theta = tmp_value; 
			}
            else if (parmHandle == _MEAS_PSI_ATTRIBUTE)   
            { 
				tmp_value = buffer.read_double(); 
				_EFCS.setPsi(tmp_value); 
				_BridgeData.flight_dynamics_output.psi = tmp_value;
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
    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle)
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
				tmp_value = buffer.read_double();  
				_EFCS.setU(tmp_value); 
				_BridgeData.flight_dynamics_output.v_body_u = tmp_value;
			}
            else if (parmHandle == _MEAS_V_SPEED_ATTRIBUTE) 
            {
				tmp_value = buffer.read_double();  
				_EFCS.setV(tmp_value); 
				_BridgeData.flight_dynamics_output.v_body_v = tmp_value;
			}
            else if (parmHandle == _MEAS_W_SPEED_ATTRIBUTE) 
            {
				tmp_value = buffer.read_double();  
				_EFCS.setW(tmp_value); 
				_BridgeData.flight_dynamics_output.v_body_w = tmp_value;
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
    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle)
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
				tmp_value = buffer.read_double();
				_EFCS.setP(tmp_value); 
				_BridgeData.flight_dynamics_output.phidot = tmp_value;
			}
            else if (parmHandle == _MEAS_Q_ANG_SPEED_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double();
				_EFCS.setQ(tmp_value); 
				_BridgeData.flight_dynamics_output.thetadot = tmp_value;
			}
            else if (parmHandle == _MEAS_R_ANG_SPEED_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double();  
				_EFCS.setR(tmp_value);
				_BridgeData.flight_dynamics_output.psidot = tmp_value; 
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
    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle)
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
				tmp_value = buffer.read_double();
				_EFCS.setAx(tmp_value);
				_BridgeData.flight_dynamics_output.A_X_pilot = tmp_value;  
			}
            else if (parmHandle == _MEAS_Y_ACC_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double();
				_EFCS.setAy(tmp_value); 
				_BridgeData.flight_dynamics_output.A_Y_pilot = tmp_value; 
			}
            else if (parmHandle == _MEAS_Z_ACC_ATTRIBUTE) 
            { 
				tmp_value = buffer.read_double();
				_EFCS.setAz(tmp_value);
				_BridgeData.flight_dynamics_output.A_Z_pilot = tmp_value;  
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
    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_SPEED_ObjectHandle)
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
				tmp_value = buffer.read_double(); 
				_EFCS.setIAS(tmp_value); 
				_BridgeData.flight_dynamics_output.vias = tmp_value; 
			}
            else if (parmHandle == _MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE) 
            {
				tmp_value = buffer.read_double(); 
				_EFCS.setEAS(tmp_value); 
				_BridgeData.flight_dynamics_output.veas = tmp_value; 
			}
            else if (parmHandle == _MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE) 
            {
				tmp_value = buffer.read_double();  
				_EFCS.setCAS(tmp_value); 
				_BridgeData.flight_dynamics_output.vcas = tmp_value;
			}
            else if (parmHandle == _MEAS_TRUE_AIRSPEED_ATTRIBUTE)       
            { 
				tmp_value = buffer.read_double();
				_EFCS.setTAS(tmp_value);
				_BridgeData.flight_dynamics_output.vtas = tmp_value; 
			}
            else if (parmHandle == _MEAS_GROUND_SPEED_ATTRIBUTE)        
            {
				tmp_value = buffer.read_double(); 
				_EFCS.setGS(tmp_value); 
				_BridgeData.flight_dynamics_output.ground_speed = tmp_value;
			}
            else if (parmHandle == _MEAS_VERTICAL_SPEED_ATTRIBUTE)      
            { 
				tmp_value = buffer.read_double();
				_EFCS.setVS(tmp_value); 
				_BridgeData.flight_dynamics_output.climb_rate = tmp_value;
			}
            else if (parmHandle == _MEAS_MACH_NUMBER_ATTRIBUTE)        
            { 
				tmp_value = buffer.read_double();
				_EFCS.setMach(tmp_value); 
				_BridgeData.flight_dynamics_output.vmach = tmp_value;
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
    else if (theObject == _ObjInstance_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle)
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
				tmp_value = buffer.read_double(); 
				_EFCS.setAlpha(tmp_value); 
				_BridgeData.flight_dynamics_output.alpha = tmp_value;
			}
            else if (parmHandle == _MEAS_BETA_ATTRIBUTE)             
            { 
				tmp_value = buffer.read_double();
				_EFCS.setBeta(tmp_value); 
				_BridgeData.flight_dynamics_output.beta = tmp_value;
			}
            else if (parmHandle == _MEAS_DYNAMIC_PRESSURE_ATTRIBUTE) 
            {
				tmp_value = buffer.read_double(); 
				_EFCS.setQbar(tmp_value);
				_BridgeData.flight_dynamics_output.dyn_pressure = tmp_value; 
			}
            else if (parmHandle == _MEAS_TANK_FILLING_ATTRIBUTE)     
            { 
				tmp_value = buffer.read_double();
				_EFCS.setTankFilling(tmp_value); 
				//_BridgeData.flight_dynamics_output.vmach = tmp_value;
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
	else
	{ 
		std::wcout << L"EfcsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << std::endl; 
	} 
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void EfcsFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
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
void EfcsFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
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
void EfcsFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_EFCS_FED
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void EfcsFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void EfcsFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void EfcsFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void EfcsFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void EfcsFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void EfcsFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void EfcsFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
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
void EfcsFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
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
void EfcsFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
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
void EfcsFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
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
void EfcsFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void EfcsFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void EfcsFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void EfcsFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// function : run
void EfcsFederateHla1516e::run()
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
