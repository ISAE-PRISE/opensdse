#include "SensorsFederateHla1516e.hh"

// ----------------------------------------------------------------------------
// TemplateFederateHla13 Constructor
SensorsFederateHla1516e::SensorsFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_SENSORS_FED
	std::wcout << L"SensorsFederateHla1516e.cc -> Constructor(): Start" << std::endl;
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
	_Discov_AIRCRAFT_ORIENTATION       = false;
	_Discov_AIRCRAFT_UVW_SPEED         = false;
	_Discov_AIRCRAFT_PQR_ANGULAR_SPEED = false;
	_Discov_AIRCRAFT_ACCELERATION      = false;
	_Discov_AIRCRAFT_SPEED             = false;
	_Discov_AIRCRAFT_ADDITIONAL        = false;
	_New_AIRCRAFT_POSITION             = false;
	_New_AIRCRAFT_ORIENTATION          = false;
	_New_AIRCRAFT_UVW_SPEED            = false;
	_New_AIRCRAFT_PQR_ANGULAR_SPEED    = false;
	_New_AIRCRAFT_ACCELERATION         = false;
	_New_AIRCRAFT_SPEED                = false;
	_New_AIRCRAFT_ADDITIONAL           = false;
	
	// Initialize Sensor Objects
	// TO DO: This needs to be changed for xml configuration instead of hardcoded values
    _Sensor_LONGITUDE.setFilter(1,3.0); 
    _Sensor_LATITUDE.setFilter(1,3.0);  
    _Sensor_ALTITUDE.setFilter(1,3.0);   
    _Sensor_PHI.setFilter(1,3.0);
    _Sensor_THETA.setFilter(1,3.0);
    _Sensor_PSI.setFilter(1,3.0); 
    _Sensor_U_SPEED.setFilter(1,0.5);
    _Sensor_V_SPEED.setFilter(1,0.5);
    _Sensor_W_SPEED.setFilter(1,0.5); 
    _Sensor_P_ANG_SPEED.setFilter(2,3.0);
    _Sensor_Q_ANG_SPEED.setFilter(2,3.0); 
    _Sensor_R_ANG_SPEED.setFilter(2,3.0); 
    _Sensor_X_ACC.setFilter(1,10.0);
    _Sensor_Y_ACC.setFilter(1,10.0);
    _Sensor_Z_ACC.setFilter(1,10.0);
    _Sensor_INDICATED_AIRSPEED.setFilter(1,0.5);
    _Sensor_EQUIVALENT_AIRSPEED.setFilter(1,0.5);
    _Sensor_CALIBRATED_AIRSPEED.setFilter(1,0.5); 
    _Sensor_TRUE_AIRSPEED.setFilter(1,0.5);
    _Sensor_GROUND_SPEED.setFilter(1,0.5);
    _Sensor_VERTICAL_SPEED.setFilter(1,0.5); 
    _Sensor_MACH_NUMBER.setFilter(1,0.5);   
    _Sensor_ALPHA.setFilter(1,3.0);
    _Sensor_BETA.setFilter(1,3.0);
    _Sensor_DYNAMIC_PRESSURE.setFilter(1,3.0); 
    _Sensor_TANK_FILLING.setFilter(1,3.0);

	#ifdef DEBUG_SENSORS_FED
	std::wcout << L"SensorsFederateHla1516e.cc -> Constructor(): End" << std::endl;
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
SensorsFederateHla1516e::~SensorsFederateHla1516e()
{
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void SensorsFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_SENSORS_FED
	std::wcout << L"SensorsFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_SENSORS_FED
	std::wcout << L"SensorsFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void SensorsFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_SENSORS_FED
	std::wcout << L"SensorsFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_SENSORS_FED
	std::wcout << L"SensorsFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void SensorsFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_SENSORS_FED
	std::wcout << L"SensorsFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
														 , L"sensors"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_SENSORS_FED
	std::wcout << L"SensorsFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void SensorsFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle SensorsFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void SensorsFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void SensorsFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
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
		_AIRCRAFT_POSITION_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_POSITION);
		_AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_ORIENTATION);
		_AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_UVW_SPEED);
		_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_PQR_ANGULAR_SPEED);
		_AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_ACCELERATION);
		_AIRCRAFT_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_SPEED);
		_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_ADDITIONAL);
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

	
		// Published
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
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void SensorsFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
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
		
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_POSITION_ClassHandle,attr_AIRCRAFT_POSITION);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_ORIENTATION_ClassHandle,attr_AIRCRAFT_ORIENTATION);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_UVW_SPEED_ClassHandle,attr_AIRCRAFT_UVW_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,attr_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_ACCELERATION_ClassHandle,attr_AIRCRAFT_ACCELERATION);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_SPEED_ClassHandle,attr_AIRCRAFT_SPEED);
        _RtiAmb->subscribeObjectClassAttributes(_AIRCRAFT_ADDITIONAL_ClassHandle,attr_AIRCRAFT_ADDITIONAL);
        
		// For Class/Attributes published
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
		
        _RtiAmb->publishObjectClassAttributes(_MEAS_AIRCRAFT_POSITION_ClassHandle, attr_MEAS_AIRCRAFT_POSITION);
        _RtiAmb->publishObjectClassAttributes(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle, attr_MEAS_AIRCRAFT_ORIENTATION);
        _RtiAmb->publishObjectClassAttributes(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle, attr_MEAS_AIRCRAFT_UVW_SPEED);
        _RtiAmb->publishObjectClassAttributes(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb->publishObjectClassAttributes(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle, attr_MEAS_AIRCRAFT_ACCELERATION);
        _RtiAmb->publishObjectClassAttributes(_MEAS_AIRCRAFT_SPEED_ClassHandle, attr_MEAS_AIRCRAFT_SPEED);
        _RtiAmb->publishObjectClassAttributes(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle, attr_MEAS_AIRCRAFT_ADDITIONAL); 

	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void SensorsFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring Meas_Aircraft_Position (L"Meas_Aircraft_Position");
		std::wstring Meas_Aircraft_Orientation (L"Meas_Aircraft_Orientation");
		std::wstring Meas_Aircraft_UVW_Speed (L"Meas_Aircraft_UVW_Speed");
		std::wstring Meas_Aircraft_PQR_Angular_Speed (L"Meas_Aircraft_PQR_Angular_Speed");
		std::wstring Meas_Aircraft_Acceleration (L"Meas_Aircraft_Acceleration");
		std::wstring Meas_Aircraft_Speed (L"Meas_Aircraft_Speed");
		std::wstring Meas_Aircraft_Additional (L"Meas_Aircraft_Additional");
		
        _MEAS_AIRCRAFT_POSITION_ObjectHandle = _RtiAmb->registerObjectInstance(_MEAS_AIRCRAFT_POSITION_ClassHandle,Meas_Aircraft_Position);
        _MEAS_AIRCRAFT_ORIENTATION_ObjectHandle = _RtiAmb->registerObjectInstance(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle,Meas_Aircraft_Orientation);
        _MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle = _RtiAmb->registerObjectInstance(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle,Meas_Aircraft_UVW_Speed);
        _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle = _RtiAmb->registerObjectInstance(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,Meas_Aircraft_PQR_Angular_Speed);
        _MEAS_AIRCRAFT_ACCELERATION_ObjectHandle = _RtiAmb->registerObjectInstance(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle,Meas_Aircraft_Acceleration);
        _MEAS_AIRCRAFT_SPEED_ObjectHandle = _RtiAmb->registerObjectInstance(_MEAS_AIRCRAFT_SPEED_ClassHandle,Meas_Aircraft_Speed);
        _MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle = _RtiAmb->registerObjectInstance(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle,Meas_Aircraft_Additional);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void SensorsFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
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
       
    try 
    {
        _RtiAmb->unpublishObjectClass(_MEAS_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb->unpublishObjectClass(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb->unpublishObjectClass(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb->unpublishObjectClass(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb->unpublishObjectClass(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb->unpublishObjectClass(_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb->unpublishObjectClass(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void SensorsFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
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
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void SensorsFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
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
    #ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void SensorsFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
	std::wstring Aircraft_Position (L"Aircraft_Position");
	std::wstring Aircraft_Orientation (L"Aircraft_Orientation");
    std::wstring Aircraft_UVW_Speed (L"Aircraft_UVW_Speed");
    std::wstring Aircraft_PQR_Angular_Speed (L"Aircraft_PQR_Angular_Speed");
    std::wstring Aircraft_Acceleration (L"Aircraft_Acceleration");
	std::wstring Aircraft_Speed (L"Aircraft_Speed");
    std::wstring Aircraft_Additional (L"Aircraft_Additional");
    
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
    else if ( (theObjectClass == _AIRCRAFT_UVW_SPEED_ClassHandle) && (!wcscmp(theObjectInstanceName.c_str(),Aircraft_UVW_Speed.c_str())) ) 
    {
        _Discov_AIRCRAFT_UVW_SPEED = true;
        _AIRCRAFT_UVW_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_SENSORS_FED
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
        #ifdef DEBUG_SENSORS_FED
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
        #ifdef DEBUG_SENSORS_FED
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
        #ifdef DEBUG_SENSORS_FED
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
        #ifdef DEBUG_SENSORS_FED
        std::wcout << L"< Aircraft_Additional > Object Instance has been discovered with handle "
             << _AIRCRAFT_ADDITIONAL_ObjectHandle
             << "."
             << std::endl;
        #endif
    }   
    #ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void SensorsFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void SensorsFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
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
	#ifdef DEBUG_SENSORS_FED
    std::wcout << L"SensorsFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void SensorsFederateHla1516e::initialization() 
{
	waitForAllObjectDiscovered();
	waitForAllAttributesReceived();	
	// State Initialization     
	_Sensor_LONGITUDE.setInitialState(_Sensor_LONGITUDE.getInputSignal());
	_Sensor_LATITUDE.setInitialState(_Sensor_LATITUDE.getInputSignal());
	_Sensor_ALTITUDE.setInitialState(_Sensor_ALTITUDE.getInputSignal());
	_Sensor_PHI.setInitialState(_Sensor_PHI.getInputSignal());
	_Sensor_THETA.setInitialState(_Sensor_THETA.getInputSignal());
	_Sensor_PSI.setInitialState(_Sensor_PSI.getInputSignal());
	_Sensor_U_SPEED.setInitialState(_Sensor_U_SPEED.getInputSignal());
	_Sensor_V_SPEED.setInitialState(_Sensor_V_SPEED.getInputSignal());
	_Sensor_W_SPEED.setInitialState(_Sensor_W_SPEED.getInputSignal());
	_Sensor_P_ANG_SPEED.setInitialState(_Sensor_P_ANG_SPEED.getInputSignal());
	_Sensor_Q_ANG_SPEED.setInitialState(_Sensor_Q_ANG_SPEED.getInputSignal());
	_Sensor_R_ANG_SPEED.setInitialState(_Sensor_R_ANG_SPEED.getInputSignal());
	_Sensor_X_ACC.setInitialState(_Sensor_X_ACC.getInputSignal());
	_Sensor_Y_ACC.setInitialState(_Sensor_Y_ACC.getInputSignal());
	_Sensor_Z_ACC.setInitialState(_Sensor_Z_ACC.getInputSignal());
	_Sensor_INDICATED_AIRSPEED.setInitialState( _Sensor_INDICATED_AIRSPEED.getInputSignal());
	_Sensor_EQUIVALENT_AIRSPEED.setInitialState(_Sensor_EQUIVALENT_AIRSPEED.getInputSignal());
	_Sensor_CALIBRATED_AIRSPEED.setInitialState(_Sensor_CALIBRATED_AIRSPEED.getInputSignal());
	_Sensor_TRUE_AIRSPEED.setInitialState(_Sensor_TRUE_AIRSPEED.getInputSignal());
	_Sensor_GROUND_SPEED.setInitialState(_Sensor_GROUND_SPEED.getInputSignal());
	_Sensor_VERTICAL_SPEED.setInitialState(_Sensor_VERTICAL_SPEED.getInputSignal());
	_Sensor_MACH_NUMBER.setInitialState(_Sensor_MACH_NUMBER.getInputSignal());
	_Sensor_ALPHA.setInitialState(_Sensor_ALPHA.getInputSignal());
	_Sensor_BETA.setInitialState(_Sensor_BETA.getInputSignal());
	_Sensor_DYNAMIC_PRESSURE.setInitialState(_Sensor_DYNAMIC_PRESSURE.getInputSignal());
	_Sensor_TANK_FILLING.setInitialState(_Sensor_TANK_FILLING.getInputSignal());
    // Output Initialization 
    _Sensor_LONGITUDE.setOutputSignal(_Sensor_LONGITUDE.getInputSignal());
    _Sensor_LATITUDE.setOutputSignal(_Sensor_LATITUDE.getInputSignal());
    _Sensor_ALTITUDE.setOutputSignal(_Sensor_ALTITUDE.getInputSignal());  
    _Sensor_PHI.setOutputSignal(_Sensor_PHI.getInputSignal());
    _Sensor_THETA.setOutputSignal(_Sensor_THETA.getInputSignal());
    _Sensor_PSI.setOutputSignal(_Sensor_PSI.getInputSignal());       
    _Sensor_U_SPEED.setOutputSignal(_Sensor_U_SPEED.getInputSignal());
    _Sensor_V_SPEED.setOutputSignal(_Sensor_V_SPEED.getInputSignal());
    _Sensor_W_SPEED.setOutputSignal(_Sensor_W_SPEED.getInputSignal());        
    _Sensor_P_ANG_SPEED.setOutputSignal(_Sensor_P_ANG_SPEED.getInputSignal());
    _Sensor_Q_ANG_SPEED.setOutputSignal(_Sensor_Q_ANG_SPEED.getInputSignal());
    _Sensor_R_ANG_SPEED.setOutputSignal(_Sensor_R_ANG_SPEED.getInputSignal());        
    _Sensor_X_ACC.setOutputSignal(_Sensor_X_ACC.getInputSignal());
    _Sensor_Y_ACC.setOutputSignal(_Sensor_Y_ACC.getInputSignal());
    _Sensor_Z_ACC.setOutputSignal(_Sensor_Z_ACC.getInputSignal());        
    _Sensor_INDICATED_AIRSPEED.setOutputSignal( _Sensor_INDICATED_AIRSPEED.getInputSignal());
    _Sensor_EQUIVALENT_AIRSPEED.setOutputSignal(_Sensor_EQUIVALENT_AIRSPEED.getInputSignal());
    _Sensor_CALIBRATED_AIRSPEED.setOutputSignal(_Sensor_CALIBRATED_AIRSPEED.getInputSignal());
    _Sensor_TRUE_AIRSPEED.setOutputSignal(_Sensor_TRUE_AIRSPEED.getInputSignal());
    _Sensor_GROUND_SPEED.setOutputSignal(_Sensor_GROUND_SPEED.getInputSignal());
    _Sensor_VERTICAL_SPEED.setOutputSignal(_Sensor_VERTICAL_SPEED.getInputSignal());
    _Sensor_MACH_NUMBER.setOutputSignal(_Sensor_MACH_NUMBER.getInputSignal());          
    _Sensor_ALPHA.setOutputSignal(_Sensor_ALPHA.getInputSignal());
    _Sensor_BETA.setOutputSignal(_Sensor_BETA.getInputSignal());
    _Sensor_DYNAMIC_PRESSURE.setOutputSignal(_Sensor_DYNAMIC_PRESSURE.getInputSignal());
    _Sensor_TANK_FILLING.setOutputSignal(_Sensor_TANK_FILLING.getInputSignal());
    sendInitialAllAttributes();
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void SensorsFederateHla1516e::calculateState() 
{
	// Models State calculation
    _Sensor_LONGITUDE.calculateState(); 
    _Sensor_LATITUDE.calculateState(); 
    _Sensor_ALTITUDE.calculateState();    
    _Sensor_PHI.calculateState(); 
    _Sensor_THETA.calculateState(); 
    _Sensor_PSI.calculateState();     
    _Sensor_U_SPEED.calculateState();
    _Sensor_V_SPEED.calculateState();
    _Sensor_W_SPEED.calculateState();      
    _Sensor_P_ANG_SPEED.calculateState();
    _Sensor_Q_ANG_SPEED.calculateState();
    _Sensor_R_ANG_SPEED.calculateState();       
    _Sensor_X_ACC.calculateState();
    _Sensor_Y_ACC.calculateState();
    _Sensor_Z_ACC.calculateState();    
    _Sensor_INDICATED_AIRSPEED.calculateState(); 
    _Sensor_EQUIVALENT_AIRSPEED.calculateState(); 
    _Sensor_CALIBRATED_AIRSPEED.calculateState();
    _Sensor_TRUE_AIRSPEED.calculateState();
    _Sensor_GROUND_SPEED.calculateState();
    _Sensor_VERTICAL_SPEED.calculateState();
    _Sensor_MACH_NUMBER.calculateState();        
    _Sensor_ALPHA.calculateState();
    _Sensor_BETA.calculateState();
    _Sensor_DYNAMIC_PRESSURE.calculateState();
    _Sensor_TANK_FILLING.calculateState();
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void SensorsFederateHla1516e::calculateOutput() 
{
	// Models output calculation
    _Sensor_LONGITUDE.calculateOutput(); 
    _Sensor_LATITUDE.calculateOutput(); 
    _Sensor_ALTITUDE.calculateOutput();   
    _Sensor_PHI.calculateOutput(); 
    _Sensor_THETA.calculateOutput(); 
    _Sensor_PSI.calculateOutput();        
    _Sensor_U_SPEED.calculateOutput();
    _Sensor_V_SPEED.calculateOutput();
    _Sensor_W_SPEED.calculateOutput();        
    _Sensor_P_ANG_SPEED.calculateOutput();
    _Sensor_Q_ANG_SPEED.calculateOutput();
    _Sensor_R_ANG_SPEED.calculateOutput();        
    _Sensor_X_ACC.calculateOutput();
    _Sensor_Y_ACC.calculateOutput();
    _Sensor_Z_ACC.calculateOutput();         
    _Sensor_INDICATED_AIRSPEED.calculateOutput(); 
    _Sensor_EQUIVALENT_AIRSPEED.calculateOutput(); 
    _Sensor_CALIBRATED_AIRSPEED.calculateOutput();
    _Sensor_TRUE_AIRSPEED.calculateOutput();
    _Sensor_GROUND_SPEED.calculateOutput();
    _Sensor_VERTICAL_SPEED.calculateOutput();
    _Sensor_MACH_NUMBER.calculateOutput();          
    _Sensor_ALPHA.calculateOutput();
    _Sensor_BETA.calculateOutput();
    _Sensor_DYNAMIC_PRESSURE.calculateOutput();
    _Sensor_TANK_FILLING.calculateOutput();
}

// ----------------------------------------------------------------------------
// Send All Updates for to init FD Attributes in the whole federation
void SensorsFederateHla1516e::sendInitialAllAttributes()
{
	//std::cout << "FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;
	rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_POSITION;
    rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_ORIENTATION;
    rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_UVW_SPEED;
    rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED;
    rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_ACCELERATION;
    rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_SPEED;
    rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_ADDITIONAL;  
    
    rti1516e::VariableLengthData MyTag1, MyTag2, MyTag3, MyTag4, MyTag5, MyTag6, MyTag7;
	std::string testTag ("Meas_Aircraft_Position");
	MyTag1.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag1 ("Meas_Aircraft_Orientation");
	MyTag2.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag2 ("Meas_Aircraft_UVW_Speed");
	MyTag3.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag3 ("Meas_Aircraft_PQR_Angular_Speed");
	MyTag4.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag4 ("Meas_Aircraft_UVW_Speed");
	MyTag5.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag5 ("Meas_Aircraft_Speed");
	MyTag6.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag6 ("Meas_Aircraft_Additional");
	MyTag7.setData (testTag.c_str (), testTag.size () + 1);
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_LONGITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_LONGITUDE_ATTRIBUTE,attrValue1)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_LATITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_LATITUDE_ATTRIBUTE,attrValue2)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_ALTITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_ALTITUDE_ATTRIBUTE,attrValue3)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_PHI.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ORIENTATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_PHI_ATTRIBUTE,attrValue4)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_THETA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue5 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ORIENTATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_THETA_ATTRIBUTE,attrValue5)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_THETA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue6 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ORIENTATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_PSI_ATTRIBUTE,attrValue6));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_U_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue7 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_UVW_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_U_SPEED_ATTRIBUTE,attrValue7));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_V_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue8 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_UVW_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_V_SPEED_ATTRIBUTE,attrValue8));
    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_W_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue9 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_UVW_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_W_SPEED_ATTRIBUTE,attrValue9));
    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_P_ANG_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue10 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_P_ANG_SPEED_ATTRIBUTE,attrValue10));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_Q_ANG_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue11 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_Q_ANG_SPEED_ATTRIBUTE,attrValue11));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_R_ANG_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue12 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_R_ANG_SPEED_ATTRIBUTE,attrValue12)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_X_ACC.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue13 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ACCELERATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_X_ACC_ATTRIBUTE,attrValue13));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_Y_ACC.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue14 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ACCELERATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_Y_ACC_ATTRIBUTE,attrValue14));   
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_Z_ACC.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue15 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ACCELERATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_Z_ACC_ATTRIBUTE,attrValue15));  
    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_INDICATED_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue16 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_INDICATED_AIRSPEED_ATTRIBUTE,attrValue16));   
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_EQUIVALENT_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue17 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE,attrValue17));   
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_CALIBRATED_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue18 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE,attrValue18));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_TRUE_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue19 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_TRUE_AIRSPEED_ATTRIBUTE,attrValue19));  
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_GROUND_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue20 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_GROUND_SPEED_ATTRIBUTE,attrValue20));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_VERTICAL_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue21 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_VERTICAL_SPEED_ATTRIBUTE,attrValue21));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_MACH_NUMBER.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue22 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_MACH_NUMBER_ATTRIBUTE,attrValue22));
      
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_ALPHA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue23 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_ALPHA_ATTRIBUTE,attrValue23));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_BETA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue24 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_BETA_ATTRIBUTE,attrValue24)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_DYNAMIC_PRESSURE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue25 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_DYNAMIC_PRESSURE_ATTRIBUTE,attrValue25)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_TANK_FILLING.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue26 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > ( _MEAS_TANK_FILLING_ATTRIBUTE,attrValue26));

    try 
    {
		_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_POSITION_ObjectHandle, ahvps_MEAS_AIRCRAFT_POSITION, MyTag1);
		_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle, ahvps_MEAS_AIRCRAFT_ORIENTATION, MyTag2);
		_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle, ahvps_MEAS_AIRCRAFT_UVW_SPEED, MyTag3);
		_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle, ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED, MyTag4);
		_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle, ahvps_MEAS_AIRCRAFT_ACCELERATION, MyTag5);
		_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_SPEED_ObjectHandle, ahvps_MEAS_AIRCRAFT_SPEED, MyTag6);
		_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle, ahvps_MEAS_AIRCRAFT_ADDITIONAL, MyTag7);
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
void SensorsFederateHla1516e::sendUpdateAttributes(RTI1516fedTime UpdateTime)
{   
		//std::cout << "FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;
	rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_POSITION;
    rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_ORIENTATION;
    rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_UVW_SPEED;
    rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED;
    rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_ACCELERATION;
    rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_SPEED;
    rti1516e::AttributeHandleValueMap ahvps_MEAS_AIRCRAFT_ADDITIONAL;  
    
    rti1516e::VariableLengthData MyTag1, MyTag2, MyTag3, MyTag4, MyTag5, MyTag6, MyTag7;
	std::string testTag ("Measured_Aircraft_Position");
	MyTag1.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag1 ("Measured_Aircraft_Orientation");
	MyTag2.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag2 ("MMeasured_Aircraft_UVW_Speed");
	MyTag3.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag3 ("Measured_Aircraft_PQR_Angular_Speed");
	MyTag4.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag4 ("Measured_Aircraft_UVW_Speed");
	MyTag5.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag5 ("Measured_Aircraft_Speed");
	MyTag6.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag6 ("Measured_Aircraft_Additional");
	MyTag7.setData (testTag.c_str (), testTag.size () + 1);
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_LONGITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LONGITUDE_ATTRIBUTE,attrValue1)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_LATITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LATITUDE_ATTRIBUTE,attrValue2)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_ALTITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_ALTITUDE_ATTRIBUTE,attrValue3)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_PHI.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ORIENTATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_PHI_ATTRIBUTE,attrValue4)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_THETA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue5 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ORIENTATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_THETA_ATTRIBUTE,attrValue5)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_PSI.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue6 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ORIENTATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_PSI_ATTRIBUTE,attrValue6));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_U_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue7 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_UVW_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_U_SPEED_ATTRIBUTE,attrValue7));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_V_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue8 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_UVW_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_V_SPEED_ATTRIBUTE,attrValue8));
    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_W_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue9 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_UVW_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_W_SPEED_ATTRIBUTE,attrValue9));
    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_P_ANG_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue10 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_P_ANG_SPEED_ATTRIBUTE,attrValue10));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_Q_ANG_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue11 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_Q_ANG_SPEED_ATTRIBUTE,attrValue11));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_R_ANG_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue12 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_R_ANG_SPEED_ATTRIBUTE,attrValue12)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_X_ACC.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue13 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ACCELERATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_X_ACC_ATTRIBUTE,attrValue13));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_Y_ACC.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue14 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ACCELERATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_Y_ACC_ATTRIBUTE,attrValue14));   
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_Z_ACC.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue15 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ACCELERATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_Z_ACC_ATTRIBUTE,attrValue15));  
    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_INDICATED_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue16 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_INDICATED_AIRSPEED_ATTRIBUTE,attrValue16));   
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_EQUIVALENT_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue17 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_EQUIVALENT_AIRSPEED_ATTRIBUTE,attrValue17));   
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_CALIBRATED_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue18 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_CALIBRATED_AIRSPEED_ATTRIBUTE,attrValue18));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_TRUE_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue19 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_TRUE_AIRSPEED_ATTRIBUTE,attrValue19));  
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_GROUND_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue20 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_GROUND_SPEED_ATTRIBUTE,attrValue20));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_VERTICAL_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue21 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_VERTICAL_SPEED_ATTRIBUTE,attrValue21));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_MACH_NUMBER.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue22 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_MACH_NUMBER_ATTRIBUTE,attrValue22));
      
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_ALPHA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue23 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_ALPHA_ATTRIBUTE,attrValue23));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_BETA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue24 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_BETA_ATTRIBUTE,attrValue24)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_DYNAMIC_PRESSURE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue25 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_DYNAMIC_PRESSURE_ATTRIBUTE,attrValue25)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_TANK_FILLING.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue26 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_MEAS_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_TANK_FILLING_ATTRIBUTE,attrValue26));

    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_POSITION_ObjectHandle, ahvps_MEAS_AIRCRAFT_POSITION, MyTag1);
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle, ahvps_MEAS_AIRCRAFT_ORIENTATION, MyTag2);
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle, ahvps_MEAS_AIRCRAFT_UVW_SPEED, MyTag3);
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle, ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED, MyTag4);
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle, ahvps_MEAS_AIRCRAFT_ACCELERATION, MyTag5);
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_SPEED_ObjectHandle, ahvps_MEAS_AIRCRAFT_SPEED, MyTag6);
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle, ahvps_MEAS_AIRCRAFT_ADDITIONAL, MyTag7);
        }
        else
        {
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_POSITION_ObjectHandle, ahvps_MEAS_AIRCRAFT_POSITION, MyTag1,  UpdateTime);
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle, ahvps_MEAS_AIRCRAFT_ORIENTATION, MyTag2, UpdateTime);
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle, ahvps_MEAS_AIRCRAFT_UVW_SPEED,MyTag3, UpdateTime);
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle, ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED, MyTag4, UpdateTime);
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle, ahvps_MEAS_AIRCRAFT_ACCELERATION, MyTag5, UpdateTime);
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_SPEED_ObjectHandle, ahvps_MEAS_AIRCRAFT_SPEED, MyTag6, UpdateTime);
			_RtiAmb->updateAttributeValues(_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle, ahvps_MEAS_AIRCRAFT_ADDITIONAL, MyTag7, UpdateTime);

        }
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void SensorsFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
void SensorsFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
				_Sensor_LONGITUDE.setInputSignal(buffer.read_double());
			}
            else if (parmHandle == _LATITUDE_ATTRIBUTE)  
            { 
				_Sensor_LATITUDE.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _ALTITUDE_ATTRIBUTE)  
            { 
				_Sensor_ALTITUDE.setInputSignal(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"SensorsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Sensor_PHI.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _THETA_ATTRIBUTE) 
            {
				_Sensor_THETA.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _PSI_ATTRIBUTE)   
            { 
				_Sensor_PSI.setInputSignal(buffer.read_double());
			}
            else
            { 
				std::wcout << L"SensorsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Sensor_U_SPEED.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _V_SPEED_ATTRIBUTE) 
            { 
				_Sensor_V_SPEED.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _W_SPEED_ATTRIBUTE) 
            { 
				_Sensor_W_SPEED.setInputSignal(buffer.read_double());  
			}
            else
            { 
				std::wcout << L"SensorsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Sensor_P_ANG_SPEED.setInputSignal(buffer.read_double());
			}
            else if (parmHandle == _Q_ANG_SPEED_ATTRIBUTE) 
            { 
				_Sensor_Q_ANG_SPEED.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _R_ANG_SPEED_ATTRIBUTE) 
            { 
				_Sensor_R_ANG_SPEED.setInputSignal(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"SensorsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Sensor_X_ACC.setInputSignal(buffer.read_double());  
			}
            else if (parmHandle == _Y_ACC_ATTRIBUTE) 
            { 
				_Sensor_Y_ACC.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _Z_ACC_ATTRIBUTE) 
            { 
				_Sensor_Z_ACC.setInputSignal(buffer.read_double());
			}
            else
            { 
				std::wcout << L"SensorsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Sensor_INDICATED_AIRSPEED.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _EQUIVALENT_AIRSPEED_ATTRIBUTE) 
            { 
				_Sensor_EQUIVALENT_AIRSPEED.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _CALIBRATED_AIRSPEED_ATTRIBUTE) 
            { 
				_Sensor_CALIBRATED_AIRSPEED.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _TRUE_AIRSPEED_ATTRIBUTE)       
            { 
				_Sensor_TRUE_AIRSPEED.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _GROUND_SPEED_ATTRIBUTE)        
            { 
				_Sensor_GROUND_SPEED.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _VERTICAL_SPEED_ATTRIBUTE)      
            { 
				_Sensor_VERTICAL_SPEED.setInputSignal(buffer.read_double());  
			}
            else if (parmHandle == _MACH_NUMBER_ATTRIBUTE)        
            { 
				_Sensor_MACH_NUMBER.setInputSignal(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"SensorsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Sensor_ALPHA.setInputSignal(buffer.read_double());
			}
            else if (parmHandle == _BETA_ATTRIBUTE)             
            { 
				_Sensor_BETA.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _DYNAMIC_PRESSURE_ATTRIBUTE) 
            { 
				_Sensor_DYNAMIC_PRESSURE.setInputSignal(buffer.read_double()); 
			}
            else if (parmHandle == _TANK_FILLING_ATTRIBUTE)     
            { 
				_Sensor_TANK_FILLING.setInputSignal(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"SensorsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }    
        _New_AIRCRAFT_ADDITIONAL = true;
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
void SensorsFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
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
void SensorsFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
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
void SensorsFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_SENSORS_FED
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void SensorsFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void SensorsFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void SensorsFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void SensorsFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void SensorsFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void SensorsFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void SensorsFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
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
void SensorsFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
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
void SensorsFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
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
void SensorsFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
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
void SensorsFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void SensorsFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void SensorsFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void SensorsFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// function : run
void SensorsFederateHla1516e::run()
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
