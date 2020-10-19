#include "FlightDynamicsFederateHla1516e.hh"

// ----------------------------------------------------------------------------
// TemplateFederateHla13 Constructor
FlightDynamicsFederateHla1516e::FlightDynamicsFederateHla1516e( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : rti1516e::NullFederateAmbassador()
											  , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _SimulationEndTime(0.0), _LocalTime(0.0), _RestOfTimeStep(0.0)
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> Constructor(): Start" << std::endl;
	#endif
	_FederationName = FederationName;
	_FederateName = FederateName;
	_FomFileName = FomFileName;
	_TimeStep = 0.0;
	_Lookahead = 0.0;
	_RestOfTimeStep = 0.0;
	_LocalTime = 0.0;
	_SimulationEndTime = 0000.0;
	_IsTimeReg = _IsTimeConst = _IsTimeAdvanceGrant = _IsSyncAnnonced=  false ;
	// Note: FD is creator for the SDSE federation
	_IsCreator = true;
	_SyncRegSuccess = _SyncRegFailed = _InPause = false ;
	_IsOutMesTimespamped = true;
	
    _Discov_ACTUATORS_CONTROL_SURFACES = false;
    _Discov_ACTUATORS_ENGINES          = false;
    _Discov_ENVIRONMENT_VARIABLES      = false;
    _Discov_WIND_COMPONENTS            = false;
    _New_ACTUATORS_CONTROL_SURFACES = false;
    _New_ACTUATORS_ENGINES          = false;
    _New_ENVIRONMENT_VARIABLES      = false;
    _New_WIND_COMPONENTS            = false;

	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> Constructor(): End" << std::endl;
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
FlightDynamicsFederateHla1516e::~FlightDynamicsFederateHla1516e()
{
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void FlightDynamicsFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> createFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void FlightDynamicsFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
														 , L"flightDyn"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::wcout << L"FlightDynamicsFederateHla1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> resignFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
rti1516e::FederateHandle FlightDynamicsFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}


// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void FlightDynamicsFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                                    , RTI1516fedTime Lookahead
                                                    , RTI1516fedTime LocalTime
                                                    , RTI1516fedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void FlightDynamicsFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed 
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
		
		_ACTUATORS_ClassHandle = _RtiAmb->getObjectClassHandle(ACTUATORS);
        _ENVIRONMENT_VARIABLES_ClassHandle = _RtiAmb->getObjectClassHandle(ENVIRONMENT_VARIABLES);
        _WIND_COMPONENTS_ClassHandle = _RtiAmb->getObjectClassHandle(WIND_COMPONENTS);
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
		_GEARS_POSITION_ATTRIBUTE = _RtiAmb->getAttributeHandle(_ACTUATORS_ClassHandle, GEARS_POSITION);
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
		
		// Published
		std::wstring TRIM_DATA (L"TRIM_DATA");
		std::wstring JOYSTICK (L"JOYSTICK");
		std::wstring COCKPIT (L"COCKPIT");
		std::wstring AIRCRAFT_POSITION (L"AIRCRAFT_POSITION");
		std::wstring AIRCRAFT_ORIENTATION (L"AIRCRAFT_ORIENTATION");
		std::wstring AIRCRAFT_UVW_SPEED (L"AIRCRAFT_UVW_SPEED");
		std::wstring AIRCRAFT_PQR_ANGULAR_SPEED (L"AIRCRAFT_PQR_ANGULAR_SPEED");
		std::wstring AIRCRAFT_ACCELERATION (L"AIRCRAFT_ACCELERATION");
		std::wstring AIRCRAFT_SPEED (L"AIRCRAFT_SPEED");
		std::wstring AIRCRAFT_ADDITIONAL (L"AIRCRAFT_ADDITIONAL");
		
        _TRIM_DATA_ClassHandle = _RtiAmb->getObjectClassHandle(TRIM_DATA);
        _AIRCRAFT_POSITION_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_POSITION);
        _AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_ORIENTATION);
        _AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_UVW_SPEED);
        _AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_PQR_ANGULAR_SPEED);
        _AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_ACCELERATION);
        _AIRCRAFT_SPEED_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_SPEED);
		_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb->getObjectClassHandle(AIRCRAFT_ADDITIONAL); 
		
		std::wstring RIGHT_ENGINE_THRUST_EQ (L"RIGHT_ENGINE_THRUST_EQ");
		std::wstring LEFT_ENGINE_THRUST_EQ (L"LEFT_ENGINE_THRUST_EQ");
		std::wstring ELEVATOR_DEFLECTION_EQ (L"ELEVATOR_DEFLECTION_EQ");
		std::wstring STABILIZER_DEFLECTION_EQ (L"STABILIZER_DEFLECTION_EQ");
		
		_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, RIGHT_ENGINE_THRUST_EQ); 
        _LEFT_ENGINE_THRUST_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, LEFT_ENGINE_THRUST_EQ);
        _ELEVATOR_DEFLECTION_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, ELEVATOR_DEFLECTION_EQ);
        _STABILIZER_DEFLECTION_EQ_ATTRIBUTE = _RtiAmb->getAttributeHandle(_TRIM_DATA_ClassHandle, STABILIZER_DEFLECTION_EQ);
        
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void FlightDynamicsFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
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
		
		_RtiAmb->subscribeObjectClassAttributes(_ACTUATORS_ClassHandle,attr_ACTUATORS);
        _RtiAmb->subscribeObjectClassAttributes(_ENVIRONMENT_VARIABLES_ClassHandle,attr_ENVIRONMENT_VARIABLES);
        _RtiAmb->subscribeObjectClassAttributes(_WIND_COMPONENTS_ClassHandle,attr_WIND_COMPONENTS);

		// For Class/Attributes published
		attr_TRIM_DATA.insert(_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE);
		attr_TRIM_DATA.insert(_LEFT_ENGINE_THRUST_EQ_ATTRIBUTE);
		attr_TRIM_DATA.insert(_ELEVATOR_DEFLECTION_EQ_ATTRIBUTE);
		attr_TRIM_DATA.insert(_STABILIZER_DEFLECTION_EQ_ATTRIBUTE);
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
		
        _RtiAmb->publishObjectClassAttributes(_TRIM_DATA_ClassHandle,attr_TRIM_DATA);
        _RtiAmb->publishObjectClassAttributes(_AIRCRAFT_POSITION_ClassHandle,attr_AIRCRAFT_POSITION);
        _RtiAmb->publishObjectClassAttributes(_AIRCRAFT_ORIENTATION_ClassHandle,attr_AIRCRAFT_ORIENTATION);
        _RtiAmb->publishObjectClassAttributes(_AIRCRAFT_UVW_SPEED_ClassHandle,attr_AIRCRAFT_UVW_SPEED);
        _RtiAmb->publishObjectClassAttributes(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,attr_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb->publishObjectClassAttributes(_AIRCRAFT_ACCELERATION_ClassHandle,attr_AIRCRAFT_ACCELERATION);
        _RtiAmb->publishObjectClassAttributes(_AIRCRAFT_SPEED_ClassHandle,attr_AIRCRAFT_SPEED);
        _RtiAmb->publishObjectClassAttributes(_AIRCRAFT_ADDITIONAL_ClassHandle,attr_AIRCRAFT_ADDITIONAL);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void FlightDynamicsFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		std::wstring Trim_Data_Flight_Dynamics (L"Trim_Data_Flight_Dynamics");
		std::wstring Aircraft_Position (L"Aircraft_Position");
		std::wstring Aircraft_Orientation (L"Aircraft_Orientation");
		std::wstring Aircraft_UVW_Speed (L"Aircraft_UVW_Speed");
		std::wstring Aircraft_PQR_Angular_Speed (L"Aircraft_PQR_Angular_Speed");
		std::wstring Aircraft_Acceleration (L"Aircraft_Acceleration");
		std::wstring Aircraft_Speed (L"Aircraft_Speed");
		std::wstring Aircraft_Additional (L"Aircraft_Additional");
		
        _TRIM_DATA_ObjectHandle = _RtiAmb->registerObjectInstance(_TRIM_DATA_ClassHandle,Trim_Data_Flight_Dynamics);
        _AIRCRAFT_POSITION_ObjectHandle = _RtiAmb->registerObjectInstance(_AIRCRAFT_POSITION_ClassHandle,Aircraft_Position);
        _AIRCRAFT_ORIENTATION_ObjectHandle = _RtiAmb->registerObjectInstance(_AIRCRAFT_ORIENTATION_ClassHandle,Aircraft_Orientation);
        _AIRCRAFT_UVW_SPEED_ObjectHandle = _RtiAmb->registerObjectInstance(_AIRCRAFT_UVW_SPEED_ClassHandle,Aircraft_UVW_Speed);
        _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle = _RtiAmb->registerObjectInstance(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,Aircraft_PQR_Angular_Speed);
        _AIRCRAFT_ACCELERATION_ObjectHandle = _RtiAmb->registerObjectInstance(_AIRCRAFT_ACCELERATION_ClassHandle,Aircraft_Acceleration);
        _AIRCRAFT_SPEED_ObjectHandle = _RtiAmb->registerObjectInstance(_AIRCRAFT_SPEED_ClassHandle,Aircraft_Speed);
        _AIRCRAFT_ADDITIONAL_ObjectHandle = _RtiAmb->registerObjectInstance(_AIRCRAFT_ADDITIONAL_ClassHandle,Aircraft_Additional);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void FlightDynamicsFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb->unsubscribeObjectClass(_ACTUATORS_ClassHandle);
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
       
    try 
    {
        _RtiAmb->unpublishObjectClass(_TRIM_DATA_ClassHandle);
        _RtiAmb->unpublishObjectClass(_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb->unpublishObjectClass(_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb->unpublishObjectClass(_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb->unpublishObjectClass(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb->unpublishObjectClass(_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb->unpublishObjectClass(_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb->unpublishObjectClass(_AIRCRAFT_ADDITIONAL_ClassHandle);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void FlightDynamicsFederateHla1516e::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_Discov_ACTUATORS_CONTROL_SURFACES ||
            !_Discov_ACTUATORS_ENGINES          ||
            !_Discov_ENVIRONMENT_VARIABLES      ||
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
	_Discov_ACTUATORS_CONTROL_SURFACES = false;
    _Discov_ACTUATORS_ENGINES = false;
    _Discov_ENVIRONMENT_VARIABLES = false;
    _Discov_WIND_COMPONENTS = false;
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void FlightDynamicsFederateHla1516e::waitForAllAttributesReceived()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_ENVIRONMENT_VARIABLES)
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
    _New_ENVIRONMENT_VARIABLES = false;
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void FlightDynamicsFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_FLIGHT_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    std::wstring Actuators_Control_Surfaces (L"Actuators_Control_Surfaces");
    std::wstring Actuators_Engines (L"Actuators_Engines");
    std::wstring Environment_Variables (L"Environment_Variables");
    std::wstring Wind_Components (L"Wind_Components");
    
    if ( (theObjectClass == _ACTUATORS_ClassHandle) &&  (!wcscmp(theObjectInstanceName.c_str(),Actuators_Control_Surfaces.c_str())) ) 
	{
		_Discov_ACTUATORS_CONTROL_SURFACES = true;
		_ACTUATORS_CONTROL_SURFACES_ObjectHandle = theObject;
		#ifdef DEBUG_FLIGH_DYNAMICS_FED
		std::wcout << L"< Actuators_Control_Surfaces > Object Instance has been discovered" << std::endl;
		#endif
	} 
	else if ( (theObjectClass == _ACTUATORS_ClassHandle) &&  (!wcscmp(theObjectInstanceName.c_str(),Actuators_Engines.c_str())) ) 
	{
		_Discov_ACTUATORS_ENGINES = true;
		_ACTUATORS_ENGINES_ObjectHandle = theObject;
		#ifdef DEBUG_FLIGH_DYNAMICS_FED
		std::wcout << L"< Actuators_Engines > Object Instance has been discovered" << std::endl;
		#endif
	} 
	else if ( (theObjectClass == _ENVIRONMENT_VARIABLES_ClassHandle) &&  (!wcscmp(theObjectInstanceName.c_str(),Environment_Variables.c_str())) ) 
	{
		_Discov_ENVIRONMENT_VARIABLES = true;
		_ENVIRONMENT_VARIABLES_ObjectHandle = theObject;
		#ifdef DEBUG_FLIGH_DYNAMICS_FED
		std::wcout << L"< Environment_Variables > Object Instance has been discovered" << std::endl;
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
    #ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> discoverObjectInstance(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void FlightDynamicsFederateHla1516e::pauseInitTrim()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Simu Sync
void FlightDynamicsFederateHla1516e::pauseInitSim()
{
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> pauseInitSim(): Start" << std::endl;
    #endif
	std::wstring SimString (L"Simulating");
    if (_IsCreator) 
    {
		_SyncRegSuccess = false;
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
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
    std::wcout << L"FlightDynamicsFederateHla1516e.cc -> pauseInitSim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void FlightDynamicsFederateHla1516e::initialization() 
{
	waitForAllObjectDiscovered();
	sendInitialAttributesPos();
	waitForAllAttributesReceived();
	_Aircraft.initialization(); 
	sendInitialAttributesTrim();
	sendInitialAllAttributes();
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void FlightDynamicsFederateHla1516e::calculateState() 
{
	// Model State calculation
	_Aircraft.calculate_state();
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void FlightDynamicsFederateHla1516e::calculateOutput() 
{
	// Model output calculation
	_Aircraft.calculate_output(); 
}

// ----------------------------------------------------------------------------
// Send Updates for init attributes position
void FlightDynamicsFederateHla1516e::sendInitialAttributesPos()
{
	// update attribute for initial AIRCRAFT_POSITION 
	//std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_AIRCRAFT_POSITION(RTI::AttributeSetFactory::create(3));
	rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_POSITION;
	rti1516e::VariableLengthData MyTag;
	std::string testTag ("Aircraft_Position");
	MyTag.setData (testTag.c_str (), testTag.size () + 1);
	
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getLongitude());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LONGITUDE_ATTRIBUTE,attrValue1)); 
	
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getLatitude());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LATITUDE_ATTRIBUTE,attrValue2));
	
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAltitude());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_ALTITUDE_ATTRIBUTE,attrValue3));

    try 
    {
		_RtiAmb->updateAttributeValues(_AIRCRAFT_POSITION_ObjectHandle, ahvps_AIRCRAFT_POSITION, MyTag);

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
// Send Updates for init attributes position
void FlightDynamicsFederateHla1516e::sendInitialAttributesTrim()
{
	rti1516e::AttributeHandleValueMap ahvps_TRIM_DATA;
	rti1516e::VariableLengthData MyTag;
	std::string testTag ("Trim_Data");
	MyTag.setData (testTag.c_str (), testTag.size () + 1);

    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getThrustRightEngine());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_TRIM_DATA.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE,attrValue1)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getThrustLeftEngine());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_TRIM_DATA.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LEFT_ENGINE_THRUST_EQ_ATTRIBUTE,attrValue2)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getDeltaRightElevatorRad());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_TRIM_DATA.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_ELEVATOR_DEFLECTION_EQ_ATTRIBUTE,attrValue3)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getDeltaHSRad());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_TRIM_DATA.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_STABILIZER_DEFLECTION_EQ_ATTRIBUTE,attrValue4));

    try 
    {
		_RtiAmb->updateAttributeValues(_TRIM_DATA_ObjectHandle, ahvps_TRIM_DATA, MyTag);

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
// Send All Updates for to init FD Attributes in the whole federation
void FlightDynamicsFederateHla1516e::sendInitialAllAttributes()
{
	//std::cout << "FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;
	rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_POSITION;
    rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_ORIENTATION;
    rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_UVW_SPEED;
    rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_PQR_ANGULAR_SPEED;
    rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_ACCELERATION;
    rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_SPEED;
    rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_ADDITIONAL;  
    
    rti1516e::VariableLengthData MyTag1, MyTag2, MyTag3, MyTag4, MyTag5, MyTag6, MyTag7;
	std::string testTag ("Aircraft_Position");
	MyTag1.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag1 ("Aircraft_Orientation");
	MyTag2.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag2 ("Aircraft_UVW_Speed");
	MyTag3.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag3 ("Aircraft_PQR_Angular_Speed");
	MyTag4.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag4 ("Aircraft_UVW_Speed");
	MyTag5.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag5 ("Aircraft_Speed");
	MyTag6.setData (testTag.c_str (), testTag.size () + 1);
	std::string testTag6 ("Aircraft_Additional");
	MyTag7.setData (testTag.c_str (), testTag.size () + 1);
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getLongitude());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LONGITUDE_ATTRIBUTE,attrValue1)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getLatitude());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LATITUDE_ATTRIBUTE,attrValue2)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAltitude());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_ALTITUDE_ATTRIBUTE,attrValue3)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getPhi());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ORIENTATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_PHI_ATTRIBUTE,attrValue4)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getTheta());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue5 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ORIENTATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_THETA_ATTRIBUTE,attrValue5)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getPsi());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue6 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ORIENTATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_PSI_ATTRIBUTE,attrValue6));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getU());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue7 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_UVW_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_U_SPEED_ATTRIBUTE,attrValue7));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getV());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue8 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_UVW_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_V_SPEED_ATTRIBUTE,attrValue8));
    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getW());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue9 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_UVW_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_W_SPEED_ATTRIBUTE,attrValue9));
    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getP());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue10 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_PQR_ANGULAR_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_P_ANG_SPEED_ATTRIBUTE,attrValue10));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getQ());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue11 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_PQR_ANGULAR_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_Q_ANG_SPEED_ATTRIBUTE,attrValue11));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getR());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue12 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_PQR_ANGULAR_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_R_ANG_SPEED_ATTRIBUTE,attrValue12)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAx());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue13 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ACCELERATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_X_ACC_ATTRIBUTE,attrValue13));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAy());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue14 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ACCELERATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_Y_ACC_ATTRIBUTE,attrValue14));   
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAz());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue15 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ACCELERATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_Z_ACC_ATTRIBUTE,attrValue15));  
    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getIAS());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue16 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_INDICATED_AIRSPEED_ATTRIBUTE,attrValue16));   
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getEAS());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue17 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_EQUIVALENT_AIRSPEED_ATTRIBUTE,attrValue17));   
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getCAS());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue18 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_CALIBRATED_AIRSPEED_ATTRIBUTE,attrValue18));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getTAS());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue19 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_TRUE_AIRSPEED_ATTRIBUTE,attrValue19));  
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getGS());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue20 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_GROUND_SPEED_ATTRIBUTE,attrValue20));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getVS());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue21 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_VERTICAL_SPEED_ATTRIBUTE,attrValue21));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getMach());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue22 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_MACH_NUMBER_ATTRIBUTE,attrValue22));
      
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAlpha());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue23 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_ALPHA_ATTRIBUTE,attrValue23));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getBeta());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue24 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_BETA_ATTRIBUTE,attrValue24)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getQbar());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue25 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_DYNAMIC_PRESSURE_ATTRIBUTE,attrValue25)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getTank());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue26 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_TANK_FILLING_ATTRIBUTE,attrValue26));

    try 
    {
		_RtiAmb->updateAttributeValues(_AIRCRAFT_POSITION_ObjectHandle, ahvps_AIRCRAFT_POSITION, MyTag1);
		_RtiAmb->updateAttributeValues(_AIRCRAFT_ORIENTATION_ObjectHandle, ahvps_AIRCRAFT_ORIENTATION, MyTag2);
		_RtiAmb->updateAttributeValues(_AIRCRAFT_UVW_SPEED_ObjectHandle, ahvps_AIRCRAFT_UVW_SPEED, MyTag3);
		_RtiAmb->updateAttributeValues(_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle, ahvps_AIRCRAFT_PQR_ANGULAR_SPEED, MyTag4);
		_RtiAmb->updateAttributeValues(_AIRCRAFT_ACCELERATION_ObjectHandle, ahvps_AIRCRAFT_ACCELERATION, MyTag5);
		_RtiAmb->updateAttributeValues(_AIRCRAFT_SPEED_ObjectHandle, ahvps_AIRCRAFT_SPEED, MyTag6);
		_RtiAmb->updateAttributeValues(_AIRCRAFT_ADDITIONAL_ObjectHandle, ahvps_AIRCRAFT_ADDITIONAL, MyTag7);
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
void FlightDynamicsFederateHla1516e::sendUpdateAttributes1(RTI1516fedTime UpdateTime)
{ 
	rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_POSITION;
    rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_ORIENTATION;
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getLongitude());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue1 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LONGITUDE_ATTRIBUTE,attrValue1)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getLatitude());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue2 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_LATITUDE_ATTRIBUTE,attrValue2)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAltitude());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue3 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_POSITION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_ALTITUDE_ATTRIBUTE,attrValue3)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getPhi());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue4 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ORIENTATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_PHI_ATTRIBUTE,attrValue4)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getTheta());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue5 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ORIENTATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_THETA_ATTRIBUTE,attrValue5)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getPsi());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue6 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ORIENTATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_PSI_ATTRIBUTE,attrValue6));
    
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb->updateAttributeValues(_AIRCRAFT_POSITION_ObjectHandle, ahvps_AIRCRAFT_POSITION, _MyTag);
            _RtiAmb->updateAttributeValues(_AIRCRAFT_ORIENTATION_ObjectHandle, ahvps_AIRCRAFT_ORIENTATION, _MyTag);
        }
        else
        {
            _RtiAmb->updateAttributeValues(_AIRCRAFT_POSITION_ObjectHandle, ahvps_AIRCRAFT_POSITION, _MyTag, UpdateTime);
            _RtiAmb->updateAttributeValues(_AIRCRAFT_ORIENTATION_ObjectHandle, ahvps_AIRCRAFT_ORIENTATION, _MyTag, UpdateTime);
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
// Send Updates for all attributes
void FlightDynamicsFederateHla1516e::sendUpdateAttributes2(RTI1516fedTime UpdateTime)
{ 
	//std::cout << "FlightDynamicsFederateHla13::sendUpdateAttributes: Start" << std::endl;
    rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_UVW_SPEED;
    rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_PQR_ANGULAR_SPEED;
    rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_ACCELERATION;
    rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_SPEED;
    rti1516e::AttributeHandleValueMap ahvps_AIRCRAFT_ADDITIONAL; 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getU());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue7 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_UVW_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_U_SPEED_ATTRIBUTE,attrValue7));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getV());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue8 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_UVW_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_V_SPEED_ATTRIBUTE,attrValue8));
    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getW());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue9 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_UVW_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_W_SPEED_ATTRIBUTE,attrValue9));
    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getP());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue10 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_PQR_ANGULAR_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_P_ANG_SPEED_ATTRIBUTE,attrValue10));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getQ());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue11 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_PQR_ANGULAR_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_Q_ANG_SPEED_ATTRIBUTE,attrValue11));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getR());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue12 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_PQR_ANGULAR_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_R_ANG_SPEED_ATTRIBUTE,attrValue12)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAx());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue13 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ACCELERATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_X_ACC_ATTRIBUTE,attrValue13));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAy());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue14 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ACCELERATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_Y_ACC_ATTRIBUTE,attrValue14));   
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAz());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue15 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ACCELERATION.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_Z_ACC_ATTRIBUTE,attrValue15));  
    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getIAS());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue16 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_INDICATED_AIRSPEED_ATTRIBUTE,attrValue16));   
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getEAS());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue17 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_EQUIVALENT_AIRSPEED_ATTRIBUTE,attrValue17));   
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getCAS());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue18 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_CALIBRATED_AIRSPEED_ATTRIBUTE,attrValue18));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getTAS());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue19 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_TRUE_AIRSPEED_ATTRIBUTE,attrValue19));  
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getGS());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue20 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_GROUND_SPEED_ATTRIBUTE,attrValue20));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getVS());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue21 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_VERTICAL_SPEED_ATTRIBUTE,attrValue21));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getMach());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue22 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_SPEED.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_MACH_NUMBER_ATTRIBUTE,attrValue22));
      
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getAlpha());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue23 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_ALPHA_ATTRIBUTE,attrValue23));
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getBeta());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue24 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_BETA_ATTRIBUTE,attrValue24)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getQbar());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue25 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_DYNAMIC_PRESSURE_ATTRIBUTE,attrValue25)); 
    
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Aircraft.getTank());
    _OutputMessagebuffer.updateReservedBytes();
    rti1516e::VariableLengthData attrValue26 (static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	ahvps_AIRCRAFT_ADDITIONAL.insert(std::pair < rti1516e::AttributeHandle, rti1516e::VariableLengthData > (_TANK_FILLING_ATTRIBUTE,attrValue26));

    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb->updateAttributeValues(_AIRCRAFT_UVW_SPEED_ObjectHandle, ahvps_AIRCRAFT_UVW_SPEED, _MyTag);
            _RtiAmb->updateAttributeValues(_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle, ahvps_AIRCRAFT_PQR_ANGULAR_SPEED, _MyTag);
            _RtiAmb->updateAttributeValues(_AIRCRAFT_ACCELERATION_ObjectHandle, ahvps_AIRCRAFT_ACCELERATION, _MyTag);
            _RtiAmb->updateAttributeValues(_AIRCRAFT_SPEED_ObjectHandle, ahvps_AIRCRAFT_SPEED, _MyTag);
            _RtiAmb->updateAttributeValues(_AIRCRAFT_ADDITIONAL_ObjectHandle, ahvps_AIRCRAFT_ADDITIONAL, _MyTag);
        }
        else
        {
            _RtiAmb->updateAttributeValues(_AIRCRAFT_UVW_SPEED_ObjectHandle, ahvps_AIRCRAFT_UVW_SPEED, _MyTag, UpdateTime);
            _RtiAmb->updateAttributeValues(_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle, ahvps_AIRCRAFT_PQR_ANGULAR_SPEED, _MyTag, UpdateTime);
            _RtiAmb->updateAttributeValues(_AIRCRAFT_ACCELERATION_ObjectHandle, ahvps_AIRCRAFT_ACCELERATION, _MyTag, UpdateTime);
            _RtiAmb->updateAttributeValues(_AIRCRAFT_SPEED_ObjectHandle, ahvps_AIRCRAFT_SPEED, _MyTag, UpdateTime);
            _RtiAmb->updateAttributeValues(_AIRCRAFT_ADDITIONAL_ObjectHandle, ahvps_AIRCRAFT_ADDITIONAL, _MyTag, UpdateTime);
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
void FlightDynamicsFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
void FlightDynamicsFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
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
    
    if (theObject == _ACTUATORS_CONTROL_SURFACES_ObjectHandle)
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
				_Aircraft.setDeltaRightAileronRad(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE)   
            { 
				_Aircraft.setDeltaLeftAileronRad(buffer.read_double()); 
			}
            else if (parmHandle == _RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE) 
            { 
				_Aircraft.setDeltaRightElevatorRad(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE)  
            { 
				_Aircraft.setDeltaLeftElevatorRad(buffer.read_double()); 
			}
            else if (parmHandle == _RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE)
            { 
				_Aircraft.setDeltaRudderRad(buffer.read_double()); 
			}
            else if (parmHandle == _FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE)  
            { 
				_Aircraft.setDeltaFlapRad(buffer.read_double()); 
			}
            else if (parmHandle == _SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE)  
            { 
				_Aircraft.setDeltaSpoilersRad(buffer.read_double()); 
			}
            else if (parmHandle == _STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE)  
            { 
				_Aircraft.setDeltaHSRad(buffer.read_double()); 
			}
            else if (parmHandle == _GEARS_POSITION_ATTRIBUTE)  
            { 
				_Aircraft.setDeltaGear(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"FlightDynamicsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Aircraft.setThrustRightEngine(buffer.read_double()); 
			}
            else if (parmHandle == _LEFT_ENGINE_THRUST_ATTRIBUTE)  
            { 
				_Aircraft.setThrustLeftEngine(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"FlightDynamicsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_ACTUATORS_ENGINES = true;
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
				_Aircraft.setTemperature(buffer.read_double()); 
			}
            else if (parmHandle == _DENSITY_OF_AIR_ATTRIBUTE) 
            { 
				_Aircraft.setDensity(buffer.read_double()); 
			}
            else if (parmHandle == _PRESSURE_ATTRIBUTE)       
            { 
				_Aircraft.setPressure(buffer.read_double()); 
			}
            else if (parmHandle == _SPEED_OF_SOUND_ATTRIBUTE) 
            { 
				_Aircraft.setSoundSpeed(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"FlightDynamicsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
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
				_Aircraft.setUwind(buffer.read_double()); 
			}
            else if (parmHandle == _V_WIND_ATTRIBUTE) 
            { 
				_Aircraft.setVwind(buffer.read_double()); 
			}
            else if (parmHandle == _W_WIND_ATTRIBUTE) 
            { 
				_Aircraft.setWwind(buffer.read_double()); 
			}
            else if (parmHandle == _P_WIND_ATTRIBUTE) 
            { 
				_Aircraft.setPwind(buffer.read_double()); 
			} 
            else if (parmHandle == _Q_WIND_ATTRIBUTE) 
            { 
				_Aircraft.setQwind(buffer.read_double()); 
			}
            else if (parmHandle == _R_WIND_ATTRIBUTE) 
            { 
				_Aircraft.setRwind(buffer.read_double()); 
			}
            else
            { 
				std::wcout << L"FlightDynamicsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << std::endl; 
			}
        }
        _New_WIND_COMPONENTS = true;
    }
	else
	{ 
		std::wcout << L"FlightDynamicsFederateHla1516e.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << std::endl; 
	} 
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void FlightDynamicsFederateHla1516e::timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
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
void FlightDynamicsFederateHla1516e::timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
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
void FlightDynamicsFederateHla1516e::timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
throw (rti1516e::FederateInternalError) 
/*
    throw ( rti1516e::InvalidLogicalTime,
            rti1516e::JoinedFederateIsNotInTimeAdvancingState,
            rti1516e::FederateInternalError) */
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_FLIGH_DYNAMICS_FED
	std::wcout << L" >> TAG RECU == LocalTime = " <<  _LocalTime.getFedTime() << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::enableTimeRegulation()
{
	RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
	_RtiAmb->enableTimeRegulation(lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::disableTimeRegulation()
{
	_RtiAmb->disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::enableTimeConstrained()
{
	_RtiAmb->enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is evokeCallback2 the correct option...
		_RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
	}
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::disableTimeConstrained()
{
	_RtiAmb->disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb->enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::disableAsynchronousDelivery()
{
	_RtiAmb->disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void FlightDynamicsFederateHla1516e::timeAdvanceRequest(RTI1516fedTime NextLogicalTime)
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
void FlightDynamicsFederateHla1516e::timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime)
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
void FlightDynamicsFederateHla1516e::nextEventRequest(RTI1516fedTime NextLogicalTime)
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
void FlightDynamicsFederateHla1516e::nextEventAvailable(RTI1516fedTime NextLogicalTime)
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
void FlightDynamicsFederateHla1516e::synchronizationPointRegistrationSucceeded(std::wstring const &label)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void FlightDynamicsFederateHla1516e::synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
    throw ( rti1516e::FederateInternalError) 
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void FlightDynamicsFederateHla1516e::announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag)
    throw ( rti1516e::FederateInternalError) 
{
       _InPause = true ;
       _IsSyncAnnonced = true;
        std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void FlightDynamicsFederateHla1516e::federationSynchronized(std::wstring const &label,rti1516e::FederateHandleSet const& failedToSyncSet)
    throw ( rti1516e::FederateInternalError) 
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
} 

// ----------------------------------------------------------------------------
// function : run
void FlightDynamicsFederateHla1516e::run()
{
	RTI1516fedTime updateTime(0.0);
	while (_LocalTime < _SimulationEndTime)
	{
		// Model calculations
		calculateState();
		updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
		sendUpdateAttributes1(updateTime); 
		calculateOutput();
		nextEventRequest(_TimeStep);
		updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
		sendUpdateAttributes2(updateTime);
		timeAdvanceRequest(_TimeStep);
	}
} 

// ----------------------------------------------------------------------------
// function : run
void FlightDynamicsFederateHla1516e::runOneStep()
{
	RTI1516fedTime updateTime(0.0);
	// Model calculations
	calculateState();
	updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
	sendUpdateAttributes1(updateTime); 
	calculateOutput();
	nextEventRequest(_TimeStep);
	updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
	sendUpdateAttributes2(updateTime);
	timeAdvanceRequest(_TimeStep);

} 

bool FlightDynamicsFederateHla1516e::getEndOfSimulation()
{
	return (_LocalTime < _SimulationEndTime);
}
