// ----------------------------------------------------------------------------
// OpenSDSE - HLA Compliant Distributed Aircraft Simulation
// Copyright (C) 2017  ISAE
//
// This program is free software ; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation ; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY ; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program ; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
// ----------------------------------------------------------------------------

#include "SensorsFederateHla13.hh"

// ----------------------------------------------------------------------------
// SensorsFederateHla13 Constructor
SensorsFederateHla13::SensorsFederateHla13( std::wstring FederationName
									, std::wstring FederateName
									, std::wstring FomFileName
									) 
									: NullFederateAmbassador()
									, _RtiAmb() 
{
	#ifdef DEBUG_SENSORS_FED
	std::cout << "SensorsFederateHla13.cc -> Constructor(): Start" << std::endl;
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
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = false ;
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
	std::cout << "SensorsFederateHla13.cc -> Constructor(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// SensorsFederateHla13 Destructor
SensorsFederateHla13::~SensorsFederateHla13()
	                     throw(RTI::FederateInternalError)
{

}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void SensorsFederateHla13::createFederationExecution() 
{
	#ifdef DEBUG_SENSORS_FED
	std::cout << "SensorsFederateHla13.cc -> createFederationExecution(): Start" << std::endl;
	#endif
    try 
    {
        _RtiAmb.createFederationExecution( getString(_FederationName).c_str()
										 , getString(_FomFileName).c_str()
										 );
    } 
    catch ( RTI::FederationExecutionAlreadyExists ) 
    {
		std::cout << "CFE: Federation \"" << getString(_FederationName).c_str() << "\" already created by another federate." << std::endl;
	} 
	catch ( RTI::Exception &e ) 
	{
		std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
	} 
	catch ( ... ) 
	{
		std::cerr << "Error: unknown non-RTI exception." << std::endl;
	}
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> createFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void SensorsFederateHla13::destroyFederationExecution() 
{
	#ifdef DEBUG_SENSORS_FED
	std::cout << "SensorsFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
	if (_IsCreator)
	{
		try 
		{
			_RtiAmb.destroyFederationExecution(getString(_FederationName).c_str());
		}
		catch (RTI::Exception& e) 
		{
			std::cout << "DFE: caught " << e._name << " reason " << e._reason <<std::endl;
		}
	}
	#ifdef DEBUG_SENSORS_FED
	std::cout << "SensorsFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void SensorsFederateHla13::joinFederationExecution() 
{
	#ifdef DEBUG_SENSORS_FED
	std::cout << "SensorsFederateHla13.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb.joinFederationExecution( getString(_FederateName).c_str()
                                                         , getString(_FederationName).c_str()
                                                         ,this
                                                         );
    }
    catch (RTI::Exception& e) 
    {
        std::cout << "JFE: caught " << e._name << " reason " << e._reason <<std::endl;
    }
    #ifdef DEBUG_SENSORS_FED
	std::cout << "SensorsFederateHla13.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void SensorsFederateHla13::resignFederationExecution() 
{
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb.resignFederationExecution(RTI::DELETE_OBJECTS_AND_RELEASE_ATTRIBUTES);
	}
	catch (RTI::Exception& e) 
	{
		std::cout << "RFE: caught " << e._name << " reason " << e._reason <<std::endl;
	}
	#ifdef DEBUG_SENSORS_FED
	std::cout << "SensorsFederateHla13.cc -> resignFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
RTI::FederateHandle SensorsFederateHla13::getFederateHandle() const
{
    return _FederateHandle;
}

// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void SensorsFederateHla13::setHlaTimeManagementSettings( RTIfedTime TimeStep
                                                    , RTIfedTime Lookahead
                                                    , RTIfedTime LocalTime
                                                    , RTIfedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void SensorsFederateHla13::getAllHandles()
{
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed
		_AIRCRAFT_POSITION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_POSITION");
		_AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ORIENTATION");
		_AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_UVW_SPEED");
		_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_PQR_ANGULAR_SPEED");
		_AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ACCELERATION");
		_AIRCRAFT_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_SPEED");
		_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ADDITIONAL");
        _LONGITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("LONGITUDE",_AIRCRAFT_POSITION_ClassHandle);
        _LATITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("LATITUDE", _AIRCRAFT_POSITION_ClassHandle);
        _ALTITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("ALTITUDE",_AIRCRAFT_POSITION_ClassHandle);     
        _PHI_ATTRIBUTE = _RtiAmb.getAttributeHandle("PHI", _AIRCRAFT_ORIENTATION_ClassHandle);
        _THETA_ATTRIBUTE = _RtiAmb.getAttributeHandle("THETA",_AIRCRAFT_ORIENTATION_ClassHandle);
        _PSI_ATTRIBUTE = _RtiAmb.getAttributeHandle("PSI", _AIRCRAFT_ORIENTATION_ClassHandle);     
        _U_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("U_SPEED",_AIRCRAFT_UVW_SPEED_ClassHandle);
        _V_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("V_SPEED",_AIRCRAFT_UVW_SPEED_ClassHandle);
        _W_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("W_SPEED",_AIRCRAFT_UVW_SPEED_ClassHandle);
        _P_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("P_ANG_SPEED",_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _Q_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("Q_ANG_SPEED",_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _R_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("R_ANG_SPEED",_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);    
        _X_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("X_ACC",_AIRCRAFT_ACCELERATION_ClassHandle);
        _Y_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("Y_ACC",_AIRCRAFT_ACCELERATION_ClassHandle);
        _Z_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("Z_ACC",_AIRCRAFT_ACCELERATION_ClassHandle);      
        _INDICATED_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("INDICATED_AIRSPEED",_AIRCRAFT_SPEED_ClassHandle);
        _EQUIVALENT_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("EQUIVALENT_AIRSPEED",_AIRCRAFT_SPEED_ClassHandle);
        _CALIBRATED_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("CALIBRATED_AIRSPEED",_AIRCRAFT_SPEED_ClassHandle);
        _TRUE_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("TRUE_AIRSPEED", _AIRCRAFT_SPEED_ClassHandle);
        _GROUND_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("GROUND_SPEED", _AIRCRAFT_SPEED_ClassHandle);
        _VERTICAL_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("VERTICAL_SPEED", _AIRCRAFT_SPEED_ClassHandle);
        _MACH_NUMBER_ATTRIBUTE = _RtiAmb.getAttributeHandle("MACH_NUMBER", _AIRCRAFT_SPEED_ClassHandle);  
        _ALPHA_ATTRIBUTE = _RtiAmb.getAttributeHandle("ALPHA", _AIRCRAFT_ADDITIONAL_ClassHandle);
        _BETA_ATTRIBUTE = _RtiAmb.getAttributeHandle("BETA", _AIRCRAFT_ADDITIONAL_ClassHandle);
        _DYNAMIC_PRESSURE_ATTRIBUTE = _RtiAmb.getAttributeHandle("DYNAMIC_PRESSURE",_AIRCRAFT_ADDITIONAL_ClassHandle); 
        _TANK_FILLING_ATTRIBUTE = _RtiAmb.getAttributeHandle("TANK_FILLING",    _AIRCRAFT_ADDITIONAL_ClassHandle);
        // Published
		_MEAS_AIRCRAFT_POSITION_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_POSITION");
		_MEAS_AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_ORIENTATION");
		_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_UVW_SPEED");
		_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_PQR_ANGULAR_SPEED");
		_MEAS_AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_ACCELERATION");
		_MEAS_AIRCRAFT_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_SPEED");
		_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb.getObjectClassHandle("MEAS_AIRCRAFT_ADDITIONAL");
        _MEAS_LONGITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_LONGITUDE",_MEAS_AIRCRAFT_POSITION_ClassHandle);
        _MEAS_LATITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_LATITUDE", _MEAS_AIRCRAFT_POSITION_ClassHandle);
        _MEAS_ALTITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_ALTITUDE",_MEAS_AIRCRAFT_POSITION_ClassHandle);     
        _MEAS_PHI_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_PHI", _MEAS_AIRCRAFT_ORIENTATION_ClassHandle);
        _MEAS_THETA_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_THETA",_MEAS_AIRCRAFT_ORIENTATION_ClassHandle);
        _MEAS_PSI_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_PSI", _MEAS_AIRCRAFT_ORIENTATION_ClassHandle);     
        _MEAS_U_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_U_SPEED",_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _MEAS_V_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_V_SPEED",_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _MEAS_W_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_W_SPEED",_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _MEAS_P_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_P_ANG_SPEED",_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _MEAS_Q_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_Q_ANG_SPEED",_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _MEAS_R_ANG_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_R_ANG_SPEED",_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);    
        _MEAS_X_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_X_ACC",_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);
        _MEAS_Y_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_Y_ACC",_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);
        _MEAS_Z_ACC_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_Z_ACC",_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);      
        _MEAS_INDICATED_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_INDICATED_AIRSPEED",_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_EQUIVALENT_AIRSPEED",_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_CALIBRATED_AIRSPEED",_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _MEAS_TRUE_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_TRUE_AIRSPEED", _MEAS_AIRCRAFT_SPEED_ClassHandle);
        _MEAS_GROUND_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_GROUND_SPEED", _MEAS_AIRCRAFT_SPEED_ClassHandle);
        _MEAS_VERTICAL_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_VERTICAL_SPEED", _MEAS_AIRCRAFT_SPEED_ClassHandle);
        _MEAS_MACH_NUMBER_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_MACH_NUMBER", _MEAS_AIRCRAFT_SPEED_ClassHandle);  
        _MEAS_ALPHA_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_ALPHA", _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
        _MEAS_BETA_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_BETA", _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
        _MEAS_DYNAMIC_PRESSURE_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_DYNAMIC_PRESSURE",_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle); 
        _MEAS_TANK_FILLING_ATTRIBUTE = _RtiAmb.getAttributeHandle("MEAS_TANK_FILLING",    _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle); 
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void SensorsFederateHla13::publishAndSubscribe()
{
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		attr_AIRCRAFT_POSITION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_ORIENTATION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_UVW_SPEED.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_PQR_ANGULAR_SPEED.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_ACCELERATION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_SPEED.reset(RTI::AttributeHandleSetFactory::create(7));
		attr_AIRCRAFT_ADDITIONAL.reset(RTI::AttributeHandleSetFactory::create(4));
		attr_AIRCRAFT_POSITION->add(_LONGITUDE_ATTRIBUTE);
		attr_AIRCRAFT_POSITION->add(_LATITUDE_ATTRIBUTE);
		attr_AIRCRAFT_POSITION->add(_ALTITUDE_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION->add(_PHI_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION->add(_THETA_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION->add(_PSI_ATTRIBUTE);   
		attr_AIRCRAFT_UVW_SPEED->add(_U_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_UVW_SPEED->add(_V_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_UVW_SPEED->add(_W_SPEED_ATTRIBUTE);  
		attr_AIRCRAFT_PQR_ANGULAR_SPEED->add(_P_ANG_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_PQR_ANGULAR_SPEED->add(_Q_ANG_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_PQR_ANGULAR_SPEED->add(_R_ANG_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_ACCELERATION->add(_X_ACC_ATTRIBUTE);
		attr_AIRCRAFT_ACCELERATION->add(_Y_ACC_ATTRIBUTE);
		attr_AIRCRAFT_ACCELERATION->add(_Z_ACC_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_INDICATED_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_EQUIVALENT_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_CALIBRATED_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_TRUE_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_GROUND_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_VERTICAL_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_MACH_NUMBER_ATTRIBUTE);
		attr_AIRCRAFT_ADDITIONAL->add(_ALPHA_ATTRIBUTE);
		attr_AIRCRAFT_ADDITIONAL->add(_BETA_ATTRIBUTE);
		attr_AIRCRAFT_ADDITIONAL->add(_DYNAMIC_PRESSURE_ATTRIBUTE);  
		attr_AIRCRAFT_ADDITIONAL->add(_TANK_FILLING_ATTRIBUTE);
		
		_RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_POSITION_ClassHandle, *attr_AIRCRAFT_POSITION);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_ORIENTATION_ClassHandle, *attr_AIRCRAFT_ORIENTATION);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_UVW_SPEED_ClassHandle, *attr_AIRCRAFT_UVW_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,*attr_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_ACCELERATION_ClassHandle, *attr_AIRCRAFT_ACCELERATION);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_SPEED_ClassHandle, *attr_AIRCRAFT_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_ADDITIONAL_ClassHandle, *attr_AIRCRAFT_ADDITIONAL);

		// For Class/Attributes published
		attr_MEAS_AIRCRAFT_POSITION.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_ORIENTATION.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_UVW_SPEED.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_ACCELERATION.reset(RTI::AttributeHandleSetFactory::create(3)) ;
		attr_MEAS_AIRCRAFT_SPEED.reset(RTI::AttributeHandleSetFactory::create(7)) ;
		attr_MEAS_AIRCRAFT_ADDITIONAL.reset(RTI::AttributeHandleSetFactory::create(4)) ;
		attr_MEAS_AIRCRAFT_POSITION->add(_MEAS_LONGITUDE_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_POSITION->add(_MEAS_LATITUDE_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_POSITION->add(_MEAS_ALTITUDE_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ORIENTATION->add(_MEAS_PHI_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ORIENTATION->add(_MEAS_THETA_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ORIENTATION->add(_MEAS_PSI_ATTRIBUTE);   
		attr_MEAS_AIRCRAFT_UVW_SPEED->add(_MEAS_U_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_UVW_SPEED->add(_MEAS_V_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_UVW_SPEED->add(_MEAS_W_SPEED_ATTRIBUTE);  
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED->add(_MEAS_P_ANG_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED->add(_MEAS_Q_ANG_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED->add(_MEAS_R_ANG_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ACCELERATION->add(_MEAS_X_ACC_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ACCELERATION->add(_MEAS_Y_ACC_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ACCELERATION->add(_MEAS_Z_ACC_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_INDICATED_AIRSPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_TRUE_AIRSPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_GROUND_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_VERTICAL_SPEED_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_SPEED->add(_MEAS_MACH_NUMBER_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ADDITIONAL->add(_MEAS_ALPHA_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ADDITIONAL->add(_MEAS_BETA_ATTRIBUTE);
		attr_MEAS_AIRCRAFT_ADDITIONAL->add(_MEAS_DYNAMIC_PRESSURE_ATTRIBUTE);  
		attr_MEAS_AIRCRAFT_ADDITIONAL->add(_MEAS_TANK_FILLING_ATTRIBUTE);
		
		_RtiAmb.publishObjectClass(_MEAS_AIRCRAFT_POSITION_ClassHandle, *attr_MEAS_AIRCRAFT_POSITION);
        _RtiAmb.publishObjectClass(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle, *attr_MEAS_AIRCRAFT_ORIENTATION);
        _RtiAmb.publishObjectClass(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle, *attr_MEAS_AIRCRAFT_UVW_SPEED);
        _RtiAmb.publishObjectClass(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,*attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb.publishObjectClass(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle, *attr_MEAS_AIRCRAFT_ACCELERATION);
        _RtiAmb.publishObjectClass(_MEAS_AIRCRAFT_SPEED_ClassHandle, *attr_MEAS_AIRCRAFT_SPEED);
        _RtiAmb.publishObjectClass(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle, *attr_MEAS_AIRCRAFT_ADDITIONAL);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }   
    #ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void SensorsFederateHla13::registerObjectInstances()
{
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
		_MEAS_AIRCRAFT_POSITION_ObjectHandle = _RtiAmb.registerObjectInstance(_MEAS_AIRCRAFT_POSITION_ClassHandle,"Meas_Aircraft_Position");
		_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle = _RtiAmb.registerObjectInstance(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle,"Meas_Aircraft_Orientation");
		_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle = _RtiAmb.registerObjectInstance(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle,"Meas_Aircraft_UVW_Speed");
		_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle =_RtiAmb.registerObjectInstance(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,"Meas_Aircraft_PQR_Angular_Speed");
		_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle = _RtiAmb.registerObjectInstance(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle,"Meas_Aircraft_Acceleration");
		_MEAS_AIRCRAFT_SPEED_ObjectHandle = _RtiAmb.registerObjectInstance(_MEAS_AIRCRAFT_SPEED_ClassHandle,"Meas_Aircraft_Speed");
		_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle = _RtiAmb.registerObjectInstance(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle,"Meas_Aircraft_Additional");
		ahvps_MEAS_AIRCRAFT_POSITION.reset(RTI::AttributeSetFactory::create(3));
		ahvps_MEAS_AIRCRAFT_ORIENTATION.reset(RTI::AttributeSetFactory::create(3));
		ahvps_MEAS_AIRCRAFT_UVW_SPEED.reset(RTI::AttributeSetFactory::create(3));
		ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED.reset(RTI::AttributeSetFactory::create(3));
		ahvps_MEAS_AIRCRAFT_ACCELERATION.reset(RTI::AttributeSetFactory::create(3));
		ahvps_MEAS_AIRCRAFT_SPEED.reset(RTI::AttributeSetFactory::create(7));
		ahvps_MEAS_AIRCRAFT_ADDITIONAL.reset(RTI::AttributeSetFactory::create(4));
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void SensorsFederateHla13::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_ADDITIONAL_ClassHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
       
    try 
    {
		_RtiAmb.unpublishObjectClass(_MEAS_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb.unpublishObjectClass(_MEAS_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb.unpublishObjectClass(_MEAS_AIRCRAFT_UVW_SPEED_ClassHandle);
        _RtiAmb.unpublishObjectClass(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle);
        _RtiAmb.unpublishObjectClass(_MEAS_AIRCRAFT_ACCELERATION_ClassHandle);
        _RtiAmb.unpublishObjectClass(_MEAS_AIRCRAFT_SPEED_ClassHandle);
        _RtiAmb.unpublishObjectClass(_MEAS_AIRCRAFT_ADDITIONAL_ClassHandle);
        _RtiAmb.unpublishObjectClass(_USER_DATA_ClassHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
    #ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void SensorsFederateHla13::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
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
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		} 
		catch ( RTI::Exception &e ) 
		{
			cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
		} 
		catch ( ... ) 
		{
			cerr << "Error: unknown non-RTI exception." << endl;
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
    std::cout << "SensorsFederateHla13.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void SensorsFederateHla13::waitForAllAttributesReceived()
{
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> waitForAllAttributesReceived(): Start" << std::endl;
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
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
			
		} 
		catch ( RTI::Exception &e ) 
		{
			cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
		} 
		catch ( ... ) 
		{
			cerr << "Error: unknown non-RTI exception." << endl;
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
    std::cout << "SensorsFederateHla13.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void SensorsFederateHla13::discoverObjectInstance( RTI::ObjectHandle theObject
                                                 , RTI::ObjectClassHandle theObjectClass
                                                 , const char *theObjectName
                                                 )
                                           throw ( RTI::CouldNotDiscover
                                                 , RTI::ObjectClassNotKnown
                                                 , RTI::FederateInternalError
                                                 )
{
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
if ( (theObjectClass == _AIRCRAFT_POSITION_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Position")) ) 
	{
		_Discov_AIRCRAFT_POSITION = true;
		_AIRCRAFT_POSITION_ObjectHandle = theObject;
		#ifdef DEBUG_SENSORS_FED
		cout << "< Aircraft_Position > Object Instance has been discovered" << endl;
		#endif
	} 
    else if ( (theObjectClass == _AIRCRAFT_ORIENTATION_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Orientation")) ) 
    {
        _Discov_AIRCRAFT_ORIENTATION = true;
        _AIRCRAFT_ORIENTATION_ObjectHandle = theObject;
        #ifdef DEBUG_SENSORS_FED
        cout << "< Aircraft_Orientation > Object Instance has been discovered" << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_UVW_SPEED_ClassHandle) && (!strcmp(theObjectName,"Aircraft_UVW_Speed")) ) 
    {
        _Discov_AIRCRAFT_UVW_SPEED = true;
        _AIRCRAFT_UVW_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_SENSORS_FED
        cout << "< Aircraft_UVW_Speed > Object Instance has been discovered" << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle) && (!strcmp(theObjectName,"Aircraft_PQR_Angular_Speed")) ) 
    {
        _Discov_AIRCRAFT_PQR_ANGULAR_SPEED = true;
        _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_SENSORS_FED
        cout << "< Aircraft_PQR_Angular_Speed > Object Instance has been discovered" << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_ACCELERATION_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Acceleration")) ) 
    {
        _Discov_AIRCRAFT_ACCELERATION = true;
        _AIRCRAFT_ACCELERATION_ObjectHandle = theObject;
        #ifdef DEBUG_SENSORS_FED
        cout << "< Aircraft_Acceleration > Object Instance has been discovered" << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_SPEED_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Speed")) ) 
    {
        _Discov_AIRCRAFT_SPEED = true;
        _AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_SENSORS_FED
        cout << "< Aircraft_Speed > Object Instance has been discovered" << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_ADDITIONAL_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Additional")) ) 
    {
        _Discov_AIRCRAFT_ADDITIONAL = true;
        _AIRCRAFT_ADDITIONAL_ObjectHandle = theObject;
        #ifdef DEBUG_SENSORS_FED
        cout << "< Aircraft_Additional > Object Instance has been discovered" << endl;
        #endif
    } 
    #ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void SensorsFederateHla13::pauseInitTrim()
{
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> pause_init_trim(): Start" << std::endl;
    #endif
    if (_IsCreator) 
    {
		// Trimming synchronization starts automatically (Creator)
        try 
        {
            _RtiAmb.registerFederationSynchronizationPoint("Trimming", "");
        }
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
		while (!_SyncRegSuccess && !_SyncRegFailed) 
		{
			try 
			{
				_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
			} 
			catch ( RTI::Exception &e ) 
			{
				std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
			} 
			catch ( ... ) 
			{
				std::cerr << "Error: unknown non-RTI exception." << std::endl;
			}
			std::cout << ">> Waiting for success or failure of synchronisation point Trimming. " << std::endl;
		} 
		if(_SyncRegFailed)
		{
			std::cout << ">> Error on Trimming Synchronization" << std::endl;
		} 
    }
	while (!_InPause) 
	{
		std::cout << ">> Waiting for synchronisation point Trimming announcement." << std::endl;
		try 
		{
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		} 
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	try 
	{
		_RtiAmb.synchronizationPointAchieved("Trimming");
	} 
	catch ( RTI::Exception &e ) 
	{
		std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
	} 
	catch ( ... ) 
	{
		std::cerr << "Error: unknown non-RTI exception." << std::endl;
	}
	std::cout << ">> Synchronisation point Trimming satisfied." << std::endl;     

	while (_InPause) 
	{
		std::cout << ">> Waiting for initialization phase." << std::endl ;
		try 
		{
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		} 
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void SensorsFederateHla13::pauseInitSim()
{
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> pause_init_sim(): Start" << std::endl;
    #endif
    if (_IsCreator) 
    {
		std::cout << ">> PRESS ENTER TO START SIMULATING " << endl;      
        std::cin.get();
		// Simulating synchronization starts with an action of the user (on Creator interface)
        //std::cout << "Pause requested per Creator " << std::endl;
        try 
        {
            _RtiAmb.registerFederationSynchronizationPoint("Simulating", "");
        }
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
		while (!_SyncRegSuccess && !_SyncRegFailed) 
		{
			try 
			{
				_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
			} 
			catch ( RTI::Exception &e ) 
			{
				std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
			} 
			catch ( ... ) 
			{
				std::cerr << "Error: unknown non-RTI exception." << std::endl;
			}
			std::cout << ">> Waiting for success or failure of synchronisation point Simulating. " << std::endl;
		} 
		if(_SyncRegFailed)
		{
			std::cout << ">> Error on initial Synchronization" << std::endl;
		} 
    }
	while (!_InPause) 
	{
		std::cout << ">> Waiting for synchronisation point Simulating announcement." << std::endl;
		try 
		{
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		} 
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	try 
	{
		_RtiAmb.synchronizationPointAchieved("Simulating");
	} 
	catch ( RTI::Exception &e ) 
	{
		std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
	} 
	catch ( ... ) 
	{
		std::cerr << "Error: unknown non-RTI exception." << std::endl;
	}
	cout << ">> Synchronisation point Simulating satisfied." << endl;      

	while (_InPause) 
	{
		std::cout << ">> Waiting for simulation phase." << std::endl ;
		try 
		{
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		} 
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	#ifdef DEBUG_SENSORS_FED
    std::cout << "SensorsFederateHla13.cc -> pause_init_sim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void SensorsFederateHla13::initialization() 
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
void SensorsFederateHla13::calculateState() 
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
void SensorsFederateHla13::calculateOutput() 
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
void SensorsFederateHla13::sendInitialAllAttributes()
{

	ahvps_MEAS_AIRCRAFT_POSITION-> empty();
    ahvps_MEAS_AIRCRAFT_ORIENTATION-> empty();
    ahvps_MEAS_AIRCRAFT_UVW_SPEED-> empty();
    ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED-> empty();
    ahvps_MEAS_AIRCRAFT_ACCELERATION-> empty();
    ahvps_MEAS_AIRCRAFT_SPEED-> empty();
    ahvps_MEAS_AIRCRAFT_ADDITIONAL-> empty();  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_LONGITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_POSITION -> add(_MEAS_LONGITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_LATITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_POSITION -> add(_MEAS_LATITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_ALTITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_POSITION -> add(_MEAS_ALTITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_PHI.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ORIENTATION -> add(_MEAS_PHI_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_THETA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ORIENTATION -> add(_MEAS_THETA_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
   _OutputMessagebuffer.write_double(_Sensor_THETA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ORIENTATION -> add(_MEAS_PSI_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_U_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_UVW_SPEED -> add(_MEAS_U_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_V_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_UVW_SPEED -> add(_MEAS_V_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_W_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_UVW_SPEED -> add(_MEAS_W_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_P_ANG_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED -> add(_MEAS_P_ANG_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_Q_ANG_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED -> add(_MEAS_Q_ANG_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_R_ANG_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED -> add(_MEAS_R_ANG_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_X_ACC.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ACCELERATION -> add(_MEAS_X_ACC_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_Y_ACC.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ACCELERATION -> add(_MEAS_Y_ACC_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());   
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_Z_ACC.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ACCELERATION -> add(_MEAS_Z_ACC_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_INDICATED_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_INDICATED_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_EQUIVALENT_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_CALIBRATED_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_TRUE_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_TRUE_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_GROUND_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_GROUND_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_VERTICAL_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_VERTICAL_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_MACH_NUMBER.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_MACH_NUMBER_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_ALPHA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ADDITIONAL -> add(_MEAS_ALPHA_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_BETA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ADDITIONAL -> add(_MEAS_BETA_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());   
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_DYNAMIC_PRESSURE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ADDITIONAL -> add(_MEAS_DYNAMIC_PRESSURE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_TANK_FILLING.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ADDITIONAL -> add(_MEAS_TANK_FILLING_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

    try 
    {
		_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_POSITION_ObjectHandle, *ahvps_MEAS_AIRCRAFT_POSITION, "Measured_Aircraft_Position");
		_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle, *ahvps_MEAS_AIRCRAFT_ORIENTATION, "Measured_Aircraft_Orientation");
		_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle, *ahvps_MEAS_AIRCRAFT_UVW_SPEED, "Measured_Aircraft_UVW_Speed");
		_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle, *ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED, "Measured_Aircraft_PQR_Angular_Speed");
		_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle, *ahvps_MEAS_AIRCRAFT_ACCELERATION, "Measured_Aircraft_UVW_Speed");
		_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_SPEED_ObjectHandle, *ahvps_MEAS_AIRCRAFT_SPEED, "Measured_Aircraft_Speed");
		_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle, *ahvps_MEAS_AIRCRAFT_ADDITIONAL, "Measured_Aircraft_Additional");
    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void SensorsFederateHla13::sendUpdateAttributes(const RTI::FedTime& UpdateTime)
{   
	//std::cout << "SensorsFederateHla13::sendUpdateAttributes: Start" << std::endl;
	ahvps_MEAS_AIRCRAFT_POSITION-> empty();
    ahvps_MEAS_AIRCRAFT_ORIENTATION-> empty();
    ahvps_MEAS_AIRCRAFT_UVW_SPEED-> empty();
    ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED-> empty();
    ahvps_MEAS_AIRCRAFT_ACCELERATION-> empty();
    ahvps_MEAS_AIRCRAFT_SPEED-> empty();
    ahvps_MEAS_AIRCRAFT_ADDITIONAL-> empty();  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_LONGITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_POSITION -> add(_MEAS_LONGITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_LATITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_POSITION -> add(_MEAS_LATITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_ALTITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_POSITION -> add(_MEAS_ALTITUDE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_PHI.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ORIENTATION -> add(_MEAS_PHI_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_THETA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ORIENTATION -> add(_MEAS_THETA_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_PSI.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ORIENTATION -> add(_MEAS_PSI_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_U_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_UVW_SPEED -> add(_MEAS_U_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_V_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_UVW_SPEED -> add(_MEAS_V_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_LATITUDE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_UVW_SPEED -> add(_MEAS_W_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_P_ANG_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED -> add(_MEAS_P_ANG_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_Q_ANG_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED -> add(_MEAS_Q_ANG_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_R_ANG_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED -> add(_MEAS_R_ANG_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_X_ACC.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ACCELERATION -> add(_MEAS_X_ACC_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_Y_ACC.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ACCELERATION -> add(_MEAS_Y_ACC_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());   
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_Z_ACC.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ACCELERATION -> add(_MEAS_Z_ACC_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_INDICATED_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_INDICATED_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_EQUIVALENT_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_CALIBRATED_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_TRUE_AIRSPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_TRUE_AIRSPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_GROUND_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_GROUND_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_VERTICAL_SPEED.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_VERTICAL_SPEED_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_MACH_NUMBER.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_SPEED -> add(_MEAS_MACH_NUMBER_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());    
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_ALPHA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ADDITIONAL -> add(_MEAS_ALPHA_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_BETA.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ADDITIONAL -> add(_MEAS_BETA_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());   
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_DYNAMIC_PRESSURE.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ADDITIONAL -> add(_MEAS_DYNAMIC_PRESSURE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());  
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Sensor_TANK_FILLING.getOutputSignal());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_MEAS_AIRCRAFT_ADDITIONAL -> add(_MEAS_TANK_FILLING_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_POSITION_ObjectHandle, *ahvps_MEAS_AIRCRAFT_POSITION, "Measured_Aircraft_Position");
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle, *ahvps_MEAS_AIRCRAFT_ORIENTATION, "Measured_Aircraft_Orientation");
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle, *ahvps_MEAS_AIRCRAFT_UVW_SPEED, "Measured_Aircraft_UVW_Speed");
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle, *ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED, "Measured_Aircraft_PQR_Angular_Speed");
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle, *ahvps_MEAS_AIRCRAFT_ACCELERATION, "Measured_Aircraft_UVW_Speed");
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_SPEED_ObjectHandle, *ahvps_MEAS_AIRCRAFT_SPEED, "Measured_Aircraft_Speed");
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle, *ahvps_MEAS_AIRCRAFT_ADDITIONAL, "Measured_Aircraft_Additional");
        }
        else
        {
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_POSITION_ObjectHandle, *ahvps_MEAS_AIRCRAFT_POSITION, UpdateTime, "Measured_Aircraft_Position");
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle, *ahvps_MEAS_AIRCRAFT_ORIENTATION, UpdateTime, "Measured_Aircraft_Orientation");
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle, *ahvps_MEAS_AIRCRAFT_UVW_SPEED, UpdateTime, "Measured_Aircraft_UVW_Speed");
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle, *ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED, UpdateTime, "Measured_Aircraft_PQR_Angular_Speed");
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle, *ahvps_MEAS_AIRCRAFT_ACCELERATION, UpdateTime, "Measured_Aircraft_UVW_Speed");
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_SPEED_ObjectHandle, *ahvps_MEAS_AIRCRAFT_SPEED, UpdateTime, "Measured_Aircraft_Speed");
			_RtiAmb.updateAttributeValues(_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle, *ahvps_MEAS_AIRCRAFT_ADDITIONAL, UpdateTime, "Measured_Aircraft_Additional");

        }
    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
	//std::cout << "SensorsFederateHla13::sendUpdateR: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void SensorsFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
                                                   , const RTI::AttributeHandleValuePairSet& theAttributes
                                                   , const char *theTag
                                                   )
                                             throw ( RTI::ObjectNotKnown
							                       , RTI::AttributeNotKnown
							                       , RTI::FederateOwnsAttributes
							                       , RTI::FederateInternalError
							                       ) 
{
   	RTI::ULong valueLength ;
    RTI::AttributeHandle parmHandle ;
    char *attrValue ;
    MessageBuffer buffer;

    if (theObject == _AIRCRAFT_POSITION_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
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
				cout << "SensorsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_AIRCRAFT_POSITION = true;
    } 
    else if (theObject == _AIRCRAFT_ORIENTATION_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
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
				cout << "SensorsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_AIRCRAFT_ORIENTATION = true;
    } 
    else if (theObject == _AIRCRAFT_UVW_SPEED_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
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
				cout << "SensorsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_AIRCRAFT_UVW_SPEED = true;
    } 
    else if (theObject == _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
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
				cout << "SensorsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_AIRCRAFT_PQR_ANGULAR_SPEED = true;
    }   
    else if (theObject == _AIRCRAFT_ACCELERATION_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
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
				cout << "SensorsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_AIRCRAFT_ACCELERATION = true;
    } 
    else if (theObject == _AIRCRAFT_SPEED_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
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
				cout << "SensorsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_AIRCRAFT_SPEED = true;
    }
    else if (theObject == _AIRCRAFT_ADDITIONAL_ObjectHandle)
    {
        for (unsigned int j=0 ; j<theAttributes.size(); j++) 
        {
            parmHandle = theAttributes.getHandle(j);
            valueLength = theAttributes.getValueLength(j);
            assert(valueLength>0);
            buffer.resize(valueLength);        
            buffer.reset();        
            theAttributes.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
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
				cout << "SensorsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
		}
        _New_AIRCRAFT_ADDITIONAL = true;
    }
	else
	{ 
		cout << "SensorsFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << endl; 
	}
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void SensorsFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
                                                   , const RTI::AttributeHandleValuePairSet& theAttributes
                                                   , const RTI::FedTime& theTime
                                                   , const char *theTag
                                                   , RTI::EventRetractionHandle theHandle
                                                   )
											 throw ( RTI::ObjectNotKnown
											       , RTI::AttributeNotKnown
											       , RTI::FederateOwnsAttributes
											       , RTI::InvalidFederationTime 
											       , RTI::FederateInternalError
											       ) 
{
	reflectAttributeValues(theObject, theAttributes, theTag);   
	#ifdef DEBUG_SENSORS_FED
	std::cout << "reflectAttributeValues received with ts " <<  ((RTIfedTime&)theTime).getTime() << std::endl;
	#endif 
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void SensorsFederateHla13::timeRegulationEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeRegulationWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeReg = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void SensorsFederateHla13::timeConstrainedEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeConstrainedWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeConst = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeAdvanceGrant
void SensorsFederateHla13::timeAdvanceGrant(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::TimeAdvanceWasNotInProgress,
            RTI::FederateInternalError) 
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_SENSORS_FED
	std::cout << " >> TAG RECU == LocalTime = " <<  _LocalTime << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void SensorsFederateHla13::enableTimeRegulation()
{
	_RtiAmb.enableTimeRegulation(_LocalTime, _Lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void SensorsFederateHla13::disableTimeRegulation()
{
	_RtiAmb.disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void SensorsFederateHla13::enableTimeConstrained()
{
	_RtiAmb.enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void SensorsFederateHla13::disableTimeConstrained()
{
	_RtiAmb.disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void SensorsFederateHla13::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb.enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void SensorsFederateHla13::disableAsynchronousDelivery()
{
	_RtiAmb.disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void SensorsFederateHla13::timeAdvanceRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_SENSORS_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.timeAdvanceRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_SENSORS_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void SensorsFederateHla13::timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime)
{
	_RtiAmb.timeAdvanceRequestAvailable(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
void SensorsFederateHla13::nextEventRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_SENSORS_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.nextEventRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_SENSORS_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// function : nextEventAvailable
void SensorsFederateHla13::nextEventAvailable(RTIfedTime NextLogicalTime)
{
	_RtiAmb.nextEventRequestAvailable(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationSucceeded
void SensorsFederateHla13::synchronizationPointRegistrationSucceeded(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegSuccess = true ;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void SensorsFederateHla13::synchronizationPointRegistrationFailed(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegFailed = true ;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void SensorsFederateHla13::announceSynchronizationPoint(const char *label, const char *tag)
    throw ( RTI::FederateInternalError) 
{
       _InPause = true ;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void SensorsFederateHla13::federationSynchronized(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _InPause = false ;
} 

// ----------------------------------------------------------------------------
// function : run
void SensorsFederateHla13::run()
{
	while (_LocalTime < _SimulationEndTime)
	{
		// Model calculations
		calculateState(); 
		calculateOutput();
		sendUpdateAttributes(_LocalTime + _Lookahead);
		timeAdvanceRequest(_TimeStep);
	}
} 
