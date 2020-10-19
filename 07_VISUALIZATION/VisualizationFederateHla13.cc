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
//

#include "VisualizationFederateHla13.hh"

// ----------------------------------------------------------------------------
// VisualizationFederateHla13 Constructor
VisualizationFederateHla13::VisualizationFederateHla13( std::wstring FederationName
									, std::wstring FederateName
									, std::wstring FomFileName
									) 
									: NullFederateAmbassador()
									, _Visualization()
									, _RtiAmb()
{
	#ifdef DEBUG_VISUALIZATION_FED
	std::cout << "VisualizationFederateHla13.cc -> Constructor(): Start" << std::endl;
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
	std::cout << "VisualizationFederateHla13.cc -> Constructor(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// VisualizationFederateHla13 Destructor
VisualizationFederateHla13::~VisualizationFederateHla13()
	                     throw(RTI::FederateInternalError)
{
	
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void VisualizationFederateHla13::createFederationExecution() 
{
	#ifdef DEBUG_VISUALIZATION_FED
	std::cout << "VisualizationFederateHla13.cc -> createFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> createFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void VisualizationFederateHla13::destroyFederationExecution() 
{
	#ifdef DEBUG_VISUALIZATION_FED
	std::cout << "VisualizationFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_VISUALIZATION_FED
	std::cout << "VisualizationFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla13::joinFederationExecution() 
{
	#ifdef DEBUG_VISUALIZATION_FED
	std::cout << "VisualizationFederateHla13.cc -> joinFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_VISUALIZATION_FED
	std::cout << "VisualizationFederateHla13.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla13::resignFederationExecution() 
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb.resignFederationExecution(RTI::DELETE_OBJECTS_AND_RELEASE_ATTRIBUTES);
	}
	catch (RTI::Exception& e) 
	{
		std::cout << "RFE: caught " << e._name << " reason " << e._reason <<std::endl;
	}
	#ifdef DEBUG_VISUALIZATION_FED
	std::cout << "VisualizationFederateHla13.cc -> resignFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
RTI::FederateHandle VisualizationFederateHla13::getFederateHandle() const
{
    return _FederateHandle;
}

// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void VisualizationFederateHla13::setHlaTimeManagementSettings( RTIfedTime TimeStep
                                                    , RTIfedTime Lookahead
                                                    , RTIfedTime LocalTime
                                                    , RTIfedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void VisualizationFederateHla13::getAllHandles()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed
		_ACTUATORS_ClassHandle = _RtiAmb.getObjectClassHandle("ACTUATORS");
		_AIRCRAFT_POSITION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_POSITION");
		_AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ORIENTATION");
		_AIRCRAFT_UVW_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_UVW_SPEED");
		_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_PQR_ANGULAR_SPEED");
		_AIRCRAFT_ACCELERATION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ACCELERATION");
		_AIRCRAFT_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_SPEED");
		_AIRCRAFT_ADDITIONAL_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ADDITIONAL");
		_RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_AILERON_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
		_LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_AILERON_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);
		_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
		_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ELEVATOR_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);
		_RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RUDDER_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);            
		_RIGHT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ENGINE_THRUST",_ACTUATORS_ClassHandle);
		_LEFT_ENGINE_THRUST_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ENGINE_THRUST", _ACTUATORS_ClassHandle);
		_FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("FLAPS_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);
		_SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("SPOILERS_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);
		_STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("STABILIZER_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);
		_GEARS_POSITION_ATTRIBUTE = _RtiAmb.getAttributeHandle("GEARS_POSITION", _ACTUATORS_ClassHandle);
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
		// None
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void VisualizationFederateHla13::publishAndSubscribe()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		attr_ACTUATORS.reset(RTI::AttributeHandleSetFactory::create(11));
		attr_AIRCRAFT_POSITION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_ORIENTATION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_UVW_SPEED.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_PQR_ANGULAR_SPEED.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_ACCELERATION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_SPEED.reset(RTI::AttributeHandleSetFactory::create(7));
		attr_AIRCRAFT_ADDITIONAL.reset(RTI::AttributeHandleSetFactory::create(4));
		attr_ACTUATORS->add(_RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_RIGHT_ENGINE_THRUST_ATTRIBUTE);
		attr_ACTUATORS->add(_LEFT_ENGINE_THRUST_ATTRIBUTE);
		attr_ACTUATORS->add(_FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_GEARS_POSITION_ATTRIBUTE);	
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
		
		_RtiAmb.subscribeObjectClassAttributes(_ACTUATORS_ClassHandle,*attr_ACTUATORS);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_POSITION_ClassHandle, *attr_AIRCRAFT_POSITION);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_ORIENTATION_ClassHandle, *attr_AIRCRAFT_ORIENTATION);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_UVW_SPEED_ClassHandle, *attr_AIRCRAFT_UVW_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle,*attr_AIRCRAFT_PQR_ANGULAR_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_ACCELERATION_ClassHandle, *attr_AIRCRAFT_ACCELERATION);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_SPEED_ClassHandle, *attr_AIRCRAFT_SPEED);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_ADDITIONAL_ClassHandle, *attr_AIRCRAFT_ADDITIONAL);

		// For Class/Attributes published
		// None
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }   
    #ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void VisualizationFederateHla13::registerObjectInstances()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	//Not used
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void VisualizationFederateHla13::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb.unsubscribeObjectClass(_ACTUATORS_ClassHandle);
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
    #ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void VisualizationFederateHla13::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
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

	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void VisualizationFederateHla13::waitForAllAttributesReceived()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> waitForAllAttributesReceived(): Start" << std::endl;
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
    #ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void VisualizationFederateHla13::discoverObjectInstance( RTI::ObjectHandle theObject
                                                   , RTI::ObjectClassHandle theObjectClass
                                                   , const char *theObjectName
                                                   )
                                             throw ( RTI::CouldNotDiscover
                                                   , RTI::ObjectClassNotKnown
                                                   , RTI::FederateInternalError
                                                   )
{
    if ( (theObjectClass == _AIRCRAFT_POSITION_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Position")) ) 
    {
        _Discov_AIRCRAFT_POSITION = true;
        _AIRCRAFT_POSITION_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        cout << "< Aircraft_Position > Object Instance has been discovered with handle "
             << _AIRCRAFT_POSITION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_ORIENTATION_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Orientation")) ) 
    {
        _Discov_AIRCRAFT_ORIENTATION = true;
        _AIRCRAFT_ORIENTATION_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        cout << "< Aircraft_Orientation > Object Instance has been discovered with handle "
             << _AIRCRAFT_ORIENTATION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
	else if ( (theObjectClass == _AIRCRAFT_UVW_SPEED_ClassHandle) && (!strcmp(theObjectName,"Aircraft_UVW_Speed")) ) 
	{
        _Discov_AIRCRAFT_UVW_SPEED = true;
        _AIRCRAFT_UVW_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        cout << "< Aircraft_UVW_Speed > Object Instance has been discovered with handle "
             << _AIRCRAFT_UVW_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle) && (!strcmp(theObjectName,"Aircraft_PQR_Angular_Speed")) ) 
    {
        _Discov_AIRCRAFT_PQR_ANGULAR_SPEED = true;
        _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        cout << "< Aircraft_PQR_Angular_Speed > Object Instance has been discovered with handle "
             << _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_ACCELERATION_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Acceleration")) ) 
    {
        _Discov_AIRCRAFT_ACCELERATION = true;
        _AIRCRAFT_ACCELERATION_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        cout << "< Aircraft_Acceleration > Object Instance has been discovered with handle "
             << _AIRCRAFT_ACCELERATION_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_SPEED_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Speed")) ) 
    {
        _Discov_AIRCRAFT_SPEED = true;
        _AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        cout << "< Aircraft_Speed > Object Instance has been discovered with handle "
             << _AIRCRAFT_SPEED_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_ADDITIONAL_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Additional")) ) 
    {
        _Discov_AIRCRAFT_ADDITIONAL = true;
        _AIRCRAFT_ADDITIONAL_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        cout << "< Aircraft_Additional > Object Instance has been discovered with handle "
             << _AIRCRAFT_ADDITIONAL_ObjectHandle
             << "."
             << endl;
        #endif
    } 
	else if ( (theObjectClass == _ACTUATORS_ClassHandle) && (!strcmp(theObjectName,"Actuators_Control_Surfaces")) ) 
	{
        _Discov_ACTUATORS_CONTROL_SURFACES = true;
        _ACTUATORS_CONTROL_SURFACES_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        cout << "< Actuators_Control_Surfaces > Object Instance has been discovered with handle "
             << _ACTUATORS_CONTROL_SURFACES_ObjectHandle
             << "."
             << endl;
        #endif
    } 
    else if ( (theObjectClass == _ACTUATORS_ClassHandle) && (!strcmp(theObjectName,"Actuators_Engines")) ) 
	{
        _Discov_ACTUATORS_ENGINES = true;
        _ACTUATORS_ENGINES_ObjectHandle = theObject;
        #ifdef DEBUG_VISUALIZATION_FED
        cout << "< Actuators_Engines > Object Instance has been discovered with handle "
             << _ACTUATORS_ENGINES_ObjectHandle
             << "."
             << endl;
        #endif
    } 
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void VisualizationFederateHla13::pauseInitTrim()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void VisualizationFederateHla13::pauseInitSim()
{
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> pause_init_sim(): Start" << std::endl;
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
	#ifdef DEBUG_VISUALIZATION_FED
    std::cout << "VisualizationFederateHla13.cc -> pause_init_sim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void VisualizationFederateHla13::initialization() 
{
	// Nothing to do
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void VisualizationFederateHla13::calculateState() 
{
	// Model State calculation
	// Nothing to do 
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void VisualizationFederateHla13::calculateOutput() 
{
	// Model output calculation
	_Visualization.FlighGearSocketSend();
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void VisualizationFederateHla13::sendUpdateAttributes(const RTI::FedTime& UpdateTime)
{   
	// Nothing to do
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void VisualizationFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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
	double tmp_value;

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
				cout << "VisualizationFederateHla13: RAV ==> ERROR: unknown handle for object " 
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
				cout << "VisualizationFederateHla13: RAV ==> ERROR: unknown handle for object " 
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
				cout << "VisualizationFederateHla13: RAV ==> ERROR: unknown handle for object " 
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
				cout << "VisualizationFederateHla13: RAV ==> ERROR: unknown handle for object " 
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
				cout << "VisualizationFederateHla13: RAV ==> ERROR: unknown handle for object " 
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
				cout << "VisualizationFederateHla13: RAV ==> ERROR: unknown handle for object " 
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
				cout << "VisualizationFederateHla13: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl;
			}
        }    
        _New_AIRCRAFT_ADDITIONAL = true;
    }
    if (theObject == _ACTUATORS_CONTROL_SURFACES_ObjectHandle)
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
				cout << "VisualizationFederateHla13: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl;
			}
        }
        _New_ACTUATORS_CONTROL_SURFACES = true;
    } 
    else if (theObject == _ACTUATORS_ENGINES_ObjectHandle)
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
				cout << "VisualizationFederateHla13: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl;
			}
        }    
        _New_ACTUATORS_ENGINES = true;
    }    
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void VisualizationFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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
	#ifdef DEBUG_VISUALIZATION_FED
	std::cout << "reflectAttributeValues received with ts " <<  ((RTIfedTime&)theTime).getTime() << std::endl;
	#endif 
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void VisualizationFederateHla13::timeRegulationEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeRegulationWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeReg = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void VisualizationFederateHla13::timeConstrainedEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeConstrainedWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeConst = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeAdvanceGrant
void VisualizationFederateHla13::timeAdvanceGrant(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::TimeAdvanceWasNotInProgress,
            RTI::FederateInternalError) 
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_VISUALIZATION_FED
	std::cout << " >> TAG RECU == LocalTime = " <<  _LocalTime << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void VisualizationFederateHla13::enableTimeRegulation()
{
	_RtiAmb.enableTimeRegulation(_LocalTime, _Lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla13::disableTimeRegulation()
{
	_RtiAmb.disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla13::enableTimeConstrained()
{
	_RtiAmb.enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla13::disableTimeConstrained()
{
	_RtiAmb.disableTimeConstrained();
	_IsTimeConst = false;
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla13::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb.enableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla13::disableAsynchronousDelivery()
{
	_RtiAmb.disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla13::timeAdvanceRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_VISUALIZATION_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.timeAdvanceRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_VISUALIZATION_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void VisualizationFederateHla13::timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime)
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
void VisualizationFederateHla13::nextEventRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_VISUALIZATION_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.nextEventRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_VISUALIZATION_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// function : nextEventAvailable
void VisualizationFederateHla13::nextEventAvailable(RTIfedTime NextLogicalTime)
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
void VisualizationFederateHla13::synchronizationPointRegistrationSucceeded(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegSuccess = true ;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void VisualizationFederateHla13::synchronizationPointRegistrationFailed(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegFailed = true ;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void VisualizationFederateHla13::announceSynchronizationPoint(const char *label, const char *tag)
    throw ( RTI::FederateInternalError) 
{
       _InPause = true ;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void VisualizationFederateHla13::federationSynchronized(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _InPause = false ;
} 

// ----------------------------------------------------------------------------
// function : run
void VisualizationFederateHla13::run()
{
	while (_LocalTime < _SimulationEndTime)
	{
		// Model calculations
		calculateOutput();
		timeAdvanceRequest(_TimeStep);
	}
} 
