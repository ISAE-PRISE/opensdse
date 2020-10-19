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

#include "EnvironmentFederateHla13.hh"

// ----------------------------------------------------------------------------
// EnvironmentFederateHla13 Constructor
EnvironmentFederateHla13::EnvironmentFederateHla13( std::wstring FederationName
									, std::wstring FederateName
									, std::wstring FomFileName
									) 
									: NullFederateAmbassador()
									, _RtiAmb() 
{
	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "EnvironmentFederateHla13.cc -> Constructor(): Start" << std::endl;
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
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = false;
	_IsOutMesTimespamped = true;
	
    _Discov_AIRCRAFT_POSITION          	= false;
    _Discov_AIRCRAFT_ORIENTATION       	= false;
    _Discov_AIRCRAFT_SPEED            	= false;
    _New_AIRCRAFT_POSITION             	= false;
    _New_AIRCRAFT_ORIENTATION          	= false;
    _New_AIRCRAFT_SPEED                	= false;  

	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "EnvironmentFederateHla13.cc -> Constructor(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// EnvironmentFederateHla13 Destructor
EnvironmentFederateHla13::~EnvironmentFederateHla13()
	                     throw(RTI::FederateInternalError)
{

}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void EnvironmentFederateHla13::createFederationExecution() 
{
	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "EnvironmentFederateHla13.cc -> createFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> createFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void EnvironmentFederateHla13::destroyFederationExecution() 
{
	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "EnvironmentFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "EnvironmentFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla13::joinFederationExecution() 
{
	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "EnvironmentFederateHla13.cc -> joinFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "EnvironmentFederateHla13.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla13::resignFederationExecution() 
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb.resignFederationExecution(RTI::DELETE_OBJECTS_AND_RELEASE_ATTRIBUTES);
	}
	catch (RTI::Exception& e) 
	{
		std::cout << "RFE: caught " << e._name << " reason " << e._reason <<std::endl;
	}
	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "EnvironmentFederateHla13.cc -> resignFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
RTI::FederateHandle EnvironmentFederateHla13::getFederateHandle() const
{
    return _FederateHandle;
}

// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void EnvironmentFederateHla13::setHlaTimeManagementSettings( RTIfedTime TimeStep
                                                    , RTIfedTime Lookahead
                                                    , RTIfedTime LocalTime
                                                    , RTIfedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void EnvironmentFederateHla13::getAllHandles()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed
        _AIRCRAFT_POSITION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_POSITION");
        _AIRCRAFT_ORIENTATION_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_ORIENTATION");
        _AIRCRAFT_SPEED_ClassHandle = _RtiAmb.getObjectClassHandle("AIRCRAFT_SPEED");
  		_LONGITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("LONGITUDE",_AIRCRAFT_POSITION_ClassHandle);
        _LATITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("LATITUDE",_AIRCRAFT_POSITION_ClassHandle);
        _ALTITUDE_ATTRIBUTE = _RtiAmb.getAttributeHandle("ALTITUDE",_AIRCRAFT_POSITION_ClassHandle);   
  		_PHI_ATTRIBUTE = _RtiAmb.getAttributeHandle("PHI",_AIRCRAFT_ORIENTATION_ClassHandle);
        _THETA_ATTRIBUTE = _RtiAmb.getAttributeHandle("THETA",_AIRCRAFT_ORIENTATION_ClassHandle);
        _PSI_ATTRIBUTE = _RtiAmb.getAttributeHandle("PSI",_AIRCRAFT_ORIENTATION_ClassHandle);   
        _INDICATED_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("INDICATED_AIRSPEED",_AIRCRAFT_SPEED_ClassHandle);
        _EQUIVALENT_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("EQUIVALENT_AIRSPEED",_AIRCRAFT_SPEED_ClassHandle);
        _CALIBRATED_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("CALIBRATED_AIRSPEED",_AIRCRAFT_SPEED_ClassHandle);
        _TRUE_AIRSPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("TRUE_AIRSPEED", _AIRCRAFT_SPEED_ClassHandle);
        _GROUND_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("GROUND_SPEED", _AIRCRAFT_SPEED_ClassHandle);
        _VERTICAL_SPEED_ATTRIBUTE = _RtiAmb.getAttributeHandle("VERTICAL_SPEED", _AIRCRAFT_SPEED_ClassHandle);
        _MACH_NUMBER_ATTRIBUTE = _RtiAmb.getAttributeHandle("MACH_NUMBER", _AIRCRAFT_SPEED_ClassHandle);         
        // Published
        _ENVIRONMENT_VARIABLES_ClassHandle = _RtiAmb.getObjectClassHandle("ENVIRONMENT_VARIABLES");
        _WIND_COMPONENTS_ClassHandle = _RtiAmb.getObjectClassHandle("WIND_COMPONENTS");
		_TEMPERATURE_ATTRIBUTE = _RtiAmb.getAttributeHandle("TEMPERATURE",_ENVIRONMENT_VARIABLES_ClassHandle);
        _DENSITY_OF_AIR_ATTRIBUTE = _RtiAmb.getAttributeHandle("DENSITY_OF_AIR",_ENVIRONMENT_VARIABLES_ClassHandle);
        _PRESSURE_ATTRIBUTE = _RtiAmb.getAttributeHandle("PRESSURE",_ENVIRONMENT_VARIABLES_ClassHandle);
        _SPEED_OF_SOUND_ATTRIBUTE = _RtiAmb.getAttributeHandle("SPEED_OF_SOUND",_ENVIRONMENT_VARIABLES_ClassHandle);       
        _U_WIND_ATTRIBUTE = _RtiAmb.getAttributeHandle("U_WIND",_WIND_COMPONENTS_ClassHandle);
        _V_WIND_ATTRIBUTE = _RtiAmb.getAttributeHandle("V_WIND",_WIND_COMPONENTS_ClassHandle);
        _W_WIND_ATTRIBUTE = _RtiAmb.getAttributeHandle("W_WIND",_WIND_COMPONENTS_ClassHandle);
        _P_WIND_ATTRIBUTE = _RtiAmb.getAttributeHandle("P_WIND",_WIND_COMPONENTS_ClassHandle);   
        _Q_WIND_ATTRIBUTE = _RtiAmb.getAttributeHandle("Q_WIND",_WIND_COMPONENTS_ClassHandle);
        _R_WIND_ATTRIBUTE = _RtiAmb.getAttributeHandle("R_WIND",_WIND_COMPONENTS_ClassHandle);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void EnvironmentFederateHla13::publishAndSubscribe()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		attr_AIRCRAFT_POSITION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_ORIENTATION.reset(RTI::AttributeHandleSetFactory::create(3));
		attr_AIRCRAFT_SPEED.reset(RTI::AttributeHandleSetFactory::create(7));
		attr_AIRCRAFT_POSITION->add(_LONGITUDE_ATTRIBUTE);
		attr_AIRCRAFT_POSITION->add(_LATITUDE_ATTRIBUTE);
		attr_AIRCRAFT_POSITION->add(_ALTITUDE_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION->add(_PHI_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION->add(_THETA_ATTRIBUTE);
		attr_AIRCRAFT_ORIENTATION->add(_PSI_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_INDICATED_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_EQUIVALENT_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_CALIBRATED_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_TRUE_AIRSPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_GROUND_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_VERTICAL_SPEED_ATTRIBUTE);
		attr_AIRCRAFT_SPEED->add(_MACH_NUMBER_ATTRIBUTE);
		
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_POSITION_ClassHandle,*attr_AIRCRAFT_POSITION);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_ORIENTATION_ClassHandle,*attr_AIRCRAFT_ORIENTATION);
        _RtiAmb.subscribeObjectClassAttributes(_AIRCRAFT_SPEED_ClassHandle,*attr_AIRCRAFT_SPEED);

		// For Class/Attributes published
		attr_ENVIRONMENT_VARIABLES.reset(RTI::AttributeHandleSetFactory::create(4));
		attr_WIND_COMPONENTS.reset(RTI::AttributeHandleSetFactory::create(6));  
		attr_ENVIRONMENT_VARIABLES->add(_TEMPERATURE_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES->add(_DENSITY_OF_AIR_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES->add(_PRESSURE_ATTRIBUTE);
		attr_ENVIRONMENT_VARIABLES->add(_SPEED_OF_SOUND_ATTRIBUTE);
		attr_WIND_COMPONENTS->add(_U_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS->add(_V_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS->add(_W_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS->add(_P_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS->add(_Q_WIND_ATTRIBUTE);
		attr_WIND_COMPONENTS->add(_R_WIND_ATTRIBUTE); 
		
        _RtiAmb.publishObjectClass(_ENVIRONMENT_VARIABLES_ClassHandle,*attr_ENVIRONMENT_VARIABLES);
        _RtiAmb.publishObjectClass(_WIND_COMPONENTS_ClassHandle,*attr_WIND_COMPONENTS);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }   
    #ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnvironmentFederateHla13::registerObjectInstances()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
        _ENVIRONMENT_VARIABLES_ObjectHandle = _RtiAmb.registerObjectInstance(_ENVIRONMENT_VARIABLES_ClassHandle,"Environment_Variables");
        _WIND_COMPONENTS_ObjectHandle = _RtiAmb.registerObjectInstance(_WIND_COMPONENTS_ClassHandle,"Wind_Components");
		ahvps_ENVIRONMENT_VARIABLES.reset(RTI::AttributeSetFactory::create(4));
		ahvps_WIND_COMPONENTS.reset(RTI::AttributeSetFactory::create(6));
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void EnvironmentFederateHla13::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_POSITION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_ORIENTATION_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_AIRCRAFT_SPEED_ClassHandle);
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
        _RtiAmb.unpublishObjectClass(_ENVIRONMENT_VARIABLES_ClassHandle);
        _RtiAmb.unpublishObjectClass(_WIND_COMPONENTS_ClassHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
    #ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnvironmentFederateHla13::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_Discov_AIRCRAFT_POSITION    ||
            !_Discov_AIRCRAFT_ORIENTATION ||
            !_Discov_AIRCRAFT_SPEED) 
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
    _Discov_AIRCRAFT_SPEED = false;
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnvironmentFederateHla13::waitForAttributesPos()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_AIRCRAFT_POSITION) 
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
    #ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void EnvironmentFederateHla13::waitForAllAttributesReceived()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_AIRCRAFT_POSITION    ||
           !_New_AIRCRAFT_ORIENTATION ||
           !_New_AIRCRAFT_SPEED)
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
    _New_AIRCRAFT_SPEED = false;
    #ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void EnvironmentFederateHla13::discoverObjectInstance( RTI::ObjectHandle theObject
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
        #ifdef DEBUG_ENVIRONMENT_FED
        cout << "< Aircraft_Position > Object Instance has been discovered" << endl;
		#endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_ORIENTATION_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Orientation")) ) 
    {
        _Discov_AIRCRAFT_ORIENTATION = true;
        _AIRCRAFT_ORIENTATION_ObjectHandle = theObject;
        #ifdef DEBUG_ENVIRONMENT_FED
        cout << "< Aircraft_Orientation > Object Instance has been discovered" << endl;
        #endif
    } 
    else if ( (theObjectClass == _AIRCRAFT_SPEED_ClassHandle) && (!strcmp(theObjectName,"Aircraft_Speed")) ) 
    {
        _Discov_AIRCRAFT_SPEED = true;
        _AIRCRAFT_SPEED_ObjectHandle = theObject;
        #ifdef DEBUG_ENVIRONMENT_FED
        cout << "< Aircraft_Speed > Object Instance has been discovered" << endl;
        #endif
    }     
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void EnvironmentFederateHla13::pauseInitTrim()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void EnvironmentFederateHla13::pauseInitSim()
{
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> pause_init_sim(): Start" << std::endl;
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
	#ifdef DEBUG_ENVIRONMENT_FED
    std::cout << "EnvironmentFederateHla13.cc -> pause_init_sim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void EnvironmentFederateHla13::initialization() 
{
	waitForAllObjectDiscovered();
	waitForAttributesPos();
	calculateState();
	sendInitialAttributesEnv();
	waitForAllAttributesReceived();
}

// ----------------------------------------------------------------------------
// Calculate State values for Efcs
void EnvironmentFederateHla13::calculateState() 
{
	// Model State calculation
	_Atmosphere.calculate_state();
}

// ----------------------------------------------------------------------------
// Calculate Ouput values for Efcs
void EnvironmentFederateHla13::calculateOutput() 
{
	// Model output calculation
	_Atmosphere.calculate_output();
}

// ----------------------------------------------------------------------------
// Send Updates for init attributes position
void EnvironmentFederateHla13::sendInitialAttributesEnv()
{ 
	ahvps_ENVIRONMENT_VARIABLES->empty();
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getTemperature());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_ENVIRONMENT_VARIABLES -> add(_TEMPERATURE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getDensity());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_ENVIRONMENT_VARIABLES -> add(_DENSITY_OF_AIR_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); ;
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getPressure());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_ENVIRONMENT_VARIABLES -> add(_PRESSURE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());   
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getSoundSpeed());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_ENVIRONMENT_VARIABLES -> add(_SPEED_OF_SOUND_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

    try 
    {
		_RtiAmb.updateAttributeValues(_ENVIRONMENT_VARIABLES_ObjectHandle, *ahvps_ENVIRONMENT_VARIABLES, "Environment_Variables");

    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void EnvironmentFederateHla13::sendUpdateAttributes(const RTI::FedTime& UpdateTime)
{   
	//std::cout << "EnvironmentFederateHla13::sendUpdateAttributes: Start" << std::endl;
  	ahvps_ENVIRONMENT_VARIABLES->empty();
	ahvps_WIND_COMPONENTS->empty();
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getTemperature());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_ENVIRONMENT_VARIABLES -> add(_TEMPERATURE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getDensity());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_ENVIRONMENT_VARIABLES -> add(_DENSITY_OF_AIR_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); ;
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getPressure());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_ENVIRONMENT_VARIABLES -> add(_PRESSURE_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());   
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_Atmosphere.getSoundSpeed());
    _OutputMessagebuffer.updateReservedBytes();
    ahvps_ENVIRONMENT_VARIABLES -> add(_SPEED_OF_SOUND_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Atmosphere.getTotalWindBody(1));
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_WIND_COMPONENTS -> add(_U_WIND_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Atmosphere.getTotalWindBody(2));
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_WIND_COMPONENTS -> add(_V_WIND_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Atmosphere.getTotalWindBody(3));
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_WIND_COMPONENTS -> add(_W_WIND_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Atmosphere.getTotalWindRateBody(1));
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_WIND_COMPONENTS -> add(_P_WIND_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Atmosphere.getTotalWindRateBody(2));
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_WIND_COMPONENTS -> add(_Q_WIND_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Atmosphere.getTotalWindRateBody(3));
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_WIND_COMPONENTS -> add(_R_WIND_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
			_RtiAmb.updateAttributeValues(_ENVIRONMENT_VARIABLES_ObjectHandle, *ahvps_ENVIRONMENT_VARIABLES, "Environment_Variables");
            _RtiAmb.updateAttributeValues(_WIND_COMPONENTS_ObjectHandle,*ahvps_WIND_COMPONENTS,"Wind_Components");
        }
        else
        {
            _RtiAmb.updateAttributeValues(_ENVIRONMENT_VARIABLES_ObjectHandle, *ahvps_ENVIRONMENT_VARIABLES, UpdateTime, "Environment_Variables");
            _RtiAmb.updateAttributeValues(_WIND_COMPONENTS_ObjectHandle,*ahvps_WIND_COMPONENTS, UpdateTime,"Wind_Components");
        }
    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
	//std::cout << "EnvironmentFederateHla13::sendUpdateR: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void EnvironmentFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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
				// TO DO: it is not used for now, it will be used with 3D environement model
				//buffer.read_double(); 
			}
            else if (parmHandle == _LATITUDE_ATTRIBUTE)  
            { 
				buffer.read_double(); 
			}
            else if (parmHandle == _ALTITUDE_ATTRIBUTE)  
            { 
				_Atmosphere.setAltitude(buffer.read_double()); 
			}
            else
            { 
				cout << "EnvironmentFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
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
				cout << "EnvironmentFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_AIRCRAFT_ORIENTATION = true;
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
				
			}
            else if (parmHandle == _EQUIVALENT_AIRSPEED_ATTRIBUTE) 
            { 
				
			}
            else if (parmHandle == _CALIBRATED_AIRSPEED_ATTRIBUTE) 
            { 
				
			}
            else if (parmHandle == _TRUE_AIRSPEED_ATTRIBUTE)       
            { 
_Atmosphere.setTrueAirSpeed(buffer.read_double()); 
				
			}
            else if (parmHandle == _GROUND_SPEED_ATTRIBUTE)        
            { 
				
			}
            else if (parmHandle == _VERTICAL_SPEED_ATTRIBUTE)      
            { 
				
			}
            else if (parmHandle == _MACH_NUMBER_ATTRIBUTE)        
            { 
				
			}
            else
            { 
				cout << "EnvironmentFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_AIRCRAFT_SPEED = true;
    } 
	else
	{ 
		cout << "EnvironmentFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << endl; 
	}
}


// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void EnvironmentFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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
	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "reflectAttributeValues received with ts " <<  ((RTIfedTime&)theTime).getTime() << std::endl;
	#endif 
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void EnvironmentFederateHla13::timeRegulationEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeRegulationWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeReg = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void EnvironmentFederateHla13::timeConstrainedEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeConstrainedWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeConst = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeAdvanceGrant
void EnvironmentFederateHla13::timeAdvanceGrant(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::TimeAdvanceWasNotInProgress,
            RTI::FederateInternalError) 
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << " >> TAG RECU == LocalTime = " <<  _LocalTime << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void EnvironmentFederateHla13::enableTimeRegulation()
{
	_RtiAmb.enableTimeRegulation(_LocalTime, _Lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla13::disableTimeRegulation()
{
	_RtiAmb.disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla13::enableTimeConstrained()
{
	_RtiAmb.enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla13::disableTimeConstrained()
{
	_RtiAmb.disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla13::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb.enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla13::disableAsynchronousDelivery()
{
	_RtiAmb.disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla13::timeAdvanceRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.timeAdvanceRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void EnvironmentFederateHla13::timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime)
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
void EnvironmentFederateHla13::nextEventRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.nextEventRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_ENVIRONMENT_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// function : nextEventAvailable
void EnvironmentFederateHla13::nextEventAvailable(RTIfedTime NextLogicalTime)
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
void EnvironmentFederateHla13::synchronizationPointRegistrationSucceeded(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegSuccess = true ;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void EnvironmentFederateHla13::synchronizationPointRegistrationFailed(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegFailed = true ;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void EnvironmentFederateHla13::announceSynchronizationPoint(const char *label, const char *tag)
    throw ( RTI::FederateInternalError) 
{
       _InPause = true ;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void EnvironmentFederateHla13::federationSynchronized(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _InPause = false ;
} 

// ----------------------------------------------------------------------------
// function : run
void EnvironmentFederateHla13::run()
{
	while (_LocalTime < _SimulationEndTime)
	{
		// Model calculations
		calculateState();
		nextEventRequest(_TimeStep); 
		calculateOutput();
		sendUpdateAttributes(_LocalTime + _Lookahead);
		timeAdvanceRequest(_TimeStep);
	}
} 
