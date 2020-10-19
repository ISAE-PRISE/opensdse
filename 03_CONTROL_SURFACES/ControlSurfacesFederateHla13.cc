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

#include "ControlSurfacesFederateHla13.hh"

// ----------------------------------------------------------------------------
// ControlSurfacesFederateHla13 Constructor
ControlSurfacesFederateHla13::ControlSurfacesFederateHla13( std::wstring FederationName
									, std::wstring FederateName
									, std::wstring FomFileName
									) 
									: NullFederateAmbassador()
									, _RtiAmb() 
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << "ControlSurfacesFederateHla13.cc -> Constructor(): Start" << std::endl;
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
	std::cout << "ControlSurfacesFederateHla13.cc -> Constructor(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// ControlSurfacesFederateHla13 Destructor
ControlSurfacesFederateHla13::~ControlSurfacesFederateHla13()
	                     throw(RTI::FederateInternalError)
{
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void ControlSurfacesFederateHla13::createFederationExecution() 
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << "ControlSurfacesFederateHla13.cc -> createFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> createFederationExecution(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void ControlSurfacesFederateHla13::destroyFederationExecution() 
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << "ControlSurfacesFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
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
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << "ControlSurfacesFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla13::joinFederationExecution() 
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << "ControlSurfacesFederateHla13.cc -> joinFederationExecution(): Start" << std::endl;
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
    #ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << "ControlSurfacesFederateHla13.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla13::resignFederationExecution() 
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb.resignFederationExecution(RTI::DELETE_OBJECTS_AND_RELEASE_ATTRIBUTES);
	}
	catch (RTI::Exception& e) 
	{
		std::cout << "RFE: caught " << e._name << " reason " << e._reason <<std::endl;
	}
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << "ControlSurfacesFederateHla13.cc -> resignFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Get the federate handle
RTI::FederateHandle ControlSurfacesFederateHla13::getFederateHandle() const
{
    return _FederateHandle;
}

// ----------------------------------------------------------------------------
// Set all Time management settings for federate
void ControlSurfacesFederateHla13::setHlaTimeManagementSettings( RTIfedTime TimeStep
                                                    , RTIfedTime Lookahead
                                                    , RTIfedTime LocalTime
                                                    , RTIfedTime SimulationEndTime
                                                    )
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
    #endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> setHlaTimeManagementSettings(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void ControlSurfacesFederateHla13::getAllHandles()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed
		_FLIGHT_CONTROLS_ClassHandle = _RtiAmb.getObjectClassHandle("FLIGHT_CONTROLS"); 
		_TRIM_DATA_ClassHandle = _RtiAmb.getObjectClassHandle("TRIM_DATA");     
        _ELEVATOR_DEFLECTION_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("ELEVATOR_DEFLECTION_EQ",_TRIM_DATA_ClassHandle);
        _STABILIZER_DEFLECTION_EQ_ATTRIBUTE = _RtiAmb.getAttributeHandle("STABILIZER_DEFLECTION_EQ",_TRIM_DATA_ClassHandle);    
        _RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_AILERON_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_AILERON_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ELEVATOR_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ELEVATOR_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RUDDER_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("FLAPS_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("SPOILERS_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("STABILIZER_COMMANDED_DEFLECTION",_FLIGHT_CONTROLS_ClassHandle);
        _GEARS_COMMANDED_POSITION_ATTRIBUTE = _RtiAmb.getAttributeHandle("GEARS_COMMANDED_POSITION",_FLIGHT_CONTROLS_ClassHandle);
        // Published 
        _ACTUATORS_ClassHandle = _RtiAmb.getObjectClassHandle("ACTUATORS");
        _RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_AILERON_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);
        _LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_AILERON_EFFECTIVE_DEFLECTION",  _ACTUATORS_ClassHandle);
        _RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION",_ACTUATORS_ClassHandle);
        _LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("LEFT_ELEVATOR_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);
        _RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("RUDDER_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);  
        _FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("FLAPS_EFFECTIVE_DEFLECTION", _ACTUATORS_ClassHandle);
        _SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("SPOILERS_EFFECTIVE_DEFLECTION",  _ACTUATORS_ClassHandle);
        _STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE = _RtiAmb.getAttributeHandle("STABILIZER_EFFECTIVE_DEFLECTION",  _ACTUATORS_ClassHandle);
        _GEARS_POSITION_ATTRIBUTE = _RtiAmb.getAttributeHandle("GEARS_POSITION",  _ACTUATORS_ClassHandle);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void ControlSurfacesFederateHla13::publishAndSubscribe()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
		// For Class/Attributes subscribed
		attr_FLIGHT_CONTROLS.reset(RTI::AttributeHandleSetFactory::create(9)) ;
		attr_TRIM_DATA_FLIGHT_DYNAMICS.reset(RTI::AttributeHandleSetFactory::create(2)) ;
			
		attr_FLIGHT_CONTROLS->add(_RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE);
		attr_FLIGHT_CONTROLS->add(_GEARS_COMMANDED_POSITION_ATTRIBUTE);
		
		attr_TRIM_DATA_FLIGHT_DYNAMICS->add(_ELEVATOR_DEFLECTION_EQ_ATTRIBUTE);
		attr_TRIM_DATA_FLIGHT_DYNAMICS->add(_STABILIZER_DEFLECTION_EQ_ATTRIBUTE);

		_RtiAmb.subscribeObjectClassAttributes(_FLIGHT_CONTROLS_ClassHandle,*attr_FLIGHT_CONTROLS);
        _RtiAmb.subscribeObjectClassAttributes(_TRIM_DATA_ClassHandle,*attr_TRIM_DATA_FLIGHT_DYNAMICS);

		// For Class/Attributes published
		attr_ACTUATORS.reset(RTI::AttributeHandleSetFactory::create(9)) ;
		attr_ACTUATORS->add(_RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE);     
		attr_ACTUATORS->add(_FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE);
		attr_ACTUATORS->add(_GEARS_POSITION_ATTRIBUTE);
		
        _RtiAmb.publishObjectClass(_ACTUATORS_ClassHandle,*attr_ACTUATORS);
	} 
	catch ( RTI::Exception &e ) 
	{
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }   
    #ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void ControlSurfacesFederateHla13::registerObjectInstances()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
	{
        _ACTUATORS_ObjectHandle = _RtiAmb.registerObjectInstance(_ACTUATORS_ClassHandle,"Actuators_Control_Surfaces");
        ahvps_ACTUATORS.reset(RTI::AttributeSetFactory::create(9));
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> registerObjectInstances(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void ControlSurfacesFederateHla13::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb.unsubscribeObjectClass(_TRIM_DATA_ClassHandle);
        _RtiAmb.unsubscribeObjectClass(_FLIGHT_CONTROLS_ClassHandle);
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
       _RtiAmb.unpublishObjectClass(_ACTUATORS_ClassHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << endl;
    } 
    catch ( ... ) 
    {
        cerr << "Error: unknown non-RTI exception." << endl;
    }
    #ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void ControlSurfacesFederateHla13::waitForAllObjectDiscovered()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> waitForAllObjectDiscovered(): Start" << std::endl;
    #endif
	while ( !_Discov_FLIGHT_CONTROLS            ||
            !_Discov_TRIM_DATA_FLIGHT_DYNAMICS)
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
    _Discov_FLIGHT_CONTROLS = false;
    _Discov_TRIM_DATA_FLIGHT_DYNAMICS = false;
    
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> waitForAllObjectDiscovered(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
void ControlSurfacesFederateHla13::waitForAllAttributesReceived()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> waitForAllAttributesReceived(): Start" << std::endl;
    #endif
    while (!_New_TRIM_DATA_FLIGHT_DYNAMICS) 
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
    _New_TRIM_DATA_FLIGHT_DYNAMICS = false;
    #ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> waitForAllAttributesReceived(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
void ControlSurfacesFederateHla13::discoverObjectInstance( RTI::ObjectHandle theObject
                                                   , RTI::ObjectClassHandle theObjectClass
                                                   , const char *theObjectName
                                                   )
                                             throw ( RTI::CouldNotDiscover
                                                   , RTI::ObjectClassNotKnown
                                                   , RTI::FederateInternalError
                                                   )
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    if ( (theObjectClass == _FLIGHT_CONTROLS_ClassHandle) && (!strcmp(theObjectName,"Flight_Controls")) ) 
    {
        _Discov_FLIGHT_CONTROLS = true;
        _FLIGHT_CONTROLS_ObjectHandle = theObject;
        #ifdef DEBUG_CONTROL_SURFACES_FED
        cout << "< Flight_Controls > Object Instance has been discovered" << endl;
        #endif
    } 
    else if ( (theObjectClass == _TRIM_DATA_ClassHandle) && (!strcmp(theObjectName,"Trim_Data_Flight_Dynamics")) ) 
    {
        _Discov_TRIM_DATA_FLIGHT_DYNAMICS = true;
        _TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle = theObject;
        #ifdef DEBUG_CONTROL_SURFACES_FED
        cout << "< Trim_Data_Flight_Dynamics > Object Instance has been discovered" << endl;
        #endif
    } 
    #ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void ControlSurfacesFederateHla13::pauseInitTrim()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> pause_init_trim(): Start" << std::endl;
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
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> pause_init_trim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Put the federation in pause waiting for Trim Sync
void ControlSurfacesFederateHla13::pauseInitSim()
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> pause_init_sim(): Start" << std::endl;
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
	#ifdef DEBUG_CONTROL_SURFACES_FED
    std::cout << "ControlSurfacesFederateHla13.cc -> pause_init_sim(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Init phase for Efcs
void ControlSurfacesFederateHla13::initialization() 
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
void ControlSurfacesFederateHla13::calculateState() 
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
void ControlSurfacesFederateHla13::calculateOutput() 
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
	cout << " Actual and Required Control Surface position (degrees) : "																		  << endl;
	cout << " RIGHT AILERON  = " << _Right_Aileron.getDeltaActualDeg() 	<< " ( Demand = "  << _Right_Aileron.getDeltaDemandDeg() << " ) " << endl;
	cout << " LEFT AILERON   = " << _Left_Aileron.getDeltaActualDeg() 	<< " ( Demand = "  << _Left_Aileron.getDeltaDemandDeg()  << " ) " << endl;
	cout << " RIGHT ELEVATOR = " << _Right_Elevator.getDeltaActualDeg() << " ( Demand = "  << _Right_Elevator.getDeltaDemandDeg()<< " ) " << endl;
	cout << " LEFT ELEVATOR  = " << _Left_Elevator.getDeltaActualDeg() 	<< " ( Demand = "  << _Left_Elevator.getDeltaDemandDeg() << " ) " << endl;
	cout << " RUDDER         = " << _Rudder.getDeltaActualDeg()			<< " ( Demand = "  << _Rudder.getDeltaDemandDeg() 		 << " ) " << endl;
	cout << " FLAPS          = " << _Flaps.getDeltaActualDeg()			<< " ( Demand = "  << _Flaps.getDeltaDemandDeg() 		 << " ) " << endl;
	cout << " SPOILERS       = " << _Spoilers.getDeltaActualDeg()       << " ( Demand = "  << _Spoilers.getDeltaDemandDeg() 	 << " ) " << endl;
	cout << " STABILIZER     = " << _Stabilizer.getDeltaActualDeg()     << " ( Demand = "  << _Stabilizer.getDeltaDemandDeg() 	 << " ) " << endl;
	cout << " GEARS          = " << _Gears.getDeltaActualDeg()			<< " ( Demand = "  << _Gears.getDeltaDemandDeg() 		 << " ) " << endl;
	cout << " " << endl ;
	#endif
}

// ----------------------------------------------------------------------------
// Send Updates for all attributes
void ControlSurfacesFederateHla13::sendUpdateAttributes(const RTI::FedTime& UpdateTime)
{   
	//std::cout << "ControlSurfacesFederateHla13::sendUpdateAttributes: Start" << std::endl;
	ahvps_ACTUATORS->empty();
    _OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Right_Aileron.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_ACTUATORS -> add(_RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Left_Aileron.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_ACTUATORS -> add(_LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Right_Elevator.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_ACTUATORS -> add(_RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Left_Elevator.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_ACTUATORS -> add(_LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Rudder.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_ACTUATORS -> add(_RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size()); 

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Flaps.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_ACTUATORS -> add(_FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Spoilers.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_ACTUATORS -> add(_SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Stabilizer.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
	ahvps_ACTUATORS -> add(_STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());

	_OutputMessagebuffer.reset() ;
	_OutputMessagebuffer.write_double(_Gears.getDeltaActualRad());
	_OutputMessagebuffer.updateReservedBytes();
    ahvps_ACTUATORS -> add(_GEARS_POSITION_ATTRIBUTE, static_cast<char*>(_OutputMessagebuffer(0)), _OutputMessagebuffer.size());
    
    try 
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb.updateAttributeValues(_ACTUATORS_ObjectHandle, *ahvps_ACTUATORS, "");
        }
        else
        {
            _RtiAmb.updateAttributeValues(_ACTUATORS_ObjectHandle, *ahvps_ACTUATORS, UpdateTime, "");
        }
    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
	//std::cout << "ControlSurfacesFederateHla13::sendUpdateR: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void ControlSurfacesFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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

    if (theObject == _FLIGHT_CONTROLS_ObjectHandle)
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
				cout << "ControlSurfacesFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }
        _New_FLIGHT_CONTROLS = true;
    } 
    else if (theObject == _TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle)
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
            double tmp;                 
            
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
				cout << "ControlSurfacesFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
				     << theObject 
				     << "." 
				     << endl; 
			}
        }    
        _New_TRIM_DATA_FLIGHT_DYNAMICS = true;
    } 
	else
	{ 
		cout << "ControlSurfacesFederateHla13.cc: RAV ==> ERROR: unknown handle for object " 
			 << theObject 
			 << "." 
			 << endl; 
	} 
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void ControlSurfacesFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
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
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << "reflectAttributeValues received with ts " <<  ((RTIfedTime&)theTime).getTime() << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void ControlSurfacesFederateHla13::timeRegulationEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeRegulationWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeReg = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void ControlSurfacesFederateHla13::timeConstrainedEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeConstrainedWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeConst = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeAdvanceGrant
void ControlSurfacesFederateHla13::timeAdvanceGrant(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::TimeAdvanceWasNotInProgress,
            RTI::FederateInternalError) 
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << " >> TAG RECU == LocalTime = " <<  _LocalTime << " <<" << std::endl;
	#endif
} 

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla13::enableTimeRegulation()
{
	_RtiAmb.enableTimeRegulation(_LocalTime, _Lookahead);
	while (!_IsTimeReg) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla13::disableTimeRegulation()
{
	_RtiAmb.disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla13::enableTimeConstrained()
{
	_RtiAmb.enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla13::disableTimeConstrained()
{
	_RtiAmb.disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla13::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb.enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla13::disableAsynchronousDelivery()
{
	_RtiAmb.disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla13::timeAdvanceRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.timeAdvanceRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << "timeAdvanceRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
void ControlSurfacesFederateHla13::timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime)
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
void ControlSurfacesFederateHla13::nextEventRequest(RTIfedTime NextLogicalTime)
{
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " begin" << std::endl;
	#endif
	_RtiAmb.nextEventRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}
	_IsTimeAdvanceGrant = false;
	#ifdef DEBUG_CONTROL_SURFACES_FED
	std::cout << "nextEventRequest requested to " <<  NextLogicalTime << " end" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// function : nextEventAvailable
void ControlSurfacesFederateHla13::nextEventAvailable(RTIfedTime NextLogicalTime)
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
void ControlSurfacesFederateHla13::synchronizationPointRegistrationSucceeded(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegSuccess = true ;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void ControlSurfacesFederateHla13::synchronizationPointRegistrationFailed(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegFailed = true ;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void ControlSurfacesFederateHla13::announceSynchronizationPoint(const char *label, const char *tag)
    throw ( RTI::FederateInternalError) 
{
       _InPause = true ;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void ControlSurfacesFederateHla13::federationSynchronized(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _InPause = false ;
} 

// ----------------------------------------------------------------------------
// function : run
void ControlSurfacesFederateHla13::run()
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
