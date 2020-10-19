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

#ifndef __FLIGHT_DYNAMICS_FEDERATE_HLA13_HH_DEF__
#define __FLIGHT_DYNAMICS_FEDERATE_HLA13_HH_DEF__

// System includes
#include <string>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <memory>
#include <vector>
#include <iterator>
#include <assert.h>
#include <time.h>
#include <limits>

// RTI includes
#include <RTI.hh>
#include <NullFederateAmbassador.hh>
#include <fedtime.hh>

// Specific includes
#include "MessageBuffer.hh"
#include "FlightDynamics.hh"
#include "Common.hh" 

class FlightDynamicsFederateHla13 : public NullFederateAmbassador 
{
	private:
	
		// Internal model
		FlightDynamics _Aircraft;
	
		RTI::RTIambassador    _RtiAmb;
	
	    std::wstring          _FederateName;
        std::wstring          _FederationName;
        std::wstring          _FomFileName;	    
	    std::wstring          _TrimSyncPointName;
		std::wstring          _SimSyncPointName;
	    
	    RTI::FederateHandle _FederateHandle ;
	
		bool _IsTimeReg;
		bool _IsTimeConst;
		bool _IsTimeAdvanceGrant;
		bool _IsOutMesTimespamped;

		RTIfedTime _TimeStep;   
		RTIfedTime _Lookahead;   
		RTIfedTime _RestOfTimeStep; 			
		RTIfedTime _LocalTime;
		RTIfedTime _SimulationEndTime;

		bool _SyncRegSuccess;
		bool _SyncRegFailed;
		bool _InPause;
		bool _IsCreator;
		
		// Handles and Flags for subscribed datas
		RTI::ObjectClassHandle _ACTUATORS_ClassHandle; 
		RTI::ObjectClassHandle _ENVIRONMENT_VARIABLES_ClassHandle; 
		RTI::ObjectClassHandle _WIND_COMPONENTS_ClassHandle;
		RTI::ObjectClassHandle _USER_DATA_ClassHandle;
		RTI::ObjectHandle _ACTUATORS_CONTROL_SURFACES_ObjectHandle;
		RTI::ObjectHandle _ACTUATORS_ENGINES_ObjectHandle;
		RTI::ObjectHandle _ENVIRONMENT_VARIABLES_ObjectHandle;
		RTI::ObjectHandle _WIND_COMPONENTS_ObjectHandle;
		RTI::AttributeHandle _RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _RIGHT_ENGINE_THRUST_ATTRIBUTE,
							 _LEFT_ENGINE_THRUST_ATTRIBUTE,
							 _FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _GEARS_POSITION_ATTRIBUTE,
							 _TEMPERATURE_ATTRIBUTE, 
							 _DENSITY_OF_AIR_ATTRIBUTE, 
							 _PRESSURE_ATTRIBUTE,
							 _SPEED_OF_SOUND_ATTRIBUTE,
							 _U_WIND_ATTRIBUTE, 
							 _V_WIND_ATTRIBUTE, 
							 _W_WIND_ATTRIBUTE,
							 _P_WIND_ATTRIBUTE, 
							 _Q_WIND_ATTRIBUTE, 
							 _R_WIND_ATTRIBUTE; 
		bool _Discov_ACTUATORS_CONTROL_SURFACES,
			 _Discov_ACTUATORS_ENGINES,
			 _Discov_ENVIRONMENT_VARIABLES,
			 _Discov_WIND_COMPONENTS;
		bool _New_ACTUATORS_CONTROL_SURFACES,
			 _New_ACTUATORS_ENGINES,
			 _New_ENVIRONMENT_VARIABLES,
			 _New_WIND_COMPONENTS;
		std::unique_ptr<RTI::AttributeHandleSet> attr_ACTUATORS;
		std::unique_ptr<RTI::AttributeHandleSet> attr_ENVIRONMENT_VARIABLES;
		std::unique_ptr<RTI::AttributeHandleSet> attr_WIND_COMPONENTS;

		// Handles and Flags for published datas
		MessageBuffer _OutputMessagebuffer;
		RTI::ObjectClassHandle _TRIM_DATA_ClassHandle;
		RTI::ObjectClassHandle _AIRCRAFT_POSITION_ClassHandle; 
		RTI::ObjectClassHandle _AIRCRAFT_ORIENTATION_ClassHandle; 
		RTI::ObjectClassHandle _AIRCRAFT_UVW_SPEED_ClassHandle; 
		RTI::ObjectClassHandle _AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle;
		RTI::ObjectClassHandle _AIRCRAFT_ACCELERATION_ClassHandle;
		RTI::ObjectClassHandle _AIRCRAFT_SPEED_ClassHandle;
		RTI::ObjectClassHandle _AIRCRAFT_ADDITIONAL_ClassHandle;
		RTI::ObjectHandle _TRIM_DATA_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_POSITION_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_ORIENTATION_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_UVW_SPEED_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_ACCELERATION_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_SPEED_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_ADDITIONAL_ObjectHandle;
		RTI::AttributeHandle _RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE, 
							 _LEFT_ENGINE_THRUST_EQ_ATTRIBUTE, 
							 _ELEVATOR_DEFLECTION_EQ_ATTRIBUTE,
							 _STABILIZER_DEFLECTION_EQ_ATTRIBUTE,
							 _LONGITUDE_ATTRIBUTE,
							 _LATITUDE_ATTRIBUTE,
							 _ALTITUDE_ATTRIBUTE,
							 _PHI_ATTRIBUTE,
							 _THETA_ATTRIBUTE,
							 _PSI_ATTRIBUTE,
							 _U_SPEED_ATTRIBUTE,
							 _V_SPEED_ATTRIBUTE,
							 _W_SPEED_ATTRIBUTE,
							 _P_ANG_SPEED_ATTRIBUTE, 
							 _Q_ANG_SPEED_ATTRIBUTE, 
							 _R_ANG_SPEED_ATTRIBUTE,
							 _X_ACC_ATTRIBUTE, 
							 _Y_ACC_ATTRIBUTE, 
							 _Z_ACC_ATTRIBUTE,
							 _INDICATED_AIRSPEED_ATTRIBUTE, 
							 _EQUIVALENT_AIRSPEED_ATTRIBUTE, 
							 _CALIBRATED_AIRSPEED_ATTRIBUTE,
							 _TRUE_AIRSPEED_ATTRIBUTE,
							 _GROUND_SPEED_ATTRIBUTE,
							 _VERTICAL_SPEED_ATTRIBUTE,
							 _MACH_NUMBER_ATTRIBUTE,
							 _ALPHA_ATTRIBUTE, 
							 _BETA_ATTRIBUTE,
							 _DYNAMIC_PRESSURE_ATTRIBUTE,
							 _TANK_FILLING_ATTRIBUTE;
		std::unique_ptr<RTI::AttributeHandleSet> attr_TRIM_DATA;  
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_POSITION;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_ORIENTATION;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_UVW_SPEED;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_PQR_ANGULAR_SPEED;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_ACCELERATION;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_SPEED;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_ADDITIONAL;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_TRIM_DATA;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_AIRCRAFT_POSITION;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_AIRCRAFT_ORIENTATION;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_AIRCRAFT_UVW_SPEED;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_AIRCRAFT_PQR_ANGULAR_SPEED;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_AIRCRAFT_ACCELERATION;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_AIRCRAFT_SPEED;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_AIRCRAFT_ADDITIONAL;
				
	public:
	
		FlightDynamicsFederateHla13 ( std::wstring FederationName
						  , std::wstring FederateName
					      , std::wstring FomFileName
					      );

		virtual ~FlightDynamicsFederateHla13() throw(RTI::FederateInternalError);
		
		// wstring handling
		static const std::string getString(const std::wstring& wstr) {
			return std::string(wstr.begin(),wstr.end());
		};

		static const std::wstring getWString(const char* cstr) {
			std::wstringstream ss;
			ss << cstr;
			return std::wstring(ss.str());
		};
		
		bool getEndOfSimulation();
		
		// Federation Management
		void createFederationExecution();
		void destroyFederationExecution();
		void joinFederationExecution();
		void resignFederationExecution();
		RTI::FederateHandle getFederateHandle() const ;
		void getAllHandles();
		void publishAndSubscribe();
		void registerObjectInstances();
		void unpublishAndUnsubscribe();
		void waitForAllObjectDiscovered();
		void waitForAllAttributesReceived();
		void setHlaTimeManagementSettings( RTIfedTime TimeStep
                                         , RTIfedTime Lookahead
                                         , RTIfedTime LocalTime
                                         , RTIfedTime SimulationEndTime
                                         );
		void initialization();
		void calculateState();
        void calculateOutput();
        void sendInitialAttributesPos();
        void sendInitialAttributesTrim();
        void sendInitialAllAttributes();
        void sendUpdateAttributes1(const RTI::FedTime& UpdateTime);
        void sendUpdateAttributes2(const RTI::FedTime& UpdateTime);
		void run();
		void runOneStep();                                        
		void pauseInitTrim();
		void pauseInitSim();

		// Callback : discover object instance
		void discoverObjectInstance ( RTI::ObjectHandle theObject
									, RTI::ObjectClassHandle theObjectClass
									, const char *theObjectName
									)
		throw ( RTI::CouldNotDiscover,
			RTI::ObjectClassNotKnown,
			RTI::FederateInternalError);

		// Callback : reflect attribute values without time
		void reflectAttributeValues ( RTI::ObjectHandle theObject
									, const RTI::AttributeHandleValuePairSet& theAttributes
									, const char *theTag
									)
							  throw ( RTI::ObjectNotKnown
							        , RTI::AttributeNotKnown
							        , RTI::FederateOwnsAttributes
							        , RTI::FederateInternalError
							        ) ;

		// Callback : reflect attribute values with time
		void reflectAttributeValues ( RTI::ObjectHandle theObject
									, const RTI::AttributeHandleValuePairSet& theAttributes
									, const RTI::FedTime& theTime
									, const char *theTag
									, RTI::EventRetractionHandle
									)
							  throw ( RTI::ObjectNotKnown
								    , RTI::AttributeNotKnown
								    , RTI::FederateOwnsAttributes
								    , RTI::InvalidFederationTime 
								    , RTI::FederateInternalError
								    ) ;


		// HLA specific methods : TIME MANAGEMENT 
		// Callback : timeRegulationEnabled
		void timeRegulationEnabled(const RTI::FedTime& theTime)
		throw ( RTI::InvalidFederationTime,
			RTI::EnableTimeRegulationWasNotPending,
			RTI::FederateInternalError) ;

		// Callback : timeConstrainedEnabled
		void timeConstrainedEnabled(const RTI::FedTime& theTime)
		throw ( RTI::InvalidFederationTime,
			RTI::EnableTimeConstrainedWasNotPending,
			RTI::FederateInternalError) ;

		// Callback : timeAdvanceGrant
		void timeAdvanceGrant(const RTI::FedTime& theTime)
		throw ( RTI::InvalidFederationTime,
			RTI::TimeAdvanceWasNotInProgress,
			RTI::FederateInternalError) ;
			
		void enableTimeRegulation();
		void enableTimeConstrained();
		void enableAsynchronousDelivery();
		void disableTimeRegulation();
		void disableTimeConstrained();
		void disableAsynchronousDelivery();
		void timeAdvanceRequest(RTIfedTime NextLogicalTime);
		void nextEventRequest(RTIfedTime NextLogicalTime);
		void timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime);
		void nextEventAvailable(RTIfedTime NextLogicalTime);

		// HLA specific methods : SYNCHRONISATION 
		// Callback : synchronizationPointRegistrationSucceeded
		void synchronizationPointRegistrationSucceeded(const char *label)
		throw (RTI::FederateInternalError) ;

		// Callback : synchronizationPointRegistrationFailed
		void synchronizationPointRegistrationFailed(const char *label)
		throw (RTI::FederateInternalError) ;

		// Callback : announceSynchronizationPoint
		void announceSynchronizationPoint(const char *label, const char *tag)
		throw (RTI::FederateInternalError) ;

		// Callback : federationSynchronized
		void federationSynchronized(const char *label)
		throw (RTI::FederateInternalError) ;
		
};

#endif //__FLIGHT_DYNAMICS_FEDERATE_HLA13_HH_DEF__
