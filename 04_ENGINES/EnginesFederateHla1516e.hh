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

#ifndef __ENGINES_FEDERATE_HLA1516E_HH_DEF__
#define __ENGINES_FEDERATE_HLA1516E_HH_DEF__

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
#include <RTI/certiRTI1516.h>
#include <RTI/RTI1516.h>
#include <RTI/Enums.h>
#include <RTI/NullFederateAmbassador.h>
#include <RTI/RTI1516fedTime.h>
#include <RTI/LogicalTime.h>

// Specific includes
#include "MessageBuffer.hh"
#include "Engine.hh"
#include "Common.hh" 

class EnginesFederateHla1516e : public rti1516e::NullFederateAmbassador 
{
	// Private  elements
	private:
	
		// Internal models
		Engine _Right_Engine;
		Engine _Left_Engine;
	
		rti1516e::RTIambassador*    _RtiAmb;
		rti1516e::FederateHandle _FedHandle;
	
	    std::wstring          _FederateName;
        std::wstring          _FederationName;
        std::wstring          _FomFileName;	    
	    std::wstring          _TrimSyncPointName;
		std::wstring          _SimSyncPointName;
	    
	    rti1516e::FederateHandle _FederateHandle ;
	
		rti1516e::VariableLengthData _MyTag;
		rti1516e::VariableLengthData _MySyncTag;
		
		rti1516e::VariableLengthData _TrimTag;
		rti1516e::VariableLengthData _ActuatorTag;
		
		bool _IsTimeReg;
		bool _IsTimeConst;
		bool _IsTimeAdvanceGrant;
		bool _IsOutMesTimespamped;

		RTI1516fedTime _TimeStep;   
		RTI1516fedTime _Lookahead;   
		RTI1516fedTime _RestOfTimeStep; 			
		RTI1516fedTime _LocalTime;
		RTI1516fedTime _SimulationEndTime;

		bool _SyncRegSuccess;
		bool _SyncRegFailed;
		bool _InPause;
		bool _IsCreator;
		bool _IsSyncAnnonced;
		
		// Handles and Flags for subscribed datas 
		rti1516e::ObjectClassHandle _FLIGHT_CONTROLS_ClassHandle;
		rti1516e::ObjectClassHandle _AIRCRAFT_POSITION_ClassHandle; 
		rti1516e::ObjectClassHandle _AIRCRAFT_SPEED_ClassHandle; 
		rti1516e::ObjectClassHandle _ENVIRONMENT_VARIABLES_ClassHandle;
		rti1516e::ObjectClassHandle _TRIM_DATA_ClassHandle;
		rti1516e::ObjectInstanceHandle _FLIGHT_CONTROLS_ObjectHandle;
		rti1516e::ObjectInstanceHandle _TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle;
		rti1516e::ObjectInstanceHandle _TRIM_DATA_ENGINES_ObjectHandle;
		rti1516e::ObjectInstanceHandle _AIRCRAFT_POSITION_ObjectHandle;
		rti1516e::ObjectInstanceHandle _AIRCRAFT_SPEED_ObjectHandle;
		rti1516e::ObjectInstanceHandle _ENVIRONMENT_VARIABLES_ObjectHandle;
		rti1516e::AttributeHandle _RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE, 
							 _LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE,
							 _ALTITUDE_ATTRIBUTE,
							 _MACH_NUMBER_ATTRIBUTE,
							 _TEMPERATURE_ATTRIBUTE,
							 _DENSITY_OF_AIR_ATTRIBUTE,
							 _PRESSURE_ATTRIBUTE,
							 _ORDER_ATTRIBUTE,
							 _RIGHT_ENGINE_THRUST_EQ_ATTRIBUTE,
							 _LEFT_ENGINE_THRUST_EQ_ATTRIBUTE,
							 _RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE,
							 _LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE; 
		bool _Discov_FLIGHT_CONTROLS,
			 _Discov_AIRCRAFT_POSITION,
			 _Discov_AIRCRAFT_SPEED,
			 _Discov_ENVIRONMENT_VARIABLES,
			 _Discov_TRIM_DATA_FLIGHT_DYNAMICS;
		bool _New_FLIGHT_CONTROLS,
			 _New_AIRCRAFT_POSITION,
			 _New_AIRCRAFT_SPEED,
			 _New_ENVIRONMENT_VARIABLES,
			 _New_TRIM_DATA_FLIGHT_DYNAMICS;
		rti1516e::AttributeHandleSet attr_FLIGHT_CONTROLS;
		rti1516e::AttributeHandleSet attr_AIRCRAFT_POSITION;
		rti1516e::AttributeHandleSet attr_AIRCRAFT_SPEED;
		rti1516e::AttributeHandleSet attr_ENVIRONMENT_VARIABLES;
		rti1516e::AttributeHandleSet attr_TRIM_DATA_FLIGHT_DYNAMICS;

		// Handles and Flags for published datas
		MessageBuffer _OutputMessagebuffer;
		rti1516e::ObjectClassHandle _ACTUATORS_ClassHandle;
		rti1516e::ObjectInstanceHandle _ACTUATORS_ObjectHandle;
		rti1516e::AttributeHandle _RIGHT_ENGINE_THRUST_ATTRIBUTE,
		                     _LEFT_ENGINE_THRUST_ATTRIBUTE;
		rti1516e::AttributeHandleSet attr_ACTUATORS;  
		rti1516e::AttributeHandleSet attr_TRIM_DATA_ENGINES;
		//std::unique_ptr<rti1516e::AttributeHandleValuePairSet> ahvps_TRIM_DATA_ENGINES;
		//std::unique_ptr<rti1516e::AttributeHandleValuePairSet> ahvps_ACTUATORS;
		//std::unique_ptr<rti1516e::AttributeHandleValuePairSet> ahvps_FLIGHT_CONTROLS;
				
    // Public elements
	public:
	
		EnginesFederateHla1516e ( std::wstring FederationName
						  , std::wstring FederateName
					      , std::wstring FomFileName
					      );

		virtual ~EnginesFederateHla1516e();
		
		// wstring handling
		static const std::string getString(const std::wstring& wstr) {
			return std::string(wstr.begin(),wstr.end());
		};

		static const std::wstring getWString(const char* cstr) {
			std::wstringstream ss;
			ss << cstr;
			return std::wstring(ss.str());
		};
		
		// Federation Management
		void createFederationExecution();
		void destroyFederationExecution();
		void joinFederationExecution();
		void resignFederationExecution();
		rti1516e::FederateHandle getFederateHandle() const ;
		void getAllHandles();
		void publishAndSubscribe();
		void registerObjectInstances();
		void unpublishAndUnsubscribe();
		void waitForAllObjectDiscovered();
		void waitForAllAttributesReceived();
		void setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                         , RTI1516fedTime Lookahead
                                         , RTI1516fedTime LocalTime
                                         , RTI1516fedTime SimulationEndTime
                                         );
		void initialization();
		void calculateState();
        void calculateOutput();
        void sendInitialAttributes();
        void sendUpdateAttributes(RTI1516fedTime UpdateTime);
		void run();                                        
		void pauseInitTrim();
		void pauseInitSim();
		
		// Callback : discover object instance
		void discoverObjectInstance (rti1516e::ObjectInstanceHandle theObject,
		                             rti1516e::ObjectClassHandle theObjectClass,
		                             std::wstring const &theObjectInstanceName)
                                     throw (rti1516e::FederateInternalError);

		// Callback : reflect attribute values without time
		void reflectAttributeValues ( rti1516e::ObjectInstanceHandle theObject,
									rti1516e::AttributeHandleValueMap const &theAttributes,
									rti1516e::VariableLengthData const &theUserSuppliedTag,
									rti1516e::OrderType sentOrdering,
									rti1516e::TransportationType theTransport,
									rti1516e::SupplementalReflectInfo theReflectInfo)
							  throw (rti1516e::FederateInternalError) ;

		// Callback : reflect attribute values with time
		void reflectAttributeValues ( rti1516e::ObjectInstanceHandle theObject,
									rti1516e::AttributeHandleValueMap const &theAttributes,
									rti1516e::VariableLengthData const &theUserSuppliedTag,
									rti1516e::OrderType sentOrdering,
									rti1516e::TransportationType theTransport,
									rti1516e::LogicalTime const &theTime,
									rti1516e::OrderType receivedOrdering,
									rti1516e::MessageRetractionHandle theHandle,
									rti1516e::SupplementalReflectInfo theReflectInfo
									)
							  throw ( rti1516e::FederateInternalError) ;


		// HLA specific methods : TIME MANAGEMENT 
		// Callback : timeRegulationEnabled
		/*virtual void timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
		throw ( rti1516e::InvalidLogicalTime,
			rti1516e::NoRequestToEnableTimeRegulationWasPending,
			rti1516e::FederateInternalError) ;*/
		void timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
		throw (rti1516e::FederateInternalError) ;

		// Callback : timeConstrainedEnabled
		/*virtual void timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
		throw ( rti1516e::InvalidLogicalTime,
			rti1516e::NoRequestToEnableTimeConstrainedWasPending,
			rti1516e::FederateInternalError) ;*/
		void timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
		throw (rti1516e::FederateInternalError) ;
		
		// Callback : timeAdvanceGrant
		/*virtual void timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
		throw ( rti1516e::InvalidLogicalTime,
			rti1516e::JoinedFederateIsNotInTimeAdvancingState,
			rti1516e::FederateInternalError) ;*/
		void timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
		throw (rti1516e::FederateInternalError) ;
			
		void enableTimeRegulation();
		void enableTimeConstrained();
		void enableAsynchronousDelivery();
		void disableTimeRegulation();
		void disableTimeConstrained();
		void disableAsynchronousDelivery();
		void timeAdvanceRequest(RTI1516fedTime NextLogicalTime);
		void nextEventRequest(RTI1516fedTime NextLogicalTime);
		void timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime);
		void nextEventAvailable(RTI1516fedTime NextLogicalTime);

		// HLA specific methods : SYNCHRONISATION 
		// Callback : synchronizationPointRegistrationSucceeded
		void synchronizationPointRegistrationSucceeded(std::wstring const &label )
		throw (rti1516e::FederateInternalError) ;

		// Callback : synchronizationPointRegistrationFailed
		void synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
		throw (rti1516e::FederateInternalError) ;

		// Callback : announceSynchronizationPoint
		void announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag )
		throw (rti1516e::FederateInternalError) ;

		// Callback : federationSynchronized
		virtual void federationSynchronized( std::wstring const &label, rti1516e::FederateHandleSet const& failedToSyncSet)
			throw( rti1516e::FederateInternalError );
		
};


#endif //__EFCS_FEDERATE_HLA1516E_HH_DEF__
/// @}
