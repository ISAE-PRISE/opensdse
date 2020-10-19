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

#ifndef __CONTROL_SURFACES_FEDERATE_HLA13_HH_DEF__
#define __CONTROL_SURFACES_FEDERATE_HLA13_HH_DEF__

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
#include "UnitConversion.hh"
#include "MessageBuffer.hh"
#include "ControlSurface.hh"
#include "A320.hh"
#include "Common.hh" 

class ControlSurfacesFederateHla13 : public NullFederateAmbassador
                                   , public A320
                                   , public UnitConversion
{
	private:
	
		// Internal models
		ControlSurface _Right_Aileron;
        ControlSurface _Left_Aileron;
        ControlSurface _Right_Elevator;
        ControlSurface _Left_Elevator;
        ControlSurface _Rudder;
        ControlSurface _Flaps;
        ControlSurface _Spoilers;
        ControlSurface _Stabilizer;
        ControlSurface _Gears;
	
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
		RTI::ObjectClassHandle _FLIGHT_CONTROLS_ClassHandle; 
		RTI::ObjectClassHandle _TRIM_DATA_ClassHandle; 
		RTI::ObjectHandle _FLIGHT_CONTROLS_ObjectHandle;
		RTI::ObjectHandle _TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle;   
		RTI::AttributeHandle _RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE, 
							 _LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _GEARS_COMMANDED_POSITION_ATTRIBUTE,
							 _ELEVATOR_DEFLECTION_EQ_ATTRIBUTE,
							 _STABILIZER_DEFLECTION_EQ_ATTRIBUTE; 
		bool _Discov_FLIGHT_CONTROLS;
		bool _Discov_TRIM_DATA_FLIGHT_DYNAMICS;  
		bool _New_FLIGHT_CONTROLS;
		bool _New_TRIM_DATA_FLIGHT_DYNAMICS;
		std::unique_ptr<RTI::AttributeHandleSet> attr_FLIGHT_CONTROLS;
		std::unique_ptr<RTI::AttributeHandleSet> attr_TRIM_DATA_FLIGHT_DYNAMICS;

		// Handles and Flags for published datas
		MessageBuffer _OutputMessagebuffer;
		RTI::ObjectClassHandle _ACTUATORS_ClassHandle;
		RTI::ObjectHandle _ACTUATORS_ObjectHandle;
		RTI::AttributeHandle _RIGHT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE, 
							 _LEFT_AILERON_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _RIGHT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _LEFT_ELEVATOR_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _RUDDER_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _FLAPS_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _SPOILERS_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _STABILIZER_EFFECTIVE_DEFLECTION_ATTRIBUTE,
							 _GEARS_POSITION_ATTRIBUTE;
		std::unique_ptr<RTI::AttributeHandleSet> attr_ACTUATORS;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_ACTUATORS;
			
		// Local variables				 
		double mDt;
		int mIterations;
		int mIntegrationMethod;
		int mSaturation;
				
	public:
	
		ControlSurfacesFederateHla13 ( std::wstring FederationName
						  , std::wstring FederateName
					      , std::wstring FomFileName
					      );

		virtual ~ControlSurfacesFederateHla13() throw(RTI::FederateInternalError);
		
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
        void sendUpdateAttributes(const RTI::FedTime& UpdateTime);
		void run();                                        
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

#endif //__CONTROL_SURFACES_FEDERATE_HLA13_HH_DEF__
