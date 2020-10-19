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

#ifndef __JOYSTICK_FEDERATE_HLA13_HH_DEF__
#define __JOYSTICK_FEDERATE_HLA13_HH_DEF__

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
#include "Joystick.hh"
#include "Common.hh" 

class JoystickFederateHla13 : public NullFederateAmbassador 
{
	private:
	
		// Internal model
		Joystick _Joystick ;
	
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
		
		bool _IsRtTimerEnabled;

		// Handles and Flags for published datas
		MessageBuffer _OutputMessagebuffer;
		RTI::ObjectClassHandle _JOYSTICK_ClassHandle; 
		RTI::ObjectHandle _JOYSTICK_ObjectHandle;
		RTI::AttributeHandle _AILERON_ATTRIBUTE, 
						 _ELEVATOR_ATTRIBUTE, 
						 _RUDDER_ATTRIBUTE, 
						 _THROTTLE_LEFT_ATTRIBUTE, 
						 _THROTTLE_RIGHT_ATTRIBUTE,
						 _FLAPS_ATTRIBUTE,
						 _SPOILERS_ATTRIBUTE,
						 _GEARS_ATTRIBUTE,
						 _BRAKES_ATTRIBUTE;
		 std::unique_ptr<RTI::AttributeHandleSet> attr_JOYSTICK;
		 std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_JOYSTICK;
						 
		// Local variables
		double _Aileron, 
		_Elevator, 
		_Rudder, 
		_Throttle_Left, 
		_Throttle_Right, 
		_Flaps, 
		_Spoilers, 
		_Gears, 
		_Brakes;

		// Port numbers of the joystick (called input in config file) 
		int _Aileron_port, 
		_Elevator_port, 
		_Rudder_port, 
		_Throttle_Left_port, 
		_Throttle_Right_port, 
		_Flaps_Up_port, 
		_Flaps_Down_port, 
		_Spoilers_port, 
		_Gears_port, 
		_Brakes_port;

		// Joystick numbers (called port in config file)
		int _Aileron_jnb, 
		_Elevator_jnb, 
		_Rudder_jnb, 
		_Throttle_Left_jnb, 
		_Throttle_Right_jnb,  
		_Flaps_Up_jnb, 
		_Flaps_Down_jnb, 
		_Spoilers_jnb, 
		_Gears_jnb, 
		_Brakes_jnb;

		// Joystick sensitivity
		double _Aileron_sens, 
		_Elevator_sens, 
		_Rudder_sens, 
		_Throttle_Left_sens, 
		_Throttle_Right_sens;

		// Booleans for buttons handling
		bool _Flaps_Up_released, 
		_Flaps_Down_released, 
		_Spoilers_released, 
		_Gears_released, 
		_Brakes_released;
		
		// For simulation versus realtime measurements
		timespec _TimeStamp, _TimeStamp_old;
		timespec _ExecutionTime;
		double _TotalRealTimeMs;
				
	public:
	
		JoystickFederateHla13 ( std::wstring FederationName
						      , std::wstring FederateName
					          , std::wstring FomFileName
					          );

		virtual ~JoystickFederateHla13() throw(RTI::FederateInternalError);
		
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
		void sendInitialAttributes();
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

#endif //__JOYSTICK_FEDERATE_HLA13_HH_DEF__
