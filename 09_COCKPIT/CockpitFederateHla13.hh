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

#ifndef __COCKPIT_FEDERATE_HLA13_HH_DEF__
#define __COCKPIT_FEDERATE_HLA13_HH_DEF__

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
#include "Cockpit.hh"
#include "Common.hh" 

class CockpitFederateHla13 : public NullFederateAmbassador 
{
	private:
	
		// Internal model
		Cockpit _Cockpit;
	
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
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_POSITION_ClassHandle; 
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_ORIENTATION_ClassHandle; 
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_UVW_SPEED_ClassHandle; 
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle;
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_ACCELERATION_ClassHandle;
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_SPEED_ClassHandle;
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle;
		RTI::ObjectClassHandle _ACTUATORS_ClassHandle;
		RTI::ObjectClassHandle _ENVIRONMENT_VARIABLES_ClassHandle; 
		RTI::ObjectHandle _MEAS_AIRCRAFT_POSITION_ObjectHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_ORIENTATION_ObjectHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_ACCELERATION_ObjectHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_SPEED_ObjectHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle; 
		RTI::ObjectHandle _ACTUATORS_ENGINES_ObjectHandle;  
		RTI::ObjectHandle _ENVIRONMENT_VARIABLES_ObjectHandle;          
		RTI::AttributeHandle _AUTOPILOT_AP_ACTIVE_ATTRIBUTE, 
							 _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE, 
							 _AUTOPILOT_SPD_ATTRIBUTE, 
							 _AUTOPILOT_HDG_ATTRIBUTE, 
							 _AUTOPILOT_ALT_ATTRIBUTE, 
							 _AUTOPILOT_VS_ATTRIBUTE,
							 _MEAS_LONGITUDE_ATTRIBUTE, 
							 _MEAS_LATITUDE_ATTRIBUTE, 
							 _MEAS_ALTITUDE_ATTRIBUTE,
							 _MEAS_PHI_ATTRIBUTE, 
							 _MEAS_THETA_ATTRIBUTE, 
							 _MEAS_PSI_ATTRIBUTE,
							 _MEAS_U_SPEED_ATTRIBUTE, 
							 _MEAS_V_SPEED_ATTRIBUTE, 
							 _MEAS_W_SPEED_ATTRIBUTE,
							 _MEAS_P_ANG_SPEED_ATTRIBUTE, 
							 _MEAS_Q_ANG_SPEED_ATTRIBUTE, 
							 _MEAS_R_ANG_SPEED_ATTRIBUTE,
							 _MEAS_X_ACC_ATTRIBUTE, 
							 _MEAS_Y_ACC_ATTRIBUTE, 
							 _MEAS_Z_ACC_ATTRIBUTE,
							 _MEAS_INDICATED_AIRSPEED_ATTRIBUTE, 
							 _MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE, 
							 _MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE,
							 _MEAS_TRUE_AIRSPEED_ATTRIBUTE,
							 _MEAS_GROUND_SPEED_ATTRIBUTE,
							 _MEAS_VERTICAL_SPEED_ATTRIBUTE,
							 _MEAS_MACH_NUMBER_ATTRIBUTE,
							 _MEAS_ALPHA_ATTRIBUTE, 
							 _MEAS_BETA_ATTRIBUTE,
							 _MEAS_DYNAMIC_PRESSURE_ATTRIBUTE,
							 _MEAS_TANK_FILLING_ATTRIBUTE,
							 _RIGHT_ENGINE_THRUST_ATTRIBUTE,
							 _LEFT_ENGINE_THRUST_ATTRIBUTE,
							 _PRESSURE_ATTRIBUTE;

		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_POSITION;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_ORIENTATION;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_UVW_SPEED;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_ACCELERATION;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_SPEED;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_ADDITIONAL;
		std::unique_ptr<RTI::AttributeHandleSet> attr_ACTUATORS_ENGINES;
		std::unique_ptr<RTI::AttributeHandleSet> attr_ENVIRONMENT_VARIABLES;
 
		bool _Discov_MEAS_AIRCRAFT_POSITION,
			 _Discov_MEAS_AIRCRAFT_ORIENTATION,
			 _Discov_MEAS_AIRCRAFT_UVW_SPEED,
			 _Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED,
			 _Discov_MEAS_AIRCRAFT_ACCELERATION,
			 _Discov_MEAS_AIRCRAFT_SPEED,
			 _Discov_MEAS_AIRCRAFT_ADDITIONAL,
			 _Discov_ACTUATORS_ENGINES,
			 _Discov_ENVIRONMENT_VARIABLES;
		bool _New_MEAS_AIRCRAFT_POSITION,
			 _New_MEAS_AIRCRAFT_ORIENTATION,
			 _New_MEAS_AIRCRAFT_UVW_SPEED,
			 _New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED,
			 _New_MEAS_AIRCRAFT_ACCELERATION,
			 _New_MEAS_AIRCRAFT_SPEED,
			 _New_MEAS_AIRCRAFT_ADDITIONAL,
			 _New_ACTUATORS_ENGINES,
			 _New_ENVIRONMENT_VARIABLES;

		// Handles and Flags for published datas
		MessageBuffer _OutputMessagebuffer;
		RTI::ObjectClassHandle _COCKPIT_ClassHandle;
		RTI::ObjectHandle _COCKPIT_ObjectHandle;
		RTI::AttributeHandle _RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE, 
							 _LEFT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _RIGHT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _LEFT_ELEVATOR_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _RUDDER_COMMANDED_DEFLECTION_ATTRIBUTE,		
							 _RIGHT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE,
							 _LEFT_ENGINE_COMMANDED_THROTTLE_ATTRIBUTE,
							 _FLAPS_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _SPOILERS_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _STABILIZER_COMMANDED_DEFLECTION_ATTRIBUTE,
							 _GEARS_COMMANDED_POSITION_ATTRIBUTE;
		std::unique_ptr<RTI::AttributeHandleSet> attr_COCKPIT;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_COCKPIT;
				
	public:
	
		CockpitFederateHla13 ( std::wstring FederationName
						  , std::wstring FederateName
					      , std::wstring FomFileName
					      );

		virtual ~CockpitFederateHla13() throw(RTI::FederateInternalError);
		
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
        void sendInitialAllAttributes();
        void sendUpdateAttributes(const RTI::FedTime& UpdateTime);
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

#endif //__COCKPIT_FEDERATE_HLA13_HH_DEF__
