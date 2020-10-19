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

#ifndef __SENSORS_FEDERATE_HLA13_HH_DEF__
#define __SENSORS_FEDERATE_HLA13_HH_DEF__

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
#include "Sensor.hh"
#include "Common.hh" 

class SensorsFederateHla13 : public NullFederateAmbassador 
{
	private:
	
		// Internal models
		Sensor _Sensor_LONGITUDE, 
			   _Sensor_LATITUDE, 
			   _Sensor_ALTITUDE,
			   _Sensor_PHI, 
			   _Sensor_THETA, 
			   _Sensor_PSI,
			   _Sensor_U_SPEED,
			   _Sensor_V_SPEED,
			   _Sensor_W_SPEED,
			   _Sensor_P_ANG_SPEED,
			   _Sensor_Q_ANG_SPEED,
			   _Sensor_R_ANG_SPEED,
			   _Sensor_X_ACC,
			   _Sensor_Y_ACC,
			   _Sensor_Z_ACC,
			   _Sensor_INDICATED_AIRSPEED, 
			   _Sensor_EQUIVALENT_AIRSPEED, 
			   _Sensor_CALIBRATED_AIRSPEED,
			   _Sensor_TRUE_AIRSPEED,
			   _Sensor_GROUND_SPEED,
			   _Sensor_VERTICAL_SPEED,
			   _Sensor_MACH_NUMBER,
			   _Sensor_ALPHA,
			   _Sensor_BETA,
			   _Sensor_DYNAMIC_PRESSURE,
			   _Sensor_TANK_FILLING;
	
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
		RTI::ObjectClassHandle _AIRCRAFT_POSITION_ClassHandle; 
		RTI::ObjectClassHandle _AIRCRAFT_ORIENTATION_ClassHandle; 
		RTI::ObjectClassHandle _AIRCRAFT_UVW_SPEED_ClassHandle; 
		RTI::ObjectClassHandle _AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle;
		RTI::ObjectClassHandle _AIRCRAFT_ACCELERATION_ClassHandle;
		RTI::ObjectClassHandle _AIRCRAFT_SPEED_ClassHandle;
		RTI::ObjectClassHandle _AIRCRAFT_ADDITIONAL_ClassHandle;
		RTI::ObjectClassHandle _USER_DATA_ClassHandle;
		RTI::ObjectHandle _AIRCRAFT_POSITION_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_ORIENTATION_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_UVW_SPEED_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_ACCELERATION_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_SPEED_ObjectHandle;
		RTI::ObjectHandle _AIRCRAFT_ADDITIONAL_ObjectHandle;
		RTI::AttributeHandle _LONGITUDE_ATTRIBUTE, 
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
		bool _Discov_AIRCRAFT_POSITION,
			 _Discov_AIRCRAFT_ORIENTATION,
			 _Discov_AIRCRAFT_UVW_SPEED,
			 _Discov_AIRCRAFT_PQR_ANGULAR_SPEED,
			 _Discov_AIRCRAFT_ACCELERATION,
			 _Discov_AIRCRAFT_SPEED,
			 _Discov_AIRCRAFT_ADDITIONAL;
		bool _New_AIRCRAFT_POSITION,
			 _New_AIRCRAFT_ORIENTATION,
			 _New_AIRCRAFT_UVW_SPEED,
			 _New_AIRCRAFT_PQR_ANGULAR_SPEED,
			 _New_AIRCRAFT_ACCELERATION,
			 _New_AIRCRAFT_SPEED,
			 _New_AIRCRAFT_ADDITIONAL;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_POSITION;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_ORIENTATION;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_UVW_SPEED;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_PQR_ANGULAR_SPEED;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_ACCELERATION;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_SPEED;
		std::unique_ptr<RTI::AttributeHandleSet> attr_AIRCRAFT_ADDITIONAL;
		
		// Handles and Flags for published datas
		MessageBuffer _OutputMessagebuffer;
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_POSITION_ClassHandle; 
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_ORIENTATION_ClassHandle; 
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_UVW_SPEED_ClassHandle; 
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle;
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_ACCELERATION_ClassHandle;
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_SPEED_ClassHandle;
		RTI::ObjectClassHandle _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_POSITION_ObjectHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_ORIENTATION_ObjectHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_ACCELERATION_ObjectHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_SPEED_ObjectHandle;
		RTI::ObjectHandle _MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle;
		RTI::AttributeHandle _MEAS_LONGITUDE_ATTRIBUTE, 
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
							 _MEAS_TANK_FILLING_ATTRIBUTE;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_POSITION;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_ORIENTATION;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_UVW_SPEED;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_ACCELERATION;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_SPEED;
		std::unique_ptr<RTI::AttributeHandleSet> attr_MEAS_AIRCRAFT_ADDITIONAL;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_MEAS_AIRCRAFT_POSITION;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_MEAS_AIRCRAFT_ORIENTATION;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_MEAS_AIRCRAFT_UVW_SPEED;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_MEAS_AIRCRAFT_ACCELERATION;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_MEAS_AIRCRAFT_SPEED;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> ahvps_MEAS_AIRCRAFT_ADDITIONAL;
				
	public:
	
		SensorsFederateHla13 ( std::wstring FederationName
						  , std::wstring FederateName
					      , std::wstring FomFileName
					      );

		virtual ~SensorsFederateHla13() throw(RTI::FederateInternalError);
		
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
        void sendInitialAllAttributes();
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

#endif //__SENSORS_FEDERATE_HLA13_HH_DEF__
