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

#ifndef __EFCS_FEDERATE_HLA1516E_HH_DEF__
#define __EFCS_FEDERATE_HLA1516E_HH_DEF__

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
#include "Efcs.hh"
#include "Common.hh" 
#include "DataStructures.h"
#include "SendingSocketUDP.hh"
#include "ReceivingSocketUDP.hh"

class EfcsFederateHla1516e : public rti1516e::NullFederateAmbassador, public UnitConversion 
{
	// Private  elements
	private:
	
		// Internal model
		Efcs _EFCS;
		
		// Flag for external EFCS
		bool _IsExternalBridgeEnabled;
		SendingSocketUDP _OuputLinkToExternalEfcs;
		SendingSocketUDP _OuputLinkToExternalEfcsTrimming;
		ReceivingSocketUDP _IncomingLinkFromExternalEfcs;	
		// Data structures
		bridge_data_t _BridgeData;
		efcs_data_t _EfcsOutput;
		trimming_values_t _TrimmingData;
		// Message Buffers
		MessageBuffer _InBuffer;
		MessageBuffer _OutBuffer;
		// Ip address and ports
		std::string _BridgeIpAddress, _ExternalEfcsIpAdress;
		int _BridgeUdpPort, _BridgeTrimmingUdpPort, _ExternalEfcsUdpPort;
		// Additional
		int _RemotePort;
	
		// HLA things
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
		rti1516e::ObjectClassHandle _TRIM_DATA_ClassHandle; 
		rti1516e::ObjectClassHandle _JOYSTICK_ClassHandle;
		rti1516e::ObjectClassHandle _COCKPIT_ClassHandle;
		rti1516e::ObjectClassHandle _MEAS_AIRCRAFT_POSITION_ClassHandle; 
		rti1516e::ObjectClassHandle _MEAS_AIRCRAFT_ORIENTATION_ClassHandle; 
		rti1516e::ObjectClassHandle _MEAS_AIRCRAFT_UVW_SPEED_ClassHandle; 
		rti1516e::ObjectClassHandle _MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ClassHandle;
		rti1516e::ObjectClassHandle _MEAS_AIRCRAFT_ACCELERATION_ClassHandle;
		rti1516e::ObjectClassHandle _MEAS_AIRCRAFT_SPEED_ClassHandle;
		rti1516e::ObjectClassHandle _MEAS_AIRCRAFT_ADDITIONAL_ClassHandle;
		rti1516e::ObjectInstanceHandle _ObjInstance_TRIM_DATA_ENGINES_ObjectHandle;
		rti1516e::ObjectInstanceHandle _ObjInstance_TRIM_DATA_FLIGHT_DYNAMICS_ObjectHandle;    
		rti1516e::ObjectInstanceHandle _ObjInstance_JOYSTICK_ObjectHandle;
		rti1516e::ObjectInstanceHandle _ObjInstance_COCKPIT_ObjectHandle;
		rti1516e::ObjectInstanceHandle _ObjInstance_MEAS_AIRCRAFT_POSITION_ObjectHandle;
		rti1516e::ObjectInstanceHandle _ObjInstance_MEAS_AIRCRAFT_ORIENTATION_ObjectHandle;
		rti1516e::ObjectInstanceHandle _ObjInstance_MEAS_AIRCRAFT_UVW_SPEED_ObjectHandle;
		rti1516e::ObjectInstanceHandle _ObjInstance_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED_ObjectHandle;
		rti1516e::ObjectInstanceHandle _ObjInstance_MEAS_AIRCRAFT_ACCELERATION_ObjectHandle;
		rti1516e::ObjectInstanceHandle _ObjInstance_MEAS_AIRCRAFT_SPEED_ObjectHandle;
		rti1516e::ObjectInstanceHandle _ObjInstance_MEAS_AIRCRAFT_ADDITIONAL_ObjectHandle;
		rti1516e::AttributeHandle _RIGHT_ENGINE_THROTTLE_EQ_ATTRIBUTE, 
							 _LEFT_ENGINE_THROTTLE_EQ_ATTRIBUTE, 
							 _ELEVATOR_DEFLECTION_EQ_ATTRIBUTE,
							 _STABILIZER_DEFLECTION_EQ_ATTRIBUTE;                      
		rti1516e::AttributeHandle _AILERON_ATTRIBUTE, 
							 _ELEVATOR_ATTRIBUTE, 
							 _RUDDER_ATTRIBUTE, 
							 _THROTTLE_LEFT_ATTRIBUTE, 
							 _THROTTLE_RIGHT_ATTRIBUTE,
							 _FLAPS_ATTRIBUTE,
							 _SPOILERS_ATTRIBUTE,
							 _GEARS_ATTRIBUTE,
							 _BRAKES_ATTRIBUTE;                    
		rti1516e::AttributeHandle _AUTOPILOT_AP_ACTIVE_ATTRIBUTE, 
							 _AUTOPILOT_ATHR_ACTIVE_ATTRIBUTE, 
							 _AUTOPILOT_SPD_ATTRIBUTE, 
							 _AUTOPILOT_HDG_ATTRIBUTE, 
							 _AUTOPILOT_ALT_ATTRIBUTE, 
							 _AUTOPILOT_VS_ATTRIBUTE;
		rti1516e::AttributeHandle _MEAS_LONGITUDE_ATTRIBUTE, 
							 _MEAS_LATITUDE_ATTRIBUTE, 
							 _MEAS_ALTITUDE_ATTRIBUTE;
		rti1516e::AttributeHandle _MEAS_PHI_ATTRIBUTE, 
							 _MEAS_THETA_ATTRIBUTE, 
							 _MEAS_PSI_ATTRIBUTE; 
		rti1516e::AttributeHandle _MEAS_U_SPEED_ATTRIBUTE, 
							 _MEAS_V_SPEED_ATTRIBUTE, 
							 _MEAS_W_SPEED_ATTRIBUTE;
		rti1516e::AttributeHandle _MEAS_P_ANG_SPEED_ATTRIBUTE, 
							 _MEAS_Q_ANG_SPEED_ATTRIBUTE, 
							 _MEAS_R_ANG_SPEED_ATTRIBUTE; 
		rti1516e::AttributeHandle _MEAS_X_ACC_ATTRIBUTE, 
							 _MEAS_Y_ACC_ATTRIBUTE, 
							 _MEAS_Z_ACC_ATTRIBUTE; 
		rti1516e::AttributeHandle _MEAS_INDICATED_AIRSPEED_ATTRIBUTE, 
							 _MEAS_EQUIVALENT_AIRSPEED_ATTRIBUTE, 
							 _MEAS_CALIBRATED_AIRSPEED_ATTRIBUTE,
							 _MEAS_TRUE_AIRSPEED_ATTRIBUTE,
							 _MEAS_GROUND_SPEED_ATTRIBUTE,
							 _MEAS_VERTICAL_SPEED_ATTRIBUTE,
							 _MEAS_MACH_NUMBER_ATTRIBUTE; 
		rti1516e::AttributeHandle _MEAS_ALPHA_ATTRIBUTE, 
							 _MEAS_BETA_ATTRIBUTE,
							 _MEAS_DYNAMIC_PRESSURE_ATTRIBUTE,
							 _MEAS_TANK_FILLING_ATTRIBUTE;
		rti1516e::AttributeHandleSet attr_TRIM_DATA; 
		rti1516e::AttributeHandleSet attr_JOYSTICK;
		rti1516e::AttributeHandleSet attr_COCKPIT;
		rti1516e::AttributeHandleSet attr_MEAS_AIRCRAFT_POSITION;
		rti1516e::AttributeHandleSet attr_MEAS_AIRCRAFT_ORIENTATION;
		rti1516e::AttributeHandleSet attr_MEAS_AIRCRAFT_UVW_SPEED;
		rti1516e::AttributeHandleSet attr_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED;
		rti1516e::AttributeHandleSet attr_MEAS_AIRCRAFT_ACCELERATION;
		rti1516e::AttributeHandleSet attr_MEAS_AIRCRAFT_SPEED;
		rti1516e::AttributeHandleSet attr_MEAS_AIRCRAFT_ADDITIONAL;

		bool _Discov_TRIM_DATA_ENGINES,
			 _Discov_TRIM_DATA_FLIGHT_DYNAMICS;
		bool _New_TRIM_DATA_ENGINES,
			 _New_TRIM_DATA_FLIGHT_DYNAMICS;  
		bool _Discov_JOYSTICK;
		bool _New_JOYSTICK;   
		bool _Discov_COCKPIT;
		bool _New_COCKPIT;   
		bool _Discov_MEAS_AIRCRAFT_POSITION,
			 _Discov_MEAS_AIRCRAFT_ORIENTATION,
			 _Discov_MEAS_AIRCRAFT_UVW_SPEED,
			 _Discov_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED,
			 _Discov_MEAS_AIRCRAFT_ACCELERATION,
			 _Discov_MEAS_AIRCRAFT_SPEED,
			 _Discov_MEAS_AIRCRAFT_ADDITIONAL;
		bool _New_MEAS_AIRCRAFT_POSITION,
			 _New_MEAS_AIRCRAFT_ORIENTATION,
			 _New_MEAS_AIRCRAFT_UVW_SPEED,
			 _New_MEAS_AIRCRAFT_PQR_ANGULAR_SPEED,
			 _New_MEAS_AIRCRAFT_ACCELERATION,
			 _New_MEAS_AIRCRAFT_SPEED,
			 _New_MEAS_AIRCRAFT_ADDITIONAL;

		// Handles and Flags for published datas
		MessageBuffer _OutputMessagebuffer;
		rti1516e::ObjectClassHandle _FLIGHT_CONTROLS_ClassHandle; 
		rti1516e::ObjectInstanceHandle _FLIGHT_CONTROLS_ObjectHandle;
		rti1516e::AttributeHandle _RIGHT_AILERON_COMMANDED_DEFLECTION_ATTRIBUTE, 
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
		rti1516e::AttributeHandleSet attr_FLIGHT_CONTROLS; 
		//std::unique_ptr<rti1516e::AttributeHandleValuePairSet> ahvps_FLIGHT_CONTROLS;
				
    // Public elements
	public:
	
		EfcsFederateHla1516e ( std::wstring FederationName
						  , std::wstring FederateName
					      , std::wstring FomFileName
					      );

		virtual ~EfcsFederateHla1516e();
		
		void handleExternalBridgeSetup();
		
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
