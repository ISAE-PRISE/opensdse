#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>

#include "ExternalEfcs.hh"

using namespace std;

// ----------------------------------------------------------------------------
// Constructor 
ExternalEfcs::ExternalEfcs() 
{
	// init ID to 0 as non defined in default constructor
	_Id = 0;
	
	// Set data structures to 0
	memset(&_BridgeData, 0, sizeof(bridge_data_t));
	memset(&_EfcsOutput, 0, sizeof(efcs_data_t));
	memset(&_TrimmingData, 0, sizeof(trimming_values_t));
	
	_IsTrimmingDone = false;
	_RemotePort = 0;
	
	// Get values from xml file
	XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);
	// Get values from xml file
	vector<string> attribute_path(3);
	attribute_path[0] = "EFCS";
	attribute_path[1] = "Bridge_Net_Config";
	attribute_path[2] = "UDP_port";
	_BridgeUdpPort = getIntValue(params, attribute_path);
	attribute_path[2] = "Trimming_UDP_port";
	_BridgeTrimmingUdpPort = getIntValue(params, attribute_path);
	attribute_path[2] = "IP_Address";
	_BridgeIpAddress = getTextValue(params, attribute_path);
	attribute_path[1] = "Efcs_Net_Config";
	attribute_path[2] = "UDP_port";
	_ExternalEfcsUdpPort = getIntValue(params, attribute_path);
	attribute_path[2] = "IP_Address";
	_ExternalEfcsIpAdress = getTextValue(params, attribute_path);
	delete params;
	
}

// ----------------------------------------------------------------------------
// Destructor 
ExternalEfcs::~ExternalEfcs()
{
	// Nothing to do
}

// ----------------------------------------------------------------------------
// Set Id
void ExternalEfcs::setId(int Id)
{
	_Id = Id;
}

// ----------------------------------------------------------------------------
// Get Id
int ExternalEfcs::getId()
{
	return _Id;
}

// ----------------------------------------------------------------------------
// Construct sockets
void ExternalEfcs::constructSockets()
{
	//set max size to max UDP frame PAYLOAD frame
	_InBuffer.resize(1472);
	_OutBuffer.resize(1472);
	
	// Output sockets
	_OuputLinkToBridge.constructSocket(_ExternalEfcsIpAdress, _ExternalEfcsUdpPort, _BridgeIpAddress, _BridgeUdpPort);

	// Input socket
	_IncomingLinkFromBridge.constructSocket(_ExternalEfcsIpAdress, _ExternalEfcsUdpPort);
}

// ----------------------------------------------------------------------------
// receive Thread
void ExternalEfcs::compute()
{
	fd_set fds;
	int maxfdp, ret;
	// set incoming link as non-blocking
    //_IncomingLink.setTimeOut(1);
    
	FD_ZERO(&fds);
	FD_SET(_IncomingLinkFromBridge.getSocket(), &fds);

	maxfdp = _IncomingLinkFromBridge.getSocket() + 1;
         
	 while(1)
	 {
			switch(select(maxfdp, &fds, NULL, NULL, NULL)) 
			{
			case -1: 
				perror("error");
				exit(-1);	
				break;

			case 0:
				printf("timeout.\n");
				break;

			default:
				if (FD_ISSET(_IncomingLinkFromBridge.getSocket(), &fds)) 
				{
					_InBuffer.reset();        
					ret = _IncomingLinkFromBridge.receive(static_cast<char*>(_InBuffer(0)), _InBuffer.maxSize());
					if (ret > 0)
					{
						_InBuffer.assumeSizeFromReservedBytes();
						_RemotePort =  _IncomingLinkFromBridge.getLastRegisteredRemoteUdpPort();
						if ( (_RemotePort == _BridgeTrimmingUdpPort) && (!_IsTrimmingDone))
						{
							_InBuffer.assumeSizeFromReservedBytes();
							_InBuffer.read_bytes((char*)&_TrimmingData, sizeof(trimming_values_t)) ;
							#ifdef DEBUG_EXTERNAL_EFCS
							std::cout << "------------------------------"<< std::endl;
							std::cout << "Trimming done for External EFCS " << _Id << " ! "<< std::endl;
							std::cout << "------------------------------"<< std::endl;
							std::cout << "elevator = "<< _TrimmingData.elevator << std::endl;
							std::cout << "stabilizer = "<< _TrimmingData.stabilizer << std::endl;
							std::cout << "rudder = "<< _TrimmingData.rudder << std::endl;
							std::cout << "left_engine_throttle = "<< _TrimmingData.left_engine_throttle << std::endl;
							std::cout << "right_engine_throttle = "<< _TrimmingData.right_engine_throttle << std::endl;
							#endif
							_EFCS.setTrimThrottle(_TrimmingData.left_engine_throttle); 
							_EFCS.setTrimElevator(_TrimmingData.elevator); 
							_EFCS.setTrimStabilizer(_TrimmingData.stabilizer); 
							_IsTrimmingDone = true;
						}
						else if ( (_RemotePort == _BridgeUdpPort) && (_IsTrimmingDone))
						{
							_InBuffer.read_bytes((char*)&_BridgeData, sizeof(bridge_data_t));
							// Joystick
							_EFCS.setAileron_Joystick(_BridgeData.joystick_output.ss_left_aileron);
							_EFCS.setElevator_Joystick(_BridgeData.joystick_output.ss_left_elevator);
							_EFCS.setRudder_Joystick(_BridgeData.joystick_output.ss_rudder);
							_EFCS.setThrottle_Left_Joystick(_BridgeData.joystick_output.ss_left_engine_throttle); 
							_EFCS.setThrottle_Right_Joystick(_BridgeData.joystick_output.ss_right_engine_throttle); 
							_EFCS.setFlaps_Joystick(_BridgeData.joystick_output.ss_left_flap);
							_EFCS.setSpoilers_Joystick(_BridgeData.joystick_output.ss_spoilers);
							_EFCS.setGears_Joystick(_BridgeData.joystick_output.ss_gears);
							_EFCS.setBrakes_Joystick(_BridgeData.joystick_output.ss_speedbrake);
							
							// Fcu
							_EFCS.setAutopilot_AP_Active(_BridgeData.fcu_output.ap1_active);
							_EFCS.setAutopilot_athr_Active(_BridgeData.fcu_output.athr_active);
							_EFCS.setAutopilot_spd(_BridgeData.fcu_output.speed_std);
							_EFCS.setAutopilot_hdg(_BridgeData.fcu_output.heading);
							_EFCS.setAutopilot_alt(_BridgeData.fcu_output.altitude);
							_EFCS.setAutopilot_vs(_BridgeData.fcu_output.vs_speed);
							
							// Flight Dynamics
							_EFCS.setLongitude(_BridgeData.flight_dynamics_output.longitude); 
							_EFCS.setLatitude(_BridgeData.flight_dynamics_output.latitude); 
							_EFCS.setAltitude(_BridgeData.flight_dynamics_output.altitude_sl); 
							_EFCS.setPhi(_BridgeData.flight_dynamics_output.phi); 
							_EFCS.setTheta(_BridgeData.flight_dynamics_output.theta); 
							_EFCS.setPsi(_BridgeData.flight_dynamics_output.psi); 
							_EFCS.setU(_BridgeData.flight_dynamics_output.v_body_u); 
							_EFCS.setV(_BridgeData.flight_dynamics_output.v_body_v); 
							_EFCS.setW(_BridgeData.flight_dynamics_output.v_body_w); 
							_EFCS.setP(_BridgeData.flight_dynamics_output.phidot); 
							_EFCS.setQ(_BridgeData.flight_dynamics_output.thetadot); 
							_EFCS.setR(_BridgeData.flight_dynamics_output.psidot); 
							_EFCS.setAx(_BridgeData.flight_dynamics_output.A_X_pilot);
							_EFCS.setAy(_BridgeData.flight_dynamics_output.A_Y_pilot);
							_EFCS.setAz(_BridgeData.flight_dynamics_output.A_Z_pilot); 
							_EFCS.setIAS(_BridgeData.flight_dynamics_output.vias);
							_EFCS.setEAS(_BridgeData.flight_dynamics_output.veas); 
							_EFCS.setCAS(_BridgeData.flight_dynamics_output.vcas); 
							_EFCS.setTAS(_BridgeData.flight_dynamics_output.vtas); 
							_EFCS.setGS(_BridgeData.flight_dynamics_output.ground_speed); 
							_EFCS.setVS(_BridgeData.flight_dynamics_output.climb_rate);
							_EFCS.setMach(_BridgeData.flight_dynamics_output.vmach); 
							_EFCS.setAlpha(_BridgeData.flight_dynamics_output.alpha); 
							_EFCS.setBeta(_BridgeData.flight_dynamics_output.beta); 
							_EFCS.setQbar(_BridgeData.flight_dynamics_output.dyn_pressure); 
							// Not used
							//_EFCS.setTankFilling(buffer.read_double());
							
							// Compute law
							_EFCS.calculate_state();  
							_EFCS.calculate_output();
							
							_EfcsOutput.cmd_right_aileron = _EFCS.getRightAileron();
							_EfcsOutput.cmd_left_aileron = _EFCS.getLeftAileron();
							_EfcsOutput.cmd_right_elevator = _EFCS.getRightElevator();
							_EfcsOutput.cmd_left_elevator = _EFCS.getLeftElevator();
							_EfcsOutput.cmd_rudder = _EFCS.getRudder();
							_EfcsOutput.cmd_right_engine_throttle = _EFCS.getRightEngine();
							_EfcsOutput.cmd_left_engine_throttle = _EFCS.getLeftEngine();
							_EfcsOutput.cmd_left_flap = _EFCS.getFlaps();
							_EfcsOutput.cmd_right_flap = _EFCS.getFlaps();
							_EfcsOutput.cmd_spoilers = _EFCS.getSpoilers();
							_EfcsOutput.cmd_stabilizer = _EFCS.getStabilizer();
							_EfcsOutput.cmd_gears = _EFCS.getGears();
							
							// Send
							_OutBuffer.reset();
							_OutBuffer.write_bytes((char*)&_EfcsOutput, sizeof(efcs_data_t)) ;
							_OutBuffer.updateReservedBytes();
							_OuputLinkToBridge.send(static_cast<char*>(_OutBuffer(0)), _OutBuffer.size());
								
						}
					}
				}
				break;
		}
	}  
}

