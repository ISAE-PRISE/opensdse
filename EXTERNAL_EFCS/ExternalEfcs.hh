#ifndef __EXTERNAL_EFCS_HH__
#define __EXTERNAL_EFCS_HH__

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <fstream>
#include <stdint.h>
#include <memory>
#include <time.h>

#include <stdio.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <errno.h>

#include "SendingSocketUDP.hh"
#include "ReceivingSocketUDP.hh"
#include "MessageBuffer.hh"
#include "ControllerPID.hh"
#include "UnitConversion.hh"
#include "DataStructures.h"
#include "Efcs.hh"
#include "Common.hh"

// For debug purposes
#define DEBUG_EXTERNAL_EFCS 1

class ExternalEfcs : public UnitConversion
{
	private:
	
		// Internal model
		Efcs _EFCS;
		int	_Id; // Should be 1, 2, 3 or 4
		
		// Data structures
		bridge_data_t _BridgeData;
		efcs_data_t _EfcsOutput;
		trimming_values_t _TrimmingData;
			
		// Sockets
		SendingSocketUDP _OuputLinkToBridge;
		ReceivingSocketUDP _IncomingLinkFromBridge;	
		
		// Ip address and ports
		std::string _BridgeIpAddress, _ExternalEfcsIpAdress;
		int _BridgeUdpPort, _BridgeTrimmingUdpPort, _ExternalEfcsUdpPort;
		
		// Message Buffers
		MessageBuffer _InBuffer;
		MessageBuffer _OutBuffer;
		
		// Additional
		int _RemotePort;
		bool _IsTrimmingDone;

	public:
	
		ExternalEfcs();
		~ExternalEfcs();
		void setId(int Id);
		int getId();
		void constructSockets();
		void compute();			
};


#endif // __EXTERNAL_EFCS_HH__
