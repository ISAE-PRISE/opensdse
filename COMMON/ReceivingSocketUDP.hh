#ifndef __RECEIVING_SOCKET_UDP_HH__
#define __RECEIVING_SOCKET_UDP_HH__

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>

class ReceivingSocketUDP
{
	private:
		int                 _Socket;
		
		int                 _LocalPort;
		std::string         _LocalAddr;
		struct sockaddr_in  _LocalAddrInfo;
		socklen_t			_LocalAddrLen;
		struct sockaddr_in  _RemoteAddrInfo;
		socklen_t			_RemoteAddrLen;
		int					_ReuseEnabled;
		
	public:
		ReceivingSocketUDP();
		~ReceivingSocketUDP();
		
		int constructSocket( const std::string& LocalAddr
		                   , int LocalPort
		                   );
		                   
		int getSocket() const;
		int getLocalPort() const;
		std::string getLocalAddr() const;
		int getLastRegisteredRemoteUdpPort() const;
		int setTimeOut(int timeout_ms);
		int receive(char *msg, size_t max_size);
};


#endif
