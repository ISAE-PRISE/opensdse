#ifndef __SENDING_SOCKET_UDP_HH__
#define __SENDING_SOCKET_UDP_HH__

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>

class SendingSocketUDP
{
	private:
		int                 _Socket;
		
		int                 _SrcPort;
		std::string         _SrcAddr;
		struct sockaddr_in  _SrcAddrInfo;
		// TO DO: How to handle this?
		socklen_t			_SrcAddrLen;
		
		int                 _DstPort;
		std::string         _DstAddr;
		struct sockaddr_in  _DstAddrInfo;
		
		int					_ReuseEnabled;
		
	public:
		SendingSocketUDP();
		~SendingSocketUDP();
		
		int constructSocket( const std::string& SrcAddr
		                   , int SrcPort
		                   , const std::string& DstAddr
		                   , int DstPort
		                   );
		int getSocket() const;
		int getSrcPort() const;
		std::string getSrcAddr() const;
		int getDstPort() const;
		std::string getDstAddr() const;
		int setTimeOut(int timeout_ms);
		int send(const char *msg, size_t size);
};


#endif
