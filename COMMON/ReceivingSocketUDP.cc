#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include "ReceivingSocketUDP.hh"

// ----------------------------------------------------------------------------
// Constructor
ReceivingSocketUDP::ReceivingSocketUDP()
{
	// Init all parameters to 0
	_Socket = 0;
	_LocalPort = 0;
	_LocalAddrLen = 0;
    memset(&_LocalAddrInfo, 0, sizeof(_LocalAddrInfo));
    memset(&_RemoteAddrInfo, 0, sizeof(_RemoteAddrInfo));
}

// ----------------------------------------------------------------------------
// Destructor
ReceivingSocketUDP::~ReceivingSocketUDP()
{
    close(_Socket);
}

// ----------------------------------------------------------------------------
// Constructor
int ReceivingSocketUDP::constructSocket( const std::string& LocalAddr
                              , int LocalPort
                              )
{
	int ret;
	
	_LocalPort = LocalPort;
	_LocalAddr = LocalAddr;
	_ReuseEnabled = 0;
	
    memset(&_LocalAddrInfo, 0, sizeof(_LocalAddrInfo));
	_LocalAddrInfo.sin_family = AF_INET;
	_LocalAddrInfo.sin_port = htons(_LocalPort);
	_LocalAddrInfo.sin_addr.s_addr = inet_addr((char * )_LocalAddr.c_str());
	_LocalAddrLen = sizeof(_LocalAddrInfo);
	
	_RemoteAddrInfo.sin_addr.s_addr = htonl(INADDR_ANY);
	_RemoteAddrLen = sizeof(_LocalAddrInfo);
	
	// Create socket
	ret = socket(AF_INET,SOCK_DGRAM,0);
	if (ret  < 0)
	{
		std::cerr << "ReceivingSocketUDP.cc: LocalSocket UDP creation error"<< std::endl;
	}
	else
	{
		_Socket = ret;
		_ReuseEnabled = 1;
	}
	
	ret = setsockopt(_Socket, SOL_SOCKET, (SO_REUSEPORT | SO_REUSEADDR), &_ReuseEnabled, sizeof(int));
	if (ret  < 0)
	{
		std::cerr << "ReceivingSocketUDP.cc: LocalSocket setsockopterror"<< std::endl;
	}
	
	// Bind socket to _LocalPort
	ret = bind(_Socket,(struct sockaddr *)&_LocalAddrInfo,sizeof(_LocalAddrInfo));
	if (ret  < 0)
	{
		std::cerr << "ReceivingSocketUDP.cc: LocalSocket UDP bind error on port "
		     << _LocalPort
		     << std::endl;
	}
	return ret;
}

// ----------------------------------------------------------------------------
// 
int ReceivingSocketUDP::getSocket() const
{
    return _Socket;
}

// ----------------------------------------------------------------------------
// 
int ReceivingSocketUDP::getLocalPort() const
{
    return _LocalPort;
}

// ----------------------------------------------------------------------------
//
std::string ReceivingSocketUDP::getLocalAddr() const
{
    return _LocalAddr;
}

// ----------------------------------------------------------------------------
// 
int ReceivingSocketUDP::getLastRegisteredRemoteUdpPort() const
{
    return ntohs(_RemoteAddrInfo.sin_port);
}

// ----------------------------------------------------------------------------
//
int ReceivingSocketUDP::setTimeOut(int timeout_ms) 
{
	// TO DO: Should this value be part of the class parameter?
	int ret;
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = timeout_ms * 1000000;
	int flags = fcntl(_Socket, F_GETFL, 0);
	ret = fcntl(_Socket, F_SETFL, flags | O_NONBLOCK);
	//ret = setsockopt(_Socket, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv));
	if (ret  < 0)
	{
		std::cerr << "ReceivingSocketUDP.cc: setTimeOut error in setsockopt " << std::endl;
	}
	return ret;
}

// ----------------------------------------------------------------------------
//
int ReceivingSocketUDP::receive(char *msg, size_t max_size)
{
	// using recvfrom
    return recvfrom( _Socket
                   , msg
                   , max_size
                   , 0
                   , (struct sockaddr *)&_RemoteAddrInfo
                   , &_RemoteAddrLen
                   );
}
