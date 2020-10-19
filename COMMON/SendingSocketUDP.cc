#include <string.h>
#include <unistd.h>
#include <iostream>
#include "SendingSocketUDP.hh"

// ----------------------------------------------------------------------------
// Constructor
SendingSocketUDP::SendingSocketUDP()
{
	// Init all parameters to 0
	_Socket = 0;
	_SrcPort = 0;
	_DstPort = 0;
	_SrcAddrLen = 0;
	memset(&_DstAddrInfo, 0, sizeof(_DstAddrInfo));
    memset(&_SrcAddrInfo, 0, sizeof(_SrcAddrInfo));
}

// ----------------------------------------------------------------------------
// Destructor
SendingSocketUDP::~SendingSocketUDP()
{
    close(_Socket);
}

// ----------------------------------------------------------------------------
// Constructor
int SendingSocketUDP::constructSocket( const std::string& SrcAddr
                              , int SrcPort
                              , const std::string& DstAddr
                              , int DstPort
                              )
{
	int ret;
	
	_SrcPort = SrcPort;
	_SrcAddr = SrcAddr;
	_DstPort = DstPort;
	_DstAddr = DstAddr;
	_ReuseEnabled = 0;
	
	memset(&_DstAddrInfo, 0, sizeof(_DstAddrInfo));
	_DstAddrInfo.sin_family = AF_INET;
	_DstAddrInfo.sin_port = htons(_DstPort);
	_DstAddrInfo.sin_addr.s_addr = inet_addr(_DstAddr.c_str());
	
    memset(&_SrcAddrInfo, 0, sizeof(_SrcAddrInfo));
	_SrcAddrInfo.sin_family = AF_INET;
	_SrcAddrInfo.sin_port = htons(_SrcPort);
	_SrcAddrInfo.sin_addr.s_addr = htonl(INADDR_ANY);
	_SrcAddrLen = sizeof(_SrcAddrInfo);
	
	// Create socket
	ret = socket(AF_INET,SOCK_DGRAM,0);
	if (ret  < 0)
	{
		std::cerr << "SendingSocketUDP.cc: SrcSocket UDP creation error"<< std::endl;
	}
	else
	{
		_Socket = ret;
		_ReuseEnabled = 1;
	}
	
	ret = setsockopt(_Socket, SOL_SOCKET, (SO_REUSEPORT | SO_REUSEADDR), &_ReuseEnabled, sizeof(int));
	if (ret  < 0)
	{
		std::cerr << "SendingSocketUDP.cc: SrcSocket setsockopterror"<< std::endl;
	}
	
	// Bind socket to _SrcPort
	ret = bind(_Socket,(struct sockaddr *)&_SrcAddrInfo,sizeof(_SrcAddrInfo));
	if (ret  < 0)
	{
		std::cerr << "SendingSocketUDP.cc: SrcSocket UDP bind error on port "
		     << _SrcPort
		     << std::endl;
	}
	return ret;
}

// ----------------------------------------------------------------------------
// 
int SendingSocketUDP::getSocket() const
{
    return _Socket;
}

// ----------------------------------------------------------------------------
// 
int SendingSocketUDP::getSrcPort() const
{
    return _SrcPort;
}

// ----------------------------------------------------------------------------
//
std::string SendingSocketUDP::getSrcAddr() const
{
    return _SrcAddr;
}

// ----------------------------------------------------------------------------
// 
int SendingSocketUDP::getDstPort() const
{
    return _DstPort;
}

// ----------------------------------------------------------------------------
//
std::string SendingSocketUDP::getDstAddr() const
{
    return _DstAddr;
}

// ----------------------------------------------------------------------------
//
int SendingSocketUDP::setTimeOut(int timeout_ms) 
{
	// TO DO: Should this value be part of the class parameter?
	int ret;
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = timeout_ms * 1000000;
	ret = setsockopt(_Socket, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv));
	if (ret  < 0)
	{
		std::cerr << "SendingSocketUDP.cc: setTimeOut error in setsockopt " << std::endl;
	}
	return ret;
}

// ----------------------------------------------------------------------------
//
int SendingSocketUDP::send(const char *msg, size_t size)
{
	// using sento
    return sendto( _Socket
                 , msg
                 , size
                 , 0
                 , (struct sockaddr *)&_DstAddrInfo
                 , sizeof(_DstAddrInfo)
                 );
}
