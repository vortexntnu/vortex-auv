#define _CRT_SECURE_NO_WARNINGS
#define NOMINMAX

#include "Network.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>

#ifdef _WIN32
#include <iphlpapi.h>
#include <Ws2tcpip.h>
#else
#include <arpa/inet.h>                  // for inet_addr
#include <netinet/in.h>                 // for sockaddr_in, ntohl, in_addr, etc
#include <sys/socket.h>                 // for getsockname, send, AF_INET, etc
#include <unistd.h>                     // for close, read, fork, etc
#include <sys/types.h>     /*  Solaris 2.5.1 fix: u_short, required by sys/socket.h */
#include <sys/socket.h>    /*  sockets */
#include <sys/time.h>      /*  timeval */
#include <sys/ioctl.h>     /*  ioctl  */
#include <string.h>        /*  bzero, for FD_SET */
#include <strings.h>       /*  bzero, for FD_ZERO (AIX) */
#include <netinet/in.h>    /*  INADDR_*, in_addr, sockaddr_in, htonl etc. */
#include <netinet/tcp.h>                // for TCP_NODELAY
#include <netdb.h>         /*  getservbyname */
#include <arpa/inet.h>     /*  inet_addr */
#include <errno.h>         /*  socket error codes */
#include <ifaddrs.h>


#define SOCKET_ERROR            (-1)

#define TIMEVAL timeval
#define closesocket close
#define ioctlsocket ioctl
#define SOCKADDR sockaddr
//#define SD_RECEIVE SHUT_RD
#define SD_SEND SHUT_WR
//#define SD_BOTH SHUT_RDWR

#endif

CNetwork::CNetwork()
{
    mSocket             = INVALID_SOCKET;
    mUDPSocket          = INVALID_SOCKET;
    mUDPBroadcastSocket = INVALID_SOCKET;
    mLastError          = 0;
    mErrorStr[0]        = 0;

    InitWinsock();
}


CNetwork::~CNetwork()
{
#ifdef _WIN32
    WSACleanup();
#endif
}


bool CNetwork::InitWinsock()
{
#ifdef _WIN32
    WORD wVersionRequested = MAKEWORD(2,2);
    WSADATA wsaData;

    // Initialize WinSock and check version
    if (WSAStartup(wVersionRequested, &wsaData) != 0)
    {
        SetErrorString();
        return false;
    }
    if (wsaData.wVersion != wVersionRequested)
    {	
        SetErrorString();
        return false;
    }
#endif
    return true;
} // InitWinsock


bool CNetwork::Connect(const char* serverAddr, unsigned short nPort)
{
    mLastError   = 0;
    mErrorStr[0] = 0;

    // Connect to QTM RT server.

    mSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (mSocket == -1)
    {
        strcpy(mErrorStr, "Socket could not be created.");
    }

    sockaddr_in sAddr;

    // First check if the address is a dotted number "A.B.C.D"
    if (inet_pton(AF_INET, serverAddr, &(sAddr.sin_addr)) == 0)
    {
        // If it wasn't a dotted number lookup the server name

        struct addrinfo hints, *servinfo;

        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;

        if (getaddrinfo(serverAddr, nullptr, &hints, &servinfo) != 0)
        {
            strcpy(mErrorStr, "Error looking up host name.");
            closesocket(mSocket);
            return false;
        }
        if (servinfo == nullptr)
        {
			strcpy(mErrorStr, "Error looking up host name.");
            closesocket(mSocket);
            return false;
        }
        sAddr.sin_addr = ((sockaddr_in *)servinfo[0].ai_addr)->sin_addr;
    }
    sAddr.sin_port = htons(nPort);
    sAddr.sin_family = AF_INET;


    if (connect(mSocket, (sockaddr*)(&sAddr), sizeof(sAddr)) == SOCKET_ERROR)
    {
        strcpy(mErrorStr, "Connect failed.");

        //SetErrorString();
        closesocket(mSocket);
        return false;
    }

    // Disable Nagle's algorithm
#ifdef _WIN32
    char bNoDelay = 1;
#else
    int bNoDelay = 1;
#endif
    if (setsockopt(mSocket, IPPROTO_TCP, TCP_NODELAY, &bNoDelay, sizeof(bNoDelay)) != 0)
    {
        SetErrorString();
        closesocket(mSocket);
        return false;
    }

    return true;
} // Connect


void CNetwork::Disconnect()
{
    // Try to shutdown gracefully

    shutdown(mSocket, SD_SEND);
    closesocket(mSocket);
    closesocket(mUDPSocket);
    closesocket(mUDPBroadcastSocket);
    mSocket             = INVALID_SOCKET;
    mUDPSocket          = INVALID_SOCKET;
    mUDPBroadcastSocket = INVALID_SOCKET;
} // Disconnect


bool CNetwork::Connected() const
{
    return mSocket != INVALID_SOCKET;
}

bool CNetwork::CreateUDPSocket(unsigned short &nUDPPort, bool bBroadcast)
{
    if (nUDPPort == 0 || nUDPPort > 1023)
    {
        SOCKET tempSocket = INVALID_SOCKET;

        // Create UDP socket for data streaming
        sockaddr_in recvAddr;
        recvAddr.sin_family = AF_INET;
        recvAddr.sin_port = htons(nUDPPort);
        recvAddr.sin_addr.s_addr = INADDR_ANY;

        tempSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (tempSocket != INVALID_SOCKET)
        {
            u_long argp = 1;
            // Make socket unblocking.
            if (ioctlsocket(tempSocket, FIONBIO, &argp) == 0)
            {
                if (bind(tempSocket, (SOCKADDR *) &recvAddr, sizeof(recvAddr)) != -1)
                {
                    nUDPPort = GetUdpServerPort(tempSocket);
                    if (bBroadcast)
                    {
#ifdef _WIN32
                        char broadcast = 1;
#else
                        int broadcast = 1;
#endif
                        if (setsockopt(tempSocket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == 0)
                        {
                            mUDPBroadcastSocket = tempSocket;
                            return true;
                        }
                        else
                        {
							strcpy(mErrorStr, "Failed to set socket options for UDP server socket.");
                        }
                    }
                    else
                    {
                        mUDPSocket = tempSocket;
                        return true;
                    }
                }
                else
                {
					strcpy(mErrorStr, "Failed to bind UDP server socket.");
                }
            }
            else
            {
				strcpy(mErrorStr, "Failed to make UDP server socket unblocking.");
            }
        }
        else
        {
			strcpy(mErrorStr, "Failed to create UDP server socket.");
        }
        closesocket(tempSocket);
    }

    return false;
}

unsigned short CNetwork::GetUdpServerPort(SOCKET socket)
{
    sockaddr_in recvAddr;
	socklen_t addrLen = sizeof(recvAddr);
    if (getsockname(socket, (struct sockaddr *)&recvAddr, &addrLen) == 0 &&
        recvAddr.sin_family == AF_INET &&
        addrLen == sizeof(recvAddr))
    {
        return ntohs(recvAddr.sin_port);
    }
    return 0;
}

unsigned short CNetwork::GetUdpServerPort()
{
    return GetUdpServerPort(mUDPSocket);
}

unsigned short CNetwork::GetUdpBroadcastServerPort()
{
    return GetUdpServerPort(mUDPBroadcastSocket);
}

// Receive a data packet. Data is stored in a local static buffer
// Returns number of bytes in received message, 0 on timeout or -1 if there is an error. 
int CNetwork::Receive(char* rtDataBuff, int dataBufSize, bool header, int timeout, unsigned int *ipAddr)
{
    int recieved = 0;
    sockaddr_in source_addr;
    socklen_t fromlen = sizeof(source_addr);

    fd_set readFDs, exceptFDs;
    FD_ZERO(&readFDs);
    FD_ZERO(&exceptFDs);

    if (mSocket != INVALID_SOCKET)
    {
        FD_SET(mSocket, &readFDs);
        FD_SET(mSocket, &exceptFDs);
    }
    if (mUDPSocket != INVALID_SOCKET)
    {
        FD_SET(mUDPSocket, &readFDs);
        FD_SET(mUDPSocket, &exceptFDs);
    }
    if (mUDPBroadcastSocket != INVALID_SOCKET)
    {
        FD_SET(mUDPBroadcastSocket, &readFDs);
        FD_SET(mUDPBroadcastSocket, &exceptFDs);
    }

    TIMEVAL* pTimeval;
    TIMEVAL  sTimeval;

    if (timeout < 0)
    {
        pTimeval = nullptr;
    }
    else
    {
        sTimeval.tv_sec  = timeout / 1000000;
        sTimeval.tv_usec = timeout % 1000000;
        pTimeval = &sTimeval;
    }

#ifdef _WIN32
    const int nfds = 0;
#else
    const int nfds = std::max(mSocket, std::max(mUDPSocket, mUDPBroadcastSocket)) + 1;
#endif

    // Wait for activity on the TCP and UDP sockets.
    int selectRes = select(nfds, &readFDs, nullptr, &exceptFDs, pTimeval);
    if (selectRes == 0)
    {
        return 0; // Select timeout.
    }
    if (selectRes > 0)
    {
        if (FD_ISSET(mSocket, &exceptFDs))
        {
            // General socket error
            FD_CLR(mSocket, &exceptFDs);
            SetErrorString();
            recieved = SOCKET_ERROR;
        }
        else if (FD_ISSET(mSocket, &readFDs))
        {
            recieved = recv(mSocket, rtDataBuff, header ? 8 : dataBufSize, 0);
            FD_CLR(mSocket, &readFDs);
        }
        else if (FD_ISSET(mUDPSocket, &exceptFDs))
        {
            // General socket error
            FD_CLR(mUDPSocket, &exceptFDs);
            SetErrorString();
            recieved = SOCKET_ERROR;
        }
        else if (FD_ISSET(mUDPSocket, &readFDs))
        {
            recieved = recvfrom(mUDPSocket, rtDataBuff, dataBufSize, 0, (sockaddr*)&source_addr, &fromlen);
            FD_CLR(mUDPSocket, &readFDs);
        }
        else if (FD_ISSET(mUDPBroadcastSocket, &exceptFDs))
        {
            // General socket error
            FD_CLR(mUDPBroadcastSocket, &exceptFDs);
            SetErrorString();
            recieved = SOCKET_ERROR;
        }
        else if (FD_ISSET(mUDPBroadcastSocket, &readFDs))
        {
            recieved = recvfrom(mUDPBroadcastSocket, rtDataBuff, dataBufSize, 0, (sockaddr*)&source_addr, &fromlen);
            FD_CLR(mUDPBroadcastSocket, &readFDs);
            if (ipAddr)
            {
                *ipAddr = source_addr.sin_addr.s_addr;
            }
        }
    }
    else
    {
        recieved = -1;
    }

    if (recieved == -1)
    {
        SetErrorString();
    }
    return recieved;
}


bool CNetwork::Send(const char* sendBuf, int size)
{
    int sent = 0;
    int totalSent = 0;

    while (totalSent < size)
    {
        sent = send(mSocket, sendBuf + totalSent, size - totalSent, 0);
        if (sent == SOCKET_ERROR)
        {
            SetErrorString();
            return false;
        }
        totalSent += sent;
    }
    return true;
}


bool CNetwork::SendUDPBroadcast(const char* sendBuf, int size, short port, unsigned int filterAddr /* = 0 */)
{
    bool broadCastSent = false;

    if (mUDPBroadcastSocket != INVALID_SOCKET)
    {
#ifdef _WIN32
        IP_ADAPTER_INFO* ifap = nullptr;
        IP_ADAPTER_INFO* ifa = nullptr;
        ULONG ulLen = 0;
        DWORD erradapt;
        
        // Find all network interfaces.
        erradapt = ::GetAdaptersInfo(ifap, &ulLen);
        if (erradapt == ERROR_BUFFER_OVERFLOW)
        {
            ifap = (IP_ADAPTER_INFO*)malloc(ulLen);
            erradapt = ::GetAdaptersInfo(ifap, &ulLen);      
        }

        if (erradapt == ERROR_SUCCESS)
        {
            sockaddr_in recvAddr;
            recvAddr.sin_family = AF_INET;
            recvAddr.sin_port = htons(port);
            recvAddr.sin_addr.s_addr = 0xffffffff;

            // Send broadcast on all Ethernet interfaces.
            ifa = ifap;
            while (ifa)
            {
                if (ifa->Type == MIB_IF_TYPE_ETHERNET)
                {
                    unsigned int nIPaddr;
                    unsigned int nIPmask;
                    
                    if (inet_pton(AF_INET, ifa->IpAddressList.IpAddress.String, &nIPaddr) == 0 ||
                        inet_pton(AF_INET, ifa->IpAddressList.IpMask.String, &nIPmask) == 0)
                    {
                        return false;
                    }
                    recvAddr.sin_addr.s_addr = nIPaddr | (~nIPmask);
                    if (recvAddr.sin_addr.s_addr != (filterAddr | (~nIPmask)))
                    {
                        if (sendto(mUDPBroadcastSocket, sendBuf, size, 0, (sockaddr*)&recvAddr, sizeof(recvAddr)) == size)
                        {
                            broadCastSent = true;
                        }
                    }
                }
                ifa = ifa->Next;
            }
        }
        free(ifap);
#else
        // Find all network interfaces.
        struct ifaddrs* ifap = nullptr;
		if (getifaddrs(&ifap) == 0)
		{
			sockaddr_in recvAddr;
			recvAddr.sin_family = AF_INET;
			recvAddr.sin_port = htons(port);
			recvAddr.sin_addr.s_addr = 0xffffffff;

			// Send broadcast on all Ethernet interfaces.
			auto* ifa = ifap;
			while (ifa)
			{
				if (ifa->ifa_addr->sa_family == AF_INET)
				{
                    auto* sa = (struct sockaddr_in *) ifa->ifa_addr;
                    auto ipAddr = sa->sin_addr.s_addr;

                    auto* saMask = (struct sockaddr_in *) ifa->ifa_netmask;
                    auto ipMask = saMask->sin_addr.s_addr;

                    recvAddr.sin_addr.s_addr = ipAddr | (~ipMask);
                    if (recvAddr.sin_addr.s_addr != (filterAddr | (~ipMask)))
                    {
                        if (sendto(mUDPBroadcastSocket, sendBuf, size, 0, (sockaddr*)&recvAddr, sizeof(recvAddr)) == size)
                        {
                            broadCastSent = true;
                        }
                    }
				}
				ifa = ifa->ifa_next;
			}
		}
		freeifaddrs(ifap);
#endif
    }

    return broadCastSent;
} // SendUDPBroadcast


void CNetwork::SetErrorString()
{ 
#ifdef _WIN32
    char* error = nullptr; 
    mLastError = GetLastError(); 
    FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, nullptr, mLastError, 0, reinterpret_cast<LPTSTR>(&error), 0, nullptr); 
    sprintf(mErrorStr, "%s", error); 
    LocalFree(error);
#else
	mLastError = errno;
	char* error = strerror(mLastError);
	if (error != nullptr)
	{
		sprintf(mErrorStr, "%s", error);
	}
#endif
}


char* CNetwork::GetErrorString()
{
    return mErrorStr;
}


int CNetwork::GetError() const
{
    return mLastError;
}


bool CNetwork::IsLocalAddress(unsigned int nAddr) const
{
#ifdef _WIN32
    IP_ADAPTER_INFO* pAdptInfo = nullptr;
    IP_ADAPTER_INFO* pNextAd = nullptr;
    DWORD            erradapt;
    ULONG            ulLen = 0;

    // Find all network interfaces.
    erradapt = ::GetAdaptersInfo(pAdptInfo, &ulLen);
    if (erradapt == ERROR_BUFFER_OVERFLOW)
    {
        pAdptInfo = (IP_ADAPTER_INFO*)malloc(ulLen);
        erradapt = ::GetAdaptersInfo(pAdptInfo, &ulLen);      
    }

    if (erradapt == ERROR_SUCCESS)
    {
        pNextAd = pAdptInfo;
        while(pNextAd)
        {
            if (pNextAd->Type == MIB_IF_TYPE_ETHERNET)
            {
                // Check if it's a response from a local interface.
                unsigned int addr;
                if (inet_pton(AF_INET, pNextAd->IpAddressList.IpAddress.String, &addr) != 0)
                {
                    return addr == nAddr;
                }
            }
            pNextAd = pNextAd->Next;
        }
    }
	free(pAdptInfo);
#else
	struct ifaddrs* pAdptInfo = nullptr;
	struct ifaddrs* pNextAd = nullptr;
	if (getifaddrs(&pAdptInfo) == 0)
	{
		pNextAd = pAdptInfo;
		while (pNextAd)
		{
			if (pNextAd->ifa_addr->sa_family == AF_INET)
			{
				struct sockaddr_in* pNextAd_in = (struct sockaddr_in *)pNextAd->ifa_addr;
				if (pNextAd_in->sin_addr.s_addr == nAddr)
				{
					return true;
				}
			}
			pNextAd = pNextAd->ifa_next;
		}
	}
	freeifaddrs(pAdptInfo);
#endif
    return false;
}