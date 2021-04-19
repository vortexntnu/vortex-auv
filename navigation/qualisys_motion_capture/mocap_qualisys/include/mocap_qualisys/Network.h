#ifndef NETWORK_H
#define NETWORK_H

#ifdef _WIN32
	#define WIN32_LEAN_AND_MEAN
	#include <winsock2.h>
#else
	#define INVALID_SOCKET	-1
	#define SOCKET int
#endif

class CNetwork
{
public:
    CNetwork();
    ~CNetwork();
    bool  Connect(const char* pServerAddr, unsigned short nPort);
    void  Disconnect();
    bool  Connected() const;
    bool  CreateUDPSocket(unsigned short &nUDPPort, bool bBroadcast = false);
    int   Receive(char* rtDataBuff, int nDataBufSize, bool bHeader, int nTimeout, unsigned int *ipAddr = nullptr);
    bool  Send(const char* pSendBuf, int nSize);
    bool  SendUDPBroadcast(const char* pSendBuf, int nSize, short nPort, unsigned int nFilterAddr = 0);
    char* GetErrorString();
    int   GetError() const;
    bool  IsLocalAddress(unsigned int nAddr) const;
    unsigned short GetUdpServerPort();
    unsigned short GetUdpBroadcastServerPort();

private:
    bool InitWinsock();
    void SetErrorString();
    unsigned short GetUdpServerPort(SOCKET nSocket);

private:
    SOCKET     mSocket;
    SOCKET     mUDPSocket;
    SOCKET     mUDPBroadcastSocket;
    char       mErrorStr[256];
    unsigned long mLastError;
};


#endif