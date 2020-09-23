#pragma once
#include <afxsock.h>
#include <wchar.h>

class CNewSocket : public CAsyncSocket
{
public:
    int m_nLenght;
    char m_szBuffer[512];

    void OnReceive(
        int nErrorCode
    );

    void OnSend(
        int nErrorCode
    );

};



class CMyServerSocket :
    public CAsyncSocket
{
public:
    CNewSocket* m_pSocket;
    bool isConnected;

    int length;
    char recvBuffer[512];

    void OnAccept(int nErrorCode);

    void OnConnect(int nErrorCode);

    void OnReceive(int nErrorCode);

};



    