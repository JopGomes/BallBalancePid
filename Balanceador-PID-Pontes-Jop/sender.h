//sender.h

#ifndef SENDER_H
#define SENDER_H

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>

#pragma comment(lib, "Ws2_32.lib")

class UDPSender {
public:
    UDPSender(const char* ip, int port);
    ~UDPSender();
    void sendString(const std::string& str);

private:
    SOCKET SendSocket;
    sockaddr_in recvAddr;
};

#endif
