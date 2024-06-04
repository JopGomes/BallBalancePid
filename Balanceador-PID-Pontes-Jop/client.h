#ifndef CLIENT_H
#define CLIENT_H

#include <iostream>
#include <cstring>
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <string>
#include <windows.h> // For Sleep function

class Client {
public:
    Client();
    ~Client();
    bool initialize(std::string serverIP,int port);
    void run(int x, int y);
    bool sendMessage(const std::string& message);
    bool closeConnection();

private:
    std::string message;
    SOCKET clientSocket;
    WSADATA wsaData;
};


#endif
