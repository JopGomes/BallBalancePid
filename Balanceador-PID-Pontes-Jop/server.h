#ifndef SERVER_H
#define SERVER_H

#include <iostream>
#include <cstring>
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <string>

#pragma comment(lib, "ws2_32.lib")

using namespace std;

class Server {
public:
    Server(int port);
    ~Server();
    bool initialize();
    void setMessage(std::string message);
    void run();
    void stopServer();

private:
    bool sendMessage(SOCKET clientSocket, const std::string& message);
    bool isAlive;
    int port;
    std::string message;
    SOCKET serverSocket;
    WSADATA wsaData;
};

#endif // SERVER_H
