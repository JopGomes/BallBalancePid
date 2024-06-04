#include <iostream>
#include <cstring>
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <string>
#include <windows.h> // For Sleep function
#include "pid.h"

#pragma comment(lib, "ws2_32.lib")
using namespace std;


class Client {
public:
    Client();
    ~Client();
    bool initialize(std::string serverIP);
    void setMessage(std::string message);
    void run();
    void stopServer();

private:
    bool sendMessage(SOCKET serverSocket, const std::string& message);
    bool isAlive;
    int port;
    std::string message;
    SOCKET clientSocket;
    WSADATA wsaData;
};

Client::Client(){
    clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (clientSocket == INVALID_SOCKET) {
        std::cerr << "Erro ao criar o socket." << std::endl;
        WSACleanup();
    }
}

Client::~Client() {
    closesocket(clientSocket);
    WSACleanup();
    
}

bool Client::initialize(std::string serverIP){
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "Erro ao inicializar o Winsock." << std::endl;
    }
    // Configuração do endereço do servidor
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port); // Porta do servidor
    inet_pton(AF_INET, serverIP.c_str(), &serverAddr.sin_addr); // IP do servidor

    // Conexão ao servidor
    if (connect(clientSocket, reinterpret_cast<SOCKADDR*>(&serverAddr), sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Erro ao conectar ao servidor." << std::endl;
        closesocket(clientSocket);
        WSACleanup();
        return false;
    }
    return true;
}

bool Client::sendMessage(SOCKET serverSocket, const std::string& message) {
    if (send(serverSocket, message.c_str(), message.length(), 0) == SOCKET_ERROR) {
        std::cerr << "Erro ao enviar dados." << std::endl;
        closesocket(serverSocket);
        WSACleanup();
        return false;
    }
    return true;
}

void Client::run() {

    int c;
    
}
