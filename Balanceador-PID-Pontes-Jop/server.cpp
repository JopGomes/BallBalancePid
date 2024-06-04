#include <iostream>
#include <cstring>
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <string>

#pragma comment(lib, "ws2_32.lib")

class Server {
public:
    Server(int port);
    ~Server();
    bool initialize();
    void run();

private:
    bool sendMessage(SOCKET clientSocket, const std::string& message);
    int port;
    SOCKET serverSocket;
    WSADATA wsaData;
};

Server::Server(int port) : port(port), serverSocket(INVALID_SOCKET) {}

Server::~Server() {
    closesocket(serverSocket);
    WSACleanup();
}

bool Server::initialize() {
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "Erro ao inicializar o Winsock." << std::endl;
        return false;
    }

    serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (serverSocket == INVALID_SOCKET) {
        std::cerr << "Erro ao criar o socket." << std::endl;
        WSACleanup();
        return false;
    }

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(serverSocket, reinterpret_cast<SOCKADDR*>(&serverAddr), sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Erro ao fazer o bind do socket." << std::endl;
        closesocket(serverSocket);
        WSACleanup();
        return false;
    }

    if (listen(serverSocket, SOMAXCONN) == SOCKET_ERROR) {
        std::cerr << "Erro ao iniciar a escuta no socket." << std::endl;
        closesocket(serverSocket);
        WSACleanup();
        return false;
    }

    std::cout << "Servidor em escuta na porta " << port << std::endl;
    return true;
}

void Server::run() {
    while (true) {
        sockaddr_in clientAddr;
        int clientAddrSize = sizeof(clientAddr);
        SOCKET clientSocket = accept(serverSocket, reinterpret_cast<SOCKADDR*>(&clientAddr), &clientAddrSize);
        if (clientSocket == INVALID_SOCKET) {
            std::cerr << "Erro ao aceitar conexão do cliente." << std::endl;
            continue;
        }

        std::cout << "Cliente conectado." << std::endl;

        char buffer[1024];
        int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
        if (bytesReceived > 0) {
            buffer[bytesReceived] = '\0';
            std::string request(buffer);
            std::cout << "Solicitação do cliente: " << request << std::endl;

            if (request == "angles\n") {
                std::string response = "x/y/z\n";
                if (sendMessage(clientSocket, response)) {
                    std::cerr << "Erro ao enviar resposta para o cliente." << std::endl;
                } else {
                    std::cout << "Resposta enviada para o cliente." << std::endl;
                }
            }
        }

        closesocket(clientSocket);
        std::cout << "Cliente desconectado." << std::endl;
    }
}

bool Server::sendMessage(SOCKET clientSocket, const std::string& message) {
    if (send(clientSocket, message.c_str(), message.length(), 0) == SOCKET_ERROR) {
        std::cerr << "Erro ao enviar dados." << std::endl;
        closesocket(clientSocket);
        return false;
    }
    return true;
}

int main() {
    int port = 80;
    Server server(port);
    if (server.initialize()) {
        server.run();
    }
    return 0;
}