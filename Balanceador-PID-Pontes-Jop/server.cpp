#include <iostream>
#include <cstring>
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <string>

#pragma comment(lib, "ws2_32.lib")
using namespace std;

int sendMessage(SOCKET clientSocket, const std::string& message) {
    if (send(clientSocket, message.c_str(), message.length(), 0) == SOCKET_ERROR) {
        std::cerr << "Erro ao enviar dados." << std::endl;
        closesocket(clientSocket);
        return 1;
    }
    return 0;
}

int main() {
    int port = 80;

    // Inicialização do Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "Erro ao inicializar o Winsock." << std::endl;
        return 1;
    }

    // Criação do socket
    SOCKET serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (serverSocket == INVALID_SOCKET) {
        std::cerr << "Erro ao criar o socket." << std::endl;
        WSACleanup();
        return 1;
    }

    // Configuração do endereço do servidor
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port); // Porta do servidor
    serverAddr.sin_addr.s_addr = INADDR_ANY; // Qualquer IP

    // Bind do socket
    if (bind(serverSocket, reinterpret_cast<SOCKADDR*>(&serverAddr), sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Erro ao fazer o bind do socket." << std::endl;
        closesocket(serverSocket);
        WSACleanup();
        return 1;
    }

    // Inicia a escuta de conexões
    if (listen(serverSocket, SOMAXCONN) == SOCKET_ERROR) {
        std::cerr << "Erro ao iniciar a escuta no socket." << std::endl;
        closesocket(serverSocket);
        WSACleanup();
        return 1;
    }

    std::cout << "Servidor em escuta na porta " << port << std::endl;

    // Loop para aceitar conexões de clientes e responder apenas quando solicitado
    while (true) {
        // Aguarda a conexão do cliente
        sockaddr_in clientAddr;
        int clientAddrSize = sizeof(clientAddr);
        SOCKET clientSocket = accept(serverSocket, reinterpret_cast<SOCKADDR*>(&clientAddr), &clientAddrSize);
        if (clientSocket == INVALID_SOCKET) {
            std::cerr << "Erro ao aceitar conexão do cliente." << std::endl;
            closesocket(serverSocket);
            WSACleanup();
            return 1;
        }

        std::cout << "Cliente conectado." << std::endl;

        // Aguarda a solicitação do cliente
        char buffer[1024];
        int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
        if (bytesReceived > 0) {
            buffer[bytesReceived] = '\0';
            std::string request(buffer);
            std::cout << "Solicitação do cliente: " << request << std::endl;

            // Envie uma resposta ao cliente
            std::string response = "x/y/z\n";
            if (sendMessage(clientSocket, response)) {
                std::cerr << "Erro ao enviar resposta para o cliente." << std::endl;
                closesocket(serverSocket);
                WSACleanup();
                return 1;
            }
            std::cout << "Resposta enviada para o cliente." << std::endl;
        }

        // Fechamento do socket do cliente
        closesocket(clientSocket);
        std::cout << "Cliente desconectado." << std::endl;
    }

    // Fechamento do socket do servidor e limpeza do Winsock
    closesocket(serverSocket);
    WSACleanup();

    return 0;
}
