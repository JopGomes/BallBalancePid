#include <iostream>
#include <cstring>
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <string>
#include <windows.h> // For Sleep function

#pragma comment(lib, "ws2_32.lib")
using namespace std;

int sendMessage(SOCKET clientSocket, const std::string& message) {
    if (send(clientSocket, message.c_str(), message.length(), 0) == SOCKET_ERROR) {
        std::cerr << "Erro ao enviar dados." << std::endl;
        closesocket(clientSocket);
        WSACleanup();
        return 1;
    }
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Uso: " << argv[0] << " <IP do servidor> <porta>" << std::endl;
        return 1;
    }

    std::string serverIP = argv[1];
    int port = std::stoi(argv[2]);
    cout << "Endereço: " << serverIP << "/porta: " << port << endl;

    // Inicialização do Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "Erro ao inicializar o Winsock." << std::endl;
        return 1;
    }

    // Criação do socket
    SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (clientSocket == INVALID_SOCKET) {
        std::cerr << "Erro ao criar o socket." << std::endl;
        WSACleanup();
        return 1;
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
        return 1;
    }

    std::cout << "Conectado ao servidor." << std::endl;

    // Loop to send messages as they are generated
    std::string message;

        message = "x/y/z\n";
        if (sendMessage(clientSocket, message + "\n")) {
            return 1;
        }
        std::cout << "Dados enviados para o servidor: " << message << std::endl;

    // Fechamento do socket e limpeza do Winsock
    closesocket(clientSocket);
    WSACleanup();

    return 0;
}
