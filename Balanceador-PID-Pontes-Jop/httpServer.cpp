#include "httpServer.h"
#include <iostream>
#include <sstream>

// Link with Ws2_32.lib
#pragma comment(lib, "Ws2_32.lib")

httpServer::httpServer(int port) 
    : port(port), server_fd(INVALID_SOCKET), isAlive(true) {}

httpServer::~httpServer() {
    // Limpa e encerra Winsock
    if (server_fd != INVALID_SOCKET) {
        closesocket(server_fd);
    }
    WSACleanup();
}

bool httpServer::start() {
    // Inicializa Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed\n";
        return false;
    }

    struct addrinfo *result = NULL, hints;
    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve o endereço e a porta do servidor
    if (getaddrinfo(NULL, std::to_string(port).c_str(), &hints, &result) != 0) {
        std::cerr << "getaddrinfo failed\n";
        WSACleanup();
        return false;
    }

    // Cria um socket para conectar ao servidor
    server_fd = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (server_fd == INVALID_SOCKET) {
        std::cerr << "Error at socket(): " << WSAGetLastError() << std::endl;
        freeaddrinfo(result);
        WSACleanup();
        return false;
    }

    // Liga o socket
    if (bind(server_fd, result->ai_addr, (int)result->ai_addrlen) == SOCKET_ERROR) {
        std::cerr << "bind failed: " << WSAGetLastError() << std::endl;
        freeaddrinfo(result);
        closesocket(server_fd);
        WSACleanup();
        return false;
    }

    freeaddrinfo(result);

    // Escuta no socket
    if (listen(server_fd, SOMAXCONN) == SOCKET_ERROR) {
        std::cerr << "listen failed: " << WSAGetLastError() << std::endl;
        closesocket(server_fd);
        WSACleanup();
        return false;
    }

    std::cout << "Server running on port " << port << "\n";
    return true;
}

void httpServer::run(std::string& pattern) {
    SOCKET new_socket;
    while (true) {
        // Aceita uma conexão do cliente
        new_socket = accept(server_fd, NULL, NULL);
        if (new_socket == INVALID_SOCKET) {
            std::cerr << "accept failed: " << WSAGetLastError() << std::endl;
            continue;
        }

        char buffer[1024];
        int valread = recv(new_socket, buffer, 1024, 0);
        if (valread == SOCKET_ERROR) {
            std::cerr << "recv failed: " << WSAGetLastError() << std::endl;
            closesocket(new_socket);
            continue;
        }

        std::string request(buffer, valread);

        // Processa a requisição
        if (request.find("GET /center") != std::string::npos) {
            pattern = "center";
        } else if (request.find("GET /square") != std::string::npos) {
            pattern = "square";
        } else if (request.find("GET /triangle") != std::string::npos) {
            pattern = "triangle";
        } else if (request.find("GET /limniscate") != std::string::npos) {
            pattern = "limniscate";
        }

        // Gera a resposta HTML
        std::string html_content = generate_html(pattern);
        char response[2048];
        int response_length = snprintf(response, sizeof(response), response_template, html_content.length(), html_content.c_str());

        // Envia a resposta ao cliente
        send(new_socket, response, response_length, 0);

        // Fecha a conexão
        closesocket(new_socket);
    }
}

void httpServer::stopServer(){
    this->isAlive = false;
}

std::string httpServer::generate_html(std::string pattern) {
    std::ostringstream oss;
    oss << "<html>"
        << "<body>"
        << "<h1>Control Panel</h1>"
        << "<button onclick=\"location.href='/center'\">Centro</button>"
        << "<button onclick=\"location.href='/square'\">Quadrado</button>"
        << "<button onclick=\"location.href='/triangle'\">Triangulo</button>"
        << "<button onclick=\"location.href='/limniscate'\">Limniscate</button>"
        << "<p>Formato: " << pattern << "</p>"
        << "</body>"
        << "</html>";
    return oss.str();
}
