#include "httpServer.h"
#include <iostream>
#include <sstream>

#pragma comment(lib, "Ws2_32.lib")

httpServer::httpServer(int port) 
    : port(port), server_fd(INVALID_SOCKET), isAlive(true) {}

httpServer::~httpServer() {
    if (server_fd != INVALID_SOCKET) {
        closesocket(server_fd);
    }
    WSACleanup();
}

bool httpServer::start() {
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed\n";
        return false;
    }

    struct addrinfo *result = nullptr, hints;
    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    if (getaddrinfo(NULL, std::to_string(port).c_str(), &hints, &result) != 0) {
        std::cerr << "getaddrinfo failed\n";
        WSACleanup();
        return false;
    }

    server_fd = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (server_fd == INVALID_SOCKET) {
        std::cerr << "Error at socket(): " << WSAGetLastError() << std::endl;
        freeaddrinfo(result);
        WSACleanup();
        return false;
    }

    if (bind(server_fd, result->ai_addr, (int)result->ai_addrlen) == SOCKET_ERROR) {
        std::cerr << "bind failed: " << WSAGetLastError() << std::endl;
        freeaddrinfo(result);
        closesocket(server_fd);
        WSACleanup();
        return false;
    }

    freeaddrinfo(result);

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
    while (isAlive) {
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

        if (request.find("GET /center") != std::string::npos) {
            pattern = "center";
        } else if (request.find("GET /square") != std::string::npos) {
            pattern = "square";
        } else if (request.find("GET /triangle") != std::string::npos) {
            pattern = "triangle";
        } else if (request.find("GET /limniscate") != std::string::npos) {
            pattern = "limniscate";
        }

        std::string html_content = generate_html(pattern);
        char response[2048];
        int response_length = snprintf(response, sizeof(response), response_template, html_content.length(), html_content.c_str());
        send(new_socket, response, response_length, 0);

        closesocket(new_socket);
    }
}

void httpServer::stopServer(){
    this->isAlive = false;
}

std::string httpServer::generate_html(std::string pattern) {
    std::ostringstream oss;
    oss << "<html>"
        << "<head><title>HTTP Server</title></head>"
        << "<body style=\"font-family: Arial, sans-serif; text-align: center;\">"
        << "<h1>Patterns Control</h1>"
        << "<button style=\"margin: 10px; padding: 8px 20px; font-size: 16px;\" onclick=\"location.href='/center'\">Center</button>"
        << "<button style=\"margin: 10px; padding: 8px 20px; font-size: 16px;\" onclick=\"location.href='/square'\">Square</button>"
        << "<button style=\"margin: 10px; padding: 8px 20px; font-size: 16px;\" onclick=\"location.href='/triangle'\">Triangle</button>"
        << "<button style=\"margin: 10px; padding: 8px 20px; font-size: 16px;\" onclick=\"location.href='/limniscate'\">Limniscate</button>"
        << "<p style=\"margin-top: 30px;\">Format: " << pattern << "</p>"
        << "</body>"
        << "</html>";
    return oss.str();
}
