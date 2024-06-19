#ifndef httpServer_H
#define httpServer_H

#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>

class httpServer {
public:
    httpServer(int port);
    ~httpServer();
    bool start();
    void run(std::string& pattern);
    void stopServer();

private:
    int port;
    bool isAlive;
    SOCKET server_fd;
    WSADATA wsaData;

    const char* response_template =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n"
        "%s";

    std::string generate_html(std::string pattern);
};

#endif // httpServer_H
