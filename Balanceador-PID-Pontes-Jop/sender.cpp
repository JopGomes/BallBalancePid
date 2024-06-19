#include "sender.h"

UDPSender::UDPSender(const char* ip, int port) {
    WSADATA wsaData;
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        std::cerr << "WSAStartup failed: " << iResult << std::endl;
        exit(1);
    }

    SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (SendSocket == INVALID_SOCKET) {
        std::cerr << "socket failed: " << WSAGetLastError() << std::endl;
        WSACleanup();
        exit(1);
    }

    recvAddr.sin_family = AF_INET;
    recvAddr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &recvAddr.sin_addr);
}

UDPSender::~UDPSender() {
    closesocket(SendSocket);
    WSACleanup();
}

void UDPSender::sendString(const std::string& str) {
    const char* sendbuf = str.c_str();
    int sendbuflen = str.length();

    int iResult = sendto(SendSocket, sendbuf, sendbuflen, 0, (SOCKADDR*)&recvAddr, sizeof(recvAddr));
    if (iResult == SOCKET_ERROR) {
        std::cerr << "sendto failed: " << WSAGetLastError() << std::endl;
    }
}
