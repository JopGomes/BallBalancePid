#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <WinSock2.h>
#include <windows.h>
#include <cmath>
#include <array>

#include "pid.h"
#include "client.h"
#include "tracker.h"
#include "utils.h"

const String windowName = "Ball Balancing PID System";
const String windowName2 = "HSV view";
const String trackbarWindowName = "HSV Trackbars";

std::string createStringFromServ(const Serv& serv) {
    std::ostringstream oss;
    oss << serv.ang1 << "/" << serv.ang2 << "/" << serv.ang3;
    return oss.str();
}


int main(){
    std::string serverIP = "x.x.x.x";
    int port = 80;
    Client client;

     VideoCapture cap(0); 

    if (!cap.isOpened()) { // Verifica se a câmera foi aberta corretamente
        std::cerr << "Erro ao abrir a câmera!" << std::endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    // Criação das janelas
    namedWindow(windowName);
    namedWindow(windowName2);

    createTrackbars();

    // Variáveis de imagem
    Mat frame, HSV, threshold;
    Ball_t ball;
    ball.detected = false;
    
    // Posição da janela
    cv::Point windowPos;
    if (getWindowPos(&windowPos, frame) != 0) {
        std::cerr << "Erro ao obter a posição da janela!" << std::endl;
        return -1;
    }
    moveWindow(windowName, windowPos.x, windowPos.y);
    moveWindow(windowName2, windowPos.x + frame.cols, windowPos.y);
    
    PID_t pidX = createPID(KPx,KIx,KLx,SETPOINT_X, true, X_MIN_ANGLE, X_MAX_ANGLE);
    PID_t pidY = createPID(KPy,KIy,KLy,SETPOINT_Y, false, Y_MIN_ANGLE, Y_MAX_ANGLE);

    Serv ang;
    while(true){cap >> frame; // Captura um frame da câmera

        if (frame.empty()) {
            std::cerr << "Frame vazio!" << std::endl;
            break;
        }

        // Converte a imagem para o espaço de cores HSV
        cvtColor(frame, HSV, COLOR_BGR2HSV);

        // Filtra a imagem HSV com base nos valores dos trackbars
        inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
        morphOps(threshold);
        trackFilteredObject(&ball, threshold);

        // Desenha o objeto rastreado na imagem original
        drawObjectV2(ball, frame, false);

        // Exibe a imagem original e a imagem HSV filtrada
        imshow(windowName, frame);
        imshow(windowName2, threshold);

        if(ball.detected){
            PIDCompute(&pidX, &pidY, ball, &ang);
            String message;
            message = createStringFromServ(ang);
            client.initialize(serverIP,port);
            client.sendMessage(message);
            client.closeConnection();
        }

        // Verifica se o usuário pressionou a tecla 'q' para sair
        if (waitKey(30) == 'q') {
            break;
        }
    }

}