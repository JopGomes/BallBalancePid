#include "pid.h"
#include "client.h"
#include <iostream>
#include <sstream>
#include <string>

std::string createStringFromServ(const Serv& serv) {
    std::ostringstream oss;
    oss << serv.ang1 << "/" << serv.ang2 << "/" << serv.ang3;
    return oss.str();
}


int main(){
    std::string serverIP;
    int port;
    Client client;
    client.initialize(serverIP,port);
    PID_t pidX = createPID(KPx,KIx,KLx,SETPOINT_X, true, X_MIN_ANGLE, X_MAX_ANGLE);
    PID_t pidY = createPID(KPy,KIy,KLy,SETPOINT_Y, false, Y_MIN_ANGLE, Y_MAX_ANGLE);

    Serv ang;
    while(true){
        Ball_t ball; //espera receber a bola
        client.initialize(serverIP,port);
        PIDCompute(&pidX, &pidY, ball, &ang);
        std::string message;
        message = createStringFromServ(ang);
        client.sendMessage(message);
        client.closeConnection();
    }

}