
#include "pid.h"



PID_t createPID(float _Kp, float _Ki, float _Kd,
                uint16_t setpoint, bool mode,
                uint16_t min_angle, uint16_t max_angle){

    PID_t pid = {
        .Kp         =   _Kp,
        .Ki         =   _Ki,
        .Kd         =   _Kd,
        .setpoint   =   setpoint,
        .error      =   { 0,0 },
        .dt         =   0.01,
        .output     =   { 0,0 }, 
        .P          =   0,
        .I          =   0,
        .D          =   0,
        .integral   =   0,
        .min        =   min_angle,
        .max        =   max_angle,
        .inverted_mode = mode
    };
    return pid;
}


Serv PIDCompute(PID_t* pidX, PID_t* pidY, Ball_t ball,float setpointX, float setpointY, float limite, Machine machine) {

float pixel_to_cm = 20.0/404;

    pidX->error[1] = pidX->error[0];
    pidX->output[1] = pidX->output[0];

    pidX->error[0] = ( (ball.x[0] - pidX->setpoint)*pixel_to_cm + setpointX*1.0); 
    cout<<"ball x: "<<(ball.x[0] )*pixel_to_cm<<endl;
    cout<<"ball y: "<<ball.y[0]*pixel_to_cm<<endl;

    pidX->integral += pidX->error[0] + pidX->error[1];

    
    float derivate = (pidX->error[0]- pidX->error[1]);
    derivate = isnan(derivate) ||isinf(derivate)? 0 : derivate;

    pidX->P = pidX->Kp * pidX->error[0];
    pidX->I = pidX->Ki * pidX->integral;
    pidX->D = pidX->Kd * derivate;

    pidX->output[0] =
             pidX->P + pidX->I + pidX->D;
    

///////////////////////////////// y-axis /////////////////////////////////////////////////////////////

    pidY->error[1] = pidY->error[0];
    pidY->output[1] = pidY->output[0]; 

    pidY->error[0] =( (ball.y[0] -pidY->setpoint)*pixel_to_cm + setpointY*1.0);
    pidY->integral += pidY->error[0] + pidY->error[1];

    derivate = (pidY->error[0]- pidY->error[1]);
    derivate = isnan(derivate) ||isinf(derivate)? 0 : derivate;

    pidY->P = pidY->Kp * pidY->error[0];
    pidY->I = pidY->Ki * pidY->integral;
    pidY->D = pidY->Kd * derivate;

    pidY->output[0] =
             pidY->P + pidY->I + pidY->D;

//==============================================================================
    pidX->output[0] = saturationFilter(pidX->output[0],-limite,limite);
    pidY->output[0] = saturationFilter(pidY->output[0],-limite,limite);
    
    Serv ang;
    uint8_t pos[3];
    for (int i = 0; i < 3; i++) {
      pos[i] = round((machine.theta(i, 7.54, -pidX->output[0], pidY->output[0])) );
    }
    ang.ang1 = pos[0];
    ang.ang2 = pos[1];
    ang.ang3 = pos[2];
    return ang;

}

inline float saturationFilter(float value , float T_MIN, float T_MAX){
    if (value <= T_MIN) return T_MIN;
    if (value >= T_MAX) return T_MAX;
    else return value;
}

bool changeSetpoint(PID_t* XPID, PID_t* YPID, Ball_t* ball, Point_t sp){
    if( ((sp.x > SETPOINT_X+CONTROL_AREA/2) || (sp.x < SETPOINT_X-CONTROL_AREA/2)) &&
        ((sp.x > SETPOINT_X+CONTROL_AREA/2) || (sp.x < SETPOINT_X-CONTROL_AREA/2)) ){
        
        ball->smooth_dx = sp.x - XPID->setpoint;
        XPID->setpoint = sp.x;

        ball->smooth_dy = sp.y - YPID->setpoint;
        YPID->setpoint = sp.y;

        return true;
    }
    else return false;
}

