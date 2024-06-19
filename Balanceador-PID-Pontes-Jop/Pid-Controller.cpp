
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



Verts getCoordinatesFromAlphas(int8_t alphaXDegrees , int8_t alphaYDegrees){
    Verts newVerts;
    float alphaX = PI*(1.0*alphaXDegrees/180);
    float alphaY = PI*(1.0*alphaYDegrees/180);
    
    newVerts.P1.R = 12*cos(alphaY) - 4*cos(alphaX);
    newVerts.P1.H = 1.0*HEIGHT- 8.0*sin(alphaY); // P11
    newVerts.P2.R = 8*cos(alphaX);
    newVerts.P2.H = 1.0*HEIGHT+4*sin(alphaY)-4*sqrt(3)*sin(alphaX); // P9
    newVerts.P3.R = 8*cos(alphaX);
    newVerts.P3.H = 1.0*HEIGHT+ 4*sin(alphaY)+4*sqrt(3)*sin(alphaX); // P10
    return newVerts;
}

int coordinatesToBeta(Pair p){
    float H = p.H;
    float R = p.R;
	float betaRadian = asin((pow(H,2) + pow((8.0-R),2) + pow(L1,2)-pow(L2,2))/(2.0*L1*sqrt(pow(H,2)+pow((8.0-R),2)))) - asin((8.0-R)/(sqrt(pow(H,2)+pow((8.0-R),2))));
    int16_t betaDegrees = round(betaRadian*(180/PI));
    if (betaDegrees < 10) betaDegrees = 10;
    if(betaDegrees > 80 ) betaDegrees = 80;
	return (180 - betaDegrees);
}


Serv PIDCompute(PID_t* pidX, PID_t* pidY, Ball_t ball,float setpointX, float setpointY, float limite, Machine machine) {

float pixel_to_cm = 20.0/404;


/*=================================================================================*/
/*      X axis     */ 


    
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

    //output
    pidX->output[0] =
             pidX->P + pidX->I + pidX->D;
    


/*=================================================================================*/
/*      Y axis     */ 

    //set old error and output
    pidY->error[1] = pidY->error[0];
    pidY->output[1] = pidY->output[0]; 

    // update erro 
    pidY->error[0] =( (ball.y[0] -pidY->setpoint)*pixel_to_cm + setpointY*1.0);
    //cout << pidY->error[0]<< "erro\n"<< endl;

    //std::cout <<"erros: " <<pidX->error[0]<< " " << pidY->error[1]<<"\n" << std::endl;
    //Integral: update
    pidY->integral += pidY->error[0] + pidY->error[1];

    //Derivative: Update and filter
    derivate = (pidY->error[0]- pidY->error[1]);
    derivate = isnan(derivate) ||isinf(derivate)? 0 : derivate;

    pidY->P = pidY->Kp * pidY->error[0];
    pidY->I = pidY->Ki * pidY->integral;
    pidY->D = pidY->Kd * derivate;

    //output
    pidY->output[0] =
             pidY->P + pidY->I + pidY->D;
    //std::cout << pidX->output[0]<< " " << pidY->output[0]<<"\n" << std::endl;
//==============================================================================
    pidX->output[0] = saturationFilter(pidX->output[0],-limite,limite);
    pidY->output[0] = saturationFilter(pidY->output[0],-limite,limite);
    
    Serv ang;
    uint8_t pos[3];
    //std::cout << pidX->output[0]<< " " << pidY->output[0]<<"\n" << std::endl;
    for (int i = 0; i < 3; i++) {
      //printf("%lf ang\n", machine.theta(i, 7.54, -pidX->output[0], -pidY->output[0]));
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


void printPID(PID_t pid){
    printf("*********************************************\n\
            KP = %.2lf , Ki = %.2lf, KD = %.2lf \n\
            error: %d\n\
            dt:    %.4lf\n\
            integr:%d\n\
            output:%d\n*********************************************\n\n",
            pid.Kp, pid.Ki, pid.Kd,
            pid.error[0], pid.dt, pid.integral, pid.output[0]);

}
