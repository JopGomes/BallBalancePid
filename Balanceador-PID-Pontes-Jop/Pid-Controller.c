
#include "utils.h"


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





Verts getCoordinatesFromAlphas(int alphax , int alphay){
    Verts newVerts;
    newVerts.P1.R = 12*cos(alphay) - 4*cos(alphax);
    newVerts.P1.H = HEIGHT-8*sin(alphay); // P11
    newVerts.P2.R = 8*cos(alphax);
    newVerts.P2.H = HEIGHT+4*sin(alphay)-4*sqrt(3)*sin(alphax); // P9
    newVerts.P3.R = 8*cos(alphax),HEIGHT;
    newVerts.P3.H = 4*sin(alphay)+4*sqrt(3)*sin(alphax); // P10
    return newVerts;
}

int coordinatesToBeta(float R, float H){
	double betaRadian = asin((pow(H,2) + pow((8-R),2) + pow(L1,2)-pow(L2,2))/(2*L1*sqrt(pow(H,2)+pow((8-R),2)))) - asin((8-R)/(sqrt(pow(H,2)+pow((8-R),2))));
	int betaDegrees = round(betaRadian*(180/PI));
	return betaDegrees;
}


void PIDCompute(PID_t* pidX, PID_t* pidY, Ball_t ball) {

/*=================================================================================*/
/*      X axis     */ 

    //set old error and output
    pidX->error[1] = pidX->error[0];
    pidX->output[1] = pidX->output[0]; 

    //compute new error
    if (!pidX->inverted_mode) pidX->error[0] = pidX->setpoint - ball.x[0];
    else pidX->error[0] = ball.x[0] - pidX->setpoint;

    //Integral: update and filter
    pidX->integral += pidX->error[0] * pidX->dt;
    pidX->integral = saturationFilter(pidX->integral, -150, +150); 

    //Derivative: Update and filter
    ball.smooth_dx = saturationFilter(ball.smooth_dx, -150, +150);
    if(!pidX->inverted_mode) ball.smooth_dx = -ball.smooth_dx;
    
    //Calculate components PID
    pidX->P = pidX->Kp * pidX->error[0];
    pidX->I = pidX->Ki * pidX->integral;
    pidX->D = pidX->Kd * (ball.smooth_dx/pidX->dt);

    //output
    pidX->output[0] =
            X_HALF_ANGLE + pidX->P + pidX->I + pidX->D;

     //output filter   
    pidX->output[0] = saturationFilter(pidX->output[0], pidX->output[1]-1500, pidX->output[1]+1500);
    pidX->output[0] = saturationFilter(pidX->output[0], pidX->min, pidX->max);


/*=================================================================================*/
/*      Y axis     */ 

    //set old error and output
    pidY->error[1] = pidY->error[0];
    pidY->output[1] = pidY->output[0]; 

    //compute new error
    if (!pidY->inverted_mode) pidY->error[0] = pidY->setpoint - ball.y[0];
    else pidY->error[0] = ball.y[0] - pidY->setpoint;

    //Integral: update and filter
    int filter_integral = 150;
    pidY->integral += pidY->error[0] * pidY->dt;
    pidY->integral = saturationFilter(pidY->integral, -filter_integral, +filter_integral); 

    //Derivative: Update and filter
    int filter_derivative = 150;
    ball.smooth_dy = saturationFilter(ball.smooth_dy, -filter_derivative, +filter_derivative);
    if(!pidY->inverted_mode) ball.smooth_dy = -ball.smooth_dy;
    
    pidY->P = pidY->Kp * pidY->error[0];
    pidY->I = pidY->Ki * pidY->integral;
    pidY->D = pidY->Kd * (ball.smooth_dy/pidY->dt);

    //output
    pidY->output[0] =
            Y_HALF_ANGLE + pidY->P + pidY->I + pidY->D;
    
    //output filter 
    int filter_value = 1500;
    pidY->output[0] = saturationFilter(pidY->output[0], pidY->output[1]-filter_value, pidY->output[1]+filter_value);
    pidY->output[0] = saturationFilter(pidY->output[0], pidY->min, pidY->max);
//===============================================================================
}

inline short saturationFilter(short value , short T_MIN, short T_MAX){
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
