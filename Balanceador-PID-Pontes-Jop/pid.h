#ifndef PID_H
#define PID_H
#include "utils.h"
#include <inttypes.h>

using namespace std;

// Functions
PID_t createPID(float Kp, float Ki, float Kd, uint16_t setpoint, bool mode,
                uint16_t min_angle, uint16_t max_angle);

Serv PIDCompute(PID_t* pidX, PID_t* pidY, Ball_t ball, int k);

short saturationFilter(short value , short T_MIN, short T_MAX);

bool changeSetpoint(PID_t* XPID, PID_t* YPID,  Ball_t* ball, Point_t sp);

void printPID(PID_t pid);


#endif
