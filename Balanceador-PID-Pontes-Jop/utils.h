#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <ostream>
#include <iostream>

#define     bool        int
#define     false       0
#define     true        1

typedef struct PID_t{
    float Kp, Ki, Kd;
    uint16_t setpoint;
    float error[2];
    float dt;
    float output[2];
    float P, I, D;
    float integral;
    uint16_t min, max;
    bool inverted_mode;
}PID_t;

typedef struct Ball {
    bool        detected;
    uint16_t    x[8];
    uint16_t    y[8];
    short       dx[8];
    short       dy[8];
    short       smooth_dx;
    short       smooth_dy;
} Ball_t;


//_ Data Structure _______________________

typedef struct ServoConfig {
    uint16_t     xPulse;
    uint16_t     yPulse;
} ServoConfig_t;

typedef struct Point{
    uint16_t x;
    uint16_t y;
} Point_t;

typedef struct Pair {
    float R;
    float H;
} Pair;

typedef struct Verts {
    Pair P1; 
    Pair P2;
    Pair P3;
} Verts;

typedef struct Serv {
    uint8_t ang1; // P11
    uint8_t ang2; // P9
    uint8_t ang3; // P10
} Serv;

//_ Global Variables _______________________

/*===============================================*/
/*ABOUT SERVOS*/
#define   ANGLE_OFFSET      20

#define   ANGLE_ORIGIN      135

#define   X_HALF_ANGLE      60
#define	  X_MAX_ANGLE       X_HALF_ANGLE+ANGLE_OFFSET
#define   X_MIN_ANGLE       X_HALF_ANGLE-ANGLE_OFFSET

#define   Y_HALF_ANGLE      60
#define	  Y_MAX_ANGLE       Y_HALF_ANGLE+ANGLE_OFFSET
#define   Y_MIN_ANGLE       Y_HALF_ANGLE-ANGLE_OFFSET

#define     PI                  3.1415

/*===============================================*/
/*ABOUT MECANIC*/
// #define     PI                  3.14159265358979323846

#define     CONTROL_AREA        360

#define     FRAME_WIDTH         640
#define     FRAME_HEIGHT        480

#define     SETPOINT_X          FRAME_WIDTH/2
#define     SETPOINT_Y          FRAME_HEIGHT/2

#define L1 4.0
#define L2 5.5
#define HEIGHT 7.54 
#define CORNER_BASE 8
#define CORNER_PLATFORM 8

/*===============================================*/
/*ABOUT PID*/

#define KPx 1e-2
#define KDx 7e-1
#define KIx 1e-4

#define KPy KPx
#define KDy KDx
#define KIy KIx

/*===============================================*/
/*ABOUT Socket*/


#endif