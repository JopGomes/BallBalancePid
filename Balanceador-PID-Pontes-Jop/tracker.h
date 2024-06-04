
#ifndef TRACKER_H
#define TRACKER_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "utils.h"
#include "pid.h"

using namespace cv;

//box area to be controlled
#define 	BOX_SIZE			200

//max number of objects to be detected in frame
#define     MAX_NUM_OBJECTS     3

//minimum and maximum object area
#define     MIN_OBJECT_AREA     14*14
#define     MAX_OBJECT_AREA     FRAME_HEIGHT*FRAME_WIDTH/5
#define N 3

//some colors
const Scalar RED =          Scalar(0, 0, 251);
const Scalar GREEN =        Scalar(24, 244, 0);
const Scalar BLUE =         Scalar(205, 45, 0);
const Scalar CYAN =         Scalar(212, 214, 44);
const Scalar ORANGE =       Scalar(0, 165, 254);
const Scalar DARK_GREEN =   Scalar(35, 149, 22);
const Scalar BLACK =   		Scalar(1, 1, 1);

//initial min and max HSV filter values.
//these will be changed using trackbars
extern int H_MIN;
extern int H_MAX;
extern int S_MIN;
extern int S_MAX;
extern int V_MIN;
extern int V_MAX;

//names that will appear at the top of each window
extern const String windowName;
extern const String windowName1;
extern const String windowName2;
extern const String windowName3;
extern const String trackbarWindowName;
extern const String gainTrackbarWindowName;

Ball_t createBall(uint16_t _x, uint16_t _y);

int getWindowPos(cv::Point* point , cv::Mat mat);

void updateBall(Ball_t* b, uint16_t _x, uint16_t _y);

void on_trackbar( int, void* );

String intToString(int number);

void createTrackbars();

void createGainTrackbars(PID_t* XPID, PID_t* YPID);

void drawObjectV2(Ball b, Mat &frame, bool noise_error);

void drawLiveData(Mat &DATA, PID_t XPID, PID_t YPID);

void plotPos (Ball b, Mat &frame, uint16_t _x, uint16_t _y);

void morphOps(Mat &thresh);

void trackFilteredObject(Ball* b, Mat threshold);

void circleDetector(Mat cameraFeed, Mat threshold);

#endif