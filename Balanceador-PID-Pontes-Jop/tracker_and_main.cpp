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
#include<thread>
#include <chrono>

#include "utils.h"
#include "httpServer.h"
#include "pid.h"
#include "InverseKinematics.h"
#include "sender.h"


using namespace cv;
using namespace std;

int H_MIN = 82;
int H_MAX = 256;
int S_MIN = 103;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

//////////// constants definition /////////////
int KP_value = 1;
int KP_MAX = 100;
int KI_value = 17;
int KI_MAX = 100;
int KD_value = 71;
int KD_MAX = 100;
int limites_value = 1;
int limites_max = 100;
int servo1_offset_value = 2;
int servo1_offset_max = 100;
int servo2_offset_value = 0;
int servo2_offset_max = 100;
int servo3_offset_value = 0;
int servo3_offset_max = 100;

const String serverIP = "192.168.0.142";
VideoCapture cap(0);
const int port = 80;
bool isAlive = true;
string pattern = "center";
long timeI, timeII;

const String windowName = "Ball Balancing PID System";
const String windowName2 = "HSV view";
const String trackbarWindowName = "HSV Trackbars";

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define MIN_OBJECT_AREA 20*20
#define MAX_OBJECT_AREA FRAME_HEIGHT*FRAME_WIDTH/1.5
#define MAX_NUM_OBJECTS 50
#define N 3

const cv::Scalar RED = Scalar(0, 0, 255);
const cv::Scalar GREEN = Scalar(0, 255, 0);
const cv::Scalar BLUE = Scalar(255, 0, 0);
const cv::Scalar CYAN = Scalar(255, 255, 0);
const cv::Scalar ORANGE = Scalar(0, 165, 255);
const cv::Scalar DARK_GREEN = Scalar(0, 100, 0);

Ball_t createBall(uint16_t _x, uint16_t _y){
	Ball_t b = {
		.detected = false,
		.x 		= { _x,_x,_x,_x,_x,_x,_x,_x },
		.y 		= { _y,_y,_y,_y,_y,_y,_y,_y },
		.dx 	= { 0,0,0,0,0,0,0,0 },
		.dy 	= { 0,0,0,0,0,0,0,0 },
		.smooth_dx 	= 0,
		.smooth_dy 	= 0
	};
	return b;
}
void updateBall(Ball_t* b, uint16_t _x, uint16_t _y){
	//update X position
	b->x[7] = b->x[6];
    b->x[6] = b->x[5];
    b->x[5] = b->x[4];
	b->x[4] = b->x[3];
	b->x[3] = b->x[2];
	b->x[2] = b->x[1];
	b->x[1] = b->x[0];
	b->x[0] = _x;

	//update X speed
	b->dx[7] = b->dx[6];
	b->dx[6] = b->dx[5];
	b->dx[5] = b->dx[4];
	b->dx[4] = b->dx[3];
	b->dx[3] = b->dx[2];
	b->dx[2] = b->dx[1];
	b->dx[1] = b->dx[0];
	b->dx[0] = b->x[0] - b->x[1];

	//recursively update moving average for X
	b->smooth_dx =
		(N * b->smooth_dx - b->dx[N] + b->dx[0]) / N;

	
	//update Y position
	b->y[7] = b->y[6];
	b->y[6] = b->y[5];
	b->y[5] = b->y[4];
	b->y[4] = b->y[3];
	b->y[3] = b->y[2];
	b->y[2] = b->y[1];
	b->y[1] = b->y[0];
	b->y[0] = _y;
	
	//update Y speed
	b->dy[7] = b->dy[6];
	b->dy[6] = b->dy[5];
	b->dy[5] = b->dy[4];
	b->dy[4] = b->dy[3];
	b->dy[3] = b->dy[2];
	b->dy[2] = b->dy[1];
	b->dy[1] = b->dy[0];
	b->dy[0] = b->y[0] - b->y[1];
	
	//recursively update moving average
	b->smooth_dy =
		(N * (b->smooth_dy) - b->dy[N] + b->dy[0]) / N;
}

int getWindowPos(cv::Point* point, cv::Mat mat) {
  // Get screen resolution
  DEVMODE displayMode;
  EnumDisplaySettings(NULL, ENUM_CURRENT_SETTINGS, &displayMode);
  int x_res = displayMode.dmPelsWidth;
  int y_res = displayMode.dmPelsHeight;

  // Validate resolution
  if (x_res < 360 || x_res > 4096 || y_res < 240 || y_res > 4096) {
    fprintf(stderr, "Screen resolution wrong!\n");
    return -1;
  }

  // Adjust minimum resolution
  if (x_res < 720) x_res = 720;
  if (y_res < 480) y_res = 480;

  // Calculate window position
  point->x = (x_res - mat.cols) / 2;
  point->y = (y_res - mat.rows) / 2;

  return 0;
}

void on_trackbar( int, void* ){
	//empty function
}

String intToString(int number){
	stringstream ss;
	ss << number;
	return ss.str();
}

void createTrackbars(){
    namedWindow(trackbarWindowName, 0);

	char TrackbarName[16];
	sprintf( TrackbarName, "H_MIN");
	sprintf( TrackbarName, "H_MAX");
	sprintf( TrackbarName, "S_MIN");
	sprintf( TrackbarName, "S_MAX");
	sprintf( TrackbarName, "V_MIN");
	sprintf( TrackbarName, "V_MAX");


	namedWindow("Configuration", 1);
	char TrackbarNameC[16];
	sprintf( TrackbarNameC, "KP");
	sprintf( TrackbarNameC, "KI");
	sprintf( TrackbarNameC, "KD");
	sprintf( TrackbarNameC, "Limites");

    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
	
	/////configuration fase
	createTrackbar( "KP", "Configuration", &KP_value, KP_MAX, on_trackbar);
	createTrackbar( "KI", "Configuration", &KI_value, KI_MAX, on_trackbar);
	createTrackbar( "KD", "Configuration", &KD_value, KD_MAX, on_trackbar);
	createTrackbar( "Limites", "Configuration", &limites_value, limites_max, on_trackbar);
	createTrackbar( "Servo1_offset", "Configuration", &servo1_offset_value, servo1_offset_max, on_trackbar);
	createTrackbar( "Servo2_offset", "Configuration", &servo2_offset_value, servo2_offset_max, on_trackbar);
	createTrackbar( "Servo3_offset", "Configuration", &servo3_offset_value, servo3_offset_max, on_trackbar);
}

inline void plotPos(Ball_t b, Mat &frame, uint16_t x, uint16_t y){
	putText(frame, "X", cv::Point(x+5, y-120), 1, 1, DARK_GREEN , 2);
	putText(frame, "Y", cv::Point(x+5, y+80), 1, 1, DARK_GREEN , 2);
	line(frame, cv::Point(x, y-100), cv::Point(x+200, y-100), CYAN, 2);
	line(frame, cv::Point(x, y+100), cv::Point(x+200, y+100), CYAN, 2);

	for(int i=1 ; i<8 ; i++){
		line(frame, cv::Point(x+i*10, (y-100+(b.x[i-1]-SETPOINT_X)/3)),
			cv::Point(x+i*20, (y-100+(b.x[i]-SETPOINT_X)/3)), DARK_GREEN, 2);
		line(frame, cv::Point(x+i*10, (y+100+(b.y[i-1]-SETPOINT_Y)/3)),
			cv::Point(x+i*20, (y+100+(b.y[i]-SETPOINT_Y)/3)), DARK_GREEN, 2);
	}
}
void drawObjectV2(Ball_t ball, Mat &frame, bool noise_error){

	if(noise_error){	//////////////////////////////////////
		rectangle(	frame,
					cv::Point(SETPOINT_X-202, SETPOINT_Y-202),
					cv::Point(SETPOINT_X+202, SETPOINT_Y+202),
					RED, 3, LINE_8, 0);
		putText(frame, "TOO MUCH NOISE! ADJUST FILTERS", cv::Point(180,220), 1, 1, RED , 2);
		return;
	}

	if (ball.detected){	//////////////////////////////////////
		//draw square area
		rectangle(	frame,
					cv::Point(SETPOINT_X-CONTROL_AREA/2-1, SETPOINT_Y-CONTROL_AREA/2-1),
					cv::Point(SETPOINT_X+CONTROL_AREA/2+1, SETPOINT_Y+CONTROL_AREA/2+1),
					CYAN, 3, LINE_8, 0);
		putText(frame, "BALL FOUND", cv::Point(123,32), 1, 1, GREEN, 2);

		//draw setpoint area
		circle(frame, cv::Point(SETPOINT_X,SETPOINT_Y), 2, CYAN, 3);
		rectangle(	frame,
					cv::Point(SETPOINT_X-30, SETPOINT_Y-30),
					cv::Point(SETPOINT_X+30, SETPOINT_Y+30),
					ORANGE, 1, LINE_4, 0);


		//draw ball lines for position spot
		rectangle(	frame,
					cv::Point(ball.x[0]-28, ball.y[0]-28),
					cv::Point(ball.x[0]+28, ball.y[0]+28),
					GREEN, 1, LINE_8, 0);

		//draw velocity arrow
		arrowedLine(frame, 	cv::Point(ball.x[0] , ball.y[0]),
							cv::Point(ball.x[0]+ball.smooth_dx, ball.y[0]+ball.smooth_dy),
							RED, 2, 8, 0 , 0.4);

		//draw previous positions
		for (int i=1 ; i<8 ; i++){
			circle( frame, cv::Point(ball.x[i], ball.y[i]), 2, ORANGE, -1, 8, 0 );

		
		plotPos(ball, frame, 522, FRAME_HEIGHT/2);

		}
		//display ball info
		putText(frame,intToString(ball.y[0]),cv::Point(ball.x[0]+2,ball.y[0]-42),1,1,BLUE,2);
		line(frame, cv::Point(ball.x[0], 0), cv::Point(ball.x[0], FRAME_HEIGHT), BLUE, 1);
		line(frame, cv::Point(0, ball.y[0]), cv::Point(FRAME_WIDTH, ball.y[0]), BLUE, 1);
		putText(frame,intToString(ball.y[0]),cv::Point(ball.x[0]+2,ball.y[0]-42),1,1,BLUE,2);
		putText(frame,intToString(ball.x[0]),cv::Point(ball.x[0]+40,ball.y[0]+14),1,1,BLUE,2);

		return;
	}

	//////////////////////////////////////
	//draw square area
	rectangle(	frame,
		cv::Point(SETPOINT_X-202, SETPOINT_Y-202),
		cv::Point(SETPOINT_X+202, SETPOINT_Y+202),
		ORANGE, 3, LINE_8, 0);

	putText(frame, "BALL NOT FOUND", cv::Point(123,32), 1, 1, RED , 2);
	return;

}
void morphOps(Mat &thresh){

	//the kernel chosen here is a 5px by 5px square
	cv::Mat erodeElement = getStructuringElement(cv::MORPH_RECT, Size(5,5));

    //dilate with larger element so make sure object is nicely visible
	cv::Mat dilateElement = getStructuringElement(cv::MORPH_RECT, Size(5,5));

	cv::erode(thresh,thresh,erodeElement);
	cv::erode(thresh,thresh,erodeElement);

	cv::dilate(thresh,thresh,dilateElement);
	cv::dilate(thresh,thresh,dilateElement);

	erodeElement.release();
	dilateElement.release();
}

void trackFilteredObject(Ball_t* ball, Mat threshold){
	vector< vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	float refArea = 0;

	//find contours of filtered image using openCV findContours function
	cv::findContours(threshold,contours,hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noise error
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
				cv::Moments moment = cv::moments((cv::Mat)contours[index]);
				float area = moment.m00;

                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					if(ball->detected){
						//circleDetector(cameraFeed, threshold);
						updateBall(ball,(moment.m10/area), 
										(moment.m01/area));
						refArea = area;
					}
					else {
						*ball = createBall(	(moment.m10/area)+(FRAME_WIDTH-CONTROL_AREA)/2,
											(moment.m01/area)+(FRAME_HEIGHT-CONTROL_AREA)/2);
						ball->detected = true;
					}
				}
			}
		}
		
		else{
			//noise_error = true;
			ball->detected = false;
		}
	}
	else{
		ball->detected = false;
	}
}

void circleDetector(Mat cameraFeed, Mat threshold){

	Mat gray;
	threshold.copyTo(gray);
	GaussianBlur( gray, gray, Size(7, 7), 1.8, 1.8 );

	vector<Vec3f> circles;
	HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 35, 100, 25, 10, 45);

	for(size_t i = 0 ; i < circles.size() ; i++){
		printf("%d    radius: %.1f \n", (int)i+1, circles[i][2]);

		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		short radius = cvRound(circles[i][2]);

		circle( cameraFeed, center, 3, ORANGE, -1, 8, 0 );     // circle center
		circle( cameraFeed, center, radius+1, GREEN, 2, 8, 0 );  // circle outline
  	}
	gray.release();
}

void drawLiveData(Mat &DATA, PID_t XPID, PID_t YPID){
	putText(DATA, intToString(XPID.output[0]), cv::Point(5,25), 1, 2, BLUE, 2);
	putText(DATA, intToString(YPID.output[0]), cv::Point(205,25), 1, 2, BLUE, 2);
	//...
}

String createStringFromServ(const Serv& serv) {
    String dados =  to_string(serv.ang1)+"/" +to_string(serv.ang2)+"/"+to_string(serv.ang3-2)+"\n";
    //cout << dados <<endl;
    return dados;
}

float getValuesInRange(float begin, float end, float value){
	value = begin + (end-begin)*value/100*1.0;
	cout<< value<< endl;
	return value; 
}

void updatePID(PID_t* pidx, PID_t* pidy){
	float k1 = KDy/KDx;
	float k2 = KIy/KIx;
	pidx->Ki = KIx*getValuesInRange(0.0,1.5,KI_value); //1
	pidx->Kd = KDx*getValuesInRange(0.0,1.5,KD_value); //1
	pidy->Ki = k2*pidx->Ki;
	pidy->Kd = k1*pidx->Kd;
}

float* getNextPosition(int& pos, vector<pair<float,float>> positions){
	float* newPair = new float[2];
	
	pos = (pos+1)%positions.size();
	newPair[0] = positions[pos].first;
	newPair[1] = positions[pos].second;
		
	return newPair;
}

void servidor(httpServer& server){
    if (server.start()) {
        server.run(pattern);
    }
}
vector<pair<float, float>> generateSquarePatter(int size){
	vector<pair<float, float>> vec;
	vec.push_back(make_pair(-size,size));
	vec.push_back(make_pair(size,size));
	vec.push_back(make_pair(size,-size));
	vec.push_back(make_pair(-size,-size));
	return vec;
}

vector<pair<float, float>> generateTrianglePattern(int G){
	vector<pair<float, float>> vec;
  	vec.push_back(make_pair(0,G));
	vec.push_back(make_pair(-sqrt(3)*G/2,-G/2));
	vec.push_back(make_pair(sqrt(3)*G/2,-G/2));
	return vec;
}

vector<pair<float, float>> generateLimniscatePattern(float r){
	vector<pair<float, float>> vec;
  	float start = 0;
 	double theta;
  	double scale;
  
	theta = start;
	for (double j = 0; j < 2 * PI; j += 0.05) {
	  	scale = r * (2 / (3 - cos(2 * theta))); 
		vec.push_back(make_pair(scale * cos(theta), scale * sin(2 * theta) / 1.5));  
	  	theta += start == 0 ? -0.05 : (start == -2 * PI ? 0.05 : 0);
	}
  
	return vec;
}


void generatePatterns(map<string, vector<pair<float, float>>>& mapPatterns, map<string,int>& mapTemp){
	vector<pair<float,float>> vecCenter = {make_pair(0.0,0.0)};
	mapPatterns["center"] = vecCenter;
	mapPatterns["square"] = generateSquarePatter(3);
	mapPatterns["triangle"] = generateTrianglePattern(4);
	mapPatterns["limniscate"] = generateLimniscatePattern(5);
	mapTemp["center"] = 60;
	mapTemp["square"] = 60;
	mapTemp["triangle"] = 60;
	mapTemp["limniscate"] = 1;
}


int main() {
	Mat frame, HSV, threshold;
    Ball_t ball;
    ball.detected = false;
	int posIndex = 0;
	cv::Point windowPos;
	int cyclesWithoutBall = 0;
	int formatTemp = 0;
	float *pos;
	pos[0] = 0;
	pos[1] = 0; 
	map<string, vector<pair<float, float>>> mapPatterns;
	map<string,int> mapTemp;
	//////////////////////// generate patterns ////////////////////////////////////////////////////////////

	generatePatterns(mapPatterns, mapTemp);

    ////////////////////////   tracker   //////////////////////////////////////////////////////////////////
    if (!cap.isOpened()) { 
        cerr << "Erro ao abrir a câmera!" << endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    // Criação das janelas
    namedWindow(windowName);
    namedWindow(windowName2);

    createTrackbars();

    
    if (getWindowPos(&windowPos, frame) != 0) {
        cerr << "Erro ao obter a posição da janela!" << endl;
        return -1;
    }
    moveWindow(windowName, windowPos.x, windowPos.y);
    moveWindow(windowName2, windowPos.x + frame.cols, windowPos.y);
    
	//////////////////////    server    //////////////////////////////////////////////////////////////
    httpServer server(80);
    thread minhaThread(servidor,ref(server));
	const char* ip = "192.168.185.78"; // Substitua pelo IP do ESP32-CAM

    UDPSender sender(ip, 12345);

    //////////////////////     PID     //////////////////////////////////////////////////////////////
    PID_t pidX = createPID(KPx,KIx,KDx,SETPOINT_X, false, X_MIN_ANGLE, X_MAX_ANGLE);
    PID_t pidY = createPID(KPy,KIy,KDy,SETPOINT_Y, false, Y_MIN_ANGLE, Y_MAX_ANGLE);
	Machine machine(8.0, 8.0, 4.0, 5.5);

    //////////////////////     Loop principal     ///////////////////////////////////////////////////////
    while (true) {
        cap >> frame; 

        if (frame.empty()) {
            cerr << "Frame vazio!" << endl;
            break;
        }

        cvtColor(frame, HSV, COLOR_BGR2HSV);

        inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
        morphOps(threshold);
        trackFilteredObject(&ball, threshold);

        drawObjectV2(ball, frame, false);

        imshow(windowName, frame);
        imshow(windowName2, threshold);

        if(ball.detected){
			cyclesWithoutBall = 0;
			formatTemp++;
			if (formatTemp == mapTemp[pattern]){
				pos = getNextPosition(ref(posIndex), mapPatterns[pattern]);
				formatTemp = 0;
			}
			updatePID(&pidX,&pidY);

            Serv ang = PIDCompute(&pidX, &pidY, ball,pos[0], pos[1], limites_value,machine);
			ang.ang1 += servo1_offset_value;
			ang.ang2 += servo2_offset_value;
			ang.ang3 += servo3_offset_value;

			string message;
            message = createStringFromServ(ang);
			sender.sendString(message);
        }
		else{
			formatTemp = 0;
			pos[0] = 0;
			pos[1] = 0; 
			if (cyclesWithoutBall < 10){
				cyclesWithoutBall++;
			}
			if (cyclesWithoutBall == 10) {
				cyclesWithoutBall++;
				Serv ang;
				pidX.integral = 0;
				pidY.integral = 0;
				ang.ang1= ANGLE_ORIGIN+servo1_offset_value;
				ang.ang2 = ANGLE_ORIGIN+servo2_offset_value;
				ang.ang3 = ANGLE_ORIGIN+servo3_offset_value;
				string message;
				message = createStringFromServ(ang);
				sender.sendString(message);
			}
		}
        
        if (waitKey(30) == 'q') {
            break;
        }
    }
	server.stopServer();
    minhaThread.join();
    destroyAllWindows();

    return 0;
}

