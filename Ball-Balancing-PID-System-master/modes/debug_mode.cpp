/**
 * @file debug_mode.cpp
 * @author Giuseppe Sensolini
 * @brief DEDBUF MODE
 * @version 0.1
 * @date 2019-03-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "modes.h"
#include "../utils.h"


//=============================================================================
//:::::::::::::: DEBUG :::::::::::::::::::::::::::::::::::::::::::::::::::::
//=============================================================================
int debug_mode(){
	
	const char* pid_data_file_name = "settings/pid_data.txt";
	const char* hsv_data_file_name = "settings/hsv_data.txt";

	int fd = -1;	//file descriptor
	int ret __attribute__((unused)); /*for unused variables suppression*/

//===== SETUP OPENCV DATA STRUCTURES ==========================================
	Mat MATS[3] = {	cv::Mat(FRAME_HEIGHT,FRAME_WIDTH, CV_8UC3) , 
					cv::Mat(CONTROL_AREA, CONTROL_AREA, CV_8UC3),
					cv::Mat(CONTROL_AREA, CONTROL_AREA, CV_8UC3),
				  };	// Mat Array = [ webcam | threshol | HSV ]
	
	cv::Mat GUI(FRAME_HEIGHT, FRAME_WIDTH+CONTROL_AREA, CV_8UC3, cv::Scalar(80,70,50));
	cv::Mat TOOL(FRAME_HEIGHT-CONTROL_AREA, CONTROL_AREA, CV_8UC3, cv::Scalar(80,70,50));
	cv::Rect controlROI(SETPOINT_X-CONTROL_AREA/2,SETPOINT_Y-CONTROL_AREA/2, 
						CONTROL_AREA, CONTROL_AREA);
	
//_ Open camera stream ____________________________
	cv::VideoCapture capture;
	int CAM_NUMBER = 0;
	for ( ; CAM_NUMBER<3 ; CAM_NUMBER++){
		capture.open(CAM_NUMBER);
		if ( capture.isOpened() ){
			printf("# /dev/video%d successfully opened\n", CAM_NUMBER);
			break;
		}
	}
	if ( CAM_NUMBER == 3 ){
		perror("ERROR: NO dev/video* DEVICE CONNECTED");
		exit(EXIT_FAILURE);
	}

//===== INITIALIZE SERIAL COMMUNICATION =======================================
	
	int device_opened = openSerialCommunication(&fd);
	if( device_opened >= 0 ){
		setSerialAttributes(fd);
		printf("# %s successfully opened\n", serialPorts[device_opened]);
	}
	else{
		perror("\nERROR: NO /dev/ttyACM* DEVICE CONNECTED");
		exit(EXIT_FAILURE);
	}

	int bytes_written = 0;
	uint8_t buf[5];
	memset(buf, 0, sizeof(buf));
	printf("# write_buffer allocated\n");

//===== SETUP DATA STRUCTURES =================================================
	
//_ Ball setup __________________________________
	printf("\n# create Ball object... ");
	Ball_t ball = createBall(FRAME_WIDTH/2, FRAME_HEIGHT/2);
	printf("Done.\n");
	printBall(ball);

//_ PID setup ___________________________________
	printf("\n# create PID objects... ");
	float x_gains[3], y_gains[3];
	arrayFromTextFile(pid_data_file_name, x_gains, 1);
	arrayFromTextFile(pid_data_file_name, y_gains, 2);

	PID_t XPID = createPID(	x_gains[0], x_gains[1], x_gains[2],
							FRAME_WIDTH/2, true, X_MIN_ANGLE, X_MAX_ANGLE);
	PID_t YPID = createPID(	y_gains[0], y_gains[1], y_gains[2],
							FRAME_HEIGHT/2, false, Y_MIN_ANGLE, Y_MAX_ANGLE);
	printf("Done. \n");
	printPID(XPID);
	printPID(YPID);

//_ Initialize servo config___________________________
	ServoConfig_t config = {
		.xPulse = X_HALF_ANGLE,
		.yPulse = Y_HALF_ANGLE
	};
	printServoConfig(config);

//_ Read HSV mask ____________________________________
	float min_hsv[3], max_hsv[3];
	arrayFromTextFile(hsv_data_file_name, min_hsv, 1);
	arrayFromTextFile(hsv_data_file_name, max_hsv, 2);
	H_MIN = (int)min_hsv[0];
	S_MIN = (int)min_hsv[1];
	V_MIN = (int)min_hsv[2];
	H_MAX = (int)max_hsv[0];
	S_MAX = (int)max_hsv[1];
	V_MAX = (int)max_hsv[2];

//_ time variables____________________________________
	//clock_t start, end;	////This timer can be used to change dt every iteration
	clock_t total_start, total_end;
	int frame_counter = 0;
	double tot;

//_ center window based on screen resolution (at least 1280p)
	cv::Point window_pos = cv::Point(1,1);
	if( getWindowPos(&window_pos, GUI) != 0 ){
		exit(EXIT_FAILURE);
	}
	else cv::moveWindow(windowName, window_pos.x, window_pos.y-100);

//======================================================================+++++++===

//_ Wait for user input... ___________________________
	char tmp;
	printf("\n # All parameters setted, ready to go...\n\
			\n -> Press enter to start <- \n");
	ret = scanf("%c", &tmp);

//_ Handshake with avr________________________________
	/*  [not a real handshake, this is needed to setup the avr)] */
	printf("# Handshake... ");
	encodeConfig(&config, buf);
	bytes_written = write(fd,(void*)buf, sizeof(buf));
	encodeConfig(&config, buf);
	bytes_written = write(fd,(void*)buf, sizeof(buf));
	printf("Done. \n");

	total_start = clock();

//=============================================================================
//:::::::::::::: MAIN LOOP ::::::::::::::::::::::::::::::::::::::::::::::::::::
//=============================================================================
		
	while(true){
	//start = clock();

		capture.read(MATS[0]);	//store image to matrix
		cvtColor(MATS[0](controlROI), MATS[2], COLOR_BGR2HSV);
		inRange(	MATS[2],
						Scalar(H_MIN, S_MIN, V_MIN),
						Scalar(H_MAX, S_MAX, V_MAX),
						MATS[1]
					);			
		morphOps(MATS[1]);
		trackFilteredObject(&ball, MATS[1]);
		
		drawObjectV2(ball, MATS[0], false);
		cvtColor(MATS[1], MATS[1], COLOR_GRAY2BGR);
		cv::vconcat(MATS[1], TOOL, MATS[1]);
		cv::hconcat(MATS[1], MATS[0], GUI);

		cv::imshow(windowName, GUI);
		cv::imshow(windowName2, MATS[2]);

		if(ball.detected){
			//compute new servo pulses
			PIDCompute(&XPID, &YPID, ball);
			config.xPulse = XPID.output[0];
			config.yPulse = YPID.output[0];

			//encode and send to avr
			encodeConfig(&config, buf);	//Create Packet
			bytes_written = write(fd,(void*)buf, sizeof(buf)); //Send packet
			if(bytes_written != 5){
				perror("Error: write() syscall failed");
				exit(EXIT_FAILURE);
			}
		}
			
		printf("[%d , %d]\n", ball.x[0], ball.y[0]);
		//printf("output:  X = %d , Y = %d , dt = %.4lf\n",XPID.output[0], YPID.output[0], YPID.dt*9.1);

		//end = clock();
		//update dt based on frame rate
		//XPID.dt = YPID.dt = (float)(end - start)/CLOCKS_PER_SEC;

		frame_counter++;
		if(waitKey(1) >= 0) break;
	}
		
	
//=============================================================================
//::::::: EXIT ROUTINE ::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//=============================================================================

//__print frame rate info_______________________
	total_end = clock();
	tot = 7*((double)(total_end-total_start)/CLOCKS_PER_SEC);
	printf("\nTime: %d frames\n", frame_counter);
	printf("Time: %.3lf seconds\n", tot);
	printf("*******************\n AVERAGE FPS: %lf \n********************",
		(float)frame_counter/tot);

//__CLOSE EVERYTHING____________________________
	printf("\n========== EXIT PROTOCOL ========== \n\n");

	//destroy all windows
	printf("# Destroy all windows... ");
	cv::destroyWindow(windowName);
	cv::destroyWindow(windowName2);
	cv::destroyAllWindows();
	printf("Done.\n");

	//release VideoCapture
	printf("# Release cv::VideoCapture... ");
	capture.release();
	printf("Done.\n");

	//release Mat
	printf("# Release cv::Mat... ");
	MATS[0].release();
	MATS[1].release();
	MATS[2].release();
	TOOL.release();
	GUI.release();
	printf("Done.\n");

	//close serial
	printf("# Close serial communication... ");
	closeSerialCommunication(&fd, &config);
	printf("Done. \n");
	
	printf("\n===================================\n\n");

	return 0;
}