/**
* @file serial.c
* @author Giuseppe Sensolini [https://github.com/JiuSenso/Ball-Balancing-PID-System.git]
*
* @brief SERIAL COMMUNICATION
* 		- open serial communication
*		- close serial communication
*		- packet encode/decode
*		- debug print
*
* @date 2019-01-11
* 
* @copyright Copyright (c) 2019
* 
*/


/********************************************
*	@Author: Giuseppe Sensolini Arra'		*
*	@Date: 01.2019							*
*											*
*	SERIAL COMMUNICATION MODULE				*
*	task:									*
*		- open serial communication			*
*		- close serial communication		*
*		- handshake routine					*
*		- encode/decode packs				*
*		- write/read data					*
*		- debug print 						*
*											*
*********************************************/

#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>

#include "serial.h"

const char* serialPorts[5]= {	ttyACM0,
								ttyACM1,
								ttyACM2,
								ttyACM3,
								ttyACM4
							};
/*=============================================================*/

/**
 * @brief find serial device and open
 * 
 * @param fd file descriptor pointer
 * @return ttyACM* device number, -1 if no device found
 */
int openSerialCommunication(int* fd){
	printf("\nSearch for AVR device on serial ports...\n");
	int k = 0;
	for ( ; k<5 ; k++){
		*fd = open(serialPorts[k], O_RDWR | O_NOCTTY | O_NDELAY);
		if (*fd >= 0){
			printf(" /dev/ttyACM%d found.\n", k);
			return k;
		}
		else { 
			printf("# /dev/ttyACM%d not found\n", k);
		}
	}
	return -1;
}


/**
 * @brief Set serial port attributes using termios structure
 * 
 * @param fd file descriptor
 */
void setSerialAttributes(int fd){
	struct termios SerialPortSettings;	/* Create the structure                          */

	tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

	cfsetispeed(&SerialPortSettings,B9600); /* Set Read  Speed as 9600                       */
	cfsetospeed(&SerialPortSettings,B9600); /* Set Write Speed as 9600                       */

	SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity*/
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits, here it is cleared so 1 Stop bit*/
	SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

	SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

	SerialPortSettings.c_oflag &= ~OPOST;		/*No Output Processing*/

	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0){
		printf("\n ERROR! cannot set serial attributes\n\n");
		exit(EXIT_FAILURE);
	}
	else{
		printf("\n  | BaudRate = %d \n  | StopBits = 1 \n  | Parity   = none\n\n", BAUD_RATE);
	}
}

/**
 * @brief Close serial communication
 * 
 * @param fd file descriptor
 * @param config servo config
 */
void closeSerialCommunication(int* fd, ServoConfig_t* config){

	uint8_t buf[5];
	config->xPulse = X_HALF_ANGLE;
	config->yPulse = Y_HALF_ANGLE;
	encodeConfig(config, buf);

	int ret = write(*fd, (void*)buf, sizeof(buf));
	if( ret != (int)(sizeof(buf)/sizeof(uint8_t)) ){
		printf("\n  -- %d bytes exprected but %d found\n",
		(int)(sizeof(buf)/sizeof(uint8_t)), ret);
	}

	sleep(2); //required to make flush work
	tcflush(*fd, TCIOFLUSH);
	ret = close(*fd);
	if(ret != 0) {
		printf("\n  -- close(fd) syscall failed [%d]\n", ret);
		exit(EXIT_FAILURE);
	}
}

/**
 * @brief ServoConfig_t* ==> uint8_t*
 * 	encode 16 bytes structure to 8 bytes buffer
 * @param config 16 bytes structure
 * @param buf 8 bytes buffer
 */
// SERVOCONFIG_T* ===> UINT8_T*
void encodeConfig(ServoConfig_t* config, uint8_t* buf){
	buf[0] = (config->xPulse) & 0xFF;	//low bits
	buf[1] = (config->xPulse) >> 8;		//high bits
	buf[2] = (config->yPulse) & 0xFF;	//low bits
	buf[3] = (config->yPulse) >> 8;		//high bits
	buf[4] = '\n';
}


/**
 * @brief Handshake Routine: actually not used [to be fixed]
 * 
 */
/*
bool handShake(int* fd){	//to be tested
	printf("\n HANDSHACKING: ");
	int i, j, bytes_written, bytes_read;
	uint8_t write_buf[5], read_buf[5];

	for (i=1; i<6; i++){
		for(j=0; j<4; j++) write_buf[j] = 7*j*i+7;
		write_buf[4] = '\n';

		bytes_written = write(*fd, write_buf, sizeof(write_buf));
		usleep(150000);
		printf("=");

		bytes_read = read(*fd, read_buf, sizeof(read_buf));
		usleep(150000);
		printf("=");

		for(j=0; j<5; j++) if(write_buf[j] != read_buf[j]) return false;
	}
	bytes_written = write(*fd, (void*)"done\n", sizeof(write_buf));
	printf(" Done.\n\n");
	usleep(250000);
	return true;
}
*/

/**
 * @brief debug print
 * 
 * @param config ServoConfig_t
 */
void printServoConfig(ServoConfig_t config){
	printf("\n   ======================================\n");
	printf("  |  xPulse: %d   ||   yPulse: %d  |\n", config.xPulse, config.yPulse);
	printf("   ======================================\n");
}

/**
 * @brief debug print
 * 
 * @param buf 
 */
void printEncodedPack(uint8_t* buf){
	printf("\n   =========================================\n  |");
	printf(" 0x%02X  |", buf[0]);
	printf(" 0x%02X  ||", buf[1]);
	printf(" 0x%02X  |", buf[2]);
	printf(" 0x%02X  ||", buf[3]);
	printf(" 0x%02X  |", buf[4]);
	printf("\n   =========================================\n");
}
