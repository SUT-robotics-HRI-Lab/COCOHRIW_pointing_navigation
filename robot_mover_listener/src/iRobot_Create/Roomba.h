#ifndef ROOMBA_H
#define ROOMBA_H

//#include <Windows.h>
#include "iCreateDef.h"
 #include <unistd.h>
#define BAUDRATE_iROBOT 57600

//	zosilnenia PID regulatora a tolerancia pre iRobot Create

#define P_iROBOT 2
#define I_iROBOT 0
#define D_iROBOT 0
#define TOL_iROBOT 10

class Roomba{
	int port;
public:
	Roomba();
	int initRoomba(char[], int baudrate = BAUDRATE_iROBOT);
	int rotateAngle(int);
	//toto je verzia, kde beriem do uvahy aj IMU data
	int rotateAngle(int, float* jozoImuDegPointer);
	void drive(int, int);
	short driveDistance(int left, int right, float distanceInMilimeters);
	int readAngle();

	/*sendToCreate ( unsigned char OI_code, unsigned char data )
	{
		SentToCreate(port, OI_SENSORS, data);
	}
	
	receivePacketFromCreate ( 0x06 )
	{
		ReceivePacketFromCreate(port, sensors, 0x06);
	}
	
	roomba.sendToCreate ( unsigned char OI_code, WORD data1, WORD data2 )
	{
		SentToCreate(port, OI_DRIVE_DIRECT , data1, data2);
	}*/
};

#endif