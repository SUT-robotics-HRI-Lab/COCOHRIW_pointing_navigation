#include "Roomba.h"
#define SPI 80
#define ROTATE_SLEEP 100

#define DOLADOVACKA 40

Roomba::Roomba(){
}

int Roomba::initRoomba( char portNum[], int baudrate ){
	bool ret = ConnectToPort( port, portNum, BAUDRATE_iROBOT );
	SentToCreate ( port, OI_START );
	usleep(1000* SPI );
	SentToCreate ( port, OI_FULL );
	usleep(1000* SPI );
	return ret;
}

void Roomba::drive(int left, int right){
	SentToCreate ( port, OI_DRIVE_DIRECT, (uint16_t) left, (uint16_t) right );
}

short Roomba::driveDistance(int left, int right, float distanceInMilimeters){

	CreateSensors sensors;
	short distance = 0;

    while ( distance < distanceInMilimeters ) //mm
    {
		if(distanceInMilimeters - distance > 300)
			SentToCreate ( port, OI_DRIVE_DIRECT, left, right );
		else if(distanceInMilimeters - distance > 200)
			SentToCreate ( port, OI_DRIVE_DIRECT, 120, 120 );
		else if(distanceInMilimeters - distance > 100)
			SentToCreate ( port, OI_DRIVE_DIRECT, 80, 80 );
		else 
			SentToCreate ( port, OI_DRIVE_DIRECT, 40, 40 );

        usleep(1000* SPI );
        SentToCreate (port, OI_SENSORS, ( unsigned char ) 0x06 );
		usleep(1000* SPI );
		ReceivePacketFromCreate(port, sensors, 0x06);
        distance += sensors.Distance;
        //printf ( "roomba distance %d, general distance %d\n", roomba.roombaSensors.Distance, distance );
    }

	//usleep(1000* SPI );
	SentToCreate ( port, OI_DRIVE_DIRECT, (uint16_t) 0, (uint16_t) 0 );
	return distance;
}


int Roomba::readAngle(){
	CreateSensors sensors;
	SentToCreate (port, OI_SENSORS, ( unsigned char ) 0x06 );
	usleep(1000* SPI );
	ReceivePacketFromCreate(port, sensors, 0x06);
	usleep(1000* SPI );
	return sensors.Angle;
}

/* stara rozhegana rotacia, vraj rychlosti pod 50 su velmi nespolahlive... 
int Roomba::rotateAngle( int angle ){
	//myso "doladenie" oneskorenia na tvrdo :)
	if(angle > 0)
		angle = angle-1;
	else
		angle = angle+2;

	int actual = 0;
	int errorNum = 0;
	CreateSensors sensors;

	if( angle > 0 ){


		while(actual <= angle){
			usleep(1000* SPI );
			if((angle - actual) > 15)
				SentToCreate ( port, OI_DRIVE_DIRECT, 50 + DOLADOVACKA, -50 - DOLADOVACKA);
			else if((angle - actual) > 5)
				SentToCreate ( port, OI_DRIVE_DIRECT, 35 + DOLADOVACKA, -35 - DOLADOVACKA);
			else 
				SentToCreate ( port, OI_DRIVE_DIRECT, 20 + DOLADOVACKA, -20 - DOLADOVACKA);

			usleep(1000* ROTATE_SLEEP );
			SentToCreate (port, OI_SENSORS, ( unsigned char ) 0x06 );
			usleep(1000* ROTATE_SLEEP );
			if( -1 == ReceivePacketFromCreate(port, sensors, 0x06))
				errorNum++;
			else
				actual += sensors.Angle;
			
		}

	}else if( angle < 0 ){


		while(actual >= angle){

			usleep(1000* SPI );
			if((-angle + actual) > 40)
				SentToCreate ( port, OI_DRIVE_DIRECT, -50 - DOLADOVACKA, 50 + DOLADOVACKA);
			else if((-angle + actual) > 5)
				SentToCreate ( port, OI_DRIVE_DIRECT, -35 - DOLADOVACKA, 35 + DOLADOVACKA);
			else 
				SentToCreate ( port, OI_DRIVE_DIRECT, -20 - DOLADOVACKA, 20 + DOLADOVACKA );
			usleep(1000* ROTATE_SLEEP );
			SentToCreate (port, OI_SENSORS, ( unsigned char ) 0x06 );
			usleep(1000* ROTATE_SLEEP );
			
			if( -1 == ReceivePacketFromCreate(port, sensors, 0x06))
				errorNum++;
			else
				actual += sensors.Angle;
		}
	}

	//usleep(1000* SPI );
	SentToCreate ( port, OI_DRIVE_DIRECT, (uint16_t) 0, (uint16_t) 0 );
	usleep(1000* SPI );
	return actual;
}*/

#define NEW_ROT_SPEED 50

int Roomba::rotateAngle( int angle ){
	//myso "doladenie" oneskorenia na tvrdo :)
	if(angle > 0)
		angle = angle-1;
	else
		angle = angle+2;

	float avg = 0;
	int avgCount = 0;

	int actual = 0;
	int errorNum = 0;
	CreateSensors sensors;

	if( angle > 0 ){


		while(actual <= angle){
			//usleep(1000* SPI );
			/*if((angle - actual) > 15)
				SentToCreate ( port, OI_DRIVE_DIRECT, NEW_ROT_SPEED, -NEW_ROT_SPEED);
			else if((angle - actual) > 5)
				SentToCreate ( port, OI_DRIVE_DIRECT, NEW_ROT_SPEED - 50, -NEW_ROT_SPEED + 50);
			else
				SentToCreate ( port, OI_DRIVE_DIRECT, NEW_ROT_SPEED - 100, -NEW_ROT_SPEED + 100);*/

			SentToCreate ( port, OI_DRIVE_DIRECT, NEW_ROT_SPEED , -NEW_ROT_SPEED);

			usleep(1000* ROTATE_SLEEP - 20);
			SentToCreate (port, OI_SENSORS, ( unsigned char ) 0x06 );
			usleep(1000* ROTATE_SLEEP );
			if( -1 == ReceivePacketFromCreate(port, sensors, 0x06))
				errorNum++;
			else
			{
				actual += sensors.Angle;
				avgCount++;
			}
		}

	}else if( angle < 0 ){


		while(actual >= angle){

			//usleep(1000* SPI );
/*			if((-angle + actual) > 15)
				SentToCreate ( port, OI_DRIVE_DIRECT, -NEW_ROT_SPEED, NEW_ROT_SPEED);
			else if((-angle + actual) > 5)
				SentToCreate ( port, OI_DRIVE_DIRECT, -NEW_ROT_SPEED + 50, NEW_ROT_SPEED - 50);
			else 
				SentToCreate ( port, OI_DRIVE_DIRECT, -NEW_ROT_SPEED + 100, NEW_ROT_SPEED - 100);*/
			SentToCreate ( port, OI_DRIVE_DIRECT, -NEW_ROT_SPEED, NEW_ROT_SPEED);
			usleep(1000* ROTATE_SLEEP - 20);
			SentToCreate (port, OI_SENSORS, ( unsigned char ) 0x06 );
			usleep(1000* ROTATE_SLEEP );
			
			if( -1 == ReceivePacketFromCreate(port, sensors, 0x06))
				errorNum++;
			else
			{
				actual += sensors.Angle;
				avgCount++;
			}
		}
	}

	//usleep(1000* SPI );
        uint16_t rychlost=0;
	SentToCreate ( port, OI_DRIVE_DIRECT, rychlost, rychlost );
	usleep(1000* SPI );

	avg = (float)actual/avgCount;
	return actual;
}


int Roomba::rotateAngle(int angle, float* jozoImuDegPointer)
{
	//::AllocConsole();
	//myso "doladenie" oneskorenia na tvrdo :)
	/*if(angle > 0)
		angle = angle-1;
	else
		angle = angle+2;*/

	int actualTotal = 0;
	int actualTotalRoomba = 0;
	int errorNum = 0;
	CreateSensors sensors;
	
	//float initialIMUdegVal = *jozoImuDegPointer;
	float previousIMU = *jozoImuDegPointer;
	float actualIMU = *jozoImuDegPointer;
	
	if( angle > 0 ){//rotacie dolava (Create rastie dolava)


		while(actualTotal <= angle){
			
	
			actualIMU=(actualIMU>360 ? actualIMU-360 :actualIMU);
			actualTotal=actualTotal-actualIMU+previousIMU;
		//	_cprintf("uhol %f\n",actualIMU);
			previousIMU = actualIMU;
			//kedze Create ma rast opacne ako IMU, musim to vynasovbit -1
		/*	if(previousIMU > actualIMU)
				actualTotal += -1*(actualIMU - previousIMU);
			else
				actualTotal +=  -1*((actualIMU +  360) - previousIMU);
*/
			usleep( SPI*1000 );
			if((angle - actualTotal) > 15)
				SentToCreate ( port, OI_DRIVE_DIRECT, 50 + DOLADOVACKA, -50 - DOLADOVACKA);
			else if((angle - actualTotal) > 5)
				SentToCreate ( port, OI_DRIVE_DIRECT, 35 + DOLADOVACKA, -35 - DOLADOVACKA);
			else 
				SentToCreate ( port, OI_DRIVE_DIRECT, 20 + DOLADOVACKA, -20 - DOLADOVACKA);

			usleep(1000* ROTATE_SLEEP );
			SentToCreate (port, OI_SENSORS, ( unsigned char ) 0x06 );
			usleep(1000* ROTATE_SLEEP );
			if( -1 == ReceivePacketFromCreate(port, sensors, 0x06))
				errorNum++;
			else
				actualTotalRoomba += sensors.Angle;
			
			actualIMU = *jozoImuDegPointer; 
			
		}

	}else if( angle < 0 ){//rotacie doprava (Create klesa doprava)


		while(actualIMU >= angle){

			usleep(1000* SPI );
			if((-angle + actualTotal) > 40)
				SentToCreate ( port, OI_DRIVE_DIRECT, -50 - DOLADOVACKA, 50 + DOLADOVACKA);
			else if((-angle + actualTotal) > 5)
				SentToCreate ( port, OI_DRIVE_DIRECT, -35 - DOLADOVACKA, 35 + DOLADOVACKA);
			else 
				SentToCreate ( port, OI_DRIVE_DIRECT, -20 - DOLADOVACKA, 20 + DOLADOVACKA );
			usleep(1000* ROTATE_SLEEP );
			SentToCreate (port, OI_SENSORS, ( unsigned char ) 0x06 );
			usleep(1000* ROTATE_SLEEP );
			
			if( -1 == ReceivePacketFromCreate(port, sensors, 0x06))
				errorNum++;
			else
				actualTotalRoomba += sensors.Angle;

			//kedze Create ma rast opacne ako IMU, musim to vynasovbit -1
			actualIMU = *jozoImuDegPointer; 
			if(previousIMU > actualIMU)
				actualTotal -= -1*(previousIMU - actualIMU);
			else
				actualTotal -= -1*((previousIMU + 360) - actualIMU);

			
			previousIMU = actualIMU;
		}
	}

	//usleep(1000* SPI );
        uint16_t rychlost=0;
	SentToCreate ( port, OI_DRIVE_DIRECT,rychlost, rychlost );
	usleep(1000* SPI );
	return actualTotal;
}