#include "iCreateDef.h"
#include "termios.h"
#include <unistd.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <stdio.h>
#include <string.h>
#include <sys/uio.h>
using namespace std;

int
set_interface_attribs (int fd, int speed, int parity)
{
	      struct termios tty;
		   memset (&tty, 0, sizeof tty);
		   if (tcgetattr (fd, &tty) != 0)
		   {
				   printf ("error %d from tcgetattr", errno);
				   return -1;
		   }

		   cfsetospeed (&tty, speed);
		   cfsetispeed (&tty, speed);

		   tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
		   // disable IGNBRK for mismatched speed tests; otherwise receive break
		   // as \000 chars
		   tty.c_iflag &= ~IGNBRK;         // disable break processing
		   tty.c_lflag = 0;                // no signaling chars, no echo,
										   // no canonical processing
		   tty.c_oflag = 0;                // no remapping, no delays
		   tty.c_cc[VMIN]  = 0;            // read doesn't block
		   tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

		   tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

		   tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
										   // enable reading
		   tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
		   tty.c_cflag |= parity;
		   tty.c_cflag &= ~CSTOPB;
		   tty.c_cflag &= ~CRTSCTS;

		   if (tcsetattr (fd, TCSANOW, &tty) != 0)
		   {
				   printf ("error %d from tcsetattr", errno);
				   return -1;
		   }
	return 0;
}

void
set_blocking (int fd, int should_block)
{
	     struct termios tty;
		 memset (&tty, 0, sizeof tty);
		 if (tcgetattr (fd, &tty) != 0)
		 {
				 printf ("error %d from tggetattr", errno);
				 return;
		 }

		 tty.c_cc[VMIN]  = should_block ? 1 : 0;
		 tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

		 if (tcsetattr (fd, TCSANOW, &tty) != 0)
				 printf ("error %d setting term attributes", errno);
}

int ConnectToPort(int &HCom,char *comportT,uint64_t BaudRate)
{
	HCom= open( comportT, O_RDWR|O_NOCTTY|O_SYNC );

	if ( HCom== -1 )
	{
          std::cout<<"neotvoril chod do pice "<<comportT<<std::endl;
		//CloseHandle(HCom);
		return -1;	
	}
	else
	{
		/*DCB PortDCB;
		PortDCB.DCBlength = sizeof(DCB);  // Inicializuj položku DCBlength
		GetCommState(HCom,&PortDCB);       // Načítaj aktuálne nastavenia
		PortDCB.BaudRate=BaudRate;
		PortDCB.ByteSize=8;
		PortDCB.Parity=0;
		SetCommState(HCom,&PortDCB);
		PurgeComm(HCom,PURGE_TXCLEAR | PURGE_RXCLEAR);
		COMMTIMEOUTS timeouts;

		timeouts.ReadIntervalTimeout         = MAXuint64_t;
		timeouts.ReadTotalTimeoutMultiplier  = 0;
		timeouts.ReadTotalTimeoutConstant    = 0;
		timeouts.WriteTotalTimeoutMultiplier = 0;
		timeouts.ReadTotalTimeoutConstant    = 0;

		SetCommTimeouts(HCom,&timeouts);*/

		set_interface_attribs (HCom, B57600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
		set_blocking (HCom, 0);                // set no blocking

		printf("mam port %i\n",HCom);
		return 0;
	}
}

int SentToCreate(int HCom,unsigned char OI_code)
{
	int Pocet=	write(HCom,&OI_code,1);
	return 0;
}

int SentToCreate(int HCom,unsigned char OI_code, unsigned char data)
{
	int Pocet;
	Pocet=write(HCom,&OI_code,1);
		
	Pocet=write(HCom,&data,1);
	return 0;
}
int SentToCreate(int HCom,unsigned char OI_code, uint16_t data)
{
	int Pocet;
	Pocet=write(HCom,&OI_code,1);
	Pocet=write(HCom,&data,2);
	return 0;
}
int SentToCreate(int HCom,unsigned char OI_code, uint16_t data1,uint16_t data2)
{
	int Pocet;
	write(HCom,&OI_code,1);
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1,pomoc2;
	pomoc1.data=data1;
	pomoc2.data=data2;
	write(HCom,&pomoc1.datab[1],1);
	write(HCom,&pomoc1.datab[0],1);
	//WriteFile(HCom,&data2,2,&Pocet,NULL);
	write(HCom,&pomoc2.datab[1],1);
	write(HCom,&pomoc2.datab[0],1L);
	return 0;
}
int SentToCreate(int HCom,unsigned char OI_code,int NumOfBytes, uint16_t data1,uint16_t data2)
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1,pomoc2;
int Pocet;
	pomoc1.data=data1;
	pomoc2.data=data2;
	switch(NumOfBytes)
	{
		case 0: SentToCreate(HCom,OI_code);
			break;
		case 1: SentToCreate(HCom,OI_code, pomoc1.datab[1]);
			break;
		case 2:SentToCreate(HCom,OI_code,data1);
			break;
case 3:SentToCreate(HCom,OI_code,data1);
write(HCom,&pomoc1.datab[1],1);
			break;
		case 4:SentToCreate(HCom,OI_code, data1,data2);
			break;
		default:
			break;
	}
	return 0;
}

int SentToCreate(int HCom,unsigned char OI_code,unsigned char NumOfBytes, unsigned char *data)
{
	int Pocet;
	write(HCom,&OI_code,1);
	write(HCom,&NumOfBytes,2);
	write(HCom,&data,NumOfBytes);
	return 0;

}

int ReceivePacketFromCreate(int HCom,CreateSensors &IO_SENSORS_CREATE,unsigned char packet)
{
	unsigned char *dataR;
	int pocet;
	dataR=(unsigned char*)malloc(100);
	pocet=read(HCom,dataR,100);
	if(pocet==OI_PacketSize[packet])
	{
	DecodeSensorsFromPacket(IO_SENSORS_CREATE,packet,dataR);
	return 0;
	}
	return -1;
}

int DecodeSensorsFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char packet,unsigned char *data)
{
	int i=0;
	int index=0;
	for(i=OI_PacketID[packet][0];i<=OI_PacketID[packet][1];i++)
	{
		switch(i)
		{
			case 7:
				DecodeBumsAndWheelsFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 8:
				DecodeWallFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 9:
				DecodeCliffLeftFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 10:
				DecodeCliffFrontLeftFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 11:
				DecodeCliffFrontRightFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 12:
				DecodeCliffRightFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 13:
				DecodeVirtualWallFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 14:
				DecodeLSDandWheelOvercurrentFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 15:
				index++;
				break;
			case 16:
				index++;
				break;
			case 17:
				DecodeIRFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 18:
				DecodeButtonsFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 19:
				DecodeDistanceFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 20:DecodeAngleFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 21:
				DecodeChargingStateFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 22:
				DecodeVoltageFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 23:
				DecodeCurrentFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 24:
				DecodeBatteryTemperatureFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 25:
				DecodeBatteryChargeFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 26:
				DecodeBatteryCapacityFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 27:
				DecodeWallSignalFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 28:
				DecodeCliffLeftSignalFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 29:
				DecodeCliffFrontLeftSignalFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 30:
				DecodeCliffFrontRightSignalFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 31:
				DecodeCliffRightSignalFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 32:
				DecodeCargobayDigitalInputFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 33:
				DecodeCargobayAnalogSignalFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 34:
				DecodeChargingSourceAvailableFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 35:
				DecodeOImodeFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 36:
				DecodeSongNumberFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 37:
				DecodeSongPlayingFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 38:
				DecodeNumberOfStreamPacketsFromPacket(IO_SENSORS_CREATE,*(data+index));
				index++;
				break;
			case 39:
				DecodeRequestedVelocityFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 40:
				DecodeRequestedRadiusFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 41:
				DecodeRequestedRightVelocityFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			case 42:
				DecodeRequestedLeftVelocityFromPacket(IO_SENSORS_CREATE,(data+index));
				index+=2;
				break;
			default:
				break;
		}
	}
	return 0;
}

void DecodeBumsAndWheelsFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data) //id7
{
	IO_SENSORS_CREATE.BumpRight=data & 0x01; // bitove and, iba ak je bit z najnizsou vahou rovny 1 bude vysledok rovny 1
	IO_SENSORS_CREATE.BumpLeft=(data & 0x02)/2;
	IO_SENSORS_CREATE.WheelpdropRight=(data & 0x04)/4;
	IO_SENSORS_CREATE.WheelpdropLeft=(data & 0x08)/8;
	IO_SENSORS_CREATE.WheelpdropCaster=(data & 0x10)/16;
}

void DecodeWallFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data) //id8
{
	IO_SENSORS_CREATE.Wall=data;
}
void DecodeCliffLeftFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data) //id9
{
	IO_SENSORS_CREATE.CliffLeft=data;
}

void DecodeCliffFrontLeftFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id10
{
	IO_SENSORS_CREATE.CliffFrontLeft=data;
}

void DecodeCliffFrontRightFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id11
{
	IO_SENSORS_CREATE.CliffFrontRight=data;
}

void DecodeCliffRightFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id12
{
	IO_SENSORS_CREATE.CliffRight=data;
}

void DecodeVirtualWallFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id13
{
	IO_SENSORS_CREATE.VirtualWall=data;
}

void DecodeLSDandWheelOvercurrentFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id14
{
	IO_SENSORS_CREATE.LSD1overcurrent=data & 0x01; // bitove and, iba ak je bit z najnizsou vahou rovny 1 bude vysledok rovny 1
	IO_SENSORS_CREATE.LSD0overcurrent=(data & 0x02)/2;
	IO_SENSORS_CREATE.LSD2overcurrent=(data & 0x04)/4;
	IO_SENSORS_CREATE.RightWheelovercurrent=(data & 0x08)/8;
	IO_SENSORS_CREATE.LeftWheelovercurrent=(data & 0x10)/16;
}

void DecodeIRFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id17
{
	IO_SENSORS_CREATE.IRbyte=data;
	
}

void DecodeButtonsFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id18
{
	IO_SENSORS_CREATE.PlayPressed=data & 0x01; // bitove and, iba ak je bit z najnizsou vahou rovny 1 bude vysledok rovny 1
	IO_SENSORS_CREATE.AdvancePressed=(data & 0x04)/4;
	
}

void DecodeDistanceFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id19
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.Distance=pomoc1.data; 
	

}

void DecodeAngleFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id20
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.Angle=pomoc1.data; 
	

}
void DecodeChargingStateFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id21
{
	IO_SENSORS_CREATE.ChargingState=data;
}

void DecodeVoltageFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id 22
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.Voltage=pomoc1.data; 
	

}

void DecodeCurrentFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id 23
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.Current=pomoc1.data; 
	

}

void DecodeBatteryTemperatureFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id24
{
	IO_SENSORS_CREATE.BatteryTemperature=(char)data; 
	
}

void DecodeBatteryChargeFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id25
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.BatteryCharge=pomoc1.data; 
	

}
void DecodeBatteryCapacityFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id26
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.BatteryCapacity=pomoc1.data; 
	

}

void DecodeWallSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id27
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.WallSignal=pomoc1.data; 
	

}
void DecodeCliffLeftSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id28
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.CliffLeftSignal=pomoc1.data; 
	

}

void DecodeCliffFrontLeftSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id29
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.CliffFrontLeftSignal=pomoc1.data; 
	

}

void DecodeCliffFrontRightSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id30
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.CliffFrontRightSignal=pomoc1.data; 
	

}

void DecodeCliffRightSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id31
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.CliffRightSignal=pomoc1.data;
	

}

void DecodeCargobayDigitalInputFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id32
{
	IO_SENSORS_CREATE.CargoBayDigitalInput0=data & 0x01; // bitove and, iba ak je bit z najnizsou vahou rovny 1 bude vysledok rovny 1
	IO_SENSORS_CREATE.CargoBayDigitalInput1=(data & 0x02)/2;
	IO_SENSORS_CREATE.CargoBayDigitalInput2=(data & 0x04)/4;
	IO_SENSORS_CREATE.CargoBayDigitalInput3=(data & 0x08)/8;
	IO_SENSORS_CREATE.DeviceDetect_BaudRateChange=(data & 0x10)/16;
}

void DecodeCargobayAnalogSignalFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id33
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.CargoBayAnalogSignal=pomoc1.data;
	

}

void DecodeChargingSourceAvailableFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id34
{
	IO_SENSORS_CREATE.InternalCharger=data & 0x01; // bitove and, iba ak je bit z najnizsou vahou rovny 1 bude vysledok rovny 1
	IO_SENSORS_CREATE.HomaBaseCharger=(data & 0x02)/2;
	
}

void DecodeOImodeFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id35
{
	IO_SENSORS_CREATE.OImode=data;
	
}

void DecodeSongNumberFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id36
{
	IO_SENSORS_CREATE.SongNumber=data;
	
}
void DecodeSongPlayingFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id37
{
	IO_SENSORS_CREATE.SongPlaying=data;
	
}

void DecodeNumberOfStreamPacketsFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char data)//id38
{
	IO_SENSORS_CREATE.NumberOfStreamPAckets=data;
	
}

void DecodeRequestedVelocityFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id39
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[0]=*data;
	pomoc1.datab[1]=*(data+1);
	IO_SENSORS_CREATE.RequestedVelocity=pomoc1.data;
	

}

void DecodeRequestedRadiusFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id40
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[0]=*data;
	pomoc1.datab[1]=*(data+1);
	IO_SENSORS_CREATE.RequestedRadius=pomoc1.data;
	

}

void DecodeRequestedRightVelocityFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id41
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.RequestedRightVelocity=pomoc1.data;
	

}

void DecodeRequestedLeftVelocityFromPacket(CreateSensors &IO_SENSORS_CREATE,unsigned char *data)//id42
{
	union pomoc_t
	{
		uint16_t data;
		unsigned char datab[2];
	}pomoc1;
	pomoc1.datab[1]=*data;
	pomoc1.datab[0]=*(data+1);
	IO_SENSORS_CREATE.RequestedLeftVelocity=pomoc1.data;
	

}