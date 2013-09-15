/***************************************
 * AXDL345 Driver for Arduino
 
***************************************/

#include <Wire.h>
#include "ADXL345.h"

ADXL345::ADXL345()
{
	status = ADXL345_OK;
	errorCode = ADXL345_NO_ERROR;
	
	gains[0] = 0.00376390;
	gains[1] = 0.00376009;
	gains[2] = 0.00349265;
}

void ADXL345::Init(int address)
{
	devAddress = address;
	GetDevID();
	if (errorCode == ADXL345_NO_ERROR)
		PowerOn();
	else
		status = ADXL345_ERROR;
}

byte ADXL345::GetDevID()
{
	byte name;
	ReadFrom(ADXL345_DEVID, 1, &name);
	if (name == ADXL345_ID)
	{
		errorCode = ADXL345_NO_ERROR;
		return name;
	}
	else
	{
		errorCode = ADXL345_NO_DEV;
		return 0x00;
	}
}

void ADXL345::PowerOn()
{
	WriteTo(ADXL345_POWER_CTL, 0x08);
}

void ADXL345::PowerDown()
{
	WriteTo(ADXL345_POWER_CTL, 0x00);
}

void ADXL345::SleepDev()
{
	SetRegisterBit(ADXL345_POWER_CTL, 2, 0);
}

void ADXL345::ReadAccel(int* xyz)
{
	ReadAccel(xyz, xyz + 1, xyz + 2);
}

void ADXL345::ReadAccel(int* x, int* y, int* z)
{
	ReadFrom(ADXL345_DATAX0, ADXL345_TO_READ, buff);

	*x = (((int)buff[1]) << 8) | buff[0];  
	*y = (((int)buff[3]) << 8) | buff[2];
	*z = (((int)buff[5]) << 8) | buff[4]; 
}

void ADXL345::GetGxyz(float* xyz)
{
	int i;
	int xyzInt[3];
	ReadAccel(xyzInt);
	for(i=0; i<3; i++)
		xyz[i] = xyzInt[i] * gains[i];
}

void ADXL345::WriteTo(byte address, byte val)
{
	Wire.beginTransmission(devAddress); // start transmission to device
	Wire.write(address);             // send register address
	Wire.write(val);                 // send value to write
	Wire.endTransmission();         // end transmission
}

void ADXL345::ReadFrom(byte address, int num, byte buff[])
{
	Wire.beginTransmission(devAddress); // start transmission to device
	Wire.write(address);       	      // sends address to read from
	Wire.endTransmission();         // end transmission

	Wire.beginTransmission(devAddress); // start transmission to device
	Wire.requestFrom(devAddress, num);    // request 6 bytes from device

	int i = 0;
	while(Wire.available())         // device may send less than requested (abnormal)
	{
		buff[i] = Wire.read();    // receive a byte
		i++;
	}
	if(i != num)
	{
		status = ADXL345_ERROR;
		errorCode = ADXL345_READ_ERROR;
	}
	Wire.endTransmission();         // end transmission
}

void ADXL345::EnableLowPower(bool enable)
{
	SetRegisterBit(ADXL345_BW_RATE, 4, enable);
}

void ADXL345::SetRate(byte bwCode)
{
	if((bwCode < ADXL345_BW_3) || (bwCode > ADXL345_BW_1600))
	{
		status = false;
		errorCode = ADXL345_BAD_ARG;
	}
	else
	{
		WriteTo(ADXL345_BW_RATE, bwCode);
	}
}

// Gets the range setting and return it into rangeSetting
// it can be 2, 4, 8 or 16
void ADXL345::GetRange(byte* range)
{
	byte _b;
	ReadFrom(ADXL345_DATA_FORMAT, 1, &_b);
	*range = _b & B00000011;
}

// Sets the range setting, possible values are: 2, 4, 8, 16
void ADXL345::SetRange(int range)
{
	switch (range)
	{
		case ADXL345_RANGE_2G:
			SetRegisterBit(ADXL345_DATA_FORMAT, 1, 0);
			SetRegisterBit(ADXL345_DATA_FORMAT, 0, 0);
			break;
		case ADXL345_RANGE_4G:
			SetRegisterBit(ADXL345_DATA_FORMAT, 1, 0);
			SetRegisterBit(ADXL345_DATA_FORMAT, 0, 1);
			break;
		case ADXL345_RANGE_8G:
			SetRegisterBit(ADXL345_DATA_FORMAT, 1, 1);
			SetRegisterBit(ADXL345_DATA_FORMAT, 0, 0);
			break;
		case ADXL345_RANGE_16G:
			SetRegisterBit(ADXL345_DATA_FORMAT, 1, 1);
			SetRegisterBit(ADXL345_DATA_FORMAT, 0, 1);
			break;
		default:
			SetRegisterBit(ADXL345_DATA_FORMAT, 1, 0);
			SetRegisterBit(ADXL345_DATA_FORMAT, 0, 0);
			break;
	}
}

bool ADXL345::GetFullResolution()
{
	return GetRegisterBit(ADXL345_DATA_FORMAT, 3);
}

void ADXL345::SetFullResolution(bool fullRes)
{
	SetRegisterBit(ADXL345_DATA_FORMAT, 3, fullRes);
}

void ADXL345::SetRegisterBit(byte regAdress, int bitPos, bool state)
{
	byte _b;
	ReadFrom(regAdress, 1, &_b);
	if (state)
	{
		_b |= (1 << bitPos);  // forces nth bit of _b to be 1.  all other bits left alone.
	}
	else
	{
		_b &= ~(1 << bitPos); // forces nth bit of _b to be 0.  all other bits left alone.
	}
	WriteTo(regAdress, _b);
}

bool ADXL345::GetRegisterBit(byte regAdress, int bitPos)
{
	byte _b;
	ReadFrom(regAdress, 1, &_b);
	return ((_b >> bitPos) & 1);
}

// print all register value to the serial ouptut, which requires it to be setup
// this can be used to manually to check the current configuration of the device
void ADXL345::PrintAllRegister()
{
	byte _b;
	Serial.print("0x00: ");
	ReadFrom(0x00, 1, &_b);
	PrintByte(_b);
	Serial.println();
	int i;
	for (i=29; i<=57; i++)
	{
		Serial.print("0x");
		Serial.print(i, HEX);
		Serial.print(": ");
		ReadFrom(i, 1, &_b);
		PrintByte(_b);
		Serial.println();    
	}
}

void ADXL345::PrintByte(byte val)
{
	int i;
	Serial.print("B");
	for(i=7; i>=0; i--)
	{
		Serial.print(val >> i & 1, BIN);
	}
}