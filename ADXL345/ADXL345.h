/***************************************
 * AXDL345 Driver for Arduino
 
***************************************/

#include "Arduino.h"

#ifndef ADXL345_h
#define ADXL345_h

/* -- ADXL345 addresses --*/
#define ADXL345_ADDR_ALT_HIGH 0x1D // ADXL345 address when ALT is connected to HIGH
#define ADXL345_ADDR_ALT_LOW  0x53 // ADXL345 address when ALT is connected to LOW

/* ------- Register names ------- */
#define ADXL345_DEVID 0x00
#define ADXL345_RESERVED1 0x01
#define ADXL345_THRESH_TAP 0x1d
#define ADXL345_OFSX 0x1e
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_THRESH_ACT 0x24
#define ADXL345_THRESH_INACT 0x25
#define ADXL345_TIME_INACT 0x26
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_THRESH_FF 0x28
#define ADXL345_TIME_FF 0x29
#define ADXL345_TAP_AXES 0x2a
#define ADXL345_ACT_TAP_STATUS 0x2b
#define ADXL345_BW_RATE 0x2c
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_FIFO_CTL 0x38
#define ADXL345_FIFO_STATUS 0x39

// Interrupt PINs INT1: 0  INT2: 1
#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01

// Interrupt bit position
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVITY_BIT   0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT  0x02
#define ADXL345_INT_WATERMARK_BIT  0x01
#define ADXL345_INT_OVERRUNY_BIT   0x00

// For BW_RATE
#define ADXL345_BW_1600 0xF // 1111
#define ADXL345_BW_800  0xE // 1110
#define ADXL345_BW_400  0xD // 1101  
#define ADXL345_BW_200  0xC // 1100
#define ADXL345_BW_100  0xB // 1011  
#define ADXL345_BW_50   0xA // 1010 
#define ADXL345_BW_25   0x9 // 1001 
#define ADXL345_BW_12   0x8 // 1000 
#define ADXL345_BW_6    0x7 // 0111
#define ADXL345_BW_3    0x6 // 0110

// For DATA_FORMAT
#define ADXL345_RANGE_2G	1
#define ADXL345_RANGE_4G	2
#define ADXL345_RANGE_8G	3
#define ADXL345_RANGE_16G	4

#define ADXL345_OK    1 // no error
#define ADXL345_ERROR 0 // indicates error is predent

#define ADXL345_NO_ERROR   	0 // initial state
#define ADXL345_READ_ERROR 	1 // problem reading accel
#define ADXL345_BAD_ARG    	2 // bad method argument
#define ADXL345_NO_DEV		3

#define ADXL345_ID		0xE5
#define ADXL345_TO_READ 6	// num of byte to be read

class ADXL345
{
public:
	bool status;			// set when error occurs, see error code for details
	byte errorCode;		// Initial state
	float gains[3];			// counts to Gs
	
	ADXL345();
	void Init(int address);
	byte GetDevID();
	void PowerOn();
	void PowerDown();
	void SleepDev();
	void ReadAccel(int* xyz);
	void ReadAccel(int* x, int* y, int* z);
	void GetGxyz(float* xyz);
	void WriteTo(byte address, byte val);
	
	void EnableLowPower(bool enable);
	void SetRate(byte bwCode);
	void GetRange(byte* range);
	void SetRange(int range);
	bool GetFullResolution();
	void SetFullResolution(bool fullRes);
	void PrintAllRegister();
	
private:
	byte buff[ADXL345_TO_READ] ;    //6 bytes buffer for saving data read from the device
	int devAddress;
	
	void ReadFrom(byte address, int num, byte buff[]);
	void SetRegisterBit(byte regAdress, int bitPos, bool state);
	bool GetRegisterBit(byte regAdress, int bitPos);
	void PrintByte(byte val);
};
#endif