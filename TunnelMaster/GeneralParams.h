#ifndef GENERAL_PARAMS
#define GENERAL_PARAMS

#include "GeneralUtils.h"

#define SERIAL_BAUDRATE 9600
#define RS485_BAUDRATE 9600
#define REDPIN 44
#define RS485_MASTER_ADDR 100
#define RS485_SLAVE_ADDR 101

// lf310.lf310Data.X // Left Joystick X
// lf310.lf310Data.Y // Left Joystick Y
// lf310.lf310Data.Z // Right Joystick X
// lf310.lf310Data.Rz // Right Joystick Y

USB Usb;
LF310 lf310(&Usb);

Simple485 rs485;
AltSoftSerial altSerial;

int joystickLeftY = 0;
int joystickRightX = 0;

int leftMotorPWM = 0;
uint8_t leftMotorDir = FORWARD;
int rightMotorPWM = 0;
uint8_t rightMotorDir = BACKWARD;

typedef struct __attribute__((packed)) ReqPacket {
	
  uint8_t leftMotorDir;
  int leftMotorPWM;
  uint8_t rightMotorDir;
  int rightMotorPWM;
  
} ReqPacket;

#endif