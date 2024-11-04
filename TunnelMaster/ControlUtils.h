#ifndef CONTORL_UTILS
#define CONTROL_UTILS

#include "GeneralUtils.h"

#define JOYSTICK_DEAD_BAND 10

MotorDir calcMotorDir(int joystickVal);

void preprocessJoystickVals(uint8_t rightXOrig,
                            uint8_t leftYOrig, 
                            int& rightX, 
                            int& leftY);

void processSteer(int joystickRightX,
                  int joystickLeftY, 
                  int& rightMotorSpeed, 
                  int& leftMotorSpeed);

void calcMotorSetpoint(int joystickRightX,
                       int joystickLeftY,
                       int& rightMotorSpeed,
                       uint8_t& rightMotorDir, 
                       int& leftMotorSpeed,
                       uint8_t& leftMotorDir);


#endif