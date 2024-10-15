#ifndef CONTORL_UTILS
#define CONTROL_UTILS

#include "GeneralUtils.h"

#define JOYSTICK_DEAD_BAND 10

enum MOTOR_DIR {
  BACKWARD,  // 0
  FORWARD, // 1
};

int calcMotorDir(int joystickVal);

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
                       int& rightMotorDir, 
                       int& leftMotorSpeed,
                       int& leftMotorDir);


#endif