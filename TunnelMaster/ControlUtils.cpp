#include "ControlUtils.h"

MotorDir calcMotorDir(int joystickVal) {
  
  if (joystickVal > 0)
    return FORWARD;
  else
    return BACKWARD;
}

void preprocessJoystickVals(uint8_t rightXOrig,
                            uint8_t leftYOrig, 
                            int& rightX, 
                            int& leftY) {

  rightX =  (rightXOrig - 128);
  leftY  = -(leftYOrig  - 128);

}

void processSteer(int joystickRightX,
                  int joystickLeftY, 
                  int& rightMotorSpeed, 
                  int& leftMotorSpeed) {

    int forwardSpeed = abs(joystickLeftY);
    int turnSpeed    = abs(joystickRightX);

    int rightMotorCorrection = 0;
    int leftMotorCorrection  = 0;

    if (joystickRightX > JOYSTICK_DEAD_BAND) {  // Turn Right
        rightMotorCorrection = 0;
        leftMotorCorrection  = min(turnSpeed, forwardSpeed);
    }
    else if (joystickRightX < -JOYSTICK_DEAD_BAND) {  // Turn Left
        rightMotorCorrection = -min(turnSpeed, forwardSpeed);
        leftMotorCorrection  = 0;
    }
    else {
        rightMotorCorrection = 0;
        leftMotorCorrection  = 0;
    }
    rightMotorSpeed = forwardSpeed + rightMotorCorrection;
    leftMotorSpeed  = forwardSpeed - leftMotorCorrection;

}

void calcMotorSetpoint(int joystickRightX,
                       int joystickLeftY,
                       int& rightMotorSpeed,
                       uint8_t& rightMotorDir, 
                       int& leftMotorSpeed,
                       uint8_t& leftMotorDir) {

    if (joystickLeftY > JOYSTICK_DEAD_BAND) {  // Forward
        rightMotorDir = FORWARD;
        leftMotorDir  = FORWARD;
        processSteer(joystickRightX,
                     joystickLeftY, 
                     rightMotorSpeed,
                     leftMotorSpeed);
    }
    else if (joystickLeftY < -JOYSTICK_DEAD_BAND) { // Backward
        rightMotorDir = BACKWARD;
        leftMotorDir  = BACKWARD;
        processSteer(joystickRightX, 
                     joystickLeftY, 
                     rightMotorSpeed,
                     leftMotorSpeed);
    }
    else { // Turn In Place
        if (joystickRightX > 10) { // Turn Right
            rightMotorDir = FORWARD;
            leftMotorDir  = BACKWARD;
            rightMotorSpeed = abs(joystickRightX);
            leftMotorSpeed  = abs(joystickRightX);
        }
        else if (joystickRightX < -10) { // Turn Left
            rightMotorDir = BACKWARD;
            leftMotorDir = FORWARD;
            rightMotorSpeed = abs(joystickRightX);
            leftMotorSpeed = abs(joystickRightX);
        }
        else {
            rightMotorDir = FORWARD;
            leftMotorDir = FORWARD;
            rightMotorSpeed = 0;
            leftMotorSpeed = 0;
        }
    }

  rightMotorSpeed = max(2 * rightMotorSpeed - 1, 0);
  leftMotorSpeed  = max(2 * leftMotorSpeed - 1, 0);
}


