// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

#include "GeneralParams.h"
#include "ControlUtils.h"

void sendReq(int addr, ReqPacket currPaecket) {
  uint8_t* currBytes = (byte*)&currPaecket;
  rs485.send(addr, sizeof(float), currBytes);
}


void setup() {

        Serial.begin(SERIAL_BAUDRATE);
        altSerial.begin(RS485_BAUDRATE);

        rs485 = Simple485(&altSerial, RS485_MASTER_ADDR, REDPIN);

#if !defined(__MIPSEL__)
        while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
        Serial.println("Starting Logitech F310 gamepad");

        if (Usb.Init() == -1)
                Serial.println("OSC did not start.");
                
        // Set this to higher values to enable more debug information
        // minimum 0x00, maximum 0xff, default 0x80
        // UsbDEBUGlvl = 0xff;

        delay(200);
        Serial.println("Ready. Addr " + String(RS485_MASTER_ADDR));

}



void loop() {

    Usb.Task();
    rs485.loop();

    if (lf310.connected()) {

      preprocessJoystickVals(lf310.lf310Data.Z,
                            lf310.lf310Data.Y, 
                            joystickRightX,
                            joystickLeftY);      
      
      calcMotorSetpoint(joystickRightX, 
                        joystickLeftY,
                        rightMotorPWM,
                        rightMotorDir,
                        leftMotorPWM,
                        leftMotorDir);
                        
      Serial.println("LeftY = " + String(joystickLeftY) +
                     " RightX = " + String(joystickRightX) + 
                     " leftMotorPWM = " + String(leftMotorPWM) + 
                     " leftMotorDir = " + String(leftMotorDir) + 
                     " rightMotorPWM = " + String(rightMotorPWM) + 
                     " rightMotorDir = " + String(rightMotorDir));

      ReqPacket currReqPacket = {0};

      currReqPacket.leftMotorDir = leftMotorDir;
      currReqPacket.leftMotorPWM = leftMotorPWM;
      currReqPacket.rightMotorDir = rightMotorDir;
      currReqPacket.rightMotorPWM = rightMotorPWM;

      sendReq(RS485_SLAVE_ADDR, currReqPacket);

      delay(100);

    }
}
