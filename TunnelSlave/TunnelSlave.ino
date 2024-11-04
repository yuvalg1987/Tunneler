#include <Simple485.h>
#include <AltSoftSerial.h>

#define BAUDRATE 9600
#define RS485_MASTER_ADDR 100
#define RS485_SLAVE_ADDR 101
#define REDEPIN 7

#define M1_PWM 3
#define M1_IN1 4
#define M1_IN2 5

#define M2_PWM 6
#define M2_IN1 11
#define M2_IN2 12

enum MotorDir {
  BACKWARD,  // 0
  FORWARD, // 1
};

Simple485 rs485;
AltSoftSerial altSerial;

typedef struct __attribute__((packed)) ReqPacket {
	                  
  uint8_t leftMotorDir;
  int leftMotorPWM;
  uint8_t rightMotorDir;
  int rightMotorPWM;
  
} ReqPacket;

float bytesToFloat(byte* bytes, int arrLen) {

  static_assert(sizeof(float) == 4, "float size is expected to be 4 bytes");
  float floatVal;
  memcpy (&floatVal, bytes, 4);
  return floatVal;
}

ReqPacket parseInput(byte* bytes, int arrLen) {

  ReqPacket currReq;
  memcpy (&currReq, bytes, arrLen);
  return currReq;
}

void M1MoveMotor(int motorSpeed, uint8_t motorDir) {

  if (motorDir == FORWARD) {
    M1MoveForward(motorSpeed);
  }
  else {
    M1MoveBackward(motorSpeed);
  }
}


void M1MoveForward(int motorSpeed){  
  
    digitalWrite (M1_IN1, HIGH);
    digitalWrite (M1_IN2, LOW);         
    analogWrite (M1_PWM, (int)fabs(motorSpeed));

}

void M2MoveMotor(int motorSpeed, uint8_t motorDir) {

  if (motorDir == FORWARD) {
    M2MoveForward(motorSpeed);
  }
  else {
    M2MoveBackward(motorSpeed);
  }

}
void M2MoveForward(int motorSpeed){  
  
    digitalWrite (M2_IN1, HIGH);
    digitalWrite (M2_IN2, LOW);         
    analogWrite (M2_PWM, (int)fabs(motorSpeed));

}

void M1MoveBackward(int motorSpeed){ 
     
    digitalWrite (M1_IN1, LOW);
    digitalWrite (M1_IN2, HIGH);           
    analogWrite (M1_PWM, (int)fabs(motorSpeed));
}

void M2MoveBackward(int motorSpeed){ 
     
    digitalWrite (M2_IN1, LOW);
    digitalWrite (M2_IN2, HIGH);           
    analogWrite (M2_PWM, (int)fabs(motorSpeed));
}

void M1StopMotor(){     

    digitalWrite (M1_IN1, LOW);
    digitalWrite (M1_IN2, LOW); 
}

void M2StopMotor() {

    digitalWrite (M2_IN1, LOW);
    digitalWrite (M2_IN2, LOW);  

}


void waitForMasterBoot() {

 while (rs485.received() <= 0) {
    rs485.loop();
    Serial.println("Waiting for Master");
    delay(500);
  }
  Message initMessage = rs485.read();
  float initFloat = bytesToFloat(initMessage.bytes, initMessage.len);
  Serial.println("Recieved Init message" + String(initFloat));

  float testFloat = 12345.0;
  uint8_t* currBytes = (byte*)&testFloat;
  rs485.send(RS485_MASTER_ADDR, sizeof(float), currBytes);
  // Serial.println("Sent Init ACK " + String(currServoReq));

}

void init_pinout() {

  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  digitalWrite(M1_PWM, LOW);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);

  digitalWrite(M2_PWM, LOW);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);


}
void setup() {

  init_pinout();

  Serial.begin(BAUDRATE);
  altSerial.begin(BAUDRATE);

  rs485 = Simple485(&altSerial, RS485_SLAVE_ADDR, REDEPIN);
  Serial.println("RS485 Ready. Addr " + String(RS485_SLAVE_ADDR));

}



void loop() {

  // Serial.println("Forward");
  // M1MoveForward(50);
  // M2MoveForward(50);
  
  // delay(5000);

  // Serial.println("Stop");
  // M1StopMotor();
  // M2StopMotor();

  // delay(5000);

  // Serial.println("Backward");
  // M1MoveBackward(50);
  // M2MoveBackward(50);

  // delay(5000);

  // Serial.println("Stop");
  // M1StopMotor();
  // M2StopMotor();

  // delay(5000);

  rs485.loop();

  ReqPacket currReq = {0};

  while (rs485.received() > 0) {

    Message m = rs485.read();
    currReq = parseInput(m.bytes, m.len);

    Serial.println(" leftMotorPWM = " + String(currReq.leftMotorPWM) + 
                   " leftMotorDir = " + String(currReq.leftMotorDir) + 
                   " rightMotorPWM = " + String(currReq.rightMotorPWM) + 
                   " rightMotorDir = " + String(currReq.rightMotorDir));

    delete [] m.bytes;

    M1MoveMotor(currReq.leftMotorPWM, currReq.leftMotorDir);
    M2MoveMotor(currReq.rightMotorPWM, currReq.rightMotorDir);
  }

  // M1MoveMotor(currReq.leftMotorPWM, currReq.leftMotorDir);
  // M2MoveMotor(currReq.rightMotorPWM, currReq.rightMotorDir);

  // delay(20);
}
