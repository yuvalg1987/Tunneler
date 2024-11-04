#ifndef GENERAL_UTILS
#define GENERAL_UTILS

#include <Arduino.h>
#include <SPI.h>
#include <Simple485.h>
#include <AltSoftSerial.h>

#include "lf310.h"

enum MotorDir {
  BACKWARD,  // 0
  FORWARD, // 1
};

#endif