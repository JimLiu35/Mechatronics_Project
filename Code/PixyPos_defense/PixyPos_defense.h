#ifndef PIXYPOS_DEFENSE_H_
#define PIXYPOS_DEFENSE_H_

#include <Arduino.h>
#include <Pixy2I2C.h>
//#include <SPI.h>
//#include <NRF24.h>
#include "Getcoordinates.h"
//#include "RFsensor.h"
#include <DRV8835MotorShield.h>

#define Initialize 0
#define Inplace 1
#define Defending 2
#define Stop 3

struct object {
  int colorSig;
  int object_x;
  int object_y;
};
struct Bot {
  int x;
  int y;
  int theta;
};
boolean objPosition(object &obj, int loopIndex, Pixy2I2C pixy);

#endif
