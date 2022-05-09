#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "IMUheader.h"
#include <math.h>
#include "RFsensor.h"
#include "Getcoordinates.h"
#include <DRV8835MotorShield.h>
#include "PixyPos.h"
//#include "pdController.h"

#define PIN_OUTPUT 3

struct Bot {
  int x;
  int y;
  int theta;
};

void Control(Bot& robot, Bot& obj, Pixy2I2C pixy);
// Output a value of speed adjustment to current motor speed.

#endif
