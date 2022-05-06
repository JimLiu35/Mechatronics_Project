#ifndef PIXYPOS_H_
#define PIXYPOS_H_

#include <Arduino.h>
#include <Pixy2.h>
#include <DRV8835MotorShield.h>

struct object {
  int colorSig;
  int object_x;
  int object_y;
};

boolean objPosition(object &obj, int loopIndex, Pixy2 pixy);


#endif
