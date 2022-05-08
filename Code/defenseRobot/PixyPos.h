#ifndef PIXYPOS_H_
#define PIXYPOS_H_

#include <Arduino.h>
#include <Pixy2I2C.h>
#include <DRV8835MotorShield.h>
#include <IRremote.h>
struct object {
  int colorSig;
  int object_x;
  int object_y;
};

boolean objPosition(object &obj, int loopIndex, Pixy2I2C pixy);
boolean pixyControl(object& robot, object& puck);
boolean puckCaptureCheck();
#endif
