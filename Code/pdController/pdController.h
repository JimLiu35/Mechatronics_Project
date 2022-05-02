#ifndef PDCONTROLLER_H_
#define PDCONTROLLER_H_

#include "IMUheader.h"
#include <PID_v1.h>
#include <math.h>
#include "RFsensor.h"
#include "Getcoordinates.h"

#define PIN_OUTPUT 3
#define posControl 1
#define angControl 2
struct object {
  int x;
  int y;
  int theta;
};

int pdControl(object& robot, object& obj, int type);
// Output a value of speed adjustment to current motor speed.

#endif
