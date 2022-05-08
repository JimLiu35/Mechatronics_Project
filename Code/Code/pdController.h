#ifndef PDCONTROLLER_H_
#define PDCONTROLLER_H_

#include "IMUheader.h"
#include <PID_v1.h>
#include <math.h>
#include "RFsensor.h"
#include "Getcoordinates.h"
#include "Controller.h"
#include <DRV8835MotorShield.h>

#define PIN_OUTPUT 3
#define posControl 1
#define angControl 2
//struct Bot {
//  int x;
//  int y;
//  int theta;
//};

void pdControl(Bot& robot, Bot& obj);
// Output a value of speed adjustment to current motor speed.

#endif
