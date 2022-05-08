#ifndef DEFENSEROBOT_H_
#define DEFENSEROBOT_H_

// include librarys
#include <SPI.h>
#include <NRF24.h>
#include "Getcoordinates.h"
#include "RFsensor.h"
#include "pdController.h"
#include <DRV8835MotorShield.h>
#include "PixyPos.h"

// Define states
#define Initialize 0
#define Inplace 1
#define Defending 2


double pidControl(const double setPoint, const double currentPosition, const double targetPosition);

// borderCheck function: check if the robot reaches the border
// Input: Robot current x and y coordinates
// Output: ture if it reaches the border, false if it not
boolean borderCheck(const Bot robot);




#endif
