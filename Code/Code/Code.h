#ifndef CODE_H_
#define CODE_H_
// include librarys
#include <SPI.h>
#include <NRF24.h>
#include "Getcoordinates.h"
#include "RFsensor.h"
#include "pdController.h"
#include "IRCheck.h"
#include <DRV8835MotorShield.h>
//#include <Arduino_AVRSTL.h>

// Define pins

// Define states
#define Initialize 0
#define puckSearching 1
#define Attacking 2
#define Defending 3
#define Stop 4

// Define variables
//struct Bot {
//  int x;
//  int y;
//  int theta;
//};


// Function prototypes

double pidControl(const double setPoint, const double currentPosition, const double targetPosition);

// borderCheck function: check if the robot reaches the border
// Input: Robot current x and y coordinates
// Output: ture if it reaches the border, false if it not
boolean borderCheck(const Bot robot);

boolean puckCaptureCheck();





#endif
