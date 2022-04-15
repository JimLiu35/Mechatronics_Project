#ifndef CODE_H_
#define CODE_H_
// include librarys
#include <SPI.h>
#include <NRF24.h>
//#include <Arduino_AVRSTL.h>

// Define pins

// Define states
#define Initialize 0
#define puckSearching 1
#define Attacking 2
#define Defending 3

// Define variables
struct Bot {
  // Can store robots' or puck's coordinates
  double x;
  double y;
};


// Function prototypes

double pidControl(const double setPoint, const double currentPosition, const double targetPosition);

// borderCheck function: check if the robot reaches the border
// Input: Robot current x and y coordinates
// Output: ture if it reaches the border, false if it not
boolean borderCheck(const struct Bot robot);




#endif
