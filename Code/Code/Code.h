#ifndef CODE_H_
#define CODE_H_
// include librarys
#include <SPI.h>
#include <NRF24.h>
//#include <Arduino_AVRSTL.h>

// Define pins



// Define variables


double pidControl(const double setPoint, const double currentPosition, const double targetPosition);

// borderCheck function: check if the robot reaches the border
// Input: Robot current x and y coordinates
// Output: ture if it reaches the border, false if it not
boolean borderCheck(const double x, const double y);


// Function prototypes



#endif
