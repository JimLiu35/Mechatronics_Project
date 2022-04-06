#ifndef CODE_H_
#define CODE_H_
// include librarys
#include <SPI.h>
#include <NRF24.h>
#include <Arduino_AVRSTL.h>

// Define pins
double currentTime;
double previousTime;


// Define variables
double pidControl(const double currentPosition, const double targetPosition);


// Function prototypes



#endif
