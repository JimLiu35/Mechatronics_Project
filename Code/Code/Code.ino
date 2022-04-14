#include "Code.h"

uint8_t robotStates;    // Create global state variable
void setup() {
  // put your setup code here, to run once:
  robotStates = Initialize;
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (robotStates) {
    case Initialize:
      /* In this state, robot keeps waiting for the match-start signal.
         If the signal is received, change state to puckSearching.
         
         Need a boolean type function to check sound sensor and
         the RF receiver. Return true if start signal is received.
      */

      // Code here
      boolean receiveSignal = false;
      // function

      if (receiveSignal == false)
        robotStates = Initialize;
      else
        robotStates = puckSearching;

      break;

    case puckSearching:
      /* In this state, robot searches for the puck. The algorithm at
          this state should
          1. generate a trajectory from robot's current location to
              the puck;
          2. follow the trajectory to the desired location;
          3. check if the puck is captured and switch to the
              corresponding states.
      */
      // Code here
      // Need a trajectory generating function return a vector of states?
      // Need a trajectory tracking function to follow the generated path
      // Need a boolean type function to check if the puck is captured
      boolean puckCaptured = false;
      // functions

      if (puckCaptured == false)
        robotStates = puckSearching;
      else
        robotStates = Attacking;
      break;

    case Attacking:
      // Code here
      break;

    case Defending:
      // Code here
      break;


  }
}
