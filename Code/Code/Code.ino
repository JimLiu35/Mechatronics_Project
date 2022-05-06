#include "Code.h"

uint8_t robotStates;      // Create global state variable
float RF_coordinates[6];  // Coordinates read by RF sensor
int matchStatus;          // matchStatus = 1 -- Start
//                                         2 -- Stop
//                                         3 -- Read Coordinates
char *text;              // Message read from the RF sensor



NRF24 radio;
boolean puckCaptured = false;
Bot robotForward;
Bot Puck;
Bot Goal;

uint8_t M1DIR = 2;
uint8_t M1PWM = 3;
uint8_t M2DIR = 4;
uint8_t M2PWM = 5;
DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  initial_RF(&radio, 1);
  robotStates = Initialize;
  Goal.x = 0;
  Goal.y = 0;
  Goal.theta = NULL;
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
      // function
      text = RF_receiver(&radio);                   // Read RF
      matchStatus = start_stop_message(text);       // Check matchStatus
      if (matchStatus == 1)
        robotStates = puckSearching;
      else
        robotStates = Initialize;

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
      puckCaptured = false;
      // functions
      // 1. Get coordinates of robot and the puck
      text = RF_receiver(&radio);                   // Read RF
      Getcoordinates(text, RF_coordinates);         // Get coordinates
      robotForward.x = RF_coordinates[0];
      robotForward.y = RF_coordinates[1];
      Puck.x = RF_coordinates[2];
      Puck.y = RF_coordinates[3];
      Puck.theta = NULL;

      // 2. PD control -- Alignment
      float distFP = sqrt((robotForward.x - Puck.x) ^ 2 + (robotForward.y - Puck.y) ^ 2);   // Distance between robotForward and Puck
      if (distFP > 3000)
        pdControl(robotForward, Puck);
      else
      {
        // pixy control
        // puckCaptured = pixyControl();
      }

      if (puckCaptured == false)
        robotStates = puckSearching;
      else
        robotStates = Attacking;
      break;

    case Attacking:
      /*
         In this state, the robot captures the puck and bring it towards the
         enemy's gate. The algorithm in this state should
         1. Generate a trajectory towards enemy's gate
         2. Keep checking if the puck is lost
         3. If not, track the generated trajectory
         4. Else, back to puckSearching state.
      */
      // Code here
      // Need a trajectory generating function return a vector of states?
      // Need a trajectory tracking function to follow the generated path
      // Need a boolean type fuunction to check if the puck is captured
      puckCaptured = true;

      // 1. Check if the puck is still captured.
      // 2. PD control to goal
      float distFG = sqrt((robotForward.x - Goal.x) ^ 2 + (robotForward.y - Goal.y) ^ 2);     // Distance between robotForward and Goal
      if (distFG > 3000)
        pdControl(robotForward, Goal);
      else
      {
        // Moving forward
        motors.setM2Speed(400);
        motors.setM1Speed(400);
      }

      if (puckCaptured == false)
        robotStates = puckSearching;
      else
        robotStates = Attacking;
      break;

    case Defending:
      /*
         In this state, the algorithm has to achieve the following:
         1. Block the way from enemy's robot to our goal;
         2. Try to re-capture the puck if possible;
      */
      // Code here
      // Need a trajectory generating function return a vector of states?
      // Need a trajectory tracking function to follow the generated path
      // Check which enemy's robots has the puck.
      // Check if the puck re-captured.


      puckCaptured = false;

      if (puckCaptured == false)
        robotStates = puckSearching;
      else
        robotStates = Attacking;
      break;


  }
}
