#include "Code.h"
//#include <IRremote.h>

//uint8_t robotStates;      // Create global state variable
int robotStates;
float RF_coordinates[6];  // Coordinates read by RF sensor
int matchStatus;          // matchStatus = 1 -- Start
//                                         2 -- Stop
//                                         3 -- Read Coordinates

float num[6];

NRF24 radio;
boolean puckCaptured = false;
Bot robotForward;
Bot Puck;
Bot Goal;
object robotForward_pixy;
object puck_pixy;
Pixy2I2C pixy;

uint8_t M1DIR = 2;
uint8_t M1PWM = 3;
uint8_t M2DIR = 4;
uint8_t M2PWM = 5;
DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);

// pixy
const int viewCenter_x = 158;
const int viewCenter_y = 100;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  initial_RF(&radio, 1);
  robotStates = puckSearching;
  Goal.x = 230;
  Goal.y = 65;
  Goal.theta = NULL;
  pinMode(IR_LED, OUTPUT);
  //  Serial.println("Inializing...");
  puck_pixy.colorSig = 5;
  boolean capture = pixyControl(robotForward_pixy, puck_pixy, pixy);
  
}

void loop() {
  int i;
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
      //      Serial.println(robotStates);
      //      digitalWrite(IR_LED,HIGH);
      matchStatus = RF_receiver(&radio, num);                  // Read RF
      //      Serial.println(matchStatus);
      if (matchStatus == 1)
        robotStates = puckSearching;

      else
        robotStates = Initialize;
      //      Serial.println(robotStates);
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
      //      Serial.println(robotStates);
      digitalWrite(IR_LED, HIGH);
      puckCaptured = false;
      // functions
      // 1. Get coordinates of robot and the puck
      matchStatus = RF_receiver(&radio, RF_coordinates);                   // Read RF & Get Coordinates
      if (matchStatus == 2) {
        robotStates = Stop;
      }
      robotForward.x = RF_coordinates[2];
      robotForward.y = RF_coordinates[3];
      Puck.x = RF_coordinates[4];
      Puck.y = RF_coordinates[5];
      Puck.theta = NULL;
      // 2. PD control -- Alignment
      float distFP = sqrt(pow((robotForward.x - Puck.x), 2) + pow((robotForward.y - Puck.y), 2)); // Distance between robotForward and Puck

      Control(robotForward, Puck, pixy);

      // pixy control
      
      
      puckCaptured = pixyControl(robotForward_pixy, puck_pixy, pixy);
      


      if (puckCaptured == false)
      {
        motors.setM2Speed(-200);
        motors.setM1Speed(-200);
        delay(100);
        robotStates = puckSearching;
      }

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
      //      float distFG = sqrt((robotForward.x - Goal.x) ^ 2 + (robotForward.y - Goal.y) ^ 2);     // Distance between robotForward and Goal
      //      if (distFG > 3000)
      Control(robotForward, Goal, pixy);
      //      else
      //      {
      //        // Moving forward
      //        motors.setM2Speed(400);
      //        motors.setM1Speed(400);
      //      }

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

    case Stop:


      break;

  }

}
