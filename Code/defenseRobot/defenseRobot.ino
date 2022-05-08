#include "defenseRobot.h"
int robotStates;
float RF_coordinates[6];  // Coordinates read by RF sensor
int matchStatus;          // matchStatus = 1 -- Start
//                                         2 -- Stop
//                                         3 -- Read Coordinates
char *text;              // Message read from the RF sensor
float num[6];
NRF24 radio;
boolean enemyAttacking = false;
Bot robotDefense;
Bot Goal;
Bot Puck;


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
  Goal.x = 15;  // self goal location
  Goal.y = 65;
  Goal.theta = NULL;
}


void loop() {
  // put your main code here, to run repeatedly:
  // switch case

  switch (robotStates) {
    case Initialize:
      /* In this stae, robot will waiting for the signal .
          if the signal is received, change state to stay in front of door

          Need a boolean type function to check sound sensor or RF receiver.
          Return true if signal has been received
      */
      // function
      text = RF_receiver(&radio);                   // Read RF
      matchStatus = start_stop_message(text);       // Check matchStatus
      if (matchStatus == 1)
        robotStates = Inplace;
      else
        robotStates = Initialize;


      break;

    case Inplace:
      /* In this state, the robot should move to right front to the goal
          1. adjust the angle
          2. move the place
          3. Don't enter to the goal
      */
      enemyAttacking = false;
      text = RF_receiver(&radio);                   // Read RF
      Getcoordinates(text, RF_coordinates);         // Get coordinates
      robotDefense.x = RF_coordinates[2];
      robotDefense.y = RF_coordinates[3];

      //      float distDP = sqrt(pow((robotDefense.x - Goal.x), 2) + pow((robotDefense.y - Goal.y), 2));
      // Distance between goal and Defense robot

      pdControl(robotDefense, Goal);


    case Defending:
      /* in this case, if defense robot see the puck or get puck's location is less than 30.
          defense robot will move to line between the puck and goal
            1. read the puck location check whether is less 30
            2. calcuate the x-coordinate difference between goal and puck
            3. move robot to the line between puck and buttom of line

      */
      // code here
      // need the RF read puck and robot
      // need PD control for angle
      // need motor sheild to move back and forward

      text = RF_receiver(&radio);                   // Read RF
      Getcoordinates(text, RF_coordinates);         // Get coordinates
      robotDefense.x = RF_coordinates[2];
      robotDefense.y = RF_coordinates[3];
      Puck.x = RF_coordinates[2];
      Puck.y = RF_coordinates[3];

      float distDP = sqrt(pow((robotDefense.x - Puck.x), 2) + pow((robotDefense.y - Puck.y), 2));
      // Distance between Puck and Defense robot


      if (distDP < 300)
      {
        enemyAttacking = true;
        pdControl(robotDefense, Puck);
      }
      else
      {
        // Moving forward
        motors.setM2Speed(400);
        motors.setM1Speed(400);
        delay(200);
        // Moving backward
        motors.setM2Speed(-400);
        motors.setM1Speed(-400);
        //         pixy control
        //         puckCaptured = pixyControl();

      }
      if (enemyAttacking == false)
        robotStates = Inplace;
      else
        robotStates = Defending ;
      break;

    case Stop:

      break;

  }
}
