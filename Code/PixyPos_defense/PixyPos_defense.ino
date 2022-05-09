
#include "PixyPos_defense.h"
#include <Pixy2I2C.h>
#include <DRV8835MotorShield.h>
int Green_PIN = 6;
int RED_PIN = 7;
int robotStates;
double kp;
float RF_coordinates[6];  // Coordinates read by RF sensor
int matchStatus;          // matchStatus = 1 -- Start
//                                         2 -- Stop
//                                         3 -- Read Coordinates
char *text;              // Message read from the RF sensor
float num[6];
//NRF24 radio;
//boolean enemyAttacking = false;
boolean isinField;

///////////////
Bot robotDefense;
Bot Goal;
Bot Puck;

///////////////
uint8_t M1DIR = 2;
uint8_t M1PWM = 3;
uint8_t M2DIR = 4;
uint8_t M2PWM = 5;
DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);

////////////////
const int viewCenter_x = 157;
const int viewCenter_y = 203;
Pixy2I2C pixy;



////////////////
object puck;


int i;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  initial_RF(&radio, 1);
  robotStates = Inplace;
  
  Goal.x = 15;  // self goal location
  Goal.y = 65;
  Goal.theta = NULL;
    Serial.println("ok");
  pixy.init();
  Serial.println("Set up");
  puck.colorSig = 1;
//  pinMode(Green_PIN, OUTPUT);
//  pinMode(RED_PIN, OUTPUT);

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
//      text = RF_receiver(&radio);                   // Read RF
      matchStatus = start_stop_message(text);       // Check matchStatus
      matchStatus = 1;
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
Serial.println("start");
      isinField = objPosition(puck, i, pixy);  // see the puck
      Serial.println(isinField);
      if (isinField) {
        if (puck.object_x - viewCenter_x < -40)
        {
          // Move to left
          Serial.println("move to left");
          motors.setM2Speed(-200);
          motors.setM1Speed(-200);

        }
        else if (puck.object_x - viewCenter_x > 40)
        {
          // Move to right
          Serial.println("move to right");
          motors.setM2Speed(200);
          motors.setM1Speed(200);

        }
        else
        {
          Serial.println("right front to the ball");
          motors.setM2Speed(10);
          motors.setM1Speed(10);
          delay(200);
          motors.setM2Speed(10);
          motors.setM1Speed(10);
          delay(200);


        }
      }
      else
      {
        /* cannot seen then listen to RF instruction */
        //        text = RF_receiver(&radio);                   // Read RF
        //        Getcoordinates(text, RF_coordinates);         // Get coordinates
        //        robotDefense.x = RF_coordinates[3];
        //        robotDefense.y = RF_coordinates[4];
        //        Puck.x = RF_coordinates[5];
        //        Puck.y = RF_coordinates[6];
        //        float distDP = sqrt(pow((robotDefense.x - Puck.x), 2) + pow((robotDefense.y - Puck.y), 2));
        //        // Distance between Puck and Defense robot
        //        int diff_y = (robotDefense.y - Puck.y);
        //        int diff_x = abs(robotDefense.x - Puck.x);
        //        int pdSpeed = kp * (diff_y) * 200;
        //        if (diff_x < 50)
        //          // if puck is in blind area but very closee
        //        {
        //          if (abs(diff_y) <= 5) {
        //            // in the center
        //            motors.setM2Speed(0);
        //            motors.setM1Speed(0);
        //          }
        //          else if ((diff_y) < -5) {
        //            // on the left side
        //            // move left
        //            motors.setM2Speed(-pdSpeed);
        //            motors.setM1Speed(-pdSpeed);
        //          }
        //          else if ((diff_y) > 5) {
        //            // on the right side
        //            motors.setM2Speed(pdSpeed);
        //            motors.setM1Speed(pdSpeed);
        //          }
        //        }
        //        else {
        //          robotStates = Inplace;
        //        }
      }
  }
}
