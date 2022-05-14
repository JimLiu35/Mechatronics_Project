#include "RFsensor.h"
#include "PixyPos_defense.h"
#include "IMUheader.h"
#include <NRF24.h>
#include<SPI.h>
#include <Pixy2I2C.h>
#include <DRV8835MotorShield.h>
sensors_event_t event;
Adafruit_BNO055 bno;

#define Address 0xF2


int Green_PIN = 6;
int RED_PIN = 7;
int robotStates;

float RF_coordinates[6];  // Coordinates read by RF sensor
int matchStatus;          // matchStatus = 1 -- Start
//                                         2 -- Stop
//                                         3 -- Read Coordinates
int text;              // Message read from the RF sensor
float num[6];
NRF24 radio;
char buf[32];
float last_x = 0;
float last_y = 0;

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
const int viewCenter_x = 160;
const int viewCenter_y = 182;
Pixy2I2C pixy;



////////////////
object puck;

///////////////
double prev_x;
double curr_x;
double ka = 1;
double kp = 1;
double aspeed;

int i;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  radio.begin(9, 10);
  pinMode(7, INPUT_PULLUP);
  bool tx = !digitalRead(7);
  radio.setAddress(Address);
  radio.startListening();

  robotStates = Inplace;


  Goal.x = 15;  // selfalize goal location
  Goal.y = 65;
  Goal.theta = NULL;
  //Serial.println("ok");
  pixy.init();
  bno = initial_IMU();
  //Serial.println("Set up");

  puck.colorSig = 1;
  //  pinMode(Green_PIN, OUTPUT);
  //  pinMode(RED_PIN, OUTPUT);
  prev_x = 0;
  //  prev_x = return_IMU_x(&event, bno);
  //  if (prev_x > 180) {
  //    prev_x = curr_x - 360;
  //  }
  //Serial.println("intial angle  " );
  //Serial.println(prev_x);
}

void loop() {
  // put your main code here, to run repeatedly:
  // switch case
  //  Serial.println(robotStates);
  switch (robotStates) {
    case Initialize: {
        //        Serial.println("Initialize");
        /* In this stae, robot will waiting for the signal .
            if the signal is received, change state to stay in front of door

            Need a boolean type function to check sound sensor or RF receiver.
            Return true if signal has been received
        */
        // function
        if (radio.available())
        {
          uint8_t numBytes = radio.read(buf, sizeof(buf));
          //Serial.println(buf);
          //          if (buf[1] == '!' && buf[2] == '!' && buf[3] == '!')
          if (int(buf[1]) == 63 && int(buf[2]) == 63 && int(buf[3]) == 63)
          {
            matchStatus = 1;
          }
        }
        if (matchStatus == 1) {
          robotStates = Inplace;
        }
        else {
          robotStates = Initialize;

        }
        delay(50);
        break;
      }

    case Inplace: {
        // Serial.println("Inplace");
        /* In this state, the robot should move to right front to the goal
            1. adjust the angle
            2. move the place
            3. Don't enter to the goal
        */
        if (radio.available())
        {
          uint8_t numBytes = radio.read(buf, sizeof(buf));
          //Serial.println(buf);
          if (int(buf[1]) == 63 && int(buf[2]) == 63 && int(buf[3]) == 63)
          {

            robotStates = Initialize;
            // Serial.println("Stop");
            break;
          }
          float first_x = int(buf[1] - 48) * 100 + int(buf[2] - 48) * 10 + int(buf[3] - 48) * 1 + int(buf[4] - 48) * 0.1;
          float first_y = int(buf[6] - 48) * 100 + int(buf[7] - 48) * 10 + int(buf[8] - 48) * 1 + int(buf[9] - 48) * 0.1;
          float second_x = int(buf[11] - 48) * 100 + int(buf[12] - 48) * 10 + int(buf[13] - 48) * 1 + int(buf[14] - 48) * 0.1;
          float second_y = int(buf[16] - 48) * 100 + int(buf[17] - 48) * 10 + int(buf[18] - 48) * 1 + int(buf[19] - 48) * 0.1;
          float third_x = int(buf[21] - 48) * 100 + int(buf[22] - 48) * 10 + int(buf[23] - 48) * 1 + int(buf[24] - 48) * 0.1;
          float third_y = int(buf[26] - 48) * 100 + int(buf[27] - 48) * 10 + int(buf[28] - 48) * 1 + int(buf[29] - 48) * 0.1;
          RF_coordinates[0] = first_x;
          RF_coordinates[1] = first_y;
          RF_coordinates[2] = second_x;
          RF_coordinates[3] = second_y;
          RF_coordinates[4] = third_x;
          RF_coordinates[5] = third_y;
        }


        curr_x = return_IMU_x(&event, bno);
        if (curr_x > 180) {
          curr_x = curr_x - 360;
        }
        //Serial.println("current angle  " );
        //Serial.println(curr_x);
        double angle = curr_x - prev_x;
        int aspeed = ka * map(abs(angle), 0, 90, 0, 100) + 30;
        //Serial.println("aspeed :  " );
        // Serial.println(aspeed);
        if (angle > 10 ) {
          //Serial.println("rotate CCW");
          motors.setM2Speed(aspeed); // CCW
          motors.setM1Speed(-aspeed);
          delay(200);
        }
        else if (angle < -10 ) {
          //Serial.println("rotate CW");
          motors.setM2Speed(-aspeed); // CW
          motors.setM1Speed(aspeed);
          delay(200);
        }






        //Serial.println("start");
        isinField = objPosition(puck, i, pixy);  // see the puck
        //Serial.println("Working!");
        if (isinField) {
          if (puck.object_x - viewCenter_x < -10)
          {
            // Move to left
            //            Serial.println("move to left");
            motors.setM2Speed(-150);
            motors.setM1Speed(-150);
            delay(200);

          }
          else if (puck.object_x - viewCenter_x > 10)
          {
            // Move to right
            //Serial.println("move to right");
            motors.setM2Speed(150);
            motors.setM1Speed(150);
            delay(200);

          }
          else
          {
            //Serial.println("right front to the ball");
            motors.setM2Speed(50);
            motors.setM1Speed(50);
            delay(200);
            motors.setM2Speed(50);
            motors.setM1Speed(50);
            delay(200);


          }
        }
        else
        {

          Serial.println("no sure where is it");
          motors.setM2Speed(30);
          motors.setM1Speed(30);
          delay(200);
          motors.setM2Speed(-30);
          motors.setM1Speed(-30);
          delay(200);



        }
        robotStates = Inplace;

        break;
      }


  }
}
