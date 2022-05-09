#include <SPI.h>
#include <NRF24.h>
#include <Pixy2I2C.h>
#include <DRV8835MotorShield.h>
#include "IMUheader.h"
#include "RFsensor.h"
#include "IRCheck.h"


// Define states
int robotStates;
#define Initialize 0
#define nrfControl 1
#define pixyControl 2
#define Attacking 3
#define Defending 4
#define Stop 5

struct Bot {
  int x;
  int y;
  int theta;
  int colorSig;
  int pixy_x = 0;
  int pixy_y = 0;
};

Bot robotForward;
Bot Puck;
Bot Goal;

Pixy2I2C pixy;

// Motor shells
uint8_t M1DIR = 2;
uint8_t M1PWM = 3;
uint8_t M2DIR = 4;
uint8_t M2PWM = 5;
DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);


// RF Coordinates and match status
//int robotStates;
float RF_coordinates[6];  // Coordinates read by RF sensor
int matchStatus;          // matchStatus = 1 -- Start
//                                         2 -- Stop
//                                         3 -- Read Coordinates


// Pixy center
const int viewCenter_x = 158;
const int viewCenter_y = 100;


NRF24 radio;

// IMU
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
sensors_event_t event;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

boolean capture = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  initial_RF(&radio, 1);
  robotStates = Initialize;
  Goal.x = 230;
  Goal.y = 65;
  Goal.theta = NULL;
  Goal.colorSig = NULL;
  robotForward.colorSig = NULL;
  Puck.colorSig = 2;
  Puck.theta = NULL;
  pixy.init();
  pinMode(IR_LED, OUTPUT);
  bno = initial_IMU();
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
//  Serial.println(robotStates);

  
  switch (robotStates) {
    case Initialize:{
//      Serial.println("Initializing...");
      matchStatus = RF_receiver(&radio, RF_coordinates);                  // Read RF
//      Serial.println(matchStatus);
      if (matchStatus == 1)
      {
        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks) {
          if (pixy.ccc.blocks[i].m_signature == Puck.colorSig) {
            robotStates = pixyControl;
            break;
          }
        }
        else {
          robotStates = nrfControl;
        }
      }
      else
        robotStates = Initialize;
      break;
    }







    case nrfControl:{
//      Serial.println(robotStates);
      matchStatus = RF_receiver(&radio, RF_coordinates);                   // Read RF & Get Coordinates
      if (matchStatus == 2) {
        robotStates = Stop;
      }
      robotForward.x = RF_coordinates[2];
      robotForward.y = RF_coordinates[3];
      robotForward.theta = return_IMU_x(&event, bno);
      if (robotForward.theta > 180) {
        robotForward.theta = robotForward.theta - 360;
      }
      Puck.x = RF_coordinates[4];
      Puck.y = RF_coordinates[5];

      int dx = Puck.x - robotForward.x;
      int dy = Puck.y - robotForward.y;
      int setPoint = atan2(dy, dx) * 180 / PI;
      if (setPoint - robotForward.theta > 10) {
        // Move right
        motors.setM2Speed(-100);
        motors.setM1Speed(100);
      }
      else if (setPoint - robotForward.theta < -10) {
        // Move left
        motors.setM2Speed(100);
        motors.setM1Speed(-100);
      }
      else
      {
        motors.setM2Speed(200);
        motors.setM1Speed(200);
      }
      pixy.ccc.getBlocks();
      if (pixy.ccc.numBlocks) {
        if (pixy.ccc.blocks[i].m_signature == Puck.colorSig) {
          robotStates = pixyControl;
        }
        else {
          robotStates = nrfControl;
        }
      }
      else
      {
        robotStates = nrfControl;
      }
      break;
    }










    case pixyControl:{
//      Serial.println(robotStates);
      matchStatus = RF_receiver(&radio, RF_coordinates);                   // Read RF & Get Coordinates
      if (matchStatus == 2) {
        robotStates = Stop;
      }
      pixy.ccc.getBlocks();
      if (pixy.ccc.numBlocks) {
        if (pixy.ccc.blocks[i].m_signature == Puck.colorSig)
        {
          Puck.pixy_x = pixy.ccc.blocks[i].m_x;
          Puck.pixy_y = pixy.ccc.blocks[i].m_y;
          if (Puck.pixy_x - viewCenter_x <= -30)
          {
            // Moving to the left
            Serial.println("moving to the left");
            motors.setM2Speed(200);
            motors.setM1Speed(100);
          }
          else if (Puck.pixy_x - viewCenter_x >= 30)
          {
            // Moving to the right
            Serial.println("moving to the right");
            motors.setM2Speed(0);
            motors.setM1Speed(-200);
          }
          else
          {
            Serial.println("Center");
            motors.setM2Speed(200);
            motors.setM1Speed(200);
          }
        }
      }
      else {
        // Not detect object, Moving forward
        Serial.println("Checking");
        motors.setM2Speed(200);
        motors.setM1Speed(200);
        capture = puckCaptureCheck();
      }
      if (capture == true) {
        robotStates = Attacking;
      }
      else
      {
        motors.setM2Speed(-200);
        motors.setM1Speed(-200);
        delay(100);
        robotStates = pixyControl;
      }

      break;
    }



    case Attacking:{


      break;
    }
















  }
}
