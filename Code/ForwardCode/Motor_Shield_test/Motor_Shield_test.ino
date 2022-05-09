#include <DRV8835MotorShield.h>
#include <SPI.h>
#include <NRF24.h>
#include <Pixy2I2C.h>
#include "IMUheader.h"
#include "RFsensor.h"
#include "IRCheck.h"

uint8_t M1DIR = 2;
uint8_t M1PWM = 3;
uint8_t M2DIR = 4;
uint8_t M2PWM = 5;


DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);
// Define states
int robotStates;
#define Initialize 0
#define nrfControl 1
#define pixyControl 2
#define Attacking 3
#define Defending 4
#define Stop 5

#define Address 0xF2

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
float coordinates[6];
int speed = 200;
char buf[32];
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  //   initialize nrf sensor
  radio.begin(9, 10);
  pinMode(7, INPUT_PULLUP);
  bool tx = !digitalRead(7);
  radio.setAddress(Address);
  radio.startListening();



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

void loop()
{
  int i;
  //  motors.setM2Speed(speed - 100);   // right motor viewing from the rear
  //  motors.setM1Speed(speed);     // left motor viewing from the rear
  delay(2);

  switch (robotStates) {
    case Initialize: {
        //      Serial.println("Initialize!");
        if (radio.available())
        {
          //          Serial.println("Im here!");
          uint8_t numBytes = radio.read(buf, sizeof(buf));
          Serial.println(buf);
          if (buf[1] == '!' && buf[2] == '!' && buf[3] == '!')
          {
            Serial.println("Start signal!");
            matchStatus = 1;
          }
        }
        if (matchStatus == 1)
        {
          pixy.ccc.getBlocks();

          if (pixy.ccc.numBlocks)
          {
            if (pixy.ccc.blocks[i].m_signature == Puck.colorSig)
            {
              robotStates = pixyControl;
              break;
            }
            else
            {
              robotStates = Initialize;
              break;
            }
          }
          else
          {
            // Enable or Disable nrfControl
//            Serial.println("Here");
//            motors.setM1Speed(80);
//            motors.setM2Speed(80);
//            delay(100);
//            robotStates = Initialize;
            robotStates = nrfControl;
            break;
          }
        }
        else
        {
          robotStates = Initialize;
          break;
        }
      }

    case nrfControl: {
        Serial.println("nrfControl");
        if (radio.available())
        {
          uint8_t numBytes = radio.read(buf, sizeof(buf));
          if (buf[1] == '?' && buf[2] == '?' && buf[3] == '?')
          {
            robotStates = Stop;
          }
          float first_x = int(buf[1] - 48) * 100 + int(buf[2] - 48) * 10 + int(buf[3] - 48) * 1 + int(buf[4] - 48) * 0.1;
          float first_y = int(buf[6] - 48) * 100 + int(buf[7] - 48) * 10 + int(buf[8] - 48) * 1 + int(buf[9] - 48) * 0.1;
          float second_x = int(buf[11] - 48) * 100 + int(buf[12] - 48) * 10 + int(buf[13] - 48) * 1 + int(buf[14] - 48) * 0.1;
          float second_y = int(buf[16] - 48) * 100 + int(buf[17] - 48) * 10 + int(buf[18] - 48) * 1 + int(buf[19] - 48) * 0.1;
          float third_x = int(buf[21] - 48) * 100 + int(buf[22] - 48) * 10 + int(buf[23] - 48) * 1 + int(buf[24] - 48) * 0.1;
          float third_y = int(buf[26] - 48) * 100 + int(buf[27] - 48) * 10 + int(buf[28] - 48) * 1 + int(buf[29] - 48) * 0.1;
          coordinates[0] = first_x;
          coordinates[1] = first_y;
          coordinates[2] = second_x;
          coordinates[3] = second_y;
          coordinates[4] = third_x;
          coordinates[5] = third_y;
        }
        robotForward.x = coordinates[0];
        robotForward.y = coordinates[1];
        Puck.x = coordinates[4];
        Puck.y = coordinates[5];
        robotForward.theta = return_IMU_x(&event, bno);
        
        if (robotForward.theta > 180) {
          robotForward.theta = robotForward.theta - 360;
        }
        
        float dx = Puck.x - robotForward.x;
        float dy = Puck.y - robotForward.y;
        float setPoint = atan2(dy, dx) * 180 / PI;
        float ang_diff = setPoint - robotForward.theta;
//        Serial.println(setPoint);
        if (ang_diff > 8)
        {
          // move to right
          motors.setM2Speed(-80);
          motors.setM1Speed(80);
          delay(100);
        }
        else if (ang_diff < -8) {
          // Move left
          motors.setM2Speed(80);
          motors.setM1Speed(-80);
          delay(100);
        }
        else
        {
          motors.setM2Speed(100);
          motors.setM1Speed(100);
          delay(100);
        }
        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks)
        {
          if (pixy.ccc.blocks[i].m_signature == Puck.colorSig &&  pixy.ccc.blocks[i].m_width >= 80)
          {
            robotStates = pixyControl;
            break;
          }
        }
        else {
          robotStates = nrfControl;
          break;
        }


      }



    case pixyControl: {
        if (radio.available())
        {
          uint8_t numBytes = radio.read(buf, sizeof(buf));
          if (buf[1] == '?' && buf[2] == '?' && buf[3] == '?')
          {
            robotStates = Stop;
          }
        }
        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks) {
          if (pixy.ccc.blocks[i].m_signature == Puck.colorSig && pixy.ccc.blocks[i].m_width >= 50)
          {
            Puck.pixy_x = pixy.ccc.blocks[i].m_x;
            Puck.pixy_y = pixy.ccc.blocks[i].m_y;
            int width = pixy.ccc.blocks[i].m_width;
            if (Puck.pixy_x - viewCenter_x <= -30)
            {
              // Moving to the left
              Serial.println("moving to the left");
              motors.setM2Speed(100);
              motors.setM1Speed(-100);
              delay(100);
            }
            else if (Puck.pixy_x - viewCenter_x >= 30)
            {
              // Moving to the right
              Serial.println("moving to the right");
              motors.setM2Speed(-100);
              motors.setM1Speed(100);
              delay(100);
            }
            else
            {
              Serial.println("Center");
              motors.setM2Speed(200);
              motors.setM1Speed(200);
              delay(100);
            }
          }
          if (Puck.pixy_y > 170 && pixy.ccc.blocks[i].m_width > 160)
          {
            capture = true;
          }
        }
        else {
          motors.setM2Speed(200);
          motors.setM1Speed(200);
          delay(200);
        }
        if (capture == true) {
          robotStates = Attacking;
        }
        else
        {
          robotStates = pixyControl;
        }
        break;
      }


    case Attacking: {
        Serial.println("Attack");
        if (radio.available())
        {
          uint8_t numBytes = radio.read(buf, sizeof(buf));
          if (buf[1] == '?' && buf[2] == '?' && buf[3] == '?')
          {
            robotStates = Stop;
          }
          float first_x = int(buf[1] - 48) * 100 + int(buf[2] - 48) * 10 + int(buf[3] - 48) * 1 + int(buf[4] - 48) * 0.1;
          float first_y = int(buf[6] - 48) * 100 + int(buf[7] - 48) * 10 + int(buf[8] - 48) * 1 + int(buf[9] - 48) * 0.1;
          float second_x = int(buf[11] - 48) * 100 + int(buf[12] - 48) * 10 + int(buf[13] - 48) * 1 + int(buf[14] - 48) * 0.1;
          float second_y = int(buf[16] - 48) * 100 + int(buf[17] - 48) * 10 + int(buf[18] - 48) * 1 + int(buf[19] - 48) * 0.1;
          float third_x = int(buf[21] - 48) * 100 + int(buf[22] - 48) * 10 + int(buf[23] - 48) * 1 + int(buf[24] - 48) * 0.1;
          float third_y = int(buf[26] - 48) * 100 + int(buf[27] - 48) * 10 + int(buf[28] - 48) * 1 + int(buf[29] - 48) * 0.1;
          coordinates[0] = 100;
          coordinates[1] = 50;
          coordinates[2] = second_x;
          coordinates[3] = second_y;
          coordinates[4] = third_x;
          coordinates[5] = third_y;
        }
        //        robotForward.x = coordinates[0];
        //        robotForward.y = coordinates[1];
        robotForward.x = 110;
        robotForward.y = 30;
        Puck.x = coordinates[4];
        Puck.y = coordinates[5];
        robotForward.theta = return_IMU_x(&event, bno);
        if (robotForward.theta > 180) {
          robotForward.theta = robotForward.theta - 360;
        }
        float dx = Goal.x - robotForward.x;
        float dy = Goal.y - robotForward.y;
        float setPoint = atan2(dy, dx) * 180 / PI;
        float ang_diff = setPoint - robotForward.theta;
        if (ang_diff > 8)
        {
          // move to right
          motors.setM2Speed(-80);
          motors.setM1Speed(80);
          delay(100);
        }
        else if (ang_diff < -8) {
          // Move left
          motors.setM2Speed(80);
          motors.setM1Speed(-80);
          delay(100);
        }
        else
        {
          motors.setM2Speed(100);
          motors.setM1Speed(100);
          delay(100);
        }
        capture = true;
        if (capture == true)
        {
          robotStates = Attacking;
          break;
        }
        else
        {
          robotStates = pixyControl;
          break;
        }








      }


      case Stop :{
          motors.setM2Speed(0);
          motors.setM1Speed(0);
          delay(20000);
          robotState = Stop;
          break;
      }




  }













}
