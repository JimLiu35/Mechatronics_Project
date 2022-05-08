#include "PixyPos_defense.h"
//#include <Pixy2.h>
#include <Pixy2I2C.h>
#include <DRV8835MotorShield.h>
uint8_t M1DIR = 2;
uint8_t M1PWM = 3;
uint8_t M2DIR = 4;
uint8_t M2PWM = 5;
DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);

Pixy2I2C pixy;

const int viewCenter_x = 157;
const int viewCenter_y = 203;

object puck;
boolean isinField;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pixy.init();
  puck.colorSig = 1;
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  isinField = objPosition(puck, i, pixy);

//  Serial.println(isinField);

  if (isinField) {
    if (puck.object_x - viewCenter_x < -10)
    {
      // Move to left
//      Serial.println("move to left");
      motors.setM2Speed(-200);
      motors.setM1Speed(-200);
    }
    else if (puck.object_x - viewCenter_x > 10)
    {
      // Move to right
//      Serial.println("move to right");
      motors.setM2Speed(200);
      motors.setM1Speed(200);
    }
    else
    {
//      Serial.println("right front to the ball");
      motors.setM2Speed(0);
      motors.setM1Speed(0);

    }
  }
  else
  {
//     Serial.println("wondering");
    motors.setM2Speed(100);
    motors.setM1Speed(100);
    delay(200);
    motors.setM2Speed(-100);
    motors.setM1Speed(-100);
    delay(200);
  }

}
