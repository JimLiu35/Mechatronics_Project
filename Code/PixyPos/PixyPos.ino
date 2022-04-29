#include "PixyPos.h"
#include <Pixy2.h>
#include <DRV8835MotorShield.h>
uint8_t M1DIR = 2;
uint8_t M1PWM = 3;
uint8_t M2DIR = 4;
uint8_t M2PWM = 5;
DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);

Pixy2 pixy;

const int viewCenter_x = 158;
const int viewCenter_y = 100;

object puck;
boolean isinField;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pixy.init();
  puck.colorSig = 5;
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
      motors.setM2Speed(400);
      motors.setM1Speed(200);
    }
    else if (puck.object_x - viewCenter_x > 10)
    {
      // Move to right
      motors.setM2Speed(200);
      motors.setM1Speed(400);
    }
    else
    {
      motors.setM2Speed(400);
      motors.setM1Speed(400);
    }
  }
  else
  {
    motors.setM2Speed(0);
    motors.setM1Speed(-0);
  }

}
