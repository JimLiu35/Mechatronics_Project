#include "PixyPos.h"
#include <DRV8835MotorShield.h>
//uint8_t M1DIR = 2;
//uint8_t M1PWM = 3;
//uint8_t M2DIR = 4;
//uint8_t M2PWM = 5;
//DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);

Pixy2I2C pixy;

const int viewCenter_x = 158;
const int viewCenter_y = 100;

object puck;
boolean isinField;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  pixy.init();
//  puck.colorSig = 5;
}

void loop() {
  // put your main code here, to run repeatedly:
//  int i;
//  isinField = objPosition(puck, i, pixy);
//    Serial.println(puck.object_y);

  //  Serial.println(isinField);
//  if (isinField) {
//    if (puck.object_x - viewCenter_x < -40)
//    {
//      // Move to left
////      Serial.println("Working1");
//      motors.setM2Speed(200);
//      motors.setM1Speed(100);
//    }
//    else if (puck.object_x - viewCenter_x >40)
//    {
//      // Move to right
////      Serial.println("Working2");
//      motors.setM2Speed(100);
//      motors.setM1Speed(200);
//    }
//    else
//    {
////      Serial.println("Working3");
//      motors.setM2Speed(200);
//      motors.setM1Speed(200);
//    }
//  }
//  else
//  {
//    if (abs(puck.object_x - viewCenter_x) < 40 && puck.object_y > 150) {
////      Serial.println("Working4");
//      motors.setM2Speed(200);
//      motors.setM1Speed(200);
//      delay(1000);
//      motors.setM2Speed(0);
//      motors.setM1Speed(0);
//      delay(3000);
//    }
//  }


}
