#include "PixyPos_defense.h"
#include <DRV8835MotorShield.h>
#include <Arduino.h>
//#include <IRremote.h>

boolean pixyControl(object& robot, object& puck) {
  uint8_t M1DIR = 2;
  uint8_t M1PWM = 3;
  uint8_t M2DIR = 4;
  uint8_t M2PWM = 5;


  DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);

  const int viewCenter_x = 157;
  const int viewCenter_y = 203;
  Pixy2I2C pixy;

  boolean isinField;
  puck.colorSig = 1;
  int i;
  while (1<2) {

    isinField = objPosition(puck, i, pixy);  // see the puck

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
      //     Serial.println("wondering");
      //      motors.setM2Speed(50);
      //      motors.setM1Speed(50);
      //      delay(1000);
      //      motors.setM2Speed(-50);
      //      motors.setM1Speed(-50);
      //      delay(1000);
      //      break;
      /* cannot seen then listem to RF instruction */

    }
    if (isinField == false) {
      return isinField;
    }
  }

}


boolean objPosition(object & obj, int loopIndex, Pixy2I2C pixy) {
  boolean isinField;
  pixy.ccc.getBlocks();
  if (pixy.ccc.blocks[loopIndex].m_signature == obj.colorSig)
  {
    obj.object_x = pixy.ccc.blocks[loopIndex].m_x;
    //    Serial.println("obj.object_x");
    //    Serial.println(obj.object_x);
    obj.object_y = pixy.ccc.blocks[loopIndex].m_y;
    //    Serial.println("obj.object_y");
    //    Serial.println(obj.object_y);
    isinField = true;
  }
  else
    isinField = false;
  //  Serial.println(isinField);
  return isinField;
}
