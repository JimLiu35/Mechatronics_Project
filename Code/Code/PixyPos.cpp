#include "PixyPos.h"
#include <DRV8835MotorShield.h>
#include <Arduino.h>

//object puck;
//boolean isinField;

boolean pixyControl(object& robot, object& puck, Pixy2I2C pixy) {
  uint8_t M1DIR = 2;
  uint8_t M1PWM = 3;
  uint8_t M2DIR = 4;
  uint8_t M2PWM = 5;

  DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);

  const int viewCenter_x = 158;
  const int viewCenter_y = 100;



  boolean catchPuck;
  boolean isinField;


  while (1 < 2) {
    int i;
    isinField = objPosition(puck, i, pixy);
    if (isinField) {
      if (puck.object_x - viewCenter_x < -40)
      {
        // Move to left
              Serial.println("Working1");
        motors.setM2Speed(200);
        motors.setM1Speed(100);
      }
      else if (puck.object_x - viewCenter_x > 40)
      {
        // Move to right
              Serial.println("Working2");
        motors.setM2Speed(100);
        motors.setM1Speed(200);
      }
      else
      {
              Serial.println("Working3");
        motors.setM2Speed(200);
        motors.setM1Speed(200);
      }

      catchPuck = false;
    }
    else
    {
      if (abs(puck.object_x - viewCenter_x) < 40 && puck.object_y > 150) {
        Serial.println("Working4");
        motors.setM2Speed(200);
        motors.setM1Speed(200);
        delay(100);
        catchPuck = puckCaptureCheck();
        if (catchPuck == true)
          return true;
        else
          return false;
      }
    }
  }
}

boolean objPosition(object &obj, int loopIndex, Pixy2I2C pixy) {
  boolean isinField;
  pixy.ccc.getBlocks();
  if (pixy.ccc.blocks[loopIndex].m_signature == obj.colorSig)
  {
    obj.object_x = pixy.ccc.blocks[loopIndex].m_x;
    obj.object_y = pixy.ccc.blocks[loopIndex].m_y;
    isinField = true;
  }
  else
    isinField = false;
  //  Serial.println(isinField);
  return isinField;
}
