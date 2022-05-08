#include "PixyPos.h"
#include <DRV8835MotorShield.h>
#include <Arduino.h>

//object puck;
//boolean isinField;

boolean pixyControl(object& robot, object& puck) {
  uint8_t M1DIR = 2;
  uint8_t M1PWM = 3;
  uint8_t M2DIR = 4;
  uint8_t M2PWM = 5;

  DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);

  const int viewCenter_x = 158;
  const int viewCenter_y = 100;


  Pixy2I2C pixy;
  boolean catchPuck;
  boolean isinField;
  puck.colorSig = 5;
  int i;
  while (1 < 2) {
    isinField = objPosition(puck, i, pixy);
    if (isinField) {
      if (puck.object_x - viewCenter_x < -40)
      {
        // Move to left
        //      Serial.println("Working1");
        motors.setM2Speed(200);
        motors.setM1Speed(100);
      }
      else if (puck.object_x - viewCenter_x > 40)
      {
        // Move to right
        //      Serial.println("Working2");
        motors.setM2Speed(100);
        motors.setM1Speed(200);
      }
      else
      {
        //      Serial.println("Working3");
        motors.setM2Speed(200);
        motors.setM1Speed(200);
      }
      catchPuck = false;
    }
    else
    {
      if (abs(puck.object_x - viewCenter_x) < 40 && puck.object_y > 150) {
        //      Serial.println("Working4");
        motors.setM2Speed(200);
        motors.setM1Speed(200);
        delay(1000);
        bool puckCapture = puckCaptureCheck()
//       capture the puck
        if (puckCapture == true){
        motors.setM2Speed(0);
        motors.setM1Speed(0);
        delay(3000);
        catchPuck = true;
        else 
        catchPuck = false;
        

      }
    }
    if (catchPuck == true) {
      return catchPuck;
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



  
boolean puckCaptureCheck()
{
  int result = 0;
  pinMode(IR_LED, OUTPUT);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  for (int i = 0; i <= 9 ; i++)
  {
    digitalWrite(IR_LED, HIGH);
    delay(25);
    digitalWrite(IR_LED, LOW);
    delay(25);

    if (IrReceiver.decode())
    {
      //Serial.println("nothing");
      result += 1;
      IrReceiver.resume();
    }
    else
    {
      //Serial.println("Captured");
      IrReceiver.resume();
      result += 0;

    }
  }

  if (result == 0) {
    Serial.println("Captured");
    return true;
  }
  else {
    Serial.println("Nothing");
    return false;
  }
}
}
