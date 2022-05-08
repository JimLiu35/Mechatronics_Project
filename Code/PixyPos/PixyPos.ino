#include "PixyPos.h"
#include <DRV8835MotorShield.h>
const int IR_RECEIVE_PIN = 6;
const int  IR_LED = 7;

Pixy2I2C pixy;

const int viewCenter_x = 158;
const int viewCenter_y = 100;

object puck;
boolean isinField;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pixy.init();
  puck.colorSig = 5;
  //  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  pinMode(IR_LED, OUTPUT);
}

void loop() {

//bool result = 

}

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

//
//   boolean puckCaptureCheck()
//    {
//      int result = 0;
//      pinMode(IR_LED, OUTPUT);
//      IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
//      for (int i = 0; i <= 9 ; i++)
//      {
//        digitalWrite(IR_LED, HIGH);
//        delay(25);
//        digitalWrite(IR_LED, LOW);
//        delay(25);
//
//        if (IrReceiver.decode())
//        {
//          //Serial.println("nothing");
//          result += 1;
//          IrReceiver.resume();
//        }
//        else
//        {
//          //Serial.println("Captured");
//          IrReceiver.resume();
//          result += 0;
//
//        }
//      }
//
//      if (result == 0) {
//        Serial.println("Captured");
//        return true;
//      }
//      else {
//        Serial.println("Nothing");
//        return false;
//      }
//    }
//  }
