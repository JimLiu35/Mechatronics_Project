#include "PixyPos.h"
#include <DRV8835MotorShield.h>
const int IR_RECEIVE_PIN = 6;
const int  IR_LED = 7;

Pixy2I2C pixy;

const int viewCenter_x = 158;
const int viewCenter_y = 100;

object puck;
object robot;
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

if (pixyControl(robot, puck)){
// catch the ball
// attack robot go to the goal
// defense robot 

}
}
