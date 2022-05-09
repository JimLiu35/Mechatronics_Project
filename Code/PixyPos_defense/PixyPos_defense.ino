#include "PixyPos_defense.h"
//#include <Pixy2.h>
#include <Pixy2I2C.h>
#include <DRV8835MotorShield.h>
int Green_PIN = 6;
int RED_PIN = 7;


Pixy2I2C pixy;
//
//const int viewCenter_x = 157;
//const int viewCenter_y = 203;

object puck;
object defenseRobot;
boolean isinField;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pixy.init();
  puck.colorSig = 1;
  pinMode(Green_PIN,OUTPUT);
  pinMode(RED_PIN,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (pixyControl(defenseRobot, puck)){
  Serial.print("True");
  digitalWrite(Green_PIN,HIGH);
  digitalWrite(RED_PIN,LOW);
  }
  else
  {
  Serial.print("False");
  digitalWrite(Green_PIN,LOW);
  digitalWrite(RED_PIN,HIGH);
  }
// catch the ball
// attack robot go to the goal
// defense robot 

}
