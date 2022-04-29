#include <Pixy2.h>
Pixy2 pixy;
const int viewCenter_x = 158;
const int viewCenter_y = 100;
boolean isinField = false;
struct object {
  int colorSig;
  int object_x;
  int object_y;
};

object puck;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pixy.init();
  puck.colorSig = 5;
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  pixy.ccc.getBlocks();
  if (pixy.ccc.blocks[i].m_signature == puck.colorSig)
  {
    puck.object_x = pixy.ccc.blocks[i].m_x;
    puck.object_y = pixy.ccc.blocks[i].m_y;
    isinField = true;
  }
  else
    isinField = false;
//  if (abs(puck.object_x - viewCenter_x) <= 5 && abs(puck.object_y - viewCenter_y) <= 5)
//  {
//    Serial.println("At Center!");
//  }
//  else
//    Serial.println("Not at center!");

}
