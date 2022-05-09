#include "PixyPos_defense.h"
#include <Arduino.h>


boolean objPosition(object & obj, int loopIndex, Pixy2I2C pixy) {
  boolean isinField;
  pixy.ccc.getBlocks();
  if (pixy.ccc.blocks[loopIndex].m_signature == obj.colorSig)
  {
    obj.object_x = pixy.ccc.blocks[loopIndex].m_x;
        Serial.println("obj.object_x");
        Serial.println(obj.object_x);
    obj.object_y = pixy.ccc.blocks[loopIndex].m_y;
        Serial.println("obj.object_y");
        Serial.println(obj.object_y);
    isinField = true;
  }
  else
    isinField = false;
  //  Serial.println(isinField);
  return isinField;
}
