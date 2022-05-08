#include "RFSensor.h"
#include <Pixy2I2C.h>
Pixy2I2C pixy;

NRF24 radio;
// for reciver
char *res;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //  test receiver
  initial_RF(&radio, 1);
  pixy.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;

  pixy.ccc.getBlocks();
  int x = pixy.ccc.blocks[i].m_x;
  Serial.println(x);

  res = RF_receiver(&radio);
  if (res == NULL) {
//    Serial.println("empty");
  }
  else {
    Serial.println(res);
  }
//  delay(10);
//  Serial.println("---------------------");
//
//  res = RF_receiver(&radio);
//  if (res == NULL) {
////    Serial.println("empty");
//  }
//  else {
//    Serial.println(res);
//  }
}
