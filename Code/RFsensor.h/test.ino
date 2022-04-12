#include "RFsensor.h"
const int pin_CE = ;
const int pin_CSN = ;

void setup(){
  Serail.begin(115200);
}

void loop(){
  res = RF_sender(uint8_t pin_CE, uint8_t pin_CSN,char *address ,char *content);
  Serial.println(res);
  delay(1000);

}
