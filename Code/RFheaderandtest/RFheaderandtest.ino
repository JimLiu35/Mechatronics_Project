#include "RFsensor.h"
const int pin_CE = 9;
const int pin_CSN = 10;
const int address = 0xD2;
char *res;

void setup(){
  Serial.begin(115200);
  
}

void loop(){
  res = RF_receiver(pin_CE,pin_CSN,address);
  Serial.println(res);
}
