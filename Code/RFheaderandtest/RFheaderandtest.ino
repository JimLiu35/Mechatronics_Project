#include "RFsensor.h"
const int pin_CE = 9;
const int pin_CSN = 10;
const int address = 0xD2;
NRF24 radio;
// for reciver
char *res;
// for sender
bool result;
char* content = (char*)"this is from xys";


void setup(){
  Serial.begin(115200);
  //test sender
  initial_RF(pin_CE,pin_CSN,&radio,address,0);
  //test receiver
  //initial_RF(pin_CE,pin_CSN,&radio,address,1);
}

void loop(){
  // test sender
  result = RF_sender(content,&radio);
  Serial.println(result);
  
  // test receiver
  //res = RF_receiver(&radio);
  //Serial.println(res);
  
  delay(10);
}
