#include "RFsensor.h"
//const int pin_CE = 9;
//const int pin_CSN = 10;
//const int address = 0xD2;
NRF24 radio;
// for reciver
char *res;
// for sender
bool result;
char* content = (char*)"12.2342.34,23.4556.78,12.5635.78";


void setup(){
  Serial.begin(115200);
  //test sender
//  initial_RF(&radio,0);
  //test receiver
  initial_RF(&radio,1);
}

void loop(){
  // test sender
//  result = RF_sender(content,&radio);
//  Serial.println(result);
  
  // test receiver
  res = RF_receiver(&radio);
  if (res == NULL){
//    Serial.println("empty");
  }
  else{
    Serial.println(res);
  }
  delay(10);
}
