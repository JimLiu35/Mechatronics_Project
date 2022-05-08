#include "RFsensor.h"
//const int pin_CE = 9;
//const int pin_CSN = 10;
//const int address = 0xAA;
NRF24 radio;
// for reciver
char *res;
// for sender
bool result;
char* content = (char*)"g1234:8454,3434:3536,7685:1293";


void setup(){
  Serial.begin(115200);
  //test sender
  initial_RF(&radio,0);
  //test receiver
//  initial_RF(&radio,1);
}

void loop(){
  // test sender
  result = RF_sender(content,&radio);
  Serial.println(result);
  
  // test receiver
//  res = RF_receiver(&radio);
//  if (res == NULL){
////    Serial.println("empty");
//  }
//  else{
//    Serial.println(res);
//  }
  delay(10);
}
