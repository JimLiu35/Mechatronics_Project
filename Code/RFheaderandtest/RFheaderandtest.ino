#include "RFsensor.h"

NRF24 radio;
// for reciver
char *res;
// for sender
bool result;
char* content = (char*)"!!!!!!!!";

char* content1 = (char*)"g0000:0000,0500:0500,3435:5465";


void setup(){
  Serial.begin(115200);
  //test sender
//  initial_RF(&radio,0);
  //test receiver
  initial_RF(&radio,1);
  delay(1000);
//  result = RF_sender(content,&radio);
  delay(3000);
}

void loop(){
  // test sender
  
//  Serial.println(result);
//  result = RF_sender(content1,&radio);
  
  // test receiver
  res = RF_receiver(&radio);
  if (res == NULL){
    Serial.println("empty");
  }
  else{
    Serial.println(res);
  }
  
}
