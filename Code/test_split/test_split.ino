#include "Getcoordinates.h"

float num[6];
char test[] =  "?????";
//"!!!!!";
//"b0000:1111,2222:3333,8888:9999";
//"12.2342.34,23.4556.78,12.5635.78";
int flag;
void setup() {
  Serial.begin(115200);
}
void loop() {
  flag = start_stop_message(test);
  if(flag == 1){
    Serial.println("start!");
  }
  else if(flag == 2){
    Serial.println("stop!");
  }
  else{
    Getcoordinates(test,num);
    // output
    for(int i =0 ; i<=5; i++){
      Serial.println(num[i]);
    }
  }
  delay(100);
  
}
