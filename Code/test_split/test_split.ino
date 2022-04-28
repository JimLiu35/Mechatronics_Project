#include "Getcoordinates.h"

float num[6];
char test[] =  "12.2342.34,23.4556.78,12.5635.78";
void setup() {
  Serial.begin(9600);
}
void loop() {
  Getcoordinates(test,num);
  // output
  for(int i =0 ; i<=5; i++){
    Serial.println(num[i]);
    delay(100);
  }
  
}
