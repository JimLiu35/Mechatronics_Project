#include "soundsensor.h"

int Sensor = A3;

void setup(){
  Serial.begin(115200);
  pinMode(Sensor,INPUT);
  pinMode(13,OUTPUT);
}

void loop(){
  bool res = ifstart(Sensor);
  if(res == 1){
    digitalWrite(13,HIGH);
    Serial.println("start");
  }
  else{
    Serial.println("hold");
    digitalWrite(13,LOW);
  }

}
