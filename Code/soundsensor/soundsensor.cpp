#include "soundsensor.h"
#include <Arduino.h>

bool ifstart(int Sensor){
  int status_sensor;
  int clap = 0;
  long detection_range_start = 0;
  long detection_range = 0;
  bool status_lights = false;

  detection_range_start = millis();
  detection_range = detection_range_start;
  while(millis() - detection_range_start <= 1000)
  {
    status_sensor = digitalRead(Sensor);
    if (status_sensor == 0 && millis()-detection_range >= 25)
    {
        detection_range = millis();
        clap++;
    }
    
    if (clap >= 1)
    {
      status_lights = true;
      return status_lights;
    }
  }
  status_lights = false;
  return status_lights;
}
