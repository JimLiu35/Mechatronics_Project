#include "IMUheader.h"

Adafruit_BNO055 initial_IMU(){
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
  while (bno.begin() != 1){
    ;
  }
  return bno;   
}

double return_IMU_x(sensors_event_t* event,Adafruit_BNO055 bno){
  double result = -1000000;
  bno.getEvent(event, Adafruit_BNO055::VECTOR_EULER);
  result = event->orientation.x;
  return result;
}
double return_IMU_y(sensors_event_t* event,Adafruit_BNO055 bno){
  double result = -1000000;
  bno.getEvent(event, Adafruit_BNO055::VECTOR_EULER);
  result = event->orientation.y;
  return result;
}
double return_IMU_z(sensors_event_t* event,Adafruit_BNO055 bno){
  double result = -1000000;
  bno.getEvent(event, Adafruit_BNO055::VECTOR_EULER);
  result = event->orientation.z;
  return result;
}
