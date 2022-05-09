#ifndef _IMUHEADER_H
#define _IMUHEADER_H
#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BNO055.h>
#include<utility/imumaths.h>

Adafruit_BNO055 initial_IMU();
double return_IMU_x(sensors_event_t* event,Adafruit_BNO055 bno);
double return_IMU_y(sensors_event_t* event,Adafruit_BNO055 bno);
double return_IMU_z(sensors_event_t* event,Adafruit_BNO055 bno);

#endif
