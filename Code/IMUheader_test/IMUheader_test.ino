#include "IMUheader.h"
sensors_event_t event;
Adafruit_BNO055 bno;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  bno = initial_IMU();
}

void loop() {
  double res_x,res_y,res_z;
  res_x = return_IMU_x(&event,bno);
  res_y = return_IMU_y(&event,bno);
  res_z = return_IMU_z(&event,bno);
  Serial.print("x: ");
  Serial.print(res_x);
  Serial.print(" y: ");
  Serial.print(res_y);
  Serial.print(" z: ");
  Serial.println(res_z);
}
