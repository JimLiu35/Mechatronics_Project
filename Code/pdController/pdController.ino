//#include <PID_v1.h>
//#include "IMUheader.h"
#include "pdController.h"



sensors_event_t event;
Adafruit_BNO055 bno;
double Setpoint, Input, Output;
object robot;
object puck;

//Define the aggressive and conservative Tuning Parameters
//double aggKp = 4, aggKi = 0.2, aggKd = 1;
//double consKp = 1, consKi = 0.05, consKd = 0.25;

//Specify the links and initial tuning parameters
//PID pidCon(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
//PID angPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
//PID posPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup() {
  Serial.begin(115200);
  
  // put your setup code here, to run once:
//  Serial.begin(115200);
//  bno = initial_IMU();
//  int dPos_x = 1;
//  int dPos_y = 1;
//  int dAng = 20;
//
//  angPID.SetMode(AUTOMATIC);
//  posPID.SetMode(AUTOMATIC);
}

void loop() {
  pdControl(robot,puck,2);

}
