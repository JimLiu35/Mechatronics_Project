#include <PID_v1.h>
#include "IMUheader.h"

#define PIN_OUTPUT 3

sensors_event_t event;
Adafruit_BNO055 bno;
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;

//Specify the links and initial tuning parameters
PID angPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
PID posPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  bno = initial_IMU();
  int dPos_x = 1;
  int dPos_y = 1;
  int dAng = 20;

  angPID.SetMode(AUTOMATIC);
  posPID.SetMode(AUTOMATIC);
}

void loop() {
  // put your main code here, to run repeatedly:
  int res_x, res_y, res_z;
  //  res_x = return_IMU_x(&event,bno);
  res_y = return_IMU_y(&event, bno);
  //  res_z = return_IMU_z(&event,bno);
  // IMU y range? -pi to pi?
  int angGap = abs(dAng - res_y);
  if (angGap < 10)
  { //we're close to setpoint, use conservative tuning parameters
    angPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    angPID.SetTunings(aggKp, aggKi, aggKd);
  }

  angPID.Compute();

  // Change motors speed
  if (res_y < dAng{
    // change motor speeds by using output
  }
  else{
    // change motor speeds by using output
  }
}
