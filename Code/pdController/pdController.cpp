#include "pdController.h"

int pdControl(object& robot, object& obj, int type)
{
  type = 2;
  double Setpoint, Input, Output;
  sensors_event_t event;
  Adafruit_BNO055 bno;
  bno = initial_IMU();
  NRF24 radio;
  // for reciver
  char *res;
  initial_RF(&radio, 1);

  int res_x, res_y, res_z;

  double aggAngKp = 4, aggAngKi = 0.2, aggAngKd = 1;
  double consAngKp = 1, consAngKi = 0.05, consAngKd = 0.25;
  PID myPID(&Input, &Output, &Setpoint, consAngKp, consAngKi, consAngKd, DIRECT);
  myPID.SetMode(AUTOMATIC);

  while (1 < 2) {
    res_y = return_IMU_y(&event, bno);
    res = RF_receiver(&radio);    // ?? what kind of format does the data in res have??
    robot.x = res[0];
    robot.y = res[1];
    robot.theta = res[2];
    obj.x = res[3];
    obj.y = res[4];
    obj.theta = NULL;

    if (type == 2) {
      Setpoint = (double)atan2((obj.y - robot.y), (obj.x - robot.x));
      Input = robot.theta;
      int ang_diff = Setpoint - Input;
      if (abs(ang_diff) < 10) {
        myPID.SetTunings(consAngKp, consAngKi, consAngKd);
      }
      else {
        myPID.SetTunings(aggAngKp, aggAngKi, aggAngKd);
      }
      myPID.Compute();
      int speedAdj = constrain(Output, -400, 400);
      return speedAdj;
    }

  }
}
