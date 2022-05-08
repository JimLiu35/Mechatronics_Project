#include "pdController.h"

void pdControl(Bot& robot, Bot& obj)
{
  double Setpoint, Input, Output;
  float coordinates[6];  // Coordinates read by RF sensor
  sensors_event_t event;
  Adafruit_BNO055 bno;
  bno = initial_IMU();
  NRF24 radio;
  // Motor
  uint8_t M1DIR = 2;
  uint8_t M1PWM = 3;
  uint8_t M2DIR = 4;
  uint8_t M2PWM = 5;
  DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);
  // for reciver
  int res;
  initial_RF(&radio, 1);

  int res_x, res_y, res_z;

  double aggAngKp = 4, aggAngKi = 0, aggAngKd = 1;
  double consAngKp = 1, consAngKi = 0, consAngKd = 0.25;
  PID myPID(&Input, &Output, &Setpoint, consAngKp, consAngKi, consAngKd, DIRECT);
  myPID.SetMode(AUTOMATIC);

//  Serial.println("PD before loop!!!");
  while (1 < 2) {
    res_x = return_IMU_x(&event, bno);
    //    Serial.println(res_x);
    Serial.println("--------------------------------");
    res = RF_receiver(&radio, coordinates);
    Serial.println(res);
    robot.x = coordinates[0];
    robot.y = coordinates[1];
    robot.theta = res_x;
    if (res_x > 180) {
      robot.theta = res_x - 360;
    }
    obj.x = coordinates[2];
    obj.y = coordinates[3];
    obj.theta = NULL;
    float dist = sqrt(pow((robot.x - obj.x), 2) + pow((robot.y - obj.y), 2));
    //    Serial.print("Robot's x coordinates: ");
    //    Serial.print(robot.x);
    //    Serial.print(", Robot's y coordinates: ");
    //    Serial.println(robot.y);
    //
    //    Serial.print("Puck's x coordinates: ");
    //    Serial.print(obj.x);
    //    Serial.print(", Puck's y coordinates: ");
    //    Serial.println(obj.y);
    if (dist <= 30)
    {
      break;
    }

    Setpoint = (double)atan2((obj.y - robot.y), (obj.x - robot.x)) * 180 / PI;
//    Serial.print("Current input is ");
//    Serial.print(robot.theta);
//    Serial.print(" deg.");
//    Serial.print(" Setpoint is ");
//    Serial.print(Setpoint);
//    Serial.println(" deg.");
    Input = robot.theta;
    int ang_diff = Setpoint - Input;
    if (abs(ang_diff) < 10) {
      myPID.SetTunings(consAngKp, consAngKi, consAngKd);
    }
    else {
      myPID.SetTunings(aggAngKp, aggAngKi, aggAngKd);
    }
    myPID.SetOutputLimits(0, 200);
    myPID.Compute();
    //      int speedAdj = constrain(Output, -400, 400);
//    Serial.print("Current output is ");
//    Serial.println(Output);
    if (ang_diff < -5) {
      // Move to left
      motors.setM2Speed(200 + Output);
      motors.setM1Speed(200 - Output);
    }
    else if (ang_diff > 5) {
      // Move to right
      Serial.println("Moving to right!!!");
      motors.setM2Speed(200 - Output);
      motors.setM1Speed(200 + Output);
    }
    else {
      // Go straight
      motors.setM2Speed(300);
      motors.setM1Speed(300);
    }

  }
}
