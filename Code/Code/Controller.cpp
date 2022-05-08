#include "Controller.h"


void Control(Bot& robot, Bot& obj)
{
  double Setpoint;
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

  int res_x;



  while (1 < 2) {
    res_x = return_IMU_x(&event, bno);
    //    Serial.println(res_x);
    //    Serial.println("--------------------------------");
    res = RF_receiver(&radio, coordinates);
    robot.x = coordinates[2];
    robot.y = coordinates[3];
    robot.theta = res_x;
    if (res_x > 180) {
      robot.theta = res_x - 360;
    }
    obj.x = coordinates[4];
    obj.y = coordinates[5];
    obj.theta = NULL;
    float dist = sqrt(pow((robot.x - obj.x), 2) + pow((robot.y - obj.y), 2));
    //        Serial.print("Robot's x coordinates: ");
    //        Serial.print(robot.x);
    //        Serial.print(", Robot's y coordinates: ");
    //        Serial.println(robot.y);
    //
    //        Serial.print("Puck's x coordinates: ");
    //        Serial.print(obj.x);
    //        Serial.print(", Puck's y coordinates: ");
    //        Serial.println(obj.y);
    //    if (dist <= 30)
    //    {
    //      break;
    //    }

    Setpoint = (double)atan2((obj.y - robot.y), (obj.x - robot.x)) * 180 / PI;
    Serial.print("Current orientation is ");
    Serial.print(robot.theta);
    //        Serial.print(" deg.");
    //    Serial.print(" Setpoint is ");
    //    Serial.print(Setpoint);
    Serial.println(" deg.");

    double ang_diff = Setpoint - robot.theta;

    //      int speedAdj = constrain(Output, -400, 400);
    Serial.print("Current ang_diffi is ");
    //    Serial.println(Output);
    Serial.println(ang_diff);

    if (ang_diff < -20) {
      // Move to left
      //      int M2Speed = -Output;
      //      int M1Speed = Output;
      //      Serial.print("M1 Speed = ");
      //      Serial.print(M1Speed);
      //      Serial.print(", M2 Speed = ");
      //      Serial.println(M2Speed);
      motors.setM2Speed(100);
      motors.setM1Speed(-100);
    }
    else if (ang_diff > 20) {
      // Move to right
      //      Serial.println("Moving to right!!!");
      //      int M2Speed = -Output;
      //      int M1Speed = Output;
      //      Serial.print("M1 Speed = ");
      //      Serial.print(M1Speed);
      //      Serial.print(", M2 Speed = ");
      //      Serial.println(M2Speed);
      motors.setM2Speed(-100);
      motors.setM1Speed(100);
    }
    else {
      // Go straight
      motors.setM2Speed(200);
      motors.setM1Speed(200);
    }
    //    delay(40);
  }


}
