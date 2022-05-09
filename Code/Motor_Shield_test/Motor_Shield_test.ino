#include <DRV8835MotorShield.h>

uint8_t M1DIR = 2;
uint8_t M1PWM = 3;
uint8_t M2DIR = 4;
uint8_t M2PWM = 5;


DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);
int speed = 200;
void setup()
{

}

void loop()
{
  //  motors.setM2Speed(speed);     // right motor viewing from the rear  // rear wheel positive move right
  //  motors.setM1Speed(speed);     // left motor viewing from the rear
  //  delay(1000);
  //  motors.setM2Speed(-speed);     // right motor viewing from the rear
  //  motors.setM1Speed(-speed);     // left motor viewing from the rear
  //  delay(1000);

  motors.setM2Speed(-speed); // CW as speed is +
  motors.setM1Speed(speed);
  delay(1000);
}
