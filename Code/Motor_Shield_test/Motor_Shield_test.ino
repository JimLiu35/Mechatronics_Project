#include <DRV8835MotorShield.h>

uint8_t M1DIR =2;
uint8_t M1PWM =3;
uint8_t M2DIR = 4;
uint8_t M2PWM = 5;


DRV8835MotorShield motors = DRV8835MotorShield(M1DIR,M1PWM,M2DIR,M2PWM);
int speed = -200;
void setup()
{
  
}

void loop()
{
    motors.setM2Speed(speed);
    motors.setM1Speed(speed);
    delay(2);
}