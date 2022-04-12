#include "Code.h"

// Variable declaration
int currentTime;
int elapsedTime;
int previousTime;
double lastError;
const int kp = 0;
const int kd = 0;

double pidControl(const double setPoint, const double currentPosition, const double targetPosition)
{
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

  double error = setPoint - currentPosition;                        // error for the proportional control
  double error_dot = (error - lastError) / elapsedTime;             // error for the derivative control

  //  double output = (float) m*pow(l,2) - (g/l*sin(currentPosition)) + kf/(m*pow(l,2))*(currentPosition/(float)(elapsedTime)) - kp * error + kd * error_dot;               // output signal for the PD controller
  double output = kp * error + kd * error_dot;
  lastError = error;
  previousTime = currentTime;

  return output;
}
