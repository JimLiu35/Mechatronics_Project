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

boolean borderCheck(const double x, const double y)
{
  boolean atBorder = false;
  double botRadius = 8;         // robot radius in mm
  double xBorder1 = 0;          
  double xBorder2 = 2100;       // in mm
  double yBorder1 = 0;
  double yBorder2 = 1050;       // in mm
  if (x - botRadius <= xBorder1 || x + botRadius >= xBorder2)
    atBorder = true;
  else if (y - botRadius <= yBorder1 || y + botRadius >= yBorder2)
    atBorder = true;
  else
    atBorder = false;

  return atBorder;
}
