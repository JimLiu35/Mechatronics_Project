#include "IRCheck.h"

boolean puckCaptureCheck()
{
  int result = 0;
  pinMode(IR_LED, OUTPUT);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  for (int i = 0; i <= 9 ; i++)
  {
    digitalWrite(IR_LED, HIGH);
    delay(25);
    digitalWrite(IR_LED, LOW);
    delay(25);

    if (IrReceiver.decode())
    {
      //Serial.println("nothing");
      result += 1;
      IrReceiver.resume();
    }
    else
    {
      //Serial.println("Captured");
      IrReceiver.resume();
      result += 0;

    }
  }

  if (result == 0) {
    Serial.println("Captured");
    return true;
  }
  else {
    Serial.println("Nothing");
    return false;
  }
}
