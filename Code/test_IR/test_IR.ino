#include <IRremote.h>
const byte IR_RECEIVE_PIN = 6;
const int IR_LED = 7;
boolean puckCaptured = false;
int res = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("IR Receive test");
//  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  pinMode(IR_LED, OUTPUT);
}

void loop() {
  int res = 0;
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
      res = res + 1;
      IrReceiver.resume();
    }
    else
    {
      //Serial.println("Captured");
      IrReceiver.resume();
      res = res + 0;

    }
  }

  if (res == 0) {
    Serial.println("Captured");
    puckCaptured = true;
  }
  else {
    Serial.println("Nothing");
    puckCaptured = false;
  }

  res = 0;
}
