
#include <IRremote.h>
int RECV_PIN = 6;
IRrecv irrecv(RECV_PIN);
decode_results results;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  
  irrecv.enableIRIn();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (irrecv.decode(&results)){
    Serial.println("blocked");
    irrecv.resume();
  }
  delay(10);
}
