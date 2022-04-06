//this is test code for Receiver code
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const int Pin_CE = 7;
const int Pin_CSN = 8;

const byte address[6] = "00001";

RF24 radio(Pin_CE,Pin_CSN);

void setup() {
  
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()){
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }

}
