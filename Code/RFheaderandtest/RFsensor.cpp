#include "RFsensor.h"
bool RF_sender(uint8_t pin_CE,uint8_t pin_CSN,uint8_t address ,char *content)
{
  NRF24 radio;
  //Serial.begin(115200);
  radio.begin(pin_CE,pin_CSN);
  radio.setAddress(address);
  bool res;
  res = radio.broadcast(content);
  return res;
}

char *RF_receiver(uint8_t pin_CE, uint8_t pin_CSN, uint8_t address)
{
  NRF24 radio;
  radio.begin(pin_CE,pin_CSN);
  radio.listenToAddress(address);
  delay(20);
  if(radio.available()){
    Serial.println("ok");
    char buf[32];
    uint8_t numBytes = radio.read(buf,sizeof(buf));
    //Serial.println(numBytes);
    char *content = buf;
    return content;
  }
}
