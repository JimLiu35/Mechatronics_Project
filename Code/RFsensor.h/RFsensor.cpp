#include "RFsensor.h"
bool RF_sender(uint8_t pin_CE,uint8_t pin_CSN,char *address ,char *content)
{
  NRF24 radio;
  //Serial.begin(115200);
  radio.begin(pin_CE,pin_CSN);
  radio.setAddress(address);
  res = radio.broadcast(content);
  return res;
}

char RF_receiver(uint8_t pin_CE, uint8_t pin_CSN, char *address)
{
  NRF24 radio;
  radio.begin(pin_CE,pin_CSN);
  radio.listenToAddress(address);
  if(radio.available()){
    char buf[32];
    uint8_ numBytes = radio.read(buf,sizeof(buf));
    content = buf;
  else
    content = NULL;
  }
  return content;
}
