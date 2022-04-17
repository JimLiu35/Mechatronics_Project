#include "RFsensor.h"

void initial_RF(uint8_t pin_CE,uint8_t pin_CSN,NRF24* radio, uint8_t address, bool flag){
  radio->begin(pin_CE,pin_CSN);
  // flag = 0 sender
  // flag = 1 receiver
  
  if (flag){
    radio->listenToAddress(address);
  }
  else{
    radio->setAddress(address);
  }
}






bool RF_sender(char *content,NRF24* radio)
{
  bool res;
  res = radio->broadcast(content);
  return res;
}


char *RF_receiver(NRF24* radio)
{
  
  if(radio->available()){
    char buf[32];
    uint8_t numBytes = radio->read(buf,sizeof(buf));
    //Serial.println(numBytes);
    char *content = buf;
    return content;
  }
}
