#include "RFsensor.h"
#include "Getcoordinates.h"
void initial_RF(NRF24* radio, bool flag) {
  const int pin_CE = 9;
  const int pin_CSN = 10;
  const int address = 0xD2;
  radio->begin(pin_CE, pin_CSN);
  // flag = 0 sender
  // flag = 1 receiver

  if (flag) {
    radio->listenToAddress(address);
  }
  else {
    radio->setAddress(address);
  }
}






bool RF_sender(char *content, NRF24* radio)
{
  bool res;
  res = radio->broadcast(content);
  return res;
}


int RF_receiver(NRF24* radio, float*num)
{
  int flag;
  while (true) {
//    Serial.println("Working1");
    if (radio->available()) {
      char buf[32];
      uint8_t numBytes = radio->read(buf, sizeof(buf));
      delay(10);
      //char *content = buf;
      flag = start_stop_message(buf);

      if (flag == 3) {
        Getcoordinates(buf, num);
      }
//      Serial.println(buf);
      return flag;
    }
  }
}
