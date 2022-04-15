#ifndef RFsensor_H_
#define RFsensor_H_

#include<SPI.h>
#include<NRF24.h>

char *RF_receiver(uint8_t pin_CE, uint8_t pin_CSN, uint8_t address);
bool RF_sender(uint8_t pin_CE, uint8_t pin_CSN,uint8_t address ,char *content);

#endif
