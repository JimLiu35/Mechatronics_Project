#ifndef RFsensor_H_
#define RFsensor_H_

#include<SPI.h>
#include<NRF24.h>
#define address 0xD2
char RF_receiver(uint8_t pin_CE, uint8_t pin_CSN, char *address);
bool RF_sender(uint8_t pin_CE, uint8_t pin_CSN,char *address ,char *content);

#endif
