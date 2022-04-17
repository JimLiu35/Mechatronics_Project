#ifndef RFsensor_H_
#define RFsensor_H_

#include<SPI.h>
#include<NRF24.h>






void initial_RF(uint8_t pin_CE,uint8_t pin_CSN,NRF24* radio, uint8_t address, bool flag);
char *RF_receiver(NRF24* radio);
bool RF_sender(char *content,NRF24* radio);

#endif
