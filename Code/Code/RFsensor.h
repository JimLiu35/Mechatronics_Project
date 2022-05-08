#ifndef RFsensor_H_
#define RFsensor_H_

#include<SPI.h>
#include<NRF24.h>






void initial_RF(NRF24* radio, bool flag);
int RF_receiver(NRF24* radio,float*num);
bool RF_sender(char *content,NRF24* radio);

#endif
