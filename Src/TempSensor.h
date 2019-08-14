#ifndef _TEMPSENSOR_H
#define _TEMPSENSOR_H

#include "sensor.h"

typedef struct registerInfo registerInfo;
struct registerInfo{
  register_state reg_state;
  readWrite_state rs_state;
};

float getTemperature(float temp);
uint8_t getLcdDisplay(uint8_t data);

#endif // _TEMPSENSOR_H
