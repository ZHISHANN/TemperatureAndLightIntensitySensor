#ifndef _TEMPSENSOR_H
#define _TEMPSENSOR_H

#include "sensor.h"

typdef struct registerInfo registerInfo;
struct registerInfo{
  register_state reg_state;
};

float getTemperature();

#endif // _TEMPSENSOR_H
