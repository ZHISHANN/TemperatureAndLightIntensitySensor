#ifndef _SENSOR_H
#define _SENSOR_H

#include <stdint.h>

typdef enum{
  write,
  read,
  enable;
  register_select;
} register_state;

register_state getRegisterState();

#endif // _SENSOR_H
