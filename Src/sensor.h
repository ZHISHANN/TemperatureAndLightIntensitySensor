#ifndef _SENSOR_H
#define _SENSOR_H

#include <stdint.h>
#include <stdio.h>

typedef enum{
  READ_WRITE,
  ENABLE,
  REG_SELECT,
} register_state;

typedef enum{
  READ,
  WRITE,
} readWrite_state;

register_state getRegisterState();
readWrite_state getReadWriteState();
uint8_t write8BitData(uint8_t data);

#endif // _SENSOR_H
