#include "unity.h"
#include "TempSensor.h"
#include "sensor.h"

float getTemperature(float temp)
{
  register_state state;
  state = getRegisterState();

  readWrite_state rwState;
  rwState = getReadWriteState();

  switch(state)
  {
    case READ_WRITE:  if(rwState == WRITE)
                        return temp;
                      else
                        printf("Temperature : %.2f C", temp);
                      break;
    case ENABLE:  return 1;
                  break;
    case REG_SELECT: return 1;
                     break;
    default: return 0;
  }
}

uint8_t getLcdDisplay(uint8_t data)
{

}
