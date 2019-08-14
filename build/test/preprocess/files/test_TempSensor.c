#include "build/temp/_test_TempSensor.c"
#include "mock_sensor.h"
#include "TempSensor.h"
#include "unity.h"


void setUp(void){}



void tearDown(void){}



void test_TempSensor_given_write_data_expect_write_data_into_LCD(void)

{

    registerInfo info = {READ_WRITE, WRITE};



    getRegisterState_CMockExpectAndReturn(13, READ_WRITE);

    getReadWriteState_CMockExpectAndReturn(14, WRITE);

    float T = getTemperature(27.12);



    UnityAssertEqualNumber((UNITY_INT)((27.12)), (UNITY_INT)((T)), (

   ((void *)0)

   ), (UNITY_UINT)(17), UNITY_DISPLAY_STYLE_INT);

}



void test_TempSensor_given_read_data_expect_read_data_from_LCD(void)

{

    registerInfo info = {READ_WRITE, READ};



    getRegisterState_CMockExpectAndReturn(24, READ_WRITE);

    getReadWriteState_CMockExpectAndReturn(25, READ);

    float T = getTemperature(27.12);

}



void test_TempSensor_given_8bitData_expect_write_8BitData_to_LCD(void)

{

    uint8_t data = 0x41;

    registerInfo info = {READ_WRITE, WRITE};



    getRegisterState_CMockExpectAndReturn(34, READ_WRITE);

    getReadWriteState_CMockExpectAndReturn(35, WRITE);

    write8BitData_CMockExpectAndReturn(36, data, data);

    float T = getTemperature(27.12);



    printf("%c", data);

}
