#include "unity.h"
#include "TempSensor.h"
#include "mock_sensor.h"

void setUp(void){}

void tearDown(void){}

void test_TempSensor_given_write_data_expect_write_data_into_LCD(void)
{
    registerInfo info = {READ_WRITE, WRITE};

    getRegisterState_ExpectAndReturn(READ_WRITE);
    getReadWriteState_ExpectAndReturn(WRITE);
    float T = getTemperature(27.12);

    TEST_ASSERT_EQUAL(27.12, T);
}

void test_TempSensor_given_read_data_expect_read_data_from_LCD(void)
{
    registerInfo info = {READ_WRITE, READ};

    getRegisterState_ExpectAndReturn(READ_WRITE);
    getReadWriteState_ExpectAndReturn(READ);
    float T = getTemperature(27.12);
}

void test_TempSensor_given_8bitData_expect_write_8BitData_to_LCD(void)
{
    uint8_t data = 0x41;
    registerInfo info = {READ_WRITE, WRITE};

    getRegisterState_ExpectAndReturn(READ_WRITE);
    getReadWriteState_ExpectAndReturn(WRITE);
    write8BitData_ExpectAndReturn(data, data);

    printf("%c", data);
}
