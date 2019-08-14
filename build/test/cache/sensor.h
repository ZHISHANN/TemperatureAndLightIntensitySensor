









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
