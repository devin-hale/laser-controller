#include "i2c.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_i2c.h"
#include "utils.h"

void i2c_init(void) {
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  LL_I2C_DeInit(I2C1);
  LL_I2C_Disable(I2C1);

  LL_I2C_InitTypeDef i2c_init;
  LL_I2C_StructInit(&i2c_init);
  i2c_init.PeripheralMode = LL_I2C_MODE_I2C;
  i2c_init.ClockSpeed = 100000U;
  i2c_init.OwnAddress1 = 0U;
  i2c_init.DutyCycle = LL_I2C_DUTYCYCLE_2;
  i2c_init.TypeAcknowledge = LL_I2C_ACK;
  i2c_init.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;

  if (LL_I2C_Init(I2C1, &i2c_init) != SUCCESS) {
    // Initialization error
    while (1) {
      ErrorFlash();
    };
  }
  LL_I2C_Enable(I2C1);
}

void i2c_write(uint8_t address, uint8_t *data, uint8_t size) {

  LL_I2C_GenerateStartCondition(I2C1);
  while (!LL_I2C_IsActiveFlag_SB(I2C1)) {
  };

  // Send Slave address with Write indication (0)
  // uint8_t address_with_write_bit = (address << 1);
  LL_I2C_TransmitData8(I2C1, address << 1);
  while (!LL_I2C_IsActiveFlag_ADDR(I2C1)) {
    while (LL_I2C_IsActiveFlag_AF(I2C1)) {
    }
  };
  LL_I2C_ClearFlag_ADDR(I2C1);

  // Transmit data
  for (uint8_t i = 0; i < size; i++) {
    while (!LL_I2C_IsActiveFlag_TXE(I2C1))
      ;
    LL_I2C_TransmitData8(I2C1, data[i]);
  }

  // Wait until the transfer is complete (BTF flag should be set)
  while (!LL_I2C_IsActiveFlag_BTF(I2C1)) {
    delay(2);
  }

  LL_I2C_GenerateStopCondition(I2C1);
}

