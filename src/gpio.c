#include "gpio.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"

void gpio_init(void) {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

  // Servo 1
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);

  // Pot 1 (uses channel 4)
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);

  // Servo 2
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);

  // Pot 2 (uses channel 5)
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ANALOG);

  // PC13 Onboard LED
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_HIGH);
  GPIOC->ODR |= GPIO_ODR_ODR13;

  // PC14 Laser Pointer
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_14, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_HIGH);
  GPIOC->ODR |= GPIO_ODR_ODR14;
  
  // Configure I2C SDA and SCL pins for 1602 LCD I2C Backpack
  LL_GPIO_InitTypeDef gpio_init;
  LL_GPIO_StructInit(&gpio_init);
  gpio_init.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7; // PB6=SCL, PB7=SDA
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &gpio_init);

  GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
  GPIOB->CRL |= (0x3 << GPIO_CRL_CNF6_Pos);
  GPIOB->CRL |= (0x1 << GPIO_CRL_MODE6_Pos);

  GPIOB->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
  GPIOB->CRL |= (0x3 << GPIO_CRL_CNF7_Pos);
  GPIOB->CRL |= (0x1 << GPIO_CRL_MODE7_Pos);
}

