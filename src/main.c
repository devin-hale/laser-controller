#include "stm32f103xb.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_tim.h"
#include <stdint.h>

void ErrorFlash(void);

void systick_init(void) {
  static int ms = 72000;
  SysTick->LOAD = ms - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

void delay(int ms) {
  for (int i = 0; i < ms; i++) {
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
      ;
  }
}
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
      // ErrorFlash();
    };
  }
  LL_I2C_Enable(I2C1);
}

void gpio_init(void) {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

  // Servo 1
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);

  // Servo 2
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);

  // PC13 Onboard LED
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_HIGH);
  GPIOC->ODR |= GPIO_ODR_ODR13;

  // PC14 Pan Clockwise Button
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_14, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_14, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_HIGH);

  // PC15 Pan Counter clockwise button
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_15, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_15, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_HIGH);

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
  ErrorFlash();
  delay(1000);
}

void pwm_init(void) {
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  LL_TIM_SetPrescaler(
      TIM2, 72 - 1); // Prescaler to get 1 MHz timer clock (72 MHz / 72)
  LL_TIM_SetAutoReload(TIM2, 20000 - 1); // Auto-reload for 20 ms period (50 Hz)

  // Channel 1
  LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);

  // Channel 2
  LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);

  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_EnableCounter(TIM2);
  LL_TIM_GenerateEvent_UPDATE(TIM2);
}

void SystemClock_Config(void) {
  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

  /* Enable HSE */
  LL_RCC_HSE_Enable();
  while (LL_RCC_HSE_IsReady() != 1)
    ;

  /* Configure and enable PLL */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();
  while (LL_RCC_PLL_IsReady() != 1)
    ;

  /* Set the SYSCLK source to PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    ;

  /* Set AHB, APB1, and APB2 prescalers */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
}

/*---- Servo Logic ----*/

#define INCREMENT_AMT 50
#define POSMIN 500
#define POSMAX 2500

void set_servo(int change, uint8_t channel) {
  switch (channel) {
  case 1:
    LL_TIM_OC_SetCompareCH1(TIM2, change);
    break;
  case 2:
    LL_TIM_OC_SetCompareCH2(TIM2, change);
    break;
  default:
    break;
  }
}

volatile static int current_pan = 2500;
void pan_clockwise(void) {
  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
  current_pan -= INCREMENT_AMT;
  if (current_pan < POSMIN) {
    current_pan = POSMIN;
  }
  set_servo(current_pan, 1);
  delay(10);
  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
}
void pan_counterclockwise(void) {
  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
  current_pan += INCREMENT_AMT;
  if (current_pan > POSMAX) {
    current_pan = POSMAX;
  }
  set_servo(current_pan, 1);
  delay(10);
  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
}

#define LCD_ADDRESS 0x27
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define LCD_ENABLE 0x04
#define LCD_COMMAND 0x00
#define LCD_DATA 0x01

void ErrorFlash(void) {
  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
  delay(50);
  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
  delay(50);
}

void Test_Write(uint8_t address) {
  delay(100);
  LL_I2C_GenerateStartCondition(I2C1);
  while (!LL_I2C_IsActiveFlag_SB(I2C1)) {
  };

  LL_I2C_TransmitData8(I2C1, address << 1);
  while (!LL_I2C_IsActiveFlag_ADDR(I2C1)) {
    if (LL_I2C_IsActiveFlag_AF(I2C1)) {
      ErrorFlash();
    }
  };
  LL_I2C_GenerateStopCondition(I2C1);
}

void I2C_Write(uint8_t address, uint8_t *data, uint8_t size) {

  LL_I2C_GenerateStartCondition(I2C1);
  while (!LL_I2C_IsActiveFlag_SB(I2C1)) {
  };

  // Send Slave address with Write indication (0)
  // uint8_t address_with_write_bit = (address << 1);
  LL_I2C_TransmitData8(I2C1, address << 1);
  while (!LL_I2C_IsActiveFlag_ADDR(I2C1)) {
    if (LL_I2C_IsActiveFlag_AF(I2C1)) {
      ErrorFlash();
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
    ErrorFlash();
  }

  LL_I2C_GenerateStopCondition(I2C1);
}

void LCD_WriteNibble(uint8_t data, uint8_t control) {
  uint8_t highnib = data & 0xF0;
  uint8_t buffer[2];
  buffer[0] = highnib | control | LCD_ENABLE | LCD_BACKLIGHT;
  buffer[1] = highnib | control | LCD_BACKLIGHT;
  I2C_Write(LCD_ADDRESS, buffer, 2);
}

void LCD_SendCommand(uint8_t command) {
  LCD_WriteNibble(command, LCD_COMMAND);
  LCD_WriteNibble(command << 4, LCD_COMMAND);
}

void LCD_SendData(uint8_t data) {
  LCD_WriteNibble(data, LCD_DATA);
  LCD_WriteNibble(data << 4, LCD_DATA);
}

void LCD_Init(void) {
  delay(50); // Wait for LCD to power up

  // Initialize LCD in 4-bit mode
  LCD_WriteNibble(0x30, LCD_COMMAND);
  delay(5);
  LCD_WriteNibble(0x30, LCD_COMMAND);
  delay(1);
  LCD_WriteNibble(0x30, LCD_COMMAND);
  delay(1);
  LCD_WriteNibble(0x20, LCD_COMMAND); // Set to 4-bit mode

  //// Function Set
  LCD_SendCommand(0x28); // 4-bit mode, 2 lines, 5x8 font

  //// Display Control
  LCD_SendCommand(0x08); // Display off, cursor off, blink off

  //// Clear Display
  LCD_SendCommand(0x01); // Clear display

  //// Entry Mode Set
  LCD_SendCommand(0x06); // Increment cursor, no shift

  //// Display On
  LCD_SendCommand(0x0C); // Display on, cursor off, blink off

  delay(50); // Wait for the LCD to process commands
}

void LCD_SetCursor(uint8_t col, uint8_t row) {
  uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  if (row > 1) {
    row = 1; // We only support 2 rows: 0 and 1
  }
  LCD_SendCommand(0x80 | (col + row_offsets[row]));
}

void LCD_SendString(char *str) {
  while (*str) {
    LCD_SendData(*str++);
  }
}

// --------
int main(void) {
  systick_init();
  SystemClock_Config();
  gpio_init();
  pwm_init();
  i2c_init();
  LCD_Init();

  LCD_SendString("pee pee");
  LCD_SetCursor(0, 1);
  LCD_SendString("poo poo");

  set_servo(POSMIN, 1);
  volatile static uint8_t gpio14_state = 0;
  volatile static uint8_t gpio15_state = 0;
  while (1) {
    gpio14_state = LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_14);
    gpio15_state = LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_15);
    if (gpio14_state == 1) {
      pan_clockwise();
    }
    if (gpio15_state == 1) {
      pan_counterclockwise();
    }
  }
}
