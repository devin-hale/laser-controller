#include "stm32f103xb.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_tim.h"
#include <stdint.h>

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
}

/*---- Servo Logic ----*/

#define INCREMENT_AMT 50
#define POSMIN 500
#define POSMAX 2500

void SetServoPosition(int change, uint8_t channel) {
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
  SetServoPosition(current_pan, 1);
  delay(10);
  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
}
void pan_counterclockwise(void) {
  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
  current_pan += INCREMENT_AMT;
  if (current_pan > POSMAX) {
    current_pan = POSMAX;
  }
  SetServoPosition(current_pan, 1);
  delay(10);
  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
}

// --------
int main(void) {
  systick_init();
  SystemClock_Config();
  gpio_init();
  pwm_init();

  SetServoPosition(2500, 1);
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
