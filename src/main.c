
#include "stm32f103xb.h"

void systick_init(void) {
  static int ms = 8000;
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

int main(void) {
  systick_init();

  while (1) {
    GPIOC->ODR ^= GPIO_ODR_ODR13;
    delay(1000);
  }
}
