#include "lcd.h"
#include "servo.h"
#include "stm32f103xb.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_tim.h"
#include "sys.h"
#include "utils.h"
#include "gpio.h"
#include "i2c.h"
#include "adc.h"
#include <stdint.h>


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


// --------
int main(void) {
  clock_init();
  gpio_init();
  adc_init();
  pwm_init();
  i2c_init();
  lcd_init();

  set_servo_position(get_yaw(), sc_yaw);
  lcd_set_cursor(0, 0);
  lcd_send_string("YAW: 90");
  lcd_set_cursor(0, 1);
  lcd_send_string("PITCH: 90");
  lcd_set_cursor(5, 0);

  volatile static uint8_t gpiob13_state = 0;
  volatile static uint8_t gpiob14_state = 0;
  static char pan_value[255];
  while (1) {
    gpiob13_state = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_13);
    gpiob14_state = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_14);
    if (gpiob13_state == 1) {
      yaw_clockwise();
      double perc =
          (double)(get_yaw() - POSITION_MIN) / (double)(POSITION_MAX - POSITION_MIN);
      int angle = (double)180 * perc;
      itos(pan_value, angle);

      lcd_set_cursor(5, 0);
      lcd_send_string("   ");
      lcd_set_cursor(5, 0);
      lcd_send_string(pan_value);
    }
    if (gpiob14_state == 1) {
      yaw_counterclockwise();
      double perc =
          (double)(get_yaw() - POSITION_MIN) / (double)(POSITION_MAX - POSITION_MIN);
      int angle = (double)180 * perc;
      itos(pan_value, angle);
      lcd_set_cursor(5, 0);
      lcd_send_string("   ");
      lcd_set_cursor(5, 0);
      lcd_send_string(pan_value);
    }
  }
}
