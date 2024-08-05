#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "lcd.h"
#include "pwm.h"
#include "servo.h"
#include "stm32f103xb.h"
#include "stm32f1xx_ll_gpio.h"
#include "sys.h"
#include "utils.h"
#include <stdint.h>

int main(void) {
  clock_init();
  gpio_init();
  adc_init();
  pwm_init();
  i2c_init();
  lcd_init();

  set_servo_position(get_yaw(), sc_yaw);
  lcd_init_pitch_yaw();
  
  volatile static uint8_t gpiob13_state = 0;
  volatile static uint8_t gpiob14_state = 0;
  static char pan_value[255];
  while (1) {
    gpiob13_state = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_13);
    gpiob14_state = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_14);
    if (gpiob13_state == 1) {
      yaw_clockwise();
      double perc = (double)(get_yaw() - POSITION_MIN) /
                    (double)(POSITION_MAX - POSITION_MIN);
      int angle = (double)180 * perc;
      itos(pan_value, angle);

      lcd_update_yaw(pan_value);
    }
    if (gpiob14_state == 1) {
      yaw_counterclockwise();
      double perc = (double)(get_yaw() - POSITION_MIN) /
                    (double)(POSITION_MAX - POSITION_MIN);
      int angle = (double)180 * perc;
      itos(pan_value, angle);
      lcd_set_cursor(5, 0);
      lcd_send_string("   ");
      lcd_set_cursor(5, 0);

      lcd_update_yaw(pan_value);
    }
  }
}
