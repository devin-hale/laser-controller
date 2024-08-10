#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "lcd.h"
#include "pwm.h"
#include "servo.h"
#include "stm32f103xb.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_gpio.h"
#include "sys.h"
#include "utils.h"
#include <stdint.h>

int main(void) {
  clock_init();
  gpio_init();
  adc_init();
  pwm_init();
  //i2c_init();
  //lcd_init();

  set_servo_position(500, sc_yaw);
  delay(500);
  set_servo_position(2500, sc_yaw);
  delay(500);
  //lcd_init_pitch_yaw();
  
  //static char pan_value[255];
  volatile uint16_t pot_max = 0xFFF;
  uint16_t pot1_value = read_adc(LL_ADC_CHANNEL_4);
  double pot_perc = (double)pot1_value/(double)pot_max;
  int new_pos = (double)POSITION_MAX * pot_perc;
  set_servo_position(new_pos,sc_yaw);
  while (1) {
	volatile uint16_t new_pot1_value = read_adc(LL_ADC_CHANNEL_4);
		pot1_value = new_pot1_value;
		double pot_perc = (double)pot1_value/(double)pot_max;
		int new_pos = (double)POSITION_MAX * pot_perc;
		set_servo_position(new_pos,sc_yaw);
    //gpiob13_state = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_13);
    //gpiob14_state = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_14);
    //if (gpiob13_state == 1) {
    //  yaw_clockwise();
    //  double perc = (double)(get_yaw() - POSITION_MIN) /
    //                (double)(POSITION_MAX - POSITION_MIN);
    //  int angle = (double)180 * perc;
    //  itos(pan_value, angle);

    //  lcd_update_yaw(pan_value);
    //}
    //if (gpiob14_state == 1) {
    //  yaw_counterclockwise();
    //  double perc = (double)(get_yaw() - POSITION_MIN) /
    //                (double)(POSITION_MAX - POSITION_MIN);
    //  int angle = (double)180 * perc;
    //  itos(pan_value, angle);
    //  lcd_set_cursor(5, 0);
    //  lcd_send_string("   ");
    //  lcd_set_cursor(5, 0);

    //  lcd_update_yaw(pan_value);
    //}
  }
}
