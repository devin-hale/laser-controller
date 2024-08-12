#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "lcd.h"
#include "pwm.h"
#include "servo.h"
#include "stm32f1xx_ll_adc.h"
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

  lcd_init_pitch_yaw();

  static char yaw_value[255];
  volatile uint16_t pot_max = 0xFFF;
  uint16_t pot1_value = read_adc(LL_ADC_CHANNEL_6);
  double pot1_perc = (double)pot1_value / (double)pot_max;
  int new_pos = (double)POSITION_MAX * pot1_perc;
  set_servo_position(new_pos, sc_yaw);
  double yaw_perc =
      (double)(new_pos - POSITION_MIN) / (double)(POSITION_MAX - POSITION_MIN);
  int yaw_angle = (double)180 * yaw_perc;
  itos(yaw_value, yaw_angle);
  lcd_update_yaw(yaw_value);

  static char pitch_value[255];
  uint16_t pot2_value = read_adc(LL_ADC_CHANNEL_4);
  double pot2_perc = (double)pot2_value / (double)pot_max;
  new_pos = (double)POSITION_MAX * pot2_perc;
  set_servo_position(new_pos, sc_pitch);

  double pitch_perc =
      (double)(new_pos - POSITION_MIN) / (double)(POSITION_MAX - POSITION_MIN);
  int pitch_angle = (double)180 * pitch_perc;
  itos(pitch_value, pitch_angle);

  lcd_update_pitch(pitch_value);

  while (1) {
    volatile uint16_t new_pot1_value = read_adc(LL_ADC_CHANNEL_6);
    pot1_value = new_pot1_value;
    pot1_perc = (double)pot1_value / (double)pot_max;
    new_pos = (double)POSITION_MAX * pot1_perc;
    set_servo_position(new_pos, sc_yaw);
    yaw_perc = (double)(new_pos - POSITION_MIN) /
               (double)(POSITION_MAX - POSITION_MIN);
    yaw_angle = (double)180 * yaw_perc;
    itos(yaw_value, yaw_angle);
    lcd_update_yaw(yaw_value);

    volatile uint16_t new_pot2_value = read_adc(LL_ADC_CHANNEL_4);
    pot2_value = new_pot2_value;
    pot2_perc = (double)pot2_value / (double)pot_max;
    new_pos = (double)POSITION_MAX * pot2_perc;
    set_servo_position(new_pos, sc_pitch);
    pitch_perc = (double)(new_pos - POSITION_MIN) /
                 (double)(POSITION_MAX - POSITION_MIN);
    pitch_angle = (double)180 * pitch_perc;
    itos(pitch_value, pitch_angle);
    lcd_update_pitch(pitch_value);
  }
}
