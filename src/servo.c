#include "main.h"
#include "servo.h"
#include "stm32f103xb.h"
#include "stm32f1xx_ll_tim.h"
#include <stdint.h>

volatile static int current_pan_position = 1500;
volatile static int current_tilt_position = 1500;

void set_servo_position(int position, servo_channel channel) {
  switch (channel) {
  case 0:
    LL_TIM_OC_SetCompareCH1(TIM2, position);
    break;
  case 1:
    LL_TIM_OC_SetCompareCH2(TIM2, position);
    break;
  }
}

void initialize(void){

}

int get_pan_position(void){
	return current_pan_position;
}

int get_tilt_position(void){
	return current_tilt_position;

}

void pan_clockwise(void) {
  current_pan_position -= INCREMENT_AMT;
  if (current_pan_position < POSITION_MIN) {
    current_pan_position = POSITION_MIN;
  }
  set_servo_position(current_pan_position, sc_pan);
  delay(10);
}
void pan_counterclockwise(void) {
  current_pan_position += INCREMENT_AMT;
  if (current_pan_position > POSITION_MAX) {
    current_pan_position = POSITION_MAX;
  }
  set_servo_position(current_pan_position, sc_pan);
  delay(10);
}

void tilt_clockwise(void) {
  current_pan_position -= INCREMENT_AMT;
  if (current_pan_position < POSITION_MIN) {
    current_pan_position = POSITION_MIN;
  }
  set_servo_position(current_pan_position, sc_tilt);
  delay(10);
}
void tilt_counterclockwise(void) {
  current_pan_position += INCREMENT_AMT;
  if (current_pan_position > POSITION_MAX) {
    current_pan_position = POSITION_MAX;
  }
  set_servo_position(current_pan_position, sc_tilt);
  delay(10);
}


