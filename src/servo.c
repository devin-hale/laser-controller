#include "servo.h"
#include "stm32f103xb.h"
#include "stm32f1xx_ll_tim.h"
#include <stdint.h>
#include "utils.h"

volatile static int current_yaw = 1500;
volatile static int current_pitch = 1500;

void set_servo_position(int position, servo_channel channel) {
  switch (channel) {
  case 1:
    LL_TIM_OC_SetCompareCH1(TIM2, position);
    break;
  case 2:
    LL_TIM_OC_SetCompareCH2(TIM2, position);
    break;
  }
}

void initialize(void){

}

int get_yaw(void){
	return current_yaw;
}

int get_pitch(void){
	return current_pitch;

}

void yaw_clockwise(void) {
  current_yaw -= INCREMENT_AMT;
  if (current_yaw < POSITION_MIN) {
    current_yaw = POSITION_MIN;
  }
  set_servo_position(current_yaw, sc_yaw);
  delay(INCREMENT_RATE);
}
void yaw_counterclockwise(void) {
  current_yaw += INCREMENT_AMT;
  if (current_yaw > POSITION_MAX) {
    current_yaw = POSITION_MAX;
  }
  set_servo_position(current_yaw, sc_yaw);
  delay(INCREMENT_RATE);
}

void pitch_clockwise(void) {
  current_pitch -= INCREMENT_AMT;
  if (current_pitch < POSITION_MIN) {
    current_pitch = POSITION_MIN;
  }
  set_servo_position(current_pitch, sc_pitch);
  delay(10);
}
void pitch_counterclockwise(void) {
  current_pitch += INCREMENT_AMT;
  if (current_pitch > POSITION_MAX) {
    current_pitch = POSITION_MAX;
  }
  set_servo_position(current_pitch, sc_pitch);
  delay(10);
}


