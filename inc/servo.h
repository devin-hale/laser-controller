#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx_ll_gpio.h"
#include <stdint.h>

#define YAW_PIN LL_GPIO_PIN_0
#define PITCH_PIN LL_GPIO_PIN_1

typedef enum { sc_yaw = 1, sc_pitch } servo_channel;

#define INCREMENT_AMT 100
#define INCREMENT_RATE 0 // ms
#define POSITION_MIN 500
#define POSITION_MAX 2500

void set_servo_position(int change, servo_channel channel);
int get_yaw(void);
void yaw_clockwise(void);
void yaw_counterclockwise(void);

#endif
