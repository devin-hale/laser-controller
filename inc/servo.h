#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx_ll_gpio.h"
#include <stdint.h>

#define PAN_PIN LL_GPIO_PIN_0
#define TILT_PIN LL_GPIO_PIN_1

typedef enum { sc_pan = 1, sc_tilt } servo_channel;

#define INCREMENT_AMT 100
#define INCREMENT_RATE 0 // ms
#define POSITION_MIN 500
#define POSITION_MAX 2500

void set_servo_position(int change, servo_channel channel);
int get_pan_position(void);
void pan_clockwise(void);
void pan_counterclockwise(void);

#endif
