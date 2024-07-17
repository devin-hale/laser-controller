#pragma once

#include "stm32f1xx_ll_gpio.h"
#include <stdint.h>

#define PAN_PIN LL_GPIO_PIN_0
#define TILT_PIN LL_GPIO_PIN_1

typedef enum { sc_pan, sc_tilt } servo_channel;

#define PAN_CHANNEL 1
#define TILT_CHANNEL 2

#define INCREMENT_AMT 50
#define INCREMENT_RATE 50 // ms
#define POSITION_MIN 500
#define POSITION_MAX 2500

void set_servo_position(int change, servo_channel channel);
void pan_clockwise(void);
void pan_counterclockwise(void);
