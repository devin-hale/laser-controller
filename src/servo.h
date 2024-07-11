#pragma once

#include <stdint.h>

#define INCREMENT_AMT 50
#define POSMIN 500
#define POSMAX 2500

void set_servo(int change, uint8_t channel);
void pan_clockwise(void);
void pan_counterclockwise(void);

