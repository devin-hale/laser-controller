#pragma once

#include <stdint.h>

void adc_init(void);
uint16_t read_adc(uint32_t channel);
