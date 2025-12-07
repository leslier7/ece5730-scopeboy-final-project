#ifndef ADC_H
#define ADC_H

#include "pico/stdlib.h"

#define CAPTURE_DEPTH 320

extern uint8_t capture_buf[CAPTURE_DEPTH];

void init_adc_capture();

float adc_to_volt(uint8_t adc_val);

#endif