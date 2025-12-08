#ifndef ADC_H
#define ADC_H

#include "pico/stdlib.h"

#define CAPTURE_DEPTH 320

typedef enum gain_mode{
    GAIN_LOW,
    GAIN_MEDIUM,
    GAIN_HIGH
} gain_mode_t;

extern uint8_t capture_buf[CAPTURE_DEPTH];
extern uint8_t frame_buf[CAPTURE_DEPTH];

extern volatile bool trigger_fired;  // set by ISR
extern volatile bool capture_ready;  // main loop can use frame_buf when true
extern volatile bool trigger_armed;  // allow/ignore triggers

void init_adc_capture();

float adc_to_volt(uint8_t adc_val);

void set_gain(gain_mode_t gain);

#endif