#ifndef TRIGGER_H
#define TRIGGER_H

#define TRIG 7

void init_trigger();

int set_trigger_voltage(float voltage);

void trigger_isr();

void trigger_copy();

#endif