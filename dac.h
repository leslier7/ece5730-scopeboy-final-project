#ifndef DAC_H
#define DAC_H

enum DAC_Chan {
    CHAN_TRIG,
    CHAN_OFFSET
};

void initDac();

int setVoltage(int channel, float voltage);

#endif