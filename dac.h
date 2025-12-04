#ifndef DAC_H
#define DAC_H

enum DAC_Chan {
    CHAN_A,
    CHAN_B
};

void initDac();

int setVoltage(int channel, float voltage);

#endif