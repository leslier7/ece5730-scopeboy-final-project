/*
    DAC code
    By Robbie Leslie 2025

    Pins:
    CS    -> GPIO 8
    LDAC  -> GPIO 9
    MOSI  -> GPIO 19
    SCK   -> GPIO 17
    VOUT_A is for trigger
    VOUT_B is for offset


*/

#include "dac.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define PIN_CS   8
#define PIN_LDAC 9
#define PIN_MOSI 19
#define PIN_SCK  17
#define SPI_PORT spi0

#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

void initDac(){

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(PIN_LDAC) ;
    gpio_set_dir(PIN_LDAC, GPIO_OUT) ;
    gpio_put(PIN_LDAC, 0) ;
}

int setVoltage(int channel, float voltage){

    if(voltage > 3.3){
        return -1;
    }
    uint16_t raw = (uint16_t)(voltage * 4095.0f / 3.3f + 0.5f); // convert + round
    uint16_t dac_data; 

    switch (channel){
        case CHAN_A:
            // Perform an SPI transaction
            dac_data = (uint16_t)(DAC_config_chan_A | (raw & 0x0FFF));
            spi_write16_blocking(SPI_PORT, &dac_data, 1) ;
            return 1;
            break;
        case CHAN_B:
            dac_data = (uint16_t)(DAC_config_chan_B | (raw & 0x0FFF));
            spi_write16_blocking(SPI_PORT, &dac_data, 1) ;
            return 2;
            break;
        default: return -1;
    }
    
    return -1;
}

