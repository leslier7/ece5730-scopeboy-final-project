#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

#include "TFTMaster.h"
#include "rotary.h"



int main()
{
    // Initialize stdio
    stdio_init_all();

    tft_init_hw();
    tft_begin();
    tft_setRotation(3); 
    tft_fillScreen(ILI9340_GREEN);

    rotary_init();

    printf("\nHello world");

    while(true){
        rotary_service();
        display_counts();
    }
}
