#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "pico/multicore.h"

#include "TFTMaster.h"
#include "rotary.h"

void core1_entry(){

    while(true){
        
    }
}

int main()
{
    // Initialize stdio
    stdio_init_all();

    // tft_init_hw();
    // tft_begin();
    // tft_setRotation(3); 
    // tft_fillScreen(ILI9340_GREEN);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    rotary_init();

    printf("\nHello world");

    // start core 1 
    //multicore_reset_core1();
    //multicore_launch_core1(core1_entry);

    while(true){
        //display_counts();
        rotary_service();
        sleep_ms(15);
    }
}
