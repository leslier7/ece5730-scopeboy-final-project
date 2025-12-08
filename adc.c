// Code by Robbie Leslie
// Based on the dma_capture example from the Pico Examples repo

#include "adc.h"
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "dac.h"

#define SEL_0 9
#define SEL_1 8

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0

#define ADC_RESOLUTION 256.0 // 8-bit adc

uint8_t capture_buf[CAPTURE_DEPTH];
uint8_t frame_buf[CAPTURE_DEPTH];

volatile bool trigger_fired = false;  // set by ISR
volatile bool capture_ready = false;  // main loop can use frame_buf when true
volatile bool trigger_armed = true;   // allow/ignore triggers

// Pointer to the address of the ADC array
uint8_t * dma_address_pointer = &capture_buf[0] ;

void init_adc_capture(){
    
    // Set up gain selector switch
    gpio_init(SEL_0);
    gpio_set_dir(SEL_0, GPIO_OUT);

    gpio_init(SEL_1);
    gpio_set_dir(SEL_1, GPIO_OUT);

    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(26 + CAPTURE_CHANNEL);

    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );

    // ------------------- From pico examples ------------------------
    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock.
    adc_set_clkdiv(0);

    uint ctrl_chan = dma_claim_unused_channel(true);
    uint data_chan = dma_claim_unused_channel(true);

    // Set up control channel
    // Setup the control channel
    dma_channel_config c = dma_channel_get_default_config(ctrl_chan);   // default configs
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);             // 32-bit txfers
    channel_config_set_read_increment(&c, false);                       // no read incrementing
    channel_config_set_write_increment(&c, false);                      // no write incrementing
    channel_config_set_chain_to(&c, data_chan);                         // chain to data channel

    dma_channel_configure(
        ctrl_chan,                          // Channel to be configured
        &c,                                 // The configuration we just created
        &dma_hw->ch[data_chan].write_addr,   // Write address (data channel read address)
        &dma_address_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // Number of transfers
        false                               // Don't start immediately
    );

    // Set up the data channel
    // Set up the DMA to start transferring data as soon as it appears in FIFO
    dma_channel_config cfg = dma_channel_get_default_config(data_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);
    channel_config_set_chain_to(&cfg, ctrl_chan);   // Chain to control channel

    dma_channel_configure(data_chan, &cfg,
        capture_buf,    // dst
        &adc_hw->fifo,  // src
        CAPTURE_DEPTH,  // transfer count
        true            // start immediately
    );

    // start the control channel
    dma_start_channel_mask(1u << ctrl_chan) ;

    adc_run(true);

}

float adc_to_volt(uint8_t adc_val){
    return (adc_val / ADC_RESOLUTION) * 3.3;
}

void set_gain(gain_mode_t gain){
    switch(gain){
        case GAIN_LOW:
            gpio_put(SEL_0, false);
            gpio_put(SEL_1, false);
            break;
        case GAIN_MEDIUM:
            gpio_put(SEL_0, false);
            gpio_put(SEL_1, true);
            break;
        case GAIN_HIGH:
            gpio_put(SEL_0, true);
            gpio_put(SEL_1, false);
            break;
        default: //Default to low gain mode
            gpio_put(SEL_0, false);
            gpio_put(SEL_1, false);
            break;
    }
}