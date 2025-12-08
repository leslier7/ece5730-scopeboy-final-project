// Robbie Leslie 2025
// Trigger code

#include "trigger.h"
#include "pico/stdlib.h"
#include "dac.h"
#include "adc.h"
#include <string.h>

void init_trigger(){
    // Set up trigger
    gpio_init(TRIG);
    gpio_set_dir(TRIG, GPIO_IN);
    gpio_set_irq_enabled(TRIG, GPIO_IRQ_EDGE_RISE, true);
}

int set_trigger_voltage(float voltage){

    if(voltage > 3.3 || voltage < 0.0f) return -1;

    return setVoltage(CHAN_TRIG, voltage);
}


// Dont really use, instead use interupt
bool get_trigger(){
    return gpio_get(TRIG);
}

// Call this in the global GPIO ISR function
void trigger_isr(){
    if (!trigger_armed) return;

    // Disarm until this capture is processed
    trigger_armed = false;

    // Tell main loop to freeze/copy the capture buffer
    trigger_fired = true;

}

void trigger_copy(){
    if (trigger_fired) {
            // Make a local copy of the flag as soon as possible
            trigger_fired = false;

            // At this moment, DMA is still running and overwriting capture_buf,
            // but the copy is very fast so you effectively "freeze" a window.
            memcpy(frame_buf, capture_buf, CAPTURE_DEPTH);

            capture_ready = true;

            // Re-arm trigger right away (continuous mode)
            // or wait until after you've drawn the frame / a holdoff.
            trigger_armed = true;
        }

        if (capture_ready) {
            // Use frame_buf for your display
            // draw_waveform(frame_buf, CAPTURE_DEPTH);
            capture_ready = false;
        }
}