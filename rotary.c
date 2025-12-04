#include "rotary.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>

#include "TFTMaster.h"

volatile int dtcount = 0;
volatile int clkcount = 0;
volatile bool clk_event = false;
volatile bool dt_event = false;

void gpio_callback(uint gpio, uint32_t events){
    if (gpio == ENCCLK) {
        // ISR: do minimal work only
        clkcount++;
        clk_event = true;
    } else if (gpio == ENCDT) {
        dtcount++;
        dt_event = true;
    }
}

void rotary_init(){
    gpio_init(ENCCLK);
    gpio_pull_up(ENCCLK);                 // ensure defined level
    gpio_set_irq_enabled_with_callback(ENCCLK, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    gpio_init(ENCDT);
    gpio_pull_up(ENCDT);
    gpio_set_irq_enabled_with_callback(ENCDT, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
}

void rotary_service(void){
    if (clk_event){
        clk_event = false;
        //printf("\nclk triggered %d", clkcount);
    }
    if (dt_event){
        dt_event = false;
        //printf("\ndt triggered %d", dtcount);
    }
}

void display_counts(){
    // copy volatile counters to locals to avoid races with ISR
    int clk = clkcount;
    int dt = dtcount;

    char line1[32];
    char line2[32];
    snprintf(line1, sizeof(line1), "CLK: %d", clk);
    snprintf(line2, sizeof(line2), "DT:  %d", dt);

    const int tsz = 3;                 // text size (tweak to taste)
    const int char_w = 6 * tsz;        // approx pixels per character (5px font + 1px space)
    const int char_h = 8 * tsz;        // approx character height
    int w1 = strlen(line1) * char_w;
    int w2 = strlen(line2) * char_w;
    int maxw = (w1 > w2) ? w1 : w2;
    int spacing = char_h / 2;          // vertical spacing between lines
    int total_h = char_h * 2 + spacing;

    int x = (ILI9340_TFTWIDTH - maxw) / 2;
    int y = (ILI9340_TFTHEIGHT - total_h) / 2;

    // clear screen (or replace with a smaller fillRect if you prefer)
    tft_fillScreen(ILI9340_BLACK);

    tft_setTextSize(tsz);
    tft_setTextColor(ILI9340_WHITE);
    tft_setCursor(x, y);
    tft_writeString(line1);
    tft_setCursor(x, y + char_h + spacing);
    tft_writeString(line2);
}

// void clk_callback(){
//     printf("\nclk triggered %d", clkcount);
//     clkcount++;
// }

// void dt_callback(){
//     printf("\ndt triggered %d", dtcount);
//     dtcount++;
// }

// void rotary_init(){
//     gpio_init(ENCCLK);
//     //gpio_pull_up(ENCCLK);
//     gpio_set_irq_enabled_with_callback(ENCCLK, GPIO_IRQ_EDGE_RISE, true, &clk_callback);

//     gpio_init(ENCDT);
//     //gpio_pull_up(ENCDT);
//     gpio_set_irq_enabled_with_callback(ENCDT, GPIO_IRQ_EDGE_RISE, true, &dt_callback);
// }

