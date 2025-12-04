// Final Project ECE5730
// Author: Immanuel Varghese Koshy , Robert Leslie
// Date: 2025-11-20
#include <stdio.h>
#include <string.h>
#include <math.h>   
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h" 
#include "pt_cornell_rp2040_v1_4.h"
#include "dac.h"
#include "TFTMaster.h"


// ==========================================
// --- ROTARY ENCODER DEFINITIONS ---
// ==========================================
#define PICO_ENC_CLK    14
#define PICO_ENC_DT     15
#define PICO_ENC_SW     8

// --- I2C/Seesaw Definitions ---
#define SEESAW_I2C_ADDR         0x50 
#define I2C_PORT                i2c1  
#define I2C_SDA_PIN             2     
#define I2C_SCL_PIN             3     

// Seesaw Registers
#define SEESAW_GPIO_BASE        0x01
#define SEESAW_GPIO_BULK_SET    0x05 
#define SEESAW_GPIO_BULK        0x04 
#define SEESAW_ADC_BASE         0x09
#define SEESAW_ADC_OFFSET       0x07

// --- PIN MAPPING (Adafruit Mini Gamepad PID 5743) ---
#define PIN_BTN_SELECT  0
#define PIN_BTN_B       1   
#define PIN_BTN_Y       2   
#define PIN_BTN_A       5   
#define PIN_BTN_X       6   
#define PIN_BTN_START   16
#define PIN_JOY_X       14
#define PIN_JOY_Y       15

// Bitmasks
#define MASK_SELECT     (1UL << PIN_BTN_SELECT)
#define MASK_B          (1UL << PIN_BTN_B)
#define MASK_Y          (1UL << PIN_BTN_Y)
#define MASK_A          (1UL << PIN_BTN_A)
#define MASK_X          (1UL << PIN_BTN_X)
#define MASK_START      (1UL << PIN_BTN_START)

// Logical Buttons
#define BTN_CONFIRM     MASK_B      
#define BTN_BACK        MASK_A      
#define BTN_MENU        MASK_SELECT 
#define BTN_RECORD      MASK_START  

// Joystick Constants
#define JOY_CENTER      512         
#define JOY_DEADZONE    400         
#define JOY_THRESHOLD_HIGH (JOY_CENTER + JOY_DEADZONE) 
#define JOY_THRESHOLD_LOW  (JOY_CENTER - JOY_DEADZONE) 

// Colors 
#define TFT_BLACK       ILI9340_BLACK
#define TFT_NAVY        0x0010  
#define TFT_DARKGREY    0x4208  
#define TFT_BLUE        0x001F
#define TFT_GREEN       ILI9340_GREEN
#define TFT_RED         ILI9340_RED
#define TFT_YELLOW      ILI9340_YELLOW
#define TFT_WHITE       ILI9340_WHITE
#define TFT_LIGHTGREY   0xC618
#define TFT_MAGENTA     ILI9340_MAGENTA
#define TFT_CYAN        0x07FF

// --- Scope Settings ---
bool isRunning = true;
float voltsPerDiv = 1.0;  
float timePerDiv = 10.0; 
bool showCursors = false;
float cursorV1_volts = 2.5;
float cursorV2_volts = 0.5;

// --- Menu System ---
enum MenuIndex {
    MENU_RUN_STOP = 0,
    MENU_V_DIV,
    MENU_T_DIV,
    MENU_CURSORS_EN,
    MENU_CUR_V1,
    MENU_CUR_V2,
    MENU_COUNT 
};

const char* menuNames[] = {
    "Run/Stop", "V / Div", "T / Div", "Cursors", "Cur V1", "Cur V2"
};

// --- State Machine ---
bool isMenuOpen = false;
bool isEditing = false; 
int selectedMenuItem = 0;
bool forceFullRedraw = true; 
bool isRecording = false; 

// --- Input State ---
bool btnMenuPressed = false;
bool btnConfirmPressed = false; 
bool btnBackPressed = false;    
bool btnRecordPressed = false;  
bool joyUpHeld = false;
bool joyDownHeld = false;
bool encSwPressed = false; 

// --- Waveform Buffer ---
short oldWaveY[320]; 

// --- Rotary Encoder Global ---
volatile int rotaryDelta = 0;

// ==========================================
// --- INTERRUPT FOR ENCODER ---
// ==========================================
void encoder_callback(uint gpio, uint32_t events) {
    if (gpio == PICO_ENC_CLK) {
        if (gpio_get(PICO_ENC_DT)) {
            rotaryDelta--;
        } else {
            rotaryDelta++;
        }
    }
}

// --- Helper Functions ---
void seesaw_pin_mode_bulk(uint32_t pins);
uint32_t seesaw_read_buttons();
uint16_t seesaw_read_analog(uint8_t pin);

void rotary_init() {
    gpio_init(PICO_ENC_CLK); gpio_set_dir(PICO_ENC_CLK, GPIO_IN); gpio_pull_up(PICO_ENC_CLK);
    gpio_init(PICO_ENC_DT);  gpio_set_dir(PICO_ENC_DT, GPIO_IN);  gpio_pull_up(PICO_ENC_DT);
    gpio_init(PICO_ENC_SW);  gpio_set_dir(PICO_ENC_SW, GPIO_IN);  gpio_pull_up(PICO_ENC_SW); 
    gpio_set_irq_enabled_with_callback(PICO_ENC_CLK, GPIO_IRQ_EDGE_RISE, true, &encoder_callback);
}

short voltToPixel(float volts) {
    float centerVolts = 1.65; 
    float pixelsPerDiv = 48.0;
    if (voltsPerDiv < 0.1) voltsPerDiv = 0.1;
    float diff = volts - centerVolts;
    float divsFromCenter = diff / voltsPerDiv;
    short y = 120 - (short)(divsFromCenter * pixelsPerDiv);
    if (y < 0) y = 0; if (y > 239) y = 239;
    return y;
}

// --- Drawing Functions ---

void drawGrid(short width) {
    tft_fillRect(0, 0, width, 240, TFT_BLACK);
    for (int i = 1; i < 5; i++) {
        short pos = 48 * i;
        if (pos < 240) tft_drawFastHLine(0, pos, width, TFT_DARKGREY);
        if (pos < width) tft_drawFastVLine(pos, 0, 240, TFT_DARKGREY);
    }
    tft_drawFastHLine(0, 120, width, 0x7BEF); 
}

void drawWaveform(short width) {
    static float phase = 0;
    if (isRunning) phase += 0.2;
    
    int prevX = 0;
    int prevY_new = 120;
    
    float amp = 0.5; 
    float center = 1.65;
    float freqFactor = 0.05 * (timePerDiv / 10.0); // Scale wave with T/Div

    for (int x = 0; x < width; x++) {
        
        float dummyVolts = center + (amp * sin((x * freqFactor) + phase));
        int newY = voltToPixel(dummyVolts);
        
        if (x > 0) {
            // Smart Erase & Draw
            tft_drawLine(prevX, oldWaveY[prevX], x, oldWaveY[x], TFT_BLACK);
            tft_drawLine(prevX, prevY_new, x, newY, TFT_YELLOW);
            
            // Repair Grid
            if (x % 48 == 0) tft_drawFastVLine(x, 0, 240, TFT_DARKGREY);
            if (oldWaveY[x] % 48 == 0) tft_drawFastHLine(0, oldWaveY[x], width, TFT_DARKGREY);
        }
        oldWaveY[x] = newY; 
        prevX = x; prevY_new = newY;
    }
}

void drawCursors(short width) {
    if (!showCursors) return;

    short y1 = voltToPixel(cursorV1_volts);
    short y2 = voltToPixel(cursorV2_volts);

    tft_drawFastHLine(0, y1, width, TFT_MAGENTA);
    tft_drawFastHLine(0, y2, width, TFT_CYAN);
    
    float deltaV = cursorV1_volts - cursorV2_volts;
    if (deltaV < 0) deltaV = -deltaV; 
    
    tft_setTextSize(1);
    tft_setTextColor(TFT_WHITE);
    tft_setCursor(5, 25); 
    char buf[20];
    sprintf(buf, "dV: %.2f V", deltaV);
    tft_writeString(buf);
}

void drawUI() {
    short scopeWidth = isMenuOpen ? 240 : 320;

    if (forceFullRedraw) {
        drawGrid(scopeWidth);
        for(int i=0; i<320; i++) oldWaveY[i] = 120; 
        forceFullRedraw = false; 
    }

    if (isRunning) drawWaveform(scopeWidth);
    drawCursors(scopeWidth);

    // --- Smart Text Updates ---
    static float oldVoltsPerDiv = -1;
    static float oldTimePerDiv = -1;
    static bool oldIsRecording = false;
    
    // 1. V/Div Text
    if (voltsPerDiv != oldVoltsPerDiv || forceFullRedraw) {
        tft_fillRect(5, 5, 110, 20, TFT_BLACK); 
        tft_setTextSize(2);
        tft_setTextColor(TFT_GREEN); 
        tft_setCursor(5, 5);
        char buf[32];
        sprintf(buf, "%.1f V/d", voltsPerDiv);
        tft_writeString(buf);
        oldVoltsPerDiv = voltsPerDiv;
    }

    // 2. T/Div Text
    if (timePerDiv != oldTimePerDiv || forceFullRedraw) {
        tft_fillRect(120, 5, 110, 20, TFT_BLACK); 
        tft_setTextSize(2);
        tft_setTextColor(TFT_YELLOW); 
        tft_setCursor(120, 5);
        char buf[32];
        sprintf(buf, "%.0f ms/d", timePerDiv); 
        tft_writeString(buf);
        oldTimePerDiv = timePerDiv;
    }

    // 3. REC Indicator
    if (isRecording != oldIsRecording || forceFullRedraw) {
        tft_fillRect(scopeWidth - 40, 5, 40, 20, TFT_BLACK);
        if (isRecording) {
            tft_setTextColor(TFT_RED); 
            tft_setCursor(scopeWidth - 40, 5);
            tft_writeString((char*)"REC");
        }
        oldIsRecording = isRecording;
    }

    // --- Menu Drawing ---
    if (isMenuOpen) {
        tft_fillRect(240, 0, 80, 240, TFT_NAVY);
        tft_setTextSize(1);

        for (int i = 0; i < MENU_COUNT; i++) {
            short yPos = 10 + (i * 40);
            uint16_t boxColor = TFT_NAVY;
            uint16_t textColor = TFT_LIGHTGREY;

            if (i == selectedMenuItem) {
                boxColor = isEditing ? TFT_RED : TFT_BLUE;
                textColor = TFT_WHITE;
            }

            tft_fillRect(240, yPos - 2, 80, 35, boxColor);
            tft_setTextColor(textColor); 
            tft_setCursor(245, yPos + 10);
            tft_writeString((char*)menuNames[i]);
            
            tft_setCursor(245, yPos + 22);
            tft_setTextColor(TFT_WHITE); 
            
            char buf[32];
            if (i == MENU_V_DIV) sprintf(buf, "%.1fV", voltsPerDiv);
            else if (i == MENU_T_DIV) sprintf(buf, "%.0fms", timePerDiv); 
            else if (i == MENU_CUR_V1) sprintf(buf, "%.1fV", cursorV1_volts);
            else if (i == MENU_CUR_V2) sprintf(buf, "%.1fV", cursorV2_volts);
            else if (i == MENU_RUN_STOP) sprintf(buf, "%s", isRunning ? "RUN" : "STOP");
            else if (i == MENU_CURSORS_EN) sprintf(buf, "%s", showCursors ? "ON" : "OFF");
            else sprintf(buf, " ");
            
            tft_writeString(buf);
        }
    }
}

void handleInput() {
    uint32_t buttons = seesaw_read_buttons();
    uint16_t joyY = seesaw_read_analog(PIN_JOY_Y);
    
    bool currentEncSw = !gpio_get(PICO_ENC_SW);
    int delta = rotaryDelta;
    rotaryDelta = 0;

    // Global Menu Toggle
    if (!(buttons & BTN_MENU)) {
        if (!btnMenuPressed) { 
            isMenuOpen = !isMenuOpen; 
            isEditing = false; 
            forceFullRedraw = true; 
        }
        btnMenuPressed = true; 
    } else btnMenuPressed = false;

    // --- INVERTED LOGIC FOR ROTATION 3 ---
    // Stick UP (High Y) -> Screen DOWN (Next Item)
    bool nav_next = (joyY > JOY_THRESHOLD_HIGH); 
    // Stick DOWN (Low Y) -> Screen UP (Prev Item)
    bool nav_prev = (joyY < JOY_THRESHOLD_LOW);
    
    if (isMenuOpen) {
        if (isEditing) {
            // --- EDIT MODE (ROTARY) ---
            if (delta != 0) {
                switch(selectedMenuItem) {
                    case MENU_V_DIV: 
                        voltsPerDiv += (delta * 0.1); 
                        if (voltsPerDiv < 0.1) voltsPerDiv = 0.1;
                        forceFullRedraw = true; 
                        break;
                    case MENU_T_DIV: 
                        timePerDiv += (delta * 1.0); // Change Time
                        if (timePerDiv < 1.0) timePerDiv = 1.0;
                        forceFullRedraw = true; 
                        break;
                    case MENU_CUR_V1:
                        cursorV1_volts += (delta * 0.1);
                        break;
                    case MENU_CUR_V2:
                        cursorV2_volts += (delta * 0.1);
                        break;
                }
            }
            
            // Exit Edit Mode Logic
            bool confirm = !(buttons & BTN_CONFIRM);
            bool back = !(buttons & BTN_BACK);
            if (currentEncSw && !encSwPressed) { confirm = true; encSwPressed = true; }
            else if (!currentEncSw) encSwPressed = false;

            if (confirm && !btnConfirmPressed) { isEditing = false; btnConfirmPressed = true; forceFullRedraw = true; }
            if (back && !btnBackPressed) { isEditing = false; btnBackPressed = true; forceFullRedraw = true; }

        } else {
            // --- NAV MODE (JOYSTICK) ---
            if (nav_prev) { // Go "UP" the list
                if (!joyUpHeld) {
                    selectedMenuItem--;
                    if (selectedMenuItem < 0) selectedMenuItem = MENU_COUNT - 1;
                }
                joyUpHeld = true;
            } else joyUpHeld = false;

            if (nav_next) { // Go "DOWN" the list
                if (!joyDownHeld) {
                    selectedMenuItem++;
                    if (selectedMenuItem >= MENU_COUNT) selectedMenuItem = 0;
                }
                joyDownHeld = true;
            } else joyDownHeld = false;

            // Enter Edit Mode Logic
            bool confirm = !(buttons & BTN_CONFIRM);
            if (currentEncSw && !encSwPressed) { confirm = true; encSwPressed = true; }
            else if (!currentEncSw) encSwPressed = false;

            if (confirm && !btnConfirmPressed) {
                if (selectedMenuItem == MENU_RUN_STOP) isRunning = !isRunning;
                else if (selectedMenuItem == MENU_CURSORS_EN) showCursors = !showCursors;
                else isEditing = true;
                btnConfirmPressed = true;
                forceFullRedraw = true;
            }
        }
    } 
    if ((buttons & BTN_CONFIRM)) btnConfirmPressed = false;
    if ((buttons & BTN_BACK)) btnBackPressed = false;
    
    // Record
    if (!(buttons & BTN_RECORD)) {
        if (!btnRecordPressed) isRecording = !isRecording;
        btnRecordPressed = true;
    } else btnRecordPressed = false;
}

// --- Main ---
int main() {
    stdio_init_all(); 
    
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    

    i2c_init(I2C_PORT, 400 * 1000); 
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    

    sleep_ms(100); 

    uint32_t digital_pins = MASK_A | MASK_B | MASK_X | MASK_Y | MASK_START | MASK_SELECT;
    seesaw_pin_mode_bulk(digital_pins); 

    bool led_on = true;
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);

    rotary_init(); 

    

    tft_init_hw();
    tft_begin();
    tft_setRotation(3); // ROTATION 3 (INVERTED LANDSCAPE)
    tft_fillScreen(TFT_BLACK);
    
    initDac();

    for(int i=0; i<320; i++) oldWaveY[i] = 120;

    

    while (true) {
        handleInput();
        drawUI();
        sleep_ms(16); 

        int dac_val = setVoltage(CHAN_A, 1.5);
        printf("\nDac return value: %d", dac_val);

        gpio_put(PICO_DEFAULT_LED_PIN, led_on);
        led_on = !led_on;
    }
    return 0;
}

// --- I2C Implementations ---
void seesaw_pin_mode_bulk(uint32_t pins) {
    uint8_t buf[6];
    buf[0] = SEESAW_GPIO_BASE; buf[1] = SEESAW_GPIO_BULK_SET; 
    buf[2] = (pins >> 24) & 0xFF; buf[3] = (pins >> 16) & 0xFF;
    buf[4] = (pins >> 8) & 0xFF; buf[5] = pins & 0xFF;
    i2c_write_blocking(I2C_PORT, SEESAW_I2C_ADDR, buf, 6, false);
}

uint32_t seesaw_read_buttons() {
    uint8_t write_buf[2] = {SEESAW_GPIO_BASE, SEESAW_GPIO_BULK};
    uint8_t read_buf[4];
    i2c_write_blocking(I2C_PORT, SEESAW_I2C_ADDR, write_buf, 2, false); 
    sleep_us(600); 
    i2c_read_blocking(I2C_PORT, SEESAW_I2C_ADDR, read_buf, 4, false); 
    return ((uint32_t)read_buf[0] << 24) | ((uint32_t)read_buf[1] << 16) | 
           ((uint32_t)read_buf[2] << 8) | (uint32_t)read_buf[3];
}

uint16_t seesaw_read_analog(uint8_t pin) {
    uint8_t write_buf[2] = {SEESAW_ADC_BASE, (uint8_t)(SEESAW_ADC_OFFSET + pin)};
    uint8_t read_buf[2];
    i2c_write_blocking(I2C_PORT, SEESAW_I2C_ADDR, write_buf, 2, false);
    sleep_us(1000); 
    i2c_read_blocking(I2C_PORT, SEESAW_I2C_ADDR, read_buf, 2, false);
    return ((uint16_t)read_buf[0] << 8) | read_buf[1];
}