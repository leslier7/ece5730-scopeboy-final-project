// Final Project ECE5730
// Author: Immanuel Varghese Koshy, Robert Leslie
// Date: 2025-11-20
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "pt_cornell_rp2040_v1_4.h"
#include "TFTMaster.h"
#include "dac.h"
#include "adc.h"
#include "trigger.h"

// ==========================================
// --- ROTARY ENCODER DEFINITIONS ---
// ==========================================
#define PICO_ENC_CLK    14
#define PICO_ENC_DT     15
#define PICO_ENC_SW     6

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
#define BTN_FFT         MASK_X      

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
#define TFT_ORANGE      0xFD20

// --- Scope Settings ---
bool isRunning = true;
float voltsPerDiv = 1.0;  
float timePerDiv = 10.0; 
bool showCursors = false;
float cursorV1_volts = 2.5;
float cursorV2_volts = 0.5;

// --- Gain Settings (FIXED) ---
#define SCOPE_GAIN_LOW  0
#define SCOPE_GAIN_MED  1
#define SCOPE_GAIN_HIGH 2

int currentGainMode = SCOPE_GAIN_MED; 
float hardwareGainFactor = 0.39; // Default for Med

// --- Menu System ---
enum MenuIndex {
    MENU_RUN_STOP = 0,
    MENU_V_DIV,
    MENU_T_DIV,
    MENU_GAIN,      
    MENU_CURSORS_EN,
    MENU_CUR_V1,
    MENU_CUR_V2,
    MENU_COUNT 
};

const char* menuNames[] = {
    "Run/Stop", "V / Div", "T / Div", "Gain", "Cursors", "Cur V1", "Cur V2"
};

// --- State Machine ---
bool isMenuOpen = false;
bool isEditing = false; 
int selectedMenuItem = 0;
bool forceFullRedraw = true; 
bool isRecording = false; 
bool menuDirty = true; 

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

// --- FFT Variables ---
#define NUM_SAMPLES 128
#define SAMPLE_RATE_HZ 500000 
int8_t real_component[NUM_SAMPLES];
int8_t imag_component[NUM_SAMPLES];
int fft_output[NUM_SAMPLES/2];   
bool isFFTMode = false;
bool btnXPressed = true; 
float hanning_window[NUM_SAMPLES];
bool windowInitialized = false;

// --- SNAKE GAME VARIABLES ---
bool isSnakeMode = false;
#define SNAKE_BLOCK_SIZE 10
#define GRID_W (320 / SNAKE_BLOCK_SIZE)
#define GRID_H (240 / SNAKE_BLOCK_SIZE)
int8_t snakeX[100], snakeY[100]; 
int snakeLen = 5;
int snakeDir = 1; // 0:Up, 1:Right, 2:Down, 3:Left
int foodX = 10, foodY = 10;
bool gameOver = false;
bool gameOverDrawn = false; // Prevents Flicker
absolute_time_t lastSnakeMove;

// --- Rotary Encoder Global ---
volatile int rotaryDelta = 0;

// semaphore
struct pt_sem trigger_semaphore ;

// Forward declarations
void computeDFT(); 
void initSnake();
void updateSnake();
void drawSnake();

// ==========================================
// --- INTERRUPT FOR ENCODER ---
// ==========================================
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == PICO_ENC_CLK) {
        if (gpio_get(PICO_ENC_DT)) {
            rotaryDelta--;
        } else {
            rotaryDelta++;
        }
    } else if (gpio == TRIG){
        trigger_isr();
        PT_SEM_SIGNAL(pt, &trigger_semaphore);
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
    gpio_set_irq_enabled_with_callback(PICO_ENC_CLK, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
}

// --- UPDATE GAIN HELPER ---
void updateGainState(int direction) {
    int nextMode = currentGainMode + direction;
    if (nextMode < 0) nextMode = 2; 
    if (nextMode > 2) nextMode = 0; 
    
    currentGainMode = nextMode;
    set_gain((gain_mode_t)currentGainMode); 
    
    switch(currentGainMode) {
        case SCOPE_GAIN_LOW:  hardwareGainFactor = 0.21f; break;
        case SCOPE_GAIN_MED:  hardwareGainFactor = 0.39f; break;
        case SCOPE_GAIN_HIGH: hardwareGainFactor = 1.98f; break;
    }
}

// --- ADC TO VOLT ---
float raw_to_real_volts(uint8_t raw_val) {
    float pin_voltage = (raw_val / 255.0f) * 3.3f;
    return pin_voltage / hardwareGainFactor;
}

short voltToPixel(float volts) {
    float centerVolts = 1.65f / hardwareGainFactor; 
    float pixelsPerDiv = 48.0;
    if (voltsPerDiv < 0.1) voltsPerDiv = 0.1;
    
    float diff = volts - centerVolts;
    float divsFromCenter = diff / voltsPerDiv;
    short y = 120 - (short)(divsFromCenter * pixelsPerDiv);
    return y;
}

// --- Drawing Functions ---
#define PIXELS_PER_DIV 48 

void drawGrid(short width) {
    tft_fillRect(0, 0, width, 240, TFT_BLACK);
    tft_setTextSize(1);
    tft_setTextColor(TFT_LIGHTGREY);

    short centerX = width / 2;
    short centerY = 120;
    float trueCenterV = 1.65f / hardwareGainFactor;

    for (int i = -4; i <= 4; i++) {
        short x = centerX + (i * PIXELS_PER_DIV);
        if (x > 0 && x < width) {
            uint16_t color = (i == 0) ? 0x7BEF : TFT_DARKGREY;
            tft_drawFastVLine(x, 0, 240, color);
            tft_drawFastVLine(x, centerY - 4, 9, TFT_WHITE); 
            float t = (float)i * timePerDiv; 
            char buf[10]; sprintf(buf, "%.0f", t); 
            tft_setCursor(x + 2, 230); tft_writeString(buf);
        }
    }
    for (int i = -2; i <= 2; i++) {
        short y = centerY + (i * PIXELS_PER_DIV);
        if (y >= 0 && y < 240) {
            uint16_t color = (i == 0) ? 0x7BEF : TFT_DARKGREY;
            tft_drawFastHLine(0, y, width, color);
            tft_drawFastHLine(centerX - 4, y, 9, TFT_WHITE); 
            float v = trueCenterV - ((float)i * voltsPerDiv);
            char buf[10]; sprintf(buf, "%.1fV", v);
            tft_setCursor(2, y - 10); tft_writeString(buf);
        }
    }
}

// --- WAVEFORM DRAWING CONSTANTS ---
#define MARGIN_LEFT   35
#define MARGIN_RIGHT  5
#define MARGIN_TOP    25
#define MARGIN_BOTTOM 20

// Optimized Waveform Drawer 
void drawWaveformFromBuffer(short width) {
    float timeScale = timePerDiv / 10.0f; 
    short centerX = width / 2;
    short centerY = 120;
    const int hGridYs[] = {120, 72, 24, 168, 216};
    const int numHGrids = 5;

    int prevX = MARGIN_LEFT;
    int buffIdx0 = (int)(prevX * timeScale); 
    if (buffIdx0 >= CAPTURE_DEPTH) buffIdx0 = CAPTURE_DEPTH - 1;

    float firstVolts = raw_to_real_volts(frame_buf[buffIdx0]);
    int prevY_new = voltToPixel(firstVolts);
    
    if (prevY_new < MARGIN_TOP) prevY_new = MARGIN_TOP;
    if (prevY_new > 240 - MARGIN_BOTTOM) prevY_new = 240 - MARGIN_BOTTOM;

    int prevY_old = oldWaveY[prevX]; 

    for (int x = MARGIN_LEFT + 1; x < width - MARGIN_RIGHT; x++) {
        int buffIdx = (int)(x * timeScale);
        if (buffIdx >= CAPTURE_DEPTH) break; 
        
        float volts = raw_to_real_volts(frame_buf[buffIdx]);
        int currY_new = voltToPixel(volts);
        
        if (currY_new < MARGIN_TOP) currY_new = MARGIN_TOP;
        if (currY_new > 240 - MARGIN_BOTTOM) currY_new = 240 - MARGIN_BOTTOM;

        int currY_old = oldWaveY[x]; 

        if (prevY_old != currY_old || prevY_new != currY_new || true) { 
            tft_drawLine(prevX, prevY_old, x, currY_old, TFT_BLACK); 
            if (abs(x - centerX) % 48 == 0) {
                uint16_t color = (x == centerX) ? 0x7BEF : TFT_DARKGREY;
                tft_drawFastVLine(x, MARGIN_TOP, (240 - MARGIN_BOTTOM) - MARGIN_TOP, color);
            }
            int yMin = (prevY_old < currY_old) ? prevY_old : currY_old;
            int yMax = (prevY_old > currY_old) ? prevY_old : currY_old;
            for (int i = 0; i < numHGrids; i++) {
                int gy = hGridYs[i];
                if (gy >= yMin && gy <= yMax) {
                    uint16_t color = (gy == centerY) ? 0x7BEF : TFT_DARKGREY;
                    tft_drawPixel(x, gy, color);
                    tft_drawPixel(prevX, gy, color); 
                }
            }
            tft_drawLine(prevX, prevY_new, x, currY_new, TFT_YELLOW); 
        }
        oldWaveY[prevX] = prevY_new;
        prevX = x;
        prevY_new = currY_new;
        prevY_old = currY_old; 
    }
    if (prevX < 320) oldWaveY[prevX] = prevY_new;
}

void restoreCursorBg(short y, short width) {
    if (y < 0 || y >= 240) return;
    uint16_t color = TFT_BLACK;
    if (y > 0 && (y % 48 == 0)) color = TFT_DARKGREY;
    if (y == 120) color = 0x7BEF; 
    tft_drawFastHLine(0, y, width, color);
    for (int x = 48; x < width; x += 48) {
        uint16_t vColor = (x == 144) ? 0x7BEF : TFT_DARKGREY;
        tft_drawPixel(x, y, vColor);
    }
}

void drawCursors(short width) {
    static bool wasShowing = false;
    static short oldY1 = -1;
    static short oldY2 = -1;
    static float oldDeltaV = -1.0;

    if (!showCursors) {
        if (wasShowing) {
            if (oldY1 != -1) restoreCursorBg(oldY1, width);
            if (oldY2 != -1) restoreCursorBg(oldY2, width);
            tft_fillRect(5, 25, 100, 15, TFT_BLACK); 
            wasShowing = false;
        }
        return;
    }
    wasShowing = true;

    short newY1 = voltToPixel(cursorV1_volts);
    short newY2 = voltToPixel(cursorV2_volts);

    if (newY1 != oldY1) {
        if (oldY1 != -1) restoreCursorBg(oldY1, width);
        tft_drawFastHLine(0, newY1, width, TFT_MAGENTA);
        oldY1 = newY1;
    } else tft_drawFastHLine(0, newY1, width, TFT_MAGENTA);

    if (newY2 != oldY2) {
        if (oldY2 != -1) restoreCursorBg(oldY2, width);
        tft_drawFastHLine(0, newY2, width, TFT_CYAN);
        oldY2 = newY2;
    } else tft_drawFastHLine(0, newY2, width, TFT_CYAN);
    
    float deltaV = cursorV1_volts - cursorV2_volts;
    if (deltaV < 0) deltaV = -deltaV; 
    
    if (deltaV != oldDeltaV || forceFullRedraw) {
        tft_fillRect(5, 25, 100, 15, TFT_BLACK); 
        tft_setTextSize(1);
        tft_setTextColor(TFT_WHITE);
        tft_setCursor(5, 25); 
        char buf[20];
        sprintf(buf, "dV: %.2f V", deltaV);
        tft_writeString(buf);
        oldDeltaV = deltaV;
    }
}

// --- MAIN DRAW FUNCTION ---
void drawUI() {
    static bool wasSnakeMode = false;
    // === SNAKE MODE ===
    if (isSnakeMode) {
        // One-time Setup when entering Game
        if (!wasSnakeMode) {
            tft_fillScreen(TFT_BLACK); 
            wasSnakeMode = true;
        }
        drawSnake();
        return; 
    } 
    
    // If we just exited snake mode, force a scope redraw
    if (wasSnakeMode) {
        forceFullRedraw = true;
        wasSnakeMode = false;
    }

    // === NORMAL SCOPE UI ===
    short scopeWidth = isMenuOpen ? 240 : 320;
    static bool lastModeWasFFT = false; 

    // Mode Switching Logic
    if (isFFTMode) {
        if (!lastModeWasFFT) {
            tft_fillScreen(TFT_BLACK); 
            lastModeWasFFT = true;     
        }
        tft_fillRect(20, 40, 256, 180, TFT_BLACK); 

        for (int i=0; i<64; i++) {
            int height = fft_output[i] * 0.5; 
            if (height > 180) height = 180; 
            int x = 20 + (i * 4); 
            if (height > 0) {
                 uint16_t color = (height > 100) ? TFT_RED : TFT_GREEN;
                 tft_fillRect(x, 220 - height, 3, height, color);
            }
        }
        tft_setCursor(100, 5); tft_setTextColor(TFT_MAGENTA); tft_setTextSize(2); tft_writeString("FFT MODE");
        tft_setTextSize(1); tft_setTextColor(TFT_WHITE);
        tft_setCursor(20, 225); tft_writeString("0Hz");
        tft_setCursor(240, 225); char buf[32]; sprintf(buf, "%dk", SAMPLE_RATE_HZ / 2 / 1000); tft_writeString(buf);
        
        int maxBin = 0; int maxVal = 0;
        for(int i=1; i<64; i++) { if(fft_output[i] > maxVal) { maxVal = fft_output[i]; maxBin = i; } }
        float peakFreq = maxBin * (SAMPLE_RATE_HZ / 128.0); 
        tft_fillRect(0, 23, 320, 15, TFT_BLACK); 
        tft_setTextColor(TFT_WHITE); tft_setCursor(20, 25); sprintf(buf, "Peak: %.1fkHz", peakFreq/1000.0); tft_writeString(buf);

    } else {
        if (lastModeWasFFT) { forceFullRedraw = true; lastModeWasFFT = false; }
        if (forceFullRedraw) {
            drawGrid(scopeWidth);
            if (isMenuOpen) { tft_fillRect(240, 0, 80, 240, TFT_NAVY); menuDirty = true; }
            for(int i=0; i<320; i++) oldWaveY[i] = 120; 
            forceFullRedraw = false; 
        }
        if (isRunning) drawWaveformFromBuffer(scopeWidth); 
        drawCursors(scopeWidth); 
    }

    // Shared Text
    if (!isFFTMode) {
        static float oldVoltsPerDiv = -1;
        static float oldTimePerDiv = -1;
        tft_setTextColor(TFT_GREEN); 
        if (voltsPerDiv != oldVoltsPerDiv) {
            tft_fillRect(5, 5, 110, 20, TFT_BLACK); tft_setTextSize(2); tft_setCursor(5, 5);
            char buf[32]; sprintf(buf, "%.1f V/d", voltsPerDiv); tft_writeString(buf);
            oldVoltsPerDiv = voltsPerDiv;
        }
        tft_setTextColor(TFT_YELLOW); 
        if (timePerDiv != oldTimePerDiv) {
            tft_fillRect(120, 5, 110, 20, TFT_BLACK); tft_setTextSize(2); tft_setCursor(120, 5);
            char buf[32]; sprintf(buf, "%.0f ms/d", timePerDiv); tft_writeString(buf);
            oldTimePerDiv = timePerDiv;
        }
    }

    static bool oldIsRecording = false;
    if (isRecording != oldIsRecording) {
        tft_fillRect(scopeWidth - 40, 5, 40, 20, TFT_BLACK);
        if (isRecording) { tft_setTextColor(TFT_RED); tft_setCursor(scopeWidth - 40, 5); tft_writeString((char*)"REC"); }
        oldIsRecording = isRecording;
    }

    if (isMenuOpen && menuDirty && !isFFTMode) {
        tft_setTextSize(1);
        for (int i = 0; i < MENU_COUNT; i++) {
            short yPos = 5 + (i * 32); 
            uint16_t boxColor = TFT_NAVY; uint16_t textColor = TFT_LIGHTGREY;
            if (i == selectedMenuItem) { boxColor = isEditing ? TFT_RED : TFT_DARKGREY; textColor = TFT_WHITE; }
            tft_fillRect(240, yPos - 2, 80, 28, boxColor);
            tft_setTextColor(textColor); tft_setCursor(245, yPos + 8); tft_writeString((char*)menuNames[i]);
            tft_setCursor(245, yPos + 18); tft_setTextColor(TFT_WHITE); 
            char buf[32];
            if (i == MENU_V_DIV) sprintf(buf, "%.1fV", voltsPerDiv);
            else if (i == MENU_T_DIV) sprintf(buf, "%.0fms", timePerDiv); 
            else if (i == MENU_GAIN) {
                if (currentGainMode == SCOPE_GAIN_LOW) sprintf(buf, "LOW");
                else if (currentGainMode == SCOPE_GAIN_MED) sprintf(buf, "MED");
                else sprintf(buf, "HIGH");
            }
            else if (i == MENU_CUR_V1) sprintf(buf, "%.1fV", cursorV1_volts);
            else if (i == MENU_CUR_V2) sprintf(buf, "%.1fV", cursorV2_volts);
            else if (i == MENU_RUN_STOP) sprintf(buf, "%s", isRunning ? "RUN" : "STOP");
            else if (i == MENU_CURSORS_EN) sprintf(buf, "%s", showCursors ? "ON" : "OFF");
            else sprintf(buf, " ");
            tft_writeString(buf);
        }
        menuDirty = false; 
    }
}

// --- SNAKE LOGIC ---
void initSnake() {
    snakeX[0] = 5; snakeY[0] = 5;
    snakeLen = 5; snakeDir = 1;
    for(int i=1; i<snakeLen; i++) { snakeX[i] = 5-i; snakeY[i] = 5; }
    foodX = rand() % GRID_W; foodY = rand() % GRID_H;
    gameOver = false;
    gameOverDrawn = false; // Reset the Draw Once flag
    lastSnakeMove = get_absolute_time();
}
void updateSnake() {
    if(gameOver) return;
    
    // Timer check (Speed)
    if(absolute_time_diff_us(lastSnakeMove, get_absolute_time()) < 100000) return;
    lastSnakeMove = get_absolute_time();

    // 1. Predict next Head Position
    int nextX = snakeX[0];
    int nextY = snakeY[0];
    if(snakeDir == 0) nextY--; 
    else if(snakeDir == 1) nextX++; 
    else if(snakeDir == 2) nextY++; 
    else if(snakeDir == 3) nextX--; 

    // 2. Check Collisions (Walls)
    if(nextX < 0 || nextX >= GRID_W || nextY < 0 || nextY >= GRID_H) { gameOver = true; return; }
    // Check Self
    for(int i=0; i<snakeLen; i++) { if(snakeX[i] == nextX && snakeY[i] == nextY) { gameOver = true; return; } }

    // 3. Check Food (Grow) vs Move
    bool grew = false;
    if(nextX == foodX && nextY == foodY) {
        snakeLen++;
        if(snakeLen >= 100) snakeLen = 100;
        foodX = rand() % GRID_W; foodY = rand() % GRID_H;
        grew = true;
    }

    // 4. ERASE TAIL (The Flicker Fix)
    // If we didn't grow, the last segment (tail) will disappear. Erase it now.
    if (!grew) {
        tft_fillRect(snakeX[snakeLen-1]*SNAKE_BLOCK_SIZE, snakeY[snakeLen-1]*SNAKE_BLOCK_SIZE, SNAKE_BLOCK_SIZE, SNAKE_BLOCK_SIZE, TFT_BLACK);
    }

    // 5. Shift Body
    for(int i=snakeLen-1; i>0; i--) { snakeX[i] = snakeX[i-1]; snakeY[i] = snakeY[i-1]; }
    
    // 6. Update Head
    snakeX[0] = nextX;
    snakeY[0] = nextY;
}
void drawSnake() {
    // --- Game Over Screen ---
    if(gameOver) {
        if (!gameOverDrawn) {
            tft_fillScreen(TFT_BLACK);
            tft_setCursor(80, 100); tft_setTextColor(TFT_RED); tft_setTextSize(3); tft_writeString("GAME OVER");
            tft_setCursor(60, 140); tft_setTextColor(TFT_WHITE); tft_setTextSize(1); tft_writeString("Press BACK to Exit");
            gameOverDrawn = true;
        }
        return;
    }
    
    // NO tft_fillScreen HERE! That caused the flicker.
    
    // Draw Food
    tft_fillRect(foodX*SNAKE_BLOCK_SIZE, foodY*SNAKE_BLOCK_SIZE, SNAKE_BLOCK_SIZE, SNAKE_BLOCK_SIZE, TFT_RED);
    
    // Draw Snake
    // Optimization: We technically only need to redraw Head (Green) and the segment after it (Orange).
    // But redrawing the whole small body is fast enough and safer.
    for(int i=0; i<snakeLen; i++) {
        uint16_t c = (i==0) ? TFT_GREEN : TFT_ORANGE;
        tft_fillRect(snakeX[i]*SNAKE_BLOCK_SIZE, snakeY[i]*SNAKE_BLOCK_SIZE, SNAKE_BLOCK_SIZE, SNAKE_BLOCK_SIZE, c);
    }
}

void handleInput() {
    uint32_t buttons = seesaw_read_buttons();
    uint16_t joyX = seesaw_read_analog(PIN_JOY_X);
    uint16_t joyY = seesaw_read_analog(PIN_JOY_Y);
    
    bool currentEncSw = !gpio_get(PICO_ENC_SW);
    int delta = rotaryDelta;
    rotaryDelta = 0;

    if (isSnakeMode) {
        // --- JOYSTICK INVERSION FIX ---
        // JoyX > High = LEFT (3), JoyX < Low = RIGHT (1)
        if(joyY < JOY_THRESHOLD_LOW && snakeDir != 2) snakeDir = 0; // Up
        if(joyX > JOY_THRESHOLD_HIGH && snakeDir != 1) snakeDir = 3; // LEFT (Inverted from Right)
        if(joyY > JOY_THRESHOLD_HIGH && snakeDir != 0) snakeDir = 2; // Down
        if(joyX < JOY_THRESHOLD_LOW && snakeDir != 3) snakeDir = 1; // RIGHT (Inverted from Left)
        
        if(!(buttons & BTN_BACK)) {
            isSnakeMode = false;
            forceFullRedraw = true; 
        }
        updateSnake();
        return; 
    }

    static absolute_time_t lastStart = 0;
    static int startCount = 0;
    if (!(buttons & BTN_RECORD)) {
        if (!btnRecordPressed) {
            absolute_time_t now = get_absolute_time();
            if(absolute_time_diff_us(lastStart, now) < 5000000) {
                startCount++;
            } else {
                startCount = 1;
            }
            lastStart = now;
            
            if(startCount >= 3) {
                isSnakeMode = true;
                initSnake();
                startCount = 0;
            }
            isRecording = !isRecording; 
        }
        btnRecordPressed = true;
    } else btnRecordPressed = false;

    if ((buttons & MASK_X)) { if (!btnXPressed) { isFFTMode = !isFFTMode; btnXPressed = true; } } 
    else { btnXPressed = false; }

    if (!(buttons & BTN_MENU)) { if (!btnMenuPressed) { isMenuOpen = !isMenuOpen; isEditing = false; forceFullRedraw = true; menuDirty = true; } btnMenuPressed = true; } else btnMenuPressed = false;

    bool nav_next = (joyY > JOY_THRESHOLD_HIGH); 
    bool nav_prev = (joyY < JOY_THRESHOLD_LOW);
    
    if (nav_next || nav_prev || delta != 0) { if (isMenuOpen) menuDirty = true; }
    
    if (isMenuOpen) {
        if (isEditing) {
            if (delta != 0) {
                switch(selectedMenuItem) {
                    case MENU_V_DIV: voltsPerDiv += (delta * 0.1); if (voltsPerDiv < 0.1) voltsPerDiv = 0.1; forceFullRedraw = true; break;
                    case MENU_T_DIV: timePerDiv += (delta * 1.0); if (timePerDiv < 1.0) timePerDiv = 1.0; forceFullRedraw = true; break;
                    case MENU_GAIN: updateGainState(delta); forceFullRedraw = true; break;
                    case MENU_CUR_V1: cursorV1_volts += (delta * 0.1); break;
                    case MENU_CUR_V2: cursorV2_volts += (delta * 0.1); break;
                }
            }
            bool confirm = !(buttons & BTN_CONFIRM);
            bool back = !(buttons & BTN_BACK);
            if (currentEncSw && !encSwPressed) { confirm = true; encSwPressed = true; } else if (!currentEncSw) encSwPressed = false;
            if (confirm && !btnConfirmPressed) { isEditing = false; btnConfirmPressed = true; menuDirty = true; forceFullRedraw = true; }
            if (back && !btnBackPressed) { isEditing = false; btnBackPressed = true; menuDirty = true; forceFullRedraw = true; }
        } else {
            if (nav_prev) { if (!joyUpHeld) { selectedMenuItem--; if (selectedMenuItem < 0) selectedMenuItem = MENU_COUNT - 1; menuDirty = true; } joyUpHeld = true; } else joyUpHeld = false;
            if (nav_next) { if (!joyDownHeld) { selectedMenuItem++; if (selectedMenuItem >= MENU_COUNT) selectedMenuItem = 0; menuDirty = true; } joyDownHeld = true; } else joyDownHeld = false;
            bool confirm = !(buttons & BTN_CONFIRM);
            if (currentEncSw && !encSwPressed) { confirm = true; encSwPressed = true; } else if (!currentEncSw) encSwPressed = false;
            if (confirm && !btnConfirmPressed) {
                if (selectedMenuItem == MENU_RUN_STOP) { isRunning = !isRunning; menuDirty = true; }
                else if (selectedMenuItem == MENU_CURSORS_EN) { showCursors = !showCursors; menuDirty = true; }
                else { isEditing = true; menuDirty = true; }
                btnConfirmPressed = true; forceFullRedraw = true;
            }
        }
    } 
    if ((buttons & BTN_CONFIRM)) btnConfirmPressed = false;
    if ((buttons & BTN_BACK)) btnBackPressed = false;
}

// ==================== Graphics thread ====================
static PT_THREAD (protothread_graphics(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1){
        handleInput();
        drawUI();
        PT_YIELD_usec(16667); //60FPS 
    }
    PT_END(pt);
}

// ==================== Trigger thread =====================
static PT_THREAD (protothread_trigger(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1){
        PT_SEM_WAIT(pt, &trigger_semaphore); 
        trigger_copy();
    }
    PT_END(pt);
}

// ==================== Blinky Thread ======================
static PT_THREAD (protothread_blinky(struct pt *pt))
{
    PT_BEGIN(pt);
    static bool led_val = false;
    while(1){
        gpio_put(PICO_DEFAULT_LED_PIN, led_val);
        led_val = !led_val;
        PT_YIELD_usec(200000); 
    }
    PT_END(pt);
}

// ====================== FFT CALCULATION THREAD =================
static PT_THREAD (protothread_fft_calc(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1){
        if (isFFTMode && !isSnakeMode) {
             computeDFT(); 
             PT_YIELD_usec(10000); 
        } else {
             PT_YIELD_usec(100000); 
        }
    }
    PT_END(pt);
}

// Entry point for core 0
void core0_entry() {
    pt_add_thread(protothread_trigger);
    pt_add_thread(protothread_graphics);
    pt_schedule_start ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_blinky);
    pt_add_thread(protothread_fft_calc); 
    pt_schedule_start ;
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

    rotary_init(); 

    tft_init_hw();
    tft_begin();
    tft_setRotation(3); 
    tft_fillScreen(TFT_BLACK);
    
    initDac();
    int dac_val = setVoltage(CHAN_TRIG, 1.65f);
    
    for(int i=0; i<320; i++) oldWaveY[i] = 120;

    init_adc_capture();
    init_trigger();
    
    // Set Initial Gain State
    currentGainMode = SCOPE_GAIN_MED;
    updateGainState(0); // Applies factor 0.39 and relays

    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    core0_entry();

    while (true) {
        // Core 0 loop is handled by pt_schedule_start
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

void computeDFT() {
    if (!windowInitialized) {
        for (int i = 0; i < NUM_SAMPLES; i++) {
            hanning_window[i] = 0.5 * (1.0 - cos(2.0 * 3.14159 * i / (NUM_SAMPLES - 1)));
        }
        windowInitialized = true;
    }

    for (int k = 0; k < 64; k++) { 
        float sumReal = 0;
        float sumImag = 0;
        for (int t = 0; t < 128; t++) {
            float angle = 2 * 3.14159 * t * k / 128;
            
            // Adjust DFT for new gain scale (remove effective DC offset)
            float sample = raw_to_real_volts(frame_buf[t]); 
            // We need to center this. Center V is 1.65 / Gain
            float center = 1.65f / hardwareGainFactor;
            
            sample -= center; // Remove DC component for FFT
            sample *= hanning_window[t];

            sumReal += sample * cos(angle);
            sumImag += sample * -sin(angle);
        }
        // Scale magnitude for display (tuned for visual height)
        fft_output[k] = (int)(sqrt(sumReal*sumReal + sumImag*sumImag) * 50.0);
    }
}