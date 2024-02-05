// Copyright (c) 2020 Daiki Yasuda
//
// This is licensed under
// - Creative Commons Attribution-NonCommercial 3.0 Unported
// - https://creativecommons.org/licenses/by-nc/3.0/
// - Or see LICENSE.md
//
// The short of it is...
//   You are free to:
//     Share — copy and redistribute the material in any medium or format
//     Adapt — remix, transform, and build upon the material
//   Under the following terms:
//     NonCommercial — You may not use the material for commercial purposes.

#ifndef __ESP32_WIIMOTE_H__
#define __ESP32_WIIMOTE_H__

#include "freertos/queue.h"
#include "esp_bt.h"
#include "TinyWiimote.h"

typedef struct {
        uint8_t xStick;
        uint8_t yStick;
        uint8_t xAxis;
        uint8_t yAxis;
        uint8_t zAxis;
        uint8_t cBtn;
        uint8_t zBtn;
} WiimoteNunchukState_t;

typedef enum {
  FILTER_NONE                = 0x0000,
  FILTER_REMOTE_BUTTON       = 0x0001,
  FILTER_NUNCHUK_BUTTON      = 0x0002,
  FILTER_NUNCHUK_STICK       = 0x0004,
  FILTER_NUNCHUK_ACCEL       = 0x0008,
} WiimoteFilter_t;

typedef enum {
  ACTION_IGNORE,
} WiimoteAction_t;

typedef enum {
    BUTTON_LEFT       = 0x0100,
    BUTTON_RIGHT      = 0x0200,
    BUTTON_UP         = 0x0800,
    BUTTON_DOWN       = 0x0400,
    BUTTON_A          = 0x0008,
    BUTTON_B          = 0x0004,
    BUTTON_PLUS       = 0x1000,
    BUTTON_HOME       = 0x0080,
    BUTTON_MINUS      = 0x0010,
    BUTTON_ONE        = 0x0002,
    BUTTON_TWO        = 0x0001
} WiimoteButton_t;
    

void ESP32Wiimote_init(void);
void ESP32Wiimote_task(void);
bool ESP32Wiimote_available(void);
bool ESP32Wiimote_connected(void);
bool ESP32Wiimote_waitConnection(uint8_t);
WiimoteButton_t ESP32Wiimote_getButtonState(void);
WiimoteNunchukState_t ESP32Wiimote_getNunchukState(void);
void ESP32Wiimote_setPlayerLEDs(uint8_t leds);
void ESP32Wiimote_rumble(bool rumble);
void ESP32Wiimote_addFilter(int action, int filter);
void ESP32Wiimote_reset(void);

#endif // __ESP32_WIIMOTE_H__
