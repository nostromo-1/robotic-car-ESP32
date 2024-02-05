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

#ifndef _TINY_WIIMOTE_H_
#define _TINY_WIIMOTE_H_

#define RECEIVED_DATA_MAX_LEN     (50)
#define TWII_OFFSET_BTNS1 (2)
#define TWII_OFFSET_BTNS2 (3)
#define TWII_OFFSET_EXTCTRL (4) // Offset for Extension Controllers data


typedef struct {
  uint8_t number;
  uint8_t data[RECEIVED_DATA_MAX_LEN];
  uint8_t len;
} TinyWiimoteData_t;


typedef void (*TwHciInterface_t)(uint8_t *data, size_t len);

void TinyWiimoteInit(TwHciInterface_t hciInterface);
int TinyWiimoteAvailable(void);
TinyWiimoteData_t TinyWiimoteRead(void);
void TinyWiimote_setPlayerLEDs(uint8_t leds);
void TinyWiimote_rumble(bool rumble);
bool TinyWiimote_connected(void);

void TinyWiimoteResetDevice(void);
bool TinyWiimoteDeviceIsInited(void);
void handleHciData(uint8_t* data, size_t len);

char* format2Hex(uint8_t* data, uint16_t len);

#endif // _TINY_WIIMOTE_H_

