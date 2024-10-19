#ifndef OLED96_H
#define OLED96_H

/*************************************************************************
OLED control code, for the 0.96" SSD1306 128x64 OLED display.
Based on the following original code:

//
// OLED96
// Library for accessing the 0.96" SSD1306 128x64 OLED display
// Written by Larry Bank (bitbank@pobox.com)
// Copyright (c) 2017 BitBank Software, Inc.
// Project started 1/15/2017

*****************************************************************************/

#define OLED_MAX_LINE_SIZE 17   // Max number of characters+1 in a line on the display, small font

// Initialize the OLED96 library for a specific I2C address
int oledInit(uint8_t iAddress);


// Turns off the display and closes the I2C handle
void oledShutdown(void);

// Clear display
void oledClear(void);

// Write a text string to the display at x (column 0-127) and y (row 0-7)
// bLarge = 0 - 8x8 font, bLarge = 1 - 16x24 font
int oledWriteString(int x, int y, const char *szText, bool bLarge);


// Sets the contrast (brightness) level of the display
// Valid values are 0-255 where 0=off and 255=max brightness
int oledSetContrast(uint8_t ucContrast);

// Sets the inversion state of the display
// 0 is normal image, anything else inverted image 
int oledSetInversion(bool invert);

// Write an 8x8 bitmap to display
// graph is an 8 byte array, glyph must be turned 90 degrees to the right
int oledSetBitmap8x8(uint8_t x, uint8_t y, const uint8_t *graph);

// Write a message in big font on display
int oledBigMessage(uint8_t line, const char *msg);


#endif // OLED96_H
