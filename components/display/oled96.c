/*************************************************************************
OLED control code, for the 0.96" SSD1306 128x64 OLED display via I2C interface.
Based on code written by Larry Bank (bitbank@pobox.com)

*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "oled96.h"
#include "fonts.h"

#define I2C_BUS                 I2C_NUM_0    // I2C bus of display
#define I2C_MASTER_TIMEOUT_MS   150          // 100 ms creates problemas (timeout) with IMU
#define ACK_CHECK_EN            0x1          /* I2C master will check ack from slave*/
#define ACK_CHECK_DIS           0x0          /* I2C master will not check ack from slave */
#define ACK_VAL                 0x0          /* I2C ack value */
#define NACK_VAL                0x1          /* I2C nack value */


#define ERR(ret, format, arg...)                                       \
   {                                                                   \
         printf("%s: " format "\n" , __func__ , ## arg);      \
         return ret;                                                   \
   }
   

static int iScreenOffset; // current write offset of screen data
static uint8_t displayAddr;
static SemaphoreHandle_t mutex = NULL;

    
static int oledWriteCommand(uint8_t);
static int oledWriteCommand2(uint8_t c, uint8_t d);
static int oledSetPosition(int x, int y);
static int oledWriteDataBlock(const uint8_t *ucBuf, int iLen);


// Opens a handle to the I2C device using pigpio library
// Initializes the OLED controller into "page mode"
// Prepares the font data for the orientation of the display
int oledInit(uint8_t iAddr)
{
esp_err_t rc;
uint8_t initbuf[]={0x00,0xae,0xa8,0x3f,0xd3,0x00,0x40,0xa0,0xa1,0xc0,0xc8,
			0xda,0x12,0x81,0xff,0xa4,0xa6,0xd5,0x80,0x8d,0x14,0x20,0x02};

    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL) goto rw_error;
    displayAddr = iAddr;
    rc = i2c_master_write_to_device(I2C_BUS, displayAddr, initbuf, sizeof(initbuf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (rc != ESP_OK) goto rw_error;   
    
    oledFill(0x00);    // Set display memory to zero   
    oledWriteCommand(0xAF);  // turn on OLED
    oledSetContrast(128);
    //RotateFont90();     // fix font orientation for OLED
    return 0;  

   // error handling if read operation from I2C bus failed
rw_error:
   displayAddr = 0;
   ERR(-1, "Cannot write data to display"); 
} 


// Fill the frame buffer with a byte pattern
// e.g. all off (0x00) or all on (0xff)
int oledFill(uint8_t ucData)
{
int y, rc;
uint8_t temp[128];

	memset(temp, ucData, sizeof(temp));
	for (y=0; y<8; y++) {
      xSemaphoreTake(mutex, portMAX_DELAY);
		rc = oledSetPosition(0, y); // set to (0,Y)
		rc |= oledWriteDataBlock(temp, sizeof(temp)); // fill line with data byte
      xSemaphoreGive(mutex);
      if (rc<0) goto rw_error; 
	} 
    
	return 0;
   
   // error handling if operation from I2C bus failed
rw_error:
   ERR(-1, "Cannot write data to display");   
}


// Send commands to position the "cursor" to the given
// row and column. It assumes that the mutex is locked
static int oledSetPosition(int x, int y)
{
uint8_t buf[4];
esp_err_t rc;

   if (y<0 || y>7 || x<0 || x>127) ERR(-1, "Invalid coordinates in display");
   buf[0] = 0x00;      // 0x00 is the command introducer
   buf[1] = 0xB0 | y;  // go to page Y
   buf[2] = 0x00 | (x & 0x0f);   // lower col addr
   buf[3] = 0x10 | (x >> 4);     // upper col addr
   rc = i2c_master_write_to_device(I2C_BUS, displayAddr, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
   if (rc != ESP_OK) goto rw_error; 
	iScreenOffset = (y*128)+x;
   return 0;
   
   // error handling if read operation from I2C bus failed
rw_error:
   ERR(-1, "Cannot write data to display");  
}


// Write a block of pixel data to the OLED
// Length can be anything from 1 to 128 (whole line)
static int oledWriteDataBlock(const uint8_t *ucBuf, int iLen)
{
uint8_t ucTemp[129];
esp_err_t rc;
int rest;

	 ucTemp[0] = 0x40; // data command
    
    if (!ucBuf) ERR(-1, "Error writing to display: Invalid buffer");
    if (iLen < 0 || iLen > 128) ERR(-1, "Error writing to display: Invalid length for display data");
    if (iLen == 0) return 0;
    memcpy(&ucTemp[1], ucBuf, iLen);
    
	// Keep a copy in local buffer, taking care not to overflow to beginning of row (display in page mode)
    rest = 128 - iScreenOffset%128;
    if (rest >= iLen) {
        rc = i2c_master_write_to_device(I2C_BUS, displayAddr, ucTemp, iLen+1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        if (rc != ESP_OK) goto rw_error;       
        iScreenOffset += iLen;
    }
    else {
        rc = i2c_master_write_to_device(I2C_BUS, displayAddr, ucTemp, rest+1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        if (rc != ESP_OK) goto rw_error;         
        iScreenOffset += rest;
    } 
    return 0;
    
   // error handling if read operation from I2C bus failed
rw_error:
   ERR(-1, "Cannot write data to display");     
}


// Send a single byte command to the OLED controller
static int oledWriteCommand(uint8_t c)
{
esp_err_t rc;
uint8_t buf[2];

    buf[0] = 0x00;  // 0x00 is the command introducer
    buf[1] = c;
    rc = i2c_master_write_to_device(I2C_BUS, displayAddr, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);   
    if (rc != ESP_OK) ERR(-1, "Error writing to display"); 
    return 0;
}

// Send a two byte command to the OLED controller
static int oledWriteCommand2(uint8_t c, uint8_t d)
{
esp_err_t rc;
uint8_t buf[3];

    buf[0] = 0x00;  // 0x00 is the command introducer
    buf[1] = c;
    buf[2] = d;
    rc = i2c_master_write_to_device(I2C_BUS, displayAddr, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);     
    if (rc != ESP_OK) ERR(-1, "Error writing to display"); 
    return 0;    
} 



int oledSetContrast(uint8_t ucContrast)
{
int rc;

   xSemaphoreTake(mutex, portMAX_DELAY);
	rc = oledWriteCommand2(0x81, ucContrast);
   xSemaphoreGive(mutex);
	return rc;
} 



// Sends a command to turn off the OLED display
// Closes the I2C file handle
void oledShutdown()
{
    printf("Closing display...\n");

    xSemaphoreTake(mutex, portMAX_DELAY);
    oledWriteCommand(0xAE); // turn off OLED
    displayAddr = 0;
    xSemaphoreGive(mutex);
}



int oledSetInversion(bool invert)
{
int rc;

    xSemaphoreTake(mutex, portMAX_DELAY);  
    rc = oledWriteCommand(invert?0xA7:0xA6);
    xSemaphoreGive(mutex);
    return rc;
}




// Draw a string of small (8x8) or large (16x24) characters
// At the given col+row
// String must have a maximum of 16 characters for small font and 8 for large font
int oledWriteString(int x, int y, const char *szMsg, bool bLarge)
{
int i, j, iLen, rc;
const uint8_t *s;
uint8_t buf[16*8];

   if (y<0 || y>7 || x<0 || x>127) ERR(-1, "Invalid coordinates for display");
   if (!szMsg) ERR(-1, "Invalid string");
    
	iLen = strlen(szMsg);
	if (bLarge) {  // draw 16x24 font, 8 characters per line
        if (iLen>8) ERR(-1, "length of string with large font is over 8 characters");
        for (i=0; i<3; i++) {
            xSemaphoreTake(mutex, portMAX_DELAY); 
            rc = oledSetPosition(x, y+i);
            for (j=0; j<iLen; j++) {
                s = &ucFont[9728 + (uint8_t)(szMsg[j]&0x7F)*64];  // 0x7F: large font has only 128 characters
                memcpy(buf+j*16, s+16*i, 16);
            }
            rc |= oledWriteDataBlock(buf, iLen*16);
            xSemaphoreGive(mutex); 
            if (rc < 0) goto rw_error;  
        }
	}
	else {  // draw 8x8 font, 16 characters per line
      if (iLen>16) ERR(-1, "length is over 16 characters");
      xSemaphoreTake(mutex, portMAX_DELAY); 
		rc = oledSetPosition(x, y);
		for (i=0; i<iLen; i++) memcpy(buf+i*8, &ucFont[(uint8_t)szMsg[i]*8], 8);
      rc |= oledWriteDataBlock(buf, iLen*8); // write character pattern
      xSemaphoreGive(mutex); 
      if (rc < 0) goto rw_error; 
	}

	return 0;
   
   // error handling if read operation from I2C bus failed
rw_error:
   ERR(-1, "Cannot write data to display");    
} 



// Write a message in big font on display
// line: 0 or 1 (writes message in lines 2,3,4 or 5,6,7 respectively)
int oledBigMessage(int line, const char *msg)
{
static const char *empty = "        ";
char *buf;
    
    if (line<0 || line>1) ERR(-1, "line must be 0 or 1");
    if (msg) buf = (char *)msg; 
    else buf = (char *)empty;
    
    return oledWriteString(0, 2+3*line, buf, true); 
}



// Write an 8x8 bitmap to display
// graph is an 8 byte array, glyph must be turned 90 degrees to the right
int oledSetBitmap8x8(int x, int y, const uint8_t* graph)
{
   static const uint8_t empty[] = {0, 0, 0, 0, 0, 0, 0, 0};  // empty space
   const uint8_t *buf;
   int rc;
    
   if (y<0 || y>7 || x<0 || x>127) ERR(-1, "Invalid coordinates for display");
   if (graph) buf = graph;
   else buf = empty;
    
   xSemaphoreTake(mutex, portMAX_DELAY);
   rc = oledSetPosition(x, y);
   rc |= oledWriteDataBlock(buf, 8);
   xSemaphoreGive(mutex); 
   
	return rc;
} 



/*
This code is no longer used. Instead, the ucFont array now contains the rotated characters.
This way, the array can be loaded in flash memory and does not use RAM


// Fix the orientation of the font image data, defined in fonts.c
static void RotateFont90(void)
{
uint8_t ucTemp[64];
int i, j, x, y;
uint8_t c, c2, ucMask, *s, *d;

	// Rotate the 8x8 font
	for (i=0; i<256; i++) {  // fix 8x8 font by rotating it 90 deg clockwise
		s = &ucFont[i*8];
		ucMask = 0x1;
		for (y=0; y<8; y++) {
			c = 0;
			for (x=0; x<8; x++) {
				c >>= 1;
				if (s[x] & ucMask) c |= 0x80;
			}
			ucMask <<= 1;
			ucTemp[7-y] = c;
		}
		memcpy(s, ucTemp, 8);
	}
   
	// Rotate the 16x32 font
	for (i=0; i<128; i++) { // only 128 characters
		for (j=0; j<4; j++) {
			s = &ucFont[9728 + 12 + (i*64) + (j*16)];
			d = &ucTemp[j*16];
			ucMask = 0x1;
			for (y=0; y<8; y++) {
				c = c2 = 0;
				for (x=0; x<8; x++) {
					c >>= 1;
					c2 >>= 1;
					if (s[(x*2)] & ucMask) c |= 0x80;
					if (s[(x*2)+1] & ucMask) c2 |= 0x80;
				}
				ucMask <<= 1;
				d[7-y] = c;
				d[15-y] = c2;
			} 
		} 
		memcpy(&ucFont[9728 + (i*64)], ucTemp, 64);
	}
} 
*/


