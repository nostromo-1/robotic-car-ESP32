/*************************************************************************

NXP PCF8591 control code.
This chip is an 8 bit A/D converter with 4 channels and D/A converter with 1 channel.
It is read via I2C bus (max freq is 100 kHz)
We use it in single-ended mode (4 input channels). 
Reference voltage for ADC is a 3.3V precision voltage regulator: NCP5146, 1% accuracy

*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pcf8591.h"
#include "oled96.h"


#define ERR(ret, format, arg...)                                       \
   {                                                                   \
         fprintf(stderr, "%s: " format "\n" , __func__ , ## arg);      \
         return ret;                                                   \
   }
   
  
/* i2c bus where PCF8591 is connected */
#define I2C_BUS                 I2C_NUM_1 
#define I2C_MASTER_TIMEOUT_MS   100
#define ACK_CHECK_EN            0x1          /* I2C master will check ack from slave*/
#define ACK_CHECK_DIS           0x0          /* I2C master will not check ack from slave */
#define ACK_VAL                 0x0          /* I2C ack value */
#define NACK_VAL                0x1          /* I2C nack value */

static uint8_t chipAddr;
static float voltage, current;


// 3.3 is the voltage reference (+-1%), 255 are the steps (8 bits ADC resolution)
// 22000 and 12100 are the precision (1%) resistors in series connected to ADC#0 for battery voltage
// Accuracy: about 13 mV (3.3/255) quantisation error due to ADC, times 2.82 (resistors), 
// which is a total error of about +-18 mV (+-13/2*2.82)
// Max. allowed voltage value: 9.3 V
static const float factor_v = 3.3/255*(22000+12100)/12100; 

// ADC#3 is connected to the middle point of the battery pack, via 2 22k precision (1%) resistors
static const float factor_v2 = 3.3/255*2; 

// 1100 and 100 are the precision (1%) resistors in the current sensing circuit connected to ADC#1
// 0.1 is the sensing resistor (1%)
// current = voltage measured / 1.1
// Accuracy: 6 mA due to offset voltage in opamp (600 uV in NPN stage, thus 0.6 mV/0.1) 
// plus 12 mA due to ADC error (13 mV/1.1), which is a total error of about +-9 mA
// Max. allowed current value: 3 A
static const float factor_i = 3.3/255/(0.1*1100/100);  


int setupPCF8591(uint8_t addr)
{
esp_err_t rc;
uint8_t buf[2];
   
   // Initialize chip
   buf[0] = 0b01000110;  // Control byte: set autoincrement flag, start reading channel 2, single ended inputs, enable DAC 
   buf[1] = 0;           // Set DAC output to zero
   rc = i2c_master_write_to_device(I2C_BUS, addr, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);       
   if (rc != ESP_OK) goto rw_error;  
   
   vTaskDelay(pdMS_TO_TICKS(10));   // Does not read correctly without delay
   
   // Read previous conversion and ignore it
   rc = i2c_master_read_from_device(I2C_BUS, addr, buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);        
   if (rc != ESP_OK) goto rw_error;  
   
   chipAddr = addr;
   return 0;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   ERR(-1, "Cannot read/write data from PCF8591");   
}



float getMainVoltageValue(void)
{
   return voltage;
}



/* 
This function gets called at fixed intervals
Voltage: It reads the ADC#0, connected to the main power supply (max voltage is 9.3V).
Current: It reads the ADC#1, connected to a current sensing circuit (max current is 3A). 
Low currents (in tens of mA) are overestimated.
It displays a symbol in the display according to the battery status.
*/
void checkPower(void)
{
uint8_t battery_glyph[] = {0, 254, 130, 131, 131, 130, 254, 0};  // glyph for empty battery
int step;
uint8_t adc[4];  // Store ADC values
char str[17];
float bat1, bat2;
static char str_old[17];
static int n, old_step = -1;
static const uint32_t maxUndervoltageTime = 4000;  // Milliseconds with undervoltage before shutdown is triggered
static uint32_t underVoltageTime = 0;
static int64_t previousTick;
esp_err_t ret;
int64_t tick;

   if (chipAddr == 0) vTaskDelete(NULL);  // Could not initialize: task deletes itself
   tick = esp_timer_get_time();
   if (previousTick == 0) previousTick = tick;  // Only first time

   /* 
      Read all 4 ADC channels: ch2, ch3, ch0, ch1 into adc array
      ch2 was triggered in the previous read, so the value we get 
      is a past value (by the period of the call to this function)
      Reading in this order is done so that ch0 (V) and ch1 (I) 
      are triggered and read in this function call, no delay between both
   */
   ret = i2c_master_read_from_device(I2C_BUS, chipAddr, adc, sizeof(adc), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
   if (ret != ESP_OK) goto rw_error;  

   voltage = factor_v*adc[2];  // Battery voltage level, channel 0
   current = factor_i*adc[3];  // Current draw, channel 1
   if (adc[1]>10)   // channel 3, if there is a non-zero reading, a cable is connected at the mid-battery point
      bat1 = factor_v2*adc[1];  // Voltage level at the middle of the battey pack (1 18650 if 2 in series are used)
   else  // if voltage is too low, it means cable is not connected
      bat1 = voltage/2;  // if mid-point cable is not connected, assume this is half the battery voltage
   bat2 = voltage - bat1;
   
   if (voltage < 6.2) step = 0;        // Battery at 0%
   else if (voltage < 6.6) step = 64;  // Battery at 20%
   else if (voltage < 7.0) step = 64+32;      // Battery at 40%  
   else if (voltage < 7.4) step = 64+32+16;   // Battery at 60%
   else if (voltage < 7.8) step = 64+32+16+8; // Battery at 80%
   else step = 64+32+16+8+4;  // Battery at 100%
   
   battery_glyph[2] += step;
   battery_glyph[3] += step;
   battery_glyph[4] = battery_glyph[3];
   battery_glyph[5] = battery_glyph[2];     
    
   // If battery state changed, update battery symbol on display
   if (step != old_step) {    
      oledSetBitmap8x8(14*8, 0, battery_glyph);  
      old_step = step;
   }
   
   // Symbol blinks when battery low
   if (step <= 64) { 
      if (n++ & 1) oledSetBitmap8x8(14*8, 0, NULL);
      else oledSetBitmap8x8(14*8, 0, battery_glyph);
   }
    
   // Update display only if values changed (it is a slow operation)
   snprintf(str, sizeof(str), "%.1fV %.2fA", voltage, current);
   if (strcmp(str, str_old)) {
      oledWriteString(0, 1, str, false);
      strcpy(str_old, str);
   }

   // Shutdown if voltage is too low for a long period
   if (bat1<2.9 || bat2<2.9) underVoltageTime += (tick-previousTick)/1000;
   else underVoltageTime = 0;
   if (underVoltageTime >= maxUndervoltageTime) {  
      oledBigMessage(0, "Battery!");   
      oledBigMessage(1, "SHUTDOWN");
      esp_system_abort("Out of battery");   // Shutdown chip
   }
   
   previousTick = tick; 
   return;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   ERR(, "Cannot read data from PCF8591");       
}




