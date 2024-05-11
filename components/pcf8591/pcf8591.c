/*************************************************************************

NXP PCF8591 control code.
This chip is an 8 bit A/D converter with 4 channels and D/A converter with 1 channel.
It is read via I2C bus (max freq is 100 kHz)
We use it in single-ended mode (4 input channels). 
Reference voltage for ADC is a 3.3V precision voltage regulator: NCP5146, 1% accuracy

*****************************************************************************/

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include "pcf8591.h"


#define ERR(ret, format, arg...)                                       \
   {                                                                   \
         ESP_LOGE(TAG, "%s: " format, __func__ , ## arg);              \
         return ret;                                                   \
   }
   
  
/* i2c bus where PCF8591 is connected */
#define I2C_BUS                 I2C_NUM_1 
#define I2C_MASTER_TIMEOUT_MS   100

static const char* TAG = __FILE__;
static uint8_t chipAddr;
static float voltage, current, battery1;


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



float getSupplyVoltage(void)
{
   return voltage;
}


float getSupplyBattery1(void)
{
   return battery1;
}


float getSupplyCurrent(void)
{
   return current;
}


/* 
This function gets called at fixed intervals
Voltage: It reads the ADC#0, connected to the main power supply (max voltage is 9.3V).
Current: It reads the ADC#1, connected to a current sensing circuit (max current is 3A). 
Low currents (in tens of mA) are overestimated.
*/
void readPowerSupply(void)
{
uint8_t adc[4];  // Store ADC values
esp_err_t ret;

   if (chipAddr == 0) return;  // Could not initialize

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
      battery1 = factor_v2*adc[1];  // Voltage level at the middle of the battey pack (1 18650 if 2 in series are used)
   else  // if voltage is too low, it means cable is not connected
      battery1 = voltage/2;  // if mid-point cable is not connected, assume this is half the battery voltage
   
   return;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   ERR(, "Cannot read data from PCF8591");       
}




