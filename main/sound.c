/*************************************************************************

Reproduce un fichero de audio en formato WAV

*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/dac_continuous.h"
#include "esp_log.h"
#include "esp_check.h"

#include "sound.h"

#define NUMBER32(p) (*(p) + (*(p+1)<<8) + (*(p+2)<<16) + (*(p+3)<<24))
#define NUMBER16(p) (*(p) + (*(p+1)<<8))
#define BUFSIZE 2048
#define STACK_SIZE 2048

#define READ_ATOMIC(var) atomic_load_explicit(&var, memory_order_acquire)
#define WRITE_ATOMIC(var,value) atomic_store_explicit(&var, value, memory_order_release)


static const char* TAG = __FILE__;
static int ampliPIN;   // GPIO pin for amplifier shutdown
static dac_continuous_handle_t dac_handle;
static uint8_t buf[BUFSIZE], buf_dac[BUFSIZE];

extern _Atomic bool playing_audio, cancel_audio;


static dac_continuous_config_t cont_cfg = {
   .chan_mask = DAC_CHANNEL_MASK_CH0,  // Pin 25 on ESP32, Pin 17 on ESP32S2
   .desc_num = 4,
   .buf_size = 128,  // Not used, but must be non-zero
   .freq_hz = 0,     // Sample rate
   .offset = 0,      // The offset of the DAC digital data. Range -128~127 
   .clk_src = DAC_DIGI_CLK_SRC_APLL,    // Using APLL as clock source to get a wider frequency range. The range is 648 Hz to several MHz on ESP32
   .chan_mode = DAC_CHANNEL_MODE_SIMUL, // Unused, we only use one channel
};
   


void setupSound(int enable_pin)
{
   ampliPIN = enable_pin;
   gpio_reset_pin(ampliPIN);
   gpio_set_direction(ampliPIN, GPIO_MODE_OUTPUT);
   gpio_set_level(ampliPIN, 0); 
}




void play_wav(const char *filename)
{
int fd;
uint32_t fileLen, channels, rate, bitsPerSample, frameSize, dataSize;  
char *p, preamble[44];
ssize_t n_read;
esp_err_t ret;  // Do not delete, ESP_GOTO_ON_ERROR needs it

   WRITE_ATOMIC(playing_audio, true);
   fd = open(filename, O_RDONLY);
   if (fd == -1) 
      ESP_GOTO_ON_ERROR(ESP_ERR_INVALID_STATE, close_fd_return, TAG, "Failed to open sound file %s: %s", filename, strerror(errno)); 

   n_read = read(fd, preamble, sizeof(preamble));
   if (n_read < sizeof(preamble)) 
      ESP_GOTO_ON_ERROR(ESP_ERR_INVALID_STATE, close_fd_return, TAG, "Invalid read in sound file %s", filename); 

   p = preamble;
   if (strncmp(p, "RIFF", 4)) 
      ESP_GOTO_ON_ERROR(ESP_ERR_INVALID_STATE, close_fd_return, TAG, "Invalid format of sound file %s", filename);  
   p += 4;
   
   fileLen = 8 + NUMBER32(p);
   //printf("Length of file %s: %lu\n", filename, fileLen);  
   p += 4;
   
   if (strncmp(p, "WAVE", 4)) 
      ESP_GOTO_ON_ERROR(ESP_ERR_INVALID_STATE, close_fd_return, TAG, "Invalid format of sound file %s", filename); 
   p += 4;
   
   if (strncmp(p, "fmt ", 4)) 
      ESP_GOTO_ON_ERROR(ESP_ERR_INVALID_STATE, close_fd_return, TAG, "Invalid format of sound file %s", filename); 
   p += 8;
   
   if (*p != 1) 
      ESP_GOTO_ON_ERROR(ESP_ERR_INVALID_STATE, close_fd_return, TAG, "Invalid format of sound file %s, audio must be PCM", filename); 
   p += 2;
   
   channels = NUMBER16(p);
   if (channels != 1 && channels != 2) 
      ESP_GOTO_ON_ERROR(ESP_ERR_INVALID_STATE, close_fd_return, TAG, "Invalid format of sound file %s: only 1 or 2 channels allowed", filename); 
   p += 2;
   
   rate = NUMBER32(p);
   p += 10;
   
   bitsPerSample = NUMBER16(p); 
   if (bitsPerSample !=8 && bitsPerSample != 16) 
      ESP_GOTO_ON_ERROR(ESP_ERR_INVALID_STATE, close_fd_return, TAG, "Invalid format of sound file %s: only 8 or 16 bits per sample allowed", filename);

   frameSize = channels * bitsPerSample/8;
   //printf("Channels=%lu, sample rate=%lu, bits per sample=%lu, bytes per frame=%lu\n", channels, rate, bitsPerSample, frameSize);
   p += 2;
   
   if (strncmp(p, "data", 4)) 
      ESP_GOTO_ON_ERROR(ESP_ERR_INVALID_STATE, close_fd_return, TAG, "Invalid format of sound file %s", filename);
   p += 4;
   
   dataSize = NUMBER32(p);
   //numSamples = dataSize/frameSize;
   //printf("dataSize=%lu, numSamples=%lu\n", dataSize, numSamples);
   if (dataSize != fileLen-sizeof(preamble)) 
      ESP_GOTO_ON_ERROR(ESP_ERR_INVALID_STATE, close_fd_return, TAG, "Invalid format of sound file %s", filename);

   /* Comienza el volcado de los datos de audio, situados después del preamble */
   cont_cfg.freq_hz = rate;
   cont_cfg.offset = (bitsPerSample==16)?-128:0;
   /* Allocate continuous channels */
   ESP_ERROR_CHECK(dac_continuous_new_channels(&cont_cfg, &dac_handle));
   ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));
   
   gpio_set_level(ampliPIN, 1);     // Activate amplifier
   vTaskDelay(pdMS_TO_TICKS(100));  // Wait for ampli to stabilize
   
   /* Read all dataSize bytes, in chunks of size BUFSIZE */
   for (uint32_t rest = dataSize; rest > 0; rest -= n_read) {
      size_t count=0;
      
      n_read = read(fd, buf, rest>BUFSIZE?BUFSIZE:rest); 
      if (n_read < 0)
         ESP_GOTO_ON_ERROR(ESP_ERR_INVALID_STATE, clean_return, TAG, "Error playing file %s: %s", filename, strerror(errno));
      
      /* Convert the data in the buffer buf into DAC suitable: unsigned 8 bits */
      for (int n = 0; n < n_read; n += frameSize) {
         int16_t value16_l, value16_r;
         uint32_t value8_l, value8_r;
         int32_t value=0;  
         
         switch (bitsPerSample) {
            case 8:
               value8_l = buf[n];
               if (channels == 2) value8_r = buf[n+1];
               else value8_r = value8_l;
               value = (value8_l+value8_r)/2;
               break;
            case 16:
               value16_l = NUMBER16(buf+n);
               if (channels == 2) value16_r = NUMBER16(buf+n+2);
               else value16_r = value16_l;
               value = (((int32_t)value16_l+(int32_t)value16_r)/2*256)/65536;
               break;  
         }
         buf_dac[count++] = (value*120)/100;   // Amplify and store
      }
      if (atomic_load_explicit(&cancel_audio, memory_order_relaxed)) break;
      ESP_ERROR_CHECK_WITHOUT_ABORT(dac_continuous_write(dac_handle, buf_dac, count, NULL, -1));
   }
   
clean_return:
   dac_continuous_disable(dac_handle);
   dac_continuous_del_channels(dac_handle);
   gpio_set_level(ampliPIN, 0);  
close_fd_return:   
   WRITE_ATOMIC(playing_audio, false);
   WRITE_ATOMIC(cancel_audio, false);
   close(fd); 
}

