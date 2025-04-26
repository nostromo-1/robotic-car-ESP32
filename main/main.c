/************************************************************************+*

Main file of robotic car project.
This is a ESP32 port of the original Raspberry Pi project.

************************************************************************+*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_chip_info.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "esp_rom_sys.h"
#include "esp32/rom/ets_sys.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_psram.h"

#include "wiimote.h"
#include "oled96.h"
#include "ultrasonic.h"
#include "pcf8591.h"
#include "sound.h"
#include "imu.h"
#include "network.h"


/************ Define system values *********************/
#if CONFIG_FREERTOS_UNICORE
#define RUNNING_CORE 0  // CPU0
#else
#define RUNNING_CORE 1  // CPU1
#endif
#define PRO_CORE 0


#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#define READ_ATOMIC(var) atomic_load_explicit(&var, memory_order_acquire)
#define WRITE_ATOMIC(var,value) atomic_store_explicit(&var, value, memory_order_release)

#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b;       \
})

#define min(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b;       \
})


#define I2C0_SCL_IO       22          /* GPIO number used for I2C0 master clock */
#define I2C0_SDA_IO       21          /* GPIO number used for I2C0 master data  */

#define STACK_SIZE 2048


/***************** I2C bus addresses ****************/
#define PCF8591_I2C 0x48           /* Dirección i2c del PCF8591 (AD/DA converter) */
#define DISPLAY_I2C 0x3C           /* Dirección i2c del display SSD1306 */
#define LSM9DS1_GYR_ACEL_I2C 0x6B  /* Dirección i2c del módulo acelerómetro/giroscopio del IMU LSM9DS1 */
#define LSM9DS1_MAG_I2C 0x1E       /* Dirección i2c del módulo magnetómetro del IMU LSM9DS1 */


/************ Define pin numbers *********************/

/*
Strapping pin: GPIO0, GPIO2, GPIO5, GPIO12 (MTDI), and GPIO15 (MTDO) are strapping pins. For more infomation, please refer to ESP32 datasheet.
SPI0/1: GPIO6-11 and GPIO16-17 are usually connected to the SPI flash and PSRAM integrated on the module and therefore should not be used for other purposes.
JTAG: GPIO12-15 are usually used for inline debug.
GPI: GPIO34-39 can only be set as input mode and do not have software-enabled pullup or pulldown functions.
TXD & RXD are usually used for flashing and debugging.
ADC2: ADC2 pins (10 channels, attached to GPIOs 0, 2, 4, 12 - 15 and 25 - 27) cannot be used when Wi-Fi is used. 
So, if you are having trouble getting the value from an ADC2 GPIO while using Wi-Fi, 
you may consider using an ADC1 GPIO instead (8 channels, attached to GPIOs 32 - 39), which should solve your problem. 
Please do not use the interrupt of GPIO36 and GPIO39 when using ADC or Wi-Fi and Bluetooth with sleep mode enabled.
*/

// 35, 23, 19 free

#define MI_ENA_PIN 26
#define MI_IN1_PIN 32
#define MI_IN2_PIN 33
    
#define MD_ENA_PIN 4
#define MD_IN1_PIN 18
#define MD_IN2_PIN 5

#define SONAR_TRIGGER_PIN 14
#define SONAR_ECHO_PIN    14
//#define SONAR_ECHO_PIN    35

#define PITO_PIN    12  // This pin remains low when deep sleep; others are high, so buzzer would sound
#define WMSCAN_PIN  15
#define AUDR_PIN    25  // DAC channel 0 in ESP32
#define AMPLI_PIN   13
#define LSENSOR_PIN 36
#define RSENSOR_PIN 39
#define KARR_PIN    27
//#define MIC_PIN     34  // ADC channel 6 in ESP32

/***************** Define constants and parameters ****************/
#define DISTMIN 45           /* distancia en cm a la que entendemos que hay un obstáculo */
#define INITIAL_SPEED 50     /* Entre 0 y 100% */
#define SONARDELAY 30        /* Time in ms between sonar triggers */
#define KARRDELAY 150        /* Time in ms to wait between leds in KARR scan */
#define NUMPULSES (16*120)   /* Motor assumed is a DFRobot FIT0450 with encoder. 16 pulses per round, 1:120 gearbox */
#define WHEELD 68            /* Wheel diameter in mm */


/****************** Variables y tipos globales **************************/

typedef enum {ADELANTE, ATRAS} Sentido_t;
typedef enum {CW, CCW} Rotation_t;

typedef struct {
    const char *id;             /* left or right */
    const gpio_num_t en_pin, in1_pin, in2_pin, sensor_pin;  /* Pines ESP32 */
    ledc_channel_t PWMchannel;  /* PWM channel in ESP32 */
    Sentido_t sentido;          /* ADELANTE, ATRAS */
    uint8_t velocidad;          /* 0 a 100, velocidad objetivo (no real) impuesta al motor */
    int16_t PWMduty;            /* Valor de PWM para alcanzar la velocidad objetivo, 0-100; no need for atomic type */
    int rpm;                    /* RPM of motor, only valid if encoder is used */
    _Atomic uint32_t counter;   /* counter for encoder pulses */
    uint64_t speedsetTick;      /* system tick value when the variable velocidad is set */
    SemaphoreHandle_t mutex;    /* Mutex to avoid collision when several threads access motor */
} Motor_t;


typedef struct {
    bool wiimote;
    _Atomic WiimoteButton_t buttons;
    int scan_pin;
    TaskHandle_t taskHandle;
} MandoWii_t;


typedef struct {
    const gpio_num_t pin;
    uint32_t pitando;
    SemaphoreHandle_t mutex;
    TaskHandle_t xHandle;
} Bocina_t;



/** These are the shared memory variables used for thread intercommunication **/
_Atomic int32_t distance = INT32_MAX;
_Atomic int8_t velocidadCoche = INITIAL_SPEED;  // velocidad objetivo del coche. Entre 0 y 100; el sentido de la marcha viene dado por el botón pulsado (A/B)
_Atomic bool esquivando; // Car is avoiding obstacle
_Atomic bool stalled;    // Car is stalled: it does not change its distance to objects
_Atomic bool collision;  // Car has crashed, when moving forwards or backwards
_Atomic bool scanningWiimote;  // User pressed scan button and car is scanning for wiimotes
_Atomic bool playing_audio, cancel_audio;   // Variables compartidas con fichero sound.c


/* Generic global variables */
uint8_t soundVolume = 96;     // 0 - 100%
SemaphoreHandle_t semaphore;  // Used to synchronize the main loop with the sonar measurement thread
static const char *alarmFile = "/spiffs/police.wav";  // File to play when user presses "UP" in wiimote
static const char* TAG = __FILE__;
static TaskHandle_t xMainTask;
static QueueHandle_t wav_queue;
static int LEDs[] = {0b0001, 0b0011, 0b0111, 0b1111};
static const esp_app_desc_t* fw_description;
static i2c_master_bus_handle_t i2c_bus0_handle, i2c_bus1_handle;


// program options, specified in menuconfig
#ifndef CONFIG_REMOTE_ONLY
#define CONFIG_REMOTE_ONLY 0
#endif 
#ifndef CONFIG_USE_ENCODER
#define CONFIG_USE_ENCODER 0
#endif 
#ifndef CONFIG_CHECK_BATTERY
#define CONFIG_CHECK_BATTERY 0
#endif 
#ifndef CONFIG_SOFT_TURN
#define CONFIG_SOFT_TURN 0
#endif 
#ifndef CONFIG_CALIBRATE_IMU
#define CONFIG_CALIBRATE_IMU 0
#endif 

bool remoteOnly=(CONFIG_REMOTE_ONLY==1);
bool useEncoder=(CONFIG_USE_ENCODER==1);
bool softTurn=(CONFIG_SOFT_TURN==1);
bool calibrateIMU=(CONFIG_CALIBRATE_IMU==1);



/********** Define and initialize the objects which compose the robotic car *********/
MandoWii_t mando ={
    .scan_pin = WMSCAN_PIN,
    .wiimote = false
};

Bocina_t bocina = {
    .pitando = false,
    .pin = PITO_PIN,
    .mutex = NULL
};

ultrasonic_sensor_t sonarHCSR04 = {
    .trigger_pin = SONAR_TRIGGER_PIN,
    .echo_pin = SONAR_ECHO_PIN,
    //.distance = UINT32_MAX
};
    

Motor_t m_izdo = {
    .id = "left",
    .en_pin = MI_ENA_PIN,
    .in1_pin = MI_IN1_PIN,
    .in2_pin = MI_IN2_PIN,
    .sensor_pin = LSENSOR_PIN,
    .PWMchannel = LEDC_CHANNEL_0,
    .mutex = NULL
};

Motor_t m_dcho = {
    .id = "right",
    .en_pin = MD_ENA_PIN,
    .in1_pin = MD_IN1_PIN,
    .in2_pin = MD_IN2_PIN,
    .sensor_pin = RSENSOR_PIN,
    .PWMchannel = LEDC_CHANNEL_1,
    .mutex = NULL
};




/* Forward declarations of internal functions of this module */
void TaskBlink(void *pvParameters);
void TaskKarr(void *pvParameters);
void TaskWiimote(void *pvParameters);
void TaskUltrasonic(void *pvParameters);
void TaskCheckPower(void *pvParameters);
void TaskSpeedControl(void *pvParameters);
void TaskPita(void *pvParameters);
void TaskPlayWav(void *pvParameters);
void TaskIMURead(void *pvParameters);

static void wiiCallback(void);
static void ajustaCocheConMando(WiimoteButton_t buttons);
static void ajustaMotor(Motor_t *motor, int v, Sentido_t sentido);
static void ajustaSentido(Motor_t *motor, Sentido_t dir);
static void fastStopMotor(Motor_t *motor);
static void speedSensor(void *args);

static void audioplay(const char *file, int modo);
static int retreatBackwards(void);
static void avoidObstacle(void);
void pito(uint32_t decimas, int modo);  // Not static, used elsewhere


static void startTasks(void)
{
TaskHandle_t xHandle = NULL;
   
   xTaskCreatePinnedToCore(
     TaskKarr,
     "TaskKarr",    // A name just for humans
     STACK_SIZE,    // in bytes. This stack size can be checked & adjusted by reading the Stack Highwater
     (void*)KARRDELAY,   // Task parameter: period in ms to blink
     tskIDLE_PRIORITY,   // Priority, with (configMAX_PRIORITIES - 1) being the highest, and tskIDLE_PRIORITY (0) being the lowest
     &xHandle, 
     RUNNING_CORE);
   configASSERT(xHandle);
   
   xTaskCreatePinnedToCore(
     TaskWiimote,
     "TaskWiimote",   // A name just for humans
     STACK_SIZE,      // in bytes. This stack size can be checked & adjusted by reading the Stack Highwater
     (void*)50,       // Task parameter: period in ms to check wiimote
     tskIDLE_PRIORITY+2,  // Priority, with (configMAX_PRIORITIES - 1) being the highest, and tskIDLE_PRIORITY (0) being the lowest
     &xHandle, 
     RUNNING_CORE);
   configASSERT(xHandle);
   mando.taskHandle = xHandle;

   xTaskCreatePinnedToCore(
     TaskUltrasonic,
     "TaskUltrasonic",    // A name just for humans
     STACK_SIZE,          // in bytes. This stack size can be checked & adjusted by reading the Stack Highwater
     (void*)SONARDELAY,   // Task parameter: period in ms to check sonar
     tskIDLE_PRIORITY+3,  // Priority, with (configMAX_PRIORITIES - 1) being the highest, and tskIDLE_PRIORITY (0) being the lowest
     &xHandle, 
     RUNNING_CORE);
   configASSERT(xHandle);

   xTaskCreatePinnedToCore(
     TaskCheckPower,
     "TaskCheckPower",   // A name just for humans
     STACK_SIZE,     // in bytes. This stack size can be checked & adjusted by reading the Stack Highwater
     (void*)200,     // Task parameter: period in ms to check power supply
     tskIDLE_PRIORITY+1,  // Priority, with (configMAX_PRIORITIES - 1) being the highest, and tskIDLE_PRIORITY (0) being the lowest
     &xHandle, 
     RUNNING_CORE);
   configASSERT(xHandle);   

   
   xTaskCreatePinnedToCore(
     TaskPita,
     "TaskPita",          // A name just for humans
     STACK_SIZE,          // in bytes. This stack size can be checked & adjusted by reading the Stack Highwater
     (void*)NULL,         // Task parameter: NULL
     tskIDLE_PRIORITY+1,  // Priority, with (configMAX_PRIORITIES - 1) being the highest, and tskIDLE_PRIORITY (0) being the lowest
     &xHandle, 
     RUNNING_CORE);
   configASSERT(xHandle); 
   bocina.xHandle = xHandle;

   
   xTaskCreatePinnedToCore(
     TaskPlayWav,
     "TaskPlayWav",   // A name just for humans
     STACK_SIZE,      // in bytes. This stack size can be checked & adjusted by reading the Stack Highwater
     (void*)NULL,     // Task parameter
     tskIDLE_PRIORITY+2,  // Priority, with (configMAX_PRIORITIES - 1) being the highest, and tskIDLE_PRIORITY (0) being the lowest
     &xHandle, 
     RUNNING_CORE);
   configASSERT(xHandle);
         
         
   if (useEncoder) {
      xTaskCreatePinnedToCore(
         TaskSpeedControl,
         "TaskSpeedControl",  // A name just for humans
         STACK_SIZE,          // in bytes. This stack size can be checked & adjusted by reading the Stack Highwater
         (void*)100,          // Task parameter: period in ms to check motor speed
         tskIDLE_PRIORITY+2,  // Priority, with (configMAX_PRIORITIES - 1) being the highest, and tskIDLE_PRIORITY (0) being the lowest
         &xHandle, 
         RUNNING_CORE);
      configASSERT(xHandle);
   }
   
}


// setup i2c master buses
static void i2c_master_init()
{  
   i2c_master_bus_config_t i2c0_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = I2C0_SCL_IO,
    .sda_io_num = I2C0_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
   };

   ESP_ERROR_CHECK(i2c_new_master_bus(&i2c0_mst_config, &i2c_bus0_handle));
}



int setupMotor(Motor_t *motor)
{
    motor->mutex = xSemaphoreCreateMutex();
    if (motor->mutex == NULL) return 1;
    gpio_reset_pin(motor->in1_pin);
    gpio_reset_pin(motor->in2_pin);
    gpio_set_direction(motor->in1_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(motor->in2_pin, GPIO_MODE_OUTPUT);
    
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,  // duty cycle values can range from 0 to 255
        .freq_hz = 500,  // 500 Hz, low but not very audible
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    ledc_channel_config_t ledc_channel = {
        .duty = 0,
        .channel = motor->PWMchannel,
        .gpio_num = motor->en_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        
    if (useEncoder) {
        ESP_ERROR_CHECK(gpio_reset_pin(motor->sensor_pin));
        ESP_ERROR_CHECK(gpio_set_direction(motor->sensor_pin, GPIO_MODE_INPUT));
        ESP_ERROR_CHECK(gpio_set_intr_type(motor->sensor_pin, GPIO_INTR_ANYEDGE));
        ESP_ERROR_CHECK(gpio_isr_handler_add(motor->sensor_pin, speedSensor, motor));
    }
    
    return 0;
}



int setupSonarHCSR04(void)
{
   if (ultrasonic_init(&sonarHCSR04) == ESP_OK) return 0;
   return 1;
}



/* ISR llamado cuando el pin WMSCAN_PIN es LOW. Tiene un pull-up a VCC, 0=>pulsado */
static void IRAM_ATTR wmScan(void *args)
{
int pin = (int)args;
int level;
const int64_t bounce_time = 100000UL;
static int64_t first_call_time;

    /* xHigherPriorityTaskWoken must be initialised to pdFALSE.
    If calling vTaskNotifyGiveFromISR() unblocks the handling
    task, and the priority of the handling task is higher than
    the priority of the currently running task, then
    xHigherPriorityTaskWoken will be automatically set to pdTRUE. */
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

/* For some reason, the ISR gets called continously with level=1 when motor is running */
/* The ISR gets called continously with level=0 when WM_SCAN button is pressed */
   level = gpio_get_level(pin);
   //esp_rom_printf("Pin=%d, state=%d\n", pin, level);
   
   // Debounce logic: level 0 (button press event) only valid if maintained for a certain time
   if (level != 0) {
      first_call_time = 0;
      return;
   }
   if (first_call_time == 0) {
      first_call_time = esp_timer_get_time();
      return;
   }
   if (esp_timer_get_time() < first_call_time + bounce_time) return;

   if (level == 0 && !READ_ATOMIC(scanningWiimote)) {
      /* Unblock the handling task so the task can perform
      any processing necessitated by the interrupt.  xHandlingTask
      is the task's handle, which was obtained when the task was
      created.  vTaskNotifyGiveFromISR() also increments
      the receiving task's notification value. */

      if (mando.taskHandle) vTaskNotifyGiveFromISR(mando.taskHandle, &xHigherPriorityTaskWoken);
   
      /* Force a context switch if xHigherPriorityTaskWoken is now
      set to pdTRUE. The macro used to do this is dependent on
      the port and may be called portEND_SWITCHING_ISR. */
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   }
}


int setupWiimote(void)
{
static const uint8_t bluetooth_glyph[] = {0, 66, 36, 255, 153, 90, 36, 0}; 
const uint8_t wiimote_timeout = 20;  // Max time in seconds to wait for wiimote
   
   oledSetBitmap8x8(15*8, 0, NULL);  // 15: last position in line (0-15), clear BT icon
   oledBigMessage(0, "Scan... ");
   
   ESP32Wiimote_init();
   ESP32Wiimote_addFilter(ACTION_IGNORE, FILTER_NUNCHUK_STICK | FILTER_NUNCHUK_BUTTON | FILTER_NUNCHUK_ACCEL); 
   if (!ESP32Wiimote_waitConnection(wiimote_timeout)) {
      mando.wiimote = false;
      oledBigMessage(0, "Wiimote?");   
      return 1;
   }

   mando.wiimote = true; 
   oledBigMessage(0, NULL);
   oledSetBitmap8x8(15*8, 0, bluetooth_glyph);  // Put BT icon
   
   ESP32Wiimote_rumble(true);        // señala mediante zumbido el mando sincronizado
   for (int i=0;i<10;i++) ESP32Wiimote_task();   // When program starts, TaskWiimote is not yet active, so we have to call ESP32Wiimote_task here
   vTaskDelay(pdMS_TO_TICKS(500));  
   ESP32Wiimote_rumble(false);
   for (int i=0;i<10;i++) ESP32Wiimote_task();
   return 0;
}



int setup(void)
{
int rc;
uint32_t voltage, current, battery1;
   
   i2c_master_init();
   
   // Display
   if (oledInit(i2c_bus0_handle, DISPLAY_I2C)) return 1;
   oledSetInversion(true);   // Fill display, as life sign
   
   // Write app name and version on display
   oledWriteString(0, 0, fw_description->project_name, false);
   oledWriteString(0, 1, fw_description->version, false);
   vTaskDelay(pdMS_TO_TICKS(500));
   
   // Power supply checker
   if (setupPCF8591(i2c_bus0_handle, PCF8591_I2C)) return 1;
   readPowerSupply(&voltage, &battery1, &current);
   if (voltage < 2*3000) {
      ESP_LOGE(TAG, "Power supply is too low (%ld mV). Aborting start.", voltage);
      return 1;
   }
   
   // Re-scan button; button pressed gives a 0
   gpio_reset_pin(mando.scan_pin);  // Enables pull-up
   gpio_set_direction(mando.scan_pin, GPIO_MODE_INPUT);
   
   // Wifi
   bool do_wps = (gpio_get_level(mando.scan_pin) == 0);
   rc = init_wifi_network(do_wps);   // Start wifi; uses WPS if mando.scan_pin is pressed when starting wifi
   if (rc == 0) {   // wifi is up
      if (voltage > 2*3600) get_ota_firmware();  // Only if enough power supply: try to get new firmware version. If successful, it restars the CPU
      start_webserver();   // Start file server of spiffs filesystem
      esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // When both WiFi and BT are running, WiFi modem has to go down
   }
   oledClear();
   
   // Sound
   wav_queue = xQueueCreate(1, sizeof(char*));  // Queue used to communicate with wav playing task
   if (wav_queue == NULL) return 1;
   setupSound(AMPLI_PIN);  // Initialize and setup sound
   
   // Buzzer
   gpio_reset_pin(bocina.pin);
   gpio_set_pull_mode(bocina.pin, GPIO_PULLDOWN_ONLY);
   gpio_set_direction(bocina.pin, GPIO_MODE_OUTPUT);
   gpio_set_level(bocina.pin, 0);
   bocina.mutex = xSemaphoreCreateMutex();
   if (bocina.mutex == NULL) return 1;
   
   // Motors
   if (setupMotor(&m_izdo)) return 1;
   if (setupMotor(&m_dcho)) return 1;
   
   // Wiimote
   if (setupWiimote()) return 1;
      
   // IMU
   oledBigMessage(0, "CALIB?");
   vTaskDelay(pdMS_TO_TICKS(2000));
   bool do_calibrate = (gpio_get_level(mando.scan_pin) == 0);  // if button is pressed, the user wants to calibrate IMU
   oledBigMessage(0, NULL);
   while (gpio_get_level(mando.scan_pin) == 0) vTaskDelay(pdMS_TO_TICKS(100));   // Wait till user releases button
   
   rc = setupLSM9DS1(i2c_bus0_handle, LSM9DS1_GYR_ACEL_I2C, LSM9DS1_MAG_I2C, do_calibrate);  // Setup IMU sensor
   if (rc == -2) {  // No calibration data, and user did not press push button
      oledBigMessage(0, "PLEASE");
      oledBigMessage(1, "CALIB ME");
      pito(10, 1);    // Buzz for 10 tenths of a second, wait till done
      esp_system_abort("Must calibrate IMU first");
   }
   
   // Sonar
   if (setupSonarHCSR04()) return 1;
      
   // Call wmScan when button is pressed
   gpio_set_intr_type(mando.scan_pin, GPIO_INTR_LOW_LEVEL);
   gpio_isr_handler_add(mando.scan_pin, wmScan, (void*)mando.scan_pin);

   oledSetInversion(false); // set display to normal mode
   return 0;
}



void init_CPU(void)
{
esp_chip_info_t chip_info;
esp_err_t ret;

   /* Check chip info */
   esp_chip_info(&chip_info);
   if (!(chip_info.features & CHIP_FEATURE_BT)) {
      ESP_LOGE(TAG, "Bluetooth system not present. Shutting down");
      esp_system_abort(NULL);   // Shutdown chip
   }
      
   /* Mount filesystem */
   esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 3,   // Maximum files that could be open at the same time
        .format_if_mount_failed = false
   };
   
   ret = esp_vfs_spiffs_register(&conf);
   if (ret != ESP_OK) {
      if (ret == ESP_FAIL) {
          ESP_LOGE(TAG, "Failed to mount or format filesystem");
       } else if (ret == ESP_ERR_NOT_FOUND) {
          ESP_LOGE(TAG, "Failed to find SPIFFS partition");
       } else {
          ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
       }
      esp_system_abort(NULL);  
   }

   size_t total, used;
   ret = esp_spiffs_info(conf.partition_label, &total, &used);
   if (ret != ESP_OK) {
       ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
       esp_system_abort(NULL);
       return;
   } else {
       ESP_LOGI(TAG, "Size of partition '%s': total %d, used %d", conf.partition_label, total, used);
   }
   
   esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
   esp_sleep_config_gpio_isolate();
   esp_log_level_set("gpio", ESP_LOG_WARN);
   esp_log_level_set("wifi", ESP_LOG_WARN);  // Avoid those many info messages
   esp_log_level_set("wifi_init", ESP_LOG_WARN); 
   esp_log_level_set("esp_https_ota", ESP_LOG_WARN);
   esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
   esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
   
   ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LOWMED));  // For GPIO interrupts, add handler with gpio_isr_handler_add
   
   //Initialize NVS
   ret = nvs_flash_init();
   if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
     ESP_ERROR_CHECK(nvs_flash_erase());
     ret = nvs_flash_init();
   }
   ESP_ERROR_CHECK(ret);
}


void app_main(void)
{
   fw_description = esp_app_get_description();
   printf("%s, version %s\n", fw_description->project_name, fw_description->version);
   printf("Hello world from CPU %d\n", xPortGetCoreID());
   printf("RTOS version %s\n", tskKERNEL_VERSION_NUMBER);
   printf("Max prio value=%u\n", configMAX_PRIORITIES-1);
   printf("Port tick period=%lu ms\n", portTICK_PERIOD_MS);
   printf("Minimal stack size=%u\n", configMINIMAL_STACK_SIZE);
   printf("Max internal memory=%u\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
   printf("Max internal memory block=%u\n", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
#ifdef CONFIG_SPIRAM
   printf("PSRAM size=%d bytes\n", esp_psram_get_size());
   // char *ptr = heap_caps_malloc(1e6, MALLOC_CAP_SPIRAM);  // malloc from PSRAM
#endif
   printf("Max mapped PSRAM memory=%u\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
   printf("Max malloc default memory=%u\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
      
   init_CPU();
   
   /* Initial setup */
   if (setup()) {
       ESP_LOGE(TAG, "Error al inicializar. Coche no arranca!");
       oledBigMessage(1, "SHUTDOWN");
       esp_system_abort(NULL);  // Or esp_deep_sleep_start();   
   }
   
   xMainTask = xTaskGetCurrentTaskHandle();
   startTasks();
   vTaskDelay(pdMS_TO_TICKS(100)); // Wait for tasks to activate
   ESP32Wiimote_setPlayerLEDs(LEDs[velocidadCoche/26]);
 
   // Check if battery low
   uint32_t volts = getSupplyVoltage();  
   if (volts < 2*3300) {
      oledBigMessage(0, "Battery!");
      audioplay("/spiffs/batterylow.wav", 1);
      oledBigMessage(0, NULL);           
   }
   else {
      oledBigMessage(0, " Ready  ");
      audioplay("/spiffs/ready.wav", 1);
      oledBigMessage(0, NULL);      
   }

   /*****
      Main loop. It sleeps until a problem is found. During sleep, the car is controlled by the wiimote.
      After awakening, it solves the problem, and goes back to sleep.
   ******/
   
   for (;;) {
      WiimoteButton_t buttons;  
      
      WRITE_ATOMIC(esquivando, false);  // Signal to sonarEcho that the semaphore can be activated: car is not 'esquivando' 
      /* Sleep until semaphore awakens us; it will happen in 3 cases:
      either the distance to an obstacle is below the threshold, or the car is stalled, or there was a collision */
      ulTaskNotifyTake(
         pdTRUE, // Clear the notification value before exiting: act as a binary semaphore
         portMAX_DELAY // Block indefinitely if task is not notified
      );
      
      buttons = READ_ATOMIC(mando.buttons);
      if (mando.wiimote && ~buttons&BUTTON_A) {
         ajustaCocheConMando(buttons);
         continue;
      }
      // Take action only if A is pressed
      if (READ_ATOMIC(collision) || READ_ATOMIC(stalled)) {
         oledBigMessage(0, "STALLED"); 
         retreatBackwards(); 
      }
      else {
         oledBigMessage(0, "OBSTACLE"); 
         avoidObstacle();  // Distance is below threshold  
      } 
      
      /* Obstacle is avoided, go back to normality */
      oledBigMessage(0, NULL);
      /* Adjust car to move */
      if (mando.wiimote) {
         ajustaCocheConMando(READ_ATOMIC(mando.buttons));  // wiimote controlled car
      }
      else {  // autonomous car
         ajustaMotor(&m_izdo, velocidadCoche, ADELANTE);
         ajustaMotor(&m_dcho, velocidadCoche, ADELANTE);             
      }
      
   }

}

   
void TaskUltrasonic(void *pvParameters)
{
TickType_t xDelay = (TickType_t)pvParameters;
int64_t tick, reference_tick;
uint32_t stalledTime;
const uint32_t maxStalledTime = 1200*1e3;  // Time in microseconds to flag car as stopped (it does not change its distance)

int32_t distance_mm, previous_distance, reference_distance;  // Must be signed values. All in mm, to avoid rounding errors in filter integer operations
static const char displayText[] = "Dist (cm):";
bool firstTime = true;
const uint32_t alpha = 20;  // 0-100; confidence in new value of distance

    oledWriteString(0, 0, displayText, false);  // Write fixed text to display only once
    TickType_t xWakeTime = xTaskGetTickCount();
    for (;;) {
        uint32_t measured_distance;  // In cm
        
        xTaskDelayUntil(&xWakeTime, pdMS_TO_TICKS(xDelay));
        tick = esp_timer_get_time();
        if (ultrasonic_measure_cm(&sonarHCSR04, &measured_distance) != ESP_OK) continue;
        
        if (firstTime) {
            firstTime = false;
            reference_tick = tick;  // Reference for stalled time calculation
            reference_distance = 10*measured_distance;
            previous_distance = reference_distance;
            continue;
        }
        
        distance_mm = (alpha*10*measured_distance + (100-alpha)*previous_distance)/100; // Apply a lowpass filter (IIR complementary filter)
        /* Set global variable "distance", this is the only producer */
        WRITE_ATOMIC(distance, distance_mm/10);  
        
        /* Update display if distance changed since last reading */
        if (distance != previous_distance/10) {
            char str[9];
            snprintf(str, sizeof(str), "%-3lu", distance);
            oledWriteString(8*sizeof(displayText), 0, str, false);  // update only distance number, to shorten the time
        }
        previous_distance = distance_mm;
                
        /* If car should be moving, look at change in distance to object since reference was taken; 
           if distance change is small, compute time passed as stalled, otherwise, reset values */
        if ((m_izdo.velocidad || m_dcho.velocidad) && abs(reference_distance - 10*distance)<=20) {  // if travelled distance < 20 mm, car is stalled
           stalledTime = tick - reference_tick;
        }
        else {
            stalledTime = 0;
            reference_tick = tick;
            reference_distance = distance_mm;
        }
        
        /* If the stalled time is above threshold, set global variable "stalled" as true, otherwise as false */
        WRITE_ATOMIC(stalled, stalledTime >= maxStalledTime); // this is the only producer
        
        /* 
           Activate semaphore to indicate main loop that it must awake;
           check specific situations first, and then activate semaphore if one of 3 conditions is met:
           either the distance to obstacle is below threshold 
           or the car is stalled (below or over threshold)
           or the car has crashed into something
        */
        if (!remoteOnly && !READ_ATOMIC(scanningWiimote) && !READ_ATOMIC(esquivando)) {
            if (distance < DISTMIN || stalled || READ_ATOMIC(collision)) {
               WRITE_ATOMIC(esquivando, true);  // Set global variable
               xTaskNotifyGive(xMainTask);      // Awake main loop
            }
        }

    }  // for (;;)
}

 


/*
  KARR effect
  Thread in charge of sending a clock pulse to the circuit implementing the KARR scan effect.
  At each LH transition, the led will change 
*/
void TaskKarr(void *pvParameters)  
{
TickType_t xDelay = (TickType_t)pvParameters;
int count = 0, state = 0;
  
   gpio_reset_pin(KARR_PIN);  
   gpio_set_direction(KARR_PIN, GPIO_MODE_OUTPUT);  
   gpio_reset_pin(LED_BUILTIN);
   gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);
   
   for (;;) { // A Task shall never return or exit
      if (count++ == 3) {
         state = !state;
         gpio_set_level(LED_BUILTIN, state);   // Blinks the built-in led as a life sign
         count = 0;
      }
      // Now, send a pulse to the 74HC4017
      gpio_set_level(KARR_PIN, 1);  // Small pulse, 20 ns length is enough
      gpio_set_level(KARR_PIN, 0);  // This achieves ca. 240 ns pulse length
      vTaskDelay(pdMS_TO_TICKS(xDelay));
  }
}



void TaskWiimote(void *pvParameters)  
{
TickType_t xDelay = (TickType_t)pvParameters;
uint32_t ulNotifiedValue;
int rc;
   
   for (;;) { // A Task shall never return or exit     
      ESP32Wiimote_task();
      if (ESP32Wiimote_available() > 0) wiiCallback();     
      
      // Wait for xDelay ms to go back to loop. If during this time the scan wiimote button is pressed, execute code below
      ulNotifiedValue = ulTaskNotifyTake(
         pdTRUE, // Clear the notification value before exiting: act as a binary semaphore
         pdMS_TO_TICKS(xDelay) // Block until task is notified or timeout occurs
      );
      if (ulNotifiedValue == 0) {
         // No notification: scan button was not pressed
         continue;
      }

      // Scan button was pressed
      WRITE_ATOMIC(scanningWiimote, true); // signal that scanning is in place
      ESP_LOGI(TAG, "Scan Wiimote button pressed");
      fastStopMotor(&m_izdo); fastStopMotor(&m_dcho);   // Para el coche mientras escanea wiimotes 
      WRITE_ATOMIC(velocidadCoche, 0);
      oledWriteString(12*8, 1, "    ", false); // Borra mensaje de "Auto", si está  
      
      ESP32Wiimote_reset();  // Disconnect wiimote
      for (int i=0;i<10;i++) ESP32Wiimote_task();  // Ensure ESP32Wiimote_task is called, to process bluetooth events
      vTaskDelay(pdMS_TO_TICKS(1000));  // para dar tiempo a desconectar el mando si estaba conectado
      
      // Try to connect to wiimote
      rc = setupWiimote();
      if (rc) {  // No wiimote found
         oledBigMessage(1, "SHUTDOWN");
         esp_deep_sleep_start();   // Shutdown chip
      }  
      WRITE_ATOMIC(velocidadCoche, INITIAL_SPEED); // Nueva velocidad inicial, con o sin mando 
      WRITE_ATOMIC(scanningWiimote, false);  // signal that scanning is over
   }
}


/*
  CheckPower: periodically checks main voltage of power supply and current consumption
*/
void TaskCheckPower(void *pvParameters)  
{
TickType_t xDelay = (TickType_t)pvParameters;
static const uint8_t empty_battery[] = {0, 254, 130, 131, 131, 130, 254, 0};  // glyph for empty battery
uint8_t battery_glyph[sizeof(empty_battery)];
uint32_t voltage, current, battery1, battery2;
char str[OLED_MAX_LINE_SIZE+1];
char str_old[sizeof(str)] = {0};
int8_t step, old_step = -1;
const uint32_t maxUndervoltageTime = 4000;  // Milliseconds with undervoltage before shutdown is triggered
uint32_t underVoltageTime = 0, n = 0;
int64_t previousTick, tick;
   
   previousTick = esp_timer_get_time();
   for (;;) { // A Task shall never return or exit  
      readPowerSupply(&voltage, &battery1, &current);  // Read values in mV and mA
      battery2 = voltage - battery1;  // Attention: with the current HW, v-bat1 is not bat2 exactly, the error is too big (ca. 150 mV)

      if (voltage < 2*3330) step = 0;        // Battery at 0%
      else if (voltage < 2*3510) step = 64;  // Battery at 20%
      else if (voltage < 2*3620) step = 64+32;      // Battery at 40%  
      else if (voltage < 2*3770) step = 64+32+16;   // Battery at 60%
      else if (voltage < 2*4000) step = 64+32+16+8; // Battery at 80%
      else step = 64+32+16+8+4;  // Battery at 100%

      // If battery state changed, update battery symbol on display
      if (step != old_step) {
         memcpy(battery_glyph, empty_battery, sizeof(empty_battery));
         battery_glyph[2] += step;
         battery_glyph[3] += step;
         battery_glyph[4] = battery_glyph[3];
         battery_glyph[5] = battery_glyph[2]; 
         oledSetBitmap8x8(14*8, 0, battery_glyph);  
         old_step = step;
      }
   
      // Symbol blinks when battery low
      if (step == 0) oledSetBitmap8x8(14*8, 0, (n++ & 1)?battery_glyph:NULL);
    
      // Update display only if values changed (it is a slow operation)
      voltage += 50;  // 1 digit after decimal, round to nearest integer
      uint8_t volts = voltage/1000;
      uint8_t decivolts = (voltage-volts*1000)/100;  
      current += 5;   // 2 digits after decimal, round to nearest integer
      uint8_t amps = current/1000;
      uint8_t centiamps = (current-amps*1000)/10;     
      snprintf(str, sizeof(str), "%u.%uV %u.%02uA", volts, decivolts, amps, centiamps);
      if (strcmp(str, str_old)) {
         oledWriteString(0, 1, str, false);
         strcpy(str_old, str);
      }

      // Shutdown if voltage is too low for a long period
      tick = esp_timer_get_time();
      if (battery1 < 2900 || battery2 < 2900) underVoltageTime += (tick-previousTick)/1000;
      else underVoltageTime = 0;
      if (underVoltageTime >= maxUndervoltageTime) {  
         oledBigMessage(0, "Battery!");   
         oledBigMessage(1, "SHUTDOWN");
         //vTaskSuspendAll();
         gpio_set_level(bocina.pin, 0);
         fastStopMotor(&m_izdo); fastStopMotor(&m_dcho);
         esp_system_abort("Out of battery");   // Shutdown chip
      }
      
      previousTick = tick;  
      vTaskDelay(pdMS_TO_TICKS(xDelay));      
  }
}


/****************** Funciones de la bocina **************************/
static void activaPito(void)
{
    xSemaphoreTake(bocina.mutex, portMAX_DELAY);
    if (bocina.pitando == 0) gpio_set_level(bocina.pin, 1);
    bocina.pitando++;
    xSemaphoreGive(bocina.mutex);
}


static void desactivaPito(void)
{
    xSemaphoreTake(bocina.mutex, portMAX_DELAY);
    if (bocina.pitando > 0) bocina.pitando--;
    if (bocina.pitando == 0) gpio_set_level(bocina.pin, 0);
    xSemaphoreGive(bocina.mutex);
}


/*  Función interna auxiliar */
void TaskPita(void *pvParameters)  
{
uint32_t decimas;

   for (;;) { // A Task shall never return or exit     
      xTaskNotifyWait(
         0x00,          // Do not clear any notification bits on entry
         0xffffffff,    // Reset the notification value to 0 on exit (clear all bits)
         &decimas,      // Notified value
         portMAX_DELAY  // Block indefinitely
      );
      activaPito();
      vTaskDelay(pdMS_TO_TICKS(decimas*100)); 
      desactivaPito();
   }
}


/* Toca el pito durante un tiempo (en decimas de segundo) 
modo=0; vuelve inmediatamente y pita en paralelo
modo=1; vuelve después de haber pitado */
void pito(uint32_t decimas, int modo)
{      
    if (decimas == 0) return;
    if (modo == 0) {
      xTaskNotify(bocina.xHandle, decimas, eSetValueWithoutOverwrite);
    } else {
      activaPito();
      vTaskDelay(pdMS_TO_TICKS(decimas*100)); 
      desactivaPito(); 
    }
}





/****************** Funciones de control de los motores **************************/
void ajustaSentido(Motor_t *motor, Sentido_t dir)
{
    switch(dir) {
      case ADELANTE:
        gpio_set_level(motor->in1_pin, 0);
        gpio_set_level(motor->in2_pin, 1);
        break;
      case ATRAS:
        gpio_set_level(motor->in1_pin, 1);
        gpio_set_level(motor->in2_pin, 0);
        break;
    }
    motor->sentido = dir;
}


void fastStopMotor(Motor_t *motor)
{
    xSemaphoreTake(motor->mutex, portMAX_DELAY);
    gpio_set_level(motor->in1_pin, 0);
    gpio_set_level(motor->in2_pin, 0);
    motor->PWMduty = motor->velocidad = 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->PWMchannel, motor->PWMduty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->PWMchannel);
    motor->speedsetTick = esp_timer_get_time();
    xSemaphoreGive(motor->mutex); 
}


/* v va de 0 a 100 */
void ajustaMotor(Motor_t *motor, int v, Sentido_t sentido)
{    
//printf("Set %s motor %d\n", motor->id, (sentido==ADELANTE)?v:-v);
    if (v > 100) v = 100;
    if (v < 0) {
        fprintf(stderr, "Error en ajustaMotor: v<0!\n");
        v = 0;
    }
    if (motor->velocidad == v && motor->sentido == sentido) return;
    
    xSemaphoreTake(motor->mutex, portMAX_DELAY);
    ajustaSentido(motor, sentido);
    motor->velocidad = v;
    motor->PWMduty = v*255/100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->PWMchannel, motor->PWMduty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->PWMchannel);
    motor->speedsetTick = esp_timer_get_time();
    xSemaphoreGive(motor->mutex);  
}


/* Adjust speed accordingly to the pressed wiimote buttons, as passed in the parameter */
void ajustaCocheConMando(WiimoteButton_t buttons)
{
int v_izdo, v_dcho;
Sentido_t s_izdo, s_dcho;

   v_izdo = v_dcho = 0;
   s_izdo = s_dcho = ADELANTE;
   
   /*** Botones A y B, leen la variable global "velocidadCoche" ***/
   if (ESP32Wiimote_connected() && buttons&(BUTTON_A | BUTTON_B)) { // if A or B or both pressed
      v_izdo = v_dcho = velocidadCoche;
      if (buttons&BUTTON_A) s_izdo = s_dcho = ADELANTE;
      else s_izdo = s_dcho = ATRAS;  // si vamos marcha atrás (botón B), invierte sentido
    
      /*** Botones LEFT y RIGHT, giran el coche ***/
      if (buttons&BUTTON_RIGHT) {
         s_dcho = 1 - s_dcho;  // Invert direction of movement
         if (softTurn) v_dcho = 0;
         else v_dcho = 50;
         v_izdo += 10; 
      } 
            
      if (buttons&BUTTON_LEFT) {
         s_izdo = 1 - s_izdo;  // Invert direction of movement
         if (softTurn) v_izdo = 0; 
         else v_izdo = 50;
         v_dcho += 10;                
      }
   }
   
   /*** Ahora activa la velocidadCoche calculada en cada motor ***/
   ajustaMotor(&m_izdo, v_izdo, s_izdo);
   ajustaMotor(&m_dcho, v_dcho, s_dcho);           
}


/***************Funciones de control de la velocidad ********************/
/* ISR llamado cuando el pin LSENSOR_PIN o RSENSOR_PIN cambia de estado
Se usa para medir la velocidad de rotación de las ruedas */
static void IRAM_ATTR speedSensor(void *args)
{
Motor_t *motor = (Motor_t*)args;

    // Increment counter by one in an atomic way (other threads must see correct value)
    atomic_fetch_add_explicit(&motor->counter, 1, memory_order_relaxed);
}




/* Callback llamado regularmente. Realiza el lazo de control de la velocidad, comparando
la diferencia de velocidades entre los motores para igualarlas */
void TaskSpeedControl(void *pvParameters)
{
TickType_t xDelay = (TickType_t)pvParameters;   
static int64_t past_tick; 
int64_t current_tick;
int32_t period;
   
static uint32_t past_lcounter;  
uint32_t current_lcounter;  
int lpulses, lfreq, lpwm;

static uint32_t past_rcounter;    
uint32_t current_rcounter;  
int rpulses, rfreq, rpwm;
  
int pv, kp=1;  // parameters of PID filter: pv is measured error (process value), kp is proportionality constant
static const int minMotorSetupTime = 1E6;  // Allow for 1 second (1E6 microseconds) for motors to stabilise before PID control loop works 
    
   for (;;) {
       vTaskDelay(pdMS_TO_TICKS(xDelay));   
       current_tick = esp_timer_get_time();
       current_lcounter =  m_izdo.counter;
       current_rcounter =  m_dcho.counter;
       if (past_tick == 0) {     // First time speedControl gets called
          past_tick = current_tick;
          past_lcounter = current_lcounter;
          past_rcounter = current_rcounter;
          continue;
       }
       period = current_tick - past_tick;
       past_tick = current_tick;
           
       /***** Left motor *****/
       lpulses = current_lcounter - past_lcounter;
       lfreq = (1E6*lpulses)/period;  
       m_izdo.rpm = lfreq*60/NUMPULSES;
       past_lcounter = current_lcounter;
       //if (lpulses) printf("Left motor: pulses=%d, freq=%d, rpm=%d\n", lpulses, lfreq, m_izdo.rpm);
       
       /***** Right motor *****/
       rpulses = current_rcounter - past_rcounter;
       rfreq = (1E6*rpulses)/period;  
       m_dcho.rpm = rfreq*60/NUMPULSES;
       past_rcounter = current_rcounter;
       //if (rpulses) printf("Right motor: pulses=%d, freq=%d, rpm=%d\n", rpulses, rfreq, m_dcho.rpm);

       /******* P control loop. SP=0, PV=lpulses-rpulses *********/
       if (m_izdo.velocidad != m_dcho.velocidad) continue;  // Enter control section only if straight line desired: both speeds equal
       if (m_izdo.velocidad == 0) continue;  // If speed is 0 (in both), do not enter control section
       /*** If time elapsed since speed was set in motor is below a threshold, 
            so that it had no time to stabilise, do not enter control section ***/ 
       if (current_tick - m_izdo.speedsetTick < minMotorSetupTime) continue;
       if (current_tick - m_dcho.speedsetTick < minMotorSetupTime) continue;
       
       pv = lpulses - rpulses;
       if (abs(pv) < 10) continue;  // Tolerable error, do not enter control section
       
       /** Control section loop; adjust parameters **/
       //printf("pv=%d     Adjust left: %i, right: %i\n", pv, -(kp*pv)/10, (kp*pv)/10);
       //printf("   m_izdo.PWMduty=%d, m_dcho.PWMduty=%d\n", m_izdo.PWMduty, m_dcho.PWMduty);
       xSemaphoreTake(m_izdo.mutex, portMAX_DELAY);
       xSemaphoreTake(m_dcho.mutex, portMAX_DELAY);

       m_izdo.PWMduty -= (kp*pv)/10;  // PWMduty can go outside the interval [0,255], let it go
       m_dcho.PWMduty += (kp*pv)/10;
       lpwm = m_izdo.PWMduty; rpwm = m_dcho.PWMduty;  // But lpwm and rpwm cannot go out of [0,255]
       if (lpwm>255) lpwm = 255; 
       if (lpwm<0) lpwm = 0;
       if (rpwm>255) rpwm = 255; 
       if (rpwm<0) rpwm = 0;
       ledc_set_duty(LEDC_LOW_SPEED_MODE, m_izdo.PWMchannel, lpwm);
       ledc_update_duty(LEDC_LOW_SPEED_MODE, m_izdo.PWMchannel);
       ledc_set_duty(LEDC_LOW_SPEED_MODE, m_dcho.PWMchannel, rpwm);
       ledc_update_duty(LEDC_LOW_SPEED_MODE, m_dcho.PWMchannel);
       
       xSemaphoreGive(m_izdo.mutex); 
       xSemaphoreGive(m_dcho.mutex); 
   }
}




static void wiiCallback(void)
{
static WiimoteButton_t previous_buttons;   
                       
   WiimoteButton_t buttons  = ESP32Wiimote_getButtonState();
   WRITE_ATOMIC(mando.buttons, buttons);   

   /*** Botones + y - ***/           
   if (previous_buttons&BUTTON_PLUS && ~mando.buttons&BUTTON_PLUS) {
      /* ajusta la velocidad del coche y la marca en leds del mando */
      atomic_fetch_add_explicit(&velocidadCoche, 10, memory_order_relaxed);
      if (velocidadCoche > 100) WRITE_ATOMIC(velocidadCoche, 100);
      ESP32Wiimote_setPlayerLEDs(LEDs[velocidadCoche/26]);
   }
            
   if (previous_buttons&BUTTON_MINUS && ~mando.buttons&BUTTON_MINUS) {
      /* ajusta la velocidad del coche y la marca en leds del mando */
      atomic_fetch_sub_explicit(&velocidadCoche, 10, memory_order_relaxed);
      if (velocidadCoche < 0) WRITE_ATOMIC(velocidadCoche, 0);
      ESP32Wiimote_setPlayerLEDs(LEDs[velocidadCoche/26]);
   }         
            
   /*** Botones A, B y RIGHT, LEFT; si estamos esquivando, no: el loop de main tiene el control ***/
   if (!READ_ATOMIC(esquivando)) ajustaCocheConMando(buttons);
   
   /*** pito ***/
   if (~previous_buttons&BUTTON_DOWN && mando.buttons&BUTTON_DOWN) activaPito();    
   if (previous_buttons&BUTTON_DOWN && ~mando.buttons&BUTTON_DOWN) desactivaPito();  
   
   /*** sonido ***/
   if (~previous_buttons&BUTTON_UP && mando.buttons&BUTTON_UP) audioplay(alarmFile, 0);
            
   previous_buttons = buttons;
}


/****************** Funciones auxiliares varias **************************/

/*
  Play a wav file. 
  If a file is already playing, it will cancel the reproduction and return.
  If modo is 0, it will play in a separate task and return immediately.
  If modo is 1, it will play in the calling task and return when done.
*/
void audioplay(const char *file, int modo)
{
   /* Si ya estamos reproduciendo algo, manda señal de cancelación al thread de audio */
   if (READ_ATOMIC(playing_audio)) {
       atomic_store_explicit(&cancel_audio, true, memory_order_relaxed);  // Signal cancel to sound thread
       return;
   }
   switch (modo) {
      case 0:
         xQueueSend(wav_queue, &file, 0);  // Block time of 0 says don't block if the queue is already full
         break;
      
      case 1:
         play_wav(file);
         break;
   }
}


/*
  Task that waits until instructed to play a given wav file
*/
void TaskPlayWav(void *pvParameters)  
{
const char *file;

   for (;;) { // A Task shall never return or exit
      // Block till a file to play is signalled via the queue
      if (xQueuePeek(wav_queue, &file, portMAX_DELAY) == pdPASS) {  // Peek, not Receive, so that queue is full while playing
         play_wav(file);
         xQueueReset(wav_queue);  // Empty the queue, so we start afresh
      }
  }
}


/**
Wait for the specified duration in milliseconds.
Min. duration is portTICK_PERIOD_MS
If there was a collision while waiting, it returns -1; otherwise, it returns 0
**/
static int interruptibleWait(int duration)
{
int32_t rest, lapse;
int64_t startTick;
const int32_t chunk = 10;  // 10 ms chunks
   
   configASSERT(chunk >= portTICK_PERIOD_MS);
   if (duration < portTICK_PERIOD_MS) duration = portTICK_PERIOD_MS;
   startTick = esp_timer_get_time();

   while (rest = duration - (esp_timer_get_time() - startTick)/1000, rest>0) {
      lapse = (rest>chunk)?chunk:rest;
      vTaskDelay(pdMS_TO_TICKS(lapse)); 
      if (READ_ATOMIC(collision)) return -1;
   }
   return 0;
}




/**
Rota el coche a la derecha (dextrógiro, rotation==CW) o a la izquierda (levógiro, rotation==CCW). 
Rota durante 'duration' milliseconds
If there was a collision while rotating, it returns -1; otherwise, it returns 0
**/
static int rota(Rotation_t rotation, Sentido_t marcha, int duration)  
{
Motor_t *pivot=NULL, *non_pivot=NULL;

   switch (marcha) {
      case ADELANTE: if (rotation == CW) {
                        pivot = &m_dcho;
                        non_pivot = &m_izdo;
                     }
                     else {
                        pivot = &m_izdo;
                        non_pivot = &m_dcho;
                     }
                     break;
      case ATRAS: if (rotation == CW) {
                        pivot = &m_izdo;
                        non_pivot = &m_dcho;
                  }
                  else {
                     pivot = &m_dcho;
                     non_pivot = &m_izdo;
                  }
                  break;
   }

   ajustaMotor(pivot, softTurn?0:velocidadCoche, 1-marcha);
   ajustaMotor(non_pivot, velocidadCoche, marcha);
   return interruptibleWait(duration);
}



/**
The car has found an obstacle in front, move backwards and turn slightly 
If there was a collision while moving, it returns -1; otherwise, it returns 0
**/
static int retreatBackwards(void)
{
int rc;

   //printf("Car seems stalled or collisioned, move a bit backwards...\n");
   fastStopMotor(&m_izdo); fastStopMotor(&m_dcho);
   vTaskDelay(pdMS_TO_TICKS(200)); 
   ajustaMotor(&m_izdo, 50, ATRAS);
   ajustaMotor(&m_dcho, 50, ATRAS);
   rc = interruptibleWait(softTurn?400:800);  // Move a little backwards first
   if (rc == 0) rc = rota(CW, ATRAS, velocidadCoche>70?300:600);  // If all went well, rotate backwards

   fastStopMotor(&m_izdo); fastStopMotor(&m_dcho); 
   return rc;
}



/**
Loop called when an obstable was detected by the sonar. It tries to avoid it,
and only returns if it was avoided or the user stopped pressing A.
The variable "esquivando" will be true when called, main loop will wait for this
routine to finish. So it has the only control of the car.
**/
static void avoidObstacle(void)
{
int rc;
WiimoteButton_t buttons;
  
   //printf("Obstacle at %d cm, avoiding...\n", distance);
   /** Loop for obstacle avoidance **/
   // distance is atomic, and is set asynchronously in another thread
   while (READ_ATOMIC(distance) < DISTMIN) {  
      /** Check that the button to scan the wiimote was not pressed **/
      if (READ_ATOMIC(scanningWiimote)) break;
      buttons = READ_ATOMIC(mando.buttons);
      /** Check that the user keeps pressing A **/
      if (mando.wiimote && ~buttons&BUTTON_A) break;

      /**  Rotate the car to avoid obstacle; rotate for the time to get a new distance measure **/
      rc = rota(CW, ADELANTE, SONARDELAY);  
      if (rc < 0 || READ_ATOMIC(stalled)) retreatBackwards();  // stalled is a global variable, set by the sonar asynchronously
   }
}







