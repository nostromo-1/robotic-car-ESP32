/**************************************************************************
Main file of robotic car project.
This is a ESP32 port of the original Raspberry Pi project.

************************************************************************+*/

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "esp_rom_sys.h"
#include "esp32/rom/ets_sys.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
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


#define I2C0_SCL_IO       22          /* GPIO number used for I2C0 master clock */
#define I2C0_SDA_IO       21          /* GPIO number used for I2C0 master data  */
#define I2C0_NUM          I2C_NUM_0   /* I2C master i2c0 port number */
#define I2C0_FREQ         400000      /* I2C master clock frequency */

#define I2C1_SCL_IO       23          /* GPIO number used for I2C1 master clock */
#define I2C1_SDA_IO       19          /* GPIO number used for I2C1 master data  */
#define I2C1_NUM          I2C_NUM_1   /* I2C master i2c1 port number */
#define I2C1_FREQ         100000      /* I2C master clock frequency */

#define I2C_TIMEOUT       100          /* I2C bus timeout in ms */
#define ACK_CHECK_EN      0x1          /* I2C master will check ack from slave */
#define ACK_CHECK_DIS     0x0          /* I2C master will not check ack from slave */
#define ACK_VAL           0x0          /* I2C ack value */
#define NACK_VAL          0x1          /* I2C nack value */

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
ADC2: ADC2 pins cannot be used when Wi-Fi is used. So, if you are having trouble getting the value from an ADC2 GPIO while using Wi-Fi, 
you may consider using an ADC1 GPIO instead, which should solve your problem. 
Please do not use the interrupt of GPIO36 and GPIO39 when using ADC or Wi-Fi and Bluetooth with sleep mode enabled.
*/

#define MI_ENA_PIN 26
#define MI_IN1_PIN 32
#define MI_IN2_PIN 33
    
#define MD_ENA_PIN 4
#define MD_IN1_PIN 18
#define MD_IN2_PIN 5

#define SONAR_TRIGGER_PIN 14
#define SONAR_ECHO_PIN    35

#define PITO_PIN   12  // This pin remains low when deep sleep
#define WMSCAN_PIN 15
#define AUDR_PIN   25  // DAC channel 0 in ESP32
#define AMPLI_PIN  13
#define LSENSOR_PIN 36
#define RSENSOR_PIN 39
#define KARR_PIN    27


/***************** Define constants and parameters ****************/
#define DISTMIN 45           /* distancia en cm a la que entendemos que hay un obstáculo */
#define INITIAL_SPEED 50     /* Entre 0 y 100% */
#define SONARDELAY 50        /* Time in ms between sonar triggers */
#define NUMPOS 3             /* Número de medidas de posición del sonar para promediar */
#define NUMPULSES (16*120)   /* Motor assumed is a DFRobot FIT0450 with encoder. 16 pulses per round, 1:120 gearbox */
#define WHEELD 68            /* Wheel diameter in mm */
#define KARRDELAY 150        /* Time in ms to wait between leds in KARR scan */
#define MAX_DISTANCE_CM 300  /* max distance to be measured by sonar */



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
_Atomic uint32_t distance = UINT32_MAX;
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
bool checkBattery=(CONFIG_CHECK_BATTERY==1);
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
    .distance = UINT32_MAX
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
static void pito(uint32_t decimas, int modo);


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
   i2c_config_t conf0 = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C0_SDA_IO,         // select SDA GPIO
    .scl_io_num = I2C0_SCL_IO,         // select SCL GPIO
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C0_FREQ,     // select frequency
    .clk_flags = 0,                    // optional; you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
   };

   ESP_ERROR_CHECK(i2c_param_config(I2C0_NUM, &conf0));
   ESP_ERROR_CHECK(i2c_driver_install(I2C0_NUM, conf0.mode, 0, 0, 0));
   
   i2c_config_t conf1 = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C1_SDA_IO,         // select SDA GPIO
    .scl_io_num = I2C1_SCL_IO,         // select SCL GPIO
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C1_FREQ,     // select frequency 
    .clk_flags = 0,                    // optional; you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
   };

   ESP_ERROR_CHECK(i2c_param_config(I2C1_NUM, &conf1));
   ESP_ERROR_CHECK(i2c_driver_install(I2C1_NUM, conf1.mode, 0, 0, 0));  

   
   // Scan I2C buses
   for (int bus=0; bus<=1; bus++) {
      printf("i2c%i scan: \n", bus);
      for (uint8_t i = 1; i < 127; i++) {
         esp_err_t ret;
         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
         i2c_master_start(cmd);
         i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
         i2c_master_stop(cmd);
         ret = i2c_master_cmd_begin((bus==0)?I2C0_NUM:I2C1_NUM, cmd, I2C_TIMEOUT / portTICK_PERIOD_MS);
         i2c_cmd_link_delete(cmd);
    
         if (ret == ESP_OK) printf("Found device at: 0x%2x\n", i);
      }
   }
   
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
   /*
   gpio_set_direction(sonarHCSR04.trigger_pin, GPIO_MODE_OUTPUT);
   gpio_set_direction(sonarHCSR04.echo_pin, GPIO_MODE_INPUT);
   gpio_set_level(sonarHCSR04.trigger_pin, 0);
   */
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

      vTaskNotifyGiveFromISR(mando.taskHandle, &xHigherPriorityTaskWoken);
   
      /* Force a context switch if xHigherPriorityTaskWoken is now
      set to pdTRUE. The macro used to do this is dependent on
      the port and may be called portEND_SWITCHING_ISR. */
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   }
}


int setupWiimote(void)
{
static uint8_t bluetooth_glyph[] = {0, 66, 36, 255, 153, 90, 36, 0}; 
const uint8_t wiimote_timeout = 20;  // Max time in seconds to wait for wiimote
   
   oledSetBitmap8x8(15*8, 0, NULL);  // 15: last position in line (0-15), clear BT icon
   oledBigMessage(0, "Scan... ");
   //pito(5, 1);   // Pita 5 décimas para avisar que comienza búsqueda de mando
   
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
   i2c_master_init(); 
   if (oledInit(DISPLAY_I2C)) return 1;
   if (setupPCF8591(PCF8591_I2C)) return 1;
   oledSetInversion(true);   // Fill display, as life sign

   // Queue used to communicate with wav playing task
   wav_queue = xQueueCreate(1, sizeof(char*));
   if (wav_queue == NULL) return 1;

   // Inicializa altavoz
   setupSound(AMPLI_PIN);
   //setVolume(soundVolume);
   
   /* bocina */
   gpio_reset_pin(bocina.pin);
   gpio_set_pull_mode(bocina.pin, GPIO_PULLDOWN_ONLY);
   gpio_set_direction(bocina.pin, GPIO_MODE_OUTPUT);
   gpio_set_level(bocina.pin, 0);
   bocina.mutex = xSemaphoreCreateMutex();
   if (bocina.mutex == NULL) return 1;
   
   if (setupMotor(&m_izdo)) return 1;
   if (setupMotor(&m_dcho)) return 1;
   
   
   //setupBMP280(BMP280_I2C, TIMER4);  // Setup temperature/pressure sensor
   
   /* Re-scan button; button pressed gives a 0 */
   gpio_reset_pin(mando.scan_pin);  // Enables pull-up
   gpio_set_direction(mando.scan_pin, GPIO_MODE_INPUT);

   if (setupWiimote()) return 1;
   gpio_set_intr_type(mando.scan_pin, GPIO_INTR_LOW_LEVEL);
   gpio_isr_handler_add(mando.scan_pin, wmScan, (void*)mando.scan_pin);  // Call wmScan when button changes. Debe llamarse después de setupWiimote
   
   if (setupLSM9DS1(LSM9DS1_GYR_ACEL_I2C, LSM9DS1_MAG_I2C)) return 1;    // Setup IMU sensor
   oledSetInversion(false); // clear display
   
   if (setupSonarHCSR04()) return 1;  // last to call, as it starts measuring distance and triggering semaphore

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
   //printf("Number of cores=%u\n", chip_info.cores);  
      
   /* Mount filesystem */
   esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 3,
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
   
   esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
   esp_sleep_config_gpio_isolate();
   
   ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LOWMED));  // For GPIO interrupts, add handler with gpio_isr_handler_add
   
   //Initialize NVS
   //nvs_flash_erase();
   ret = nvs_flash_init();
   if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
     ESP_ERROR_CHECK(nvs_flash_erase());
     ret = nvs_flash_init();
   }
   ESP_ERROR_CHECK(ret);
}


void app_main(void)
{
   printf("Hello world from CPU %d\n", xPortGetCoreID());
   printf("RTOS version %s\n", tskKERNEL_VERSION_NUMBER);
   printf("Max prio value=%u\n", configMAX_PRIORITIES-1);
   printf("Port tick period=%lu ms\n", portTICK_PERIOD_MS);
   printf("Minimal stack size=%u\n", configMINIMAL_STACK_SIZE);
   printf("Max internal memory=%u\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
   printf("Max internal memory block=%u\n", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
   
   printf("PSRAM size=%d bytes\n", esp_psram_get_size());
   printf("Max mapped PSRAM memory=%u\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
   printf("Max malloc default memory=%u\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
   // char *ptr = heap_caps_malloc(1e6, MALLOC_CAP_SPIRAM);  // malloc from PSRAM
      
   init_CPU();
   init_wifi_network();
   
   /* Initial setup */
   xMainTask = xTaskGetCurrentTaskHandle();
   if (setup()) {
       ESP_LOGE(TAG, "Error al inicializar. Coche no arranca!");
       oledBigMessage(1, "SHUTDOWN");
       esp_system_abort(NULL);  // Or esp_deep_sleep_start();   
   }
     
   startTasks();
   vTaskDelay(pdMS_TO_TICKS(100)); // Wait for tasks to activate
   ESP32Wiimote_setPlayerLEDs(LEDs[velocidadCoche/26]);
 
   // Check if battery low; -1 means that the ADC does not work correctly
   float volts = getMainVoltageValue();  
   if (checkBattery && volts>=0 && volts<6.6) {
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
      //if (mando.wiimote && ~buttons&BUTTON_A) continue;  // Take action only if A is pressed
      if (mando.wiimote && ~buttons&BUTTON_A) {
         ajustaCocheConMando(buttons);
         continue;
      }
      
      if (READ_ATOMIC(collision) || READ_ATOMIC(stalled)) {
         //printf("Collision/stall!\n");
         oledBigMessage(0, "STALLED"); 
         retreatBackwards(); 
      }
      else {
         //printf("Esquivando...\n");
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
int64_t tick, referenceTick = 0;
uint32_t stalledTime;
const uint32_t maxStalledTime = 1200*1e3;  // Time in microseconds to flag car as stopped (it does not change its distance)
uint32_t distance_array[NUMPOS], pos_array = 0;

int32_t previous_distance = 0, reference_distance = 0;
static const char displayText[] = "Dist (cm):";
int32_t suma, distance_local;
bool is_stalled;
int firstTime = 0;

    TickType_t xWakeTime = xTaskGetTickCount();
    for (;;) {
        uint32_t dist;
        
        xTaskDelayUntil(&xWakeTime, pdMS_TO_TICKS(xDelay));
        tick = esp_timer_get_time();
        if (ultrasonic_measure_cm(&sonarHCSR04, &dist) != ESP_OK) continue;
        
        distance_array[pos_array++] = dist;  // sonar measured distance in cm
        if (pos_array == NUMPOS) pos_array = 0;
        
        if (firstTime>=0) {
            if (firstTime < NUMPOS) { /* The first NUMPOS times until array is filled */
               firstTime++;
               continue;
            } else {
               firstTime = -1;  /* Initialisation of distance_array is over */
               referenceTick = tick;  // Reference for stalled time calculation
               oledWriteString(0, 0, displayText, false);  // Write fixed text to display only once
            }
        }
 
        /* Calculate moving average */
        suma = 0;
        for (int i=0; i<NUMPOS; i++) suma += distance_array[i]; 
        sonarHCSR04.distance = suma/NUMPOS; 

        /* Set global variable "distance", this is the only producer */
        WRITE_ATOMIC(distance, sonarHCSR04.distance);
        distance_local = sonarHCSR04.distance; // local copy of variable
        if (referenceTick == tick) reference_distance = distance_local;  // Will only happen once, at the beginning
        
        /* Update display if distance changed since last reading */
        if (distance_local != previous_distance) {
            char str[6];
            snprintf(str, sizeof(str), "%-3lu", distance_local);
            oledWriteString(8*sizeof(displayText), 0, str, false);  // update only distance number
            previous_distance = distance_local;
        }
        
        /* If car should be moving, look at change in distance to object since reference was taken; 
           if distance change is small, compute time passed as stalled, otherwise, reset values */
        if ((m_izdo.velocidad || m_dcho.velocidad) && abs(reference_distance - distance_local)<=2) {
           stalledTime = tick - referenceTick;
        }
        else {
            stalledTime = 0;
            referenceTick = tick;
            reference_distance = distance_local;
        }
        
        /* If the stalled time is above threshold, set global variable "stalled" as true, otherwise as false */
        is_stalled = stalledTime >= maxStalledTime;
        WRITE_ATOMIC(stalled, is_stalled); 
        
        /* 
           Activate semaphore to indicate main loop that it must awake;
           check specific situations first, and then activate semaphore if one of 3 conditions is met:
           either the distance to obstacle is below threshold 
           or the car is stalled (below or over threshold)
           or the car has crashed into something
        */
        if (!remoteOnly && !READ_ATOMIC(scanningWiimote) && !READ_ATOMIC(esquivando)) {
            if (distance_local < DISTMIN || is_stalled || READ_ATOMIC(collision)) {
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
      gpio_set_level(KARR_PIN, 0);
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
   
   for (;;) { // A Task shall never return or exit
      checkPower();
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







