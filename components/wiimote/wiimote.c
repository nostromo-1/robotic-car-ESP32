
// Based on the following code:

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


#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "time.h"
#include "sys/time.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_timer.h"



#include "wiimote.h"


#define WIIMOTE_VERBOSE 0

#if WIIMOTE_VERBOSE
#define VERBOSE_PRINT(...) printf(__VA_ARGS__)
#define VERBOSE_PRINTLN(...) printf(__VA_ARGS__); printf("\n")
#else
#define VERBOSE_PRINT(...) do {} while(0)
#define VERBOSE_PRINTLN(...) do {} while(0)
#endif

#define RX_QUEUE_SIZE 32
#define TX_QUEUE_SIZE 32


#ifdef CONFIG_BTDM_CONTROLLER_MODE_BTDM
#define BT_MODE ESP_BT_MODE_BTDM
#elif defined(CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY)
#define BT_MODE ESP_BT_MODE_CLASSIC_BT
#else
#define BT_MODE ESP_BT_MODE_BLE
#endif



typedef struct {
        size_t len;
        uint8_t data[];
} queuedata_t;
  

static const char* TAG = __FILE__;
static const int NUNCHUK_STICK_THRESHOLD = 2;
static TinyWiimoteData_t _gotData;
static WiimoteButton_t _buttonState, _oldButtonState;
static WiimoteNunchukState_t *_pNunchukState, *_pOldNunchukState;
static WiimoteNunchukState_t _nunchukStateA, _nunchukStateB;
static int _nunStickThreshold;
static int _filter;
static esp_vhci_host_callback_t vhci_callback;
static QueueHandle_t txQueue, rxQueue;


static void createQueue(void);
static void handleTxQueue(void);
static void handleRxQueue(void);
static esp_err_t sendQueueData(QueueHandle_t queue, uint8_t *data, size_t len);
static void notifyHostSendAvailable(void);
static int notifyHostRecv(uint8_t *data, uint16_t len);
static void hciHostSendPacket(uint8_t *data, size_t len);
  
  

static bool btStarted() {
    return (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED);
}


static bool btStart() {
    esp_err_t ret;
    
    if (btStarted()) return true;
    ESP_LOGI(TAG, "BT Start");
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) { 
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }
    
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) return true;
    ESP_LOGE(TAG, "BT Start failed");
    return false;
}


static bool btStop() {
    ESP_LOGI(TAG, "BT Srop");
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) return true;
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        if (esp_bt_controller_disable()) {
            ESP_LOGE(TAG, "BT Disable failed");
            return false;
        }
        while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED);
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        if (esp_bt_controller_deinit()) {
			ESP_LOGE(TAG, "BT deinit failed");
			return false;
		}
		vTaskDelay(1);
		if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE) return false;		
      return true;
    }
    ESP_LOGE(TAG, "BT Stop failed");
    return false;
}




static void notifyHostSendAvailable(void) {
  VERBOSE_PRINT("notifyHostSendAvailable\n");
  if(!TinyWiimoteDeviceIsInited()){
    TinyWiimoteResetDevice();
  }
}

static void createQueue(void) {
  txQueue = xQueueCreate(TX_QUEUE_SIZE, sizeof(queuedata_t*));
  if (txQueue == NULL){
    VERBOSE_PRINTLN("xQueueCreate(txQueue) failed");
    return;
  }
  rxQueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(queuedata_t*));
  if (rxQueue == NULL){
    VERBOSE_PRINTLN("xQueueCreate(rxQueue) failed");
    return;
  }
}

static void handleTxQueue(void) {
  if(uxQueueMessagesWaiting(txQueue)){
    bool ok = esp_vhci_host_check_send_available();
    VERBOSE_PRINT("esp_vhci_host_check_send_available=%d", ok);
    if(ok){
      queuedata_t *queuedata = NULL;
      if(xQueueReceive(txQueue, &queuedata, 0) == pdTRUE){
        esp_vhci_host_send_packet(queuedata->data, queuedata->len);
        VERBOSE_PRINT("SEND => %s", format2Hex(queuedata->data, queuedata->len));
        free(queuedata);
      }
    }
  }
}

static void handleRxQueue(void) {
  if(uxQueueMessagesWaiting(rxQueue)){
    queuedata_t *queuedata = NULL;
    if(xQueueReceive(rxQueue, &queuedata, 0) == pdTRUE){
      handleHciData(queuedata->data, queuedata->len);
      free(queuedata);
    }
  }
}

static esp_err_t sendQueueData(QueueHandle_t queue, uint8_t *data, size_t len) {
    VERBOSE_PRINTLN("sendQueueData");
    if(!data || !len){
        VERBOSE_PRINTLN("no data");
        return ESP_OK;
    }
    queuedata_t * queuedata = (queuedata_t*)malloc(sizeof(queuedata_t) + len);
    if(!queuedata){
        VERBOSE_PRINTLN("malloc failed");
        return ESP_FAIL;
    }
    queuedata->len = len;
    memcpy(queuedata->data, data, len);
    if (xQueueSend(queue, &queuedata, portMAX_DELAY) != pdPASS) {
        VERBOSE_PRINTLN("xQueueSend failed");
        free(queuedata);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void hciHostSendPacket(uint8_t *data, size_t len) {
  sendQueueData(txQueue, data, len);
}


static int notifyHostRecv(uint8_t *data, uint16_t len) 
{
   /*
  printf("notifyHostRecv:");
  for (int i = 0; i < len; i++) {
    printf(" %02x", data[i]);
  }
  printf("\n");
*/
  
  if (sendQueueData(rxQueue, data, len) == ESP_OK) return ESP_OK;
  else return ESP_FAIL;
}



void ESP32Wiimote_init(void)
{
    esp_err_t ret;
   
    if (btStarted()) return;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);  
    
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    
    _pNunchukState = &_nunchukStateA;
    _pOldNunchukState = &_nunchukStateB;
    _nunStickThreshold = NUNCHUK_STICK_THRESHOLD * NUNCHUK_STICK_THRESHOLD;
    _filter = FILTER_NONE;
    
    TinyWiimoteInit(hciHostSendPacket);
    createQueue();
    vhci_callback.notify_host_recv = notifyHostRecv;
    vhci_callback.notify_host_send_available = notifyHostSendAvailable;    

    if (!btStart()) {
        ESP_LOGE(TAG, "btStart failed");
        return;
    }

    if ((ret = esp_vhci_host_register_callback(&vhci_callback)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_vhci_host_register_callback failed");
        return;
    }
}


void ESP32Wiimote_reset(void)
{
    if (rxQueue) xQueueReset(rxQueue);
    if (txQueue) xQueueReset(txQueue);
    if (TinyWiimoteDeviceIsInited()) TinyWiimoteResetDevice();
}


bool ESP32Wiimote_waitConnection(uint8_t secs_timeout)
{  
     if (!btStarted()) {
      ESP_LOGE(TAG, "ESP32Wiimote was not initialized");
      return false;
    }
    
    int64_t t0 = esp_timer_get_time();
    do {
       vTaskDelay(pdMS_TO_TICKS(50)); 
       ESP32Wiimote_task();
    } while (!TinyWiimote_connected() && (esp_timer_get_time() - t0) < secs_timeout*1000000UL);
    
    if (TinyWiimote_connected()) return true;  // wiimote found within given time
    else {   // stop searching for wiimote
       ESP32Wiimote_reset();
       for (int i=0;i<10;i++) ESP32Wiimote_task();
       return false;
    }
}



void ESP32Wiimote_task(void)
{
    if (btStarted()) {
      handleTxQueue();
      handleRxQueue();
    }
}


bool ESP32Wiimote_connected(void)
{
    return TinyWiimote_connected();
}


bool ESP32Wiimote_available(void)
{
    WiimoteNunchukState_t *pTmpNunchuck;
       
    if (TinyWiimoteAvailable() == 0) return false;
    _gotData = TinyWiimoteRead();

    // update button state
    _oldButtonState = _buttonState;
    _buttonState = 0;
    _buttonState = (_gotData.data[TWII_OFFSET_BTNS1] << 8) | _gotData.data[TWII_OFFSET_BTNS2];

    // update old nunchuck state(= exchange nunchuk state area)
    pTmpNunchuck =  _pOldNunchukState;
    _pOldNunchukState =  _pNunchukState;
    _pNunchukState =  pTmpNunchuck;

    // update nunchuk state
    _pNunchukState->xStick = _gotData.data[TWII_OFFSET_EXTCTRL + 0];
    _pNunchukState->yStick = _gotData.data[TWII_OFFSET_EXTCTRL + 1];
    _pNunchukState->xAxis = _gotData.data[TWII_OFFSET_EXTCTRL + 2];
    _pNunchukState->yAxis = _gotData.data[TWII_OFFSET_EXTCTRL + 3];
    _pNunchukState->zAxis = _gotData.data[TWII_OFFSET_EXTCTRL + 4];
    _pNunchukState->cBtn = ((_gotData.data[TWII_OFFSET_EXTCTRL + 5] & 0x02) >> 1) ^ 0x01;
    _pNunchukState->zBtn = (_gotData.data[TWII_OFFSET_EXTCTRL + 5] & 0x01) ^ 0x01;

    // check button change
    bool buttonIsChanged = false;
    if (!(_filter & FILTER_REMOTE_BUTTON) && (_buttonState != _oldButtonState)) buttonIsChanged = true;

    // check nunchuk stick change
    bool nunchukStickIsChanged = false;
    int nunXStickDelta = (int)(_pNunchukState->xStick) - _pOldNunchukState->xStick;
    int nunYStickDelta = (int)(_pNunchukState->yStick) - _pOldNunchukState->yStick;
    int nunStickDelta = (nunXStickDelta*nunXStickDelta + nunYStickDelta*nunYStickDelta) / 2;

    if (!(_filter & FILTER_NUNCHUK_STICK) && (nunStickDelta >= _nunStickThreshold)) nunchukStickIsChanged = true;

    // check nunchuk button change
    bool nunchukButtonIsChanged = false;
    if (!(_filter & FILTER_NUNCHUK_BUTTON) && 
      ((_pNunchukState->cBtn != _pOldNunchukState->cBtn)
      || (_pNunchukState->zBtn != _pOldNunchukState->zBtn)
      )) nunchukButtonIsChanged = true;

    // check accel change
    bool accelIsChanged = false;
    if (!(_filter & FILTER_NUNCHUK_ACCEL)) accelIsChanged = true;
   
    return (buttonIsChanged || nunchukStickIsChanged || nunchukButtonIsChanged || accelIsChanged);
}

WiimoteButton_t ESP32Wiimote_getButtonState(void)
{
  return _buttonState;
}

WiimoteNunchukState_t ESP32Wiimote_getNunchukState(void)
{
  return *_pNunchukState;
}

void ESP32Wiimote_addFilter(int action, int filter) {
  if (action == ACTION_IGNORE) {
    _filter = _filter | filter;
  }
}


void ESP32Wiimote_setPlayerLEDs(uint8_t leds)
{
   TinyWiimote_setPlayerLEDs(leds);
}


void ESP32Wiimote_rumble(bool rumble)
{
   TinyWiimote_rumble(rumble);
}



