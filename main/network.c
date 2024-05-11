/**********
File network.c
It takes care of all network issues: wifi set-up, WPS, NTP
It can also download a new firmware version from github

***********/

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wps.h"
#include "esp_netif_sntp.h"

#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"

#include "oled96.h"

#define MAX_RETRY_ATTEMPTS  3
#define OTA_TIMEOUT_MS  5000


/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char* TAG = __FILE__;
// URL of firmware to download at start
static const char* ota_url = "https://raw.githubusercontent.com/nostromo-1/robotic-car-ESP32/master/build/robotic-car.bin";
// PEM certificate for downloading OTA firmware. It is embedded in the program file at build time.
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_raw_githubusercontent_com_pem_start");

static esp_wps_config_t wps_config = WPS_CONFIG_INIT_DEFAULT(WPS_TYPE_PBC);
static wifi_config_t wps_ap_creds[MAX_WPS_AP_CRED];
static int s_ap_creds_num = 0;
static int s_retry_num = 0;




static void ota_event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == ESP_HTTPS_OTA_EVENT) {
        switch (event_id) {
            case ESP_HTTPS_OTA_START:
                ESP_LOGD(TAG, "OTA started");
                break;
            case ESP_HTTPS_OTA_CONNECTED:
                ESP_LOGD(TAG, "Connected to server");
                break;
            case ESP_HTTPS_OTA_GET_IMG_DESC:
                ESP_LOGD(TAG, "Reading Image Description");
                break;
            case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
                ESP_LOGD(TAG, "Verifying chip id of new image: %d", *(esp_chip_id_t *)event_data);
                break;
            case ESP_HTTPS_OTA_DECRYPT_CB:
                ESP_LOGD(TAG, "Callback to decrypt function");
                break;
            case ESP_HTTPS_OTA_WRITE_FLASH:
                ESP_LOGD(TAG, "Writing to flash: %d written", *(int *)event_data);
                break;
            case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
                ESP_LOGD(TAG, "Boot partition updated. Next Partition: %d", *(esp_partition_subtype_t *)event_data);
                break;
            case ESP_HTTPS_OTA_FINISH:
                ESP_LOGD(TAG, "OTA finish");
                break;
            case ESP_HTTPS_OTA_ABORT:
                ESP_LOGD(TAG, "OTA abort");
                break;
        }
    }
}



static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
        break;
    }
    return ESP_OK;
}



static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
static int ap_idx = 1;

    switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
            break;
            
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");
            if (s_retry_num < MAX_RETRY_ATTEMPTS) {
                ESP_LOGI(TAG, "retry to connect to the AP");
                esp_wifi_connect();
                s_retry_num++;
            } else if (ap_idx < s_ap_creds_num) {
                /* Try the next AP credential if first one fails */
                if (ap_idx < s_ap_creds_num) {
                    ESP_LOGI(TAG, "Connecting to SSID: %s, Passphrase: %s",
                             wps_ap_creds[ap_idx].sta.ssid, wps_ap_creds[ap_idx].sta.password);
                    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wps_ap_creds[ap_idx++]) );
                    esp_wifi_connect();
                }
                s_retry_num = 0;
            } else {
                ESP_LOGI(TAG, "Failed to connect!");
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            break;
            
        case WIFI_EVENT_STA_WPS_ER_SUCCESS:
            ESP_LOGI(TAG, "WIFI_EVENT_STA_WPS_ER_SUCCESS");
            {
                wifi_event_sta_wps_er_success_t *evt = (wifi_event_sta_wps_er_success_t *)event_data;
                if (evt) {
                    s_ap_creds_num = evt->ap_cred_cnt;
                    for (int i = 0; i < s_ap_creds_num; i++) {
                        memcpy(wps_ap_creds[i].sta.ssid, evt->ap_cred[i].ssid,
                               sizeof(evt->ap_cred[i].ssid));
                        memcpy(wps_ap_creds[i].sta.password, evt->ap_cred[i].passphrase,
                               sizeof(evt->ap_cred[i].passphrase));
                    }
                    /* If multiple AP credentials are received from WPS, connect with first one */
                    ESP_LOGI(TAG, "Connecting to SSID: %s, Passphrase: %s",
                             wps_ap_creds[0].sta.ssid, wps_ap_creds[0].sta.password);
                    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wps_ap_creds[0]));
                }
                /*
                 * If only one AP credential is received from WPS, there will be no event data and
                 * esp_wifi_set_config() is already called by WPS modules for backward compatibility
                 * with legacy apps. So directly attempt connection here.
                 */
                ESP_ERROR_CHECK(esp_wifi_wps_disable());
                esp_wifi_connect();
            }
            break;
            
        case WIFI_EVENT_STA_WPS_ER_FAILED:
            ESP_LOGI(TAG, "WIFI_EVENT_STA_WPS_ER_FAILED");
            ESP_ERROR_CHECK(esp_wifi_wps_disable());
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            break;
            
        case WIFI_EVENT_STA_WPS_ER_TIMEOUT:
            ESP_LOGI(TAG, "WIFI_EVENT_STA_WPS_ER_TIMEOUT");
            ESP_ERROR_CHECK(esp_wifi_wps_disable());
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            break;
            
        default:
            break;
    }
}


static void got_ip_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data)
{
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
}



static int wifi_init_sta(bool do_wps)
{
wifi_config_t wifi_config;
   
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (!sta_netif) return -1;
   
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();  // Default values store config in NVS when esp_wifi_set_config
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
                                                  
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip_event_handler, NULL));    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));  // Station mode (not AP mode)
    ESP_ERROR_CHECK(esp_wifi_start());    
    
    // If wifi data is stored in NVS, use it. Otherwise, run WPS
    // But if user presses WPS button when car starts, run WPS service in any case
    if (esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config) == ESP_OK) {
       size_t ssidLen = strlen((char*)wifi_config.sta.ssid);
       if (ssidLen == 0 || do_wps) {
         ESP_LOGI(TAG, "Start WPS service");
         oledBigMessage(0, "  WPS  ");
         oledWriteString(0, 6, "Press WPS button", false);
         oledWriteString(0, 7, "in wifi router", false);
         ESP_ERROR_CHECK(esp_wifi_wps_enable(&wps_config));
         ESP_ERROR_CHECK(esp_wifi_wps_start(0));
       }
       else esp_wifi_connect();  // Credentials found in NVS 
    }
    else return -1;
    
    /* Wait until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries. The bits are set by the event handler */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to ap SSID: %s", wifi_config.sta.ssid);
        return 0;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to wifi");
        esp_wifi_disconnect();
        esp_wifi_stop();
        return -1;
    } 
    else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return -1;
    }
}



/* CB called when NTP sync was successful */
void time_sync_notification_cb(struct timeval *tv)
{
time_t now;

    time(&now);
    setenv("TZ", "CET-1CEST", 1); // Set timezone to Central European Standard Time. See https://ftp.fau.de/aminet/util/time/tzinfo.txt
    tzset();
    ESP_LOGI(TAG, "Time set via NTP: %s", ctime(&now));
}


static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
{
    esp_err_t err = ESP_OK;
    /* Uncomment to add custom headers to HTTP request */
    // err = esp_http_client_set_header(http_client, "Custom-Header", "Value");
    return err;
}


static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
{
esp_err_t err;
esp_app_desc_t running_app_info;
char remote_version[sizeof(new_app_info->version)];
char local_version[sizeof(new_app_info->version)];

    if (new_app_info == NULL) return ESP_ERR_INVALID_ARG;
    const esp_partition_t *running = esp_ota_get_running_partition();
    err = esp_ota_get_partition_description(running, &running_app_info);
    if (err != ESP_OK) return err;
    
    ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
    ESP_LOGI(TAG, "Remote firmware version: %s", new_app_info->version);
    
    for (int i=0; i<sizeof(new_app_info->version); i++) {
       char p;
       p = running_app_info.version[i];
       if (p == '\0') local_version[i] = p;
       else local_version[i] = (p < '0' || p > '9')?'0':p;
       p = new_app_info->version[i];
       if (p == '\0') remote_version[i] = p;
       else remote_version[i] = (p < '0' || p > '9')?'0':p;
    }
    
    int local_v = atoi(local_version);
    int remote_v = atoi(remote_version);
    return (remote_v > local_v)?ESP_OK:ESP_FAIL;
}


/**
Look for new firmware version in github. 
If firmware there has a different version than the local one, download it and restart CPU.
**/
void get_ota_firmware(void)
{
esp_err_t err, ota_finish_err = ESP_OK;
esp_http_client_config_t config = {
        .url = ota_url,
        .cert_pem = (char *)server_cert_pem_start,
        .event_handler = http_event_handler,
        .timeout_ms = OTA_TIMEOUT_MS,
        .keep_alive_enable = true,
        .skip_cert_common_name_check = false,
};
esp_https_ota_config_t ota_config = {
        .http_config = &config,
        .http_client_init_cb = _http_client_init_cb, // Register a callback to be invoked after esp_http_client is initialized
        .partial_http_download = true,
        .max_http_request_size = CONFIG_MBEDTLS_SSL_IN_CONTENT_LEN,
};
    
    ESP_LOGI(TAG, "Attempting to download firmware update from %s", config.url);
    ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &ota_event_handler, NULL));  
    esp_wifi_set_ps(WIFI_PS_NONE);  // Disable any WiFi power save mode, this allows best throughput
    
    esp_https_ota_handle_t https_ota_handle = NULL;
    err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
        return;
    }
    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed");
        goto ota_end;
    }        
    err = validate_image_header(&app_desc);
    if (err != ESP_OK) goto ota_end;
    
    int ota_size = esp_https_ota_get_image_size(https_ota_handle);
    if (ota_size == -1) {
        ESP_LOGE(TAG, "esp_https_ota_get_image_size failed");
        goto ota_end;       
    }
    oledBigMessage(0, "Firmware");
    oledBigMessage(1, "download");
    // Read firmware file in loop
    while (1) {
        static const uint8_t glyph[] = {0xFF, 0, 0, 0, 0, 0, 0, 0}; 
        uint8_t pos = 0;
       
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) break;
        
        int len = esp_https_ota_get_image_len_read(https_ota_handle);
        if (len != -1) {
            ESP_LOGD(TAG, "Image bytes read: %d", len);
            // Draw progress bar on display
            oledSetBitmap8x8((len*127)/ota_size, 0, glyph);
            oledSetBitmap8x8((len*127)/ota_size, 1, glyph);
        }
    }
    
    if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
        // the OTA image was not completely received
        ESP_LOGE(TAG, "Complete data was not received");
        goto ota_end;
    } 
    ota_finish_err = esp_https_ota_finish(https_ota_handle);
    if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
        ESP_LOGI(TAG, "Firmware update successful. Rebooting ...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    } else {
        if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        }
        ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
        ESP_LOGI(TAG, "Firmware NOT updated");
    }
    return;
    
ota_end:
    esp_https_ota_abort(https_ota_handle);
    ESP_LOGI(TAG, "Firmware NOT updated");
    return;
}


/**
Initialize wifi network:
1) If do_wps is true, then it will always look for a WPS enabled station. If found, it will connect to it.
Otherwise, it will connect to the last successful station.
2) Set time from NTP server.
**/
int init_wifi_network(bool do_wps)
{
    // Prepare NTP configuration
    //esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");  
    esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2, ESP_SNTP_SERVER_LIST("hora.roa.es", "pool.ntp.org"));  
    sntp_config.sync_cb = time_sync_notification_cb;
    esp_netif_sntp_init(&sntp_config); 
    
    // Start wifi connection
    if (wifi_init_sta(do_wps) < 0) return -1;

    // Set time with NTP
    int retry = 0;
    const int retry_count = 15;
    while (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(2000)) != ESP_OK && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }
    
    return 0;  // NTP error is not a reason for failure
}



