#ifndef NETWORK_H
#define NETWORK_H

#include "esp_http_server.h"

int init_wifi_network(bool do_wps);
void get_ota_firmware(void);
httpd_handle_t start_webserver(void);

#endif
