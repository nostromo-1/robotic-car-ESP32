/*
 * Based on ultrasonic by Ruslan V. Uss
 * 
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file ultrasonic.c
 *
 * ESP-IDF driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05 and the like
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include "ultrasonic.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp32/rom/ets_sys.h>

#define TRIGGER_HIGH_DELAY 10
#define PING_TIMEOUT_uS 1000
#define ROUNDTRIP_CM 58
#define MAXDISTANCE 320
#define MINDISTANCE 5


#define timeout_expired(start, len) ((esp_timer_get_time() - (start)) >= (len))

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)

   
static TaskHandle_t callingTask;
static volatile int64_t down_time;

   
/* ISR llamado cuando el trigger_pin cambia de estado HIGH a LOW
Se usa para detectar la respuesta del sensor */
static void IRAM_ATTR level_change(void *args)
{
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

   if (callingTask == NULL) return;
   down_time = esp_timer_get_time();
   
   vTaskNotifyGiveFromISR(callingTask, &xHigherPriorityTaskWoken);
   /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
   portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



esp_err_t ultrasonic_init(const ultrasonic_sensor_t *dev)
{
    CHECK_ARG(dev);

    ESP_ERROR_CHECK(gpio_reset_pin(dev->trigger_pin));
    ESP_ERROR_CHECK(gpio_reset_pin(dev->echo_pin));
    ESP_ERROR_CHECK(gpio_set_direction(dev->trigger_pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(dev->echo_pin, GPIO_MODE_INPUT));
    
    ESP_ERROR_CHECK(gpio_set_level(dev->trigger_pin, 0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(dev->echo_pin, level_change, NULL));  
    return ESP_OK;
}


esp_err_t ultrasonic_measure_raw(const ultrasonic_sensor_t *dev, const uint32_t max_time_ms, uint32_t *time_us)
{
uint32_t ret;
int64_t start_time;

    CHECK_ARG(dev && time_us);
    callingTask = xTaskGetCurrentTaskHandle();

    // Previous ping isn't ended
    if (gpio_get_level(dev->echo_pin)) return ESP_ERR_ULTRASONIC_PING;

    // Ping: high for 10 us
    CHECK(gpio_set_level(dev->trigger_pin, 1));
    ets_delay_us(TRIGGER_HIGH_DELAY);
    CHECK(gpio_set_level(dev->trigger_pin, 0));

    // Wait for echo, takes ca. 600 us to arrive
    start_time = esp_timer_get_time();
    while (!gpio_get_level(dev->echo_pin)) {
        if (timeout_expired(start_time, PING_TIMEOUT_uS))
            return ESP_ERR_ULTRASONIC_PING_TIMEOUT;
    }

    // got echo, measuring
    start_time = down_time = esp_timer_get_time();
    CHECK(gpio_set_intr_type(dev->echo_pin, GPIO_INTR_NEGEDGE));
    ret = ulTaskNotifyTake(
         pdTRUE, // Clear the notification value before exiting: act as a binary semaphore
         pdMS_TO_TICKS(max_time_ms) // Time for timeout
    );   
    CHECK(gpio_set_intr_type(dev->echo_pin, GPIO_INTR_DISABLE));
    if (ret == 0) return ESP_ERR_ULTRASONIC_ECHO_TIMEOUT;

    *time_us = down_time - start_time;
    if (*time_us < (MINDISTANCE * ROUNDTRIP_CM)) return ESP_ERR_ULTRASONIC_PING;  // Echo is too small, error
    return ESP_OK;
}



esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t *dev, uint32_t *distance)
{
    CHECK_ARG(dev && distance);

    uint32_t time_us;
    CHECK(ultrasonic_measure_raw(dev, (MAXDISTANCE * ROUNDTRIP_CM)/1000, &time_us));
    *distance = time_us / ROUNDTRIP_CM;

    return ESP_OK;
}

