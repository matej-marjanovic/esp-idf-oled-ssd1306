/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// Basic u8g2 example on esp-idf
// u8g2 is installed as suggested: https://github.com/mkfrey/u8g2-hal-esp-idf
// CONFIG_FREERTOS_HZ is set to 1000 in sdkconfig
// everything else is default.
// tested on ESP32-C3 gives about 4.2 FPS which is very slow.
// similar basic default sketch running on the same chip on Arduino gives FPS of over 30.

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "u8g2.h"
#include "u8g2_esp32_hal.h"
#include <esp_log.h>
#include <string.h>
#include <driver/gpio.h>

#include <time.h>
#include <stdlib.h>

// SDA - GPIO21
#define PIN_SDA 3
// SCL - GPIO22
#define PIN_SCL 4

static const char *TAG = "SH1106";

void app_main(void)
{
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.bus.i2c.sda = PIN_SDA;
    u8g2_esp32_hal.bus.i2c.scl = PIN_SCL;
    u8g2_esp32_hal_init(u8g2_esp32_hal);
    
    u8g2_t u8g2; // a structure which will contain all the data for one display
    u8g2_Setup_sh1106_i2c_128x64_vcomh0_f(
		&u8g2,
		U8G2_R2,
		//u8x8_byte_sw_i2c,
		u8g2_esp32_i2c_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
	u8x8_SetI2CAddress(&u8g2.u8x8,0x78);
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
    u8g2_SetFont(&u8g2, u8g2_font_logisoso50_tn); // set big font

    unsigned long currentMillis = xTaskGetTickCount(); // set CONFIG_FREERTOS_HZ=1000 in sdkconfig (default is 100)
    unsigned long prevMillis = currentMillis; 

    while(1) {
        u8g2_ClearBuffer(&u8g2);
        int twoDigitNum = rand() % 100;
        char twoDigitNumStr[12];
        sprintf(twoDigitNumStr, "%d", twoDigitNum);
        u8g2_DrawStr(&u8g2, -4,64, twoDigitNumStr); // 0 to 99
        u8g2_SendBuffer(&u8g2);
        // u8g2_UpdateDisplay(&u8g2);

        prevMillis = currentMillis;
        currentMillis = xTaskGetTickCount();
        unsigned long timeDelta = currentMillis - prevMillis;
        float fps = 1000.0 / (float) timeDelta;
        ESP_LOGI(TAG, "ms delta = %ld \n ", timeDelta);
        ESP_LOGI(TAG, "FPS = %f \n ", fps);
    }
    
}
