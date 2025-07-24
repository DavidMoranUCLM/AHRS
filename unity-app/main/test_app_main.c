/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "esp_log.h"


void app_main(void)
{
    esp_log_level_set("BUFFER", ESP_LOG_DEBUG);
    unity_run_menu();
}