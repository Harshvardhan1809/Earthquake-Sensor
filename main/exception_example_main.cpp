// =========================================================================
// Released under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu_spi.cpp
 * Example on how to setup MPU through SPI for basic usage.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "MPU.hpp"
#include "SPIbus.hpp"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "mpu/math.hpp"
#include "mpu/types.hpp"
#include "sdkconfig.h"


// For communication
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include <time.h>
#include <sys/time.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "sdkconfig.h"
#include <esp_adc/adc_oneshot.h>
// For communication


// For sensor
static const char *TAG = "example";
static constexpr int MOSI = 26;
static constexpr int MISO = 25;
static constexpr int SCLK = 27;
static constexpr int CS = 33;
static constexpr uint32_t CLOCK_SPEED = 100000; // up to 1MHz for all registers, and 20MHz for sensor data registers only
// CHANGED TO 0.1 MHZ
// For sensor


// For communication
#define WEB_SERVER "127.0.0.1" //	<- Use the ip address of your Raspberry pi here.
#define WEB_PORT "50000"
#define WEB_PATH "/id="
#define CONTINUED_WEB_PATH "&TIME="

static adc_oneshot_unit_handle_t adc1_handle;
extern "C" void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

    while (1)
    {
        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if (err != 0 || res == NULL)
        {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if (s < 0)
        {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if (connect(s, res->ai_addr, res->ai_addrlen) != 0)
        {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);

        struct timeval now;
        gettimeofday(&now, NULL);
        struct tm *local_time = localtime(&now.tv_sec);
        char timestamp[10];
        sprintf(timestamp, "%02d:%02d:%02d", local_time->tm_hour, local_time->tm_min, local_time->tm_sec);

        static const char *REQ1 = "GET " WEB_PATH "1";
        static const char *REQ2 = " HTTP/1.0\r\n"
                                  "Host: " WEB_SERVER ":" WEB_PORT "\r\n"
                                  "User-Agent: esp-idf/1.0 esp32\r\n\r\n";
        static char REQUEST[100];
        int value = 0;
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &value);
        strcpy(REQUEST, REQ1);
        sprintf(REQUEST + strlen(REQUEST), "%d", value);
        strcat(REQUEST, CONTINUED_WEB_PATH);
        strcat(REQUEST, timestamp);
        strcat(REQUEST, REQ2);

        if (write(s, REQUEST, strlen(REQUEST)) < 0)
        {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                       sizeof(receiving_timeout)) < 0)
        {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        do
        {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf) - 1);
            for (int i = 0; i < r; i++)
            {
                putchar(recv_buf[i]);
            }
        } while (r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        for (int countdown = 6; countdown >= 0; countdown--)
        {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");
    }
}
// For communication


// For sensor
extern "C" void app_main()
{

    // For communication
    static adc_oneshot_unit_init_cfg_t adc_init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    static adc_oneshot_chan_cfg_t adc1_chan6_cfg = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config1, &adc1_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &adc1_chan6_cfg));

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL);
    // For communication

    printf("$ MPU Driver Example: MPU-SPI\n");
    fflush(stdout);

    spi_device_handle_t mpu_spi_handle;
    // Initialize SPI on HSPI host through SPIbus interface:
    hspi.begin(MOSI, MISO, SCLK);
    hspi.addDevice(0, CLOCK_SPEED, CS, &mpu_spi_handle);

    // Or directly with esp-idf API:
    /*
    spi_bus_config_t spi_config;
    spi_config.mosi_io_num = MOSI;
    spi_config.miso_io_num = MISO;
    spi_config.sclk_io_num = SCLK;
    spi_config.quadwp_io_num = -1;
    spi_config.quadhd_io_num = -1;
    spi_config.max_transfer_sz = SPI_MAX_DMA_LEN;
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &spi_config, 0));
    spi_device_interface_config_t dev_config;
    dev_config.command_bits = 0;
    dev_config.address_bits = 8;
    dev_config.dummy_bits = 0;
    dev_config.mode = 0;
    dev_config.duty_cycle_pos = 128;
    dev_config.cs_ena_pretrans = 0;
    dev_config.cs_ena_posttrans = 0;
    dev_config.clock_speed_hz = CLOCK_SPEED;
    dev_config.spics_io_num = CS;
    dev_config.flags = 0;
    dev_config.queue_size = 1;
    dev_config.pre_cb = NULL;
    dev_config.post_cb = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &mpu_spi_handle));
    */

    MPU_t MPU;                   // create a default MPU object
    MPU.setBus(hspi);            // set bus port, not really needed here since default is HSPI
    MPU.setAddr(mpu_spi_handle); // set spi_device_handle, always needed!

    // Great! Let's verify the communication
    // (this also check if the connected MPU supports the implementation of chip selected in the component menu)
    while (esp_err_t err = MPU.testConnection())
    {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    // Initialize
    ESP_ERROR_CHECK(MPU.initialize()); // initialize the chip and set initial configurations
    // Setup with your configurations
    // ESP_ERROR_CHECK(MPU.setSampleRate(50));  // set sample rate to 50 Hz
    // ESP_ERROR_CHECK(MPU.setGyroFullScale(mpud::GYRO_FS_500DPS));
    // ESP_ERROR_CHECK(MPU.setAccelFullScale(mpud::ACCEL_FS_4G));

    // Reading sensor data
    printf("Reading sensor data:\n");
    mpud::raw_axes_t accelRaw;  // x, y, z axes as int16
    mpud::raw_axes_t gyroRaw;   // x, y, z axes as int16
    mpud::float_axes_t accelG;  // accel axes in (g) gravity format
    mpud::float_axes_t gyroDPS; // gyro axes in (DPS) º/s format
    while (true)
    {
        // Read
        MPU.acceleration(&accelRaw); // fetch raw data from the registers
        MPU.rotation(&gyroRaw);      // fetch raw data from the registers
        // MPU.motion(&accelRaw, &gyroRaw);  // read both in one shot
        // Convert
        accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
        gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
        // Debug
        printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.x, accelG.y, accelG.z);
        printf("gyro: [%+7.2f %+7.2f %+7.2f ] (º/s)\n", gyroDPS[0], gyroDPS[1], gyroDPS[2]);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
// For sensor
