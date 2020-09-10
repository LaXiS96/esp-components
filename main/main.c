#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "mpu.h"

static void imuReadTask(void *arg)
{
    while (1)
    {
        // uint8_t values[14];

        // imuRegRead(IMU_REG_ACCEL_XOUT_H, values, sizeof(values));

        // // ESP_LOGI("IMU", "%.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X"
        // //                 "%.2X %.2X %.2X %.2X %.2X %.2X",
        // //          values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7],
        // //          values[8], values[9], values[10], values[11], values[12], values[13]);

        // ESP_LOGI("IMU", "aX=%5d aY=%5d aZ=%5d "
        //                 "gX=%5d gY=%5d gZ=%5d "
        //                 "T=%5d",
        //          (int16_t)(values[0] << 8 | values[1]), (int16_t)(values[2] << 8 | values[3]), (int16_t)(values[4] << 8 | values[5]),
        //          (int16_t)(values[8] << 8 | values[9]), (int16_t)(values[10] << 8 | values[11]), (int16_t)(values[12] << 8 | values[13]),
        //          (int16_t)(values[6] << 8 | values[7]));

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    mpuInit();

    xTaskCreate(imuReadTask, "IMU READ", 2048, NULL, 1, NULL);
}
