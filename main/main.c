#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "mpu6050.h"
#include "ili9341.h"

static void testTask(void *arg)
{
    while (1)
    {
        // mpuDmpTest();

        ili9341Test();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ili9341Config_t displayConfig = {
        .spiHost = SPI2_HOST,
        .spiMisoIo = GPIO_NUM_12,
        .spiMosiIo = GPIO_NUM_13,
        .spiSclkIo = GPIO_NUM_14,
        .spiCsIo = GPIO_NUM_15,
        .spiDcIo = GPIO_NUM_2,
        .spiResetIo = GPIO_NUM_4,
        .spiClockHz = 1 * 1000 * 1000,
    };
    ili9341Init(&displayConfig);

    // mpuInit();

    xTaskCreate(testTask, "TEST", 2048, NULL, 1, NULL);
}
