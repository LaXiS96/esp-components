/**
 * https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf
 */

#include "ili9341.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

static const char *TAG = "ILI9341";

static ili9341Config_t *_config;
static spi_device_handle_t *_device;

void ili9341Reset(void)
{
    gpio_set_level(_config->spiResetIo, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(_config->spiResetIo, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Send command on the SPI bus and read returned data (if readData and readLen are specified)
 */
void ili9341SendCommand(uint8_t cmd, uint8_t *readData, size_t readLen, bool useDummyBit)
{
    static uint32_t user = 0; // Command
    spi_transaction_ext_t trans = {
        .base = {
            .cmd = cmd,
            .user = &user,
            .flags = SPI_TRANS_VARIABLE_DUMMY,
        },
        .dummy_bits = useDummyBit ? 1 : 0,
    };

    if (readData != NULL && readLen > 0)
    {
        trans.base.rx_buffer = readData;
        trans.base.rxlength = readLen * 8;
        trans.base.length += trans.base.rxlength;
    }

    // TODO non-terminating error check
    ESP_ERROR_CHECK(spi_device_polling_transmit(*_device, (spi_transaction_t *)&trans));
}

void ili9341SendData(uint8_t *data, size_t len)
{
    static uint32_t user = 1; // Data
    spi_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = data,
        .user = &user,
    };
    // TODO non-terminating error check
    ESP_ERROR_CHECK(spi_device_polling_transmit(*_device, &trans));
}

// void ili9341ReadData(uint8_t *data, size_t len)
// {
//     static uint32_t user = 1; // Data
//     spi_transaction_t trans = {
//         .length = len * 8,
//         .rx_buffer = data,
//         .user = &user,
//     };
//     // TODO non-terminating error check
//     ESP_ERROR_CHECK(spi_device_polling_transmit(*_device, &trans));
// }

static void ili9341PreTransferCallback(spi_transaction_t *trans)
{
    if (trans->user != NULL)
    {
        uint32_t dc = *(uint32_t *)(trans->user);
        gpio_set_level(_config->spiDcIo, dc);
    }
}

void ili9341Init(ili9341Config_t *config)
{
    // Save a local copy of passed configuration
    _config = malloc(sizeof(ili9341Config_t));
    memcpy(_config, config, sizeof(ili9341Config_t));

    gpio_set_direction(config->spiDcIo, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->spiResetIo, GPIO_MODE_OUTPUT);

    spi_bus_config_t busConfig = {
        .miso_io_num = config->spiMisoIo,
        .mosi_io_num = config->spiMosiIo,
        .sclk_io_num = config->spiSclkIo,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    // TODO use DMA
    ESP_ERROR_CHECK(spi_bus_initialize(config->spiHost, &busConfig, 0));

    spi_device_interface_config_t deviceConfig = {
        .spics_io_num = config->spiCsIo,
        .clock_speed_hz = config->spiClockHz,
        .command_bits = 8,
        .mode = 0,
        .queue_size = 1,
        .pre_cb = ili9341PreTransferCallback,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    _device = malloc(sizeof(spi_device_handle_t));
    ESP_ERROR_CHECK(spi_bus_add_device(config->spiHost, &deviceConfig, _device));

    ili9341Reset();
}

void ili9341Test(void)
{
    uint8_t data[5] = {0xAA};

    ili9341SendCommand(ILI9341_CMD_RDDIDIF, data, 4, true);
    // ili9341ReadData(data, 4);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, 4, ESP_LOG_INFO);

    ili9341SendCommand(ILI9341_CMD_RDDST, data, 5, true);
    // ili9341ReadData(data, 5);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, 5, ESP_LOG_INFO);
}

void ili9341Dispose(void)
{
    spi_bus_remove_device(*_device);
    spi_bus_free(_config->spiHost);

    free(_config);
    free(_device);
}
