#pragma once

#include "hal/gpio_types.h"
#include "hal/spi_types.h"

#define ILI9341_CMD_NOP 0x00
#define ILI9341_CMD_SWRESET 0x01
#define ILI9341_CMD_RDDIDIF 0x04
#define ILI9341_CMD_RDDST 0x09

typedef struct
{
    spi_host_device_t spiHost;
    gpio_num_t spiMisoIo;
    gpio_num_t spiMosiIo;
    gpio_num_t spiSclkIo;
    gpio_num_t spiCsIo;
    gpio_num_t spiDcIo;
    gpio_num_t spiResetIo;
    int32_t spiClockHz;
} ili9341Config_t;

void ili9341Init(ili9341Config_t *config);
void ili9341Test(void);
void ili9341Dispose(void);
