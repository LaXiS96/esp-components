#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define IMU_I2C_NUM I2C_NUM_0
#define IMU_I2C_ADDR 0x68

#define IMU_REG_PWR_MGMT_1 0x6B
#define IMU_REG_GYRO_CONFIG 0x1B
#define IMU_REG_ACCEL_CONFIG 0x1C
#define IMU_REG_ACCEL_XOUT_H 0x3B
// #define IMU_REG_ACCEL_XOUT_L 0x3C
// #define IMU_REG_ACCEL_YOUT_H 0x3D
// #define IMU_REG_ACCEL_YOUT_L 0x3E
// #define IMU_REG_ACCEL_ZOUT_H 0x3F
// #define IMU_REG_ACCEL_ZOUT_L 0x40

void imuRegWrite(uint8_t reg, uint8_t *data, size_t dataLen)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);

    // Slave address (write bit)
    i2c_master_write_byte(cmd, IMU_I2C_ADDR << 1 | I2C_MASTER_WRITE, true);
    // Register address
    i2c_master_write_byte(cmd, reg, true);

    // Write data
    i2c_master_write(cmd, data, dataLen, true);

    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(IMU_I2C_NUM, cmd, portMAX_DELAY));
    i2c_cmd_link_delete(cmd);
}

void imuRegRead(uint8_t reg, uint8_t *data, size_t dataLen)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);

    // Slave address (write bit)
    i2c_master_write_byte(cmd, IMU_I2C_ADDR << 1 | I2C_MASTER_WRITE, true);
    // Register address
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);

    // Slave address (read bit)
    i2c_master_write_byte(cmd, IMU_I2C_ADDR << 1 | I2C_MASTER_READ, true);

    // Read data
    i2c_master_read(cmd, data, dataLen, I2C_MASTER_LAST_NACK);

    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(IMU_I2C_NUM, cmd, portMAX_DELAY));
    i2c_cmd_link_delete(cmd);
}

void imuInit(void)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = GPIO_NUM_22,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(IMU_I2C_NUM, &config);

    i2c_driver_install(IMU_I2C_NUM, config.mode, 0, 0, 0);

    imuRegWrite(IMU_REG_PWR_MGMT_1, (uint8_t[]){0}, 1);
    imuRegWrite(IMU_REG_GYRO_CONFIG, (uint8_t[]){0x18}, 1);  // Set gyro full scale range to +-2000°/s
    imuRegWrite(IMU_REG_ACCEL_CONFIG, (uint8_t[]){0x18}, 1); // Set accel full scale range to +-16g
}

static void imuReadTask(void *arg)
{
    while (1)
    {
        uint8_t values[14];

        imuRegRead(IMU_REG_ACCEL_XOUT_H, values, sizeof(values));

        // ESP_LOGI("IMU", "%.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X"
        //                 "%.2X %.2X %.2X %.2X %.2X %.2X",
        //          values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7],
        //          values[8], values[9], values[10], values[11], values[12], values[13]);

        ESP_LOGI("IMU", "aX=%5d aY=%5d aZ=%5d "
                        "gX=%5d gY=%5d gZ=%5d "
                        "T=%5d",
                 (int16_t)(values[0] << 8 | values[1]), (int16_t)(values[2] << 8 | values[3]), (int16_t)(values[4] << 8 | values[5]),
                 (int16_t)(values[8] << 8 | values[9]), (int16_t)(values[10] << 8 | values[11]), (int16_t)(values[12] << 8 | values[13]),
                 (int16_t)(values[6] << 8 | values[7]));

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    imuInit();

    xTaskCreate(imuReadTask, "IMU READ", 2048, NULL, 1, NULL);
}
