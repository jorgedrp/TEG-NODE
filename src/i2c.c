#include "i2c.h"
#include "mpu6050.h"
#include "bme280.h"

static i2c_master_dev_handle_t dev_handle_mpu;
static i2c_master_dev_handle_t dev_handle_bme;

void i2c_master_init(void)
{
    i2c_master_bus_handle_t bus_handle = NULL;

    i2c_master_bus_config_t i2c_mst_config = {
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM_0,
        .glitch_ignore_cnt = 7,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg_mpu = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MPU6050_ADDR,
    .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_mpu, &dev_handle_mpu));

    i2c_device_config_t dev_cfg_bme = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_bme, &dev_handle_bme));
}

i2c_master_dev_handle_t get_i2c_dev_handle_mpu(void)
{
    return dev_handle_mpu;
}

i2c_master_dev_handle_t get_i2c_dev_handle_bme(void)
{
    return dev_handle_bme;
}
