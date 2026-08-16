#include "driver/i2c_master.h"
#include "esp_err.h"

#define BME280_ADDR       0x76

typedef struct __attribute__((packed)) {
    uint8_t timestamp_l;
    uint8_t timestamp_m;
    uint8_t timestamp_h;
    float temperature;
    float pressure;
    float humidity;
} bme280_data_t;

esp_err_t bme280_init(i2c_master_dev_handle_t dev_handle);
esp_err_t bme280_read_data(i2c_master_dev_handle_t dev_handle, bme280_data_t *out_data);