#include "bme280.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BME280";

// Registros del BME280
#define BME280_REG_CALIB_1    0x88
#define BME280_REG_CALIB_2    0xE1
#define BME280_REG_ID         0xD0
#define BME280_REG_RESET      0xE0
#define BME280_REG_CTRL_HUM   0xF2
#define BME280_REG_STATUS     0xF3
#define BME280_REG_CTRL_MEAS  0xF4
#define BME280_REG_CONFIG     0xF5
#define BME280_REG_PRESS_MSB  0xF7

extern SemaphoreHandle_t i2c_mutex;

struct {
    uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
    uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3; int16_t dig_P4; int16_t dig_P5; int16_t dig_P6; int16_t dig_P7; int16_t dig_P8; int16_t dig_P9;
    uint8_t  dig_H1; int16_t dig_H2; uint8_t  dig_H3; int16_t dig_H4; int16_t dig_H5; int8_t  dig_H6;
} calib;

static int32_t t_fine;

static esp_err_t write_reg8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_transmit(dev, write_buf, sizeof(write_buf), -1);
}

static esp_err_t read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, -1);
}

static esp_err_t bme280_read_calibration(i2c_master_dev_handle_t dev) {
    uint8_t buf[24]; 
    uint8_t h_buf[7];

    ESP_ERROR_CHECK(read_regs(dev, BME280_REG_CALIB_1, buf, 24));
    
    calib.dig_T1 = (buf[1] << 8) | buf[0];
    calib.dig_T2 = (int16_t)((buf[3] << 8) | buf[2]);
    calib.dig_T3 = (int16_t)((buf[5] << 8) | buf[4]);

    calib.dig_P1 = (buf[7] << 8) | buf[6];
    calib.dig_P2 = (int16_t)((buf[9] << 8) | buf[8]);
    calib.dig_P3 = (int16_t)((buf[11] << 8) | buf[10]);
    calib.dig_P4 = (int16_t)((buf[13] << 8) | buf[12]);
    calib.dig_P5 = (int16_t)((buf[15] << 8) | buf[14]);
    calib.dig_P6 = (int16_t)((buf[17] << 8) | buf[16]);
    calib.dig_P7 = (int16_t)((buf[19] << 8) | buf[18]);
    calib.dig_P8 = (int16_t)((buf[21] << 8) | buf[20]);
    calib.dig_P9 = (int16_t)((buf[23] << 8) | buf[22]);

    ESP_ERROR_CHECK(read_regs(dev, 0xA1, &calib.dig_H1, 1));
    
    ESP_ERROR_CHECK(read_regs(dev, BME280_REG_CALIB_2, h_buf, 7));

    calib.dig_H2 = (int16_t)((h_buf[1] << 8) | h_buf[0]);
    calib.dig_H3 = h_buf[2];
    calib.dig_H4 = (int16_t)((h_buf[3] << 4) | (h_buf[4] & 0x0F));
    calib.dig_H5 = (int16_t)((h_buf[5] << 4) | (h_buf[4] >> 4));
    calib.dig_H6 = (int8_t)h_buf[6];

    return ESP_OK;
}

esp_err_t bme280_init(i2c_master_dev_handle_t dev_handle) {
    uint8_t id;
    ESP_ERROR_CHECK(read_regs(dev_handle, BME280_REG_ID, &id, 1));
    if (id != 0x60) {
        ESP_LOGE(TAG, "BME280 ID incorrecto: 0x%02x (esperado 0x60)", id);
        return ESP_FAIL;
    }

    write_reg8(dev_handle, BME280_REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(100));

    bme280_read_calibration(dev_handle);

    // --- CONFIGURACIÓN INDOOR RECOMENDADA ---
    // 1. Config (0xF5): Standby 1000ms (101) | Filter x16 (100) | SPI 3w off (0)
    // Bits: [7:5] T_sb, [4:2] Filter, [0] Spi3w_en
    // 0b10110000 = 0xB0
    ESP_ERROR_CHECK(write_reg8(dev_handle, BME280_REG_CONFIG, 0xB0));

    // 2. Ctrl Hum (0xF2): Oversampling x1
    // Bits: [2:0] osrs_h. x1 = 001
    ESP_ERROR_CHECK(write_reg8(dev_handle, BME280_REG_CTRL_HUM, 0x01));

    // 3. Ctrl Meas (0xF4): Oversampling T x2, P x16, Mode Normal
    // Bits: [7:5] osrs_t (x2=010), [4:2] osrs_p (x16=101), [1:0] mode (Normal=11)
    // 0b01010111 = 0x57
    // NOTA: Escribir en ctrl_meas activa los cambios de ctrl_hum
    ESP_ERROR_CHECK(write_reg8(dev_handle, BME280_REG_CTRL_MEAS, 0x57));

    return ESP_OK;
}

static float compensate_temp(int32_t adc_T) {
    double var1, var2, T;
    var1 = (((double)adc_T) / 16384.0 - ((double)calib.dig_T1) / 1024.0) * ((double)calib.dig_T2);
    var2 = ((((double)adc_T) / 131072.0 - ((double)calib.dig_T1) / 8192.0) *
            (((double)adc_T) / 131072.0 - ((double)calib.dig_T1) / 8192.0)) * ((double)calib.dig_T3);
    t_fine = (int32_t)(var1 + var2);
    T = (var1 + var2) / 5120.0;
    return (float)T;
}

static float compensate_pressure(int32_t adc_P) {
    double var1, var2, p;
    var1 = ((double)t_fine/2.0) - 64000.0;
    var2 = var1 * var1 * ((double)calib.dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)calib.dig_P5) * 2.0;
    var2 = (var2/4.0) + (((double)calib.dig_P4) * 65536.0);
    var1 = (((double)calib.dig_P3) * var1 * var1 / 524288.0 + ((double)calib.dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calib.dig_P1);
    
    if (var1 == 0.0) return 0; // Evitar división por cero

    p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)calib.dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)calib.dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((double)calib.dig_P7)) / 16.0;
    return (float)(p / 100.0); // Retornar en hPa
}

static float compensate_humidity(int32_t adc_H) {
    double var_H;
    var_H = (((double)t_fine) - 76800.0);
    var_H = (adc_H - (((double)calib.dig_H4) * 64.0 + ((double)calib.dig_H5) / 16384.0 * var_H)) *
            (((double)calib.dig_H2) / 65536.0 * (1.0 + ((double)calib.dig_H6) / 67108864.0 * var_H *
            (1.0 + ((double)calib.dig_H3) / 67108864.0 * var_H)));
    var_H = var_H * (1.0 - ((double)calib.dig_H1) * var_H / 524288.0);
    if (var_H > 100.0) var_H = 100.0;
    else if (var_H < 0.0) var_H = 0.0;
    return (float)var_H;
}

esp_err_t bme280_read_data(i2c_master_dev_handle_t dev_handle, bme280_data_t *out_data) {

    uint8_t data[8]; // Press (3) + Temp (3) + Hum (2)
    esp_err_t ret = ESP_OK;
    
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        ret = read_regs(dev_handle, BME280_REG_PRESS_MSB, data, 8);
        xSemaphoreGive(i2c_mutex);
    }
    else
    {
        return ESP_ERR_TIMEOUT;
    }

    if (ret != ESP_OK) return ret;

    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    int32_t adc_H = (data[6] << 8) | data[7];

    out_data->temperature = compensate_temp(adc_T);
    out_data->pressure = compensate_pressure(adc_P);
    out_data->humidity = compensate_humidity(adc_H);

    return ESP_OK;
}