#include "mpu6050.h"
#include "freertos/FreeRTOS.h"

extern SemaphoreHandle_t i2c_mutex;

esp_err_t mpu6050_write_byte(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t data)
{
    return i2c_master_transmit(i2c_dev, (uint8_t[]){reg_addr, data}, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_read_byte(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t *data)
{
    i2c_master_transmit(i2c_dev, &reg_addr, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return i2c_master_receive(i2c_dev, data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_read_bytes(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(i2c_dev, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

int16_t combine_bytes(uint8_t msb, uint8_t lsb)
{
    return (int16_t)((msb << 8) | lsb);
}

mpu6050_data_t mpu6050_read_accel_gyro(i2c_master_dev_handle_t i2c_dev)
{
    uint8_t raw_data[14]; // 6 bytes acelerómetro + 2 temp + 6 bytes giroscopio
    mpu6050_data_t data;
    esp_err_t ret = ESP_OK;

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1)) == pdTRUE)
    {
        ret = mpu6050_read_bytes(i2c_dev, 0x3B, raw_data, 14);
        xSemaphoreGive(i2c_mutex);
    }
    else
    {
        printf("No se tomaron los datos del MPU en el tiempo adecuado.\n");
    }
    
    if (ret != ESP_OK) {
        printf("Error leyendo datos del sensor: %s\n", esp_err_to_name(ret));
    }

    data.ax = combine_bytes(raw_data[0], raw_data[1]);
    data.ay = combine_bytes(raw_data[2], raw_data[3]);
    data.az = combine_bytes(raw_data[4], raw_data[5]);

    data.gx = combine_bytes(raw_data[8], raw_data[9]);
    data.gy = combine_bytes(raw_data[10], raw_data[11]);
    data.gz = combine_bytes(raw_data[12], raw_data[13]);

    //printf("Accel [X:%.2f Y:%.2f Z:%.2f] | Gyro [X:%.2f Y:%.2f Z:%.2f]\n", data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
    return data;
}

void mpu6050_init(i2c_master_dev_handle_t i2c_dev)
{
    esp_err_t ret;
    //mpu6050_data_t data;
    //mpu6050_offset_t offset_data;

    mpu6050_write_byte(i2c_dev, MPU6050_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu6050_write_byte(i2c_dev, 0x68, 0x07);
    vTaskDelay(pdMS_TO_TICKS(100));

    ret = mpu6050_write_byte(i2c_dev, MPU6050_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));
    if (ret != ESP_OK) 
    {
        printf("Error inicializando el MPU6050: %s\n", esp_err_to_name(ret));
    }

    //Filtro pasa bajo del MPU6050
    //0     260Hz       0ms
    //1     184Hz       2.0ms
    //2     94Hz        3.0ms
    //3     44Hz        4.9ms
    //4     21Hz        8.5ms
    //5     10Hz        13.8ms
    //6     5Hz         19.0ms
    //mpu6050_write_byte(i2c_dev, 0x1A, 0x02);

    // for(int i = 0 ; i < 1000 ; i++)
    // {
    //     data = mpu6050_read_accel_gyro(i2c_dev);

    //     offset_data.ax_offset += data.ax;
    //     offset_data.ay_offset += data.ay;
    //     offset_data.az_offset += data.az - 16384;
    //     offset_data.gx_offset += data.gx;
    //     offset_data.gy_offset += data.gy;
    //     offset_data.gz_offset += data.gz;

    //     if(i == 999)
    //     {
    //         offset_data.ax_offset = (double)(offset_data.ax_offset/1000);
    //         offset_data.ay_offset = (double)(offset_data.ay_offset/1000);
    //         offset_data.az_offset = (double)(offset_data.az_offset/1000);
    //         offset_data.gx_offset = (double)(offset_data.gx_offset/1000);
    //         offset_data.gy_offset = (double)(offset_data.gy_offset/1000);
    //         offset_data.gz_offset = (double)(offset_data.gz_offset/1000);
    //     }

    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }

    // return offset_data;

    // // Leer WHO_AM_I
    // uint8_t who_am_i = 0;
    // ret = mpu6050_read_byte(i2c_dev, MPU6050_WHO_AM_I, &who_am_i);
    // if (ret == ESP_OK) {
    //     printf("WHO_AM_I = 0x%02X\n", who_am_i);
    //     if (who_am_i == MPU6050_ADDR) {
    //         printf("MPU6050 detectado correctamente\n");
    //     } else {
    //         printf("ID del dispositivo inesperado\n");
    //     }
    // } else {
    //     printf("Error leyendo WHO_AM_I: %s\n", esp_err_to_name(ret));
    // }
}

