#include <stdio.h>
#include "esp_err.h"
#include "i2c.h"

#define MPU6050_ADDR         0x68 // Dirección I2C del MPU6050
#define MPU6050_WHO_AM_I     0x75 // Registro de identificación
#define MPU6050_PWR_MGMT_1   0x6B // Registro de gestión de energía

//Ratios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250

typedef struct __attribute__((packed)) {
    uint8_t timestamp_l;
    uint8_t timestamp_m;
    uint8_t timestamp_h;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} mpu6050_data_t;

// typedef struct {
//     int16_t ax_offset, ay_offset, az_offset;
//     int16_t gx_offset, gy_offset, gz_offset;
// } mpu6050_offset_t;

typedef struct {
    float umbral_x_pos, umbral_x_neg;
    float umbral_y_pos, umbral_y_neg;
    float umbral_z_pos, umbral_z_neg;
} mpu6050_umbral_t;

typedef struct __attribute__((packed)) {
    uint8_t dev_id;
    uint8_t transmit_data;
    uint8_t timestamp_1;
    uint8_t timestamp_2;
    uint8_t timestamp_3;
    uint8_t timestamp_4;
    uint8_t timestamp_5;
    uint8_t timestamp_6;
    uint8_t timestamp_7;
    uint8_t timestamp_8;
} data_available_t;

esp_err_t mpu6050_write_byte(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t data);
esp_err_t mpu6050_read_byte(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t *data);
esp_err_t mpu6050_read_bytes(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t *data, size_t len);
void mpu6050_init(i2c_master_dev_handle_t i2c_dev);
int16_t combine_bytes(uint8_t msb, uint8_t lsb);
mpu6050_data_t mpu6050_read_accel_gyro(i2c_master_dev_handle_t i2c_dev);
