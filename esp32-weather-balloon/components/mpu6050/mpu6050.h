#ifndef _MPU6050_H
#define _MPU6050_H

#include <esp_system.h>

#define SENSORS_GRAVITY_EARTH (9.80665F)
#define SENSORS_DPS_TO_RADS (0.017453293F)

#define MPU6050_ADDR (0x68)

#define MPU6050_SELF_TEST_X_REG (0x0D)
#define MPU6050_SELF_TEST_Y_REG (0x0E)
#define MPU6050_SELF_TEST_Z_REG (0x0F)
#define MPU6050_SELF_TEST_A_REG (0x10)
#define MPU6050_SMPRT_DIV_REG (0x19)
#define MPU6050_CONFIG_REG (0x1A)
#define MPU6050_GYRO_CONFIG_REG (0x1B)
#define MPU6050_ACCEL_CONFIG_REG (0x1C)
#define MPU6050_INT_PIN_CONFIG_REG (0x37)
#define MPU6050_WHO_AM_I_REG (0x75)
#define MPU6050_SIGNAL_PATH_RESET_REG (0x68)
#define MPU6050_USER_CTRL_REG (0x6A)
#define MPU6050_PWR_MGMT_1_REG (0x6B)
#define MPU6050_PWR_MGMT_2_REG (0x6C)
#define MPU6050_TEMP_H_REG (0x41)
#define MPU6050_TEMP_L_REG (0x42)
#define MPU6050_ACCEL_OUT_REG (0x3B)
#define MPU6050_RESET_BIT 7

#define MPU6050_FSYNC_OUT_DISABLED (0)
#define MPU6050_FSYNC_OUT_TEMP (1)
#define MPU6050_FSYNC_OUT_GYROX (2)
#define MPU6050_FSYNC_OUT_GYROY (3)
#define MPU6050_FSYNC_OUT_GYROZ (4)
#define MPU6050_FSYNC_OUT_ACCELX (5)
#define MPU6050_FSYNC_OUT_ACCELY (6)
#define MPU6050_FSYNC_OUT_ACCEL_Z (7)

#define MPU6050_INTR_8MHz (0)
#define MPU6050_PLL_GYROX (1)
#define MPU6050_PLL_GYROY (2)
#define MPU6050_PLL_GYROZ (3)
#define MPU6050_PLL_EXT_32K (4)
#define MPU6050_PLL_EXT_19MHz (5)
#define MPU6050_STOP (7)

#define MPU6050_RANGE_2_G (0)
#define MPU6050_RANGE_4_G (1)
#define MPU6050_RANGE_8_G (2)
#define MPU6050_RANGE_16_G (3)

#define MPU6050_RANGE_250_DEG (0)
#define MPU6050_RANGE_500_DEG (1)
#define MPU6050_RANGE_1000_DEG (2)
#define MPU6050_RANGE_2000_DEG (3)

#define MPU6050_BAND_260_HZ (0)
#define MPU6050_BAND_184_HZ (1)
#define MPU6050_BAND_94_HZ (2)
#define MPU6050_BAND_44_HZ (3)
#define MPU6050_BAND_21_HZ (4)
#define MPU6050_BAND_10_HZ (5)
#define MPU6050_BAND_5_HZ (6)

#define MPU6050_CYCLE_1_25_HZ (0)
#define MPU6050_CYCLE_5_HZ (1)
#define MPU6050_CYCLE_20_HZ (2)
#define MPU6050_CYCLE_40_HZ (3)

typedef struct data_out_t
{
    float temperature;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_data_out_t;

esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t mpu6050_init(void);
esp_err_t mpu6050_reset(void);
esp_err_t mpu6050_read(mpu6050_data_out_t *data);

#endif /* _MPU6050_H */