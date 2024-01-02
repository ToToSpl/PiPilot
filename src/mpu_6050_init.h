#ifndef MPU_6050_INIT
#define MPU_6050_INIT

#include "hardware/i2c.h"
#include "haw/MPU6050.h"
#include <stdio.h>

#define MPU_6050_I2C_SDA_PIN 6
#define MPU_6050_I2C_SCL_PIN 7
#define MPU_6050_I2C_LINE i2c1
#define MPU_6050_I2C_ADRESS 0x68

mpu6050_t mpu_6050_custom_init() {

  // Setup I2C properly
  gpio_init(MPU_6050_I2C_SDA_PIN);
  gpio_init(MPU_6050_I2C_SCL_PIN);
  gpio_set_function(MPU_6050_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(MPU_6050_I2C_SCL_PIN, GPIO_FUNC_I2C);
  // Don't forget the pull ups! | Or use external ones
  gpio_pull_up(MPU_6050_I2C_SDA_PIN);
  gpio_pull_up(MPU_6050_I2C_SCL_PIN);

  mpu6050_t mpu6050 = mpu6050_init(MPU_6050_I2C_LINE, MPU_6050_I2C_ADRESS);
  // Check if the MPU6050 can initialize
  if (mpu6050_begin(&mpu6050)) {
    // Set scale of gyroscope
    mpu6050_set_scale(&mpu6050, MPU6050_SCALE_500DPS);
    // Set range of accelerometer
    mpu6050_set_range(&mpu6050, MPU6050_RANGE_4G);

    // Enable temperature, gyroscope and accelerometer readings
    mpu6050_set_gyroscope_measuring(&mpu6050, true);
    mpu6050_set_accelerometer_measuring(&mpu6050, true);

    // Disable free fall, motion and zero motion interrupt flags
    mpu6050_set_int_free_fall(&mpu6050, false);
    mpu6050_set_int_motion(&mpu6050, false);
    mpu6050_set_int_zero_motion(&mpu6050, false);
  } else {
    while (1) {
      // Endless loop
      printf("Error! MPU6050 could not be initialized. Make sure you've "
             "entered the correct address. And double check your connections.\n");
      sleep_ms(500);
    }
  }

  return mpu6050;
}

#endif
