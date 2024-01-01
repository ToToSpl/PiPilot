#include "hardware/i2c.h"
#include "haw/MPU6050.h"
#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

#define MPU_6050_I2C_SDA_PIN 6
#define MPU_6050_I2C_SCL_PIN 7

void main_core1() {
  // Setup I2C properly
  gpio_init(MPU_6050_I2C_SDA_PIN);
  gpio_init(MPU_6050_I2C_SCL_PIN);
  gpio_set_function(MPU_6050_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(MPU_6050_I2C_SCL_PIN, GPIO_FUNC_I2C);
  // Don't forget the pull ups! | Or use external ones
  gpio_pull_up(MPU_6050_I2C_SDA_PIN);
  gpio_pull_up(MPU_6050_I2C_SCL_PIN);

  mpu6050_t mpu6050 = mpu6050_init(i2c1, 0x68);
  // Check if the MPU6050 can initialize
  if (mpu6050_begin(&mpu6050)) {
    // Set scale of gyroscope
    mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);
    // Set range of accelerometer
    mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);

    // Enable temperature, gyroscope and accelerometer readings
    mpu6050_set_temperature_measuring(&mpu6050, true);
    mpu6050_set_gyroscope_measuring(&mpu6050, true);
    mpu6050_set_accelerometer_measuring(&mpu6050, true);

    // Enable free fall, motion and zero motion interrupt flags
    mpu6050_set_int_free_fall(&mpu6050, false);
    mpu6050_set_int_motion(&mpu6050, false);
    mpu6050_set_int_zero_motion(&mpu6050, false);

    // Set motion detection threshold and duration
    mpu6050_set_motion_detection_threshold(&mpu6050, 2);
    mpu6050_set_motion_detection_duration(&mpu6050, 5);

    // Set zero motion detection threshold and duration
    mpu6050_set_zero_motion_detection_threshold(&mpu6050, 4);
    mpu6050_set_zero_motion_detection_duration(&mpu6050, 2);
  } else {
    while (1) {
      // Endless loop
      printf("Error! MPU6050 could not be initialized. Make sure you've "
             "entered the correct address. And double check your connections.");
      sleep_ms(500);
    }
  }

  while (1) {
    // Fetch all data from the sensor | I2C is only used here
    mpu6050_event(&mpu6050);

    // Pointers to float vectors with all the results
    mpu6050_vectorf_t *accel = mpu6050_get_accelerometer(&mpu6050);
    mpu6050_vectorf_t *gyro = mpu6050_get_gyroscope(&mpu6050);

    // Print all the measurements
    printf("Accelerometer: %f, %f, %f - Gyroscope: %f, %f, %f\n", accel->x,
           accel->y, accel->z, gyro->x, gyro->y, gyro->z);
  }
}

int main() {

  stdio_init_all();
  setup_default_uart();

  printf("Hello World!\n");

  multicore_launch_core1(main_core1);

  while (1) {
    // printf("Hello World1!\n");

    sleep_ms(1000);
  }

  return 0;
}
