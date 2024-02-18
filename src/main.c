#include "crsf_pio_rx.h"
#include "haw/MPU6050.h"
#include "madgwick.h"
#include "mpu_6050_init.h"
#include "servo.h"

#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"

#include <stdio.h>

#define PIO_RX_PIN 22
#define SERIAL_BAUD 115200

servo_h m_left_front, m_left_rear, m_right_front, m_right_rear;

void main_core1() {

  mpu6050_t mpu6050 = mpu_6050_custom_init();

  float khz = 0.0;
  absolute_time_t last_updated_start = 0;

  float est_roll = 0.f, est_pitch = 0.f, est_yaw = 0.f;
  float acc_n_x, acc_n_y, acc_n_z;
  float gyro_n_x, gyro_n_y, gyro_n_z;

  mpu6050_vectorf_t *acc, *gyro;

  mpu6050_calibrate_gyro(&mpu6050, 255);

  while (1) {
    absolute_time_t start = get_absolute_time();

    // Fetch all data from the sensor | I2C is only used here
    mpu6050_event(&mpu6050);

    // Pointers to float vectors with all the results
    acc = mpu6050_get_accelerometer(&mpu6050);
    gyro = mpu6050_get_gyroscope(&mpu6050);

    acc_n_x = acc->x;
    acc_n_y = acc->y;
    acc_n_z = acc->z;

    gyro_n_x = (PI / 180.f) * gyro->x;
    gyro_n_y = (PI / 180.f) * gyro->y;
    gyro_n_z = (PI / 180.f) * gyro->z;

    // filter should be updated with rate of DELTA_T
    {
      int64_t delta = absolute_time_diff_us(last_updated_start, get_absolute_time());
      if (delta < DELTA_T_US) {
        sleep_us(DELTA_T_US - delta);
      }
      last_updated_start = get_absolute_time();
    }
    imu_filter(acc_n_x, acc_n_y, acc_n_z, gyro_n_x, gyro_n_y, gyro_n_z);
    eulerAngles(q_est, &est_roll, &est_pitch, &est_yaw);

    // Print all the measurements
    // printf("Acc: %.2f, %.2f, %.2f\tGyro: %.2f, %.2f, %.2f\n", acc->x, acc->y, acc->z, gyro->x,
    //        gyro->y, gyro->z);
    // est_roll += DELTA_T * gyro->x;
    // est_pitch += DELTA_T * gyro->y;
    // est_yaw += DELTA_T * gyro->z;
    // printQuaternion(q_est);
    // printf("kHz: %.2f\tPitch: %.2f\tRoll: %.2f\tYaw: %.2f\n", khz, est_pitch, est_roll, est_yaw);
    khz = (1.0 / (0.000001 * (float)absolute_time_diff_us(start, get_absolute_time()))) / 1000.0;
  }
}

int main() {

  stdio_init_all();
  setup_default_uart();

  printf("Hello World!\n");

  multicore_launch_core1(main_core1);

  crsf_init(PIO_RX_PIN, SERIAL_BAUD);

  crsf_packet rc_packet;

  servo_init(&m_left_front, 20);
  servo_init(&m_left_rear, 21);
  servo_init(&m_right_front, 19);
  servo_init(&m_right_rear, 18);

  sleep_ms(5000);

  servo_set_pos(&m_left_front, 100);
  servo_set_pos(&m_right_front, 100);
  servo_set_pos(&m_left_rear, 100);
  servo_set_pos(&m_right_rear, 100);

  sleep_ms(5000);

  bool direction = true;

  while (1) {

    float step = direction ? 5.0 : -5.0;

    servo_set_pos(&m_left_front, m_left_front.range + step);
    if (m_left_front.range >= 100.0) {
      direction = false;
    }
    if (m_left_front.range <= 0.0) {
      direction = true;
    }

    sleep_ms(200);

    // crsf_get_packet(&rc_packet);
    //
    // if (rc_packet.crc_ok) {
    //   for (uint8_t i = 0; i < 8; i++) {
    //     printf("%i\t", rc_packet.channels[i]);
    //   }
    //   printf("\n");
    // }
  }

  return 0;
}
