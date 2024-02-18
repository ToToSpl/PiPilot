#include "servo.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/platform.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include <stdio.h>

#define PWM_INIT 900
#define PWM_LOW 1000
#define PWM_HIGH 2000

float clockDiv = 64;
float wrap = 39062;

inline float pwm_to_range(uint16_t pwm) {
  return (float)(pwm - PWM_LOW) / (float)(PWM_HIGH - PWM_LOW);
}

inline uint16_t range_to_pwm(float range) {
  float r = MIN(range, 100.0);
  r = MAX(r, 0.0);
  return PWM_LOW + (uint16_t)(range * ((float)(PWM_HIGH - PWM_LOW) / 100.0));
}

void servo_init(servo_h *servo, uint16_t pin) {

  servo->pin = pin;
  servo->pwm = PWM_LOW;
  servo->range = pwm_to_range(PWM_LOW);

  gpio_set_function(servo->pin, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(servo->pin);

  pwm_config config = pwm_get_default_config();

  uint64_t clockspeed = clock_get_hz(5);
  clockDiv = 64;
  wrap = 39062;

  while (clockspeed / clockDiv / 50 > 65535 && clockDiv < 256)
    clockDiv += 64;
  wrap = clockspeed / clockDiv / 50;

  pwm_config_set_clkdiv(&config, clockDiv);
  pwm_config_set_wrap(&config, wrap);

  pwm_init(slice_num, &config, true);

  pwm_set_gpio_level(servo->pin, (PWM_INIT / 20000.f) * wrap);
}

void servo_set_pos(servo_h *servo, float range) {

  servo->range = range;
  servo->pwm = range_to_pwm(servo->range);

  pwm_set_gpio_level(servo->pin, (servo->pwm / 20000.f) * wrap);
}
