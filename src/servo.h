#ifndef SERVO_PWM_HEADER
#define SERVO_PWM_HEADER

#include <stdint.h>

typedef struct {
  uint16_t pin;
  uint16_t pwm;
  float range;
} servo_h;

void servo_init(servo_h *servo, uint16_t pin);

void servo_set_pos(servo_h *servo, float range);

#endif
