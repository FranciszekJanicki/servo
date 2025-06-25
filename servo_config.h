#ifndef SERVO_SERVO_CONFIG_H
#define SERVO_SERVO_CONFIG_H

#include <stdint.h>

typedef float float32_t;

typedef enum {
  SERVO_ERR_OK = 0,
  SERVO_ERR_FAIL = 1 << 0,
  SERVO_ERR_NULL = 1 << 1,
} servo_err_t;

typedef struct {
  float32_t min_angle;
  float32_t max_angle;
  uint16_t min_compare;
  uint16_t max_compare;
} servo_config_t;

typedef struct {
  void *pwm_user;
  servo_err_t (*pwm_initialize)(void *);
  servo_err_t (*pwm_deinitialize)(void *);
  servo_err_t (*pwm_set_compare)(void *, uint16_t);
} servo_interface_t;

#endif // SERVO_SERVO_CONFIG_H