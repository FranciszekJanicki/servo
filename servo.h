#ifndef SERVO_SERVO_H
#define SERVO_SERVO_H

#include "servo_config.h"

typedef struct {
    servo_config_t config;
    servo_interface_t interface;
} servo_t;

servo_err_t servo_initialize(servo_t* servo, servo_config_t const* config, servo_interface_t const* interface);
servo_err_t servo_deinitialize(servo_t* servo);

servo_err_t servo_set_angle(servo_t const* servo, float32_t angle);
servo_err_t servo_set_angle_max(servo_t const* servo);
servo_err_t servo_set_angle_min(servo_t const* servo);

#endif // SERVO_SERVO_H