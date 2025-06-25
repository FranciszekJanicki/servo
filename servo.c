#include "servo.h"
#include "servo_config.h"
#include <assert.h>
#include <string.h>

static servo_err_t servo_pwm_initialize(servo_t const* servo)
{
    return servo->interface.pwm_initialize
               ? servo->interface.pwm_initialize(servo->interface.pwm_user)
               : SERVO_ERR_NULL;
}

static servo_err_t servo_pwm_deinitialize(servo_t const* servo)
{
    return servo->interface.pwm_deinitialize
               ? servo->interface.pwm_deinitialize(servo->interface.pwm_user)
               : SERVO_ERR_NULL;
}

static servo_err_t servo_pwm_set_compare(servo_t const* servo, uint16_t compare)
{
    return servo->interface.pwm_set_compare
               ? servo->interface.pwm_set_compare(servo->interface.pwm_user, compare)
               : SERVO_ERR_NULL;
}

static float32_t servo_clamp_angle(servo_t const* servo, float32_t angle)
{
    if (angle > servo->config.max_angle) {
        angle = servo->config.max_angle;
    } else if (angle < servo->config.min_angle) {
        angle = servo->config.min_angle;
    }

    return angle;
}

static uint16_t servo_angle_to_compare(servo_t const* servo, float32_t angle)
{
    return (uint16_t)((angle - servo->config.min_angle) *
                          (float32_t)(servo->config.max_compare - servo->config.min_compare) /
                          (servo->config.max_angle - servo->config.min_angle) +
                      (float32_t)(servo->config.min_compare));
}

servo_err_t servo_initialize(servo_t* servo,
                             servo_config_t const* config,
                             servo_interface_t const* interface)
{
    assert(servo && config && interface);

    memset(servo, 0, sizeof(*servo));
    memcpy(&servo->config, config, sizeof(*config));
    memcpy(&servo->interface, interface, sizeof(*interface));

    return servo_pwm_initialize(servo);
}

servo_err_t servo_deinitialize(servo_t* servo)
{
    assert(servo);

    servo_err_t err = servo_pwm_deinitialize(servo);

    memset(servo, 0, sizeof(*servo));

    return err;
}

servo_err_t servo_set_angle(servo_t const* servo, float32_t angle)
{
    assert(servo);

    angle = servo_clamp_angle(servo, angle);
    uint16_t compare = servo_angle_to_compare(servo, angle);

    return servo_pwm_set_compare(servo, compare);
}

servo_err_t servo_set_angle_max(servo_t const* servo)
{
    assert(servo);

    return servo_set_angle(servo, servo->config.max_angle);
}

servo_err_t servo_set_angle_min(servo_t const* servo)
{
    assert(servo);

    return servo_set_angle(servo, servo->config.min_angle);
}