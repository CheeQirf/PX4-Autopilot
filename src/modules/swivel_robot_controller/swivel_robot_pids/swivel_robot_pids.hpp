#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <cmath>
#include <drivers/drv_hrt.h>
using namespace std;

float calculate_single_pid_output(float kp, float ki, float kd, float error,
                                   float *previous_error, float *integral_error,
                                   float integral_max, float dt);

void calculate_four_wheel_steer_drive_powers_advanced(
    float desired_throttle_speed,
    float current_body_vx, float current_body_vy,
    float current_global_yaw, float desired_global_yaw,
    hrt_abstime current_time_us, float wheel_steer_angle,
    float wheel_powers[4]);