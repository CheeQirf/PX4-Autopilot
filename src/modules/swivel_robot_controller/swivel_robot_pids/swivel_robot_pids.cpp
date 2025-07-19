#include "swivel_robot_pids.hpp"

// 假设 hrt_abstime 已经定义，例如：
// typedef uint64_t hrt_abstime;

// --- PID 控制器参数 (这些参数需要根据你的机器人进行调优) ---
// 前进速度PID (Vx PID)
const float PID_VX_KP = 0.5f;   // 比例增益
const float PID_VX_KI = 0.1f;   // 积分增益 (防止积分饱和)
const float PID_VX_KD = 0.01f;  // 微分增益
const float PID_VX_INTEGRAL_MAX = 0.2f; // 积分项限幅

// 偏航角PID (Yaw PID)
const float PID_YAW_KP = 1.5f;  // 比例增益
const float PID_YAW_KI = 0.0f;  // 积分增益 (通常直线行驶时，角度积分设为0或很小)
const float PID_YAW_KD = 0.1f;  // 微分增益
const float PID_YAW_INTEGRAL_MAX = 0.1f; // 积分项限幅

// 体轴侧向速度PID (Vy PID)
const float PID_VY_KP = 1.0f;   // 比例增益 (调优重点，消除侧滑)
const float PID_VY_KI = 0.05f;  // 积分增益 (调优重点，消除侧滑的稳态误差)
const float PID_VY_KD = 0.02f;  // 微分增益 (调优重点，减小侧滑的震荡)
const float PID_VY_INTEGRAL_MAX = 0.1f; // 积分项限幅

// --- 机器人物理参数 (请根据你的小车实际测量值设置) ---
const float ROBOT_HALF_WIDTH = 0.2f; // 轮子左右中心线的距离的一半 (米)

// !!! 新增物理参数：轮子到车体中心的前后距离的一半 !!!
// 这是进行通用运动学逆解所必需的。
// 请根据你的机器人实际测量值设置此常量。
// 例如：如果前后轮轴相距0.4米，则设置为0.2米。
const float ROBOT_HALF_LENGTH = 0.2f; // 假设前后轴距的一半 (米)

// --- PID 状态变量 (使用 static 来保持状态，函数每次调用时会记住上次的值) ---
// Vx PID 状态
static float g_vx_previous_error = 0.0f;
static float g_vx_integral_error = 0.0f;
static hrt_abstime g_last_vx_timestamp_us = 0;

// Yaw PID 状态
static float g_yaw_previous_error = 0.0f;
static float g_yaw_integral_error = 0.0f;
static hrt_abstime g_last_yaw_timestamp_us = 0;

// Vy PID 状态
static float g_vy_previous_error = 0.0f;
static float g_vy_integral_error = 0.0f;
static hrt_abstime g_last_vy_timestamp_us = 0;


/**
 * @brief 辅助函数：计算单个PID控制器的输出。
 * 使用 if-else 替代 std::max/min 进行限幅。
 *
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param error 当前误差
 * @param previous_error 指向前一个时刻误差的指针 (会更新)
 * @param integral_error 指向积分误差累积值的指针 (会更新)
 * @param integral_max 积分项限幅
 * @param dt 时间步长 (秒)
 * @return PID输出值
 */
float calculate_single_pid_output(float kp, float ki, float kd, float error,
                                   float *previous_error, float *integral_error,
                                   float integral_max, float dt) {
    if (dt <= 0.0f) {
        // 避免除以零或无效时间步长
        return 0.0f;
    }

    // 积分项计算
    *integral_error += error * dt;

    // 积分项限幅 (使用 if-else)
    if (*integral_error > integral_max) {
        *integral_error = integral_max;
    } else if (*integral_error < -integral_max) {
        *integral_error = -integral_max;
    }

    // 微分项计算
    float derivative_error = (error - *previous_error) / dt;
    *previous_error = error; // 更新上一次的误差

    // PID 输出 = 比例项 + 积分项 + 微分项
    return kp * error + ki * (*integral_error) + kd * derivative_error;
}


/**
 * @brief 控制四轮独立转向（但转向角相同）驱动的机器人，使其在任意轮子转向角下走直线。
 * 同时保持车身朝向期望的全局偏航角，并消除侧滑。
 *
 * @param desired_throttle_speed 期望在轮子行进方向上的速度 (m/s)。
 * @param current_body_vx 机器人当前在小车体轴X方向上的速度 (m/s)。
 * @param current_body_vy 机器人当前在小车体轴Y方向上的速度 (m/s)。
 * @param current_global_yaw 机器人当前在地面坐标系下的偏航角 (rad)。
 * @param desired_global_yaw 机器人期望在地面坐标系下的偏航角 (rad)。
 * @param current_time_us 当前时间戳（微秒），用于计算PID的dt。
 * @param wheel_steer_angle 轮子的当前转向角 theta (rad)，相对于小车体轴X轴的夹角。
 * 所有轮子转向角必须相同。
 * @param wheel_powers 输出参数，指向一个包含4个浮点数的数组：[左前, 右前, 左后, 右后]。
 * 动力值范围为 0.0 到 1.0 (0.0为停止，1.0为最大前进动力，无后退)。
 */
void calculate_four_wheel_steer_drive_powers_advanced(
    float desired_throttle_speed,
    float current_body_vx, float current_body_vy,
    float current_global_yaw, float desired_global_yaw,
    hrt_abstime current_time_us, float wheel_steer_angle,
    float wheel_powers[4]) {

    // --- 1. 计算时间步长 (dt) ---
    // 将微秒转换为秒
    float vx_dt = (g_last_vx_timestamp_us == 0) ? 0.001f : (float)(current_time_us - g_last_vx_timestamp_us) / 1000000.0f;
    g_last_vx_timestamp_us = current_time_us;

    float vy_dt = (g_last_vy_timestamp_us == 0) ? 0.001f : (float)(current_time_us - g_last_vy_timestamp_us) / 1000000.0f;
    g_last_vy_timestamp_us = current_time_us;
    
    float yaw_dt = (g_last_yaw_timestamp_us == 0) ? 0.001f : (float)(current_time_us - g_last_yaw_timestamp_us) / 1000000.0f;
    g_last_yaw_timestamp_us = current_time_us;


    // --- 2. PID A: Yaw PID (偏航角PID) ---
    // 目标：控制机器人全局偏航角，使其达到 desired_global_yaw
    float yaw_error = desired_global_yaw - current_global_yaw;
    // 将偏航角误差归一化到 [-PI, PI] 范围，处理角度的周期性
    while (yaw_error > M_PI_F) yaw_error -= 2.f * M_PI_F;
    while (yaw_error < -M_PI_F) yaw_error += 2.f * M_PI_F;
    
    float desired_body_yaw_rate = calculate_single_pid_output(
        PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, yaw_error,
        &g_yaw_previous_error, &g_yaw_integral_error,
        PID_YAW_INTEGRAL_MAX, yaw_dt
    );
    // 期望体轴角速度限幅 (使用 if-else)
    if (desired_body_yaw_rate > 2.0f) {
        desired_body_yaw_rate = 2.0f;
    } else if (desired_body_yaw_rate < -2.0f) {
        desired_body_yaw_rate = -2.0f;
    }


    // --- 3. PID B: Vy PID (体轴侧向速度PID) ---
    // 目标：控制机器人在体轴Y方向上的速度为0，消除侧滑
    float vy_error = 0.0f - current_body_vy; // 目标体轴vy为0 (实现直线行走的关键)
    float desired_body_vy_compensation = calculate_single_pid_output(
        PID_VY_KP, PID_VY_KI, PID_VY_KD, vy_error,
        &g_vy_previous_error, &g_vy_integral_error,
        PID_VY_INTEGRAL_MAX, vy_dt
    );
    // 侧向补偿速度/力限幅 (使用 if-else)
    if (desired_body_vy_compensation > 0.5f) {
        desired_body_vy_compensation = 0.5f;
    } else if (desired_body_vy_compensation < -0.5f) {
        desired_body_vy_compensation = -0.5f;
    }


    // --- 4. PID C: Vx_wheel PID (轮子前进速度PID) ---
    // 目标：控制轮子在自身前进方向的速度，使其与 desired_throttle_speed 匹配
    // 计算小车当前整体速度在“轮子转向方向”上的投影
    float current_wheel_forward_speed = current_body_vx * cosf(wheel_steer_angle) + current_body_vy * sinf(wheel_steer_angle);

    float wheel_speed_error = desired_throttle_speed - current_wheel_forward_speed;
    float total_drive_power = calculate_single_pid_output(
        PID_VX_KP, PID_VX_KI, PID_VX_KD, wheel_speed_error,
        &g_vx_previous_error, &g_vx_integral_error,
        PID_VX_INTEGRAL_MAX, vx_dt
    );
    // 确保总驱动力在 [0.0, 1.0] 范围内 (0.0为停止，1.0为最大前进动力，无后退) (使用 if-else)
    if (total_drive_power > 1.0f) {
        total_drive_power = 1.0f;
    } else if (total_drive_power < 0.0f) {
        total_drive_power = 0.0f;
    }


    // --- 5. 运动学逆解与动力分配 (!!! 这是需要修改的部分 !!!) ---
    // 将 PID 输出转换为每个轮子的具体驱动动力

    // 轮子在体轴坐标系下的固定位置 (X, Y)
    // X坐标：沿着车头方向 (前进方向为正)
    // Y坐标：沿着车体侧向 (车体右侧为负Y，左侧为正Y)

    // 根据你的轮子编号顺序:
    // 0: 左前 (LF)
    // 1: 右前 (RF)
    // 2: 右后 (RR)
    // 3: 左后 (LR)

    // 轮子在车体坐标系中的X坐标 (前后)
    const float wheel_x_coords[4] = {
        ROBOT_HALF_LENGTH,   // 轮子0 (左前)
        ROBOT_HALF_LENGTH,   // 轮子1 (右前)
        -ROBOT_HALF_LENGTH,  // 轮子2 (右后)
        -ROBOT_HALF_LENGTH   // 轮子3 (左后)
    };

    // 轮子在车体坐标系中的Y坐标 (左右)
    const float wheel_y_coords[4] = {
        ROBOT_HALF_WIDTH,    // 轮子0 (左前)
        -ROBOT_HALF_WIDTH,   // 轮子1 (右前)
        -ROBOT_HALF_WIDTH,   // 轮子2 (右后)
        ROBOT_HALF_WIDTH     // 轮子3 (左后)
    };

    // 1. 将总的驱动力 (total_drive_power) 转换为机器人期望的体轴线速度分量
    // total_drive_power 是 Vx PID 的输出，表示期望在轮子转向方向上的速度大小
    float desired_body_vx_from_throttle = total_drive_power * cosf(wheel_steer_angle);
    float desired_body_vy_from_throttle = total_drive_power * sinf(wheel_steer_angle);

    // 2. 计算最终期望的机器人体轴Y速度，融合了油门导致的Y分量和侧滑补偿
    // desired_body_vy_compensation 是 Vy PID 的输出，用于消除侧滑
    float final_desired_body_vy = desired_body_vy_from_throttle + desired_body_vy_compensation;

    // 3. 为每个轮子计算其期望的切线速度（对应的动力）
    for (int i = 0; i < 4; ++i) {
        // 计算轮子在体轴坐标系下的期望速度分量
        // 轮子的期望X速度分量 = 机器人整体X速度 - (机器人角速度 * 轮子Y坐标)
        float wheel_vx_in_body_frame = desired_body_vx_from_throttle - desired_body_yaw_rate * wheel_y_coords[i];

        // 轮子的期望Y速度分量 = 机器人整体Y速度 + (机器人角速度 * 轮子X坐标)
        float wheel_vy_in_body_frame = final_desired_body_vy + desired_body_yaw_rate * wheel_x_coords[i];

        // 将轮子在体轴坐标系下的合成期望速度矢量
        // 投影到轮子当前转向的方向 (wheel_steer_angle)
        // 这就是该轮子实际需要达到的切线速度（或对应的动力）
        float wheel_final_power =
            wheel_vx_in_body_frame * cosf(wheel_steer_angle) +
            wheel_vy_in_body_frame * sinf(wheel_steer_angle);

        // 4. 限幅并赋值给轮子动力输出数组 (wheel_powers)
        // 轮子编号: 0:LF, 1:RF, 2:RR, 3:LR
        if (wheel_final_power > 1.0f) {
            wheel_powers[i] = 1.0f;
        } else if (wheel_final_power < 0.0f) {
            wheel_powers[i] = 0.0f; // 限制为只前进，无后退。如需后退请允许负值
        } else {
            wheel_powers[i] = wheel_final_power;
        }
    }
}