#include "foc_setting.h"

extern pi_t id_pid;
extern pi_t iq_pid;
extern pi_t speed_pid;
extern pi_t position_pid;

/**
 * @brief 电流环PID控制器初始化
 * @param kp_d D轴比例系数
 * @param kp_q Q轴比例系数
 * @param ki 积分系数
 * @param ki_limit 积分限幅值
 */
void foc_current_pi_init(float32_t kp_d, float32_t kp_q, float32_t ki, float32_t ki_limit)
{
    // 初始化D轴电流环PI参数
    id_pid.kp = kp_d;
    id_pid.ki = ki;
    id_pid.integral_limit = ki_limit;
    id_pid.integral = 0.0f;
    
    // 初始化Q轴电流环PI参数
    iq_pid.kp = kp_q;
    iq_pid.ki = ki;
    iq_pid.integral_limit = ki_limit;
    iq_pid.integral = 0.0f;
}

/**
 * @brief 速度环PI控制器初始化
 * @param kp 比例系数
 * @param ki 积分系数
 * @param ki_limit 积分限幅值
 */
void foc_speed_pi_init(float32_t kp, float32_t ki, float32_t ki_limit)
{
    // 初始化速度环PI参数
    speed_pid.kp = kp;
    speed_pid.ki = ki;
    speed_pid.integral_limit = ki_limit;
    speed_pid.integral = 0.0f;
}

/**
 * @brief 初始化位置环PI控制器
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param integral_limit 积分限幅值
 */
void foc_position_pi_init(float32_t kp, float32_t ki, float32_t kd, float32_t integral_limit)
{
    position_pid.kp = kp;
    position_pid.ki = ki;
    position_pid.kd = kd;
    position_pid.integral_limit = integral_limit;
    position_pid.integral = 0.0f;
    position_pid.error = 0.0f;
    position_pid.output = 0.0f;
    position_pid.target = 0.0f;
    position_pid.current = 0.0f;
}
