#ifndef __FOC_CONVERSION_H
#define __FOC_CONVERSION_H

#include "Config.h"
#include "adc.h"
#include "foc_encoder.h"
#include "arm_math.h"

// 两相静止坐标系结构体 (Alpha-Beta)
typedef struct
{
 // 输出dq转换
 float alpha;
 float beta;
 // 输入abc转换
 float alpha_i;
 float beta_i;
 // 输入abc转换
 float alpha_v;
 float beta_v;
} AlphaBetaTypeDef;

// SVPWM结构体参数定义
typedef struct
{
 float32_t T0;
 float32_t T1;
 float32_t T2;
 uint8_t sector;
 uint16_t pwm_a;
 uint16_t pwm_b;
 uint16_t pwm_c;
} SVPWM_t;

// FOC控制参数结构体
typedef struct
{
 float32_t out_d;            // 输出D轴电压
 float32_t out_q;            // 输出Q轴电压
 float32_t target_d;         // 目标D轴电流
 float32_t target_q;         // 目标Q轴电流
 float32_t angle;            // 电角度(弧度)
 float32_t speed;            // 速度（rad/s）
 float32_t speed_rpm;        // 速度（rpm）
 float32_t target_speed;     // 目标速度(rpm)
 float32_t target_position;  // 目标位置（弧度）
 struct
 {
   float32_t current_d;     // 实际D轴电流
   float32_t current_q;     // 实际Q轴电流
 }abc_dq;  // 经采集变换后的实际dq轴电流
} foc_control_t;

// PI控制器结构体
typedef struct {
   float32_t kp;           // 比例增益
   float32_t ki;           // 积分增益
   float32_t kd;           // 微分增益
   float32_t integral;     // 积分项
   float32_t integral_limit; // 积分限幅
   float32_t error;        // 当前误差
   float32_t last_error;   // 上次误差
   float32_t output;       // 输出值
   float32_t target;       // 目标位置
   float32_t current;      // 当前位置
} pi_t;

// FOC变换相关函数声明
/**
* @brief 电角度归一化（映射到0~2π范围）
* @param angle 输入电角度（rad，范围无限制）
* @return 归一化后电角度（rad，0~2π）
*/
extern inline float32_t angle_normalize(float32_t angle);

/**
 * @brief 电角度归一化（映射到0~360范围）
 * @param angle 输入电角度（°，范围无限制）
 * @return 归一化后电角度（°，0~360）
 */
extern inline float32_t angle_normalize_360(float32_t angle);

/**
* @brief SVPWM通用扇区判断函数
* @param alpha_beta αβ坐标系下的电压指针
* @return 扇区编号（1~6，对应0~60°~360°）
*/
extern inline uint8_t svpwm_sector_calc(AlphaBetaTypeDef *alpha_beta);

/**
* @brief SVPWM基本矢量作用时间计算
* @param alpha_beta αβ坐标系下的电压指针
* @param svpwm SVPWM结构体指针
* @param vdc 母线电压（V）
*/
extern inline void svpwm_calc_times(AlphaBetaTypeDef *alpha_beta, SVPWM_t *svpwm, float32_t vdc);

/**
* @brief SVPWM计算三相导通时间并转换为比较值
* @param svpwm SVPWM结构体指针
*/
extern inline void svpwm_duty_calc(SVPWM_t *svpwm);

/**
* @brief 反Park变换
* @param foc_ctrl FOC控制结构体指针
* @param alpha_beta αβ坐标系下的电压指针
* @param angle 电角度(弧度)
*/
extern inline void inv_park_transform_f32(foc_control_t *foc_ctrl, AlphaBetaTypeDef *alpha_beta, float32_t angle);

/**
* @brief 将三相电压电流转换为αβ坐标系下的值
* @param abc_i 三相电流指针
* @param abc_v 三相电压指针
* @param alpha_beta 输出的αβ轴值
*/
extern inline void clark_transform(void *abc_i, void *abc_v, AlphaBetaTypeDef *alpha_beta);

/**
* @brief 将αβ转换为dq坐标系下的电流值
* @param alpha_beta αβ坐标系下的电流值
* @param foc_ctrl FOC控制结构体指针
*/
extern inline void park_transform(AlphaBetaTypeDef *alpha_beta, foc_control_t *foc_ctrl);

/**
* @brief 将三相电流转换为dq坐标系下的电流值
* @param current_abc_ptr 三相电流值指针 (ia, ib, ic)
* @param foc_ctrl FOC控制结构体指针
* @param angle 电角度(弧度)
*/
extern inline void abc_to_dq_current(void *current_abc_ptr, foc_control_t *foc_ctrl, float angle);

/**
* @brief D轴电流环PID计算
* @param target_id 目标D轴电流
* @param actual_id 实际D轴电流
* @return D轴电压输出
*/
extern inline float32_t foc_id_pid_calculate(float32_t target_id, float32_t actual_id);

/**
* @brief Q轴电流环PID计算
* @param target_iq 目标Q轴电流
* @param actual_iq 实际Q轴电流
* @return Q轴电压输出
*/
extern inline float32_t foc_iq_pid_calculate(float32_t target_iq, float32_t actual_iq);

/**
* @brief 速度环PID计算
* @param target_speed 目标速度(RPM)
* @param actual_speed 实际速度(RPM)
* @return 输出值(Q轴电流)
*/
extern inline float32_t foc_speed_pid_calculate(float32_t target_speed, float32_t actual_speed);

/**
* @brief 位置环PI控制器计算
* @param target_position 目标位置（弧度）
* @param current_position 当前位置（弧度）
* @return PI控制器输出（速度指令，rad/s）
*/
extern inline float32_t foc_position_pid_calculate(float32_t target_position, float32_t current_position);

#endif /* __FOC_CONVERSION_H */
