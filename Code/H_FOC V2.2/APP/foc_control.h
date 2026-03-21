#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H

#include "arm_math.h"
#include "foc_setting.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "dac.h"
#include "Config.h"
#include "foc_conversion.h"
#include "foc_encoder.h"
#include "foc_prase.h"
#include "foc_vibration_sweep.h"
#include "foc_Parameteridentifikation.h"
#include "foc_cogging_compensation.h"
#include "foc_trajectory_planning.h"
#include "foc_sensorless.h"
#include "foc_filter.h"
#include <math.h>

#define FOC_SPEED_CONTROL_ENABLE            0   // 速度环使能
#define FOC_POSITION_CONTROL_ENABLE         0   // 位置环使能
#define FOC_MIT_CONTROL_ENABLE              1   // MIT控制使能
#define FOC_PARAMETER_IDENTIFICATION_ENABLE 0   // 电机参数辨识使能
#define FOC_COGGING_COMPENSATION_ENABLE     0   // 齿槽转矩补偿使能
#define FLUX_OBSERVER_ENABLE                0   // 磁链观测器使能
#define FOC_TEST_ENABLE                     0   // FOC扫频测试使能
#define HFI_ENABLE                          0   // 高频注入使能
#define HFI_STANDALONE_MODE                 0   // 1:纯HFI 0:HFI+观测器混合
#define FOC_RESONANCE_ENABLE                0   // 阶跃测试使能
#define FOC_SHAPER_ENABLE                   1   // 整形输入使能

// 双关节平面运动学参数
#define FOC_SECOND_JOINT_ID                 1
#define FOC_LINK1_LENGTH                    0.07748f   // 第一关节连杆长度(m)
#define FOC_LINK2_LENGTH                    0.0625f   // 第二关节连杆长度(m)
#define FOC_LINK1_ZERO                      -53.7f  // 第一关节零位角度
#define FOC_LINK2_ZERO                      172.5f  // 第二关节零位角度

typedef enum {
    SENSORLESS_STATE_HFI,
    SENSORLESS_STATE_MIX,
    SENSORLESS_STATE_FLUX,
} sensorless_state_t;

/**
 * @brief FOC初始化，锁定电机至零位并校零编码器
 */
void foc_start_init(void);

/**
 * @brief FOC内环电流环外部调用接口
 */
void foc_current_in_control(void);

/**
 * @brief FOC控制主函数
 */
void foc_control(void);

/**
 * @brief FOC打印调试信息
 */
void foc_debug(void);

/**
 * @brief 计算双关节末端相对主轴平面坐标
 * @param x_out 末端X坐标输出(单位同连杆长度)
 * @param y_out 末端Y坐标输出(单位同连杆长度)
 */
void foc_calc_end_effector_xy(float *x_out, float *y_out, float *x_out_speed, float *y_out_speed);

#endif /* __FOC_CONTROL_H */
