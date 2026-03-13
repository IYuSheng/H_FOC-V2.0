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

// XY笛卡尔PD控制参数
#define FOC_XY_KP_X                        20.5f
#define FOC_XY_KD_X                        0.00f
#define FOC_XY_KP_Y                        20.5f
#define FOC_XY_KD_Y                        0.00f

#define FOC_XY_FX_LIMIT                    2.0f   // N
#define FOC_XY_FY_LIMIT                    2.0f   // N

#define FOC_JOINT1_KT                      0.060f  // N*m/A
#define FOC_JOINT2_KT                      0.060f  // N*m/A
#define FOC_JOINT1_IQ_LIMIT                3.0f    // A
#define FOC_JOINT2_IQ_LIMIT                3.0f    // A

typedef struct
{
    float x, y;
    float vx, vy;
    float fx, fy;
    float tau1, tau2;
    float iq1, iq2;
} robot_cart_ctrl_out_t;

extern robot_cart_ctrl_out_t cart_out;

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
/**
 * @brief XY位置PD控制（D项=期望速度-当前速度）并输出双关节电流
 * @param x_ref 期望X位置(m)
 * @param y_ref 期望Y位置(m)
 * @param vx_ref 期望X速度(m/s)
 * @param vy_ref 期望Y速度(m/s)
 * @param out 输出控制中间量与iq结果
 */
void foc_xy_pd_control(float x_ref, float y_ref, float vx_ref, float vy_ref, robot_cart_ctrl_out_t *out);

/**
 * @brief 2-link IK: solve joint mechanical angles (deg) from target (x,y)
 * @param x_ref target x (m)
 * @param y_ref target y (m)
 * @param joint1_target_deg out: joint1 mechanical angle (deg)
 * @param joint2_target_deg out: joint2 mechanical angle (deg)
 * @return 1=success, 0=unreachable or invalid args
 */
uint8_t foc_xy_inverse_kinematics(float x_ref, float y_ref, float *joint1_target_deg, float *joint2_target_deg);
#endif /* __FOC_CONTROL_H */
