/**
 * @file foc_sensorless.h
 * @brief 非线性磁链观测器（无传感器FOC）
 */

#ifndef FOC_SENSORLESS_H
#define FOC_SENSORLESS_H

#include <math.h>
#include "foc_conversion.h"
#include "config.h"

typedef struct {
    float x1, x2;           // 观测器状态
    float cos_theta;        // PLL输出：cos(θ_hat)
    float sin_theta;        // PLL输出：sin(θ_hat)
    float omega_filt;       // 滤波后的角速度
    // PLL内部状态
    float theta_hat;        // 估计角度
    float omega_integ;      // 速度积分（PI的I项）
} FluxObserver_t;

extern FluxObserver_t g_flux_obs;

// 电机参数
#define FLUX_PHI_M      MOTOR_FLUX_LINKAGE      // 永磁体磁链（Wb）
#define _1_FLUX_PHI_M    1.0f / MOTOR_FLUX_LINKAGE
#define FLUX_PHI_M_2    MOTOR_FLUX_LINKAGE * MOTOR_FLUX_LINKAGE
#define FLUX_GAMMA      2700000.0f              // 观测器增益（越大收敛越快，但噪声敏感）
#define FLUX_GAMMA_K    0.5f * FLUX_GAMMA
#define FLUX_L_S        MOTOR_INDUCTANCE        // 定子电感（H）
#define FLUX_R_S        MOTOR_RESISTANCE        // 定子电阻（Ω）

// PLL参数（根据带宽设置）
#define PLL_BANDWIDTH_HZ    1000.0f     // PLL带宽
#define PLL_KP              (2.0f * 0.707f * 6.2831853f * PLL_BANDWIDTH_HZ)  // 2*zeta*wn
#define PLL_KI              ((6.2831853f * PLL_BANDWIDTH_HZ) * (6.2831853f * PLL_BANDWIDTH_HZ))  // wn^2

void flux_observer_init(FluxObserver_t *obs);
void flux_observer_update(FluxObserver_t *obs, float u_alpha, float u_beta, 
                          float i_alpha, float i_beta, float Ts);

/* ---------------------HFI高频注入观测器---------------------- */

typedef struct {
    // 注入信号参数
    float u_inject;         // 注入电压幅值(V)
    float u_inject_d;       // 当前d轴注入电压（含极性）
    uint8_t inject_flag;    // 注入极性标志(0:正, 1:负)
    uint8_t init_done;      // 初始化完成标志
    
    // 电流采样历史值（用于提取高频分量）
    float i_alpha_last[2];  // [n-1, n-2]时刻的alpha电流
    float i_beta_last[2];   // [n-1, n-2]时刻的beta电流
    
    // 高频电流分量
    float i_alpha_h;        // alpha轴高频电流
    float i_beta_h;         // beta轴高频电流
    float i_alpha_h_last;   // 上一周期高频电流
    float i_beta_h_last;    // 上一周期高频电流
    
    // 电流包络（解调后）
    float i_env_alpha;      // alpha轴电流包络
    float i_env_beta;       // beta轴电流包络
    
    // 基频电流（用于电流环反馈）
    float i_d_fund;         // d轴基频电流
    float i_q_fund;         // q轴基频电流
    
    // 极性辨识相关
    uint16_t nsd_count;     // NSD计数器
    float nsd_target_d;   // NSD阶段的d轴目标电流（A）
    float sum_pos;          // 正向脉冲累积
    float sum_neg;          // 负向脉冲累积
    uint8_t polarity_determined; // 极性已确定标志

    uint8_t state;                  // HFI状态
    uint32_t converge_cnt;        // 收敛等待计数器
    
} HFI_Observer_t;

typedef struct {
    // PLL状态
    float theta_est;        // 估计电角度（弧度）
    float omega_est;        // 估计电角速度
    float omega_est_filt;   // 滤波后的角速度
    
    // PLL控制器参数
    float kp;               // 比例增益
    float ki;               // 积分增益
    
    // 中间变量
    float error;            // 角度误差
    float p_term;           // 比例项
    float i_term;           // 积分项
    
} HFI_PLL_t;

typedef enum {
    HFI_STATE_CONVERGE, // 收敛等待（仅注入，无直流电压）
    HFI_STATE_NSD,      // 极性辨识
    HFI_STATE_RUN       // 正常运行（电流闭环）
} HFI_State_t;

extern HFI_Observer_t g_hfi_obs;
extern HFI_PLL_t g_hfi_pll;

#define HFI_INJECT_VOLTAGE      0.5f    // 注入电压幅值(V)，建议额定电压1/10
#define HFI_PLL_KP              20000.0f    // PLL比例增益
#define HFI_PLL_KI              2048000.0f * PWM_PERIOD_S   // PLL积分增益

#define HFI_NSD_ENABLE          0               // 是否启用NSD（极性辨识）功能

// HFI函数声明
void hfi_observer_init(HFI_Observer_t *obs, float u_inject);
void hfi_observer_update(HFI_Observer_t *obs, float i_alpha, float i_beta);
void hfi_extract_fundamental(HFI_Observer_t *obs, float i_d, float i_q, 
                              float *i_d_fund, float *i_q_fund);
float hfi_get_inject_voltage(HFI_Observer_t *obs);

void hfi_pll_init(HFI_PLL_t *pll, float kp, float ki);
void hfi_pll_update(HFI_PLL_t *pll, HFI_Observer_t *obs, float Ts);
void hfi_nsd_check(HFI_Observer_t *obs, HFI_PLL_t *pll, float i_d, float i_q);

#endif
