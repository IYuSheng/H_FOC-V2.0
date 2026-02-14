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
    float theta;            // PLL输出：θ_hat（°）
    float omega;            // PLL输出：角速度（rad/s）
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
#define PLL_BANDWIDTH_HZ    1000.0f     // PLL带宽，如1000Hz
#define PLL_KP              (2.0f * 0.707f * 6.2831853f * PLL_BANDWIDTH_HZ)  // 2*zeta*wn
#define PLL_KI              ((6.2831853f * PLL_BANDWIDTH_HZ) * (6.2831853f * PLL_BANDWIDTH_HZ))  // wn^2

void flux_observer_init(FluxObserver_t *obs);
void flux_observer_update(FluxObserver_t *obs, float u_alpha, float u_beta, 
                          float i_alpha, float i_beta, float Ts);

#endif
