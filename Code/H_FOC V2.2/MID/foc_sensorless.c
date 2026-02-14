#include "foc_sensorless.h"

/* ---------------------非线性磁链观测器---------------------- */

FluxObserver_t g_flux_obs;

/**
 * @brief 磁链观测器初始化函数
 * @param obs 观测器状态结构体指针
 */
void flux_observer_init(FluxObserver_t *obs)
{
    obs->x1 = 0.0f;
    obs->x2 = 0.0f;
    obs->cos_theta = 1.0f;
    obs->sin_theta = 0.0f;
    obs->theta = 0.0f;
    obs->omega = 0.0f;
}

/**
 * @brief 磁链观测器更新函数
 * @param obs 观测器状态结构体指针
 * @param u_alpha α轴电压输入
 * @param u_beta β轴电压输入
 * @param i_alpha α轴电流输入
 * @param i_beta β轴电流输入
 * @param Ts 采样周期（单位：s）
 */
void flux_observer_update(FluxObserver_t *obs, float u_alpha, float u_beta,
                          float i_alpha, float i_beta, float Ts)
{
    // ===== 1. 磁链观测器（非线性部分）=====
    float y1 = u_alpha - FLUX_R_S * i_alpha;
    float y2 = u_beta  - FLUX_R_S * i_beta;
    
    // 计算磁链误差 eta = x - Ls*i
    float eta1 = obs->x1 - FLUX_L_S * i_alpha;
    float eta2 = obs->x2 - FLUX_L_S * i_beta;
    
    // 计算误差幅值平方 ||eta||^2
    float eta_norm_sq = eta1*eta1 + eta2*eta2;

    // 非线性校正项：gamma/2 * eta * (phi_m^2 - ||eta||^2)
    float error_term = FLUX_GAMMA_K * (FLUX_PHI_M_2 - eta_norm_sq);
    
    // 更新观测器状态（欧拉积分）
    obs->x1 += Ts * (y1 + error_term * eta1);
    obs->x2 += Ts * (y2 + error_term * eta2);
    
    // ===== 2. PLL提取角度=====
    // 输入：eta = phi_m * [cosθ, sinθ]
    // PLL输出：theta_hat, cos(theta_hat), sin(theta_hat)
    
    // 计算角度误差
    // sin(θ - θ_hat) ≈ sinθ*cosθ_hat - cosθ*sinθ_hat
    //                = (eta2/phi_m)*cosθ_hat - (eta1/phi_m)*sinθ_hat
    float sin_theta_real = eta2 * _1_FLUX_PHI_M;  // sinθ
    float cos_theta_real = eta1 * _1_FLUX_PHI_M;  // cosθ
  
    // 计算角度误差（小角度近似）
    float angle_error = sin_theta_real * obs->cos_theta - cos_theta_real * obs->sin_theta;
    
    // PI控制器
    obs->omega_integ += PLL_KI * angle_error * Ts;
    float omega_pll = obs->omega_integ + PLL_KP * angle_error;
    
    // 积分得角度
    obs->theta_hat += omega_pll * Ts;
    
    // 角度归一化到 -π~π
    while (obs->theta_hat > 3.1415926f) obs->theta_hat -= 6.2831853f;
    while (obs->theta_hat < -3.1415926f) obs->theta_hat += 6.2831853f;
    
    // 更新输出
    float theta_temp = rad2deg(obs->theta_hat + 3.1415926f);
    obs->theta = angle_normalize_360(theta_temp);
    obs->omega = -omega_pll; // 这里正负与接线相关
    obs->cos_theta = cosf(obs->theta_hat);
    obs->sin_theta = sinf(obs->theta_hat);
}

/* ---------------------SMO观测器---------------------- */
// 后续添加
// ...
