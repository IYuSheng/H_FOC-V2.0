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
    static ButterworthLPF_t omega_flux_filt;
    static uint8_t filt_inited = 0;
    
    // 初始化滤波器
    if (!filt_inited) {
        butterworth_init(&omega_flux_filt, 10.0f, PWM_FREQ);
        filt_inited = 1;
    }

    // ===== 1. 磁链观测器（非线性部分）=====
    float y1 = u_alpha - FLUX_R_S * i_alpha; // 反电动势
    float y2 = u_beta  - FLUX_R_S * i_beta;
    
    // 计算磁链误差 eta = x - Ls*i
    float eta1 = obs->x1 - MOTOR_INDUCTANCE_Ld * i_alpha;  // 转子磁链(气隙磁链)
    float eta2 = obs->x2 - MOTOR_INDUCTANCE_Lq * i_beta;
    
    // 计算误差幅值平方 ||eta||^2
    float eta_norm_sq = eta1*eta1 + eta2*eta2;

    // 非线性校正项：gamma/2 * eta * (phi_m^2 - ||eta||^2)
    float error_term = FLUX_GAMMA_K * (FLUX_PHI_M_2 - eta_norm_sq);
    
    // 更新观测器状态（欧拉积分）
    obs->x1 += Ts * (y1 + error_term * eta1);   // 总磁链
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

    // 滤波PLL输出的角速度
    obs->omega_filt = butterworth_filter(&omega_flux_filt, omega_pll);

    // 积分得角度
    obs->theta_hat += omega_pll * Ts;
    
    // 角度归一化到 -π~π
    obs->theta_hat = angle_normalize_pi(obs->theta_hat);
    
    // 更新输出
    CORDIC_SinCos_Rad(obs->theta_hat, &obs->sin_theta, &obs->cos_theta);
}

/* ---------------------HFI观测器---------------------- */
HFI_Observer_t g_hfi_obs;
HFI_PLL_t g_hfi_pll;

/**
 * @brief HFI观测器初始化
 * @param obs HFI观测器结构体指针
 * @param u_inject 注入电压幅值(V)，建议为额定电压的1/10
 */
void hfi_observer_init(HFI_Observer_t *obs, float u_inject)
{
    obs->u_inject = u_inject;
    obs->u_inject_d = 0.0f;
    obs->inject_flag = 0;
    obs->init_done = 0;
    
    obs->i_alpha_last[0] = 0.0f;
    obs->i_alpha_last[1] = 0.0f;
    obs->i_beta_last[0] = 0.0f;
    obs->i_beta_last[1] = 0.0f;
    
    obs->i_alpha_h = 0.0f;
    obs->i_beta_h = 0.0f;
    obs->i_alpha_h_last = 0.0f;
    obs->i_beta_h_last = 0.0f;
    
    obs->i_env_alpha = 0.0f;
    obs->i_env_beta = 0.0f;
    
    obs->i_d_fund = 0.0f;
    obs->i_q_fund = 0.0f;
    
    obs->nsd_count = 0;
    obs->sum_pos = 0.0f;
    obs->sum_neg = 0.0f;
    obs->polarity_determined = 0;
}

/**
 * @brief HFI观测器更新 - 提取高频电流分量
 * @param obs HFI观测器结构体指针
 * @param i_alpha alpha轴电流
 * @param i_beta beta轴电流
 * @note 需在PWM周期开始时调用（注入电压切换后）
 */
void hfi_observer_update(HFI_Observer_t *obs, float i_alpha, float i_beta)
{
    // 1. 计算高频电流
    obs->i_alpha_h = (i_alpha - 2.0f * obs->i_alpha_last[0] + obs->i_alpha_last[1]) * 0.25f;
    obs->i_beta_h  = (i_beta  - 2.0f * obs->i_beta_last[0]  + obs->i_beta_last[1])  * 0.25f;
    
    // 3. 更新历史值
    obs->i_alpha_last[1] = obs->i_alpha_last[0];
    obs->i_alpha_last[0] = i_alpha;
    obs->i_beta_last[1] = obs->i_beta_last[0];
    obs->i_beta_last[0] = i_beta;
    
    // 4. 计算注入极性（注意：这个sign对应的是上一轮注入的极性）
    float sign = (obs->inject_flag == 0) ? 1.0f : -1.0f;
    
    // 5. 解调：计算电流包络
    obs->i_env_alpha = (obs->i_alpha_h - obs->i_alpha_h_last) * sign;
    obs->i_env_beta  = (obs->i_beta_h - obs->i_beta_h_last) * sign;

    // 6. 更新高频历史值
    obs->i_alpha_h_last = obs->i_alpha_h;
    obs->i_beta_h_last = obs->i_beta_h;

    // 7. 切换注入极性
    obs->inject_flag ^= 1;
}

/**
 * @brief 提取基频电流分量（用于电流环反馈）
 * @param obs HFI观测器结构体指针
 * @param i_d d轴电流（含高频）
 * @param i_q q轴电流（含高频）
 * @param i_d_fund 输出的d轴基频电流指针
 * @param i_q_fund 输出的q轴基频电流指针
 * @note 使用滑动平均滤波提取基频分量
 */
void hfi_extract_fundamental(HFI_Observer_t *obs, float i_d, float i_q, 
                              float *i_d_fund, float *i_q_fund)
{
    // 使用三点滑动平均提取基频分量
    // i_fund = (i[n] + 2*i[n-1] + i[n-2]) / 4
    static float i_d_last[2] = {0.0f, 0.0f};
    static float i_q_last[2] = {0.0f, 0.0f};
    
    *i_d_fund = (i_d + 2.0f * i_d_last[0] + i_d_last[1]) * 0.25f;
    *i_q_fund = (i_q + 2.0f * i_q_last[0] + i_q_last[1]) * 0.25f;
    
    // 保存历史值
    i_d_last[1] = i_d_last[0];
    i_d_last[0] = i_d;
    i_q_last[1] = i_q_last[0];
    i_q_last[0] = i_q;
    
    // 同时更新到结构体
    obs->i_d_fund = *i_d_fund;
    obs->i_q_fund = *i_q_fund;
}

/**
 * @brief 获取当前应注入的电压
 * @param obs HFI观测器结构体指针
 * @return 当前d轴注入电压（含极性）
 */
float hfi_get_inject_voltage(HFI_Observer_t *obs)
{
    // 方波注入：根据inject_flag决定极性
    obs->u_inject_d = (obs->inject_flag == 0) ? obs->u_inject : -obs->u_inject;
    return obs->u_inject_d;
}

/**
 * @brief HFI-PLL初始化
 * @param pll PLL结构体指针
 * @param kp 比例增益（建议2.0~3.0）
 * @param ki 积分增益（建议15~25）
 */
void hfi_pll_init(HFI_PLL_t *pll, float kp, float ki)
{
    pll->kp = kp;
    pll->ki = ki;
    pll->theta_est = 0.0f;
    pll->omega_est = 0.0f;
    pll->p_term = 0.0f;
    pll->i_term = 0.0f;
    pll->error = 0.0f;
}

/**
 * @brief HFI-PLL更新 - 跟踪转子位置
 * @param pll PLL结构体指针
 * @param obs HFI观测器结构体指针（使用其中的i_env_alpha/beta）
 * @param Ts 采样周期(s)
 * @note 基于电流包络的反正切进行位置跟踪
 */
void hfi_pll_update(HFI_PLL_t *pll, HFI_Observer_t *obs, float Ts)
{
    static ButterworthLPF_t omega_filt;
    static uint8_t filt_inited = 0;
    
    // 初始化滤波器
    if (!filt_inited) {
        butterworth_init(&omega_filt, 10.0f, PWM_FREQ);
        filt_inited = 1;
    }
    
    float sin_theta, cos_theta;
    
    // ================== 1. 角度归一化（-π ~ π）==================
    pll->theta_est = angle_normalize_pi(pll->theta_est);

    // ================== 2. 计算估计角度的 sin/cos ==================
    CORDIC_SinCos_Rad(pll->theta_est, &sin_theta, &cos_theta);
    
    // ================== 3. 获取解调后的电流包络 ==================
    float i_alpha = obs->i_env_alpha;
    float i_beta = obs->i_env_beta;
    
    // ================== 4. 正交解调计算误差 ==================
    pll->error = -i_beta * cos_theta + i_alpha * sin_theta; // 这里取反，观测出为电角度＋90度
    
    // ================== 5. PI 控制器 ==================
    pll->p_term = pll->kp * pll->error;
    pll->i_term += pll->ki * pll->error;
    
    // ================== 6. 积分限幅 ==================
    const float INT_LIMIT = 1000.0f;
    if (pll->i_term > INT_LIMIT) pll->i_term = INT_LIMIT;
    if (pll->i_term < -INT_LIMIT) pll->i_term = -INT_LIMIT;
    
    // ================== 7. 角速度计算（原始值）==================
    pll->omega_est = pll->p_term + pll->i_term;

    // ================== 8. 巴特沃斯低通滤波（对速度滤波）==================
    pll->omega_est_filt = butterworth_filter(&omega_filt, pll->omega_est);
    
    // ================== 9. 角度积分（使用滤波后的速度）==================
    pll->theta_est += pll->omega_est * Ts;
}

/**
 * @brief NSD(正负脉冲检测)极性辨识
 * @param obs HFI观测器结构体指针
 * @param pll PLL结构体指针（用于修正初始角度）
 * @param i_d d轴电流（含高频）
 * @param i_q q轴电流（含高频）
 * @note 在电机启动前执行，通过比较正负脉冲响应确定极性
 */
void hfi_nsd_check(HFI_Observer_t *obs, HFI_PLL_t *pll, float i_d, float i_q)
{ 
    if (obs->polarity_determined) return;
    obs->nsd_count++;

    #if HFI_NSD_ENABLE
    // 提取d轴高频电流
    static float i_d_last[2] = {0};
    float i_d_h = (i_d - 2.0f * i_d_last[0] + i_d_last[1]) * 0.25f;
    i_d_last[1] = i_d_last[0];
    i_d_last[0] = i_d;

    // 标准NSD序列：+Id -> 0 -> -Id -> 0 -> 比较
    // 阶段1: 0 - 200 (10ms) +1.5A 20khz
    if (obs->nsd_count < 200) {
        obs->nsd_target_d = 0.2f;
    }
    // 阶段2: 200 - 250 (10ms) 采样正
    else if (obs->nsd_count < 250) {
        obs->sum_pos += fabsf(i_d_h);
        obs->nsd_target_d = 0.2f;
    }
    // 阶段3: 250 - 350 (10ms) 施加 -1.5A
    else if (obs->nsd_count < 350) {
        obs->nsd_target_d = -0.2f;
    }
    // 阶段4: 350 - 400 (10ms) 采样负
    else if (obs->nsd_count < 400) {
        obs->sum_neg += fabsf(i_d_h);
        obs->nsd_target_d = -0.2f;
    }
    // 阶段5：判断与修正
    else {
        debug_log("%.4f, %.4f", obs->sum_pos, obs->sum_neg);
        if (obs->sum_neg > obs->sum_pos) {
            pll->theta_est += PI;  // 反转180度
            pll->theta_est = angle_normalize_pi(pll->theta_est);
        }
        obs->polarity_determined = 1;
        obs->nsd_target_d = 0.0f;
    }
    #else
    // 强拉到d轴，然后直接确定极性
    if (obs->nsd_count < 2000)
    {
        obs->nsd_target_d = 1.0f;
    }
    else
    {
        obs->nsd_target_d = 0.0f;
        // pll->theta_est += PI;
        pll->theta_est = angle_normalize_pi(pll->theta_est);
        obs->polarity_determined = 1;
    }
    #endif
}

/* ---------------------SMO观测器---------------------- */
// 后续添加
// ...
