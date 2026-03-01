#include "foc_control.h"

SCurve_Planner_t g_pos_planner;

static inline void foc_open_loop_control(float target_speed, float target_outq);
static inline void foc_current_control(float target_d, float target_q);
static inline void foc_speed_control(float target_speed);
static inline void foc_position_control(float target_position);
static inline void foc_current_control_hfi(float target_d, float target_q);

/**
 * @brief FOC打印调试信息
 */
void foc_debug(void)
{
    // debug_log("%.4f, %.4f, %.4f, %.4f, %.4f", foc_ctrl.target_q, foc_ctrl.abc_dq.current_q, foc_ctrl.abc_dq.current_d, encoder_data.electrical_speed, encoder_data.electrical_angle);
    debug_log("%.4f, %.4f, %.4f, %.4f", foc_ctrl.shaped_t_p, encoder_data.mechanical_angle, encoder_data.mechanical_speed, foc_ctrl.abc_dq.current_q);
    // debug_log("%.4f, %.4f, %.4f, %.4f", foc_ctrl.abc_dq.current_q, foc_ctrl.abc_dq.current_d, encoder_data.mechanical_speed, foc_ctrl.target_speed);
    // debug_log("%d, %d, %d", svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
    // debug_log("%.4f, %.4f, %.4f, %.4f", foc_current_data.ia, foc_current_data.ib, foc_current_data.ic, foc_ctrl.angle);
    // debug_log("%.4f", foc_voltage_data.vbus);
    // FDCAN_SendFloat4Data(foc_ctrl.target_position, encoder_data.mechanical_angle, foc_ctrl.abc_dq.current_q, foc_ctrl.target_q);

    // 电磁转矩计算
    // foc_ctrl.Te = 1.5f * MOTOR_POLE_PAIRS * (MOTOR_FLUX_LINKAGE * foc_ctrl.abc_dq.current_q);
    // debug_log("%.4f", foc_ctrl.Te);

    // debug_log("%.2f, %.2f, %.4f, %.4f", 
    // angle_normalize_pi(deg2rad(encoder_data.electrical_angle-180.0f)),
    // g_flux_obs.theta_hat,
    // -encoder_data.electrical_speed,
    // g_flux_obs.omega_filt);

    // debug_log("%.2f, %.2f, %.4f, %.4f", 
    // angle_normalize_pi(deg2rad(encoder_data.electrical_angle-180.0f)),
    // g_hfi_pll.theta_est,
    // g_flux_obs.theta_hat,
    // g_hfi_pll.i_term);

    // debug_log("%.4f, %.4f, %.4f, %.4f, %.4f", g_hfi_pll.omega_est_filt, g_flux_obs.omega_filt, -encoder_data.electrical_speed, g_hfi_pll.theta_est, g_flux_obs.theta_hat);

    #if FOC_TEST_ENABLE // 扫频测试
    // q轴正弦波扫频输出
    static float sine_angle = 0.0f;
    const float amplitude = 0.1f;
    
    // 计算当前正弦值并设置到q轴输出
    foc_ctrl.target_q = amplitude * sinf(sine_angle);
    
    // 角度每次增加1度（弧度制）
    sine_angle += 0.0174533f;  // 1度 = π/180 弧度
    
    // 限制角度在0-2π范围内
    if(sine_angle >= 6.283185f) {
        sine_angle -= 6.283185f;
    }
    #endif
}

/**
 * @brief FOC初始化，锁定电机至零位
 */
void foc_start_init(void)
{   
    foc_ctrl.target_speed = 500.0f;   // 设置目标速度(°/s)
    foc_ctrl.out_q = 0.2f; // 设置q轴输出电压(开环用)
    foc_ctrl.out_d = 0.0f; // 设置d轴输出电压(开环用)
    foc_ctrl.target_q = 0.2f;  // 设置目标Q轴电流
    foc_ctrl.target_d = 0.0f;  // 设置目标D轴电流
    foc_ctrl.target_position = -53.0f;  // 设置目标位置
    // 初始化电流环PI参数
    foc_current_pi_init(I_D_P_GAIN, I_Q_P_GAIN, I_I_GAIN, I_I_LIMIT);
    // 初始化速度环PI参数
    foc_speed_pi_init(SPEED_P_GAIN, SPEED_I_GAIN, SPEED_I_LIMIT);
    // 初始化位置环PI参数
    foc_position_pi_init(POSITION_P_GAIN, POSITION_I_GAIN, POSITION_D_GAIN, POSITION_I_LIMIT);

    // 锁定电机至零位
    get_foc_bus_voltage(); // 获取电机母线电压
    debug_log("Vbus: %.2f V", foc_voltage_data.vbus);
    // 电流偏置校准 
    adc1_current_offset_calibrate();

    #if FOC_COGGING_COMPENSATION_ENABLE
    cogging_calibration_start();
    #endif

    #if FOC_RESONANCE_ENABLE
    resonance_test_trigger();
    #endif

    #if FOC_SHAPER_ENABLE
    // EI整型器 (固有频率 V Ts)
    // ei_shaper_init(2.84f, 0.05f, POSITION_LOOP_DT);

    // 轨迹规划 (速度 加速度 加加速度 Ts)
    s_curve_planner_init(&g_pos_planner, 500.0f, 200.0f, 1000.0f, POSITION_LOOP_DT);
    #endif
    
    #if FLUX_OBSERVER_ENABLE
    // 初始化磁链观测器
    flux_observer_init(&g_flux_obs);
    #endif

    #if HFI_ENABLE
    hfi_observer_init(&g_hfi_obs, HFI_INJECT_VOLTAGE);
    hfi_pll_init(&g_hfi_pll, HFI_PLL_KP, HFI_PLL_KI);
    #endif
    
    #if 0   // 电机零位校准时启用
    LL_TIM_EnableIT_CC4(TIM8);
    foc_ctrl.out_q = 0.5f; // 初始q轴电压
    //  将电角度设为90度(直接当作驱动电机角度)锁定电机至零位
    float sin_theta, cos_theta;
    CORDIC_SinCos_Deg(90.0f, &sin_theta, &cos_theta);
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);
    svpwm.sector = svpwm_sector_calc(&alpha_beta);
    svpwm_calc_times(&alpha_beta, &svpwm, foc_voltage_data.vbus);
    svpwm_duty_calc(&svpwm);
    bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
    HAL_Delay(2000); // 等待电机稳定
    debug_log("%f", encoder_data.mechanical_angle);
    #endif

    #if FOC_PARAMETER_IDENTIFICATION_ENABLE // 电机参数自辨识启用
    foc_motor_param_ident_start(1.0f, 0.5f, 600.0f, 0.1f, 0.05f);
    #endif
}

/**
 * @brief FOC外环控制主函数 1kHZ运行
 */
void foc_control(void)
{
    #if FOC_RESONANCE_ENABLE

    static uint16_t count = 0;
    float iq_fb;
    
    switch(g_res_test.state) {
        case 1:
            g_res_test.start_pos = encoder_data.mechanical_angle;
            
            if(count < 1000)
            {
                count ++;
                foc_ctrl.target_position = encoder_data.mechanical_angle;   // 稳定一段时间
                // 保持位置
                iq_fb = foc_position_pid_calculate(foc_ctrl.target_position, 
                                                        encoder_data.mechanical_angle);
                float theta = deg2rad(encoder_data.mechanical_angle);
                float iq_grav = 2.8f * sinf(theta - MOTOR_LOW);
                foc_ctrl.target_q = iq_fb + iq_grav;
            }
            else
            {
                 // 触发阶跃
                foc_ctrl.target_position = g_res_test.start_pos + g_res_test.target_step;
                g_res_test.sample_cnt = 0;
                g_res_test.print_idx = 0;
                g_res_test.state = 2;
            }
        break;
            
        case 2: // 记录中（弱控制）
            // 弱位置环
            iq_fb = foc_position_pid_calculate(foc_ctrl.target_position, 
                                                     encoder_data.mechanical_angle);
            float theta = deg2rad(encoder_data.mechanical_angle);
            float iq_grav = 2.8f * sinf(theta - MOTOR_LOW);
            foc_ctrl.target_q = iq_fb + iq_grav;
            
            // 记录数据（索引检查）
            if (g_res_test.sample_cnt < 500) {
                g_res_test.pos_data[g_res_test.sample_cnt] = encoder_data.mechanical_angle;
                g_res_test.iq_data[g_res_test.sample_cnt] = foc_ctrl.target_q;
                g_res_test.sample_cnt++;
                
                // 1000ms后完成
                if (g_res_test.sample_cnt >= 500) {
                    g_res_test.state = 3;
                    // 恢复位置环参数
                    foc_position_pi_init(POSITION_P_GAIN, POSITION_I_GAIN, 
                                       POSITION_D_GAIN, POSITION_I_LIMIT);
                }
            }
        break;
            
        case 3: // 完成，等待打印
            g_res_test.print_idx ++;
            if(g_res_test.print_idx < 500)
            {
                debug_log("%d,%.4f,%.4f", 
                        g_res_test.print_idx,
                        g_res_test.pos_data[g_res_test.print_idx],
                        g_res_test.iq_data[g_res_test.print_idx]);
            }
            else
            {
                g_res_test.state = 4;
            }
            foc_ctrl.target_q = 0.0f;
        break;
    }
    #endif
    
    #if FOC_COGGING_COMPENSATION_ENABLE
    // 滤波iq
    static ButterworthLPF_t filt;
    static float iq_filt;
    static uint8_t filt_inited = 0;
    
    // 初始化滤波器
    if (!filt_inited) {
        butterworth_init(&filt, 200.0f, SPEED_LOOP_TIME);
        filt_inited = 1;
    }
    iq_filt = butterworth_filter(&filt, foc_ctrl.abc_dq.current_q);

    // 齿槽转矩标定
    if (g_cogging.state != COG_CAL_FINISHED)
    {
        float target_speed = 0;
        cogging_calibration_update(
            encoder_data.mechanical_angle,
            iq_filt,  // 使用滤波后的电流
            &target_speed
        );
        if (g_cogging.state == COG_CAL_VERIFY)
        {
            // 验证模式：速度环 + 齿槽补偿
            foc_speed_control(target_speed);
                
            // 叠加齿槽补偿（前馈）
            foc_ctrl.target_q += g_cogging.current_compensation;
            debug_log("%.4f", g_cogging.current_compensation);
        }
        else if (g_cogging.state != COG_CAL_FINISHED)
        {
            // 标定中：纯速度环
            foc_speed_control(target_speed);
        }
    }

    #else
        #if FOC_SPEED_CONTROL_ENABLE
        // 速度环
        foc_speed_control(foc_ctrl.target_speed);
        #endif
        #if FOC_POSITION_CONTROL_ENABLE
        // 位置环
        foc_position_control(foc_ctrl.target_position);
        // foc_position_MIT_control(foc_ctrl.target_position);
        #endif
    #endif
}

/**
 * @brief FOC内环电流环外部调用接口
 */
void foc_current_in_control(void)
{
    #if FOC_PARAMETER_IDENTIFICATION_ENABLE
    // 调用电机参数自辨识步骤
    if (foc_motor_param_ident_is_running())
    {
        foc_motor_parameter_ident_step(); // 执行辨识步骤
    }
    #else
        #if HFI_ENABLE
        foc_current_control_hfi(foc_ctrl.target_d, foc_ctrl.target_q);
        #else
        // foc_open_loop_control(foc_ctrl.target_speed, foc_ctrl.out_q);
        foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
        #endif
    #endif
}

/**
 * @brief FOC速度开环控制
 */
static inline void foc_open_loop_control(float target_speed, float target_outq)
{
    static float32_t angle_accum = 0.0f;  // 电角度累加器
    foc_ctrl.out_q = target_outq;

    clark_transform(&foc_current_data, &alpha_beta);

    /************************** 1. 开环电角度计算 **************************/
    // 机械转速 → 电角度速度（°/s）
    foc_ctrl.speed = -target_speed * MOTOR_POLE_PAIRS;

    // 积分更新电角度：θ = θ + ω_e * 控制周期（控制周期 = 1/CONTROL_LOOP_FREQ）
    float32_t ctrl_period = PWM_PERIOD_S;
    angle_accum += foc_ctrl.speed * ctrl_period;
    angle_accum = angle_normalize_360(angle_accum);
    foc_ctrl.angle = angle_accum;

    /************************** 2. Park变换（αβ→DQ） **************************/
    float sin_theta, cos_theta;
    CORDIC_SinCos_Deg(foc_ctrl.angle, &sin_theta, &cos_theta);

    // 将alpha-beta坐标系电流转换为DQ坐标系电流
    park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

    /************************** 3. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);

    /************************** 4. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    svpwm.sector = svpwm_sector_calc(&alpha_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(&alpha_beta, &svpwm, foc_voltage_data.vbus);

    svpwm_duty_calc(&svpwm);

    // 输出PWM到定时器
    bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}

/**
 * @brief FOC电流闭环控制函数
 */
static inline void foc_current_control(float target_d, float target_q)
{
    // 将ABC坐标系采集电压电流转换为alpha-beta坐标系电压电流
    clark_transform(&foc_current_data, &alpha_beta);

    #if FLUX_OBSERVER_ENABLE
    // 观测器更新
    flux_observer_update(&g_flux_obs, 
                         alpha_beta.alpha, alpha_beta.beta,  // 电压（上次输出）
                         alpha_beta.alpha_i, alpha_beta.beta_i,  // 电流
                         PWM_PERIOD_S);
    foc_ctrl.angle = g_flux_obs.theta_hat;
    float sin_theta, cos_theta;
    CORDIC_SinCos_Rad(foc_ctrl.angle, &sin_theta, &cos_theta);
    #else
    foc_ctrl.angle = encoder_data.electrical_angle;
    float sin_theta, cos_theta;
    CORDIC_SinCos_Deg(foc_ctrl.angle, &sin_theta, &cos_theta);
    #endif

    // 将alpha-beta坐标系电流转换为DQ坐标系电流
    park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

    // 滤波处理DQ轴电流，用于前馈解耦
    foc_ctrl.abc_dq.current_d_filt = foc_ctrl.abc_dq.current_d * 0.1f + foc_ctrl.abc_dq.current_d_filt * 0.9f;
    foc_ctrl.abc_dq.current_q_filt = foc_ctrl.abc_dq.current_q * 0.1f + foc_ctrl.abc_dq.current_q_filt * 0.9f;

    #if FLUX_OBSERVER_ENABLE
    // 电流环前馈解耦
    float vd_ff = -g_flux_obs.omega_filt * MOTOR_INDUCTANCE_Lq * foc_ctrl.abc_dq.current_q;
    float vq_ff =  g_flux_obs.omega_filt * (MOTOR_INDUCTANCE_Ld * foc_ctrl.abc_dq.current_d + MOTOR_FLUX_LINKAGE);
    #else
    // 电流环前馈解耦
    float vd_ff = -encoder_data.electrical_speed * MOTOR_INDUCTANCE_Lq * foc_ctrl.abc_dq.current_q_filt;
    float vq_ff =  encoder_data.electrical_speed * (MOTOR_INDUCTANCE_Ld * foc_ctrl.abc_dq.current_d_filt + MOTOR_FLUX_LINKAGE);
    #endif

    // 电流环PID计算
    float vd_pi = foc_id_pid_calculate(foc_ctrl.target_d, foc_ctrl.abc_dq.current_d);
    float vq_pi = foc_iq_pid_calculate(foc_ctrl.target_q, foc_ctrl.abc_dq.current_q);

    foc_ctrl.out_d = vd_pi + vd_ff;
    foc_ctrl.out_q = vq_pi + vq_ff;

    /************************** 1. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);

    /************************** 2. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    svpwm.sector = svpwm_sector_calc(&alpha_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(&alpha_beta, &svpwm, foc_voltage_data.vbus);

    // 计算得到PWM定时器比较值
    svpwm_duty_calc(&svpwm);

    // 输出PWM到定时器 
    bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}

/**
 * @brief FOC速度闭环控制
 */
static inline void foc_speed_control(float target_speed)
{
    // ===== 重力补偿前馈 =====
    // float theta = deg2rad(encoder_data.mechanical_angle);
    // const float K_G = 0.38f;
    // const float TH0 = MOTOR_LOW;
    // float iq_grav = K_G * sinf(theta - TH0);
    static float iq_ff = 0.0f;

    if(foc_ctrl.target_speed > 0.0f)
    {
        iq_ff = 0.13f; // 速度环前馈
    }
    else
    {
        iq_ff = -0.13f; // 速度环前馈
    }
    float iq_fb = foc_speed_pid_calculate(target_speed, encoder_data.mechanical_speed);
    foc_ctrl.target_q = iq_fb + iq_ff;
}

/**
 * @brief FOC位置闭环（外环位置环） 1Khz 运行
 * @param target_position 目标位置（机械弧度，范围：0~360）
 */
static inline void foc_position_control(float target_position)
{
    /************************** 位置环PI控制 **************************/
    #if FOC_SHAPER_ENABLE
        static float last_target = 0.0f;
        static uint8_t first_run = 1;
        
        // 检测目标位置是否变化（或首次运行）
        if (first_run || fabsf(target_position - last_target) > 0.1f) {
            s_curve_set_target(&g_pos_planner, 
                            foc_ctrl.shaped_t_p,            // 起点：当前参考位置
                            target_position);               // 终点：新目标
            last_target = target_position;
            first_run = 0;
        }
        // 获取S曲线规划后的平滑位置
        if (s_curve_is_done(&g_pos_planner)) {
            foc_ctrl.shaped_t_p = target_position; // 规划完成，直接使用目标
        } else {
            foc_ctrl.shaped_t_p = s_curve_update(&g_pos_planner);
        }
        // foc_ctrl.shaped_t_p = ei_shaper_process(foc_ctrl.shaped_t_p); // 位置环输入整形
    #else
        foc_ctrl.shaped_t_p = target_position;
    #endif

    float iq_fb = foc_position_pid_calculate(foc_ctrl.shaped_t_p, encoder_data.mechanical_angle);

    // ===== 重力补偿前馈 =====
    float theta = deg2rad(encoder_data.mechanical_angle);
    const float K_G = 3.1f;
    const float TH0 = MOTOR_LOW;
    float iq_grav = K_G * sinf(theta - TH0);

    // 计算最终目标Q轴电流
    foc_ctrl.target_q = iq_fb + iq_grav;
}

/**
 * @brief FOC电流闭环控制函数（集成HFI）
 * @param target_d 目标D轴电流
 * @param target_q 目标Q轴电流
 * @note 当编码器失效或低速时自动切换到HFI模式
 */
static inline void foc_current_control_hfi(float target_d, float target_q)
{
    clark_transform(&foc_current_data, &alpha_beta);
    
    float sin_theta, cos_theta;
    float vd_pi, vq_pi;
    float i_d_fund, i_q_fund; // 基频电流

    #if HFI_STANDALONE_MODE
        hfi_observer_update(&g_hfi_obs, alpha_beta.alpha_i, alpha_beta.beta_i);
        float u_inject = hfi_get_inject_voltage(&g_hfi_obs);

        // 状态机处理
        switch (g_hfi_obs.state) {
            case HFI_STATE_CONVERGE:
                hfi_pll_update(&g_hfi_pll, &g_hfi_obs, PWM_PERIOD_S);
                foc_ctrl.angle = g_hfi_pll.theta_est;
                CORDIC_SinCos_Rad(foc_ctrl.angle, &sin_theta, &cos_theta);
                park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

                // 开环：仅注入，PI 输出强制为0
                vd_pi = 0.0f;
                vq_pi = 0.0f;

                // 收敛判断（基于误差）
                if (fabsf(g_hfi_pll.error) < 0.05f) {
                    if (++g_hfi_obs.converge_cnt > 100) {
                        g_hfi_obs.state = HFI_STATE_NSD;
                        g_hfi_obs.converge_cnt = 0;
                    }
                } else {
                    g_hfi_obs.converge_cnt = 0;
                }
                break;

            case HFI_STATE_NSD:
                // 使用收敛后的固定角度
                foc_ctrl.angle = g_hfi_pll.theta_est;
                CORDIC_SinCos_Rad(foc_ctrl.angle, &sin_theta, &cos_theta);
                park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

                if (!g_hfi_obs.polarity_determined) {
                    hfi_nsd_check(&g_hfi_obs, &g_hfi_pll,
                                foc_ctrl.abc_dq.current_d,
                                foc_ctrl.abc_dq.current_q);
                }

                u_inject *= 0.2f; // NSD阶段降低注入电压，减少对电机的冲击
                vd_pi = g_hfi_obs.nsd_target_d;
                vq_pi = 0.0f;

                if (g_hfi_obs.polarity_determined) {
                    g_hfi_obs.state = HFI_STATE_RUN;
                }
                break;

            case HFI_STATE_RUN:
                hfi_pll_update(&g_hfi_pll, &g_hfi_obs, PWM_PERIOD_S);
                foc_ctrl.angle = g_hfi_pll.theta_est;
                CORDIC_SinCos_Rad(foc_ctrl.angle, &sin_theta, &cos_theta);
                park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

                hfi_extract_fundamental(&g_hfi_obs,
                                        foc_ctrl.abc_dq.current_d,
                                        foc_ctrl.abc_dq.current_q,
                                        &i_d_fund, &i_q_fund);

                vd_pi = foc_id_pid_calculate(target_d, i_d_fund);
                vq_pi = foc_iq_pid_calculate(target_q, i_q_fund);
                break;
        }

        foc_ctrl.out_d = vd_pi + u_inject;
        foc_ctrl.out_q = vq_pi;
    #else  // HFI混合模式
        // ========== 模式切换参数配置 ==========
        #define SPEED_HFI_LOW       80.0f   // HFI上限（进入滞环区）
        #define SPEED_HFI_HIGH      120.0f   // 磁链观测器下限（退出滞环区）
        #define SPEED_FILT_ALPHA    0.1f     // 速度滤波系数
        
        // ========== 速度估计与滤波 ==========
        static uint8_t mode_transition_cnt = 0;  // 模式切换防抖计数器
        static uint8_t hfi_settle_cnt = 0;       // HFI稳定计数器
    
        // 更新观测器
        hfi_observer_update(&g_hfi_obs, alpha_beta.alpha_i, alpha_beta.beta_i);
        hfi_pll_update(&g_hfi_pll, &g_hfi_obs, PWM_PERIOD_S);

        flux_observer_update(&g_flux_obs, alpha_beta.alpha, alpha_beta.beta,
                            alpha_beta.alpha_i, alpha_beta.beta_i, PWM_PERIOD_S);
        
        // 获取HFI注入电压
        float u_inject = hfi_get_inject_voltage(&g_hfi_obs);
        
        // HFI极性检测
        if (!g_hfi_obs.polarity_determined) {
            hfi_nsd_check(&g_hfi_obs, &g_hfi_pll, 
                        foc_ctrl.abc_dq.current_d, foc_ctrl.abc_dq.current_q);
        }
        
        // 速度估计：取两者较小值
        float speed_inst = (fabsf(g_hfi_pll.omega_est_filt) < fabsf(g_flux_obs.omega_filt)) ? 
                        fabsf(g_hfi_pll.omega_est_filt) : fabsf(g_flux_obs.omega_filt);
        
        // ========== 滞环切换逻辑 ==========
        switch (foc_ctrl.sensor_mode) {
            case SENSORLESS_STATE_HFI:  // 当前纯HFI模式
                if (speed_inst > SPEED_HFI_LOW) {
                    if (++mode_transition_cnt > 100) {  // 5ms防抖
                        foc_ctrl.sensor_mode = SENSORLESS_STATE_MIX;  // 进入滞环区
                        mode_transition_cnt = 0;
                    }
                } else {
                    mode_transition_cnt = 0;
                }
                break;
                
            case SENSORLESS_STATE_MIX:  // 当前滞环混合模式
                if (speed_inst < SPEED_HFI_LOW) {
                    if (++mode_transition_cnt > 100) {
                        foc_ctrl.sensor_mode = SENSORLESS_STATE_HFI;  // 回到纯HFI
                        mode_transition_cnt = 0;
                    }
                } else if (speed_inst > SPEED_HFI_HIGH) {
                    if (++mode_transition_cnt > 100) {
                        foc_ctrl.sensor_mode = SENSORLESS_STATE_FLUX;  // 进入纯磁链
                        mode_transition_cnt = 0;
                    }
                } else {
                    mode_transition_cnt = 0;
                }
                break;
                
            case SENSORLESS_STATE_FLUX:  // 当前纯磁链模式
                if (speed_inst < SPEED_HFI_HIGH) {
                    if (++mode_transition_cnt > 100) {
                        foc_ctrl.sensor_mode = SENSORLESS_STATE_MIX;  // 回到滞环区
                        hfi_settle_cnt = 0;  // 重置HFI稳定计数器
                        g_hfi_pll.theta_est = g_flux_obs.theta_hat;
                        g_hfi_pll.i_term = 0.0f;  // 清空积分项，避免累积误差
                        g_hfi_pll.omega_est = g_flux_obs.omega_filt;  // 继承速度估计
                        mode_transition_cnt = 0;
                    }
                } else {
                    mode_transition_cnt = 0;
                }
                break;
        }
        
        // ========== 角度融合与模式执行 ==========
        float theta_hfi = g_hfi_pll.theta_est;
        float theta_flux = g_flux_obs.theta_hat;
        float ratio = 0.0f;  // 磁链权重（0=纯HFI, 1=纯磁链）
        
        switch (foc_ctrl.sensor_mode) {
            case SENSORLESS_STATE_HFI:  // 纯HFI模式
                foc_ctrl.angle = theta_hfi;
                foc_ctrl.sensor_mode = SENSORLESS_STATE_HFI;
                ratio = 0.0f;
                break;

            case SENSORLESS_STATE_MIX:  // 滞环混合模式 - 角度加权融合
                {
                    if (hfi_settle_cnt < 100) {  // 5ms收敛时间
                        foc_ctrl.angle = g_hfi_pll.theta_est;
                        hfi_settle_cnt++;
                    }
                    else
                    {
                        // 线性加权：速度越高速，磁链权重越大
                        ratio = (speed_inst - SPEED_HFI_LOW) / (SPEED_HFI_HIGH - SPEED_HFI_LOW);
                        ratio = (ratio < 0.0f) ? 0.0f : (ratio > 1.0f) ? 1.0f : ratio;
                        
                        // 加权融合角度
                        foc_ctrl.angle = (1.0f - ratio) * theta_hfi + ratio * theta_flux;
                        foc_ctrl.angle = angle_normalize_pi(foc_ctrl.angle);
                    }
                    
                    foc_ctrl.sensor_mode = SENSORLESS_STATE_MIX;
                }
                break;
                
            case SENSORLESS_STATE_FLUX:  // 纯磁链观测器模式
                foc_ctrl.angle = theta_flux;
                foc_ctrl.sensor_mode = SENSORLESS_STATE_FLUX;
                u_inject = 0.0f;  // 纯磁链模式关闭注入
                ratio = 1.0f;
                break;
        }
        
        // ========== 坐标变换 ==========
        CORDIC_SinCos_Rad(foc_ctrl.angle, &sin_theta, &cos_theta);
        park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);
        
        // ========== 电流反馈选择 ==========
        float i_d_fb, i_q_fb;
        
        if (ratio < 0.5f) {
            // HFI主导：提取基频电流（滤除高频注入分量）
            hfi_extract_fundamental(&g_hfi_obs, foc_ctrl.abc_dq.current_d, 
                                foc_ctrl.abc_dq.current_q, &i_d_fund, &i_q_fund);
            i_d_fb = i_d_fund + g_hfi_obs.nsd_target_d;
            i_q_fb = i_q_fund;
        } else {
            // 磁链主导：直接使用采样电流
            i_d_fb = foc_ctrl.abc_dq.current_d;
            i_q_fb = foc_ctrl.abc_dq.current_q;
        }
        
        // ========== PI计算 ==========
        vd_pi = foc_id_pid_calculate(target_d, i_d_fb);
        vq_pi = foc_iq_pid_calculate(target_q, i_q_fb);
        
        // ========== 注入电压衰减 ==========
        // 滞环区内线性衰减，纯磁链模式时完全关闭注入
        foc_ctrl.out_d = vd_pi + u_inject;
        foc_ctrl.out_q = vq_pi;

    #endif

    // 统一的后处理
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);
    svpwm.sector = svpwm_sector_calc(&alpha_beta);
    svpwm_calc_times(&alpha_beta, &svpwm, foc_voltage_data.vbus);
    svpwm_duty_calc(&svpwm);
    bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}
