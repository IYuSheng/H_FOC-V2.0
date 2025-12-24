#include "foc_control.h"

// FOC控制变量
foc_control_t foc_ctrl;
AlphaBetaTypeDef alpha_beta;
SVPWM_t svpwm;

/* -----------------控制参数------------------- */
float target_speed = 10.0f; // 开环目标速度
float target_q = 0.5f;  // 开环目标q轴电压

/**
 * @brief FOC控制主函数
 */
void foc_control(void)
{
    foc_open_loop_control(target_speed, target_q);
}

/**
 * @brief FOC速度开环控制
 */
void foc_open_loop_control(float target_speed, float target_outq)
{
    static float32_t angle_accum = 0.0f;  // 电角度累加器
    foc_ctrl.out_q = target_outq;

    /************************** 1. 开环电角度计算 **************************/
    // 机械转速 → 电角度速度（rad/s）：ω_e = 2π * (n_rpm / 60) * 极对数
    arm_scale_f32(&target_speed, SPEED_FACTOR, &foc_ctrl.speed, 1);

    // 积分更新电角度：θ = θ + ω_e * 控制周期（控制周期 = 1/CONTROL_LOOP_FREQ）
    float32_t ctrl_period = PWM_PERIOD_S;
    angle_accum += foc_ctrl.speed * ctrl_period;
    angle_accum = angle_normalize(angle_accum);
    foc_ctrl.angle = angle_accum;

    /************************** 2. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, foc_ctrl.angle);

    /************************** 3. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    svpwm.sector = svpwm_sector_calc(&alpha_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(&alpha_beta, &svpwm, 13.0f);  // 假设母线电压为13V

    svpwm_duty_calc(&svpwm);

    // 输出PWM到定时器
    bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}

SVPWM_t* svpwm_get(void)
{
    return &svpwm;
}
