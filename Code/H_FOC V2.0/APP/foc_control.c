#include "foc_control.h"

static void foc_open_loop_control(float target_speed, float target_outq);
static void foc_current_control(float target_d, float target_q);
static void foc_speed_control(float target_speed);
static void foc_position_control(float target_position);

// FOC控制变量
foc_control_t foc_ctrl;
AlphaBetaTypeDef alpha_beta;
SVPWM_t svpwm;
float speed_err;

void foc_debug(void)
{
    // debug_log("%.4f, %.4f, %.4f, %.4f, %.4f", alpha_beta.beta_i, alpha_beta.alpha_i, encoder_data.electrical_angle, foc_ctrl.abc_dq.current_d, foc_ctrl.abc_dq.current_q);
    debug_log("%.4f, %.4f, %.4f, %.4f", foc_ctrl.target_position, encoder_data.mechanical_angle, foc_ctrl.abc_dq.current_q, foc_ctrl.target_q);
    // debug_log("%.4f, %.4f, %.4f, %.4f", foc_ctrl.abc_dq.current_q, encoder_data.mechanical_speed, foc_ctrl.target_q, foc_ctrl.target_speed);
    // debug_log("%.4f, %.4f, %.4f, %.4f", foc_ctrl.target_q, encoder_data.mechanical_speed, foc_ctrl.abc_dq.current_q, foc_ctrl.abc_dq.current_d);
    // debug_log("%d, %d, %d", svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}

// q负为顺时针转

/**
 * @brief FOC初始化，锁定电机至零位
 */
void foc_start_init(void)
{   
    foc_ctrl.target_speed = 120.0f;   // 设置目标速度(°/s)
    foc_ctrl.out_q = 0.2f; // 设置q轴输出电压(开环用)
    foc_ctrl.out_d = 0.0f; // 设置d轴输出电压(开环用)
    foc_ctrl.target_q = -0.14f;  // 设置目标Q轴电流
    foc_ctrl.target_d = 0.0f;  // 设置目标D轴电流
    foc_ctrl.target_position = 140.0f;  // 设置目标位置
    // 初始化电流环PI参数
    foc_current_pi_init(I_D_P_GAIN, I_Q_P_GAIN, I_I_GAIN, I_I_LIMIT);
    // 初始化速度环PI参数
    foc_speed_pi_init(SPEED_P_GAIN, SPEED_I_GAIN, SPEED_I_LIMIT);
    // 初始化位置环PI参数
    foc_position_pi_init(POSITION_P_GAIN, POSITION_I_GAIN, POSITION_D_GAIN, POSITION_I_LIMIT);

    // 锁定电机至零位
    foc_ctrl.out_q = 0.2f; // 初始q轴电压
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, 90.0f);  //将电角度设为90度(直接当作驱动电机角度)锁定电机至零位
    svpwm.sector = svpwm_sector_calc(&alpha_beta);
    svpwm_calc_times(&alpha_beta, &svpwm, 13.0f);  // 假设母线电压为13V
    svpwm_duty_calc(&svpwm);
    bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
    HAL_Delay(1000); // 等待电机稳定
}

/**
 * @brief FOC控制主函数
 */
void foc_control(void)
{
    // foc_open_loop_control(foc_ctrl.target_speed, foc_ctrl.out_q);
    // foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
    // foc_speed_control(foc_ctrl.target_speed);
    foc_position_control(foc_ctrl.target_position);
}

/**
 * @brief FOC速度开环控制
 */
static void foc_open_loop_control(float target_speed, float target_outq)
{
    static float32_t angle_accum = 0.0f;  // 电角度累加器
    foc_ctrl.out_q = target_outq;

    clark_transform(&foc_current_data,&foc_voltage_data, &alpha_beta);
    // 将alpha-beta坐标系电流转换为DQ坐标系电流
    park_transform(&alpha_beta, &foc_ctrl);

    /************************** 1. 开环电角度计算 **************************/
    // 机械转速 → 电角度速度（°/s）
    arm_scale_f32(&target_speed, SPEED_FACTOR360, &foc_ctrl.speed, 1);

    // 积分更新电角度：θ = θ + ω_e * 控制周期（控制周期 = 1/CONTROL_LOOP_FREQ）
    float32_t ctrl_period = PWM_PERIOD_S;
    angle_accum += foc_ctrl.speed * ctrl_period;
    angle_accum = angle_normalize_360(angle_accum);
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

/**
 * @brief FOC电流闭环控制函数
 */
static void foc_current_control(float target_d, float target_q)
{
    // 将ABC坐标系采集电压电流转换为alpha-beta坐标系电压电流
    clark_transform(&foc_current_data,&foc_voltage_data, &alpha_beta);

    foc_ctrl.angle = encoder_data.electrical_angle; // 读取编码器电角度

    // 将alpha-beta坐标系电流转换为DQ坐标系电流
    park_transform(&alpha_beta, &foc_ctrl);

    // 电流环PID计算
    foc_ctrl.out_d = foc_id_pid_calculate(foc_ctrl.target_d, foc_ctrl.abc_dq.current_d);
    foc_ctrl.out_q = foc_iq_pid_calculate(foc_ctrl.target_q, foc_ctrl.abc_dq.current_q);

    /************************** 1. 反Park变换（DQ→αβ） **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, foc_ctrl.angle);

    /************************** 2. SVPWM核心：扇区判断→作用时间→占空比 **************************/
    // 扇区判断
    svpwm.sector = svpwm_sector_calc(&alpha_beta);

    // 计算基本矢量与零矢量作用时间
    svpwm_calc_times(&alpha_beta, &svpwm, 13.0f);  // 假设母线电压为13V

    // 计算得到PWM定时器比较值
    svpwm_duty_calc(&svpwm);

    // 输出PWM到定时器 
    bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
}

/**
 * @brief FOC速度闭环控制
 */
static void foc_speed_control(float target_speed)
{
    static uint32_t speed_count = 0;          // 速度环执行计数

    speed_count++;

    if(speed_count > SPEED_LOOP_COUNT)
    {
    foc_ctrl.target_q = foc_speed_pid_calculate(target_speed, encoder_data.mechanical_speed);
    speed_count = 0;
    }

    foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
}

/**
 * @brief FOC位置闭环（外环位置环 + 内环电流环）
 * @param target_position 目标位置（机械弧度，范围：0~360）
 */
static void foc_position_control(float target_position)
{
    static uint32_t count = 0;
    const float Frefeedback = 0.0f;

    count++;

    /************************** 1. 位置环PI控制 **************************/
    if(count > POSITION_LOOP_COUNT)
    {
    foc_ctrl.target_q = -foc_position_pid_calculate(target_position, encoder_data.mechanical_angle);
    count = 0;
    }
    if(foc_ctrl.target_q > 0.0f)
    {
    foc_ctrl.target_q = foc_ctrl.target_q + Frefeedback;
    }
    else
    {
    foc_ctrl.target_q = foc_ctrl.target_q - Frefeedback;
    }

    /************************** 2. 内环电流环（跟踪目标Q轴电流） **************************/
    foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
}

uint16_t get_svpwm(void)
{
    return svpwm.pwm_a;
}

float foc_get_angle(void)
{
    return foc_ctrl.angle;
}
