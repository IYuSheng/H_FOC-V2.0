#ifndef __CONFIG_H
#define __CONFIG_H

#include <arm_math.h>

// --------------------------- 模式选项 --------------------------
#define DEBUG_MODE  1     // 打开仿真波形打印

// -------------------------- 电源与硬件限制 --------------------------
#define VOLTAGE_LIMIT           30.0f   // 直流母线电压限制 (V)
#define CURRENT_LIMIT           5.0f    // 最大相电流限制 (A)
#define UNDER_VOLTAGE_THRESH    10.0f   // 欠压保护阈值 (V)
#define OVER_TEMPERATURE_THRESH 80.0f   // 过温保护阈值 (摄氏度)

// -------------------------- 电机参数 (根据实际电机填写) -------------------------- 
#define MOTOR_POLE_PAIRS     11       // 电机极对数
#define MOTOR_RESISTANCE     0.36f    // 相电阻 (欧姆)
#define MOTOR_INDUCTANCE     0.000154f  // 相电感 (H)
#define MOTOR_INDUCTANCE_Lq  0.000154f // q轴电感 (H)
#define MOTOR_INDUCTANCE_Ld  0.000154f // d轴电感 (H)

#define MOTOR_LOW             -169.0f * PI / 180.0f   // 电机最低位置

// -------------------------- FOC控制参数 --------------------------
#define PWM_FREQ            20000.0f   // PWM频率 (Hz)，需与定时器配置匹配
#define HALF_PWM_FREQ       PWM_FREQ * 0.5f   // PWM频率 (Hz)，需与定时器配置匹配
#define PWM_PERIOD_S        1.0f / PWM_FREQ  // PWM周期（单位：s），与PWM_FREQ对应：T = 1/F
#define SVPWM_VOLT_COEF   (2.0f / sqrtf(3.0f)) // SVPWM基本矢量幅值系数（母线电压相关，固定值）

// -------------------------- PI调节器参数 --------------------------
// 电流环PI
#define CURRENT_LOOP_BANDWIDTH    800.0f      // 电流环带宽 (Hz)
#define CURRENT_LOOP_COUNT        PWM_FREQ
#define CURRENT_LOOP_DT           1.0f / CURRENT_LOOP_COUNT  // 电流环调用周期 (单位：s)
#define I_D_P_GAIN                1.2f
#define I_Q_P_GAIN                1.2f
#define I_I_GAIN                  2400.0f
#define I_I_LIMIT                 7.0f

// 速度环PI
#define SPEED_LOOP_TIME     2000.0f
#define SPEED_LOOP_COUNT    PWM_FREQ / SPEED_LOOP_TIME  // 速度环调用频率 (2000Hz)
#define SPEED_LOOP_DT       1.0f / SPEED_LOOP_TIME      // 速度环调用周期 (单位：s)
#define SPEED_P_GAIN        0.006f
#define SPEED_I_GAIN        0.000f
#define SPEED_I_LIMIT       0.2f
#define SPEED_OUT_LIMIT     1.0f

// 位置环PID
#define POSITION_LOOP_TIME   1000.0f  // 位置环调用频率 (1000Hz)
#define POSITION_LOOP_COUNT  PWM_FREQ / POSITION_LOOP_TIME  // 位置环调用频率 (1000Hz)
#define POSITION_LOOP_DT    1.0f / POSITION_LOOP_TIME // 位置环调用周期 (单位：s)
#define POSITION_P_GAIN     0.12f
#define POSITION_I_GAIN     1.0f
#define POSITION_D_GAIN     0.005f
#define POSITION_I_LIMIT    0.5f
#define POSITION_OUT_LIMIT  4.0f
#define POSITION_KV         8.0f
#define POSITION_W_MAX      600.0f

// -------------------------- 电流采样配置 --------------------------
#define ADC_REF_VOLTAGE     3.3f    // ADC参考电压 (V)
#define ADC_MAX_VALUE       4095.0f    // ADC分辨率 (例如：12位ADC为4095，0-4095)
#define INA240_GAIN         50.0f   // 电流采样运放增益
#define V_REF               1.65f    // REF1 引脚电压3.3V，单位：V
#define R_Current           0.006f    // 采样电阻阻值

// -------------------------- 电压采样配置 --------------------------
#define R_Voaltage_1  20.0f   // 20KΩ
#define R_Voaltage_2  2.2f    // 2.2KΩ

// -------------------------- 常用常量及中间变量定义 --------------------------
#define _PI         PI
#define _2PI        (2.0f * PI)
#define _PI_2        (PI / 2.0f)
#define _2_PI       (1.0f / (2.0f * PI))
#define _60_angle   (2.0f * PI / 6.0f)  // 60度对应的弧度
#define _SQRT3      1.732050807568877f
#define _SQRT3_2    0.866025403784439f  // sqrt(3)/2
#define _1_SQRT3    0.577350269189626f  // 1/sqrt(3)
#define _2_SQRT3    1.154700538379252f  // 2/sqrt(3)
#define RPM_TO_PI   (_2PI / 60.0f)        // RPM转换为弧度系数 (RPM * 2*PI / 60)
#define RPM_TO_DEGREE   (360.0f / 60.0f)   // RPM转换为度数系数
#define SPEED_FACTOR (_2PI / 60.0f * MOTOR_POLE_PAIRS) // 速度转换系数 (机械转速rpm → 电角速度rad/s)
#define SPEED_FACTOR360 (360.0f / 60.0f * MOTOR_POLE_PAIRS) // 速度转换系数 (机械转速rpm → 电角速度°/s)
#define RAD_TO_DEG  (180.0f / PI)
#define FACTOR       (_SQRT3 / PWM_FREQ)  // SVPWM时间中间计算系数
#define PWM_PERIOD        ((170000000.0f / (2.0f * PWM_FREQ)) - 1)  // PWM周期计数值 (TIM时钟频率需根据实际配置调整)
#define PWM_FREQ_PERIOD (PWM_FREQ * PWM_PERIOD) // SVPWM时间转比较值中间系数

// -------------------------- 故障状态定义 --------------------------
typedef enum
{
  MOTOR_FAULT_NONE,       // 无故障
  FAULT_OVER_CURRENT,     // 过流
  FAULT_OVER_VOLTAGE,     // 过压
  FAULT_UNDER_VOLTAGE,    // 欠压
  FAULT_OVER_TEMPERATURE, // 过温
  FAULT_ENCODER,          // 编码器故障
  FAULT_I_SAMPLING_ERROR  // 电流采样错误
} FOC_FAULT;

// -------------------------- FOC状态定义 --------------------------
typedef enum
{
  FOC_STATE_INIT,         // 初始化
  FOC_STATE_RUNNING,      // 运行中
  FOC_STATE_FAULT,        // 故障
  FOC_STATE_STOP          // 停止
} FOC_STATUS;

#endif /* __CONFIG_H */
