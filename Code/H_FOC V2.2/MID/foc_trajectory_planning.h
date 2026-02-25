#ifndef FOC_TRAJECTORY_PLANNING_H
#define FOC_TRAJECTORY_PLANNING_H

#include <stdint.h>
#include <math.h>

/* S曲线7段状态定义 */
typedef enum {
    S_CURVE_IDLE = 0,       // 空闲/完成
    S_CURVE_T1,             // 加加速段 (Jerk>0, Acc↑)
    S_CURVE_T2,             // 匀加速段 (Acc=max)
    S_CURVE_T3,             // 减加速段 (Jerk<0, Acc↓)
    S_CURVE_T4,             // 匀速段   (Acc=0, Vel=max)
    S_CURVE_T5,             // 加减速段 (Jerk<0, Acc↓负向)
    S_CURVE_T6,             // 匀减速段 (Acc=-max)
    S_CURVE_T7              // 减减速段 (Jerk>0, Acc↑回零)
} SCurve_State_t;

/* S曲线规划器结构体 */
typedef struct {
    // 运动学约束参数（用户设定）
    float v_max;            // 最大速度 (deg/s 或 rad/s)
    float a_max;            // 最大加速度 (deg/s²)
    float j_max;            // 最大加加速度/Jerk (deg/s³)
    float freq_t;          // 调用间隔 (s)
    
    // 目标与状态
    float target_pos;       // 目标位置
    float current_pos;      // 当前规划位置
    float current_vel;      // 当前规划速度
    float current_acc;      // 当前规划加速度
    float current_jerk;     // 当前加加速度
    
    // 阶段管理
    SCurve_State_t state;   // 当前阶段
    uint32_t step_counter;  // 当前阶段已运行步数
    float stage_time;       // 当前阶段总时间 (s)
    float stage_steps;      // 当前阶段总步数
    
    // 方向与距离
    float direction;        // 运动方向 (+1.0 或 -1.0)
    float total_dist;       // 总位移绝对值
    
    // 预计算的时间参数 (s)
    float T1, T2, T3;       // 加速段时间 (T1+T2+T3 = 总加速时间)
    float T4;               // 匀速段时间
    float T5, T6, T7;       // 减速段时间 (通常 T5=T3, T6=T2, T7=T1)
    
    uint8_t is_busy;        // 运行中标志

    // 新增：用于阶段结束强制对齐
    float start_pos;           // 起始绝对位置
    float stage_end_pos[8];    // 各阶段结束时的绝对位置（下标1~7对应T1~T7结束）
} SCurve_Planner_t;

/**
 * @brief S型曲线规划器初始化
 * @param planner 规划器实例指针
 * @param vmax    最大速度 (单位与位置一致，如 deg/s)
 * @param amax    最大加速度 (deg/s²)
 * @param jmax    最大加加速度/Jerk (deg/s³)
 * @param freq_hz 调用频率 (Hz)，如位置环1kHz则填1000.0f
 */
void s_curve_planner_init(SCurve_Planner_t *planner, 
                          float vmax, float amax, float jmax, 
                          float freq_hz);

/**
 * @brief 设置新的目标位置（触发新的S曲线规划）
 * @param planner    规划器实例
 * @param current_pos 当前实际位置（作为起点）
 * @param target_pos  目标位置
 * @return 0:成功启动 1:距离太短无法规划
 */
uint8_t s_curve_set_target(SCurve_Planner_t *planner, 
                           float current_pos, 
                           float target_pos);

/**
 * @brief S型曲线更新函数（需在位置环中周期性调用）
 * @param planner 规划器实例
 * @return 当前规划的目标位置
 * @note  每次调用会更新 planner->current_pos/vel/acc
 */
float s_curve_update(SCurve_Planner_t *planner);

/**
 * @brief 检查是否到达目标
 */
static inline uint8_t s_curve_is_done(SCurve_Planner_t *planner) {
    return (planner->state == S_CURVE_IDLE);
}

#endif // FOC_TRAJECTORY_PLANNING_H