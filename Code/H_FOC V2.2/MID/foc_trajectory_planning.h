#ifndef FOC_TRAJECTORY_PLANNING_H
#define FOC_TRAJECTORY_PLANNING_H

#include <stdint.h>
#include <math.h>

typedef enum {
    S_CURVE_IDLE = 0,       /**< 空闲/完成状态，轨迹已结束或尚未开始 */
    S_CURVE_ACCEL,          /**< 加速段：T1(加加速)+T2(匀加速)+T3(减加速)，使用6次多项式 */
    S_CURVE_COAST,          /**< 匀速段：T4，线性运动，加速度为0 */
    S_CURVE_DECEL           /**< 减速段：T5(加减速)+T6(匀减速)+T7(减减速)，使用6次多项式 */
} SCurve_State_t;

typedef struct {
    /* ========== 用户设定约束（规划前配置） ========== */
    float v_max;            /**< 最大速度限制 [deg/s]，轨迹规划的速度上限 */
    float a_max;            /**< 最大加速度限制 [deg/s²]，受电机转矩能力限制 */
    float j_max;            /**< 最大加加速度(Jerk)限制 [deg/s³]，影响机械冲击 */
    float freq_t;           /**< 控制周期 [s]，如1kHz控制频率对应0.001s */
    
    /* ========== 实时运动状态（update时更新） ========== */
    float target_pos;       /**< 目标位置 [deg]，set_target时设定，运动中不变 */
    float current_pos;      /**< 当前规划位置 [deg]，每个周期update更新，作为位置环给定 */
    float current_vel;      /**< 当前规划速度 [deg/s]，可用于速度前馈 */
    float current_acc;      /**< 当前规划加速度 [deg/s²]，可用于加速度前馈 */
    
    /* ========== 阶段管理 ========== */
    SCurve_State_t state;   /**< 当前规划阶段（IDLE/ACCEL/COAST/DECEL） */
    uint32_t tick;          /**< 全局时间计数器，从0开始递增，不随阶段重置，用于计算绝对时间 t = tick * freq_t */
    
    /* ========== 运动方向与参数 ========== */
    float direction;        /**< 运动方向：+1.0为正转，-1.0为反转，由目标与起点相对位置决定 */
    
    /* ========== 时间参数（单位：秒） ========== */
    float t_acc;            /**< 总加速时间 [s]，T1+T2+T3，6次多项式定义域 */
    float t_vel;            /**< 匀速段时间 [s]，T4，线性运动段 */
    float t_dec;            /**< 总减速时间 [s]，T5+T6+T7，6次多项式定义域 */
    float t_total;          /**< 总运动时间 [s]，t_acc + t_vel + t_dec */
    
    /* ========== 关键路点位置（用于阶段切换对齐） ========== */
    float pos_init;         /**< 轨迹起始位置 [deg]，即set_target时的current_pos */
    float pos_accel_end;    /**< 加速段结束位置 [deg]，= pos_init + 0.5*v_peak*t_acc*dir */
    float pos_coast_end;    /**< 匀速段结束位置 [deg]，= pos_accel_end + v_peak*t_vel*dir */
    float pos_end;          /**< 目标位置 [deg]，set_target传入的目标值 */
    
    /* ========== 速度参数 ========== */
    float vel_init;         /**< 初始速度 [deg/s]，通常设为0，支持非零初速连续规划 */
    float vel_max;          /**< 实际达到的最大速度 [deg/s]，可能低于v_max（短行程时），带方向 */
    
    /* ========== 6次多项式系数（归一化时间τ∈[0,1]） ========== */
    /* 位置公式：s(τ) = T * (A·τ^6 + B·τ^5 + C·τ^4 + F·τ) */
    /* 速度公式：v(τ) = 6A·τ^5 + 5B·τ^4 + 4C·τ^3 + F */
    
    /* 加速段系数（0 → v_max） */
    float A1;               /**< 6次项系数，= v_max */
    float B1;               /**< 5次项系数，= -3*v_max */
    float C1;               /**< 4次项系数，= 2.5*v_max */
    float F1;               /**< 1次项系数（初速度项），= vel_init */
    
    /* 减速段系数（v_max → 0） */
    float A2;               /**< 6次项系数，= -v_max */
    float B2;               /**< 5次项系数，= 3*v_max */
    float C2;               /**< 4次项系数，= -2.5*v_max */
    float F2;               /**< 1次项系数（初速度项），= v_max */
    
    uint8_t is_busy;        /**< 运行标志：1-正在规划中，0-空闲/完成 */
} SCurve_Planner_t;

/**
 * @brief 初始化S曲线规划器
 * @param planner 规划器实例指针
 * @param vmax    最大速度限制 [deg/s]
 * @param amax    最大加速度限制 [deg/s²]
 * @param jmax    最大加加速度限制 [deg/s³]
 * @param freq_t  控制周期 [s]
 */
void s_curve_planner_init(SCurve_Planner_t *planner, float vmax, float amax, float jmax, float freq_t);

/**
 * @brief 设置目标位置并生成轨迹规划
 * @param planner     规划器实例指针
 * @param current_pos 当前实际位置（编码器反馈）[deg]
 * @param target_pos  目标位置 [deg]
 * @return uint8_t    0:成功启动，1:距离过短无需规划
 * 
 * @warning 此函数会重置tick计数器，并立即根据当前位置生成完整轨迹参数
 * @note 调用后规划器状态变为ACCEL，需周期性调用update执行轨迹
 */
uint8_t s_curve_set_target(SCurve_Planner_t *planner, float current_pos, float target_pos);

/**
 * @brief 执行轨迹更新（周期性调用）
 * @param planner 规划器实例指针
 * @return float  当前规划的目标位置 [deg]，作为位置环给定值
 * 
 * @warning 必须在固定周期调用（频率由freq_t决定），时间连续性依赖tick累加
 * @note 首次调用返回pos_init，随后按S曲线平滑过渡至target_pos
 */
float s_curve_update(SCurve_Planner_t *planner);

/**
 * @brief 检查轨迹是否完成
 * @param planner 规划器实例指针  
 * @return uint8_t 1:已完成（IDLE状态），0:进行中
 * 
 * @note 内联函数，可在控制循环中快速查询状态
 */
static inline uint8_t s_curve_is_done(SCurve_Planner_t *planner) { 
    return (planner->state == S_CURVE_IDLE); 
}

#endif /* FOC_TRAJECTORY_PLANNING_H */
