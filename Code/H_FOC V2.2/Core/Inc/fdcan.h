#ifndef __FDCAN_H__
#define __FDCAN_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===================== 通信协议定义 ===================== */

// 节点ID范围：0-15（支持16个关节） 标准ID 11位
#define JOINT_ID_MASTER     0x00    // 主节点
#define JOINT_ID_BROADCAST  0x0F    // 广播地址

// 消息类型（低4位）
#define MSG_TYPE_STATUS     0x0     // 状态广播（位置、速度、电流、温度）
#define MSG_TYPE_CONTROL    0x1     // 控制指令（目标位置、速度、力矩）
#define MSG_TYPE_PARAM      0x2     // 参数配置（PID、限幅等）
#define MSG_TYPE_DEBUG      0x3     // 调试数据（ADC、观测器状态等）
#define MSG_TYPE_ERROR      0x4     // 错误帧
#define MSG_TYPE_SYNC       0x5     // 同步帧（多关节同步触发）

// 帧ID构造：(源ID << 8) | (目标ID << 4) | 消息类型
#define CAN_ID_MAKE(src, dst, type) (((src) << 8) | ((dst) << 4) | (type))
#define CAN_ID_GET_SRC(id)          (((id) >> 8) & 0x0F)
#define CAN_ID_GET_DST(id)          (((id) >> 4) & 0x0F)
#define CAN_ID_GET_TYPE(id)         ((id) & 0x0F)

/* ===================== 定点数缩放因子 ===================== 
 * Q格式定义：物理值 = 原始值 × 缩放因子
 * 使用int32_t可表示范围：±2.1×10⁹ × 缩放因子
 */
#define POS_SCALE       0.0001f     // 位置：0.0001°/LSB  (±214748°)
#define VEL_SCALE       0.001f      // 速度：0.001 rpm/LSB (±2.1M rpm)
#define CUR_SCALE       0.001f      // 电流：0.001 A/LSB   (±2.1M A)
#define TORQUE_SCALE    0.001f      // 力矩：0.001 Nm/LSB  (±2.1M Nm)
#define TEMP_SCALE      0.1f        // 温度：0.1°C/LSB     (±2.1亿°C)

// 定点数转换宏
#define FLOAT_TO_FIX(f, scale)      ((int32_t)((f) / (scale)))
#define FIX_TO_FLOAT(i, scale)      ((float)(i) * (scale))

#define FLOAT_TO_FIX16(f, scale)    ((int16_t)((f) / (scale)))
#define FIX16_TO_FLOAT(i, scale)    ((float)(i) * (scale))

/* ===================== 数据结构定义 ===================== */

// 关节状态数据包（16字节，每帧可传4个关节）
typedef struct __attribute__((packed)) {
    int32_t  position;      // 当前位置（定点）
    int32_t  velocity;      // 当前速度（定点）
    int16_t  current;       // 当前电流（定点）
    int16_t  temperature;   // 温度（定点，0.1°C）
    uint16_t status;        // 状态字（bit0:使能, bit1:错误, etc）
    uint16_t error_code;    // 错误码
} joint_status_t;

// 控制指令数据包（12字节）
typedef struct __attribute__((packed)) {
    int32_t  target_pos;    // 目标位置
    int32_t  target_vel;    // 目标速度
    int16_t  target_cur;    // 目标电流（力矩模式）
    int16_t  control_mode;  // 控制模式（0:位置,1:速度,2:力矩）
    uint16_t  reserve;      // 保留
} joint_control_t;

// 原始CAN帧（用于队列）
typedef struct {
    uint32_t id;            // 扩展帧ID
    uint8_t  data[64];      // CAN FD最大64字节
    uint8_t  len;           // 实际数据长度
    bool     is_extended;   // 是否为扩展帧
} can_frame_t;

/* ===================== FDCAN配置 ===================== */

// 波特率宏
#define CAN_BR_1M       8
#define CAN_BR_5M       13      // 数据域5Mbps

// 接收队列大小（根据内存调整，64字节×16=1KB）
#define FDCAN_RX_QUEUE_SIZE     16
#define FDCAN_TX_QUEUE_SIZE     8

/* ===================== 函数声明 ===================== */

// 初始化
void MX_FDCAN1_Init(void);
void FDCAN_Config_Filter(void);
void FDCAN_Start(void);

// 底层通信
int8_t FDCAN_SendPacket(uint8_t src_id, uint8_t dst_id, uint8_t msg_type, 
                        uint8_t *data, uint8_t len);
int8_t FDCAN_SendJointStatus(uint8_t joint_id, joint_status_t *status);
int8_t FDCAN_SendControlCmd(uint8_t dst_id, joint_control_t *cmd);

// 接收处理
void FDCAN_ProcessRxQueue(void);
// 接收回调
void FDCAN_RxCallback(can_frame_t *frame);

// 工具函数
uint8_t FDCAN_GetTxFreeLevel(void);
uint32_t FDCAN_GetRxDropCount(void);

#endif /* __FDCAN_H__ */
