#ifndef __FDCAN_H__
#define __FDCAN_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===================== 通信协议定义 ===================== */

// 当前关节ID
#define JOINT_ID            1

// 节点ID范围：0-14（支持15个关节，0x0F为广播） 标准ID 11位
#define JOINT_ID_MAX        15
#define JOINT_ID_MASTER     0x00    // 主节点
#define JOINT_ID_BROADCAST  0x0F    // 广播地址

// 消息类型（低4位）
#define MSG_TYPE_STATUS     0x0     // 状态广播（位置、速度）
#define MSG_TYPE_CONTROL    0x1     // 控制指令（目标位置、目标电流、控制模式）
#define MSG_TYPE_PARAM      0x2     // 参数配置（PID、限幅等）
#define MSG_TYPE_DEBUG      0x3     // 调试数据（ADC、观测器状态等）
#define MSG_TYPE_ERROR      0x4     // 错误帧
#define MSG_TYPE_SYNC       0x5     // 同步帧（多关节同步触发）

// 控制模式定义
#define CONTROL_MODE_POSITION  0U
#define CONTROL_MODE_SPEED     1U
#define CONTROL_MODE_CURRENT   2U

// 帧ID构造：(源ID << 8) | (目标ID << 4) | 消息类型
#define CAN_ID_MAKE(src, dst, type) (((src) << 8) | ((dst) << 4) | (type))
#define CAN_ID_GET_SRC(id)          (((id) >> 8) & 0x0F)
#define CAN_ID_GET_DST(id)          (((id) >> 4) & 0x0F)
#define CAN_ID_GET_TYPE(id)         ((id) & 0x0F)

/* ===================== 定点数缩放因子 ===================== 
 * Q格式定义：物理值 = 原始值 × 缩放因子
 */
#define SCALE_32          0.00001f
#define INV_SCALE_32      (1.0f / SCALE_32)
#define SCALE_16          0.01f
#define INV_SCALE_16      (1.0f / SCALE_16)

// 定点数转换宏
#define FLOAT_TO_FIX(f)      ((int32_t)((f) * INV_SCALE_32))
#define FIX_TO_FLOAT(i)      ((float)(i) * SCALE_32)

#define FLOAT_TO_FIX16(f)    ((int16_t)((f) * INV_SCALE_16))
#define FIX16_TO_FLOAT(i)    ((float)(i) * SCALE_16)

/* ===================== 数据结构定义 ===================== */

// 关节状态数据包（8字节）
typedef struct __attribute__((packed)) {
    int32_t  position;      // 当前位置（定点）
    int32_t  velocity;      // 当前速度（定点）
} joint_status_t;

// 控制指令数据包（8字节）
typedef struct __attribute__((packed)) {
    int32_t  target_pos;    // 目标位置
    int16_t  target_cur;    // 目标电流（力矩模式）
    uint8_t  control_mode;  // 控制模式
    uint8_t  reserve;       // 保留（对齐）
} joint_control_t;

// 原始CAN帧（用于队列）
typedef struct {
    uint32_t id;            // 扩展帧ID
    uint8_t  data[64];      // CAN FD最大64字节
    uint8_t  len;           // 实际数据长度
    bool     is_extended;   // 是否为扩展帧
} can_frame_t;

typedef struct {
    uint16_t tx_error_cnt;
    uint8_t  rx_error_cnt;
    uint8_t  last_error_code;
    uint8_t  data_last_error_code;
    uint8_t  error_passive;
    uint8_t  warning;
    uint8_t  bus_off;
    uint32_t restart_count;
    uint32_t tx_fail_count;
    uint32_t last_error_status;
} can_diagnostics_t;

extern joint_control_t joint_control;

/* ===================== FDCAN配置 ===================== */

// FDCAN 170MHz时钟:
// 仲裁段 1Mbps : 170 / (10 * (1 + 13 + 3)) = 1M
// 数据段 5Mbps : 170 / (2  * (1 + 13 + 3)) = 5M
#define CAN_NOMINAL_PRESCALER      10U
#define CAN_NOMINAL_SYNC_JUMP      2U
#define CAN_NOMINAL_TIMESEG1       13U
#define CAN_NOMINAL_TIMESEG2       3U
// Data phase actual bitrate: 170 MHz / (5 * (1 + 13 + 3)) = 2 Mbps
#define CAN_DATA_PRESCALER         5U
#define CAN_DATA_SYNC_JUMP         2U
#define CAN_DATA_TIMESEG1          13U
#define CAN_DATA_TIMESEG2          3U

// CAN FD+BRS高速数据相位建议开启发送延迟补偿(TDC)
// ST常用推荐值: TDCO = DataTimeSeg1 * DataPrescaler

// 接收队列大小（根据内存调整，64字节×16=1KB）
#define FDCAN_RX_QUEUE_SIZE     64
#define FDCAN_TX_QUEUE_SIZE     64
// 每次 FDCAN_ProcessRxQueue 调用最多处理的普通队列帧数（避免主循环抖动）
#define FDCAN_RX_PROCESS_BUDGET 8U
// 每次 FDCAN_ProcessRxQueue 调用最多处理的控制帧数（latest mailbox）
#define FDCAN_CTRL_PROCESS_BUDGET 1U

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
void FDCAN_Service(void);
void FDCAN_GetDiagnostics(can_diagnostics_t *diag);

// 工具函数
uint8_t FDCAN_GetTxFreeLevel(void);
uint32_t FDCAN_GetRxDropCount(void);

#endif /* __FDCAN_H__ */
