#ifndef __FDCAN_H__
#define __FDCAN_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ===================== FDCAN 协议定义 ===================== */

/* 本机关节 ID，用于组成标准帧 ID 的源地址字段 */
#define JOINT_ID            2U

/* 关节 ID 范围：0x0 ~ 0xE，0x0F 保留为广播地址 */
#define JOINT_ID_MAX        15U
#define JOINT_ID_MASTER     0x00U   /* 主控节点 */
#define JOINT_ID_BROADCAST  0x0FU   /* 广播地址 */

/* 消息类型，占标准帧 ID 的低 4 bit */
#define MSG_TYPE_STATUS     0x0U    /* 关节状态上报 */
#define MSG_TYPE_CONTROL    0x1U    /* 关节控制指令 */
#define MSG_TYPE_PARAM      0x2U    /* 参数配置 */
#define MSG_TYPE_DEBUG      0x3U    /* 调试数据 */
#define MSG_TYPE_ERROR      0x4U    /* 故障/错误信息 */
#define MSG_TYPE_SYNC       0x5U    /* 多节点同步触发 */

/* 控制模式定义 */
#define CONTROL_MODE_NORMAL      0U
#define CONTROL_MODE_TRAPEZOIDAL 1U
#define CONTROL_MODE_S_CURVE     2U

/* 标准帧 ID 组织方式：bit[10:8]=src，bit[7:4]=dst，bit[3:0]=type */
#define CAN_ID_MAKE(src, dst, type) (((src) << 8) | ((dst) << 4) | (type))
#define CAN_ID_GET_SRC(id)          (((id) >> 8) & 0x0F)
#define CAN_ID_GET_DST(id)          (((id) >> 4) & 0x0F)
#define CAN_ID_GET_TYPE(id)         ((id) & 0x0F)

/* ===================== 定点缩放定义 ===================== */

/* 当前协议默认所有 int32 字段都按 1e-5 的物理量缩放 */
#define SCALE_32          0.00001f
#define INV_SCALE_32      (1.0f / SCALE_32)

/* 浮点 <-> 定点换算宏 */
#define FLOAT_TO_FIX(f)      ((int32_t)((f) * INV_SCALE_32))
#define FIX_TO_FLOAT(i)      ((float)(i) * SCALE_32)

/* ===================== 协议负载结构 ===================== */

/* 关节状态帧负载：5 个 int32，共 20 字节 */
typedef struct __attribute__((packed)) {
    int32_t position;       /* 当前机械角度 */
    int32_t velocity;       /* 当前机械角速度 */
    int32_t current;        /* 当前 q 轴电流 */
    int32_t temperature;    /* 当前温度 */
    int32_t acceleration;   /* 当前机械角加速度 */
} joint_status_t;

/* 控制指令帧负载：7 个 int32 + 1 个 uint8，实际 29 字节，CAN FD DLC 会占用 32 字节 */
typedef struct __attribute__((packed)) {
    int32_t target_current;      /* 目标电流 / 前馈力矩电流 */
    int32_t target_angle;        /* 目标机械角度 */
    int32_t target_velocity;     /* 目标机械角速度 */
    int32_t target_acceleration; /* 目标机械角加速度 */
    int32_t target_jerk;         /* 目标机械角加加速度 */
    int32_t stiffness;           /* 刚度系数 */
    int32_t damping;             /* 阻尼系数 */
    uint8_t control_mode;        /* 控制模式 */
} joint_control_t;

/* 接收队列中使用的原始 CAN FD 帧缓存结构 */
typedef struct {
    uint32_t id;            /* 标准帧 ID 或扩展帧 ID */
    uint8_t  data[64];      /* CAN FD 最大 64 字节数据区 */
    uint8_t  len;           /* 当前有效数据长度 */
    bool     is_extended;   /* 是否为扩展帧 */
} can_frame_t;

/* FDCAN 诊断信息，便于调试总线状态 */
typedef struct {
    uint16_t tx_error_cnt;         /* 发送错误计数 */
    uint8_t  rx_error_cnt;         /* 接收错误计数 */
    uint8_t  last_error_code;      /* 仲裁阶段最后错误码 */
    uint8_t  data_last_error_code; /* 数据阶段最后错误码 */
    uint8_t  error_passive;        /* 是否进入 error passive */
    uint8_t  warning;              /* 是否进入 warning 状态 */
    uint8_t  bus_off;              /* 是否 bus-off */
    uint32_t restart_count;        /* 总线重启次数 */
    uint32_t tx_fail_count;        /* 发送失败次数 */
    uint32_t last_error_status;    /* 最近一次 HAL 错误状态位 */
} can_diagnostics_t;

/* 保存最近一次接收到的控制指令原始负载 */
extern joint_control_t joint_control;

/* ===================== FDCAN 位时序与队列配置 ===================== */

/* 仲裁段位时序配置 */
#define CAN_NOMINAL_PRESCALER      10U
#define CAN_NOMINAL_SYNC_JUMP      2U
#define CAN_NOMINAL_TIMESEG1       13U
#define CAN_NOMINAL_TIMESEG2       3U

/* 数据段位时序配置 */
#define CAN_DATA_PRESCALER         5U
#define CAN_DATA_SYNC_JUMP         2U
#define CAN_DATA_TIMESEG1          13U
#define CAN_DATA_TIMESEG2          3U

/* 软件接收/发送缓存配置 */
#define FDCAN_RX_QUEUE_SIZE        64U
#define FDCAN_TX_QUEUE_SIZE        64U
#define FDCAN_RX_PROCESS_BUDGET    8U  /* 每次主循环最多处理的普通接收帧数 */
#define FDCAN_CTRL_PROCESS_BUDGET  1U  /* 每次主循环最多处理的控制帧数 */

/* ===================== 对外接口 ===================== */

/* FDCAN 底层初始化与启动 */
void MX_FDCAN1_Init(void);
void FDCAN_Config_Filter(void);
void FDCAN_Start(void);

/* 通用发包接口及协议封装接口 */
int8_t FDCAN_SendPacket(uint8_t src_id, uint8_t dst_id, uint8_t msg_type,
                        uint8_t *data, uint8_t len);
int8_t FDCAN_SendJointStatus(uint8_t joint_id, joint_status_t *status);
int8_t FDCAN_SendJointStatus_Master(uint8_t joint_id, joint_status_t *status);
int8_t FDCAN_SendControlCmd(uint8_t dst_id, joint_control_t *cmd);

/* 接收处理与服务函数 */
void FDCAN_ProcessRxQueue(void);
void FDCAN_RxCallback(can_frame_t *frame);
void FDCAN_Service(void);
void FDCAN_GetDiagnostics(can_diagnostics_t *diag);

/* 诊断辅助接口 */
uint8_t FDCAN_GetTxFreeLevel(void);
uint32_t FDCAN_GetRxDropCount(void);

#endif /* __FDCAN_H__ */
