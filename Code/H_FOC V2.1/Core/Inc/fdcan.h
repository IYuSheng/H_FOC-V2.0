#ifndef __FDCAN_H__
#define __FDCAN_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Node and protocol settings */
#define JOINT_ID              2U
#define JOINT_ID_MAX          14U
#define JOINT_NODE_COUNT      (JOINT_ID_MAX + 1U)
#define JOINT_ID_MASTER       0x00U
#define JOINT_ID_BROADCAST    0x0FU

/* Message type (low 4 bits of CAN ID) */
#define MSG_TYPE_STATUS       0x0U
#define MSG_TYPE_CONTROL      0x1U
#define MSG_TYPE_PARAM        0x2U
#define MSG_TYPE_DEBUG        0x3U
#define MSG_TYPE_ERROR        0x4U
#define MSG_TYPE_SYNC         0x5U

/* 11-bit standard ID layout: [src(4)][dst(4)][type(4)] */
#define CAN_ID_MAKE(src, dst, type) \
    ((((uint32_t)(src) & 0x0FU) << 8) | (((uint32_t)(dst) & 0x0FU) << 4) | ((uint32_t)(type) & 0x0FU))
#define CAN_ID_GET_SRC(id)    (((uint32_t)(id) >> 8) & 0x0FU)
#define CAN_ID_GET_DST(id)    (((uint32_t)(id) >> 4) & 0x0FU)
#define CAN_ID_GET_TYPE(id)   ((uint32_t)(id) & 0x0FU)

/* Fixed-point scaling */
#define SCALE_32              0.00001f
#define INV_SCALE_32          (1.0f / SCALE_32)
#define SCALE_16              0.01f
#define INV_SCALE_16          (1.0f / SCALE_16)

#define FLOAT_TO_FIX(f)       ((int32_t)((f) * INV_SCALE_32))
#define FIX_TO_FLOAT(i)       ((float)(i) * SCALE_32)
#define FLOAT_TO_FIX16(f)     ((int16_t)((f) * INV_SCALE_16))
#define FIX16_TO_FLOAT(i)     ((float)(i) * SCALE_16)

/* Broadcast status payload (classic CAN 8 bytes) */
typedef struct __attribute__((packed)) {
    int32_t position;
    int32_t velocity;
} joint_status_t;

/* Control payload (currently 12 bytes, not single-frame compatible in classic CAN) */
typedef struct __attribute__((packed)) {
    int32_t target_pos;
    int32_t target_vel;
    int16_t target_cur;
    int16_t control_mode;
    uint16_t reserve;
} joint_control_t;

/* Parsed CAN frame stored in software RX queue */
typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    bool is_extended;
} can_frame_t;

/* Runtime diagnostics for fast health check */
typedef struct {
    uint8_t init_ok;
    uint8_t start_ok;
    uint32_t tx_ok_count;
    uint32_t tx_fail_count;
    uint32_t rx_irq_count;
    uint32_t rx_processed_count;
    uint32_t rx_drop_count;
    uint32_t last_tx_id;
    uint8_t last_tx_len;
    uint32_t last_rx_id;
    uint8_t last_rx_len;
} fdcan_diag_t;

extern FDCAN_HandleTypeDef hfdcan1;
extern joint_control_t joint_control;

#define FDCAN_RX_QUEUE_SIZE   16U
#define FDCAN_TX_QUEUE_SIZE   8U

void MX_FDCAN1_Init(void);
void FDCAN_Config_Filter(void);
void FDCAN_Start(void);

int8_t FDCAN_SendPacket(uint8_t src_id, uint8_t dst_id, uint8_t msg_type, uint8_t *data, uint8_t len);
int8_t FDCAN_SendJointStatus(uint8_t joint_id, joint_status_t *status);
int8_t FDCAN_SendControlCmd(uint8_t dst_id, joint_control_t *cmd);

void FDCAN_ProcessRxQueue(void);
void FDCAN_RxCallback(can_frame_t *frame);

uint8_t FDCAN_GetTxFreeLevel(void);
uint32_t FDCAN_GetRxDropCount(void);
void FDCAN_GetDiag(fdcan_diag_t *out_diag);
void FDCAN_ResetDiag(void);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */
