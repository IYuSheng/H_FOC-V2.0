#ifndef FOC_COMMUNICATION_H
#define FOC_COMMUNICATION_H

#include "foc_prase.h"
#include "fdcan.h"
#include "foc_conversion.h"

extern uint8_t g_local_node_id;  // 本关节ID

/**
 * @brief FOC设置外部参数
 */
void foc_control_set(void);

/**
 * @brief 发送当前关节状态
 */
void CAN_ReportStatus(float pos, float vel);

/**
 * @brief 主循环调用（1ms周期）
 */
void CAN_Process(void);

#endif // FOC_COMMUNICATION_H
