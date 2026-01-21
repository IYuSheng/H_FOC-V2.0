#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_TxHeaderTypeDef g_fdcan1_tx_header;
extern FDCAN_RxHeaderTypeDef g_fdcan1_rx_header;
extern uint8_t g_fdcan1_tx_data[16];
extern uint8_t g_fdcan1_rx_data[16];
extern uint8_t g_fdcan1_rx_flag;

// 接收队列参数
#define FDCAN_RX_QUEUE_SIZE 32
// 用于将CAN数据转换为字符串格式进行解析
#define FDCAN_RX_PARSE_BUFFER_SIZE 256

void MX_FDCAN1_Init(void);

/**
 * @brief 发送FDCAN数据
 * @param tx_data 要发送的数据指针
 */
void my_FDCAN1_Transmit(uint8_t *tx_data);

/**
 * @brief 发送四个浮点数数据
 * @param data1 第一个浮点数
 * @param data2 第二个浮点数
 * @param data3 第三个浮点数
 * @param data4 第四个浮点数
 */
void FDCAN_SendFloat4Data(float data1, float data2, float data3, float data4);

/**
 * @brief 在非中断环境中处理FDCAN接收队列
 * @note 应在主循环或定时任务中定期调用此函数
 */
void FDCAN1_ProcessRxQueue(void);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */
