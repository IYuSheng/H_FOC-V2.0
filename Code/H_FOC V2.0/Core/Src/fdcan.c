#include "fdcan.h"
#include "foc_prase.h"
#include <string.h>
#include <stdio.h>

FDCAN_TxHeaderTypeDef g_fdcan1_tx_header;
FDCAN_RxHeaderTypeDef g_fdcan1_rx_header;
uint8_t g_fdcan1_tx_data[16] = {0x00};
uint8_t g_fdcan1_rx_data[16] = {0x00};
uint8_t g_fdcan1_rx_flag = 0;

// ===================== CAN 接收队列定义 =====================
// 定义接收队列项结构体
typedef struct {
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
    uint8_t length;
} fdcan_rx_queue_item_t;

// 接收队列变量
static volatile uint8_t g_fdcan_rx_queue_write_idx = 0;
static volatile uint8_t g_fdcan_rx_queue_read_idx = 0;
static fdcan_rx_queue_item_t g_fdcan_rx_queue[FDCAN_RX_QUEUE_SIZE];

// ===================== CAN 接收数据解析缓冲区 =====================

static char g_fdcan_parse_buffer[FDCAN_RX_PARSE_BUFFER_SIZE];
static uint16_t g_fdcan_parse_buffer_idx = 0;

// 统计变量
static volatile uint32_t g_fdcan_rx_queue_drop_count = 0;
static volatile uint32_t g_fdcan_parse_overflow_count = 0;

/**
 * @brief 计算下一个队列索引
 * @param current_idx 当前索引
 * @return 下一个索引值
 */
static inline uint8_t fdcan_get_next_queue_index(uint8_t current_idx) 
{ 
    return (uint8_t)((current_idx + 1) % FDCAN_RX_QUEUE_SIZE); 
}

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{
  hfdcan1.Instance = FDCAN1;

  //========================================================
  // 0) FDCAN kernel clock: 170MHz (你已确认)
  //    ClockDivider=DIV1 表示不分频，直接用 170MHz
  //========================================================
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;

  //========================================================
  // 1) 关键：启用 CAN FD + BRS
  //    - FDCAN_FRAME_CLASSIC：经典 CAN（你原来是这个）
  //    - FDCAN_FRAME_FD_NO_BRS：CAN FD 但数据域不加速
  //    - FDCAN_FRAME_FD_BRS：CAN FD + 数据域加速（推荐）
  //========================================================
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;

  // 正常工作模式
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;

  //========================================================
  // 2) 自动重发建议 ENABLE（更符合 CAN 行为，调试更省心）
  //    如果你做实时控制且希望"发送失败立即返回"，再 DISABLE
  //========================================================
  hfdcan1.Init.AutoRetransmission = ENABLE;

  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;

  //========================================================
  // 3) Nominal (仲裁域) 配置：目标 1Mbps
  //
  // NominalBitRate = 170MHz / (NominalPrescaler * (1 + Seg1 + Seg2))
  //
  // 选择：
  //   NominalPrescaler = 10
  //   (1 + Seg1 + Seg2) = 17  -> 170MHz/(10*17)=1MHz = 1Mbps
  //
  // 采样点：SP = (1+Seg1)/17
  //   Seg1=13, Seg2=3 -> SP=14/17=82.35%
  // SJW <= Seg2，取 2
  //========================================================
  hfdcan1.Init.NominalPrescaler     = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1      = 13;
  hfdcan1.Init.NominalTimeSeg2      = 3;

  //========================================================
  // 4) Data (数据域) 配置：目标 5Mbps（BRS 开启后生效）
  //
  // DataBitRate = 170MHz / (DataPrescaler * (1 + Seg1 + Seg2))
  //
  // 选择：
  //   DataPrescaler = 2
  //   (1 + Seg1 + Seg2) = 17 -> 170MHz/(2*17)=5MHz = 5Mbps
  //
  // 同样采样点约 82.35%
  // SJW <= Seg2，取 2
  //========================================================
  hfdcan1.Init.DataPrescaler     = 2;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1      = 13;
  hfdcan1.Init.DataTimeSeg2      = 3;

  //========================================================
  // 5) 过滤器数量：你用了 FilterIndex=0，所以必须 >=1
  //========================================================
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;

  // TX FIFO 模式
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  //==================== USER CODE: Filter/TxHeader ====================

  // 6) 配置接收过滤器（标准帧，Mask 模式）
  //    FilterID1=0x002, Mask=0x7FF 表示只接收 ID == 0x002
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x002;
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // 7) 配置发送头（注意：FD + BRS）
  g_fdcan1_tx_header.Identifier = 0x001;
  g_fdcan1_tx_header.IdType = FDCAN_STANDARD_ID;
  g_fdcan1_tx_header.TxFrameType = FDCAN_DATA_FRAME;

  // 你现在发送 8 字节：OK
  g_fdcan1_tx_header.DataLength = FDCAN_DLC_BYTES_8;

  g_fdcan1_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;

  // 不开启 BRS
  g_fdcan1_tx_header.BitRateSwitch = FDCAN_BRS_OFF;

  // 设置为 CAN FD 帧
  g_fdcan1_tx_header.FDFormat = FDCAN_FD_CAN;

  g_fdcan1_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  g_fdcan1_tx_header.MessageMarker = 0;

  // 8) 启动 FDCAN
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  // 9) 开启 FIFO0 新消息中断
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  g_fdcan1_rx_flag = 0;
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */
    LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PCLK1);

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
 
/**
 * @brief 发送FDCAN数据
 * @param tx_data 要发送的数据指针
 */
void my_FDCAN1_Transmit(uint8_t *tx_data)
{
    // 发送数据
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &g_fdcan1_tx_header, tx_data) != HAL_OK)
    {
        debug_log("HAL_FDCAN_AddMessageToTxFifoQ failed\r\n");
        Error_Handler();
    }
}

/**
 * @brief 发送四个浮点数数据
 * @param data1 第一个浮点数
 * @param data2 第二个浮点数
 * @param data3 第三个浮点数
 * @param data4 第四个浮点数
 */
void FDCAN_SendFloat4Data(float data1, float data2, float data3, float data4)
{
    // 将4个浮点数打包到8字节数据中
    // 每个浮点数占用2字节（使用16位定点表示）
    // 将浮点数乘以10000再转换为整数，以保留小数点后4位精度
    int16_t int_data1 = (int16_t)(data1 * 10000);
    int16_t int_data2 = (int16_t)(data2 * 10000);
    int16_t int_data3 = (int16_t)(data3 * 10000);
    int16_t int_data4 = (int16_t)(data4 * 10000);
    
    uint8_t tx_data[8];
    
    // 将16位整数拆分为8个字节
    tx_data[0] = (uint8_t)(int_data1 & 0xFF);
    tx_data[1] = (uint8_t)((int_data1 >> 8) & 0xFF);
    tx_data[2] = (uint8_t)(int_data2 & 0xFF);
    tx_data[3] = (uint8_t)((int_data2 >> 8) & 0xFF);
    tx_data[4] = (uint8_t)(int_data3 & 0xFF);
    tx_data[5] = (uint8_t)((int_data3 >> 8) & 0xFF);
    tx_data[6] = (uint8_t)(int_data4 & 0xFF);
    tx_data[7] = (uint8_t)((int_data4 >> 8) & 0xFF);
    
    my_FDCAN1_Transmit(tx_data);
}

/**
 * @brief 将FDCAN DLC值转换为实际数据长度
 * @param dlc FDCAN DLC值
 * @return 对应的实际数据长度
 */
static inline uint8_t fdcan_dlc_to_length(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_0:  return 0;
    case FDCAN_DLC_BYTES_1:  return 1;
    case FDCAN_DLC_BYTES_2:  return 2;
    case FDCAN_DLC_BYTES_3:  return 3;
    case FDCAN_DLC_BYTES_4:  return 4;
    case FDCAN_DLC_BYTES_5:  return 5;
    case FDCAN_DLC_BYTES_6:  return 6;
    case FDCAN_DLC_BYTES_7:  return 7;
    case FDCAN_DLC_BYTES_8:  return 8;
    case FDCAN_DLC_BYTES_12: return 12;
    case FDCAN_DLC_BYTES_16: return 16;
    case FDCAN_DLC_BYTES_20: return 20;
    case FDCAN_DLC_BYTES_24: return 24;
    case FDCAN_DLC_BYTES_32: return 32;
    case FDCAN_DLC_BYTES_48: return 48;
    case FDCAN_DLC_BYTES_64: return 64;
    default: return 0;
    }
}
 
/**
 * @brief FDCAN1接收回调函数
 * @param hfdcan FDCAN句柄指针
 * @param rx_fifo0_its 接收FIFO0中断状态
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    if (hfdcan->Instance != FDCAN1) return;
    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
        return;
    }

    uint8_t length = fdcan_dlc_to_length(header.DataLength);

    uint8_t write_idx = g_fdcan_rx_queue_write_idx;
    uint8_t next_write_idx = fdcan_get_next_queue_index(write_idx);
    if (next_write_idx == g_fdcan_rx_queue_read_idx) {
        // 队列满了：丢包（这里计数）
        g_fdcan_rx_queue_drop_count++;
        return;
    }

    g_fdcan_rx_queue[write_idx].header = header;
    g_fdcan_rx_queue[write_idx].length = length;
    if (length > 0) memcpy(g_fdcan_rx_queue[write_idx].data, data, length);

    g_fdcan_rx_queue_write_idx = next_write_idx;  // 提交写指针
}

/**
 * @brief 在非中断环境中处理FDCAN接收队列
 * @note 应在主循环或定时任务中定期调用此函数
 */
void FDCAN1_ProcessRxQueue(void)
{
    while (g_fdcan_rx_queue_read_idx != g_fdcan_rx_queue_write_idx)
    {

        // 取出一条队列元素
        fdcan_rx_queue_item_t *queue_item = &g_fdcan_rx_queue[g_fdcan_rx_queue_read_idx];

        // 如果你只希望处理某个 ID，也可以在这里做二次过滤（可选）
        // 例如：只处理 0x002
        // if (queue_item->header.Identifier != 0x002) { g_fdcan_rx_queue_read_idx = fdcan_get_next_queue_index(g_fdcan_rx_queue_read_idx); continue; }

        // 把 payload 当作 ASCII 字符流，逐字节处理
        for (uint8_t i = 0; i < queue_item->length; i++)
        {
            char received_char = (char)queue_item->data[i];

            // 过滤掉 0 字节（有些人会补 0）
            if (received_char == '\0') {
                continue;
            }

            if (received_char == '\n') {
                // 一条命令结束：如果缓冲里有内容 -> 调 parse_received_data
                if (g_fdcan_parse_buffer_idx > 0)
                {
                    // 添加字符串结束符（便于你在 parse 里用字符串操作）
                    if (g_fdcan_parse_buffer_idx >= (FDCAN_RX_PARSE_BUFFER_SIZE - 1))
                    {
                        // 理论上不会到这，因为下面有溢出保护，这里再防一下
                        g_fdcan_parse_buffer_idx = FDCAN_RX_PARSE_BUFFER_SIZE - 1;
                    }
                    g_fdcan_parse_buffer[g_fdcan_parse_buffer_idx] = '\0';

                    // 调用你的解析函数（长度不包含 '\0'）
                    parse_received_data(g_fdcan_parse_buffer, (int)g_fdcan_parse_buffer_idx);

                    // 重置索引，准备下一条命令
                    g_fdcan_parse_buffer_idx = 0;
                }
            }
            else if (received_char == '\r')
            {
                // 忽略 '\r'，兼容 "\r\n"
                continue;
            }
            else
            {
                // 普通字符：写入解析缓冲
                if (g_fdcan_parse_buffer_idx < (FDCAN_RX_PARSE_BUFFER_SIZE - 1))
                {
                    g_fdcan_parse_buffer[g_fdcan_parse_buffer_idx++] = received_char;
                }
                else
                {
                    // 缓冲区满：直接丢弃当前累计（防止一直卡死）
                    g_fdcan_parse_overflow_count++;
                    g_fdcan_parse_buffer_idx = 0;
                }
            }
        }

        // 出队：移动读指针
        g_fdcan_rx_queue_read_idx = fdcan_get_next_queue_index(g_fdcan_rx_queue_read_idx);
    }
}
