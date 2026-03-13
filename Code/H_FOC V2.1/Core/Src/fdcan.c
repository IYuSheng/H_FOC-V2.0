#include "fdcan.h"
#include "string.h"
#include "stdio.h"

joint_control_t joint_control;

/* ===================== 私有变量 ===================== */

FDCAN_HandleTypeDef hfdcan1;
static FDCAN_TxHeaderTypeDef tx_header;
static uint8_t tx_data[64];  // 发送缓冲区

// 接收环形队列
static volatile can_frame_t rx_queue[FDCAN_RX_QUEUE_SIZE];
static volatile uint8_t rx_write_idx = 0;
static volatile uint8_t rx_read_idx = 0;
static volatile uint32_t rx_drop_count = 0;
static volatile uint32_t fdcan_restart_count = 0;
static volatile uint32_t fdcan_tx_fail_count = 0;
static volatile uint32_t fdcan_last_error_status = 0;
static volatile uint8_t fdcan_restart_requested = 0;
// Latest control frame mailbox: always keep the newest control command.
static volatile can_frame_t ctrl_mailbox;
static volatile uint8_t ctrl_mailbox_valid = 0U;

/* ===================== 内部函数 ===================== */

static HAL_StatusTypeDef FDCAN_EnableNotifications(void)
{
    return HAL_FDCAN_ActivateNotification(
        &hfdcan1,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
        FDCAN_IT_BUS_OFF |
        FDCAN_IT_ERROR_WARNING |
        FDCAN_IT_ERROR_PASSIVE |
        FDCAN_IT_ARB_PROTOCOL_ERROR |
        FDCAN_IT_DATA_PROTOCOL_ERROR,
        0);
}

static void FDCAN_CheckProtocolHealth(void)
{
    FDCAN_ProtocolStatusTypeDef protocol_status;

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) != HAL_OK) {
        return;
    }

    if (protocol_status.BusOff != 0U) {
        fdcan_restart_requested = 1U;
    }
}

/**
 * @brief 长度转DLC
 */
static uint32_t len_to_dlc(uint8_t len)
{
    if (len <= 8) return len;
    else if (len <= 12) return FDCAN_DLC_BYTES_12;
    else if (len <= 16) return FDCAN_DLC_BYTES_16;
    else if (len <= 20) return FDCAN_DLC_BYTES_20;
    else if (len <= 24) return FDCAN_DLC_BYTES_24;
    else if (len <= 32) return FDCAN_DLC_BYTES_32;
    else if (len <= 48) return FDCAN_DLC_BYTES_48;
    else return FDCAN_DLC_BYTES_64;
}

/**
 * @brief DLC转长度
 */
static uint8_t dlc_to_len(uint32_t dlc)
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
 * @brief Store latest control frame (overwrite old command).
 */
static void FDCAN_StoreLatestControl(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data)
{
    can_frame_t *frame = (can_frame_t *)&ctrl_mailbox;
    uint8_t len = dlc_to_len(header->DataLength);

    if (len > 64U) {
        len = 64U;
    }

    frame->id = header->Identifier;
    frame->is_extended = (header->IdType == FDCAN_EXTENDED_ID);
    frame->len = len;
    if (len > 0U) {
        memcpy((void *)frame->data, data, len);
    }

    ctrl_mailbox_valid = 1U;
}

/* ===================== 初始化函数 ===================== */

void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance = FDCAN1;
    
    // 时钟配置：170MHz（假设PCLK1=170M）
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    
    // 启用 CAN FD + BRS（仲裁段 1Mbps，数据段 2Mbps）
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    
    // 仲裁域：1Mbps
    hfdcan1.Init.NominalPrescaler = CAN_NOMINAL_PRESCALER;
    hfdcan1.Init.NominalSyncJumpWidth = CAN_NOMINAL_SYNC_JUMP;
    hfdcan1.Init.NominalTimeSeg1 = CAN_NOMINAL_TIMESEG1;
    hfdcan1.Init.NominalTimeSeg2 = CAN_NOMINAL_TIMESEG2;
    
    // 数据域：2Mbps
    hfdcan1.Init.DataPrescaler = CAN_DATA_PRESCALER;
    hfdcan1.Init.DataSyncJumpWidth = CAN_DATA_SYNC_JUMP;
    hfdcan1.Init.DataTimeSeg1 = CAN_DATA_TIMESEG1;
    hfdcan1.Init.DataTimeSeg2 = CAN_DATA_TIMESEG2;
    
    // 过滤器配置
    hfdcan1.Init.StdFiltersNbr = 2;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    debug_log("[CAN] INIT nominal=1M data=2M npre=%lu dpre=%lu nts1=%lu nts2=%lu dts1=%lu dts2=%lu",
              (unsigned long)CAN_NOMINAL_PRESCALER,
              (unsigned long)CAN_DATA_PRESCALER,
              (unsigned long)CAN_NOMINAL_TIMESEG1,
              (unsigned long)CAN_NOMINAL_TIMESEG2,
              (unsigned long)CAN_DATA_TIMESEG1,
              (unsigned long)CAN_DATA_TIMESEG2);
    
    FDCAN_Config_Filter();
    FDCAN_Start();
}

/**
 * @brief 配置接收过滤器
 */
void FDCAN_Config_Filter(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    
    /* ========== 过滤器0：接收目标为本节点的帧 ========== */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = ((uint32_t)JOINT_ID << 4);  // Dst = 本节点
    sFilterConfig.FilterID2 = 0x0F0;                      // 只检查Dst
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    /* ========== 过滤器1：接收广播帧 ========== */
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterID1 = ((uint32_t)JOINT_ID_BROADCAST << 4);  // Dst = 0x0F
    sFilterConfig.FilterID2 = 0x0F0;
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    /* ========== 全局过滤器 ========== */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
                                FDCAN_REJECT,           // 拒绝非匹配标准帧
                                FDCAN_REJECT,           // 拒绝非匹配扩展帧
                                FDCAN_REJECT_REMOTE,    // 拒绝标准远程帧
                                FDCAN_REJECT_REMOTE);   // 拒绝扩展远程帧
}

void FDCAN_Start(void)
{
    // 启动FDCAN
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    
    // 开启FIFO0新消息中断
    if (FDCAN_EnableNotifications() != HAL_OK) {
        Error_Handler();
    }
}

/* ===================== 发送函数 ===================== */

/**
 * @brief 通用数据包发送（支持64字节）
 * @param src_id   源节点ID (0-15)
 * @param dst_id   目标节点ID (0-15, 0x0F为广播)
 * @param msg_type 消息类型 (MSG_TYPE_XXX)
 * @param data     数据指针
 * @param len      数据长度 (0-64字节)
 * @return 0:成功, -1:失败
 */
int8_t FDCAN_SendPacket(uint8_t src_id, uint8_t dst_id, uint8_t msg_type,
                        uint8_t *data, uint8_t len)
{
    FDCAN_ProtocolStatusTypeDef ps = {0};
    FDCAN_ErrorCountersTypeDef ec = {0};
    uint32_t free_level;
    uint32_t can_id;

    if (len > 64) return -1;

    can_id = CAN_ID_MAKE(src_id, dst_id, msg_type);

    free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    if (free_level == 0U) {
        HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
        HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
        debug_log("[CAN] FIFO FULL id=0x%03lX len=%u free=%lu lec=%lu dlec=%lu ep=%lu warn=%lu bo=%lu tec=%lu rec=%lu",
                  (unsigned long)can_id,
                  (unsigned int)len,
                  (unsigned long)free_level,
                  (unsigned long)ps.LastErrorCode,
                  (unsigned long)ps.DataLastErrorCode,
                  (unsigned long)ps.ErrorPassive,
                  (unsigned long)ps.Warning,
                  (unsigned long)ps.BusOff,
                  (unsigned long)ec.TxErrorCnt,
                  (unsigned long)ec.RxErrorCnt);
        return -1;
    }

    tx_header.Identifier = can_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = len_to_dlc(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (len > 0 && data != NULL) {
        memcpy(tx_data, data, len);
    }

    if (len < 64 && len > 8) {
        memset(&tx_data[len], 0, 64 - len);
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) != HAL_OK) {
        free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
        HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
        HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
        debug_log("[CAN] TX FAIL id=0x%03lX len=%u free=%lu lec=%lu dlec=%lu ep=%lu warn=%lu bo=%lu tec=%lu rec=%lu",
                  (unsigned long)can_id,
                  (unsigned int)len,
                  (unsigned long)free_level,
                  (unsigned long)ps.LastErrorCode,
                  (unsigned long)ps.DataLastErrorCode,
                  (unsigned long)ps.ErrorPassive,
                  (unsigned long)ps.Warning,
                  (unsigned long)ps.BusOff,
                  (unsigned long)ec.TxErrorCnt,
                  (unsigned long)ec.RxErrorCnt);
        return -1;
    }

    return 0;
}

/**
 * @brief 发送关节状态（广播自身状态）
 * @param joint_id 本关节ID
 * @param status   状态数据结构
 * @return 0:成功
 */
int8_t FDCAN_SendJointStatus(uint8_t joint_id, joint_status_t *status)
{
    int8_t ret = FDCAN_SendPacket(joint_id, JOINT_ID_BROADCAST, MSG_TYPE_STATUS,
                           (uint8_t*)status, sizeof(joint_status_t));

    return ret;
}

/**
 * @brief 发送关节状态（发送给主节点）
 * @param joint_id 本关节ID
 * @param status   状态数据结构
 * @return 0:成功
 */
int8_t FDCAN_SendJointStatus_Master(uint8_t joint_id, joint_status_t *status)
{
    int8_t ret = FDCAN_SendPacket(joint_id, JOINT_ID_MASTER, MSG_TYPE_STATUS,
                           (uint8_t*)status, sizeof(joint_status_t));

    return ret;
}

/**
 * @brief 发送控制指令（主节点调用）
 * @param dst_id 目标关节ID
 * @param cmd    控制指令结构
 * @return 0:成功
 */
int8_t FDCAN_SendControlCmd(uint8_t dst_id, joint_control_t *cmd)
{
    int8_t rc;
    // 只有主节点才能发送控制指令
    if(JOINT_ID == JOINT_ID_MASTER)
    {
        rc = FDCAN_SendPacket(JOINT_ID_MASTER, dst_id, MSG_TYPE_CONTROL,
                            (uint8_t*)cmd, sizeof(joint_control_t));
    }
    else
    {
        rc = -1;
    }
    return rc;
}

/* ===================== 接收处理 ===================== */

/**
 * @brief 接收中断回调（ISR中调用）
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN1) return;
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[64];
        uint8_t msg_type;

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
            break;
        }

        msg_type = CAN_ID_GET_TYPE(header.Identifier);
        if (msg_type == MSG_TYPE_CONTROL) {
            FDCAN_StoreLatestControl(&header, data);
            continue;
        }

        uint8_t next_write = (uint8_t)((rx_write_idx + 1U) % FDCAN_RX_QUEUE_SIZE);
        if (next_write == rx_read_idx) {
            rx_read_idx = (uint8_t)((rx_read_idx + 1U) % FDCAN_RX_QUEUE_SIZE);
            rx_drop_count++;
        }

        can_frame_t *frame = (can_frame_t *)&rx_queue[rx_write_idx];
        frame->id = header.Identifier;
        frame->is_extended = (header.IdType == FDCAN_EXTENDED_ID);
        frame->len = dlc_to_len(header.DataLength);
        if (frame->len > 64U) {
            frame->len = 64U;
        }
        memcpy((void *)frame->data, data, frame->len);
        rx_write_idx = next_write;
    }
}

/**
 * @brief 主循环调用，处理接收队列
 * @note 非ISR环境，可安全处理复杂逻辑
 */
void FDCAN_ProcessRxQueue(void)
{
    uint32_t processed = 0U;
    can_frame_t latest_ctrl_frame;
    uint8_t has_latest_ctrl = 0U;
    extern uint8_t g_local_node_id;

    while ((rx_read_idx != rx_write_idx) && (processed < FDCAN_RX_PROCESS_BUDGET)) {
        can_frame_t frame;
        memcpy(&frame, (void *)&rx_queue[rx_read_idx], sizeof(can_frame_t));
        rx_read_idx = (uint8_t)((rx_read_idx + 1U) % FDCAN_RX_QUEUE_SIZE);
        processed++;

        uint8_t dst = CAN_ID_GET_DST(frame.id);
        uint8_t type = CAN_ID_GET_TYPE(frame.id);

        if (dst != JOINT_ID_BROADCAST && dst != g_local_node_id) {
            continue;
        }

        if (type == MSG_TYPE_CONTROL) {
            latest_ctrl_frame = frame;
            has_latest_ctrl = 1U;
            continue;
        }

        FDCAN_RxCallback(&frame);
    }

    if (FDCAN_CTRL_PROCESS_BUDGET > 0U) {
        __disable_irq();
        if (ctrl_mailbox_valid != 0U) {
            memcpy(&latest_ctrl_frame, (const void *)&ctrl_mailbox, sizeof(can_frame_t));
            ctrl_mailbox_valid = 0U;
            has_latest_ctrl = 1U;
        }
        __enable_irq();
    }

    if (has_latest_ctrl != 0U) {
        uint8_t dst = CAN_ID_GET_DST(latest_ctrl_frame.id);
        if (dst == JOINT_ID_BROADCAST || dst == g_local_node_id) {
            FDCAN_RxCallback(&latest_ctrl_frame);
        }
    }
}

void FDCAN_Service(void)
{
    if (fdcan_restart_requested == 0U) {
        return;
    }

    fdcan_restart_requested = 0U;

    (void)HAL_FDCAN_Stop(&hfdcan1);

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        fdcan_restart_requested = 1U;
        return;
    }

    if (FDCAN_EnableNotifications() != HAL_OK) {
        fdcan_restart_requested = 1U;
        return;
    }

    fdcan_restart_count++;
}

/* ===================== 工具函数 ===================== */

uint8_t FDCAN_GetTxFreeLevel(void)
{
    return HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
}

uint32_t FDCAN_GetRxDropCount(void)
{
    return rx_drop_count;
}

void FDCAN_GetDiagnostics(can_diagnostics_t *diag)
{
    FDCAN_ProtocolStatusTypeDef protocol_status = {0};
    FDCAN_ErrorCountersTypeDef error_counters = {0};

    if (diag == NULL) {
        return;
    }

    memset(diag, 0, sizeof(*diag));

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) == HAL_OK) {
        diag->last_error_code = (uint8_t)protocol_status.LastErrorCode;
        diag->data_last_error_code = (uint8_t)protocol_status.DataLastErrorCode;
        diag->error_passive = (uint8_t)protocol_status.ErrorPassive;
        diag->warning = (uint8_t)protocol_status.Warning;
        diag->bus_off = (uint8_t)protocol_status.BusOff;
    }

    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) == HAL_OK) {
        diag->tx_error_cnt = (uint16_t)error_counters.TxErrorCnt;
        diag->rx_error_cnt = (uint8_t)error_counters.RxErrorCnt;
    }

    diag->restart_count = fdcan_restart_count;
    diag->tx_fail_count = fdcan_tx_fail_count;
    diag->last_error_status = fdcan_last_error_status;
}

__attribute__((weak)) void FDCAN_RxCallback(can_frame_t *frame)
{
    (void)frame;
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    fdcan_last_error_status = ErrorStatusITs;

    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U) {
        fdcan_restart_requested = 1U;
    }
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
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 3, 0);
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
