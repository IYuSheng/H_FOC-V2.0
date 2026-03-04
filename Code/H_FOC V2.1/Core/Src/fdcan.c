#include "fdcan.h"
#include "string.h"

joint_control_t joint_control;

/* Private state */
FDCAN_HandleTypeDef hfdcan1;
static FDCAN_TxHeaderTypeDef tx_header;
static uint8_t tx_data[8];

static volatile can_frame_t rx_queue[FDCAN_RX_QUEUE_SIZE];
static volatile uint8_t rx_write_idx = 0U;
static volatile uint8_t rx_read_idx = 0U;
static volatile uint32_t rx_drop_count = 0U;
static volatile fdcan_diag_t g_fdcan_diag = {0};

static uint32_t len_to_dlc(uint8_t len)
{
    return (len <= 8U) ? (uint32_t)len : FDCAN_DLC_BYTES_8;
}

static uint8_t dlc_to_len(uint32_t dlc)
{
    switch (dlc) {
        case FDCAN_DLC_BYTES_0: return 0U;
        case FDCAN_DLC_BYTES_1: return 1U;
        case FDCAN_DLC_BYTES_2: return 2U;
        case FDCAN_DLC_BYTES_3: return 3U;
        case FDCAN_DLC_BYTES_4: return 4U;
        case FDCAN_DLC_BYTES_5: return 5U;
        case FDCAN_DLC_BYTES_6: return 6U;
        case FDCAN_DLC_BYTES_7: return 7U;
        case FDCAN_DLC_BYTES_8: return 8U;
        default: return 0U;
    }
}

void MX_FDCAN1_Init(void)
{
    memset((void *)&g_fdcan_diag, 0, sizeof(g_fdcan_diag));
    rx_write_idx = 0U;
    rx_read_idx = 0U;
    rx_drop_count = 0U;

    hfdcan1.Instance = FDCAN1;

    /* FDCAN kernel clock assumed 170 MHz from PCLK1 */
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;

    /* Classic CAN mode */
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;

    /* 1 Mbps nominal bit timing: 170 MHz / (10 * (1 + 13 + 3)) */
    hfdcan1.Init.NominalPrescaler = 10;
    hfdcan1.Init.NominalSyncJumpWidth = 2;
    hfdcan1.Init.NominalTimeSeg1 = 13;
    hfdcan1.Init.NominalTimeSeg2 = 3;

    /* Data timing fields are ignored in classic CAN, keep aligned */
    hfdcan1.Init.DataPrescaler = 10;
    hfdcan1.Init.DataSyncJumpWidth = 2;
    hfdcan1.Init.DataTimeSeg1 = 13;
    hfdcan1.Init.DataTimeSeg2 = 3;

    hfdcan1.Init.StdFiltersNbr = 2;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    g_fdcan_diag.init_ok = 1U;

    FDCAN_Config_Filter();
    FDCAN_Start();
}

void FDCAN_Config_Filter(void)
{
    FDCAN_FilterTypeDef filter;

    /* Filter 0: unicast to local node (match dst nibble only) */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = ((uint32_t)JOINT_ID << 4);
    filter.FilterID2 = 0x0F0U;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
        Error_Handler();
    }

    /* Filter 1: broadcast to dst=0x0F */
    filter.FilterIndex = 1;
    filter.FilterID1 = ((uint32_t)JOINT_ID_BROADCAST << 4);
    filter.FilterID2 = 0x0F0U;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
        Error_Handler();
    }

    /* Reject unmatched frames and remote frames */
    (void)HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );
}

void FDCAN_Start(void)
{
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }

    g_fdcan_diag.start_ok = 1U;
}

int8_t FDCAN_SendPacket(uint8_t src_id, uint8_t dst_id, uint8_t msg_type, uint8_t *data, uint8_t len)
{
    if (len > sizeof(tx_data)) {
        g_fdcan_diag.tx_fail_count++;
        return -1;
    }

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
        g_fdcan_diag.tx_fail_count++;
        return -1;
    }

    tx_header.Identifier = CAN_ID_MAKE(src_id, dst_id, msg_type);
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = len_to_dlc(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    memset(tx_data, 0, sizeof(tx_data));
    if ((len > 0U) && (data != NULL)) {
        memcpy(tx_data, data, len);
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) != HAL_OK) {
        g_fdcan_diag.tx_fail_count++;
        return -1;
    }

    g_fdcan_diag.tx_ok_count++;
    g_fdcan_diag.last_tx_id = tx_header.Identifier;
    g_fdcan_diag.last_tx_len = len;

    return 0;
}

int8_t FDCAN_SendJointStatus(uint8_t joint_id, joint_status_t *status)
{
    return FDCAN_SendPacket(
        joint_id,
        JOINT_ID_BROADCAST,
        MSG_TYPE_STATUS,
        (uint8_t *)status,
        (uint8_t)sizeof(joint_status_t)
    );
}

int8_t FDCAN_SendControlCmd(uint8_t dst_id, joint_control_t *cmd)
{
    if (JOINT_ID != JOINT_ID_MASTER) {
        return -1;
    }

    /* joint_control_t is 12 bytes, so this path is not single-frame classic CAN compatible. */
    if (sizeof(joint_control_t) > sizeof(tx_data)) {
        return -1;
    }

    return FDCAN_SendPacket(
        JOINT_ID_MASTER,
        dst_id,
        MSG_TYPE_CONTROL,
        (uint8_t *)cmd,
        (uint8_t)sizeof(joint_control_t)
    );
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }
    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8] = {0};
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
        return;
    }

    uint8_t next_write = (uint8_t)((rx_write_idx + 1U) % FDCAN_RX_QUEUE_SIZE);
    if (next_write == rx_read_idx) {
        rx_drop_count++;
        g_fdcan_diag.rx_drop_count = rx_drop_count;
        return;
    }

    can_frame_t *frame = (can_frame_t *)&rx_queue[rx_write_idx];
    frame->id = header.Identifier;
    frame->is_extended = (header.IdType == FDCAN_EXTENDED_ID);

    uint8_t payload_len = dlc_to_len(header.DataLength);
    if (payload_len > sizeof(frame->data)) {
        payload_len = (uint8_t)sizeof(frame->data);
    }
    frame->len = payload_len;
    memcpy((void *)frame->data, data, frame->len);

    rx_write_idx = next_write;
    g_fdcan_diag.rx_irq_count++;
    g_fdcan_diag.last_rx_id = frame->id;
    g_fdcan_diag.last_rx_len = frame->len;
}

void FDCAN_ProcessRxQueue(void)
{
    while (rx_read_idx != rx_write_idx) {
        can_frame_t frame;
        memcpy(&frame, (const void *)&rx_queue[rx_read_idx], sizeof(can_frame_t));
        rx_read_idx = (uint8_t)((rx_read_idx + 1U) % FDCAN_RX_QUEUE_SIZE);

        uint8_t dst = (uint8_t)CAN_ID_GET_DST(frame.id);
        extern uint8_t g_local_node_id;
        if ((dst != JOINT_ID_BROADCAST) && (dst != g_local_node_id)) {
            continue;
        }

        g_fdcan_diag.rx_processed_count++;
        FDCAN_RxCallback(&frame);
    }
}

uint8_t FDCAN_GetTxFreeLevel(void)
{
    return HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
}

uint32_t FDCAN_GetRxDropCount(void)
{
    return rx_drop_count;
}

void FDCAN_GetDiag(fdcan_diag_t *out_diag)
{
    if (out_diag == NULL) {
        return;
    }

    __disable_irq();
    out_diag->init_ok = g_fdcan_diag.init_ok;
    out_diag->start_ok = g_fdcan_diag.start_ok;
    out_diag->tx_ok_count = g_fdcan_diag.tx_ok_count;
    out_diag->tx_fail_count = g_fdcan_diag.tx_fail_count;
    out_diag->rx_irq_count = g_fdcan_diag.rx_irq_count;
    out_diag->rx_processed_count = g_fdcan_diag.rx_processed_count;
    out_diag->rx_drop_count = g_fdcan_diag.rx_drop_count;
    out_diag->last_tx_id = g_fdcan_diag.last_tx_id;
    out_diag->last_tx_len = g_fdcan_diag.last_tx_len;
    out_diag->last_rx_id = g_fdcan_diag.last_rx_id;
    out_diag->last_rx_len = g_fdcan_diag.last_rx_len;
    __enable_irq();
}

void FDCAN_ResetDiag(void)
{
    __disable_irq();
    memset((void *)&g_fdcan_diag, 0, sizeof(g_fdcan_diag));
    g_fdcan_diag.init_ok = 1U;
    g_fdcan_diag.start_ok = 1U;
    g_fdcan_diag.rx_drop_count = rx_drop_count;
    __enable_irq();
}

__attribute__((weak)) void FDCAN_RxCallback(can_frame_t *frame)
{
    (void)frame;
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (fdcanHandle->Instance == FDCAN1) {
        LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PCLK1);
        __HAL_RCC_FDCAN_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /* PB8 -> FDCAN1_RX, PB9 -> FDCAN1_TX */
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    if (fdcanHandle->Instance == FDCAN1) {
        __HAL_RCC_FDCAN_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    }
}
