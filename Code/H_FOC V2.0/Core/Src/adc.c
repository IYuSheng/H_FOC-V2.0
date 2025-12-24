#include "adc.h"
#include "Config.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dma.h"
#include "arm_math.h"
#include "foc_control.h"
#include "usart.h"

#define REG_CHANNELS 5               // 规则组通道数（5个电压通道）
#define ADC2_DMA_BUFFER_SIZE REG_CHANNELS  // DMA缓冲区大小=通道数
#define ADC2_DMA_CHANNEL DMA1_Channel1      // ADC2规则组对应DMA通道（STM32G4固定映射）

// ADC2 DMA缓冲区（循环模式下自动更新，全局变量避免栈溢出）
static uint16_t adc2_dma_buffer[ADC2_DMA_BUFFER_SIZE] = {0};
// 存储ADC2 规则组通道原始采样值（对外接口用）
uint16_t adc2_raw_data[REG_CHANNELS] = {0};

// 电流转换系数 = 3.3 / 4096 / Ω / 增益倍速
static float current_conv_factor = ADC_REF_VOLTAGE / ADC_MAX_VALUE / INA240_GAIN;
// 电压转换系数 = 3.3V / 4095 * (小电阻+大电阻)kΩ / 小电阻kΩ
static float adc_calib = ADC_REF_VOLTAGE / ADC_MAX_VALUE * (R_Voaltage_1 + R_Voaltage_2) / R_Voaltage_2;

// 存储原始电流数据和处理后的电流数据
foc_data_t foc_raw_data = {0};
foc_data_i foc_current_data = {0};
foc_data_v foc_voltage_data = {0};

void MX_ADC1_Init(void)
{
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0   ------> ADC1_IN1
  PA1   ------> ADC1_IN2
  PA2   ------> ADC1_IN3
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);

  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  // 触发源为TIM8更新事件触发（与PWM同步）
  ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2; //Tim8 TRGO触发
  ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS;
  ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;  // 独立触发
  LL_ADC_INJ_Init(ADC1, &ADC_INJ_InitStruct);

  LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_DISABLE);  // 关闭队列功能
  LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);  // 上升沿触发

  LL_ADC_DisableDeepPowerDown(ADC1);
  LL_ADC_EnableInternalRegulator(ADC1);
  // 等待稳压源稳定
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_24CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);

  LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_24CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);

  LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_3, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_24CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);

  // 启用注入转换完成中断
  LL_ADC_ClearFlag_JEOS(ADC1);
  LL_ADC_EnableIT_JEOS(ADC1);

  NVIC_SetPriority(ADC1_2_IRQn, 1);  // 仅次于紧急事件优先级
  NVIC_EnableIRQ(ADC1_2_IRQn);       // 使能ADC1_2_IRQn电流采样注入组中断

  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  while(LL_ADC_IsCalibrationOnGoing(ADC1)); // 等待校准完成

  LL_ADC_SetOffset(ADC1, LL_ADC_OFFSET_1, LL_ADC_CHANNEL_1, 69);
  // 设置偏移符号：负（原始数据 - 偏移值），正（原始数据 + 偏移值）
  LL_ADC_SetOffsetSign(ADC1, LL_ADC_OFFSET_1, LL_ADC_OFFSET_SIGN_POSITIVE);
  // 设置偏移饱和模式：启用（溢出时饱和，比如结果为负则固定为0）
  LL_ADC_SetOffsetSaturation(ADC1, LL_ADC_OFFSET_1, LL_ADC_OFFSET_SATURATION_ENABLE);

  // 配置OFFSET_2：对应ADC1_CH2（IB）
  LL_ADC_SetOffset(ADC1, LL_ADC_OFFSET_2, LL_ADC_CHANNEL_2, 69);
  LL_ADC_SetOffsetSign(ADC1, LL_ADC_OFFSET_2, LL_ADC_OFFSET_SIGN_POSITIVE);
  LL_ADC_SetOffsetSaturation(ADC1, LL_ADC_OFFSET_2, LL_ADC_OFFSET_SATURATION_ENABLE);

  // 配置OFFSET_3：对应ADC1_CH3（IC）
  LL_ADC_SetOffset(ADC1, LL_ADC_OFFSET_3, LL_ADC_CHANNEL_3, 69);
  LL_ADC_SetOffsetSign(ADC1, LL_ADC_OFFSET_3, LL_ADC_OFFSET_SIGN_POSITIVE);
  LL_ADC_SetOffsetSaturation(ADC1, LL_ADC_OFFSET_3, LL_ADC_OFFSET_SATURATION_ENABLE);
  
  // 启用ADC和注入组转换
  LL_ADC_Enable(ADC1);
  // 等待ADC就绪
  while(!LL_ADC_IsActiveFlag_ADRDY(ADC1));
  LL_ADC_ClearFlag_ADRDY(ADC1);

  LL_ADC_INJ_StartConversion(ADC1); // 启动注入组转换
}

/* ADC2 init function */
void MX_ADC2_Init(void)
{
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};
  LL_DMA_InitTypeDef DMA_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  // 2. ADC时钟和GPIO配置（保持不变）
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /**ADC2 GPIO Configuration
  PC0   ------> ADC2_IN6
  PC1   ------> ADC2_IN7
  PC2   ------> ADC2_IN8
  PC3   ------> ADC2_IN9
  PC4   ------> ADC2_IN5
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // 3. ADC基础配置（保持不变）
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);

  // 4. 核心修改：启用DMA传输
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE; // 软件触发
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS; // 5通道扫描
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE; // 关闭间断模式
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS; // 连续转换模式
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED; // 无限制DMA传输模式
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED; // 溢出时保留数据
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);

  // 5. 注入组配置（无用，保持默认）
  LL_ADC_SetGainCompensation(ADC2, 0);
  LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
  ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
  LL_ADC_INJ_Init(ADC2, &ADC_INJ_InitStruct);

  // 7. 通道序列配置（保持不变，顺序：PC0→PC1→PC2→PC3→PC4）
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_6, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_7);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_7, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_8);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_9);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_9, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_5, LL_ADC_CHANNEL_5);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_47CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SINGLE_ENDED);

  // 先禁用DMA通道
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  
  // 配置DMA
  DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_ADC2;  // ADC2 DMA请求
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStruct.Mode = LL_DMA_MODE_CIRCULAR;  // 循环模式
  DMA_InitStruct.PeriphOrM2MSrcDataSize  = LL_DMA_PDATAALIGN_HALFWORD;  // 16位
  DMA_InitStruct.MemoryOrM2MDstDataSize  = LL_DMA_MDATAALIGN_HALFWORD;     // 16位
  DMA_InitStruct.Priority = LL_DMA_PRIORITY_MEDIUM;
  DMA_InitStruct.NbData = ADC2_DMA_BUFFER_SIZE;  // 传输数据数量
  DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&ADC2->DR;
  DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)adc2_dma_buffer;
  
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);

  /* 9. 使能ADC内部稳压器并校准 */
  LL_ADC_DisableDeepPowerDown(ADC2);
  LL_ADC_EnableInternalRegulator(ADC2);

  // 等待稳压器稳定
  uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0) {
    wait_loop_index--;
  }

  // 校准ADC
  LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
  while(LL_ADC_IsCalibrationOnGoing(ADC2)); // 等待校准完成

  // 10. 使能DMA
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  // 11. 启动ADC2连续转换（DMA自动搬运数据）
  LL_ADC_Enable(ADC2);
  while(!LL_ADC_IsActiveFlag_ADRDY(ADC2)); // 等待ADC2就绪
  LL_ADC_ClearFlag_ADRDY(ADC2);
  LL_ADC_REG_StartConversion(ADC2); // 启动规则组连续扫描转换
}

/**
 * @brief  读取ADC2 5通道原始采样值（DMA缓冲区数据）
 * @param  无
 * @retval uint16_t*：5通道原始值数组指针（顺序：PC0→PC1→PC2→PC3→PC4）
 */
uint16_t* bsp_adc2_read_five_channels(void)
{
  // DMA循环模式下，直接拷贝缓冲区数据到对外接口数组（避免直接操作DMA缓冲区）
  for(uint8_t i=0; i<REG_CHANNELS; i++)
  {
    adc2_raw_data[i] = adc2_dma_buffer[i];
  }
  return adc2_raw_data;
}

/**
 * @brief 获取ADC2 5通道原始采样值（对外接口）
 * @return 5通道原始值数组指针
 */
uint16_t* bsp_adc2_get_raw_data(void)
{
  bsp_adc2_read_five_channels(); // 读取最新数据
  return adc2_raw_data;
}

float* bsp_adc2_get_voltage(void)
{
  static float voltage[REG_CHANNELS] = {0};
  uint16_t* raw_data = bsp_adc2_get_raw_data();
  
  for(uint8_t i=0; i<REG_CHANNELS; i++)
  {
    voltage[i] = (float)raw_data[i] * adc_calib; // 转换为实际电压
  }
  
  return voltage;
}

/**
 * @brief ADC注入组中断服务函数（处理电流数据并迅速进行FOC计算控制）
 */
void ADC1_2_IRQHandler(void)
{
  if (LL_ADC_IsActiveFlag_JEOS(ADC1)) 
  {
    LL_ADC_ClearFlag_JEOS(ADC1);
    // 读取注入通道的电流值
    foc_raw_data.ia = 2048 - LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    foc_raw_data.ib = 2048 - LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
    foc_raw_data.ic = 2048 - LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3);

    // 转换为实际电流值 (安培)·
    foc_current_data.ia = (float32_t)foc_raw_data.ia * current_conv_factor;
    foc_current_data.ib = (float32_t)foc_raw_data.ib * current_conv_factor;
    foc_current_data.ic = (float32_t)foc_raw_data.ic * current_conv_factor;
    foc_control(); // FOC控制
  }
}

/**
 * @brief 获取原始电流数据
 * @return 原始电流数据结构体指针
 */
foc_data_t* bsp_adc_get_raw_data(void)
{
    return &foc_raw_data;
}

/**
 * @brief 获取处理后的电流数据
 * @return 处理后的电流数据结构体指针
 */
foc_data_i* bsp_adc_get_current_data(void)
{
    return &foc_current_data;
}
