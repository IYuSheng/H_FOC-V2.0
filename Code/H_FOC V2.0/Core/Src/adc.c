#include "adc.h"
#include "Config.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dma.h"
#include "arm_math.h"
#include <math.h>
#include "foc_control.h"
#include "usart.h"

// ADC与电路参数
#define R45         10000.0f // 上拉电阻阻值（10kΩ）
// NTC热敏电阻参数
#define NTC_R0      10000.0f // 25℃时的标称阻值（10kΩ）
#define NTC_B       3380.0f  // NTC的B值（K）
#define NTC_T0      298.15f  // 25℃对应的开尔文温度（25+273.15）

#define REG_CHANNELS 5               // 规则组通道数（5个电压通道）
#define ADC2_DMA_BUFFER_SIZE (REG_CHANNELS * 2)  // 双缓冲区
#define ADC2_DMA_CHANNEL DMA1_Channel1      // ADC2规则组对应DMA通道（STM32G4固定映射）

// ADC2 DMA缓冲区（循环模式下自动更新，全局变量避免栈溢出）
static uint16_t adc2_dma_buffer[ADC2_DMA_BUFFER_SIZE] = {0};
// 存储ADC2 规则组通道原始采样值
static uint16_t adc2_raw_data[REG_CHANNELS] = {0};

// 电流转换系数 = 3.3 / 4096 / Ω / 增益倍速
static float current_conv_factor = ADC_REF_VOLTAGE / ADC_MAX_VALUE / R_Current / INA240_GAIN;
// 电压转换系数 = 3.3V / 4095 * (小电阻+大电阻)kΩ / 小电阻kΩ
static float voltage_conv_factor = ADC_REF_VOLTAGE / ADC_MAX_VALUE * (R_Voaltage_1 + R_Voaltage_2) / R_Voaltage_2;

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
  // NVIC_EnableIRQ(ADC1_2_IRQn);       // FOC初始化完成后开启中断,这里注释

  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  while(LL_ADC_IsCalibrationOnGoing(ADC1)); // 等待校准完成

  LL_ADC_SetOffset(ADC1, LL_ADC_OFFSET_1, LL_ADC_CHANNEL_1, 75);
  // 设置偏移符号：负（原始数据 - 偏移值），正（原始数据 + 偏移值）
  LL_ADC_SetOffsetSign(ADC1, LL_ADC_OFFSET_1, LL_ADC_OFFSET_SIGN_POSITIVE);
  // 设置偏移饱和模式：启用（溢出时饱和，比如结果为负则固定为0）
  LL_ADC_SetOffsetSaturation(ADC1, LL_ADC_OFFSET_1, LL_ADC_OFFSET_SATURATION_ENABLE);

  // 配置OFFSET_2：对应ADC1_CH2（IB）
  LL_ADC_SetOffset(ADC1, LL_ADC_OFFSET_2, LL_ADC_CHANNEL_2, 75);
  LL_ADC_SetOffsetSign(ADC1, LL_ADC_OFFSET_2, LL_ADC_OFFSET_SIGN_POSITIVE);
  LL_ADC_SetOffsetSaturation(ADC1, LL_ADC_OFFSET_2, LL_ADC_OFFSET_SATURATION_ENABLE);

  // 配置OFFSET_3：对应ADC1_CH3（IC）
  LL_ADC_SetOffset(ADC1, LL_ADC_OFFSET_3, LL_ADC_CHANNEL_3, 75);
  LL_ADC_SetOffsetSign(ADC1, LL_ADC_OFFSET_3, LL_ADC_OFFSET_SIGN_POSITIVE);
  LL_ADC_SetOffsetSaturation(ADC1, LL_ADC_OFFSET_3, LL_ADC_OFFSET_SATURATION_ENABLE);
  
  // 启用ADC和注入组转换
  LL_ADC_Enable(ADC1);
  // 等待ADC就绪
  while(!LL_ADC_IsActiveFlag_ADRDY(ADC1));
  LL_ADC_ClearFlag_ADRDY(ADC1);

  // 启动注入组转换
  LL_ADC_INJ_StartConversion(ADC1);
}

/* ADC2 init function */
void MX_ADC2_Init(void)
{
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};
  LL_DMA_InitTypeDef DMA_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  // 2. ADC时钟和GPIO配置
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
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

  // 3. ADC基础配置
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);

  // 4. 启用DMA传输
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE; // 软件触发(第一次触发)
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS; // 5通道扫描
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE; // 关闭间断模式
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS; // 连续转换模式
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED; // 无限制DMA传输模式
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN; // 覆盖
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);

  // 5. 注入组配置（无用，保持默认）
  LL_ADC_SetGainCompensation(ADC2, 0);
  LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
  ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
  LL_ADC_INJ_Init(ADC2, &ADC_INJ_InitStruct);

  // 7. 通道序列配置（保持不变，顺序：PC0→PC1→PC2→PC3→PC4）
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_92CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_6, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_7);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_92CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_7, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_8);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_92CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_9);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_92CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_9, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_5, LL_ADC_CHANNEL_5);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_92CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SINGLE_ENDED);
  
  // 配置DMA
  DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_ADC2;  // ADC2 DMA请求
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStruct.Mode = LL_DMA_MODE_CIRCULAR;  // 循环模式
  DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;  // 外设地址不递增
  DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;    // 内存地址递增  
  DMA_InitStruct.PeriphOrM2MSrcDataSize  = LL_DMA_PDATAALIGN_HALFWORD;  // 16位
  DMA_InitStruct.MemoryOrM2MDstDataSize  = LL_DMA_MDATAALIGN_HALFWORD;     // 16位
  DMA_InitStruct.Priority = LL_DMA_PRIORITY_MEDIUM;
  DMA_InitStruct.NbData = ADC2_DMA_BUFFER_SIZE;  // 传输数据数量
  DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&ADC2->DR;
  DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)adc2_dma_buffer;
  
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);

  // 使能DMA中断
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);  // 半传输中断
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);  // 传输完成中断
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);  // 传输错误中断
  
  // 设置DMA中断优先级
  NVIC_SetPriority(DMA1_Channel1_IRQn, 3);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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

  // 使能DMA
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  // 等待DMA就绪
  HAL_Delay(1);

  // 启动ADC2连续转换（DMA自动搬运数据）
  LL_ADC_Enable(ADC2);
  while(!LL_ADC_IsActiveFlag_ADRDY(ADC2)); // 等待ADC2就绪
  LL_ADC_ClearFlag_ADRDY(ADC2);

   // 启动规则组连续扫描转换
  LL_ADC_REG_StartConversion(ADC2);
}

/**
 * @brief ADC注入组中断服务函数（处理电流数据并迅速进行FOC计算控制）
 */
void ADC1_2_IRQHandler(void)
{
  if (LL_ADC_IsActiveFlag_JEOS(ADC1)) 
  {
    // 清除结束标志
    LL_ADC_ClearFlag_JEOS(ADC1);

    // 读取注入通道的电流值
    foc_raw_data.ia = 2048 - LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    foc_raw_data.ib = 2048 - LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
    foc_raw_data.ic = 2048 - LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3);

    // 转换为实际电流值 (安培)·
    foc_current_data.ia = (float32_t)foc_raw_data.ia * current_conv_factor;
    foc_current_data.ib = (float32_t)foc_raw_data.ib * current_conv_factor;
    foc_current_data.ic = (float32_t)foc_raw_data.ic * current_conv_factor;
    
     // FOC控制
    foc_control();
  }
}

// DMA传输完成中断处理
void DMA1_Channel1_IRQHandler(void)
{
  // 半传输完成
  if (LL_DMA_IsActiveFlag_HT1(DMA1))
  {
    // 前半缓冲区完成，读取后半部分数据到输出数组
    for(uint8_t i = 0; i < REG_CHANNELS; i++)
    {
      adc2_raw_data[i] = adc2_dma_buffer[REG_CHANNELS + i]; // 从索引5开始读取
    }
    LL_DMA_ClearFlag_HT1(DMA1);
  }

  // 全传输完成
  if (LL_DMA_IsActiveFlag_TC1(DMA1))
  {
    // 后半缓冲区完成，读取前半部分数据到输出数组
    for(uint8_t i = 0; i < REG_CHANNELS; i++)
    {
      adc2_raw_data[i] = adc2_dma_buffer[i]; // 从索引0开始读取
    }
    LL_DMA_ClearFlag_TC1(DMA1);
  }

  // 传输错误
  if(LL_DMA_IsActiveFlag_TE1(DMA1))
  {
    LL_DMA_ClearFlag_TE1(DMA1);
  }
}

/**
 * @brief 获取母线电压
 */
float get_foc_bus_voltage(void)
{
  foc_voltage_data.vbus = adc2_raw_data[3] * voltage_conv_factor;
  return foc_voltage_data.vbus;
}

/**
 * @brief 更新三相端电压
 */
void update_Three_phase_voltage(void)
{
  foc_voltage_data.va = adc2_raw_data[0] * voltage_conv_factor;
  foc_voltage_data.vb = adc2_raw_data[1] * voltage_conv_factor;
  foc_voltage_data.vc = adc2_raw_data[2] * voltage_conv_factor;
}

/**
 * @brief 更新MOSFET温度（ADC转温度）
 */
void update_mosfet_temperature(void)
{
    uint16_t adc_raw = adc2_raw_data[4]; // 获取ADC原始值
    
    // 步骤1：ADC原始值 → 对应电压（V_ADC）
    float v_adc = adc_raw * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
    
    // 步骤2：分压公式 → 计算NTC当前阻值（R_T）
    // 分压公式：V_ADC = ADC_REF_VOLTAGE * R_T / (R45 + R_T) → 解R_T
    float r_t = (v_adc * R45) / (ADC_REF_VOLTAGE - v_adc);
    
    // 步骤3：NTC B值公式 → 计算温度（开尔文转摄氏度）
    float ratio = r_t / NTC_R0;          // R_T与R0的比值
    float inv_t = 1.0f / NTC_T0 + log(ratio) / NTC_B; // 1/T = 1/T0 + (ln(RT/R0))/B
    float temp_k = 1.0f / inv_t;         // 开尔文温度
    foc_voltage_data.temp = temp_k - 273.15f; // 转摄氏度
}
