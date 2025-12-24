#include "tim.h"
#include "stm32g4xx_ll_tim.h"
#include "Config.h"

void MX_TIM8_Init(void)
{

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC5_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
  
  // 配置定时器基础参数
  // 中心对称PWM模式，用于FOC控制
  // 预分频器 = 0: 170MHz / (0+1) = 170MHz
  // 自动重载值 = 4249: PWM频率 = 170MHz / (2 * 4250) = 20kHz (中心对称模式频率计算)
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP_DOWN;  // 中心对称模式
  TIM_InitStruct.Autoreload = PWM_PERIOD;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM8, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM8);
  LL_TIM_SetClockSource(TIM8, LL_TIM_CLOCKSOURCE_INTERNAL);
  
  // 配置PWM通道1 (主通道)
  LL_TIM_OC_EnablePreload(TIM8, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_ENABLE;  // 使能互补通道
  TIM_OC_InitStruct.CompareValue = PWM_PERIOD / 2;  // 初始占空比50% (2125/4250)
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;  // 互补通道极性
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM8, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM8, LL_TIM_CHANNEL_CH1);
  
  // 配置PWM通道2 (主通道)
  LL_TIM_OC_EnablePreload(TIM8, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.CompareValue = PWM_PERIOD / 2;  // 初始占空比50%
  LL_TIM_OC_Init(TIM8, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM8, LL_TIM_CHANNEL_CH2);
  
  // 配置PWM通道3 (主通道)
  LL_TIM_OC_EnablePreload(TIM8, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.CompareValue = PWM_PERIOD / 2;  // 初始占空比50%
  LL_TIM_OC_Init(TIM8, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM8, LL_TIM_CHANNEL_CH3);
  
  // 配置通道4用于触发中断TRGO
  LL_TIM_OC_EnablePreload(TIM8, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = PWM_PERIOD - 20 - 1700;          // 在ADC触发前1700个计数值处触发
  LL_TIM_OC_Init(TIM8, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  

  // 配置通道5用于触发ADC注入组TRGO2
  LL_TIM_OC_EnablePreload(TIM8, LL_TIM_CHANNEL_CH5);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = PWM_PERIOD - 20;          // 在最大值前20处触发
  LL_TIM_OC_Init(TIM8, LL_TIM_CHANNEL_CH5, &TIM_OC_InitStruct);

  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH4);
  LL_TIM_ClearFlag_CC4(TIM8);  // 清除CC4初始中断标志
  
  // 2. 启用CC4中断(编码器读取需初始化后，后面再启用)
//   LL_TIM_EnableIT_CC4(TIM8);

  // 设置触发输出为通道4比较事件
  LL_TIM_SetTriggerOutput(TIM8, LL_TIM_TRGO_OC4REF);

  LL_TIM_SetTriggerOutput2(TIM8, LL_TIM_TRGO2_OC5);

  // 禁用主从模式
  LL_TIM_DisableMasterSlaveMode(TIM8);

  NVIC_SetPriority(TIM8_CC_IRQn, 1);  // 优先级高于ADC中断
  NVIC_EnableIRQ(TIM8_CC_IRQn);
  
  // 配置GPIO
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  
  /* TIM8 GPIO Configuration
  PC6     ------> TIM8_CH1    (主通道)
  PC7     ------> TIM8_CH2    (主通道)
  PC8     ------> TIM8_CH3    (主通道)
  PC10    ------> TIM8_CH1N   (互补通道)
  PC11    ------> TIM8_CH2N   (互补通道)
  PC12    ------> TIM8_CH3N   (互补通道)
  */
  
  // 配置通道1 GPIO (主通道)
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // 配置通道2 GPIO (主通道)
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // 配置通道3 GPIO (主通道)
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  // 配置通道1N GPIO (互补通道)
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  // 配置通道2N GPIO (互补通道)
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  // 配置通道3N GPIO (互补通道)
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
 * @brief 设置三相PWM占空比
 * @param duty_u U相比较值
 * @param duty_v V相比较值
 * @param duty_w W相比较值
 */
void bsp_pwm_set_duty_three_phase(uint32_t duty_u, uint32_t duty_v, uint32_t duty_w)
{
    // 设置比较值
    LL_TIM_OC_SetCompareCH1(TIM8, duty_u);  // U相
    LL_TIM_OC_SetCompareCH2(TIM8, duty_v);  // V相
    LL_TIM_OC_SetCompareCH3(TIM8, duty_w);  // W相
}

/**
 * @brief 启动PWM输出
 */
void bsp_pwm_start(void)
{
    // 使能预装载寄存器
    LL_TIM_OC_EnablePreload(TIM8, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM8, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM8, LL_TIM_CHANNEL_CH3);
    
    // 启动计数器
    LL_TIM_EnableCounter(TIM8);
    
    // 使能PWM输出通道 (主通道)
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3);
    
    // 使能PWM输出通道 (互补通道)
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3N);
    
    // 对于高级定时器TIM8，必须使能主输出
    LL_TIM_EnableAllOutputs(TIM8);
}

/**
 * @brief 停止PWM输出
 */
void bsp_pwm_stop(void)
{
    // 对于高级定时器TIM8，先禁用主输出
    LL_TIM_DisableAllOutputs(TIM8);
    
    // 禁用PWM输出通道 (主通道)
    LL_TIM_CC_DisableChannel(TIM8, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIM8, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_DisableChannel(TIM8, LL_TIM_CHANNEL_CH3);
    
    // 禁用PWM输出通道 (互补通道)
    LL_TIM_CC_DisableChannel(TIM8, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_DisableChannel(TIM8, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_DisableChannel(TIM8, LL_TIM_CHANNEL_CH3N);
    
    // 停止计数器
    LL_TIM_DisableCounter(TIM8);
    
    // 禁用预装载寄存器
    LL_TIM_OC_DisablePreload(TIM8, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_DisablePreload(TIM8, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_DisablePreload(TIM8, LL_TIM_CHANNEL_CH3);
}
