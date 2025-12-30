#include "foc_sys.h"

FOC_TASK foc_task;

// 系统节拍计数器
static volatile uint32_t system_tick = 0;

// 任务计数器
static volatile uint32_t task_vbus_counter = 0;
static volatile uint32_t task_three_voltage_counter = 0;
static volatile uint32_t task_temperature_counter = 0;

// 任务执行周期(以系统滴答为单位)
static const uint32_t task_vbus_period = SYSTICK_FREQUENCY_HZ / TASK_VBUS_UPDATE_FREQUENCY_HZ;
static const uint32_t task_three_voltage_period = SYSTICK_FREQUENCY_HZ / TASK_THREE_VOLTAGE_UPDATE_FREQUENCY_HZ;
static const uint32_t task_temperature_period = SYSTICK_FREQUENCY_HZ / TASK_TEMPERATURE_UPDATE_FREQUENCY_HZ;

/**
  * @brief 1ms定时中断处理函数
  */
void TIM3_IRQHandler(void)
{
  // 检查更新中断标志
  if(LL_TIM_IsActiveFlag_UPDATE(TIM3))
  {
    // 清除中断标志
    LL_TIM_ClearFlag_UPDATE(TIM3);

    /* 增加系统滴答计数器 */
    system_tick++;

    // 普通任务更新
    foc_task.task_sys_common = 1;

    /* 更新任务计数器并设置任务标志 */
    if (++task_vbus_counter >= task_vbus_period)
    {
        task_vbus_counter = 0;  // 重置计数器
        foc_task.task_update_vbus = 1;
    }

    if (++task_three_voltage_counter >= task_three_voltage_period)
    {
        task_three_voltage_counter = 0;  // 重置计数器
        foc_task.task_update_three_phase_voltage = 1;
    }

    if (++task_temperature_counter >= task_temperature_period)
    {
        task_temperature_counter = 0;  // 重置计数器
        foc_task.task_update_temperature = 1;
    }
  }
}

/**
  * @brief 编码器值获取中断
  */
void TIM8_CC_IRQHandler(void)
{
  // 仅处理CH5的比较中断（TRGO2触发源）
  if (LL_TIM_IsActiveFlag_CC4(TIM8))
  {
    // 清除中断标志，上升下降都会触发，所以要在外面清除
    LL_TIM_ClearFlag_CC4(TIM8);
    // 检测TIM8计数器方向，仅UP阶段处理
    if ((TIM8->CR1 & TIM_CR1_DIR) == 0) 
    {
      // 读取编码器电角度
      encoder_read_electrical_angle();
      // 更新电机转速
      encoder_get_mechanical_speed();
    }
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
