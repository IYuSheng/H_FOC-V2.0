#include "main.h"
#include "adc.h"

// 全局变量：存储编码器数据
encoder_data_t encoder_data;
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    FOC_Init();
    debug_log("System Init Success");
    
    // 设置当前角度为零位
    encoder_status_t zero_status = encoder_set_zero_temp();
    // 全部初始化完成后再启用编码器读取中断
    LL_TIM_EnableIT_CC4(TIM8);
    while(1)
    {
      debug_log("%.6f",encoder_data.mechanical_angle);

      HAL_Delay(2);
    }
}

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
      encoder_data.mechanical_angle = encoder_read_mechanical_angle();
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
