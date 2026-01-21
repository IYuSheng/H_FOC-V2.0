#include "main.h"

Motor_Control_t motor_ctrl = {
    .foc_state =  FOC_STATE_INIT,
    .fault_type = MOTOR_FAULT_NONE,
    .temp_max = OVER_TEMPERATURE_THRESH,    // 过温阈值
    .vbus_max = VOLTAGE_LIMIT,              // 过压阈值
    .vbus_min = UNDER_VOLTAGE_THRESH,       // 欠压阈值
    .current_max = CURRENT_LIMIT            // 过流阈值
};

encoder_status_t enc_status; // 编码器状态变量

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    // 初始化FOC相关外设
    FOC_Init();
    
    while(1)
    {
      /* ------------------ FOC状态机 ------------------ */
      switch(motor_ctrl.foc_state)
      {
        case FOC_STATE_INIT:
          // 初始化编码器
          enc_status = encoder_init();
          foc_start_init();
          if(enc_status != ENCODER_STATUS_OK)
          {
            motor_ctrl.fault_type = FAULT_ENCODER;
            motor_ctrl.foc_state = FOC_STATE_FAULT;
          }
          // 初始化完成后再启用电机中断及编码器读取中断
          NVIC_EnableIRQ(DMA1_Channel2_IRQn); // ADC转换及FOC核心中断开启
          LL_TIM_EnableIT_CC4(TIM8);          // 编码器读取中断开启
          motor_ctrl.foc_state = FOC_STATE_RUNNING;
          break;

        case FOC_STATE_RUNNING:

        // 设置控制参数
        foc_control_set();

        // CAN数据处理
        // my_FDCAN1_Transmit();

          break;

        case FOC_STATE_FAULT:
          // 停止PWM输出
          bsp_pwm_stop();
          switch(motor_ctrl.fault_type)
          {
            case FAULT_OVER_VOLTAGE:
              debug_log("[ERROR] OVER_VOLTAGE");
              break;
              
            case FAULT_UNDER_VOLTAGE:
              debug_log("[ERROR] UNDER_VOLTAGE");
              break;
              
            case FAULT_OVER_CURRENT:
              debug_log("[ERROR] OVER_CURRENT");
              break;
              
            case FAULT_ENCODER:
              debug_log("[ERROR] ENCODER_ERROR");
              break;
              
            case FAULT_OVER_TEMPERATURE:
              debug_log("[ERROR] OVER_TEMPERATURE");
              break;

            case FAULT_I_SAMPLING_ERROR:
              debug_log("[ERROR] I_SAMPLING_ERROR");
              break;
              
            case MOTOR_FAULT_NONE:
              // 待添加恢复机制

              break;
            default:
              debug_log("[ERROR] UNKNOWN_ERROR");
              break;
          }
          motor_ctrl.foc_state = FOC_STATE_STOP;
          break;

        case FOC_STATE_STOP:
          debug_log("[ERROR]");

          break;

        default:
          break;
      }

      /* --------------- 定时处理任务 --------------- */
      if(foc_task.task_update_vbus)
      {
        foc_task.task_update_vbus = 0;
        get_foc_bus_voltage();
      }
      if(foc_task.task_update_three_phase_voltage)
      {
        foc_task.task_update_three_phase_voltage = 0;
        update_Three_phase_voltage();
      }
      if(foc_task.task_update_temperature)
      {
        foc_task.task_update_temperature = 0;
        update_mosfet_temperature();
      }
      if(foc_task.task_sys_common)
      {
        foc_task.task_sys_common = 0;
        // foc打印调试任务
        foc_debug();
        // CAN数据处理
        FDCAN1_ProcessRxQueue();
      }

      /* --------------- 异常状态检测 --------------- */
      if(foc_voltage_data.vbus > motor_ctrl.vbus_max)
      {
        motor_ctrl.fault_type = FAULT_OVER_VOLTAGE;
        if(motor_ctrl.foc_state == FOC_STATE_RUNNING)
        motor_ctrl.foc_state = FOC_STATE_FAULT;
      }
      if(foc_voltage_data.vbus < motor_ctrl.vbus_min)
      {
        motor_ctrl.fault_type = FAULT_UNDER_VOLTAGE;
        if(motor_ctrl.foc_state == FOC_STATE_RUNNING)
        motor_ctrl.foc_state = FOC_STATE_FAULT;
      }
      if(foc_current_data.ia > motor_ctrl.current_max
      || foc_current_data.ib > motor_ctrl.current_max
      || foc_current_data.ic > motor_ctrl.current_max)
      {
        motor_ctrl.fault_type = FAULT_OVER_CURRENT;
        if(motor_ctrl.foc_state == FOC_STATE_RUNNING)
        motor_ctrl.foc_state = FOC_STATE_FAULT;
      }
      if(foc_voltage_data.temp > motor_ctrl.temp_max)
      {
        motor_ctrl.fault_type = FAULT_OVER_TEMPERATURE;
        if(motor_ctrl.foc_state == FOC_STATE_RUNNING)
        motor_ctrl.foc_state = FOC_STATE_FAULT;
      }
      if(foc_current_data.ia + foc_current_data.ib + foc_current_data.ic > 10.0f
      || foc_current_data.ia + foc_current_data.ib + foc_current_data.ic < -10.0f)
      {
        motor_ctrl.fault_type = FAULT_I_SAMPLING_ERROR;
        if(motor_ctrl.foc_state == FOC_STATE_RUNNING)
        motor_ctrl.foc_state = FOC_STATE_FAULT;
      }
    }
}
