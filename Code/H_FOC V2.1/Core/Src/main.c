#include "main.h"

Motor_Control_t motor_ctrl = {
    .foc_state =  FOC_STATE_INIT,
    .fault_type = MOTOR_FAULT_NONE,
    .temp_max = OVER_TEMPERATURE_THRESH,    // 过温阈�?
    .vbus_max = VOLTAGE_LIMIT,              // 过压阈�?
    .vbus_min = UNDER_VOLTAGE_THRESH,       // 欠压阈�?
    .current_max = CURRENT_LIMIT,            // 过浝阈�?
    .angle_max = 70.0f,                      // 最大角度阈值（°/s�?
    .angle_min = -170.0f                     // 最尝角度阈值（°/s�?
};

encoder_status_t enc_status; // 编砝器状思坘�?

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
      /* ------------------ FOC状思机 ------------------ */
      switch(motor_ctrl.foc_state)
      {
        case FOC_STATE_INIT:  /* 电机初始�?*/
          // 初始化编砝器
          enc_status = encoder_init();
          foc_start_init();
          if(enc_status != ENCODER_STATUS_OK)
          {
            motor_ctrl.fault_type = FAULT_ENCODER;
            motor_ctrl.foc_state = FOC_STATE_FAULT;
          }
          // 开坯FOC中断
          foc_start();
          motor_ctrl.foc_state = FOC_STATE_RUNNING;
          break;

        case FOC_STATE_RUNNING: /* 电机违行�?*/
          // 设置外部控制坂数
          foc_control_set();

          break;

        case FOC_STATE_FAULT: /* 电机故障�?*/
          // 坜止PWM输出
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

            case FAULT_ENCODER_ANGLE:
              debug_log("[ERROR] ENCODER_ANGLE_ERROR");
              break;
              
            case MOTOR_FAULT_NONE:
              // 待添加杢夝机�?

              break;
            default:
              debug_log("[ERROR] UNKNOWN_ERROR");
              break;
          }
          motor_ctrl.foc_state = FOC_STATE_STOP;
          break;

        case FOC_STATE_STOP:  /* 电机坜止�?*/

          break;

        default:
          break;
      }

      /* --------------- 定时处睆任务 --------------- */
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
        // foc打坰调试任务
        foc_debug();
        // CAN数杮坑�?
        CAN_ReportStatus(encoder_data.mechanical_angle, encoder_data.mechanical_speed);
        // CAN数杮处睆
        CAN_Process();
      }

      /* --------------- 异常状思检�?--------------- */
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
    //  if(encoder_data.mechanical_angle > motor_ctrl.angle_max || encoder_data.mechanical_angle < motor_ctrl.angle_min)
    //  {
    //    motor_ctrl.fault_type = FAULT_ENCODER_ANGLE;
    //    if(motor_ctrl.foc_state == FOC_STATE_RUNNING)
    //    motor_ctrl.foc_state = FOC_STATE_FAULT;
    //  }
    }
}
