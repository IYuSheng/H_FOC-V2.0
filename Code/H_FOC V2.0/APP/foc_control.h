#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H

#include "arm_math.h"
#include "tim.h"
#include "Config.h"
#include "foc_conversion.h"

/**
 * @brief FOC¿ØÖÆÖ÷º¯Êý
 */
void foc_control(void);

void foc_open_loop_control(float target_speed, float target_outq);
void foc_current_control(void);
void foc_speed_control(void);
void foc_position_control(void);

SVPWM_t* svpwm_get(void);

#endif /* __FOC_CONTROL_H */
