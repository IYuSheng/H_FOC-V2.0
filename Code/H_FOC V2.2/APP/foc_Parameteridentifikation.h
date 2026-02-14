#ifndef __FOC_PARAMETERIDENTIFIKATION_H
#define __FOC_PARAMETERIDENTIFIKATION_H

#include "Config.h"
#include "foc_control.h"

typedef struct {
    float Rs;
    float Ld;
    float Lq;
    float Ls;
    float flux_linkage;
    uint8_t valid;
} motor_param_t;

void foc_motor_param_ident_start(float v_align, float v_rs, float hf_freq, float v_hf, float v_lock_d);
uint8_t foc_motor_param_ident_is_running(void);
void foc_motor_parameter_ident_step(void);

#endif // __FOC_PARAMETERIDENTIFIKATION_H
