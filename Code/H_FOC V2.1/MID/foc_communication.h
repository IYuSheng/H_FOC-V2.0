#ifndef FOC_COMMUNICATION_H
#define FOC_COMMUNICATION_H

#include "foc_prase.h"
#include "fdcan.h"
#include "foc_conversion.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t g_local_node_id;

void foc_control_set(void);
void CAN_ReportStatus(float pos, float vel);
void CAN_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_COMMUNICATION_H */
