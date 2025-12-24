#ifndef __FOC_INIT_H__
#define __FOC_INIT_H__

#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_fmac.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

#include "adc.h"
#include "cordic.h"
#include "dac.h"
#include "fdcan.h"
#include "fmac.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "foc_encoder.h"

void FOC_Init(void);

#endif /*__ GPIO_H__ */
