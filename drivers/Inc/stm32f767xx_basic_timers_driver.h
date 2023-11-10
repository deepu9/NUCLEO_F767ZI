#ifndef INC_STM32F767XX_BASIC_TIMERS_DRIVER_H_
#define INC_STM32F767XX_BASIC_TIMERS_DRIVER_H_

#include "stm32f767xx.h"


typedef struct
{
	uint32_t Prescaler;
	uint32_t Period;
}Basic_TIM_Config_t;

typedef struct
{
	Basic_TIM_RegDef_t *pBasic_TIMx;
	Basic_TIM_Config_t TIM_Config;
}Basic_TIM_Handle_t;


/************************************************************************
 * API's supported by STM32F767xx
 ***********************************************************************/
/**
 * Peripheral RCC clock setup
 */
void Basic_TIM_PeriClockControl(Basic_TIM_RegDef_t *pBasic_TIMx, uint8_t EnorDi);

/**
 * Init and De-Init
 */
void Basic_TIM_Init(Basic_TIM_Handle_t *pBasic_TIM_Handle);
void Basic_TIM_DeInit(Basic_TIM_RegDef_t *pBasic_TIMx);

/**
 * Other Peripheral control API's
 */
//To enable/disable TIM peripheral
void Basic_TIM_PeripheralControl(Basic_TIM_RegDef_t *pBasic_TIMx, uint8_t EnorDi);

#endif /* INC_STM32F767XX_BASIC_TIMERS_DRIVER_H_ */
