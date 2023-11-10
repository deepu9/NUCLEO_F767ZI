#include "stm32f767xx_basic_timers_driver.h"


/**
 * Peripheral RCC clock setup
 */
void Basic_TIM_PeriClockControl(Basic_TIM_RegDef_t *pBasic_TIMx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pBasic_TIMx == TIM6)
		{
			TIM6_PCLK_EN();
		} else if (pBasic_TIMx == TIM7)
		{
			TIM7_PCLK_EN();
		}
	} else {
		if (pBasic_TIMx == TIM6)
		{
			TIM6_PCLK_DI();
		} else if (pBasic_TIMx == TIM7)
		{
			TIM7_PCLK_DI();
		}
	}
}

/**
 * Init and De-Init
 */
void Basic_TIM_Init(Basic_TIM_Handle_t *pBasic_TIM_Handle)
{
	Basic_TIM_Config_t TIM_Config = pBasic_TIM_Handle->TIM_Config;

	// Configure Prescaler
	pBasic_TIM_Handle->pBasic_TIMx->PSC |= TIM_Config.Prescaler;

	// Configure ARR
	pBasic_TIM_Handle->pBasic_TIMx->ARR |= TIM_Config.Period;
}

void Basic_TIM_DeInit(Basic_TIM_RegDef_t *pBasic_TIMx)
{
	if (pBasic_TIMx == TIM6)
	{
		TIM6_REG_RESET();
	} else if (pBasic_TIMx == TIM7)
	{
		TIM6_REG_RESET();
	}
}

void Basic_TIM_PeripheralControl(Basic_TIM_RegDef_t *pBasic_TIMx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pBasic_TIMx->CR1 |= (1 << BASIC_TIM_CR1_CEN);
	} else {
		pBasic_TIMx->CR1 &= ~(1 << BASIC_TIM_CR1_CEN);
	}
}
