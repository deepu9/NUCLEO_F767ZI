#include "stm32f767xx_gpio_driver.h"


/***********************************************************************
 *
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- Enables or disables peripheral clock for a given GPIO port
 *
 * @param[in]	- Base address of GPIO peripheral
 * @param[in]	- ENABLE OR DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		} else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		} else if (pGPIOx == GPIOK)
		{
			GPIOK_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		} else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_DI();
		} else if (pGPIOx == GPIOK)
		{
			GPIOK_PCLK_DI();
		}
	}
}

/***********************************************************************
 *
 * @fn			- GPIO_BaseAddrToCode
 *
 * @brief		- Convert the GPIO base address to Code
 *
 * @param[in]	- pGPIOx
 *
 * @return		- uint8_t
 *
 * @Note		- none
 */
uint8_t GPIO_BaseAddrToCode(GPIO_RegDef_t *pGPIOx)
{
	uint8_t portcode;

	if (pGPIOx == GPIOA)
	{
		portcode = EXTI_CRx_GPIOA;
	} else if (pGPIOx == GPIOB)
	{
		portcode = EXTI_CRx_GPIOB;
	} else if (pGPIOx == GPIOC)
	{
		portcode = EXTI_CRx_GPIOC;
	} else if (pGPIOx == GPIOD)
	{
		portcode = EXTI_CRx_GPIOD;
	} else if (pGPIOx == GPIOE)
	{
		portcode = EXTI_CRx_GPIOE;
	} else if (pGPIOx == GPIOF)
	{
		portcode = EXTI_CRx_GPIOF;
	} else if (pGPIOx == GPIOG)
	{
		portcode = EXTI_CRx_GPIOG;
	} else if (pGPIOx == GPIOH)
	{
		portcode = EXTI_CRx_GPIOH;
	} else if (pGPIOx == GPIOI)
	{
		portcode = EXTI_CRx_GPIOI;
	} else if (pGPIOx == GPIOJ)
	{
		portcode = EXTI_CRx_GPIOJ;
	} else if (pGPIOx == GPIOK)
	{
		portcode = EXTI_CRx_GPIOK;
	}

	return portcode;
}

/***********************************************************************
 *
 * @fn			- GPIO_Init
 *
 * @brief		- Initialise a GPIO port
 *
 * @param[in]	- GPIO handle
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	GPIO_PinConfig_t pinConfig = pGPIOHandle->GPIO_PinConfig;

	if (pinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//1. Configure the mode of GPIO pin
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (pinConfig.GPIO_PinNumber * 2)); //clearing bits
		pGPIOHandle->pGPIOx->MODER |= (pinConfig.GPIO_PinMode << (pinConfig.GPIO_PinNumber * 2)); //setting bits
	} else {
		if (pinConfig.GPIO_PinMode == GPIO_MODE_IT_FET)
		{
			//Clear the RTSR
			EXTI->RTSR &= ~(1 << pinConfig.GPIO_PinNumber);

			//Configure the FTSR
			EXTI->FTSR |= (1 << pinConfig.GPIO_PinNumber);
		} else if (pinConfig.GPIO_PinMode == GPIO_MODE_IT_RET)
		{
			//Clear the FTSR
			EXTI->FTSR &= ~(1 << pinConfig.GPIO_PinNumber);

			//Configure the RTSR
			EXTI->RTSR |= (1 << pinConfig.GPIO_PinNumber);
		} else if (pinConfig.GPIO_PinMode == GPIO_MODE_IT_REFET)
		{
			//Configure both FTSR & RTSR
			EXTI->FTSR |= (1 << pinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pinConfig.GPIO_PinNumber);
		}

		//Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BaseAddrToCode(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN(); //Enable SYSCFG

		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pinConfig.GPIO_PinNumber);
	}

	//2. Configure the speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (pinConfig.GPIO_PinNumber * 2)); //clearing bits
	pGPIOHandle->pGPIOx->OSPEEDR |= (pinConfig.GPIO_PinSpeed << (pinConfig.GPIO_PinNumber * 2)); //setting bits

	//3. Configure the pupd settings
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (pinConfig.GPIO_PinNumber * 2)); //clearing bits
	pGPIOHandle->pGPIOx->PUPDR |= (pinConfig.GPIO_PinPuPdControl << (pinConfig.GPIO_PinNumber * 2)); //setting bits

	if (pinConfig.GPIO_PinMode == GPIO_MODE_OUT)
	{
		//4. Configure the optype
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pinConfig.GPIO_PinNumber); //clearing bits
		pGPIOHandle->pGPIOx->OTYPER |= (pinConfig.GPIO_PinOpType << pinConfig.GPIO_PinNumber); //setting bits
	}

	//5. Configure the alt function
	if (pinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//Configure ALT function registers
		uint8_t temp1, temp2;

		temp1 = pinConfig.GPIO_PinNumber / 8;
		temp2 = pinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing bits
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pinConfig.GPIO_PinAltFunMode << (4 * temp2)); //setting bits
	}
}

/***********************************************************************
 *
 * @fn			- GPIO_DeInit
 *
 * @brief		- De-Initialise a GPIO port
 *
 * @param[in]	- Base address of GPIO peripheral
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	} else if (pGPIOx == GPIOJ)
	{
		GPIOJ_REG_RESET();
	} else if (pGPIOx == GPIOK)
	{
		GPIOK_REG_RESET();
	}
}

/***********************************************************************
 *
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- Read data from GPIO input pin
 *
 * @param[in]	- Base address of GPIO peripheral
 * @param[in]	- Pin number of the GPIO port
 *
 * @return		- uint8_t - 0 or 1
 *
 * @Note		- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001)); //Bit extraction with mask

	return value;
}

/***********************************************************************
 *
 * @fn			- GPIO_ReadFromInputPort
 *
 * @brief		- Read data from GPIO input port
 *
 * @param[in]	- Base address of GPIO peripheral
 *
 * @return		- uint16_t
 *
 * @Note		- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/***********************************************************************
 *
 * @fn			- GPIO_WriteToOutputPin
 *
 * @brief		- Write to GPIO output pin
 *
 * @param[in]	- Base address of GPIO peripheral
 * @param[in]	- Pin number
 * @param[in]	- value/data to write to the pin
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/***********************************************************************
 *
 * @fn			- GPIO_WriteToOutputPort
 *
 * @brief		- Write to GPIO output port
 *
 * @param[in]	- Base address of GPIO peripheral
 * @param[in]	- value/data to write to the port
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/***********************************************************************
 *
 * @fn			- GPIO_ToggleOutputPin
 *
 * @brief		- Toggle output pin
 *
 * @param[in]	- Base address of GPIO peripheral
 * @param[in]	- Pin number
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/***********************************************************************
 *
 * @fn			- GPIO_IRQInterruptConfig
 *
 * @brief		- Configure IRQ for GPIO
 *
 * @param[in]	- IRQNumber
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			// Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			// Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		} else if (IRQNumber >= 96 && IRQNumber < 128)
		{
			// Program ISER3 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 96));
		}
	} else {
		if (IRQNumber <= 31)
		{
			// Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			// Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		} else if (IRQNumber >= 96 && IRQNumber < 128)
		{
			// Program ICER3 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 96));
		}
	}
}

/***********************************************************************
 *
 * @fn			- GPIO_IRQPriorityConfig
 *
 * @brief		- Configure IRQ Priority for GPIO
 *
 * @param[in]	- IRQNumber
 * @param[in]	- IRQPriority
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//Find IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = ((IRQNumber % 4) * 8);
	/**
	 * Out of 8bits, only second half(bit4 to bit8) are implemented.
	 * So, to start from second half, we use the following:
	 * No.of bits(8) - Not_Implemented_Bits(4) = 4(Second half)
	 */
	uint8_t shift_amount = (iprx_section + (8 - NO_PR_BITS_IMPLEMENTED));

	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/***********************************************************************
 *
 * @fn			- GPIO_IRQHandling
 *
 * @brief		- IRQ handling
 *
 * @param[in]	- Pin number
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}

