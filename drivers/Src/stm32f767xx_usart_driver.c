#include "stm32f767xx_usart_driver.h"


void USART_ConfigureCR1(USART_Handle_t *pUSART_Handle);
void USART_ConfigureCR2(USART_Handle_t *pUSART_Handle);
void USART_ConfigureCR3(USART_Handle_t *pUSART_Handle);


void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		} else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		} else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		} else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		} else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		} else if (pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		} else if (pUSARTx == UART7)
		{
			UART7_PCLK_EN();
		} else if (pUSARTx == UART8)
		{
			UART8_PCLK_EN();
		}
	} else {
		if (pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		} else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		} else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		} else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		} else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		} else if (pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		} else if (pUSARTx == UART7)
		{
			UART7_PCLK_DI();
		} else if (pUSARTx == UART8)
		{
			UART8_PCLK_DI();
		}
	}
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	} else {
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

void USART_Init(USART_Handle_t *pUSART_Handle)
{
	// peripheral clock enable
	USART_PeriClockControl(pUSART_Handle->pUSARTx, ENABLE);

	// Configure CR1
	USART_ConfigureCR1(pUSART_Handle);

	// Configure CR2
	USART_ConfigureCR2(pUSART_Handle);

	// Configure CR3
	USART_ConfigureCR3(pUSART_Handle);

	/********************* COnfiguration of BRR ***********************************/
//	USART_SetBaudRate(pUSART_Handle);
}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if (pUSARTx == USART1)
	{
		USART1_REG_RESET();
	} else if (pUSARTx == USART2)
	{
		USART2_REG_RESET();
	} else if (pUSARTx == USART3)
	{
		USART3_REG_RESET();
	} else if (pUSARTx == UART4)
	{
		UART4_REG_RESET();
	} else if (pUSARTx == UART5)
	{
		UART5_REG_RESET();
	} else if (pUSARTx == USART6)
	{
		USART6_REG_RESET();
	} else if (pUSARTx == UART7)
	{
		UART7_REG_RESET();
	} else if (pUSARTx == UART8)
	{
		UART8_REG_RESET();
	}
}

void USART_SendData(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint32_t Len)
{
	for (uint32_t i = 0; i < Len; i++)
	{
		while (! USART_GetFlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_TXE));

		if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_7BITS)
		{
			pUSART_Handle->pUSARTx->TDR |= (*pTxBuffer & (uint8_t)0x7F);

			//Whether parity is enabled or disabled, 1 byte has to be transferred.
			pTxBuffer++;
		} else if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			pUSART_Handle->pUSARTx->TDR |= (uint16_t)pTxBuffer & (uint16_t)0x1FF;

			pTxBuffer++;

			if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				/**
				 * When parity is disabled, then all 9bits are used for data.
				 * To transfer 9bits, we need two bytes. So, increment the buffer again.
				 */
				pTxBuffer++;
			}
		} else
		{
			//If word length is 8 bits
			pUSART_Handle->pUSARTx->TDR |= (*pTxBuffer & (uint8_t)0xFF);

			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the ISR
	while (! USART_GetFlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint32_t Len);
void USART_SendDataIT(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveDataIT(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint32_t Len);

/**
 * Calculations are from Reference manual
 */
//void USART_SetBaudRate(USART_Handle_t *pUSART_Handle)
//{
//	uint32_t usartdiv, PCLKx;
//
//	if (pUSART_Handle->pUSARTx == USART1 || pUSART_Handle->pUSARTx == USART2
//		|| pUSART_Handle->pUSARTx == USART3 || pUSART_Handle->pUSARTx == USART6
//	) {
//		PCLKx = RCC_GetPCLK2Value();
//	} else {
//		PCLKx = RCC_GetPCLK1Value();
//	}
//
//	unsigned int last_4_bits, rest_of_bits;
//
//	usartdiv = PCLKx / pUSART_Handle->USART_Config.USART_Baud;
//
//	if (pUSART_Handle->pUSARTx->CR1 & (1 << USART_CR1_OVER8))
//	{
//		usartdiv *= 2;
//		//Mask last 4 bits and shift them by 1 bit
//		last_4_bits = ((usartdiv & 0x000F) >> 1);
//		/**
//		 * Get the MSB bits upto the 3rd bit position(excluding)
//		 * Mask the last four bits
//		 */
//		rest_of_bits = (usartdiv & ~(0x000F << 0));
//		usartdiv = rest_of_bits + last_4_bits;
//	}
//
//	pUSART_Handle->pUSARTx->BRR = dec2hex(usartdiv);
//}

void USART_ConfigureCR1(USART_Handle_t *pUSART_Handle)
{
	//Mode select - Enable Tx and Rx based on USART_Mode config.
	if (pUSART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_TE);
	} else if (pUSART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_RE);
	} else if (pUSART_Handle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		pUSART_Handle->pUSARTx->CR1 &= ~(3 << USART_CR1_RE);
		pUSART_Handle->pUSARTx->CR1 |= (3 << USART_CR1_RE);
	}

	// Configure word length
	if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
	{
		pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_M0);
		pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_M1);
	} else if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
	{
		pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_M0);
		pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_M1);
	} else if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
	{
		pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_M0);
		pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_M1);
	}

	// Configure parity control bit fields
	pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_PCE); //Enable Parity Control

	if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_PS);
	}
}

void USART_ConfigureCR2(USART_Handle_t *pUSART_Handle)
{
	pUSART_Handle->pUSARTx->CR2 &= ~(pUSART_Handle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);
	pUSART_Handle->pUSARTx->CR2 |= (pUSART_Handle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);
}

void USART_ConfigureCR3(USART_Handle_t *pUSART_Handle)
{
	if (
		pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS
		|| pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS
	)
	{
		pUSART_Handle->pUSARTx->CR3 |= (1 << USART_CR3_CTSE);
	}

	if (
		pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS
		|| pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS
	)
	{
		pUSART_Handle->pUSARTx->CR3 |= (1 << USART_CR3_RTSE);
	}
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if (pUSARTx->ISR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}
