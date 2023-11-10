#include "stm32f767xx_spi_driver.h"


void static SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
void static SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
void static SPI_OVR_Interrupt_Handle(SPI_Handle_t *pSPIHandle);

/***********************************************************************
 *
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- Enables or disables peripheral clock for a given GPIO port
 *
 * @param[in]	- Base address of SPI peripheral
 * @param[in]	- ENABLE OR DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		} else if (pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		} else if (pSPIx == SPI6)
		{
			SPI6_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		} else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		} else if (pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		} else if (pSPIx == SPI6)
		{
			SPI6_PCLK_DI();
		}
	}
}

/***********************************************************************
 *
 * @fn			- SPI_Init
 *
 * @brief		- Initialise a SPI port
 *
 * @param[in]	- SPI_Handle_t
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	SPI_Config_t spiConfig = pSPIHandle->SPIConfig;

	//Configure Device Mode
	pSPIHandle->pSPIx->CR1 |= (spiConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	//Configure Bus Config
	if (spiConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDIMODE should be cleared
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (spiConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDIMODE should be set
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDIMODE);
	} else if (spiConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDIMODE should be cleared
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY to set
		pSPIHandle->pSPIx->CR1 |= (spiConfig.SPI_RXONLY << SPI_CR1_RXONLY);
	}

	//Configure Clock speed (Baud Rate)
	pSPIHandle->pSPIx->CR1 &= ~(7 << SPI_CR1_BR);//Clear the bits
	pSPIHandle->pSPIx->CR1 |= (spiConfig.SPI_SclkSpeed << SPI_CR1_BR);//Set the bits

	//Configure CPOL
	pSPIHandle->pSPIx->CR1 |= (spiConfig.SPI_CPOL << SPI_CR1_CPOL);

	//Configure CPHA
	pSPIHandle->pSPIx->CR1 |= (spiConfig.SPI_CPHA << SPI_CR1_CPHA);

	//Configure SSM
	pSPIHandle->pSPIx->CR1 |= (spiConfig.SPI_SSM << SPI_CR1_SSM);

	//Configure DS
	pSPIHandle->pSPIx->CR2 &= ~(0xF << SPI_CR2_DS);//Clear the bits
	pSPIHandle->pSPIx->CR2 |= (spiConfig.SPI_DS << SPI_CR2_DS);//Set the bits

	//Configure FRXTH
	pSPIHandle->pSPIx->CR2 |= (spiConfig.SPI_FRXTH << SPI_CR2_FRXTH);
}

/***********************************************************************
 *
 * @fn			- SPI_DeInit
 *
 * @brief		- DeInitialise a SPI port
 *
 * @param[in]	- SPI_RegDef_t
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	} else if (pSPIx == SPI5)
	{
		SPI5_REG_RESET();
	} else if (pSPIx == SPI6)
	{
		SPI6_REG_RESET();
	}
}

/***********************************************************************
 *
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- Enables or disables peripheral clock for a given GPIO port
 *
 * @param[in]	- Base address of SPI peripheral
 * @param[in]	- ENABLE OR DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/***********************************************************************
 *
 * @fn			- SPI_SendData
 *
 * @brief		- Send data over SPI port
 *
 * @param[in]	- SPI_RegDef_t
 * @param[in]	- uint8_t
 * @param[in]	- uint32_t
 *
 * @return		- none
 *
 * @Note		- This is a blocking call, as it waits for all the data gets transmitted.
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//Wait until the Tx buffer get's empty
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		if (((pSPIx->CR2 >> 8) & (0xF << 0)) == SPI_DS_16BIT)
		{
			/**
			 * @TODO check whether DS is from 9 to 16bit
			 * 	If so, then use the 16bit transmission
			 */

			//Load DR with 2 bytes of data
			pSPIx->DR = *((uint16_t*) pTxBuffer);//Set the bits

			//Decrement the Length
			Len--;
			Len--;

			//Increment the Tx buffer
			(uint16_t*) pTxBuffer++;
		} else if (((pSPIx->CR2 >> 8) & (0xF << 0)) == SPI_DS_8BIT) {
//			pSPIx->DR = *pTxBuffer;//Set the bits
			*((__vo uint8_t *)&pSPIx->DR) = *pTxBuffer;

			//Decrement the length
			Len--;

			//Increment the Tx buffer
			pTxBuffer++;
		}

		//Clear OVR flag
//		SPI_ClearOVRFlag(pSPIx);
	}
}

/***********************************************************************
 *
 * @fn			- SPI_ReceiveData
 *
 * @brief		- Receive data over SPI port
 *
 * @param[in]	- SPI_RegDef_t
 * @param[in]	- uint8_t
 * @param[in]	- uint32_t
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	pSPIx->DR &= ~(0xFFFF << 0);//Clear the bits

	while (Len > 0)
	{
		//Wait until the Rx buffer get's empty
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//Check whether DFF is a 8-bit or 16-bit
		if (pSPIx->CR2 & (SPI_DS_8BIT << SPI_CR2_DS))
		{
			*pRxBuffer = pSPIx->DR;//Load the data from DR into RxBuffer

			//Decrement the length
			Len--;

			//Increment the Rx buffer
			pRxBuffer++;
		} else if (pSPIx->CR2 & (SPI_DS_16BIT << SPI_CR2_DS))
		{
			//Load DR with 2 bytes of data
			*((uint16_t*) pRxBuffer) = pSPIx->DR;//Load the data from DR into RxBuffer

			//Decrement the Length
			Len--;
			Len--;

			//Increment the Tx buffer
			(uint16_t*) pRxBuffer++;
		}
	}
}

/***********************************************************************
 *
 * @fn			- SPI_PeripheralControl
 *
 * @brief		- Enables or disables SPI peripheral
 *
 * @param[in]	- Base address of SPI peripheral
 * @param[in]	- ENABLE OR DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/***********************************************************************
 *
 * @fn			- SPI_SSIConfig
 *
 * @brief		- Enables or disables SPI SSI
 *
 * @param[in]	- Base address of SPI peripheral
 * @param[in]	- ENABLE OR DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/***********************************************************************
 *
 * @fn			- SPI_SSIConfig
 *
 * @brief		- Enables or disables SPI SSI
 *
 * @param[in]	- Base address of SPI peripheral
 * @param[in]	- ENABLE OR DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/***********************************************************************
 *
 * @fn			- SPI_IRQInterruptConfig
 *
 * @brief		- Configure IRQ for SPI
 *
 * @param[in]	- IRQNumber
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			// Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	} else {
		if (IRQNumber <= 31)
		{
			// Program ISER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// Program ISER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ISER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/***********************************************************************
 *
 * @fn			- SPI_IRQPriorityConfig
 *
 * @brief		- Configure IRQ Priority for SPI
 *
 * @param[in]	- IRQNumber
 * @param[in]	- IRQPriority
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//Find IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = ((IRQNumber % 4) * 8);
	uint8_t shift_amount = (iprx_section + (8 - NO_PR_BITS_IMPLEMENTED));

	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/***********************************************************************
 *
 * @fn			- SPI_SendDataWithIT
 *
 * @brief		- Send data over SPI port using Interrupts
 *
 * @param[in]	- SPI_Handle_t
 * @param[in]	- uint8_t
 * @param[in]	- uint32_t
 *
 * @return		- uint8_t
 *
 * @Note		- This is based on Interrupts.
 */
uint8_t SPI_SendDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX)
	{
		//1. Store the TxBuffer and Len in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as BUSY in Tx
		//so that no other code can take over SPI Tx until the current one is over.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3 If TXE flag is set, then enable Tx Interrupt
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}

/***********************************************************************
 *
 * @fn			- SPI_ReceiveDataWithIT
 *
 * @brief		- Receive data over SPI port using Interrupts
 *
 * @param[in]	- SPI_RegDef_t
 * @param[in]	- uint8_t
 * @param[in]	- uint32_t
 *
 * @return		- uint8_t
 *
 * @Note		- This is based on Interrupts.
 */
uint8_t SPI_ReceiveDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX)
	{
		//1. Store the RxBuffer and Len in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as BUSY in Rx
		//so that no other code can take over SPI Rx until the current one is over.
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3 If RXNE flag is set, then enable Rx Interrupt
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
}

/***********************************************************************
 *
 * @fn			- SPI_IRQHandling
 *
 * @brief		- IRQ handling
 *
 * @param[in]	- SPI_Handle_t
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	temp1 = SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_SR_TXE);
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE));

	if (temp1 && temp2)
	{
		//Handle TXE
		SPI_TXE_Interrupt_Handle(pSPIHandle);
	}

	temp1 = SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_SR_RXNE);
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE));

	if (temp1 && temp2)
	{
		//Handle RXNE
		SPI_RXNE_Interrupt_Handle(pSPIHandle);
	}

	temp1 = SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_SR_OVR);
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE));

	if (temp1 && temp2)
	{
		//Handle OVR
		SPI_OVR_Interrupt_Handle(pSPIHandle);
	}
}

void static SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	//Check whether DFF is a 8-bit or 16-bit
	if (pSPIHandle->pSPIx->CR2 & (SPI_DS_8BIT << SPI_CR2_DS))
	{
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;//Set the bits

		//Decrement the length
		pSPIHandle->TxLen--;

		//Increment the Tx buffer
		pSPIHandle->pTxBuffer++;
	} else if (pSPIHandle->pSPIx->CR2 & (SPI_DS_16BIT << SPI_CR2_DS))
	{
		//Load DR with 2 bytes of data
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);//Set the bits

		//Decrement the Length
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;

		//Increment the Tx buffer
		(uint16_t*) pSPIHandle->pTxBuffer++;
	}

	if (! pSPIHandle->TxLen)
	{
		//TxLen is Zero, then close the SPI Tx
		//This prevents interrupts from setting the TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

void static SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	//Check whether DFF is a 8-bit or 16-bit
	if (pSPIHandle->pSPIx->CR2 & (SPI_DS_8BIT << SPI_CR2_DS))
	{
		*pSPIHandle->pRxBuffer = (uint8_t) pSPIHandle->pSPIx->DR;//Load the data from DR into RxBuffer

		//Decrement the length
		pSPIHandle->RxLen--;

		//Increment the Rx buffer
		pSPIHandle->pRxBuffer++;
	} else if (pSPIHandle->pSPIx->CR2 & (SPI_DS_16BIT << SPI_CR2_DS))
	{
		//Load DR with 2 bytes of data
		*((uint16_t*) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;//Load the data from DR into RxBuffer

		//Decrement the Length
		pSPIHandle->RxLen -= 2;

		//Increment the Tx buffer
		(uint16_t*) pSPIHandle->pRxBuffer++;
	}

	if (! pSPIHandle->RxLen)
		{
			//TxLen is Zero, then close the SPI Rx
			//This prevents interrupts from setting the RXNE flag
			SPI_CloseReception(pSPIHandle);

			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		}
}

void static SPI_OVR_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	//1. clear the OVR flag.
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	(void) temp;

	//2. Inform the application.
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void) temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	//This is a weak implementation. User application needs to implement.
}

