#include "stm32f767xx_i2c_driver.h"


static uint32_t I2C_SetTimingRegister(uint32_t pclk, uint8_t index);
static void I2C_DigitalNoiseFilterControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
static void I2C_AnalogNoiseFilterOFFControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
static void I2C_ClkStretchDisable(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_TransferDirectionControl(I2C_RegDef_t *pI2Cx, uint8_t RDWR);

static void I2C_MasterCommunicationInit(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t RD_WRN);

static void I2C_SlaveInit(I2C_Handle_t *pI2CHandle);
static void I2C_SlaveOwnAddressControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/***********************************************************************
 *
 * @fn			- I2C_PeriClockControl
 *
 * @brief		- Enables or disables peripheral clock for a given GPIO port
 *
 * @param[in]	- Base address of I2C peripheral
 * @param[in]	- ENABLE OR DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		} else if (pI2Cx == I2C4)
		{
			I2C4_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		} else if (pI2Cx == I2C4)
		{
			I2C4_PCLK_DI();
		}
	}
}

/***********************************************************************
 *
 * @fn			- I2C_Init
 *
 * @brief		- Initialise a I2C port
 *
 * @param[in]	- I2C_Handle_t
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	//1. Clear PE(Enable Peripheral) bit in I2C_CR1 register
	I2C_PeripheralControl(pI2CHandle->pI2Cx, DISABLE);

	//2. Configuration of ANFOFF and DNF bits
	I2C_DigitalNoiseFilterControl(pI2CHandle->pI2Cx, DISABLE);
	I2C_AnalogNoiseFilterOFFControl(pI2CHandle->pI2Cx, ENABLE);

	//Slave Initialization
	I2C_SlaveInit(pI2CHandle);

	//3. Configuration of PRESC, SDADEL, SCLDEL, SCLH, SCLL bits in I2C_TIMINGR
	uint32_t pclk = RCC_GetPCLK1Value();
	uint32_t i2cTimingReg = I2C_SetTimingRegister(pclk, pI2CHandle->I2C_Config.I2C_SCLSpeed);

	pI2CHandle->pI2Cx->TIMINGR |= i2cTimingReg;

	//4. Configure NOSTRETCH in I2C_CR1
	I2C_ClkStretchDisable(pI2CHandle->pI2Cx, ENABLE);

	//5. Set PE(Enable Peripheral) bit in I2C_CR1 register
	I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
}

/***********************************************************************
 *
 * @fn			- I2C_DeInit
 *
 * @brief		- DeInitialise a I2C port
 *
 * @param[in]	- I2C_RegDef_t
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	} else if (pI2Cx == I2C4)
	{
		I2C4_REG_RESET();
	}
}

/***********************************************************************
 *
 * @fn			- I2C_MasterSendData
 *
 * @brief		- Master transmits data
 *
 * @param[in]	- I2C Handle
 * @param[in]	- Transmission buffer
 * @param[in]	- Number of bytes to transfer
 * @param[in]	- Slave address
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//Master Communication Init
	I2C_MasterCommunicationInit(pI2CHandle, SlaveAddr, I2C_WR_TRANSFER);

	//Set NBYTES and AUTOEND
	pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);

	//Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Confirm that the address phase is completed
	while (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) == FLAG_RESET);

	while (Len > 0)
	{
		//Wait till the TXE is SET
		while (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) == FLAG_RESET);

		//Write I2C_TXDR
		pI2CHandle->pI2Cx->TXDR = *pTxBuffer;

		pTxBuffer++;

		Len--;
	}

	//Wait until TC flag is set(when NBYTES are transferred)
	while (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TC) == FLAG_RESET);


	//Generate STOP condition
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

/***********************************************************************
 *
 * @fn			- I2C_MasterReceiveData
 *
 * @brief		- Master receive data
 *
 * @param[in]	- I2C Handle
 * @param[in]	- Receive buffer
 * @param[in]	- Number of bytes to receive
 * @param[in]	- Slave address
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//Master Communication Init
	I2C_MasterCommunicationInit(pI2CHandle, SlaveAddr, I2C_RD_TRANSFER);

	//Set NBYTES and AUTOEND
	pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);

	//Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Confirm that the address phase is completed
	while (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) == FLAG_RESET);

	while (Len > 0)
	{
		//Wait till the RXNE is SET
		while (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) == FLAG_RESET);

		//Write I2C_RXDR
		*pRxBuffer = pI2CHandle->pI2Cx->RXDR;

		pRxBuffer++;

		Len--;
	}

	//Wait until TC flag is set(when NBYTES are transferred)
	while (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TC) == FLAG_RESET);


	//Generate STOP condition
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

static void I2C_MasterCommunicationInit(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t RD_WRN)
{
	//1. Set 7-bit address mode
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ADD10);

	//2. Set Slave address to be sent: SADD[7:1]
	pI2CHandle->pI2Cx->CR2 |= (SlaveAddr << I2C_CR2_SADD1_7);

	//3. Transfer direction: RD_WRN
	I2C_TransferDirectionControl(pI2CHandle->pI2Cx, RD_WRN);
}

/***********************************************************************
 *
 * @fn			- I2C_PeripheralControl
 *
 * @brief		- Enables or disables I2C peripheral
 *
 * @param[in]	- Base address of I2C peripheral
 * @param[in]	- ENABLE OR DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

static void I2C_SlaveInit(I2C_Handle_t *pI2CHandle)
{
	//1. clear OA1EN in I2C_OAR1
	I2C_SlaveOwnAddressControl(pI2CHandle->pI2Cx, DISABLE);

	//3. Configure OA1[7:1], OA1EN
	pI2CHandle->pI2Cx->OAR1 &= ~(1 << I2C_OAR1_OA1_MODE);//Clear the I2C slave own address to mode to set to 7-bit address
	pI2CHandle->pI2Cx->OAR1 |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_OA1_ADDR1_7;

	//3. Enable I2C OA1EN
	I2C_SlaveOwnAddressControl(pI2CHandle->pI2Cx, ENABLE);

	//4. Enable interrupts and/or DMA
}

/***********************************************************************
 *
 * @fn			- I2C_DigitalNoiseFilterControl
 *
 * @brief		- Enables or disables Digital Noise Filter(DNF)
 *
 * @param[in]	- Base address of I2C peripheral
 * @param[in]	- ENABLE OR DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
static void I2C_DigitalNoiseFilterControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_DNF);
	} else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_DNF);
	}
}

/***********************************************************************
 *
 * @fn			- I2C_AnalogNoiseFilterOFFControl
 *
 * @brief		- Enables or disables Analog Noise Filter(ANFOFF)
 *
 * @param[in]	- Base address of I2C peripheral
 * @param[in]	- ENABLE OR DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
static void I2C_AnalogNoiseFilterOFFControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ANFOFF);
	} else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ANFOFF);
	}
}

/***********************************************************************
 *
 * @fn			- I2C_ClkStretchDisable
 *
 * @brief		- Enables or disables Clock stretching(NOSTRETCH)
 *
 * @param[in]	- Base address of I2C peripheral
 * @param[in]	- ENABLE OR DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
static void I2C_ClkStretchDisable(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_NOSTRERCH);
	} else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_NOSTRERCH);
	}
}

static uint32_t I2C_SetTimingRegister(uint32_t pclk, uint8_t index)
{
	uint32_t i2c_timings_8[4]  = {0x1042C3C7, 0x10420F13, 0x00310309, 0x00100306};
	uint32_t i2c_timings_16[4] = {0x3042C3C7, 0x303D5B, 0x10320309, 0x00200204}; // SM_100k timing value from STM32CubeMX, for avoiding glitch (it works)
	uint32_t i2c_timings_48[4] = {0xB042C3C7, 0xB0420F13, 0x50330309, 0x50100103};
	uint32_t result;

	if (pclk == 8000000)
	{
		result = i2c_timings_8[index];
	} else if (pclk == 16000000)
	{
		result = i2c_timings_16[index];
	} else if (pclk == 48000000)
	{
		result = i2c_timings_48[index];
	}

	return result;
}

static void I2C_SlaveOwnAddressControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->OAR1 |= (1 << I2C_OAR1_OA1_EN);
	} else
	{
		pI2Cx->OAR1 &= ~(1 << I2C_OAR1_OA1_EN);
	}
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR2 |= (1 << I2C_CR2_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR2 |= (1 << I2C_CR2_STOP);
}

static void I2C_TransferDirectionControl(I2C_RegDef_t *pI2Cx, uint8_t RD_WR)
{
	if (RD_WR == I2C_WR_TRANSFER)
	{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_RD_WRN);
	} else {
		pI2Cx->CR2 |= (1 << I2C_CR2_RD_WRN);
	}
}

/***********************************************************************
 *
 * @fn			- I2C_PeriClockControl
 *
 * @brief		- Enables or disables peripheral clock for a given GPIO port
 *
 * @param[in]	- Base address of I2C peripheral
 * @param[in]	- ENABLE OR DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName)
{
	if (pI2Cx->ISR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

