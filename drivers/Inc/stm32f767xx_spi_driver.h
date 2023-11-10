/*
 * stm32f767xx_spi_driver.h
 *
 */

#ifndef INC_STM32F767XX_SPI_DRIVER_H_
#define INC_STM32F767XX_SPI_DRIVER_H_

#include "stm32f767xx.h"


typedef struct
{
	uint8_t SPI_DeviceMode;		/*!< DeviceMode - Possible values from @SPI_DeviceMode >*/
	uint8_t SPI_BusConfig;		/*!< BusConfig - Possible values from @SPI_BusConfig >*/
	uint8_t SPI_SclkSpeed;		/*!< Serial Clock Speed - Possible values from @SPI_SclkSpeed>*/
	uint8_t SPI_DS;				/*!< Data sizes - Possible values from @SPI_DS >*/
	uint8_t SPI_CPOL;			/*!< Clock Polarity - Possible values from @SPI_CPOL >*/
	uint8_t SPI_CPHA;			/*!< Clock Phase - Possible values from @SPI_CPHA >*/
	uint8_t SPI_SSM;			/*!< Slave Select Management - Possible values from @SPI_SSM >*/
	uint8_t SPI_RXONLY;			/*!< RXONLY - Possible values from @SPI_RXONLY >*/
	uint8_t SPI_FRXTH;			/*!< FRXTH - Possible values from @SPI_FRXTH >*/
}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;

/**
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER			(1)
#define SPI_DEVICE_MODE_SLAVE			(0)

/**
 * @SPI_BusConfig
 * FD - Full Duplex
 * HD - Half Duplex
 */
#define SPI_BUS_CONFIG_FD				(1)
#define SPI_BUS_CONFIG_HD				(2)
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	(3)

/**
 * @SPI_SclkSpeed
 * SPI -> CR1 -> BR
 */
#define SPI_SCLK_SPEED_DIV2				(0)
#define SPI_SCLK_SPEED_DIV4				(1)
#define SPI_SCLK_SPEED_DIV8				(2)
#define SPI_SCLK_SPEED_DIV16			(3)
#define SPI_SCLK_SPEED_DIV32			(4)
#define SPI_SCLK_SPEED_DIV64			(5)
#define SPI_SCLK_SPEED_DIV128			(6)
#define SPI_SCLK_SPEED_DIV256			(7)

/**
 * @SPI_DS
 */
#define SPI_DS_4BIT						(3)
#define SPI_DS_5BIT						(4)
#define SPI_DS_6BIT						(5)
#define SPI_DS_7BIT						(6)
#define SPI_DS_8BIT						(7)
#define SPI_DS_9BIT						(8)
#define SPI_DS_10BIT					(9)
#define SPI_DS_11BIT					(10)
#define SPI_DS_12BIT					(11)
#define SPI_DS_13BIT					(12)
#define SPI_DS_14BIT					(13)
#define SPI_DS_15BIT					(14)
#define SPI_DS_16BIT					(15)

/**
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH					(1)
#define SPI_CPOL_LOW					(0)

/**
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH					(1)
#define SPI_CPHA_LOW					(0)

/**
 * @SPI_SSM
 */
#define SPI_SSM_EN						(1)
#define SPI_SSM_DI						(0)

/**
 * @SPI_RXONLY
 */
#define SPI_RXONLY_EN					(1)
#define SPI_RXONLY_DI					(0)

/**
 * @SPI_FRXTH
 */
#define SPI_FRXTH_8BIT					(1)
#define SPI_FRXTH_16BIT					(0)

/**
 * SPI Flags
 */
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG					(1 << SPI_SR_BSY)

/**
 * SPI Application States
 */
#define SPI_READY						(0)
#define SPI_BUSY_IN_RX					(1)
#define SPI_BUSY_IN_TX					(2)

/**
 * Possible SPI Application Events
 */
#define SPI_EVENT_TX_CMPLT				(1)		/*!< SPI Transmission Event Complete >*/
#define SPI_EVENT_RX_CMPLT				(2)		/*!< SPI Receive Event Complete >*/
#define SPI_EVENT_OVR_ERR				(3)		/*!< SPI Overrun Error Event Complete >*/
#define SPI_EVENT_CRC_ERR				(4)		/*!< SPI CRC error Event Complete >*/

/************************************************************************
 * API's supported by STM32F767xx
 ***********************************************************************/
/**
 * Peripheral RCC clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * Data Send and Receive
 * Blocking type
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/**
 * Interrupt based API's to send and receive data
 * Non-Blocking type
 */
uint8_t SPI_SendDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


/**
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/**
 * Other Peripheral control API's
 */
//To enable/disable SPI peripheral
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//If SSI is 0(default), then Master-Slave doesn't work.
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//To enable/set NSS, SSOE should be SET
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);

void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/**
 * Application Callbacks
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);

#endif /* INC_STM32F767XX_SPI_DRIVER_H_ */
