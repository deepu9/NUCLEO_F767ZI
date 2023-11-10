#ifndef INC_STM32F767XX_I2C_DRIVER_H_
#define INC_STM32F767XX_I2C_DRIVER_H_

#include "stm32f767xx.h"


/**
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint8_t I2C_SCLSpeed;				/*!< possible values from @I2C_SCLSpeed >*/
	uint8_t I2C_DeviceAddress;			/*!< Applicable only when peripheral is Slave >*/
}I2C_Config_t;

/**
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;


/**
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM10K			(0)
#define I2C_SCL_SPEED_SM100K		(1)
#define I2C_SCL_SPEED_FM			(2)
#define I2C_SCL_SPEED_FMPPLUS		(3)

/**
 * I2C related status flags definition
 */
#define I2C_FLAG_TXE				(1 << I2C_SR_TXE)
#define I2C_FLAG_TXIS				(1 << I2C_SR_TXIS)
#define I2C_FLAG_RXNE				(1 << I2C_SR_RXNE)
#define I2C_FLAG_ADDR				(1 << I2C_SR_ADDR)
#define I2C_FLAG_NACKF				(1 << I2C_SR_NACKF)
#define I2C_FLAG_STOPF				(1 << I2C_SR_STOPF)
#define I2C_FLAG_TC					(1 << I2C_SR_TC)
#define I2C_FLAG_TCR				(1 << I2C_SR_TCR)
#define I2C_FLAG_BERR				(1 << I2C_SR_BERR)
#define I2C_FLAG_ARLO				(1 << I2C_SR_ARLO)
#define I2C_FLAG_OVR				(1 << I2C_SR_OVR)
#define I2C_FLAG_PECERR				(1 << I2C_SR_PECERR)
#define I2C_FLAG_TIMEOUT			(1 << I2C_SR_TIMEOUT)
#define I2C_FLAG_ALERT				(1 << I2C_SR_ALERT)
#define I2C_FLAG_BUSY				(1 << I2C_SR_BUSY)
#define I2C_FLAG_DIR				(1 << I2C_SR_DIR)


/**
 * Others
 */
#define SYS_CLK_16M					(16000000U)
#define SYS_CLK_8M					(8000000U)


/************************************************************************
 * API's supported by STM32F767xx
 ***********************************************************************/
/**
 * Peripheral RCC clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/**
 * Init and De-Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/**
 * Other Peripheral control API's
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName);

#endif /* INC_STM32F767XX_I2C_DRIVER_H_ */
