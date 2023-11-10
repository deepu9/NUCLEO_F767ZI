#ifndef INC_STM32F767XX_GPIO_DRIVER_H_
#define INC_STM32F767XX_GPIO_DRIVER_H_

#include "stm32f767xx.h"


/**
 * GPIO pin configuration structure
 */
typedef struct {
	uint8_t GPIO_PinNumber;			/*!< possible values from @GPIO_PIN_NUMBER >*/
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_OUT_SPEEDS >*/
	uint8_t GPIO_PinPuPdControl;	/*!< possible values from @GPIO_PUPD >*/
	uint8_t GPIO_PinOpType;			/*!< possible values from @GPIO_OUT_TYPES >*/
	uint8_t GPIO_PinAltFunMode;		/*!< possible values from @GPIO_PinAltFunMode >*/
}GPIO_PinConfig_t;

/**
 * Handler structure for a GPIO pin
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx;				/*!< Base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*!< GPIO pin configuration settings >*/
}GPIO_Handle_t;

/**
 * @GPIO_PIN_NUMBER
 * GPIO Pin numbers
 */
#define GPIO_PIN_0				(0)
#define GPIO_PIN_1				(1)
#define GPIO_PIN_2				(2)
#define GPIO_PIN_3				(3)
#define GPIO_PIN_4				(4)
#define GPIO_PIN_5				(5)
#define GPIO_PIN_6				(6)
#define GPIO_PIN_7				(7)
#define GPIO_PIN_8				(8)
#define GPIO_PIN_9				(9)
#define GPIO_PIN_10				(10)
#define GPIO_PIN_11				(11)
#define GPIO_PIN_12				(12)
#define GPIO_PIN_13				(13)
#define GPIO_PIN_14				(14)
#define GPIO_PIN_15				(15)

/**
 * @GPIO_PIN_MODES
 * GPIO modes
 */
#define GPIO_MODE_IN			(0)		/*!< Input mode >*/
#define GPIO_MODE_OUT			(1)		/*!< Output mode >*/
#define GPIO_MODE_ALTFN			(2)		/*!< Alternate function mode >*/
#define GPIO_MODE_ANALOG		(3)		/*!< Analog mode >*/
#define GPIO_MODE_IT_FET		(4)		/*!< Interrupt Falling Edge Trigger mode >*/
#define GPIO_MODE_IT_RET		(5)		/*!< Interrupt Rising Edge Trigger mode >*/
#define GPIO_MODE_IT_REFET		(6)		/*!< Interrupt Falling and Rising Edge Trigger mode >*/

/**
 * @GPIO_OUT_TYPES
 * GPIO Output Types
 */
#define GPIO_OUT_TYPE_PP		(0)		/*!< Push-Pull >*/
#define GPIO_OUT_TYPE_OD		(1)		/*!< Open-Drain >*/

/**
 * @GPIO_OUT_SPEEDS
 * GPIO Output Speed
 */
#define GPIO_OUT_SPEED_LOW		(0)		/*!< Low Speed >*/
#define GPIO_OUT_SPEED_MEDIUM	(1)		/*!< Medium Speed >*/
#define GPIO_OUT_SPEED_HIGH		(2)		/*!< High Speed >*/
#define GPIO_OUT_SPEED_VHIGH	(3)		/*!< Very High Speed >*/

/**
 * @GPIO_PUPD
 * GPIO Pull-up/Pull-down
 */
#define GPIO_NO_PUPD			(0)		/*!< No Pull-up/Pull-down >*/
#define GPIO_PU					(1)		/*!< Pull-up >*/
#define GPIO_PD					(2)		/*!< Pull-down >*/

/**
 * @GPIO_PinAltFunMode
 */
#define GPIO_AFR0				(0)
#define GPIO_AFR1				(1)
#define GPIO_AFR2				(2)
#define GPIO_AFR3				(3)
#define GPIO_AFR4				(4)
#define GPIO_AFR5				(5)
#define GPIO_AFR6				(6)
#define GPIO_AFR7				(7)
#define GPIO_AFR8				(8)
#define GPIO_AFR9				(9)
#define GPIO_AFR10				(0xA)
#define GPIO_AFR11				(0xB)
#define GPIO_AFR12				(0xC)
#define GPIO_AFR13				(0xD)
#define GPIO_AFR14				(0xE)
#define GPIO_AFR15				(0xF)

/**
 * EXTICR - EXTI Configuration Register GPIO codes
 */
#define EXTI_CRx_GPIOA			(0)
#define EXTI_CRx_GPIOB			(1)
#define EXTI_CRx_GPIOC			(2)
#define EXTI_CRx_GPIOD			(3)
#define EXTI_CRx_GPIOE			(4)
#define EXTI_CRx_GPIOF			(5)
#define EXTI_CRx_GPIOG			(6)
#define EXTI_CRx_GPIOH			(7)
#define EXTI_CRx_GPIOI			(8)
#define EXTI_CRx_GPIOJ			(9)
#define EXTI_CRx_GPIOK			(10)

/************************************************************************
 * API's supported by STM32F767xx
 ***********************************************************************/
/**
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/**
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

/**
 * Convert GPIOx base address to Portcode
 */
uint8_t GPIO_BaseAddrToCode(GPIO_RegDef_t *pGPIOx);


#endif /* INC_STM32F767XX_GPIO_DRIVER_H_ */
