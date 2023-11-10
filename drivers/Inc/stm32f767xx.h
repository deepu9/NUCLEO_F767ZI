#ifndef INC_STM32F767XX_H_
#define INC_STM32F767XX_H_


#include <stddef.h>
#include <stdint.h>

/**
 * Aliases
 */
#define __vo						volatile
#define __weak						__attribute__((weak))

/****************************************************************
 * CORTEX M-7 Processor Specific Addresses
 ***************************************************************/
/**
 * NVIC Registers
 */
#define NVIC_ISER0					((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER3					((__vo uint32_t*) 0xE000E10C)

#define NVIC_ICER0					((__vo uint32_t*) 0xE000E180U)
#define NVIC_ICER1					((__vo uint32_t*) 0xE000E184U)
#define NVIC_ICER2					((__vo uint32_t*) 0xE000E188U)
#define NVIC_ICER3					((__vo uint32_t*) 0xE000E18CU)

#define NVIC_IPR_BASE_ADDR			((__vo uint32_t*) 0xE000E400U)

#define NO_PR_BITS_IMPLEMENTED		(4) //Number of priority bits implemented

/**
 * Base addresses of FLASH & SRAM
 */
#define FLASH_BASE_ADDR				(0x08000000U)
#define SRAM1_BASE_ADDR				(0x20020000U)
#define SRAM2_BASE_ADDR				(0x2007C000U)
#define SRAM						SRAM1_BASE_ADDR
#define ROM_ADDR					(0x1FF00000U)


/**
 * Peripherals(AHB & APB) Base Address
 */
#define PERIPHERAL_BASE_ADDR		(0x40000000U)
#define APB1_BASE_ADDR				PERIPHERAL_BASE_ADDR
#define APB2_BASE_ADDR				(0x40010000U)
#define AHB1_BASE_ADDR				(0x40020000U)
#define AHB2_BASE_ADDR				(0x50000000U)
#define AHB3_BASE_ADDR				(0xA0000000U)

/**
 * Base address of peripherals on AHB1 bus
 * GPIO's
 */
#define GPIOA_BASE_ADDR				(AHB1_BASE_ADDR + (0x0000U))
#define GPIOB_BASE_ADDR				(AHB1_BASE_ADDR + (0x0400U))
#define GPIOC_BASE_ADDR				(AHB1_BASE_ADDR + (0x0800U))
#define GPIOD_BASE_ADDR				(AHB1_BASE_ADDR + (0x0C00U))
#define GPIOE_BASE_ADDR				(AHB1_BASE_ADDR + (0x1000U))
#define GPIOF_BASE_ADDR				(AHB1_BASE_ADDR + (0x1400U))
#define GPIOG_BASE_ADDR				(AHB1_BASE_ADDR + (0x1800U))
#define GPIOH_BASE_ADDR				(AHB1_BASE_ADDR + (0x1C00U))
#define GPIOI_BASE_ADDR				(AHB1_BASE_ADDR + (0x2000U))
#define GPIOJ_BASE_ADDR				(AHB1_BASE_ADDR + (0x2400U))
#define GPIOK_BASE_ADDR				(AHB1_BASE_ADDR + (0x2800U))

#define RCC_BASE_ADDR				(AHB1_BASE_ADDR + (0x3800U))

/**
 * Base address of peripherals on APB1 bus
 * I2C, SPI, USART, UART & TIM(Timers)
 */
#define SPI2_BASE_ADDR				(APB1_BASE_ADDR + (0x3800U))
#define SPI3_BASE_ADDR				(APB1_BASE_ADDR + (0x3C00U))

#define USART2_BASE_ADDR			(APB1_BASE_ADDR + (0x4400U))
#define USART3_BASE_ADDR			(APB1_BASE_ADDR + (0x4800U))
#define UART4_BASE_ADDR				(APB1_BASE_ADDR + (0x4C00U))
#define UART5_BASE_ADDR				(APB1_BASE_ADDR + (0x5000U))
#define UART7_BASE_ADDR				(APB1_BASE_ADDR + (0x7800U))
#define UART8_BASE_ADDR				(APB1_BASE_ADDR + (0x7C00U))

#define I2C1_BASE_ADDR				(APB1_BASE_ADDR + (0x5400U))
#define I2C2_BASE_ADDR				(APB1_BASE_ADDR + (0x5800U))
#define I2C3_BASE_ADDR				(APB1_BASE_ADDR + (0x5C00U))
#define I2C4_BASE_ADDR				(APB1_BASE_ADDR + (0x6000U))

#define TIM2_BASE_ADDR				(APB1_BASE_ADDR)
#define TIM3_BASE_ADDR				(APB1_BASE_ADDR + (0x0400U))
#define TIM4_BASE_ADDR				(APB1_BASE_ADDR + (0x0800U))
#define TIM5_BASE_ADDR				(APB1_BASE_ADDR + (0x0C00U))
#define TIM6_BASE_ADDR				(APB1_BASE_ADDR + (0x1000U))
#define TIM7_BASE_ADDR				(APB1_BASE_ADDR + (0x1400U))
#define TIM12_BASE_ADDR				(APB1_BASE_ADDR + (0x1800U))
#define TIM13_BASE_ADDR				(APB1_BASE_ADDR + (0x1C00U))
#define TIM14_BASE_ADDR				(APB1_BASE_ADDR + (0x2000U))

/**
 * Base address of peripherals on APB2 bus
 * SPI, USART, TIM(Timers), EXTI & SYSCFG
 */
#define USART1_BASE_ADDR			(APB2_BASE_ADDR + (0x1000U))
#define USART6_BASE_ADDR			(APB2_BASE_ADDR + (0x1400U))

#define SPI1_BASE_ADDR				(APB2_BASE_ADDR + (0x3000U))
#define SPI4_BASE_ADDR				(APB2_BASE_ADDR + (0x3400U))
#define SPI5_BASE_ADDR				(APB2_BASE_ADDR + (0x5000U))
#define SPI6_BASE_ADDR				(APB2_BASE_ADDR + (0x5400U))

#define TIM1_BASE_ADDR				(APB2_BASE_ADDR)
#define TIM8_BASE_ADDR				(APB2_BASE_ADDR + (0x0400U))
#define TIM9_BASE_ADDR				(APB2_BASE_ADDR + (0x4000U))
#define TIM10_BASE_ADDR				(APB2_BASE_ADDR + (0x4400U))
#define TIM11_BASE_ADDR				(APB2_BASE_ADDR + (0x4800U))

#define SYSCFG_BASE_ADDR			(APB2_BASE_ADDR + (0x3800U))

#define EXTI_BASE_ADDR				(APB2_BASE_ADDR + (0x3C00U))


/**
 * GPIOx Peripheral registers
 */
typedef struct {
	__vo uint32_t MODER;		/*!< GPIO port mode register					Address Offset - 0x00 >*/
	__vo uint32_t OTYPER;		/*!< GPIO port output type register				Address Offset - 0x04 >*/
	__vo uint32_t OSPEEDR;		/*!< GPIO port speed register					Address Offset - 0x08 >*/
	__vo uint32_t PUPDR;		/*!< GPIO port pull-up/pull-down register		Address Offset - 0x0C >*/
	__vo uint32_t IDR;			/*!< GPIO port input data register				Address Offset - 0x10 >*/
	__vo uint32_t ODR;			/*!< GPIO port output data register				Address Offset - 0x14 >*/
	__vo uint32_t BSRR;			/*!< GPIO port bit set/reset register			Address Offset - 0x18 >*/
	__vo uint32_t LCKR;			/*!< GPIO port configuration lock register		Address Offset - 0x1C >*/
	__vo uint32_t AFR[2];		/*!< GPIO alternate function low register		Address Offset - 0x20 GPIO alternate function high register		Address Offset - 0x24 >*/
}GPIO_RegDef_t;

/**
 * RCC registers
 */
typedef struct {
	__vo uint32_t CR;			/*!< RCC clock control register						Address offset - 0x00 >*/
	__vo uint32_t PLLCFGR;		/*!< RCC PLL configuration register					Address offset - 0x04 >*/
	__vo uint32_t CFGR;			/*!< RCC clock configuration register				Address offset - 0x08 >*/
	__vo uint32_t CIR;			/*!< RCC clock interrupt register					Address offset - 0x0C >*/
	__vo uint32_t AHB1RSTR;		/*!< RCC AHB1 peripheral reset register				Address offset - 0x10 >*/
	__vo uint32_t AHB2RSTR;		/*!< RCC AHB2 peripheral reset register				Address offset - 0x14 >*/
	__vo uint32_t AHB3RSTR;		/*!< RCC AHB3 peripheral reset register				Address offset - 0x18 >*/
	uint32_t RESERVED0;			/*!< RESERVED0 register								Address offset - 0x1C >*/
	__vo uint32_t APB1RSTR;		/*!< RCC APB1 peripheral reset register				Address offset - 0x20 >*/
	__vo uint32_t APB2RSTR;		/*!< RCC APB2 peripheral reset register				Address offset - 0x24 >*/
	uint32_t RESERVED1[2];		/*!< RESERVED1[0] register	Address offset - 0x28, RESERVED1[1]		Address Offset - 0x2C >*/
	__vo uint32_t AHB1ENR;		/*!< RCC AHB1 peripheral clock register				Address offset - 0x30 >*/
	__vo uint32_t AHB2ENR;		/*!< RCC AHB2 peripheral clock register				Address offset - 0x34 >*/
	__vo uint32_t AHB3ENR;		/*!< RCC AHB3 peripheral clock register				Address offset - 0x38 >*/
	uint32_t RESERVED2;			/*!< RESERVED2 register								Address offset - 0x3C >*/
	__vo uint32_t APB1ENR;		/*!< RCC APB1 peripheral clock register				Address offset - 0x40 >*/
	__vo uint32_t APB2ENR;		/*!< RCC APB2 peripheral clock register				Address offset - 0x44 >*/
	uint32_t RESERVED3[2];		/*!< RESERVED3[0] register	Address offset - 0x48, RESERVED3[1]		Address Offset - 0x4C >*/
	__vo uint32_t AHB1LPENR;	/*!< RCC AHB1LPENR peripheral clock register		Address offset - 0x50 >*/
	__vo uint32_t AHB2LPENR;	/*!< RCC AHB2LPENR peripheral clock register		Address offset - 0x54 >*/
	__vo uint32_t AHB3LPENR;	/*!< RCC AHB3LPENR peripheral clock register		Address offset - 0x58 >*/
	uint32_t RESERVED4;			/*!< RESERVED4 register								Address offset - 0x5C >*/
	__vo uint32_t APB1LPENR;	/*!< RCC APB1LPENR peripheral clock register		Address offset - 0x60 >*/
	__vo uint32_t APB2LPENR;	/*!< RCC APB2LPENR peripheral clock register		Address offset - 0x64 >*/
	uint32_t RESERVED5[2];		/*!< RESERVED5[0] register	Address offset - 0x68, RESERVED5[1]		Address Offset - 0x6C >*/
	__vo uint32_t BDCR;			/*!< RCC backup domain control register				Address offset - 0x70 >*/
	__vo uint32_t CSR;			/*!< RCC clock control & status register			Address offset - 0x74 >*/
	uint32_t RESERVED6[2];		/*!< RESERVED6[0] register	Address offset - 0x78, RESERVED6[1]		Address Offset - 0x7C >*/
	__vo uint32_t SSCGR;		/*!< RCC spread spectrum clock generation register	Address offset - 0x80 >*/
	__vo uint32_t PLLI2SCFGR;	/*!< RCC PLLI2S configuration register				Address offset - 0x84 >*/
	__vo uint32_t PLLSAICFGR;	/*!< RCC PLLSAI configuration register				Address offset - 0x88 >*/
	__vo uint32_t DCKCFGR1;		/*!< RCC dedicated clocks configuration register1	Address offset - 0x8C >*/
	__vo uint32_t DCKCFGR2;		/*!< RCC dedicated clocks configuration register2	Address offset - 0x90 >*/
}RCC_RegDef_t;

/**
 * EXTI Registers
 */
typedef struct
{
	__vo uint32_t IMR;			/*!< Interrupt Mask Register				Address Offset - 0x00 >*/
	__vo uint32_t EMR;			/*!< Event Mask Register					Address Offset - 0x04 >*/
	__vo uint32_t RTSR;			/*!< Rising Trigger Selection Register		Address Offset - 0x08 >*/
	__vo uint32_t FTSR;			/*!< Falling Trigger Selection Register		Address Offset - 0x0C >*/
	__vo uint32_t SWIER;		/*!< Software Interrupt Event Register		Address Offset - 0x10 >*/
	__vo uint32_t PR;			/*!< Pending Register						Address Offset - 0x14 >*/
}EXTI_RegDef_t;

/**
 * SYSCFG Registers
 */
typedef struct
{
	__vo uint32_t MEMRMP;		/*!< Memory Remap Register							Address Offset - 0x00 >*/
	__vo uint32_t PMC;			/*!< Peripheral Mode Configuration Register			Address Offset - 0x04 >*/
	__vo uint32_t EXTICR[4];	/*!< External Interrupt Configuration Register 1-4	Address Offset - 0x08 - 0x14 >*/
	uint32_t RESERVED;			/*!< RESERVED										Address Offset - 0x18 >*/
	__vo uint32_t CBR;			/*!< Class B Register								Address Offset - 0x1C >*/
	__vo uint32_t CMPCR;		/*!< Compensation Cell Control Register				Address Offset - 0x20 >*/
}SYSCFG_RegDef_t;

/**
 * SPI Registers
 */
typedef struct
{
	__vo uint32_t CR1;			/*!< Control Registers 1			Address Offset - 0x00 >*/
	__vo uint32_t CR2;			/*!< Control Registers 2			Address Offset - 0x04 >*/
	__vo uint32_t SR;			/*!< Status Register				Address Offset - 0x08 >*/
	__vo uint32_t DR;			/*!< Data Register					Address Offset - 0x0C >*/
	__vo uint32_t CRCPR;		/*!< CRC Polynomial Register		Address Offset - 0x10 >*/
	__vo uint32_t RXCRCR;		/*!< Rx CRC Register				Address Offset - 0x14 >*/
	__vo uint32_t TXCRCR;		/*!< Tx CRC Register				Address Offset - 0x18 >*/
	__vo uint32_t I2SCFGR;		/*!< I2S Configuration Register		Address Offset - 0x1C >*/
	__vo uint32_t I2SPR;		/*!< I2S PreScaler Register			Address Offset - 0x20 >*/
}SPI_RegDef_t;

/**
 * I2C Registers
 */
typedef struct
{
	__vo uint32_t CR1; /*!< Control Register 1				Address Offset - 0x00 >*/
	__vo uint32_t CR2; /*!< Control Register 2				Address Offset - 0x04 >*/
	__vo uint32_t OAR1; /*!< Own Address 1 Register			Address Offset - 0x08 >*/
	__vo uint32_t OAR2; /*!< Own Address 2 Register			Address Offset - 0x0C >*/
	__vo uint32_t TIMINGR; /*!< Timing Register				Address Offset - 0x10 >*/
	__vo uint32_t TIMEOUTR; /*!< Timeout Register			Address Offset - 0x14 >*/
	__vo uint32_t ISR; /*!< Interrupt and Status Register	Address Offset - 0x18 >*/
	__vo uint32_t ICR; /*!< Interrupt Clear Register		Address Offset - 0x1C >*/
	__vo uint32_t PECR; /*!< PEC Register					Address Offset - 0x20 >*/
	__vo uint32_t RXDR; /*!< Receive Data Register			Address Offset - 0x24 >*/
	__vo uint32_t TXDR; /*!< Transmit Data Register			Address Offset - 0x28 >*/
}I2C_RegDef_t;

/**
 * UART/USART Registers
 */
typedef struct
{
	__vo uint32_t CR1; /*!< Control Register 1								Address Offset - 0x00 >*/
	__vo uint32_t CR2; /*!< Control Register 2								Address Offset - 0x04 >*/
	__vo uint32_t CR3; /*!< Control Register 3								Address Offset - 0x08 >*/
	__vo uint32_t BRR; /*!< Baud rate register								Address Offset - 0x0C >*/
	__vo uint32_t GTPR; /*!< Guard time and Prescaler register				Address Offset - 0x10 >*/
	__vo uint32_t RTOR; /*!< Receiver timeout register						Address Offset - 0x14 >*/
	__vo uint32_t RQR; /*!< Request Register								Address Offset - 0x18 >*/
	__vo uint32_t ISR; /*!< Interrupt and Status Register					Address Offset - 0x1C >*/
	__vo uint32_t ICR; /*!< Interrupt flag clear Register					Address Offset - 0x20 >*/
	__vo uint32_t RDR; /*!< Receive data Register							Address Offset - 0x24 >*/
	__vo uint32_t TDR; /*!< Transmit data Register							Address Offset - 0x28 >*/
}USART_RegDef_t;

/**
 * Base Timers
 * TIM6/TIM7
 */
typedef struct
{
	__vo uint32_t CR1;			/*!<Control register 1	Address offset: 0x00>*/
	__vo uint32_t CR2;			/*!<Control register 2	Address offset: 0x04>*/
	uint32_t RESERVED0;			/*!<Reserved	Address offset: 0x08>*/
	__vo uint32_t DIER;			/*!<DMA/Interrupt enable register	Address offset: 0x0C>*/
	__vo uint32_t SR;			/*!<Status register	Address offset: 0x10>*/
	__vo uint32_t EGR;			/*!<Event generation register	Address offset: 0x14>*/
	uint32_t RESERVED1[2];		/*!<Reserved	Address offset: 0x18-0x20>*/
	__vo uint32_t CNT;			/*!<Counter register	Address offset: 0x24>*/
	__vo uint32_t PSC;			/*!<Prescaler register	Address offset: 0x28>*/
	__vo uint32_t ARR;			/*!<Auto reload register	Address offset: 0x2C>*/
}Basic_TIM_RegDef_t;

/**
 * Peripheral definitions
 */
#define GPIOA						((GPIO_RegDef_t*) GPIOA_BASE_ADDR)
#define GPIOB						((GPIO_RegDef_t*) GPIOB_BASE_ADDR)
#define GPIOC						((GPIO_RegDef_t*) GPIOC_BASE_ADDR)
#define GPIOD						((GPIO_RegDef_t*) GPIOD_BASE_ADDR)
#define GPIOE						((GPIO_RegDef_t*) GPIOE_BASE_ADDR)
#define GPIOF						((GPIO_RegDef_t*) GPIOF_BASE_ADDR)
#define GPIOG						((GPIO_RegDef_t*) GPIOG_BASE_ADDR)
#define GPIOH						((GPIO_RegDef_t*) GPIOH_BASE_ADDR)
#define GPIOI						((GPIO_RegDef_t*) GPIOI_BASE_ADDR)
#define GPIOJ						((GPIO_RegDef_t*) GPIOJ_BASE_ADDR)
#define GPIOK						((GPIO_RegDef_t*) GPIOK_BASE_ADDR)

#define RCC							((RCC_RegDef_t*) RCC_BASE_ADDR)
#define EXTI						((EXTI_RegDef_t*) EXTI_BASE_ADDR)
#define SYSCFG						((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDR)

#define SPI1						((SPI_RegDef_t*) SPI1_BASE_ADDR)
#define SPI2						((SPI_RegDef_t*) SPI2_BASE_ADDR)
#define SPI3						((SPI_RegDef_t*) SPI3_BASE_ADDR)
#define SPI4						((SPI_RegDef_t*) SPI4_BASE_ADDR)
#define SPI5						((SPI_RegDef_t*) SPI5_BASE_ADDR)
#define SPI6						((SPI_RegDef_t*) SPI6_BASE_ADDR)

#define I2C1						((I2C_RegDef_t*) I2C1_BASE_ADDR)
#define I2C2						((I2C_RegDef_t*) I2C2_BASE_ADDR)
#define I2C3						((I2C_RegDef_t*) I2C3_BASE_ADDR)
#define I2C4						((I2C_RegDef_t*) I2C4_BASE_ADDR)

#define USART1						((USART_RegDef_t*) USART1_BASE_ADDR)
#define USART2						((USART_RegDef_t*) USART2_BASE_ADDR)
#define USART3						((USART_RegDef_t*) USART3_BASE_ADDR)
#define UART4						((USART_RegDef_t*) UART4_BASE_ADDR)
#define UART5						((USART_RegDef_t*) UART5_BASE_ADDR)
#define USART6						((USART_RegDef_t*) USART6_BASE_ADDR)
#define UART7						((USART_RegDef_t*) UART7_BASE_ADDR)
#define UART8						((USART_RegDef_t*) UART8_BASE_ADDR)

#define TIM6						((Basic_TIM_RegDef_t*) TIM6_BASE_ADDR)
#define TIM7						((Basic_TIM_RegDef_t*) TIM7_BASE_ADDR)

/**
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()				(RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()				(RCC->AHB1ENR |= (1 << 10))

/**
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 10))

/**
 * Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << 23))
#define I2C4_PCLK_EN()				(RCC->APB1ENR |= (1 << 24))

/**
 * Clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 23))
#define I2C4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 24))

/**
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()				(RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()				(RCC->APB2ENR |= (1 << 21))

/**
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 21))

/**
 * Clock enable macros for USARTx peripherals
 */
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))
#define UART7_PCLK_EN()				(RCC->APB1ENR |= (1 << 30))
#define UART8_PCLK_EN()				(RCC->APB1ENR |= (1 << 31))

/**
 * Clock disable macros for USARTx peripherals
 */
#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5))
#define UART7_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 30))
#define UART8_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 31))

/**
 * Clock enable macros for TIMx peripherals
 */
#define TIM6_PCLK_EN()				(RCC->APB1ENR |= (1 << 4))
#define TIM7_PCLK_EN()				(RCC->APB1ENR |= (1 << 5))

/**
 * Clock disable macros for TIMx peripherals
 */
#define TIM6_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 4))
#define TIM7_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 5))

/**
 * Clock enable macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))

/**
 * Clock disable macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))

/****************************************************************
 * RESET Peripherals
 ***************************************************************/
/**
 * Macros to reset GPIOx peripherals
 * According to Reference manual:
 * 1. Set the port to 1, then
 * 2. Set the port to 0
 */
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)
#define GPIOJ_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 9)); (RCC->AHB1RSTR &= ~(1 << 9)); }while(0)
#define GPIOK_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR &= ~(1 << 10)); }while(0)

/**
 * Macros to reset SPIx peripherals
 * According to Reference manual:
 * 1. Set the port to 1, then
 * 2. Set the port to 0
 */
#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->AHB1RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->AHB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->AHB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->AHB1RSTR &= ~(1 << 13)); }while(0)
#define SPI5_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 20)); (RCC->AHB1RSTR &= ~(1 << 20)); }while(0)
#define SPI6_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 21)); (RCC->AHB1RSTR &= ~(1 << 21)); }while(0)

/**
 * Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)
#define I2C4_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 24)); (RCC->APB1RSTR &= ~(1 << 24)); }while(0)

/**
 * Macros to reset USART/UART peripherals
 * USART1, 2, 3 & 6
 * UART 4, 5, 7, 8
 */
#define USART1_REG_RESET()			do{(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); }while(0)
#define USART2_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART3_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define UART4_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define UART5_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0)
#define USART6_REG_RESET()			do{(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); }while(0)
#define UART7_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 30)); (RCC->APB1RSTR &= ~(1 << 30)); }while(0)
#define UART8_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 31)); (RCC->APB1RSTR &= ~(1 << 31)); }while(0)

/**
 * Macros to reset TIMx peripherals
 */
#define TIM6_REG_RESET()				do{(RCC->APB1RSTR |= (1 << 4)); (RCC->APB1RSTR &= ~(1 << 4)); }while(0)
#define TIM7_REG_RESET()				do{(RCC->APB1RSTR |= (1 << 5)); (RCC->APB1RSTR &= ~(1 << 5)); }while(0)

/**
 * IRQ(Interrupt Request) number of STM32F767xx MCU
 */
//GPIO Interrupts
#define IRQ_EXT0_POS				(6)
#define IRQ_EXT1_POS				(7)
#define IRQ_EXT2_POS				(8)
#define IRQ_EXT3_POS				(9)
#define IRQ_EXT4_POS				(10)
#define IRQ_EXT9_5_POS				(23)
#define IRQ_EXT15_10_POS			(40)

//SPI Interrupts
#define IRQ_SPI1					(35)
#define IRQ_SPI2					(36)
#define IRQ_SPI3					(51)
#define IRQ_SPI4					(84)
#define IRQ_SPI5					(85)
#define IRQ_SPI6					(86)

/**
 * Macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0				(0)
#define NVIC_IRQ_PRI15				(15)

/**
 * Generic macros
 */
#define ENABLE 						(1)
#define DISABLE						(0)
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define FLAG_SET					SET
#define FLAG_RESET					RESET
#define I2C_WR_TRANSFER				(0)
#define I2C_RD_TRANSFER				(1)


/*******************************************************
 * Bit position definitions of SPI Peripheral
 ******************************************************/
/**
 * Bit position definitions for SPI_CR1
 */
#define SPI_CR1_CPHA				(0)
#define SPI_CR1_CPOL				(1)
#define SPI_CR1_MSTR				(2)
#define SPI_CR1_BR					(3)
#define SPI_CR1_SPE					(6)
#define SPI_CR1_LSB_FIRST			(7)
#define SPI_CR1_SSI					(8)
#define SPI_CR1_SSM					(9)
#define SPI_CR1_RXONLY				(10)
#define SPI_CR1_CRCL				(11)
#define SPI_CR1_CRC_NEXT			(12)
#define SPI_CR1_CRC_EN				(13)
#define SPI_CR1_BIDIOE				(14)
#define SPI_CR1_BIDIMODE			(15)

/**
 * Bit position definitions for SPI_CR2
 */
#define SPI_CR2_RXDMA_EN			(0)
#define SPI_CR2_TXDMA_EN			(1)
#define SPI_CR2_SSOE				(2)
#define SPI_CR2_NSSP				(3)
#define SPI_CR2_FRF					(4)
#define SPI_CR2_ERRIE				(5)
#define SPI_CR2_RXNEIE				(6)
#define SPI_CR2_TXEIE				(7)
#define SPI_CR2_DS					(8)
#define SPI_CR2_FRXTH				(12)
#define SPI_CR2_LDMA_RX				(13)
#define SPI_CR2_LDMA_TX				(14)

/**
 * Bit position definitions for SPI_SR
 */
#define SPI_SR_RXNE					(0)
#define SPI_SR_TXE					(1)
#define SPI_SR_CHSIDE				(2)
#define SPI_SR_UDR					(3)
#define SPI_SR_CRC_ERR				(4)
#define SPI_SR_MODF					(5)
#define SPI_SR_OVR					(6)
#define SPI_SR_BSY					(7)
#define SPI_SR_FRE					(8)
#define SPI_SR_FRLVL				(9)
#define SPI_SR_FTLVL				(11)


/*******************************************************
 * Bit position definitions of I2C Peripheral
 ******************************************************/
/**
 * Bit position definitions for I2C_CR1
 */
#define I2C_CR1_PE					(0)
#define I2C_CR1_TXIE				(1)
#define I2C_CR1_RXIE				(2)
#define I2C_CR1_ADDRIE				(3)
#define I2C_CR1_NACKIE				(4)
#define I2C_CR1_STOPIE				(5)
#define I2C_CR1_TCIE				(6)
#define I2C_CR1_ERRIE				(7)
#define I2C_CR1_DNF					(8)
#define I2C_CR1_ANFOFF				(12)
#define I2C_CR1_TXDMAEN				(14)
#define I2C_CR1_RXDMAEN				(15)
#define I2C_CR1_SBC					(16)
#define I2C_CR1_NOSTRERCH			(17)
#define I2C_CR1_GCEN				(19)
#define I2C_CR1_SMBHEN				(20)
#define I2C_CR1_SMBDEN				(21)
#define I2C_CR1_ALERTEN				(22)
#define I2C_CR1_PECEN				(23)

/**
 * Bit position definitions for I2C_CR2
 */
#define I2C_CR2_SADD0				(0)
#define I2C_CR2_SADD1_7				(1)
#define I2C_CR2_SADD8_9				(8)
#define I2C_CR2_RD_WRN				(10)
#define I2C_CR2_ADD10				(11)
#define I2C_CR2_HEAD10R				(12)
#define I2C_CR2_START				(13)
#define I2C_CR2_STOP				(14)
#define I2C_CR2_NACK				(15)
#define I2C_CR2_NBYTES				(16)
#define I2C_CR2_RELOAD				(24)
#define I2C_CR2_AUTOEND				(25)
#define I2C_CR2_PECBYTE				(26)

/**
 * Bit position definitions for I2C_OAR1
 */
#define I2C_OAR1_OA1_ADDR0			(0)
#define I2C_OAR1_OA1_ADDR1_7		(1)
#define I2C_OAR1_OA1_ADDR8_9		(8)
#define I2C_OAR1_OA1_MODE			(10)
#define I2C_OAR1_OA1_EN				(15)

/**
 * Bit position definitions for I2C_TIMINGR
 */
#define I2C_TIMINGR_SCLL			(0)
#define I2C_TIMINGR_SCLH			(8)
#define I2C_TIMINGR_SDADEL			(16)
#define I2C_TIMINGR_SCLDEL			(20)
#define I2C_TIMINGR_PRESC			(28)

/**
 * Bit position definitions for I2C_SR
 */
#define I2C_SR_TXE					(0)
#define I2C_SR_TXIS					(1)
#define I2C_SR_RXNE					(2)
#define I2C_SR_ADDR					(3)
#define I2C_SR_NACKF				(4)
#define I2C_SR_STOPF				(5)
#define I2C_SR_TC					(6)
#define I2C_SR_TCR					(7)
#define I2C_SR_BERR					(8)
#define I2C_SR_ARLO					(9)
#define I2C_SR_OVR					(10)
#define I2C_SR_PECERR				(11)
#define I2C_SR_TIMEOUT				(12)
#define I2C_SR_ALERT				(13)
#define I2C_SR_BUSY					(15)
#define I2C_SR_DIR					(16)
#define I2C_SR_ADDCODE				(17)

/*******************************************************
 * Bit position definitions of USART/UART Peripheral
 ******************************************************/
/**
 * Bit position definitions for USART_CR1
 */
#define USART_CR1_UE				(0)
#define USART_CR1_UESM				(1)
#define USART_CR1_RE				(2)
#define USART_CR1_TE				(3)
#define USART_CR1_IDLEIE			(4)
#define USART_CR1_RXNEIE			(5)
#define USART_CR1_TCIE				(6)
#define USART_CR1_TXEIE				(7)
#define USART_CR1_PEIE				(8)
#define USART_CR1_PS				(9)
#define USART_CR1_PCE				(10)
#define USART_CR1_WAKE				(11)
#define USART_CR1_M0				(12)
#define USART_CR1_MME				(13)
#define USART_CR1_CMIE				(14)
#define USART_CR1_OVER8				(15)
#define USART_CR1_DEDT				(16)
#define USART_CR1_DEAT				(21)
#define USART_CR1_RTOIE				(26)
#define USART_CR1_EOBIE				(27)
#define USART_CR1_M1				(28)

/**
 * Bit position definitions for USART_CR2
 */
#define USART_CR2_ADDM7				(4)
#define USART_CR2_LBDL				(5)
#define USART_CR2_LBDIE				(6)
#define USART_CR2_LBCL				(8)
#define USART_CR2_CPHA				(9)
#define USART_CR2_CPOL				(10)
#define USART_CR2_CLKEN				(11)
#define USART_CR2_STOP				(12)
#define USART_CR2_LINEN				(14)
#define USART_CR2_SWAP				(15)
#define USART_CR2_RXINV				(16)
#define USART_CR2_TXINV				(17)
#define USART_CR2_DATAINV			(18)
#define USART_CR2_MSBFIRST			(19)
#define USART_CR2_ABREN				(20)
#define USART_CR2_ABRMOD			(21)
#define USART_CR2_RTOEN				(23)
#define USART_CR2_ADD_1				(24)
#define USART_CR2_ADD_2				(28)

/**
 * Bit position definitions for USART_CR3
 */
#define USART_CR3_EIE				(0)
#define USART_CR3_IREN				(1)
#define USART_CR3_IRLP				(2)
#define USART_CR3_HDSEL				(3)
#define USART_CR3_NACK				(4)
#define USART_CR3_SCEN				(5)
#define USART_CR3_DMAR				(6)
#define USART_CR3_DMAT				(7)
#define USART_CR3_RTSE				(8)
#define USART_CR3_CTSE				(9)
#define USART_CR3_CTSIE				(10)
#define USART_CR3_ONEBIT			(11)
#define USART_CR3_OVRDIS			(12)
#define USART_CR3_DDRE				(13)
#define USART_CR3_DEM				(14)
#define USART_CR3_DEP				(15)
#define USART_CR3_SCARCNT0			(17)
#define USART_CR3_SCARCNT1			(18)
#define USART_CR3_SCARCNT2			(19)
#define USART_CR3_WUS0				(20)
#define USART_CR3_WUS1				(21)
#define USART_CR3_WUFIE				(22)
#define USART_CR3_UCESM				(23)
#define USART_CR3_TCBGTIE			(24)

/**
 * Bit position definitions for USART_ISR
 */
#define USART_ISR_PE				(0)
#define USART_ISR_FE				(1)
#define USART_ISR_NF				(2)
#define USART_ISR_ORE				(3)
#define USART_ISR_IDLE				(4)
#define USART_ISR_RXNE				(5)
#define USART_ISR_TC				(6)
#define USART_ISR_TXE				(7)
#define USART_ISR_LBDF				(8)
#define USART_ISR_CTSIF				(9)
#define USART_ISR_CTS				(10)
#define USART_ISR_RTOF				(11)
#define USART_ISR_EOBF				(12)
#define USART_ISR_ABRE				(14)
#define USART_ISR_ABRF				(15)
#define USART_ISR_BUSY				(16)
#define USART_ISR_CMF				(17)
#define USART_ISR_SBKF				(18)
#define USART_ISR_RWU				(19)
#define USART_ISR_WUF				(20)
#define USART_ISR_TEACK				(21)
#define USART_ISR_REACK				(22)
#define USART_ISR_TCBGT				(25)


/**
 * Basic Timers(TIM6/TIM7)
 * Bit position definitions for TIM_CR1
 */
#define BASIC_TIM_CR1_CEN			(0)
#define BASIC_TIM_CR1_UDIS			(1)
#define BASIC_TIM_CR1_URS			(2)
#define BASIC_TIM_CR1_OPM			(3)
#define BASIC_TIM_CR1_ARPE			(7)
#define BASIC_TIM_CR1_UIFREMAP		(11)

/**
 * Basic Timers(TIM6/TIM7)
 * Bit position definitions for TIM_CR2
 */
#define BASIC_TIM_CR2_MMS			(4)

/**
 * Basic Timers(TIM6/TIM7)
 * Bit position definitions for TIM_SR
 */
#define BASIC_TIM_SR_UIF			(0)

/**
 * Basic Timers(TIM6/TIM7)
 * Bit position definitions for TIM_EGR
 */
#define BASIC_TIM_EGR_UG			(0)

/**
 * Basic Timers(TIM6/TIM7)
 * Bit position definitions for TIM_CNT
 */
#define BASIC_TIM_CNT_CNT			(0)
#define BASIC_TIM_CNT_UIFCPY		(31)

/**
 * Basic Timers(TIM6/TIM7)
 * Bit position definitions for TIM_PSC
 */
#define BASIC_TIM_PSC_PSC			(0)

/**
 * Basic Timers(TIM6/TIM7)
 * Bit position definitions for TIM_ARR
 */
#define BASIC_TIM_ARR_ARR			(0)

#include "stm32f767xx_rcc_driver.h"
#include "stm32f767xx_gpio_driver.h"
#include "stm32f767xx_spi_driver.h"
#include "stm32f767xx_i2c_driver.h"
#include "stm32f767xx_usart_driver.h"
#include "stm32f767xx_basic_timers_driver.h"

#endif /* INC_STM32F767XX_H_ */
