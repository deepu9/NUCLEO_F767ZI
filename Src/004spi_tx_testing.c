#include <string.h>
#include "stm32f767xx.h"


void SPI_GPIOInit(void);
void SPI_Init_Fn(void);

int main(void)
{
	char user_data[] = "Hello";
	uint8_t dataLen = strlen(user_data);

	SPI_GPIOInit();

	SPI_Init_Fn();

	/**
	 * Making SSOE to 1, does NSS output enable.
	 * The NSS pin is automatically managed by the hardware
	 *
	 * i.e when SPE = 1, NSS will be pulled to LOW(0)
	 * and NSS pin will be HIGH when SPE = 0.
	 */
//	SPI_SSOEConfig(SPI1, ENABLE);

	//Makes NSS signal internally HIGH and avoids MODF error
	SPI_SSIConfig(SPI1, ENABLE);

	SPI_PeripheralControl(SPI1, ENABLE);

	//Send data
	SPI_SendData(SPI1, (uint8_t*)user_data, dataLen);

	while(SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));

	SPI_PeripheralControl(SPI1, DISABLE);

	while(1);
}

void SPI_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	/**
	 * SPI2 on AF5(Alternate Function)
	 * PB12 - NSS
	 * PB13 - SCLK
	 * PB14 - MISO
	 * PB15 - MOSI
	 *
	 * OR
	 *
	 * SPI1 on AF5
	 * PA4 - NSS
	 * PA5 - SCK
	 * PA6 - MISO
	 * PA7 - MOSI
	 */
	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_HIGH;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AFR5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OUT_TYPE_PP;

	//GPIO Peripheral clock
	GPIO_PeriClockControl(SPIPins.pGPIOx, ENABLE);

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&SPIPins);

	//MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
//	GPIO_Init(&SPIPins);

	//NSS
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
//	GPIO_Init(&SPIPins);
}

void SPI_Init_Fn(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI1;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DS = SPI_DS_8BIT;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;
//	SPI2Handle.SPIConfig.SPI_RXONLY = SPI_RXONLY_DI;
//	SPI2Handle.SPIConfig.SPI_FRXTH = SPI_FRXTH_8BIT;

	SPI_PeriClockControl(SPI2Handle.pSPIx, ENABLE);
	SPI_Init(&SPI2Handle);
}

