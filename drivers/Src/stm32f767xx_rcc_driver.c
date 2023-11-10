#include <stdint.h>
#include "stm32f767xx.h"


uint8_t Get_AHBPresc_Div(uint8_t index)
{
	uint16_t ahbPresc[8] = {2, 4, 8, 16, 64, 128, 256, 512};

	return index < 8
			? 1
			: ahbPresc[index - 8];
}

uint8_t Get_APB1Presc_Div(uint8_t index)
{
	uint16_t apb1Presc[4] = {2, 4, 8, 16};

	return index < 4
			? 1
			: apb1Presc[index - 4];
}

uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, sysClk;
	uint8_t clksrc, ahbpTemp, ahbPrescDivFactor, apb1pTemp, apb1PrescDivFactor;

	clksrc = ((RCC->CFGR >> 2) & (0x3));

	switch (clksrc) {
		case 1:
			sysClk = SYS_CLK_8M;
			break;
		case 2:
			sysClk = RCC_GetPLLOutputClock();
			break;
		default:
			sysClk = SYS_CLK_16M;
			break;
	}

	ahbpTemp = ((RCC->CFGR >> 4) & (0xF));
	ahbPrescDivFactor = Get_AHBPresc_Div(ahbpTemp);

	apb1pTemp = ((RCC->CFGR >> 10) & (0x7));
	apb1PrescDivFactor = Get_APB1Presc_Div(apb1pTemp);

	pclk1 = ((sysClk / ahbPrescDivFactor) / apb1PrescDivFactor);

	return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, sysClk;
	uint8_t clksrc, ahbpTemp, ahbPrescDivFactor, apb2pTemp, apb2PrescDivFactor;

	clksrc = ((RCC->CFGR >> 2) & (0x3));

	switch (clksrc) {
		case 1:
			sysClk = SYS_CLK_8M;
			break;
		case 2:
			sysClk = RCC_GetPLLOutputClock();
			break;
		default:
			sysClk = SYS_CLK_16M;
			break;
	}

	ahbpTemp = ((RCC->CFGR >> 4) & (0xF));
	ahbPrescDivFactor = Get_AHBPresc_Div(ahbpTemp);

	apb2pTemp = ((RCC->CFGR >> 10) & (0x7));
	apb2PrescDivFactor = Get_APB1Presc_Div(apb2pTemp);

	pclk2 = ((sysClk / ahbPrescDivFactor) / apb2PrescDivFactor);

	return pclk2;
}
