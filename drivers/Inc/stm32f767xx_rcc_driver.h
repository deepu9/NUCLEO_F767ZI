#include <stdint.h>

#ifndef INC_STM32F767XX_RCC_UTIL_H_
#define INC_STM32F767XX_RCC_UTIL_H_

uint8_t Get_AHBPresc_Div(uint8_t index);
uint8_t Get_APB1Presc_Div(uint8_t index);
uint32_t RCC_GetPLLOutputClock(void);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F767XX_RCC_UTIL_H_ */
