#ifndef STM32_CONFIGURATION_H
#define STM32_CONFIGURATION_H

#include <stm32f4xx.h>

/**
  * @brief  STM32F4 Discovery LED Pins
  */
#define GREEN_LED	                     GPIO_Pin_12
#define ORANGE_LED	                     GPIO_Pin_13
#define RED_LED		                     GPIO_Pin_14
#define BLUE_LED	                     GPIO_Pin_15
#define LED_PORT			             GPIOD

void RCC_Configuration(void);
void LED_Configuration(void);
void NVIC_Configuration(void);

#endif /*STM32_CONFIGURATION_H */
