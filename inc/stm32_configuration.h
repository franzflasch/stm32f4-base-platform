#ifndef STM32_CONFIGURATION_H
#define STM32_CONFIGURATION_H

#include <stm32f4xx.h>

#define ssc32_baudrate 38400

#define CLOCK_SPEED 100000

/**
  * @brief  SD FLASH SD Detect Pin
  */
#define SD_DETECT_PIN                    GPIO_Pin_2                 /* PC.02 */
#define SD_DETECT_GPIO_PORT              GPIOC                      /* GPIOC */

/**
 *  @brief SPI CS Pins
 */
#define CS0_Pin							 GPIO_Pin_12
#define CS0_Port						 GPIOB

/**
  * @brief  STM32F4 Discovery LED Pins
  */
#define GREEN_LED	                     GPIO_Pin_12
#define ORANGE_LED	                     GPIO_Pin_13
#define RED_LED		                     GPIO_Pin_14
#define BLUE_LED	                     GPIO_Pin_15
#define LED_PORT			             GPIOD

/**
  * @brief  USART related settings
  */
#define COMMANDLINE_USART	USART2
#define DEBUG_USART			USART2

/**
  * @brief  ADC related settings
  */
#define ADC_PIN                  GPIO_Pin_1
#define ADC_PORT                 GPIOC
#define ADC_DMA_CHANNEL          DMA_Channel_0
#define ADC_DMA_STREAM           DMA2_Stream0
#define ADC_IRQ_DMA_STREAM		 DMA2_Stream0_IRQn
#define ADC_CDR_ADDRESS          ((uint32_t)0x40012308)

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART_Configuration(void);
void SPI_Configuration(void);
void SPI_Configuration_FullSpeed(void);
void ADC_Triple_Interleaved_Configuration(__IO uint32_t *pADC_TripleConvertedVale, uint32_t bufSize);

#endif /*STM32_CONFIGURATION_H */
